#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <WiFi.h>

// ---------------------------------------------------------------------------
// WiFi <-> LoRa bridge for the Seeed XIAO ESP32-S3 + Wio-SX1262 kit.
//
// Flash this SAME sketch to BOTH boards. Each board is a symmetric bridge:
//   * runs a WiFi Access Point ("dora") with a TCP server on port 5000,
//   * a TCP client sends a framed message (TYPE:filename:size + payload),
//     which the board fragments and transmits over LoRa,
//   * messages received over LoRa are reassembled and pushed back to the
//     TCP client as a "RECV:TYPE:filename:size" framed message.
//
// Everything runs in one non-blocking loop() so the SX1262 (which is
// interrupt-driven and half-duplex) is serviced every iteration. LoRa
// transmit happens only on demand, driven by what the WiFi client sends.
// ---------------------------------------------------------------------------

// ---- Pin map: FIXED by the Wio-SX1262 board-to-board (B2B) connector -------
// These are raw ESP32-S3 GPIO numbers and should NOT be changed for this kit.
#define LORA_NSS   41   // SPI chip select
#define LORA_DIO1  39   // IRQ line
#define LORA_RST   42   // reset
#define LORA_BUSY  40   // busy
#define LORA_SCK    7   // SPI clock
#define LORA_MISO   8   // SPI MISO
#define LORA_MOSI   9   // SPI MOSI

// ---- Radio configuration ---------------------------------------------------
// IMPORTANT: both boards MUST use the same frequency, and it must be legal in
// your region. The Wio-SX1262 supports 862-930 MHz:
//   868.0 -> EU868
//   915.0 -> US915 / AU915
#define LORA_FREQUENCY 868.0

// The Wio-SX1262 uses a 1.8 V TCXO powered from the radio's DIO3 pin. Passing
// the correct voltage here is required for the radio to calibrate on boot.
#define LORA_TCXO_VOLTAGE 1.8

// ---- WiFi / TCP server -----------------------------------------------------
static const char* AP_SSID = "dora";
static const char* AP_PASS = "dora1234";
static const uint16_t TCP_PORT = 5000;

// ---- Bridge sizing ---------------------------------------------------------
// LoRa packet layout: [dst][src][msgId][fragIndex][fragCount] + data.
static const size_t   LORA_HDR = 5;
// Data bytes per LoRa packet. 5 + 200 = 205 <= SX1262 max (255).
static const size_t   LORA_CHUNK = 200;
// Largest message we will bridge in either direction.
static const size_t   MAX_BRIDGE_PAYLOAD = 20480;
// Upper bound on fragments per message (metadata + data chunks), with margin.
static const uint16_t MAX_FRAGS = 20;
// dst value that every node accepts.
static const uint8_t  BROADCAST_ADDR = 0xFF;
// Drop a half-reassembled inbound message if it stalls this long.
static const unsigned long REASSEMBLY_TIMEOUT_MS = 40000;
// Cap on a single TCP header line, matching the master protocol.
static const size_t   MAX_HEADER_LENGTH = 256;

// SX1262 radio instance: Module(cs/NSS, irq/DIO1, reset, busy).
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

WiFiServer tcpServer(TCP_PORT);
WiFiClient currentClient;   // single active TCP client

uint8_t localAddress = 0x00;   // this board's id (set from MAC in setup)

// Set by the DIO1 interrupt when the current TX or RX operation completes.
volatile bool operationDone = false;
// True while a LoRa transmit is in flight, so we know how to read the IRQ.
bool transmitting = false;

// ---- Outbound (WiFi client -> LoRa) message state --------------------------
uint8_t  txAssembly[MAX_BRIDGE_PAYLOAD];  // payload staged from the TCP client
bool     txActive = false;                // a message is being sent over LoRa
uint8_t  txMsgId = 0;                      // rolling id stamped on each message
String   txType;
String   txFilename;
size_t   txPayloadSize = 0;
uint16_t txFragCount = 0;                  // metadata frag + data frags
uint16_t txNextFrag = 0;                   // index of the fragment to send next
uint32_t bridgedTx = 0;

// ---- Inbound (LoRa -> WiFi client) reassembly state ------------------------
uint8_t  rxAssembly[MAX_BRIDGE_PAYLOAD];   // payload rebuilt from LoRa fragments
bool     rxActive = false;                 // currently reassembling a message
uint8_t  rxSrc = 0;
uint8_t  rxMsgId = 0;
uint16_t rxFragCount = 0;
uint16_t rxReceivedCount = 0;
bool     rxFragGot[MAX_FRAGS];
bool     rxMetaGot = false;
String   rxType;
String   rxFilename;
size_t   rxSize = 0;
unsigned long rxLastMillis = 0;
uint32_t bridgedRx = 0;

// ---- Non-blocking TCP client parser state ----------------------------------
enum WifiState { WIFI_HEADER, WIFI_PAYLOAD, WIFI_SKIP };
WifiState wifiState = WIFI_HEADER;
String    wifiHeader;
String    wifiType;
String    wifiFilename;
size_t    wifiPayloadSize = 0;
size_t    wifiPayloadRead = 0;
size_t    wifiSkipRemaining = 0;

// Runs in interrupt context, so keep it tiny: just raise a flag.
IRAM_ATTR void onDio1(void) {
  operationDone = true;
}

// Halt with a repeating message so a wiring/config problem is obvious.
void halt(const char* reason, int code) {
  while (true) {
    Serial.printf("HALTED: %s (code %d)\n", reason, code);
    delay(2000);
  }
}

// Parse a "TYPE:filename:size" line (TYPE in TEXT/FILE). Shared by the TCP
// header parser and the LoRa metadata fragment parser.
bool parseHeader(const String& header, String& type, String& filename, size_t& size) {
  int firstColon = header.indexOf(':');
  int secondColon = header.indexOf(':', firstColon + 1);

  if (firstColon <= 0 || secondColon <= firstColon + 1) {
    return false;
  }

  type = header.substring(0, firstColon);
  filename = header.substring(firstColon + 1, secondColon);
  String sizeString = header.substring(secondColon + 1);

  if (type != "TEXT" && type != "FILE") {
    return false;
  }

  if (filename.length() == 0 || sizeString.length() == 0) {
    return false;
  }

  for (size_t i = 0; i < sizeString.length(); i++) {
    if (!isDigit(sizeString[i])) {
      return false;
    }
  }

  size = (size_t)sizeString.toInt();
  return true;
}

// ---------------------------------------------------------------------------
// Outbound path: WiFi client -> LoRa
// ---------------------------------------------------------------------------

// Build and start transmitting the fragment currently indexed by txNextFrag.
void sendNextFragment() {
  uint8_t buf[LORA_HDR + LORA_CHUNK];
  buf[0] = BROADCAST_ADDR;
  buf[1] = localAddress;
  buf[2] = txMsgId;
  buf[3] = (uint8_t)txNextFrag;
  buf[4] = (uint8_t)txFragCount;
  size_t len = LORA_HDR;

  if (txNextFrag == 0) {
    // Fragment 0 carries the metadata line the far side needs to re-frame.
    String meta = txType + ":" + txFilename + ":" + String((unsigned int)txPayloadSize);
    size_t n = meta.length();
    if (n > LORA_CHUNK) {
      n = LORA_CHUNK;
    }
    memcpy(buf + LORA_HDR, meta.c_str(), n);
    len += n;
  } else {
    size_t offset = (size_t)(txNextFrag - 1) * LORA_CHUNK;
    size_t chunk = txPayloadSize - offset;
    if (chunk > LORA_CHUNK) {
      chunk = LORA_CHUNK;
    }
    memcpy(buf + LORA_HDR, txAssembly + offset, chunk);
    len += chunk;
  }

  int state = radio.startTransmit(buf, len);
  if (state == RADIOLIB_ERR_NONE) {
    transmitting = true;
    Serial.printf(
      "[TX] msg %u frag %u/%u (%u bytes)\n",
      txMsgId,
      (unsigned int)txNextFrag,
      (unsigned int)(txFragCount - 1),
      (unsigned int)len
    );
  } else {
    Serial.printf("[TX] startTransmit failed, code %d\n", state);
    txActive = false;
    if (currentClient && currentClient.connected()) {
      currentClient.println("ERROR:TX_FAILED");
    }
    radio.startReceive();
  }
}

// Queue a fully-received TCP message (payload already staged in txAssembly).
void startBridgeTx(const String& type, const String& filename, size_t size) {
  txType = type;
  txFilename = filename;
  txPayloadSize = size;
  txMsgId++;

  uint16_t dataFrags = (uint16_t)((size + LORA_CHUNK - 1) / LORA_CHUNK); // 0 when size==0
  txFragCount = 1 + dataFrags;
  txNextFrag = 0;
  txActive = true;
  bridgedTx++;

  Serial.printf(
    "[BRIDGE] WiFi->LoRa msg %u: %s '%s' %u bytes in %u frags\n",
    txMsgId,
    type.c_str(),
    filename.c_str(),
    (unsigned int)size,
    (unsigned int)txFragCount
  );
}

// ---------------------------------------------------------------------------
// Inbound path: LoRa -> WiFi client
// ---------------------------------------------------------------------------

void resetRx() {
  rxActive = false;
  rxMetaGot = false;
  rxReceivedCount = 0;
  rxFragCount = 0;
  rxSize = 0;
  for (uint16_t i = 0; i < MAX_FRAGS; i++) {
    rxFragGot[i] = false;
  }
}

void deliverToClient() {
  if (currentClient && currentClient.connected()) {
    String hdr = "RECV:" + rxType + ":" + rxFilename + ":" + String((unsigned int)rxSize) + "\n";
    currentClient.print(hdr);
    if (rxSize > 0) {
      currentClient.write(rxAssembly, rxSize);
    }
    currentClient.flush();
    Serial.printf(
      "[BRIDGE] LoRa->WiFi delivered %s '%s' %u bytes to client\n",
      rxType.c_str(),
      rxFilename.c_str(),
      (unsigned int)rxSize
    );
  } else {
    Serial.printf(
      "[BRIDGE] LoRa msg complete (%u bytes) but no WiFi client connected; dropped\n",
      (unsigned int)rxSize
    );
  }
}

// Add one received LoRa fragment to the reassembly slot; deliver when complete.
void feedReassembler(
  uint8_t src,
  uint8_t msgId,
  uint8_t fragIndex,
  uint8_t fragCount,
  const uint8_t* data,
  size_t dataLen
) {
  if (fragCount == 0 || fragCount > MAX_FRAGS) {
    return;
  }
  if (fragIndex >= fragCount) {
    return;
  }

  // A different (src, msgId) starts a fresh message and drops any partial one.
  if (!rxActive || src != rxSrc || msgId != rxMsgId) {
    resetRx();
    rxActive = true;
    rxSrc = src;
    rxMsgId = msgId;
    rxFragCount = fragCount;
  }
  rxLastMillis = millis();

  if (fragIndex == 0) {
    String meta = "";
    for (size_t i = 0; i < dataLen; i++) {
      meta += (char)data[i];
    }
    String t;
    String f;
    size_t s = 0;
    if (!parseHeader(meta, t, f, s)) {
      Serial.println("[RX] bad metadata fragment; dropping message");
      resetRx();
      return;
    }
    if (s > MAX_BRIDGE_PAYLOAD) {
      Serial.println("[RX] incoming payload too large; dropping message");
      resetRx();
      return;
    }
    rxType = t;
    rxFilename = f;
    rxSize = s;
    rxMetaGot = true;
  } else {
    size_t offset = (size_t)(fragIndex - 1) * LORA_CHUNK;
    if (offset + dataLen > MAX_BRIDGE_PAYLOAD) {
      Serial.println("[RX] fragment out of bounds; dropping message");
      resetRx();
      return;
    }
    memcpy(rxAssembly + offset, data, dataLen);
  }

  if (!rxFragGot[fragIndex]) {
    rxFragGot[fragIndex] = true;
    rxReceivedCount++;
  }

  if (rxMetaGot && rxReceivedCount == rxFragCount) {
    bridgedRx++;
    deliverToClient();
    resetRx();
  }
}

// Read the packet the radio just latched and hand it to the reassembler.
void handleReceivedPacket() {
  uint8_t buf[LORA_HDR + LORA_CHUNK];
  size_t len = radio.getPacketLength();
  if (len > sizeof(buf)) {
    len = sizeof(buf);
  }

  int state = radio.readData(buf, len);
  if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("[RX] packet received but CRC failed.");
    return;
  }
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[RX] readData failed, code %d\n", state);
    return;
  }
  if (len < LORA_HDR) {
    Serial.println("[RX] runt packet ignored.");
    return;
  }

  uint8_t dst = buf[0];
  uint8_t src = buf[1];
  uint8_t msgId = buf[2];
  uint8_t fragIndex = buf[3];
  uint8_t fragCount = buf[4];

  if (dst != localAddress && dst != BROADCAST_ADDR) {
    return;  // not addressed to us
  }

  Serial.printf(
    "[RX] from 0x%02X msg %u frag %u/%u (RSSI %.1f dBm, SNR %.1f dB)\n",
    src,
    msgId,
    fragIndex,
    fragCount ? (fragCount - 1) : 0,
    radio.getRSSI(),
    radio.getSNR()
  );

  feedReassembler(src, msgId, fragIndex, fragCount, buf + LORA_HDR, len - LORA_HDR);
}

// ---------------------------------------------------------------------------
// TCP client servicing (non-blocking)
// ---------------------------------------------------------------------------

void resetWifiParser() {
  wifiState = WIFI_HEADER;
  wifiHeader = "";
  wifiType = "";
  wifiFilename = "";
  wifiPayloadSize = 0;
  wifiPayloadRead = 0;
  wifiSkipRemaining = 0;
}

void processWifiByte(uint8_t c) {
  switch (wifiState) {
    case WIFI_HEADER: {
      if (c == '\n') {
        String line = wifiHeader;
        line.trim();
        wifiHeader = "";

        if (line.length() == 0) {
          return;
        }

        if (line == "HELLO") {
          if (currentClient && currentClient.connected()) {
            currentClient.println("ESP32_DORA_OK");
          }
          Serial.println("[BRIDGE] handshake answered.");
          return;
        }

        String t;
        String f;
        size_t s = 0;
        if (!parseHeader(line, t, f, s)) {
          if (currentClient && currentClient.connected()) {
            currentClient.println("ERROR:BAD_HEADER");
          }
          Serial.printf("[BRIDGE] bad header: %s\n", line.c_str());
          return;
        }

        if (s > MAX_BRIDGE_PAYLOAD) {
          if (currentClient && currentClient.connected()) {
            currentClient.println("ERROR:PAYLOAD_TOO_LARGE");
          }
          Serial.println("[BRIDGE] client payload too large; skipping.");
          wifiSkipRemaining = s;
          wifiState = (s > 0) ? WIFI_SKIP : WIFI_HEADER;
          return;
        }

        if (txActive) {
          // A previous message is still going out over LoRa.
          if (currentClient && currentClient.connected()) {
            currentClient.println("ERROR:BUSY");
          }
          Serial.println("[BRIDGE] busy sending previous LoRa message; skipping new one.");
          wifiSkipRemaining = s;
          wifiState = (s > 0) ? WIFI_SKIP : WIFI_HEADER;
          return;
        }

        wifiType = t;
        wifiFilename = f;
        wifiPayloadSize = s;
        wifiPayloadRead = 0;

        if (s == 0) {
          startBridgeTx(wifiType, wifiFilename, 0);
          wifiState = WIFI_HEADER;
        } else {
          wifiState = WIFI_PAYLOAD;
        }
      } else if (c != '\r') {
        if (wifiHeader.length() < MAX_HEADER_LENGTH) {
          wifiHeader += (char)c;
        } else {
          if (currentClient && currentClient.connected()) {
            currentClient.println("ERROR:HEADER_TOO_LONG");
          }
          wifiHeader = "";
        }
      }
      break;
    }

    case WIFI_PAYLOAD: {
      txAssembly[wifiPayloadRead++] = c;
      if (wifiPayloadRead >= wifiPayloadSize) {
        startBridgeTx(wifiType, wifiFilename, wifiPayloadSize);
        wifiState = WIFI_HEADER;
      }
      break;
    }

    case WIFI_SKIP: {
      if (wifiSkipRemaining > 0) {
        wifiSkipRemaining--;
      }
      if (wifiSkipRemaining == 0) {
        wifiState = WIFI_HEADER;
      }
      break;
    }
  }
}

void serviceWifi() {
  if (!currentClient || !currentClient.connected()) {
    WiFiClient incoming = tcpServer.available();
    if (incoming) {
      currentClient = incoming;
      currentClient.setNoDelay(true);
      resetWifiParser();
      Serial.println("[BRIDGE] TCP client connected.");
    }
  }

  // Process a bounded number of bytes per pass so the radio still gets serviced.
  int guard = 0;
  while (currentClient && currentClient.connected() && currentClient.available() && guard++ < 4096) {
    processWifiByte((uint8_t)currentClient.read());
  }
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== XIAO ESP32-S3 + Wio-SX1262 WiFi<->LoRa bridge ===");

  // Unique id from the low byte of the chip MAC, so the same binary running on
  // both boards yields two different ids without editing the code per board.
  uint64_t mac = ESP.getEfuseMac();
  localAddress = (uint8_t)(mac & 0xFF);
  if (localAddress == 0x00 || localAddress == BROADCAST_ADDR) {
    localAddress = 0x01;
  }
  Serial.printf("This node id: 0x%02X\n", localAddress);

  resetRx();
  resetWifiParser();

  // Bring up SPI on the XIAO ESP32-S3 pins the Wio-SX1262 is wired to.
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  Serial.printf("Starting SX1262 at %.1f MHz...\n", (double)LORA_FREQUENCY);
  // begin(freq, bw, sf, cr, syncWord, power, preamble, tcxoVoltage, useLDO)
  int state = radio.begin(
    LORA_FREQUENCY,
    125,   // bandwidth (kHz)
    9,     // spreading factor
    7,      // coding rate (4/8)
    RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
    13,     // TX power (dBm)
    8,     // preamble length
    LORA_TCXO_VOLTAGE,
    false   // use DC-DC + TCXO, not LDO
  );
  if (state != RADIOLIB_ERR_NONE) {
    halt("SX1262 init failed - check B2B connector seating and frequency", state);
  }

  // The Wio-SX1262 routes DIO2 to its internal RF switch; RadioLib toggles it
  // automatically between RX and TX when this is enabled.
  state = radio.setDio2AsRfSwitch(true);
  if (state != RADIOLIB_ERR_NONE) {
    halt("setDio2AsRfSwitch failed", state);
  }

  // Route the DIO1 interrupt to our flag setter, then start listening.
  radio.setDio1Action(onDio1);

  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    halt("startReceive failed", state);
  }

  // Bring up the WiFi Access Point and TCP server. The SSID is suffixed with
  // this board's node id (e.g. "dora-A3") so the two boards advertise distinct
  // networks; otherwise both would broadcast "dora" and you could not attach a
  // client to a specific board to observe the LoRa->WiFi receive path.
  char apSsid[32];
  snprintf(apSsid, sizeof(apSsid), "%s-%02X", AP_SSID, localAddress);
  Serial.printf("Starting WiFi Access Point '%s'...\n", apSsid);
  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(apSsid, AP_PASS)) {
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    tcpServer.begin();
    tcpServer.setNoDelay(true);
    Serial.printf("TCP server listening on port %u\n", TCP_PORT);
  } else {
    Serial.println("Failed to start Access Point.");
  }

  Serial.println("Bridge ready. Listening on LoRa and WiFi...");
}

void loop() {
  // ---- Handle a completed radio operation signalled by the DIO1 IRQ ----
  if (operationDone) {
    operationDone = false;

    if (transmitting) {
      radio.finishTransmit();
      transmitting = false;
      txNextFrag++;

      if (txNextFrag >= txFragCount) {
        // Whole message sent: ack the client and return to listening.
        txActive = false;
        if (currentClient && currentClient.connected()) {
          currentClient.println("OK");
        }
        Serial.printf("[BRIDGE] LoRa TX complete (msg %u); client ACKed.\n", txMsgId);
        radio.startReceive();
      }
      // Otherwise leave txActive set; the TX driver below sends the next frag
      // as a tight burst without dropping back to receive between fragments.
    } else {
      handleReceivedPacket();
      radio.startReceive();
    }
  }

  // ---- TX driver: send the next/first fragment when the radio is idle ----
  if (txActive && !transmitting) {
    sendNextFragment();
  }

  // ---- Service the TCP client (accept + non-blocking framed parse) ----
  serviceWifi();

  // ---- Drop a stalled inbound reassembly so its buffer can be reused ----
  if (rxActive && (millis() - rxLastMillis > REASSEMBLY_TIMEOUT_MS)) {
    Serial.println("[BRIDGE] reassembly timeout; dropping partial message.");
    resetRx();
  }
}
