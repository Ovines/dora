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
// LoRa packet layout: [dst][src][kind][msgId][fragIndex][fragCount] + payload.
static const size_t   LORA_HDR = 6;
// Data bytes per LoRa packet. 6 + 200 = 206 <= SX1262 max (255).
static const size_t   LORA_CHUNK = 200;
// Largest message we will bridge in either direction.
static const size_t   MAX_BRIDGE_PAYLOAD = 20480;
// Enough fragments to cover MAX_BRIDGE_PAYLOAD (1 metadata frag + data chunks),
// derived so it can never fall behind MAX_BRIDGE_PAYLOAD. NOTE: fragIndex and
// fragCount are single bytes in the LoRa header, so this must stay <= 255, i.e.
// MAX_BRIDGE_PAYLOAD up to ~50 KB at LORA_CHUNK = 200.
static const uint16_t MAX_FRAGS = (MAX_BRIDGE_PAYLOAD + LORA_CHUNK - 1) / LORA_CHUNK + 2;
// dst value that every node accepts.
static const uint8_t  BROADCAST_ADDR = 0xFF;
// Cap on a single TCP header line, matching the master protocol.
static const size_t   MAX_HEADER_LENGTH = 256;

// Uncomment to simulate a single random dropped LoRa data fragment per outbound
// message and exercise the NACK-based ARQ recovery path.
#define SIMULATE_RANDOM_DROP

// ---- Reliability (NACK-based ARQ) ------------------------------------------
// Packet kinds carried in the header 'kind' byte.
enum LoraKind {
  KIND_DATA = 0,   // fragIndex/fragCount valid; payload = metadata (frag 0) or chunk
  KIND_END  = 1,   // sender -> receiver: all fragments for this pass were sent
  KIND_NACK = 2,   // receiver -> sender: payload = missing fragment indices (1 byte each)
  KIND_DONE = 3    // receiver -> sender: full message received
};
// After sending END, how long the sender waits for a NACK/DONE before it
// re-prompts by resending END.
static const unsigned long AWAIT_TIMEOUT_MS = 4000;
// Resend passes (NACK rounds + END re-prompts) before the sender gives up.
static const int           MAX_TX_ROUNDS = 12;
// Drop a half-reassembled inbound message if it stalls this long. Must comfortably
// exceed MAX_TX_ROUNDS * AWAIT_TIMEOUT_MS so a slow recovery is not dropped early.
static const unsigned long REASSEMBLY_TIMEOUT_MS = 90000;

// SX1262 radio instance: Module(cs/NSS, irq/DIO1, reset, busy).
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

WiFiServer tcpServer(TCP_PORT);
WiFiClient currentClient;   // single active TCP client

uint8_t localAddress = 0x00;   // this board's id (set from MAC in setup)

// Set by the DIO1 interrupt when the current TX or RX operation completes.
volatile bool operationDone = false;
// True while a LoRa transmit is in flight, so we know how to read the IRQ.
bool transmitting = false;

// What the current in-flight transmit is, so the completion IRQ knows what to do
// next (keep sending data, wait for a reply, or just resume listening).
enum RadioTx { RTX_NONE, RTX_DATA, RTX_END, RTX_NACK, RTX_DONE };
RadioTx currentTx = RTX_NONE;

// ---- Outbound (WiFi client -> LoRa) message state --------------------------
enum TxPhase { TX_IDLE, TX_SENDING, TX_AWAIT };
uint8_t  txAssembly[MAX_BRIDGE_PAYLOAD];  // payload staged from the TCP client
bool     txActive = false;                // a message is being sent over LoRa
TxPhase  txPhase = TX_IDLE;
uint8_t  txMsgId = 0;                      // rolling id stamped on each message
String   txType;
String   txFilename;
size_t   txPayloadSize = 0;
uint16_t txFragCount = 0;                  // metadata frag + data frags
bool     txToSend[MAX_FRAGS];              // fragments still to send this pass
int      txRound = 0;                      // resend pass counter (1-based)
unsigned long txAwaitStart = 0;            // when we began waiting after END
#ifdef SIMULATE_RANDOM_DROP
uint16_t txDropFragIndex = 0;              // simulate one dropped fragment per send
bool     txDropInjected = false;
#endif
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

// ---- Saved last inbound message (survives WiFi client disconnects) ----------
// A single-slot RAM copy of the most recently completed inbound message. It is
// held separately from rxAssembly (which the ARQ receiver reuses for the next
// message) so a client that was absent, or that dropped mid-transfer, still
// gets the message once it (re)connects. In-RAM only: lost on reboot.
uint8_t  savedPayload[MAX_BRIDGE_PAYLOAD];
bool     savedValid = false;     // a message has been stored
bool     savedPending = false;   // stored message still needs a full delivery
bool     savedAwaitingAck = false; // frame was written; waiting for client ACK
unsigned long savedSentMillis = 0; // when the frame was last written to a client
String   savedType;
String   savedFilename;
size_t   savedSize = 0;
// A local TCP write only proves the bytes were buffered, not that the client
// received them (a client whose WiFi dropped leaves a half-open socket that
// still reports connected()). So delivery is confirmed by an application-level
// "ACK" line from the app; until then the message stays pending and is resent.
static const unsigned long DELIVERY_ACK_TIMEOUT_MS = 8000;

// Remember the most recently completed inbound message so a late END (whose DONE
// was lost) can be re-acknowledged without re-delivering to the client.
bool     lastCompletedValid = false;
uint8_t  lastCompletedSrc = 0;
uint8_t  lastCompletedMsgId = 0;

// Pending receiver control response, transmitted from the scheduler when the
// radio is idle. NACK-vs-DONE and the missing list are (re)computed at send time.
bool     pendingRespond = false;
uint8_t  pendingDst = 0;
uint8_t  pendingMsgId = 0;

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
// Outbound path: WiFi client -> LoRa (sender ARQ)
// ---------------------------------------------------------------------------

// Abort the current outbound message and tell the WiFi client why.
void failTx(const char* reason) {
  txActive = false;
  txPhase = TX_IDLE;
  if (currentClient && currentClient.connected()) {
    currentClient.print("ERROR:");
    currentClient.println(reason);
  }
  radio.startReceive();
}

// Transmit one data fragment: metadata line for index 0, payload chunk otherwise.
void sendData(uint16_t index) {
  uint8_t buf[LORA_HDR + LORA_CHUNK];
  buf[0] = BROADCAST_ADDR;
  buf[1] = localAddress;
  buf[2] = KIND_DATA;
  buf[3] = txMsgId;
  buf[4] = (uint8_t)index;
  buf[5] = (uint8_t)txFragCount;
  size_t len = LORA_HDR;

  if (index == 0) {
    // Fragment 0 carries the metadata line the far side needs to re-frame.
    String meta = txType + ":" + txFilename + ":" + String((unsigned int)txPayloadSize);
    size_t n = meta.length();
    if (n > LORA_CHUNK) {
      n = LORA_CHUNK;
    }
    memcpy(buf + LORA_HDR, meta.c_str(), n);
    len += n;
  } else {
    size_t offset = (size_t)(index - 1) * LORA_CHUNK;
    size_t chunk = txPayloadSize - offset;
    if (chunk > LORA_CHUNK) {
      chunk = LORA_CHUNK;
    }
    memcpy(buf + LORA_HDR, txAssembly + offset, chunk);
    len += chunk;
  }

#ifdef SIMULATE_RANDOM_DROP
  if (!txDropInjected && index == txDropFragIndex) {
    txDropInjected = true;
    Serial.printf("[TX] simulated drop of msg %u frag %u/%u\n", txMsgId, (unsigned int)index, (unsigned int)(txFragCount - 1));
    return;
  }
#endif

  int state = radio.startTransmit(buf, len);
  if (state == RADIOLIB_ERR_NONE) {
    transmitting = true;
    currentTx = RTX_DATA;
    Serial.printf(
      "[TX] msg %u DATA frag %u/%u (%u bytes)\n",
      txMsgId,
      (unsigned int)index,
      (unsigned int)(txFragCount - 1),
      (unsigned int)len
    );
  } else {
    Serial.printf("[TX] startTransmit(DATA) failed, code %d\n", state);
    failTx("TX_FAILED");
  }
}

// Transmit an END marker: "all fragments for this pass have been sent".
void sendEnd() {
  uint8_t buf[LORA_HDR];
  buf[0] = BROADCAST_ADDR;
  buf[1] = localAddress;
  buf[2] = KIND_END;
  buf[3] = txMsgId;
  buf[4] = 0;
  buf[5] = (uint8_t)txFragCount;

  int state = radio.startTransmit(buf, LORA_HDR);
  if (state == RADIOLIB_ERR_NONE) {
    transmitting = true;
    currentTx = RTX_END;
    Serial.printf("[TX] msg %u END (round %d)\n", txMsgId, txRound);
  } else {
    Serial.printf("[TX] startTransmit(END) failed, code %d\n", state);
    failTx("TX_FAILED");
  }
}

// Index of the lowest fragment still needing transmission this pass, or -1.
int nextToSend() {
  for (uint16_t i = 0; i < txFragCount; i++) {
    if (txToSend[i]) {
      return (int)i;
    }
  }
  return -1;
}

// Queue a fully-received TCP message (payload already staged in txAssembly).
void startBridgeTx(const String& type, const String& filename, size_t size) {
  txType = type;
  txFilename = filename;
  txPayloadSize = size;
  txMsgId++;

  uint16_t dataFrags = (uint16_t)((size + LORA_CHUNK - 1) / LORA_CHUNK); // 0 when size==0
  txFragCount = 1 + dataFrags;
  for (uint16_t i = 0; i < MAX_FRAGS; i++) {
    txToSend[i] = (i < txFragCount);
  }
#ifdef SIMULATE_RANDOM_DROP
  txDropInjected = false;
  txDropFragIndex = (uint16_t)random(0, txFragCount);
#endif
  txPhase = TX_SENDING;
  txRound = 1;
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

// A NACK arrived for our in-flight message: re-mark the missing fragments and
// start another send pass, or give up once we exceed MAX_TX_ROUNDS.
void onNack(uint8_t msgId, const uint8_t* missing, size_t count) {
  if (!txActive || msgId != txMsgId) {
    return;
  }
  for (size_t i = 0; i < count; i++) {
    uint8_t idx = missing[i];
    if (idx < txFragCount) {
      txToSend[idx] = true;
    }
  }
  txRound++;
  Serial.printf(
    "[ARQ] NACK for msg %u: %u frag(s) to resend (round %d)\n",
    msgId, (unsigned int)count, txRound);

  if (txRound > MAX_TX_ROUNDS) {
    Serial.printf("[ARQ] msg %u exceeded MAX_TX_ROUNDS; giving up\n", msgId);
    failTx("LORA_INCOMPLETE");
  } else {
    txPhase = TX_SENDING;
  }
}

// A DONE arrived for our in-flight message: report success to the WiFi client.
void onDone(uint8_t msgId) {
  if (!txActive || msgId != txMsgId) {
    return;
  }
  Serial.printf("[ARQ] DONE for msg %u; delivery confirmed\n", msgId);
  txActive = false;
  txPhase = TX_IDLE;
  if (currentClient && currentClient.connected()) {
    currentClient.println("OK");
  }
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

// Copy the freshly reassembled message into the saved slot and mark it as
// needing delivery. A newer message overwrites the previous slot.
void saveInbound() {
  savedType = rxType;
  savedFilename = rxFilename;
  savedSize = rxSize;
  if (rxSize > 0) {
    memcpy(savedPayload, rxAssembly, rxSize);
  }
  savedValid = true;
  savedPending = true;
  Serial.printf(
    "[BRIDGE] stored inbound %s '%s' %u bytes (awaiting client)\n",
    savedType.c_str(),
    savedFilename.c_str(),
    (unsigned int)savedSize
  );
}

// Write the saved message to the connected client and wait for its ACK. The
// pending flag is cleared only when the client ACKs (see processWifiByte), NOT
// on a successful local write, because a half-open socket accepts buffered
// writes without the client ever receiving them. Resends after a timeout so a
// lost frame (or a stale socket that later fills up) is retried.
void tryDeliverSaved() {
  if (!savedPending || !currentClient || !currentClient.connected()) {
    return;
  }
  // Already sent and still within the ACK window: keep waiting.
  if (savedAwaitingAck && (millis() - savedSentMillis < DELIVERY_ACK_TIMEOUT_MS)) {
    return;
  }

  String hdr = "RECV:" + savedType + ":" + savedFilename + ":" + String((unsigned int)savedSize) + "\n";
  size_t hw = currentClient.print(hdr);
  size_t pw = (savedSize > 0) ? currentClient.write(savedPayload, savedSize) : 0;
  currentClient.flush();

  if ((hw == hdr.length()) && (pw == savedSize) && currentClient.connected()) {
    savedAwaitingAck = true;
    savedSentMillis = millis();
    Serial.printf(
      "[BRIDGE] sent %s '%s' %u bytes; awaiting client ACK\n",
      savedType.c_str(),
      savedFilename.c_str(),
      (unsigned int)savedSize
    );
  } else {
    // The write failed part-way: the socket is dead (likely half-open). Drop it
    // so a genuinely new client can be accepted; the message stays pending.
    savedAwaitingAck = false;
    Serial.printf(
      "[BRIDGE] delivery write failed (%u/%u bytes); dropping stale client\n",
      (unsigned int)pw,
      (unsigned int)savedSize
    );
    currentClient.stop();
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
    Serial.printf(
      "[RX] fragCount %u exceeds MAX_FRAGS %u; message too large, dropping\n",
      fragCount,
      (unsigned int)MAX_FRAGS
    );
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
    saveInbound();
    tryDeliverSaved();
    // Remember this message and queue a DONE ack back to the sender. The sender
    // keeps re-sending END until it hears DONE, so a lost DONE is re-requested.
    lastCompletedValid = true;
    lastCompletedSrc = rxSrc;
    lastCompletedMsgId = rxMsgId;
    pendingRespond = true;
    pendingDst = rxSrc;
    pendingMsgId = rxMsgId;
    resetRx();
  }
}

// Queue a control reply (NACK or DONE) for later transmission by the scheduler.
void queueRespond(uint8_t dst, uint8_t msgId) {
  pendingRespond = true;
  pendingDst = dst;
  pendingMsgId = msgId;
}

// Sender says "that's the whole pass". Reply DONE if we already have everything
// (or already completed this message), otherwise NACK the fragments we lack.
void onEnd(uint8_t src, uint8_t msgId, uint8_t fragCount) {
  // Already delivered this exact message: re-acknowledge (handles a lost DONE).
  bool activeThis = rxActive && rxSrc == src && rxMsgId == msgId;
  if (!activeThis && lastCompletedValid && src == lastCompletedSrc && msgId == lastCompletedMsgId) {
    queueRespond(src, msgId);   // sendPendingControl will emit DONE
    return;
  }

  // Seed a reassembly slot from END if no DATA fragment has arrived yet.
  if (!activeThis) {
    if (fragCount == 0 || fragCount > MAX_FRAGS) {
      return;
    }
    resetRx();
    rxActive = true;
    rxSrc = src;
    rxMsgId = msgId;
    rxFragCount = fragCount;
  }
  rxLastMillis = millis();
  queueRespond(src, msgId);   // sendPendingControl computes NACK (missing) or DONE
}

// Transmit the queued receiver control reply. NACK-vs-DONE and the missing list
// are recomputed here so they are never stale by the time the radio is free.
void sendPendingControl() {
  bool sendDone = false;
  uint8_t missing[MAX_FRAGS];
  size_t missingCount = 0;

  bool activeThis = rxActive && rxSrc == pendingDst && rxMsgId == pendingMsgId;
  if (activeThis) {
    for (uint16_t i = 0; i < rxFragCount; i++) {
      if (!rxFragGot[i]) {
        if (missingCount < sizeof(missing)) {
          missing[missingCount++] = (uint8_t)i;
        }
      }
    }
    if (missingCount == 0) {
      sendDone = true;  // became complete meanwhile
    }
  } else if (lastCompletedValid && pendingDst == lastCompletedSrc &&
             pendingMsgId == lastCompletedMsgId) {
    sendDone = true;
  } else {
    pendingRespond = false;  // nothing to say about this message anymore
    return;
  }

  uint8_t buf[LORA_HDR + MAX_FRAGS];
  buf[0] = pendingDst;
  buf[1] = localAddress;
  buf[2] = sendDone ? (uint8_t)KIND_DONE : (uint8_t)KIND_NACK;
  buf[3] = pendingMsgId;
  buf[4] = 0;
  buf[5] = (uint8_t)(activeThis ? rxFragCount : 0);
  size_t len = LORA_HDR;
  if (!sendDone) {
    memcpy(buf + LORA_HDR, missing, missingCount);
    len += missingCount;
  }

  int state = radio.startTransmit(buf, len);
  if (state == RADIOLIB_ERR_NONE) {
    transmitting = true;
    currentTx = sendDone ? RTX_DONE : RTX_NACK;
    pendingRespond = false;
    if (sendDone) {
      Serial.printf("[TX] msg %u DONE -> 0x%02X\n", pendingMsgId, pendingDst);
    } else {
      Serial.printf(
        "[TX] msg %u NACK %u missing -> 0x%02X\n",
        pendingMsgId, (unsigned int)missingCount, pendingDst);
    }
  } else {
    // Leave pendingRespond set so we retry on the next idle pass.
    Serial.printf("[TX] startTransmit(control) failed, code %d\n", state);
    radio.startReceive();
  }
}

// Read the packet the radio just latched, parse the header, and dispatch by kind.
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
  uint8_t kind = buf[2];
  uint8_t msgId = buf[3];
  uint8_t fragIndex = buf[4];
  uint8_t fragCount = buf[5];

  if (dst != localAddress && dst != BROADCAST_ADDR) {
    return;  // not addressed to us
  }
  if (src == localAddress) {
    return;  // ignore anything that looks like our own transmission
  }

  const uint8_t* payload = buf + LORA_HDR;
  size_t payloadLen = len - LORA_HDR;

  switch (kind) {
    case KIND_DATA:
      Serial.printf(
        "[RX] DATA from 0x%02X msg %u frag %u/%u (RSSI %.1f dBm, SNR %.1f dB)\n",
        src, msgId, fragIndex, fragCount ? (fragCount - 1) : 0,
        radio.getRSSI(), radio.getSNR());
      feedReassembler(src, msgId, fragIndex, fragCount, payload, payloadLen);
      break;
    case KIND_END:
      Serial.printf("[RX] END from 0x%02X msg %u\n", src, msgId);
      onEnd(src, msgId, fragCount);
      break;
    case KIND_NACK:
      Serial.printf(
        "[RX] NACK from 0x%02X msg %u (%u missing)\n",
        src, msgId, (unsigned int)payloadLen);
      onNack(msgId, payload, payloadLen);
      break;
    case KIND_DONE:
      Serial.printf("[RX] DONE from 0x%02X msg %u\n", src, msgId);
      onDone(msgId);
      break;
    default:
      Serial.printf("[RX] unknown kind %u from 0x%02X; ignored\n", kind, src);
      break;
  }
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

        if (line == "ACK") {
          // Client confirmed it received the saved message: delivery is done.
          if (savedPending && savedAwaitingAck) {
            savedPending = false;
            savedAwaitingAck = false;
            Serial.printf(
              "[BRIDGE] client ACKed %s '%s'; delivery confirmed\n",
              savedType.c_str(),
              savedFilename.c_str()
            );
          }
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
  // A newly arriving client always takes over. This also recovers from a
  // half-open socket (client's WiFi dropped without a FIN, so connected() still
  // reads true): the phone's reconnect brings a fresh socket we switch to here.
  if (tcpServer.hasClient()) {
    WiFiClient incoming = tcpServer.available();
    if (incoming) {
      if (currentClient) {
        currentClient.stop();
      }
      currentClient = incoming;
      currentClient.setNoDelay(true);
      resetWifiParser();
      // Re-arm delivery so any pending message is resent to this new client.
      savedAwaitingAck = false;
      Serial.println("[BRIDGE] TCP client connected.");
    }
  }

  // Flush any message that arrived while no client was connected (or that was
  // interrupted by a mid-transfer disconnect). No-op when nothing is pending.
  tryDeliverSaved();

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
  randomSeed((uint32_t)micros());

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

// Decide what to do once the radio finishes the transmit indicated by currentTx.
void onTransmitComplete() {
  switch (currentTx) {
    case RTX_DATA:
      // More of this pass may remain; the scheduler sends the next fragment or
      // the END marker without dropping back to receive between fragments.
      break;
    case RTX_END:
      // Whole pass announced: wait for the receiver's NACK/DONE.
      txPhase = TX_AWAIT;
      txAwaitStart = millis();
      radio.startReceive();
      break;
    case RTX_NACK:
    case RTX_DONE:
    default:
      radio.startReceive();
      break;
  }
  currentTx = RTX_NONE;
}

// Drive the radio when it is idle: push the sender's current pass first, then
// flush any pending receiver control reply.
void scheduleRadio() {
  if (transmitting) {
    return;
  }

  if (txActive && txPhase == TX_SENDING) {
    int idx = nextToSend();
    if (idx >= 0) {
      txToSend[idx] = false;
      sendData((uint16_t)idx);
    } else {
      sendEnd();
    }
    return;
  }

  if (!txActive && pendingRespond) {
    sendPendingControl();
  }
}

void loop() {
  // ---- Handle a completed radio operation signalled by the DIO1 IRQ ----
  if (operationDone) {
    operationDone = false;

    if (transmitting) {
      radio.finishTransmit();
      transmitting = false;
      onTransmitComplete();
    } else {
      handleReceivedPacket();
      radio.startReceive();
    }
  }

  // ---- Sender await: re-prompt with END on timeout; give up after the cap ----
  if (txActive && txPhase == TX_AWAIT && !transmitting) {
    if (millis() - txAwaitStart > AWAIT_TIMEOUT_MS) {
      txRound++;
      if (txRound > MAX_TX_ROUNDS) {
        Serial.printf(
          "[ARQ] msg %u timed out after %d rounds; giving up\n", txMsgId, txRound);
        failTx("LORA_INCOMPLETE");
      } else {
        // Nothing marked to resend, so the scheduler resends END and re-awaits.
        Serial.printf("[ARQ] msg %u await timeout; re-prompting (round %d)\n",
          txMsgId, txRound);
        txPhase = TX_SENDING;
      }
    }
  }

  // ---- Drive the radio (runs after the await check so a re-prompt goes out) ----
  scheduleRadio();

  // ---- Service the TCP client (accept + non-blocking framed parse) ----
  serviceWifi();

  // ---- Drop a stalled inbound reassembly so its buffer can be reused ----
  if (rxActive && (millis() - rxLastMillis > REASSEMBLY_TIMEOUT_MS)) {
    Serial.println("[BRIDGE] reassembly timeout; dropping partial message.");
    resetRx();
  }
}
