#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// ---------------------------------------------------------------------------
// LoRa full-duplex connection test for the Seeed XIAO ESP32-S3 + Wio-SX1262 kit
//
// Flash this SAME sketch to BOTH boards. Each board:
//   * derives a unique 1-byte node id from its chip MAC,
//   * stays in receive mode and prints every packet it hears,
//   * periodically transmits a counter message to the other board.
//
// Note on "full duplex": the SX1262 has a single transceiver, so it cannot
// physically transmit and receive at the exact same instant. This sketch is
// interrupt-driven and spends nearly all of its time in receive mode, only
// briefly switching to transmit. A small random jitter is added to each send
// so the two boards do not fall into lockstep and constantly collide.
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

// How often (ms) each board sends a heartbeat message.
static const unsigned long SEND_INTERVAL_MS = 15000;

// SX1262 radio instance: Module(cs/NSS, irq/DIO1, reset, busy).
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

uint8_t localAddress = 0x00;   // this board's id (set from MAC in setup)
uint32_t txCounter = 0;        // number of messages this board has sent
uint32_t rxCounter = 0;        // number of messages this board has received
unsigned long lastSendTime = 0;
unsigned long sendInterval = SEND_INTERVAL_MS;

// Set by the DIO1 interrupt when the current TX or RX operation completes.
volatile bool operationDone = false;
// True while a transmit is in flight, so we know how to interpret the IRQ.
bool transmitting = false;

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

void startTx() {
  String message = "hello from 0x" + String(localAddress, HEX) +
                   " count=" + String(txCounter);

  int state = radio.startTransmit(message);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("[TX #%lu] \"%s\"\n", (unsigned long)txCounter, message.c_str());
    transmitting = true;
    txCounter++;
  } else {
    Serial.printf("[TX] startTransmit failed, code %d\n", state);
  }
}

void handleReceivedPacket() {
  String incoming;
  int state = radio.readData(incoming);

  if (state == RADIOLIB_ERR_NONE) {
    rxCounter++;
    Serial.printf(
      "[RX #%lu] \"%s\"  (RSSI %.1f dBm, SNR %.1f dB)\n",
      (unsigned long)rxCounter,
      incoming.c_str(),
      radio.getRSSI(),
      radio.getSNR()
    );
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println("[RX] packet received but CRC failed.");
  } else {
    Serial.printf("[RX] readData failed, code %d\n", state);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== XIAO ESP32-S3 + Wio-SX1262 LoRa full-duplex test ===");

  // Unique id from the low byte of the chip MAC, so the same binary running on
  // both boards yields two different ids without editing the code per board.
  uint64_t mac = ESP.getEfuseMac();
  localAddress = (uint8_t)(mac & 0xFF);
  if (localAddress == 0x00 || localAddress == 0xFF) {
    localAddress = 0x01;
  }
  Serial.printf("This node id: 0x%02X\n", localAddress);

  // Bring up SPI on the XIAO ESP32-S3 pins the Wio-SX1262 is wired to.
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  Serial.printf("Starting SX1262 at %.1f MHz...\n", (double)LORA_FREQUENCY);
  // begin(freq, bw, sf, cr, syncWord, power, preamble, tcxoVoltage, useLDO)
  int state = radio.begin(
    LORA_FREQUENCY,
    62.5,   // bandwidth (kHz)
    12,       // spreading factor
    8,       // coding rate (4/7)
    RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
    13,      // TX power (dBm)
    12,       // preamble length
    LORA_TCXO_VOLTAGE,
    false    // use DC-DC + TCXO, not LDO
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

  Serial.println("SX1262 init OK. Listening and transmitting...");
  randomSeed((uint32_t)mac);
  lastSendTime = millis();
}

void loop() {
  // ---- Handle a completed radio operation signalled by the DIO1 IRQ ----
  if (operationDone) {
    operationDone = false;

    if (transmitting) {
      // Transmit finished: release the TX resources and return to listening.
      radio.finishTransmit();
      transmitting = false;
      radio.startReceive();
    } else {
      // A packet arrived while we were listening.
      handleReceivedPacket();
      radio.startReceive();
    }
  }

  // ---- Periodically transmit a heartbeat (only when not already sending) ----
  if (!transmitting && millis() - lastSendTime >= sendInterval) {
    startTx();
    lastSendTime = millis();
    // 0..800 ms jitter so the two boards don't stay perfectly synchronized.
    sendInterval = SEND_INTERVAL_MS + (unsigned long)random(0, 800);
  }
}
