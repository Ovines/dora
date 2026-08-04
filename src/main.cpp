#include <Arduino.h>
#include <WiFi.h>

// TCP server
#define TCP_PORT 5000
#define LED_PIN 2

// SPI LoRa pins - currently defined but not used in this TCP echo code
#define LORA_SPI_SCK   18
#define LORA_SPI_MOSI  23
#define LORA_SPI_MISO  19
#define LORA_SPI_NSS   5
#define LORA_SPI_RST   14
#define LORA_SPI_DIO0  27

// UART LoRa pins - currently defined but not used in this TCP echo code
#define LORA_UART_RX 33  // ESP32 receives from LoRa TXD
#define LORA_UART_TX 32  // ESP32 transmits to LoRa RXD

const char* ssid = "dora";
const char* password = "dora1234";

WiFiServer tcpServer(TCP_PORT);

static const unsigned long LINE_TIMEOUT_MS = 10000;
static const unsigned long PAYLOAD_TIMEOUT_MS = 15000;
static const size_t MAX_HEADER_LENGTH = 256;
static const size_t BUFFER_SIZE = 1024;

// Important:
// The ESP32 must keep the received payload in RAM so it can send it back.
// Keep this value reasonable. Large files may fail due to RAM limits.
static const size_t MAX_ECHO_PAYLOAD_SIZE = 200 * 1024; // 200 KB

void blinkLed(int count, int delayMs = 80) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    delay(delayMs);
  }
}

bool readLineFromClient(WiFiClient& client, String& line, unsigned long timeoutMs) {
  line = "";
  unsigned long start = millis();

  while (client.connected() && millis() - start < timeoutMs) {
    while (client.available()) {
      char c = (char)client.read();

      if (c == '\n') {
        line.trim();
        return true;
      }

      if (c != '\r') {
        line += c;
      }

      if (line.length() > MAX_HEADER_LENGTH) {
        Serial.println("Header too long.");
        return false;
      }
    }

    delay(1);
  }

  Serial.println("Timed out while reading line.");
  return false;
}

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

bool readPayloadBytes(
  WiFiClient& client,
  uint8_t* payload,
  size_t size,
  const String& type,
  const String& filename
) {
  size_t totalRead = 0;
  unsigned long lastDataTime = millis();

  Serial.printf(
    "Receiving %s '%s' (%u bytes)\n",
    type.c_str(),
    filename.c_str(),
    (unsigned int)size
  );

  if (type == "TEXT") {
    Serial.print("Text content: ");
  }

  while (client.connected() && totalRead < size) {
    int availableBytes = client.available();

    if (availableBytes > 0) {
      size_t remaining = size - totalRead;
      size_t toRead = min((size_t)availableBytes, remaining);

      int bytesRead = client.read(payload + totalRead, toRead);

      if (bytesRead > 0) {
        if (type == "TEXT") {
          for (int i = 0; i < bytesRead; i++) {
            Serial.write(payload[totalRead + i]);
          }
        }

        totalRead += bytesRead;
        lastDataTime = millis();
      }
    } else {
      if (millis() - lastDataTime > PAYLOAD_TIMEOUT_MS) {
        Serial.println();
        Serial.println("Timed out while reading payload.");
        return false;
      }

      delay(1);
    }
  }

  if (type == "TEXT") {
    Serial.println();
  }

  Serial.printf(
    "Finished receiving %u / %u bytes\n",
    (unsigned int)totalRead,
    (unsigned int)size
  );

  return totalRead == size;
}

bool sendEchoPacket(
  WiFiClient& client,
  const String& type,
  const String& filename,
  const uint8_t* payload,
  size_t size
) {
  String echoHeader = "ECHO:" + type + ":" + filename + ":" + String(size) + "\n";

  size_t headerWritten = client.print(echoHeader);
  if (headerWritten == 0) {
    Serial.println("Failed to send echo header.");
    return false;
  }

  size_t totalSent = 0;

  while (client.connected() && totalSent < size) {
    size_t remaining = size - totalSent;
    size_t chunkSize = min(remaining, (size_t)BUFFER_SIZE);

    size_t sent = client.write(payload + totalSent, chunkSize);

    if (sent == 0) {
      Serial.println("Failed while sending echo payload.");
      return false;
    }

    totalSent += sent;
    delay(1);
  }

  client.flush();

  Serial.printf(
    "Echo sent: %s '%s' (%u bytes)\n",
    type.c_str(),
    filename.c_str(),
    (unsigned int)size
  );

  return totalSent == size;
}

void handleClient(WiFiClient& client) {
  Serial.println("TCP client connected.");

  client.setNoDelay(true);

  String line;

  while (client.connected()) {
    bool gotLine = readLineFromClient(client, line, LINE_TIMEOUT_MS);

    if (!gotLine) {
      break;
    }

    if (line.length() == 0) {
      continue;
    }

    Serial.print("Header/message received: ");
    Serial.println(line);

    if (line == "HELLO") {
      client.println("ESP32_DORA_OK");
      Serial.println("Handshake answered.");
      continue;
    }

    String type;
    String filename;
    size_t payloadSize = 0;

    if (!parseHeader(line, type, filename, payloadSize)) {
      Serial.println("Invalid header.");
      client.println("ERROR:BAD_HEADER");
      continue;
    }

    if (payloadSize > MAX_ECHO_PAYLOAD_SIZE) {
      Serial.println("Payload too large for echo.");
      client.println("ERROR:PAYLOAD_TOO_LARGE");
      continue;
    }

    uint8_t* payload = nullptr;

    if (payloadSize > 0) {
      payload = (uint8_t*)malloc(payloadSize);

      if (payload == nullptr) {
        Serial.println("Failed to allocate payload buffer.");
        client.println("ERROR:NO_MEMORY");
        continue;
      }
    }

    bool ok = readPayloadBytes(client, payload, payloadSize, type, filename);

    if (!ok) {
      client.println("ERROR:READ_FAILED");

      if (payload != nullptr) {
        free(payload);
      }

      break;
    }

    client.println("OK");
    blinkLed(1);

    bool echoOk = sendEchoPacket(client, type, filename, payload, payloadSize);

    if (!echoOk) {
      Serial.println("Echo failed.");
      if (payload != nullptr) {
        free(payload);
      }
      break;
    }

    if (payload != nullptr) {
      free(payload);
    }

    delay(1);
  }

  client.stop();
  Serial.println("TCP client disconnected.");
}

void setup() {
  Serial.begin(115200);
  Serial.println("hello world");
  delay(200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println();
  Serial.printf("Starting WiFi Access Point '%s'...\n", ssid);

  WiFi.mode(WIFI_AP);

  bool result = WiFi.softAP(ssid, password);

  if (result) {
    Serial.println("Access Point started successfully.");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    tcpServer.begin();
    tcpServer.setNoDelay(true);

    Serial.printf("TCP server listening on port %d\n", TCP_PORT);
    blinkLed(3);
  } else {
    Serial.println("Failed to start Access Point.");
  }
}

void loop() {
  WiFiClient client = tcpServer.available();

  if (client) {
    handleClient(client);
  }

  delay(1);
}