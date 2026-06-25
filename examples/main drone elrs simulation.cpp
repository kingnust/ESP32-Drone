#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

// Wire ELRS receiver TX -> ESP32 RX, receiver RX -> ESP32 TX.
#define CRSF_RX_PIN 38
#define CRSF_TX_PIN 37

#define PRINT_PERIOD_MS 50

HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

uint32_t lastPrintMs = 0;
uint32_t uartBytesSeen = 0;
uint32_t lastUartByteMs = 0;
bool previousLinkUp = false;

void printStartupInfo() {
  Serial.println();
  Serial.println("ELRS 2.4GHz CRSF receiver monitor");
  Serial.print("CRSF UART baud: ");
  Serial.println(CRSF_BAUDRATE);
  Serial.print("ESP32 RX pin: GPIO");
  Serial.println(CRSF_RX_PIN);
  Serial.print("ESP32 TX pin: GPIO");
  Serial.println(CRSF_TX_PIN);
  Serial.println("Wiring: receiver TX -> ESP32 RX, receiver RX -> ESP32 TX, plus GND and power.");
  Serial.println("Waiting for RadioMaster Pocket / ELRS packets...");
}

void printRcHeader() {
  Serial.println("type,millis,link,uart_bytes,last_byte_ms,lq,rssi1,rssi2,snr,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,ch9,ch10,ch11,ch12,ch13,ch14,ch15,ch16");
}

void printRcFrame() {
  bool linkUp = crsf.isLinkUp();
  const crsfLinkStatistics_t *linkStats = crsf.getLinkStatistics();
  uint32_t lastByteAge = lastUartByteMs == 0 ? 0xFFFFFFFF : millis() - lastUartByteMs;

  Serial.print("RC,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(linkUp ? 1 : 0);
  Serial.print(",");
  Serial.print(uartBytesSeen);
  Serial.print(",");
  Serial.print(lastByteAge);
  Serial.print(",");
  Serial.print(linkStats->uplink_Link_quality);
  Serial.print(",");
  Serial.print(linkStats->uplink_RSSI_1);
  Serial.print(",");
  Serial.print(linkStats->uplink_RSSI_2);
  Serial.print(",");
  Serial.print(linkStats->uplink_SNR);

  if (linkUp != previousLinkUp) {
    previousLinkUp = linkUp;
  }

  for (uint8_t channel = 1; channel <= CRSF_NUM_CHANNELS; channel++) {
    Serial.print(",");
    Serial.print(crsf.getChannel(channel));
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 3000) {
    delay(10);
  }

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
  if (!crsfSerial) {
    while (true) {
      Serial.println("Invalid CRSF serial pin configuration");
      delay(1000);
    }
  }

  crsf.begin(crsfSerial);
  printStartupInfo();
  printRcHeader();
}

void loop() {
  int waitingBytes = crsfSerial.available();
  if (waitingBytes > 0) {
    uartBytesSeen += waitingBytes;
    lastUartByteMs = millis();
  }

  crsf.update();

  uint32_t now = millis();
  if (now - lastPrintMs >= PRINT_PERIOD_MS) {
    printRcFrame();
    lastPrintMs = now;
  }

  delay(1);
}
