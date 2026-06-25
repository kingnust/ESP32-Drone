#include <Arduino.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

// Wire ELRS receiver TX -> ESP32 RX, receiver RX -> ESP32 TX.
#define CRSF_RX_PIN 38
#define CRSF_TX_PIN 37

#define PRINT_PERIOD_MS 200

HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

uint32_t lastPrintMs = 0;
uint32_t uartBytesSeen = 0;
uint32_t lastUartByteMs = 0;
bool previousLinkUp = false;

int channelPercent(int channelUs) {
  return constrain(map(channelUs, 1000, 2000, -100, 100), -100, 100);
}

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

void printLinkStats() {
  const crsfLinkStatistics_t *linkStats = crsf.getLinkStatistics();

  Serial.print("Link quality: ");
  Serial.print(linkStats->uplink_Link_quality);
  Serial.print("%   RSSI1: -");
  Serial.print(linkStats->uplink_RSSI_1);
  Serial.print(" dBm   RSSI2: -");
  Serial.print(linkStats->uplink_RSSI_2);
  Serial.print(" dBm   SNR: ");
  Serial.print(linkStats->uplink_SNR);
  Serial.print(" dB   RF mode: ");
  Serial.print(linkStats->rf_Mode);
  Serial.print("   TX power index: ");
  Serial.println(linkStats->uplink_TX_Power);
}

void printChannels() {
  for (uint8_t channel = 1; channel <= CRSF_NUM_CHANNELS; channel++) {
    int channelUs = crsf.getChannel(channel);

    Serial.print("CH");
    if (channel < 10) {
      Serial.print("0");
    }
    Serial.print(channel);
    Serial.print(": ");
    Serial.print(channelUs);
    Serial.print(" us (");
    int percent = channelPercent(channelUs);
    if (percent > 0) {
      Serial.print("+");
    }
    Serial.print(percent);
    Serial.print("%)");

    if (channel % 4 == 0) {
      Serial.println();
    } else {
      Serial.print("   ");
    }
  }
}

void printReceiverState() {
  bool linkUp = crsf.isLinkUp();

  if (linkUp != previousLinkUp) {
    Serial.println();
    Serial.println(linkUp ? "ELRS link is UP" : "ELRS link is DOWN");
    previousLinkUp = linkUp;
  }

  Serial.println();
  Serial.print("Sample time: ");
  Serial.print(millis());
  Serial.println(" ms");
  Serial.print("CRSF link: ");
  Serial.println(linkUp ? "UP" : "DOWN");
  Serial.print("UART bytes seen: ");
  Serial.print(uartBytesSeen);
  Serial.print("   Last byte: ");
  if (lastUartByteMs == 0) {
    Serial.println("never");
  } else {
    Serial.print(millis() - lastUartByteMs);
    Serial.println(" ms ago");
  }

  if (!linkUp) {
    Serial.println("No valid RC channel packets yet.");
    Serial.println("Check binding, receiver power, common ground, and crossed TX/RX wiring.");
    Serial.println("----------------------------------------");
    return;
  }

  printLinkStats();
  printChannels();
  Serial.println("----------------------------------------");
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
    printReceiverState();
    lastPrintMs = now;
  }

  delay(1);
}
