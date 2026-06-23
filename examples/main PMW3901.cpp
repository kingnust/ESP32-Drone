#include "Bitcraze_PMW3901.h"
#include <SPI.h>

#define SPI_SCK 9
#define SPI_MISO 10
#define SPI_MOSI 11
#define PMW3901_CS 40

Bitcraze_PMW3901 flow(PMW3901_CS);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("PMW3901 test starting");

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, PMW3901_CS);

  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    while(1) { }
  }
}

int16_t deltaX,deltaY;

void loop() {
  // Get motion count since last call
  flow.readMotionCount(&deltaX, &deltaY);

  Serial.print("X: ");
  Serial.print(deltaX);
  Serial.print(", Y: ");
  Serial.print(deltaY);
  Serial.print("\n");

  delay(100);
}
