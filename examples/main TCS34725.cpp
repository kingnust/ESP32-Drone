#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define I2C_SDA 17
#define I2C_SCL 16
#define TCS_LED 19

Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_614MS,
  TCS34725_GAIN_1X
);

void setup(void) {
  Serial.begin(115200);
  delay(1000);

  pinMode(TCS_LED, OUTPUT);
  digitalWrite(TCS_LED, HIGH);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (true) {
      delay(1000);
    }
  }
}

void loop(void) {
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  Serial.print("Color Temp: ");
  Serial.print(colorTemp, DEC);
  Serial.print(" K - ");
  Serial.print("Lux: ");
  Serial.print(lux, DEC);
  Serial.print(" - ");
  Serial.print("R: ");
  Serial.print(r, DEC);
  Serial.print(" ");
  Serial.print("G: ");
  Serial.print(g, DEC);
  Serial.print(" ");
  Serial.print("B: ");
  Serial.print(b, DEC);
  Serial.print(" ");
  Serial.print("C: ");
  Serial.print(c, DEC);
  Serial.println(" ");

  delay(500);
}
