#include <Adafruit_NeoPixel.h>

#define LED_PIN 48   // pin RGB LED (umumnya 48 di ESP32-S3)
#define NUMPIXELS 1

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixels.begin();
}

void loop() {
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // merah
  pixels.show();
  delay(1000);

  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // hijau
  pixels.show();
  delay(1000);

  // pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // biru
  // pixels.show();
  // delay(1000);
}