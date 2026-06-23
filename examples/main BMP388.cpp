#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define I2C_SDA 17
#define I2C_SCL 16

#define BMP3XX_I2C_ADDRESS 0x77
#define BMP3XX_I2C_ADDRESS_ALT 0x76

#define SEALEVELPRESSURE_HPA 1013.25

Adafruit_BMP3XX bmp;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Adafruit BMP388 / BMP390 test");

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!bmp.begin_I2C(BMP3XX_I2C_ADDRESS) &&
      !bmp.begin_I2C(BMP3XX_I2C_ADDRESS_ALT)) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (true) {
      delay(1000);
    }
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.println("BMP3XX sensor found.");
}

void loop() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading.");
    delay(2000);
    return;
  }

  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(50);
}
