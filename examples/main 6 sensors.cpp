#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_TCS34725.h"
#include "BMI088.h"
#include "Bitcraze_PMW3901.h"
#include "DFRobot_BMM150.h"
#include <VL53L1X.h>

#define I2C_SDA 17
#define I2C_SCL 16

#define VL53_I2C_SDA 42
#define VL53_I2C_SCL 41

#define SPI_SCK 9
#define SPI_MISO 10
#define SPI_MOSI 11

#define BMI088_ACCEL_CS 7
#define BMI088_GYRO_CS 8
#define PMW3901_CS 40

#define TCS_LED 19

#define BMP3XX_I2C_ADDRESS 0x77
#define BMP3XX_I2C_ADDRESS_ALT 0x76
#define SEALEVELPRESSURE_HPA 1013.25

Adafruit_BMP3XX bmp388;
Adafruit_TCS34725 tcs34725 = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_614MS,
  TCS34725_GAIN_1X
);

Bmi088Accel bmiAccel(SPI, BMI088_ACCEL_CS);
Bmi088Gyro bmiGyro(SPI, BMI088_GYRO_CS);
Bitcraze_PMW3901 pmw3901(PMW3901_CS);
DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_1);
VL53L1X vl53l1x;

bool bmp388Ok = false;
bool tcs34725Ok = false;
bool bmiAccelOk = false;
bool bmiGyroOk = false;
bool pmw3901Ok = false;
bool bmm150Ok = false;
bool vl53l1xOk = false;

uint32_t frameNumber = 0;

float headingDegreesFromMag(const sBmm150MagData_t &magData) {
  float heading = atan2((float)magData.x, (float)magData.y);

  if (heading < 0) {
    heading += 2.0f * PI;
  }
  if (heading > 2.0f * PI) {
    heading -= 2.0f * PI;
  }

  return heading * 180.0f / PI;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Combined sensor monitor starting");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Wire1.begin(VL53_I2C_SDA, VL53_I2C_SCL);
  Wire1.setClock(400000);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  pinMode(TCS_LED, OUTPUT);
  digitalWrite(TCS_LED, HIGH);

  Serial.print("I2C bus 0 SDA/SCL: ");
  Serial.print(I2C_SDA);
  Serial.print("/");
  Serial.println(I2C_SCL);

  Serial.print("I2C bus 1 SDA/SCL: ");
  Serial.print(VL53_I2C_SDA);
  Serial.print("/");
  Serial.println(VL53_I2C_SCL);

  Serial.print("SPI SCK/MISO/MOSI: ");
  Serial.print(SPI_SCK);
  Serial.print("/");
  Serial.print(SPI_MISO);
  Serial.print("/");
  Serial.println(SPI_MOSI);

  bmp388Ok = bmp388.begin_I2C(BMP3XX_I2C_ADDRESS, &Wire) ||
             bmp388.begin_I2C(BMP3XX_I2C_ADDRESS_ALT, &Wire);
  if (bmp388Ok) {
    bmp388.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp388.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp388.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp388.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("BMP388/BMP390: OK");
  } else {
    Serial.println("BMP388/BMP390: not found");
  }

  tcs34725Ok = tcs34725.begin(TCS34725_ADDRESS, &Wire);
  Serial.println(tcs34725Ok ? "TCS34725: OK" : "TCS34725: not found");

  int bmiAccelStatus = bmiAccel.begin();
  bmiAccelOk = bmiAccelStatus >= 0;
  if (bmiAccelOk) {
    Serial.println("BMI088 accel: OK");
  } else {
    Serial.print("BMI088 accel: failed, code ");
    Serial.println(bmiAccelStatus);
  }

  int bmiGyroStatus = bmiGyro.begin();
  bmiGyroOk = bmiGyroStatus >= 0;
  if (bmiGyroOk) {
    Serial.println("BMI088 gyro: OK");
  } else {
    Serial.print("BMI088 gyro: failed, code ");
    Serial.println(bmiGyroStatus);
  }

  pmw3901Ok = pmw3901.begin();
  Serial.println(pmw3901Ok ? "PMW3901: OK" : "PMW3901: not found");

  uint8_t bmm150Status = bmm150.begin();
  bmm150Ok = bmm150Status == 0;
  if (bmm150Ok) {
    bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
    bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
    bmm150.setRate(BMM150_DATA_RATE_10HZ);
    bmm150.setMeasurementXYZ();
    Serial.println("BMM150: OK");
  } else {
    Serial.print("BMM150: failed, code ");
    Serial.println(bmm150Status);
  }

  vl53l1x.setBus(&Wire1);
  vl53l1x.setTimeout(500);
  vl53l1xOk = vl53l1x.init();
  if (vl53l1xOk) {
    vl53l1x.setDistanceMode(VL53L1X::Long);
    vl53l1x.setMeasurementTimingBudget(50000);
    vl53l1x.startContinuous(50);
    Serial.println("VL53L1X: OK");
  } else {
    Serial.println("VL53L1X: not found");
  }

  Serial.println("Setup complete");
}

void loop() {
  Serial.println();
  Serial.print("Frame ");
  Serial.print(frameNumber++);
  Serial.print(" @ ");
  Serial.print(millis());
  Serial.println(" ms");

  if (bmp388Ok) {
    if (bmp388.performReading()) {
      Serial.print("BMP388 temp_C=");
      Serial.print(bmp388.temperature);
      Serial.print(" pressure_hPa=");
      Serial.print(bmp388.pressure / 100.0);
      Serial.print(" altitude_m=");
      Serial.println(bmp388.readAltitude(SEALEVELPRESSURE_HPA));
    } else {
      Serial.println("BMP388 read failed");
    }
  } else {
    Serial.println("BMP388 not initialized");
  }

  if (tcs34725Ok) {
    uint16_t r, g, b, c, colorTemp, lux;
    tcs34725.getRawData(&r, &g, &b, &c);
    colorTemp = tcs34725.calculateColorTemperature_dn40(r, g, b, c);
    lux = tcs34725.calculateLux(r, g, b);

    Serial.print("TCS34725 colorTemp_K=");
    Serial.print(colorTemp);
    Serial.print(" lux=");
    Serial.print(lux);
    Serial.print(" raw R/G/B/C=");
    Serial.print(r);
    Serial.print("/");
    Serial.print(g);
    Serial.print("/");
    Serial.print(b);
    Serial.print("/");
    Serial.println(c);
  } else {
    Serial.println("TCS34725 not initialized");
  }

  if (bmiAccelOk || bmiGyroOk) {
    if (bmiAccelOk) {
      bmiAccel.readSensor();
      Serial.print("BMI088 accel_mss X/Y/Z=");
      Serial.print(bmiAccel.getAccelX_mss());
      Serial.print("/");
      Serial.print(bmiAccel.getAccelY_mss());
      Serial.print("/");
      Serial.print(bmiAccel.getAccelZ_mss());
      Serial.print(" temp_C=");
      Serial.println(bmiAccel.getTemperature_C());
    } else {
      Serial.println("BMI088 accel not initialized");
    }

    if (bmiGyroOk) {
      bmiGyro.readSensor();
      Serial.print("BMI088 gyro_rads X/Y/Z=");
      Serial.print(bmiGyro.getGyroX_rads());
      Serial.print("/");
      Serial.print(bmiGyro.getGyroY_rads());
      Serial.print("/");
      Serial.println(bmiGyro.getGyroZ_rads());
    } else {
      Serial.println("BMI088 gyro not initialized");
    }
  } else {
    Serial.println("BMI088 not initialized");
  }

  if (pmw3901Ok) {
    int16_t deltaX, deltaY;
    pmw3901.readMotionCount(&deltaX, &deltaY);
    Serial.print("PMW3901 delta X/Y=");
    Serial.print(deltaX);
    Serial.print("/");
    Serial.println(deltaY);
  } else {
    Serial.println("PMW3901 not initialized");
  }

  if (bmm150Ok) {
    sBmm150MagData_t magData = bmm150.getGeomagneticData();
    Serial.print("BMM150 mag_uT X/Y/Z=");
    Serial.print(magData.x);
    Serial.print("/");
    Serial.print(magData.y);
    Serial.print("/");
    Serial.print(magData.z);
    Serial.print(" heading_deg=");
    Serial.println(headingDegreesFromMag(magData));
  } else {
    Serial.println("BMM150 not initialized");
  }

  if (vl53l1xOk) {
    uint16_t range = vl53l1x.read();
    Serial.print("VL53L1X range_mm=");
    Serial.print(range);
    Serial.print(" status=");
    Serial.print(VL53L1X::rangeStatusToString(vl53l1x.ranging_data.range_status));
    Serial.print(" peak_MCPS=");
    Serial.print(vl53l1x.ranging_data.peak_signal_count_rate_MCPS);
    Serial.print(" ambient_MCPS=");
    Serial.print(vl53l1x.ranging_data.ambient_count_rate_MCPS);

    if (vl53l1x.timeoutOccurred()) {
      Serial.print(" timeout");
    }

    Serial.println();
  } else {
    Serial.println("VL53L1X not initialized");
  }

  delay(2000);
}
