#include <Wire.h>
#include <SPI.h>
#include <ReefwingAHRS.h>
#include "BMI088.h"
#include "DFRobot_BMM150.h"

#define I2C_SDA 17
#define I2C_SCL 16

#define SPI_SCK 9
#define SPI_MISO 10
#define SPI_MOSI 11

#define BMI088_ACCEL_CS 7
#define BMI088_GYRO_CS 8

#define GRAVITY_MSS 9.80665f
#define RADIANS_TO_DEGREES 57.2957795131f
#define ORIENTATION_PRINT_PERIOD_MS 100

Bmi088Accel bmiAccel(SPI, BMI088_ACCEL_CS);
Bmi088Gyro bmiGyro(SPI, BMI088_GYRO_CS);
DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_1);

ReefwingAHRS ahrs;
SensorData fusionData;

bool ahrsReady = false;
uint32_t lastPrintMs = 0;
uint32_t updateCount = 0;

void printOrientationHeader() {
  Serial.println("millis,roll_deg,pitch_deg,yaw_deg,heading_deg,q0,q1,q2,q3,updates");
}

void printOrientation() {
  Quaternion q = ahrs.getQuaternion();

  Serial.print(millis());
  Serial.print(",");
  Serial.print(ahrs.angles.roll, 2);
  Serial.print(",");
  Serial.print(ahrs.angles.pitch, 2);
  Serial.print(",");
  Serial.print(ahrs.angles.yaw, 2);
  Serial.print(",");
  Serial.print(ahrs.angles.heading, 2);
  Serial.print(",");
  Serial.print(q.q0, 5);
  Serial.print(",");
  Serial.print(q.q1, 5);
  Serial.print(",");
  Serial.print(q.q2, 5);
  Serial.print(",");
  Serial.print(q.q3, 5);
  Serial.print(",");
  Serial.println(updateCount);
}

bool setupBmi088() {
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  int accelStatus = bmiAccel.begin();
  if (accelStatus < 0) {
    Serial.print("AHRS init failed: BMI088 accel code ");
    Serial.println(accelStatus);
    return false;
  }

  int gyroStatus = bmiGyro.begin();
  if (gyroStatus < 0) {
    Serial.print("AHRS init failed: BMI088 gyro code ");
    Serial.println(gyroStatus);
    return false;
  }

  return true;
}

bool setupBmm150() {
  uint8_t magStatus = bmm150.begin();
  if (magStatus != 0) {
    Serial.print("AHRS init failed: BMM150 code ");
    Serial.println(magStatus);
    return false;
  }

  bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
  bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
  bmm150.setRate(BMM150_DATA_RATE_10HZ);
  bmm150.setMeasurementXYZ();

  return true;
}

void readFusionData() {
  bmiAccel.readSensor();
  bmiGyro.readSensor();
  sBmm150MagData_t magData = bmm150.getGeomagneticData();

  fusionData.ax = bmiAccel.getAccelX_mss() / GRAVITY_MSS;
  fusionData.ay = bmiAccel.getAccelY_mss() / GRAVITY_MSS;
  fusionData.az = bmiAccel.getAccelZ_mss() / GRAVITY_MSS;

  fusionData.gx = bmiGyro.getGyroX_rads() * RADIANS_TO_DEGREES;
  fusionData.gy = bmiGyro.getGyroY_rads() * RADIANS_TO_DEGREES;
  fusionData.gz = bmiGyro.getGyroZ_rads() * RADIANS_TO_DEGREES;

  fusionData.mx = (float)magData.x;
  fusionData.my = (float)magData.y;
  fusionData.mz = (float)magData.z;

  fusionData.aTimeStamp = millis();
  fusionData.gTimeStamp = fusionData.aTimeStamp;
  fusionData.mTimeStamp = fusionData.aTimeStamp;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  ahrs.begin();
  ahrs.setDOF(DOF::DOF_9);
  ahrs.setImuType(ImuType::UNKNOWN);
  ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
  ahrs.setDeclination(0.0f);

  bool bmiReady = setupBmi088();
  bool magReady = setupBmm150();
  ahrsReady = bmiReady && magReady;

  if (ahrsReady) {
    readFusionData();
    ahrs.setData(fusionData, false);
    ahrs.update();
    printOrientationHeader();
  }
}

void loop() {
  if (!ahrsReady) {
    delay(1000);
    return;
  }

  readFusionData();
  ahrs.setData(fusionData, false);
  ahrs.update();
  updateCount++;

  uint32_t now = millis();
  if (now - lastPrintMs >= ORIENTATION_PRINT_PERIOD_MS) {
    printOrientation();
    updateCount = 0;
    lastPrintMs = now;
  }

  delay(5);
}
