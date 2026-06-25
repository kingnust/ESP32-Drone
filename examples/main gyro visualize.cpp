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
#define PMW3901_CS 40

#define GRAVITY_MSS 9.80665f
#define DEGREES_TO_RADIANS 0.01745329252f
#define RADIANS_TO_DEGREES 57.2957795131f
#define ORIENTATION_PRINT_PERIOD_MS 50
#define GYRO_CALIBRATION_SAMPLES 500
#define GYRO_CALIBRATION_DELAY_MS 2

#define ACCEL_X_SIGN 1.0f
#define ACCEL_Y_SIGN 1.0f
#define ACCEL_Z_SIGN -1.0f
#define GYRO_X_SIGN 1.0f
#define GYRO_Y_SIGN 1.0f
#define GYRO_Z_SIGN 1.0f
#define MAG_X_SIGN 1.0f
#define MAG_Y_SIGN 1.0f
#define MAG_Z_SIGN 1.0f

Bmi088Accel bmiAccel(SPI, BMI088_ACCEL_CS);
Bmi088Gyro bmiGyro(SPI, BMI088_GYRO_CS);
DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_1);

ReefwingAHRS ahrs;
SensorData fusionData;

bool ahrsReady = false;
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
uint32_t lastPrintMs = 0;
uint32_t updateCount = 0;

void printOrientationHeader() {
  Serial.println("millis,qx,qy,qz,qw,roll_deg,pitch_deg,yaw_deg,heading_deg,updates");
}

void multiplyQuaternion(float ax, float ay, float az, float aw,
                        float bx, float by, float bz, float bw,
                        float &qx, float &qy, float &qz, float &qw) {
  qx = aw * bx + ax * bw + ay * bz - az * by;
  qy = aw * by - ax * bz + ay * bw + az * bx;
  qz = aw * bz + ax * by - ay * bx + az * bw;
  qw = aw * bw - ax * bx - ay * by - az * bz;
}

void normalizeQuaternion(float &qx, float &qy, float &qz, float &qw) {
  float norm = sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm <= 0.0f) {
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;
    qw = 1.0f;
    return;
  }

  qx /= norm;
  qy /= norm;
  qz /= norm;
  qw /= norm;
}

void eulerToVisualizerQuaternion(float rollDeg, float pitchDeg, float yawDeg,
                                 float &qx, float &qy, float &qz, float &qw) {
  float roll = -rollDeg * DEGREES_TO_RADIANS;
  float pitch = pitchDeg * DEGREES_TO_RADIANS;
  float yaw = -yawDeg * DEGREES_TO_RADIANS;

  float qYawX = 0.0f;
  float qYawY = sin(yaw * 0.5f);
  float qYawZ = 0.0f;
  float qYawW = cos(yaw * 0.5f);

  float qPitchX = sin(pitch * 0.5f);
  float qPitchY = 0.0f;
  float qPitchZ = 0.0f;
  float qPitchW = cos(pitch * 0.5f);

  float qRollX = 0.0f;
  float qRollY = 0.0f;
  float qRollZ = sin(roll * 0.5f);
  float qRollW = cos(roll * 0.5f);

  float qTempX, qTempY, qTempZ, qTempW;
  multiplyQuaternion(qYawX, qYawY, qYawZ, qYawW,
                     qPitchX, qPitchY, qPitchZ, qPitchW,
                     qTempX, qTempY, qTempZ, qTempW);
  multiplyQuaternion(qTempX, qTempY, qTempZ, qTempW,
                     qRollX, qRollY, qRollZ, qRollW,
                     qx, qy, qz, qw);
  normalizeQuaternion(qx, qy, qz, qw);
}

void printOrientation() {
  float qx, qy, qz, qw;
  eulerToVisualizerQuaternion(ahrs.angles.roll, ahrs.angles.pitch, ahrs.angles.yaw,
                              qx, qy, qz, qw);

  Serial.print(millis());
  Serial.print(",");
  Serial.print(qx, 5);
  Serial.print(",");
  Serial.print(qy, 5);
  Serial.print(",");
  Serial.print(qz, 5);
  Serial.print(",");
  Serial.print(qw, 5);
  Serial.print(",");
  Serial.print(ahrs.angles.roll, 2);
  Serial.print(",");
  Serial.print(ahrs.angles.pitch, 2);
  Serial.print(",");
  Serial.print(ahrs.angles.yaw, 2);
  Serial.print(",");
  Serial.print(ahrs.angles.heading, 2);
  Serial.print(",");
  Serial.println(updateCount);
}

bool setupBmi088() {
  pinMode(PMW3901_CS, OUTPUT);
  digitalWrite(PMW3901_CS, HIGH);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  int accelStatus = bmiAccel.begin();
  if (accelStatus < 0) {
    Serial.print("AHRS init failed: BMI088 accel code ");
    Serial.println(accelStatus);
    return false;
  }
  bmiAccel.setRange(Bmi088Accel::RANGE_3G);
  bmiAccel.setOdr(Bmi088Accel::ODR_200HZ_BW_20HZ);

  int gyroStatus = bmiGyro.begin();
  if (gyroStatus < 0) {
    Serial.print("AHRS init failed: BMI088 gyro code ");
    Serial.println(gyroStatus);
    return false;
  }
  bmiGyro.setRange(Bmi088Gyro::RANGE_250DPS);
  bmiGyro.setOdr(Bmi088Gyro::ODR_200HZ_BW_23HZ);

  return true;
}

void calibrateGyroBias() {
  float gxSum = 0.0f;
  float gySum = 0.0f;
  float gzSum = 0.0f;

  Serial.println("AHRS gyro calibration: keep the drone still");

  for (uint16_t i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    bmiGyro.readSensor();
    gxSum += bmiGyro.getGyroX_rads() * RADIANS_TO_DEGREES;
    gySum += bmiGyro.getGyroY_rads() * RADIANS_TO_DEGREES;
    gzSum += bmiGyro.getGyroZ_rads() * RADIANS_TO_DEGREES;
    delay(GYRO_CALIBRATION_DELAY_MS);
  }

  gyroBiasX = gxSum / (float)GYRO_CALIBRATION_SAMPLES;
  gyroBiasY = gySum / (float)GYRO_CALIBRATION_SAMPLES;
  gyroBiasZ = gzSum / (float)GYRO_CALIBRATION_SAMPLES;

  Serial.println("AHRS gyro calibration done");
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
  bmm150.setRate(BMM150_DATA_RATE_30HZ);
  bmm150.setMeasurementXYZ();

  return true;
}

void readFusionData() {
  bmiAccel.readSensor();
  bmiGyro.readSensor();
  sBmm150MagData_t magData = bmm150.getGeomagneticData();

  fusionData.ax = ACCEL_X_SIGN * bmiAccel.getAccelX_mss() / GRAVITY_MSS;
  fusionData.ay = ACCEL_Y_SIGN * bmiAccel.getAccelY_mss() / GRAVITY_MSS;
  fusionData.az = ACCEL_Z_SIGN * bmiAccel.getAccelZ_mss() / GRAVITY_MSS;

  fusionData.gx = GYRO_X_SIGN * (bmiGyro.getGyroX_rads() * RADIANS_TO_DEGREES - gyroBiasX);
  fusionData.gy = GYRO_Y_SIGN * (bmiGyro.getGyroY_rads() * RADIANS_TO_DEGREES - gyroBiasY);
  fusionData.gz = GYRO_Z_SIGN * (bmiGyro.getGyroZ_rads() * RADIANS_TO_DEGREES - gyroBiasZ);

  fusionData.mx = MAG_X_SIGN * (float)magData.x;
  fusionData.my = MAG_Y_SIGN * (float)magData.y;
  fusionData.mz = MAG_Z_SIGN * (float)magData.z;

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
  ahrs.setFusionAlgorithm(SensorFusion::EXTENDED_KALMAN);
  ahrs.setDeclination(0.0f);

  bool bmiReady = setupBmi088();
  bool magReady = setupBmm150();
  ahrsReady = bmiReady && magReady;

  if (ahrsReady) {
    calibrateGyroBias();
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
