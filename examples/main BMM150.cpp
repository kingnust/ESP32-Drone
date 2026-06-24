#include "DFRobot_BMM150.h"

#define I2C_SDA 17
#define I2C_SCL 16

DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_1);

/*i2c Address select, that CS and SDO pin select 1 or 0 indicates the high or low respectively. There are 4 combinations: 
 *I2C_ADDRESS_1 0x10  (CS:0 SDO:0)
 *I2C_ADDRESS_2 0x11  (CS:0 SDO:1)
 *I2C_ADDRESS_3 0x12  (CS:1 SDO:0)
 *I2C_ADDRESS_4 0x13  (CS:1 SDO:1) default i2c address */

void setup() 
{
  Serial.begin(115200);
  delay(1000);
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("BMM150 test starting");
  Serial.print("SDA = ");
  Serial.println(I2C_SDA);
  Serial.print("SCL = ");
  Serial.println(I2C_SCL);

  uint8_t status = bmm150.begin();
  while(status != 0){
    Serial.print("BMM150 init failed, code ");
    Serial.println(status);
    Serial.println("Check wiring, power, I2C/SPI mode, and selected I2C address.");
    delay(1000);
    status = bmm150.begin();
  }

  Serial.println("BMM150 init success!");

  bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);//Normal mode

  bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);//High-accuracy mode

  bmm150.setRate(BMM150_DATA_RATE_10HZ);//10HZ

  bmm150.setMeasurementXYZ();//Default config of X-, Y-, and Z-axis
}

void loop()
{
  sBmm150MagData_t magData = bmm150.getGeomagneticData();
  Serial.print("mag x = "); Serial.print(magData.x); Serial.println(" uT");
  Serial.print("mag y = "); Serial.print(magData.y); Serial.println(" uT");
  Serial.print("mag z = "); Serial.print(magData.z); Serial.println(" uT");
  
  /* float type data*/
  //Serial.print("mag x = "); Serial.print(magData.xx); Serial.println(" uT");
  //Serial.print("mag y = "); Serial.print(magData.yy); Serial.println(" uT");
  //Serial.print("mag z = "); Serial.print(magData.zz); Serial.println(" uT");
  float compassDegree = bmm150.getCompassDegree();
  Serial.print("the angle between the pointing direction and north (counterclockwise) is:");
  Serial.println(compassDegree);
  Serial.println("--------------------------------");
  delay(100);
}
