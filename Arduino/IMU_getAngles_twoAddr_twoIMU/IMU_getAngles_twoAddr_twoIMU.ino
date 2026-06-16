#include <Wire.h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR1 0x69 //default I2C address, if solder jumper bridged: 0x68
ICM20948_WE myIMU1 = ICM20948_WE(ICM20948_ADDR1); //create an ICM20948 object

#define ICM20948_ADDR2 0x68 //default I2C address, if solder jumper bridged: 0x68
ICM20948_WE myIMU2 = ICM20948_WE(ICM20948_ADDR2); //create an ICM20948 object

int ESP_version =  2; //specify the version of the ESP here (1 or 2)

// I2C pins
#define I2C_SDA 23
#define I2C_SCL 22

void setup() {
  
  if (ESP_version==1) {
    Wire.begin(I2C_SDA, I2C_SCL);//pins need to be specified explicitly for the setup with V1
    } 
  else {Wire.begin();}
  
  Serial.begin(115200);
  while(!Serial) {}
  
  if(!myIMU1.init()){
    Serial.println("ICM20948-1 does not respond");
  }
  else{
    Serial.println("ICM20948-1 is connected");
  }

  if(!myIMU2.init()){
    Serial.println("ICM20948-2 does not respond");
  }
  else{
    Serial.println("ICM20948-2 is connected");
  }

  myIMU1.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU1.setAccDLPF(ICM20948_DLPF_6);    
  myIMU1.setAccSampleRateDivider(10);
  myIMU1.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU1.setGyrDLPF(ICM20948_DLPF_6);  
  myIMU1.setGyrSampleRateDivider(10);

  myIMU2.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU2.setAccDLPF(ICM20948_DLPF_6);    
  myIMU2.setAccSampleRateDivider(10);
  myIMU2.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU2.setGyrDLPF(ICM20948_DLPF_6);  
  myIMU2.setGyrSampleRateDivider(10);
}

void loop() {
  myIMU1.readSensor();
  xyzFloat angles1 = myIMU1.getAngles();

  myIMU2.readSensor();
  xyzFloat angles2 = myIMU2.getAngles();

  Serial.print("x1:");
  Serial.print(angles1.x);
  Serial.print(",");
  Serial.print("y1:");
  Serial.print(angles1.y);
  Serial.print(",");
  Serial.print("z1:");
  Serial.print(angles1.z);
  Serial.print(",");
  Serial.print("x2:");
  Serial.print(angles2.x);
  Serial.print(",");
  Serial.print("y2:");
  Serial.print(angles2.y);
  Serial.print(",");
  Serial.print("z2:");
  Serial.println(angles2.z);

  delay(100);
}
