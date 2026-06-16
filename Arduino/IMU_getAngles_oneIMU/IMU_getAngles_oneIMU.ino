#include <Wire.h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x69 //default I2C address, if solder jumper bridged: 0x68
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR); //create an ICM20948 object

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
  
  if(!myIMU.init()){
    Serial.println("ICM20948 does not respond");
  }
  else{
    Serial.println("ICM20948 is connected");
  }

  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);    
  myIMU.setAccSampleRateDivider(10);
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);  
  myIMU.setGyrSampleRateDivider(10);
}

void loop() {
  myIMU.readSensor();
  xyzFloat angles = myIMU.getAngles();

  Serial.print("x:");
  Serial.print(angles.x);
  Serial.print(",");
  Serial.print("y:");
  Serial.print(angles.y);
  Serial.print(",");
  Serial.print("z:");
  Serial.println(angles.z);

  delay(100);
}
