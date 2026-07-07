#include <Wire.h>
#include <ICM20948_WE.h>
//#define PCAADDR1 0x70
#define PCAADDR2 0x71
#define ICM20948_ADDR1 0x69 //default I2C address, if solder jumper bridged: 0x68
#define ICM20948_ADDR2 0x68 //default I2C address, if solder jumper bridged: 0x68
ICM20948_WE myIMU0 = ICM20948_WE(ICM20948_ADDR2); //create an ICM20948 object - lumbar
ICM20948_WE myIMU1 = ICM20948_WE(ICM20948_ADDR1); //create an ICM20948 object - clavicular
//ICM20948_WE myIMU1 = ICM20948_WE(ICM20948_ADDR1); //create an ICM20948 object - clavicleR - mult1 - bus1
//ICM20948_WE myIMU2 = ICM20948_WE(ICM20948_ADDR1); //create an ICM20948 object - clavicleL - mult1 - bus2
ICM20948_WE myIMU3 = ICM20948_WE(ICM20948_ADDR1); //create an ICM20948 object - thighL - mult2 - bus1
ICM20948_WE myIMU4 = ICM20948_WE(ICM20948_ADDR1); //create an ICM20948 object - thighR - mult2 - bus2

int ESP_version =  2; //specify the version of the ESP here (1 or 2)

// I2C pins
#define I2C_SDA 23
#define I2C_SCL 22

void pcaselect(uint8_t i) {
  if (i==0) {
//    Wire.beginTransmission(PCAADDR1);
//    Wire.write(0);
//    Wire.endTransmission();
    
    Wire.beginTransmission(PCAADDR2);
    Wire.write(0);
    Wire.endTransmission();
  } else if ((i==1)||(i==2)) {
//    Wire.beginTransmission(PCAADDR2);
//    Wire.write(0);
//    Wire.endTransmission();
//    
//    Wire.beginTransmission(PCAADDR1);
//    Wire.write(1 << i);
//    Wire.endTransmission();
  } else if ((i==3)||(i==4)) {
//    Wire.beginTransmission(PCAADDR1);
//    Wire.write(0);
//    Wire.endTransmission();
    
    Wire.beginTransmission(PCAADDR2);
    i = i-2;
    Wire.write(1 << i);
    Wire.endTransmission();
  } else {
    return;
  }
}

void setup() {
  
  if (ESP_version==1) {
    Wire.begin(I2C_SDA, I2C_SCL);//pins need to be specified explicitly for the setup with V1
    } 
  else {Wire.begin();}
  
  Serial.begin(115200);
  while(!Serial) {}
  
  pcaselect(0);
  if(!myIMU0.init()){
    Serial.println("ICM20948-0 does not respond");
  }
  else{
    Serial.println("ICM20948-0 is connected");
  }
  
//  pcaselect(1);
  if(!myIMU1.init()){
    Serial.println("ICM20948-1 does not respond");
  }
  else{
    Serial.println("ICM20948-1 is connected");
  }
  
//  pcaselect(2);
//  if(!myIMU2.init()){
//    Serial.println("ICM20948-2 does not respond");
//  }
//  else{
//    Serial.println("ICM20948-2 is connected");
//  }

  pcaselect(3);
  if(!myIMU3.init()){
    Serial.println("ICM20948-3 does not respond");
  }
  else{
    Serial.println("ICM20948-3 is connected");
  }

  pcaselect(4);
  if(!myIMU4.init()){
    Serial.println("ICM20948-4 does not respond");
  }
  else{
    Serial.println("ICM20948-4 is connected");
  }

  pcaselect(0);
  myIMU0.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU0.setAccDLPF(ICM20948_DLPF_6);    
  myIMU0.setAccSampleRateDivider(10);
  myIMU0.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU0.setGyrDLPF(ICM20948_DLPF_6);  
  myIMU0.setGyrSampleRateDivider(10);
  
//  pcaselect(1);
  myIMU1.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU1.setAccDLPF(ICM20948_DLPF_6);    
  myIMU1.setAccSampleRateDivider(10);
  myIMU1.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU1.setGyrDLPF(ICM20948_DLPF_6);  
  myIMU1.setGyrSampleRateDivider(10);

//  pcaselect(2);
//  myIMU2.setAccRange(ICM20948_ACC_RANGE_2G);
//  myIMU2.setAccDLPF(ICM20948_DLPF_6);    
//  myIMU2.setAccSampleRateDivider(10);
//  myIMU2.setGyrRange(ICM20948_GYRO_RANGE_250);
//  myIMU2.setGyrDLPF(ICM20948_DLPF_6);  
//  myIMU2.setGyrSampleRateDivider(10);

  pcaselect(3);
  myIMU3.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU3.setAccDLPF(ICM20948_DLPF_6);    
  myIMU3.setAccSampleRateDivider(10);
  myIMU3.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU3.setGyrDLPF(ICM20948_DLPF_6);  
  myIMU3.setGyrSampleRateDivider(10);

  pcaselect(4);
  myIMU4.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU4.setAccDLPF(ICM20948_DLPF_6);    
  myIMU4.setAccSampleRateDivider(10);
  myIMU4.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU4.setGyrDLPF(ICM20948_DLPF_6);  
  myIMU4.setGyrSampleRateDivider(10);
}

void loop() {
  pcaselect(0);
  myIMU0.readSensor();
  xyzFloat aVal0;
  xyzFloat gVal0;
  myIMU0.getGValues(&aVal0);
  myIMU0.getGyrValues(&gVal0);
  
//  pcaselect(1);
  myIMU1.readSensor();
  xyzFloat aVal1;
  xyzFloat gVal1;
  myIMU1.getGValues(&aVal1);
  myIMU1.getGyrValues(&gVal1);


//  pcaselect(2);
//  myIMU2.readSensor();
//  xyzFloat aVal2 = myIMU2.getGValues();
//  xyzFloat gVal2 = myIMU2.getGyrValues();

  pcaselect(3);
  myIMU3.readSensor();
  xyzFloat aVal3;
  xyzFloat gVal3;
  myIMU3.getGValues(&aVal3);
  myIMU3.getGyrValues(&gVal3);
  pcaselect(4);
  myIMU4.readSensor();
  xyzFloat aVal4;
  xyzFloat gVal4;
  myIMU4.getGValues(&aVal4);
  myIMU4.getGyrValues(&gVal4);

  Serial.print("ax0:");
  Serial.print(aVal0.x);
  Serial.print(",");
  Serial.print("ay0:");
  Serial.print(aVal0.y);
  Serial.print(",");
  Serial.print("az0:");
  Serial.print(aVal0.z);
  Serial.print(",");
  Serial.print("gx0:");
  Serial.print(gVal0.x);
  Serial.print(",");
  Serial.print("gy0:");
  Serial.print(gVal0.y);
  Serial.print(",");
  Serial.print("gz0:");
  Serial.print(gVal0.z);
  Serial.print(",");

  Serial.print("ax1:");
  Serial.print(aVal1.x);
  Serial.print(",");
  Serial.print("ay1:");
  Serial.print(aVal1.y);
  Serial.print(",");
  Serial.print("az1:");
  Serial.print(aVal1.z);
  Serial.print(",");
  Serial.print("gx1:");
  Serial.print(gVal1.x);
  Serial.print(",");
  Serial.print("gy1:");
  Serial.print(gVal1.y);
  Serial.print(",");
  Serial.print("gz1:");
  Serial.print(gVal1.z);
  Serial.print(",");

//  Serial.print("ax2:");
//  Serial.print(aVal2.x);
//  Serial.print(",");
//  Serial.print("ay2:");
//  Serial.print(aVal2.y);
//  Serial.print(",");
//  Serial.print("az2:");
//  Serial.print(aVal2.z);
//  Serial.print(",");
//  Serial.print("gx2:");
//  Serial.print(gVal2.x);
//  Serial.print(",");
//  Serial.print("gy2:");
//  Serial.print(gVal2.y);
//  Serial.print(",");
//  Serial.print("gz2:");
//  Serial.print(gVal2.z);
//  Serial.print(",");

  Serial.print("ax3:");
  Serial.print(aVal3.x);
  Serial.print(",");
  Serial.print("ay3:");
  Serial.print(aVal3.y);
  Serial.print(",");
  Serial.print("az3:");
  Serial.print(aVal3.z);
  Serial.print(",");
  Serial.print("gx3:");
  Serial.print(gVal3.x);
  Serial.print(",");
  Serial.print("gy3:");
  Serial.print(gVal3.y);
  Serial.print(",");
  Serial.print("gz3:");
  Serial.print(gVal3.z);
  Serial.print(",");

  Serial.print("ax4:");
  Serial.print(aVal4.x);
  Serial.print(",");
  Serial.print("ay4:");
  Serial.print(aVal4.y);
  Serial.print(",");
  Serial.print("az4:");
  Serial.print(aVal4.z);
  Serial.print(",");
  Serial.print("gx4:");
  Serial.print(gVal4.x);
  Serial.print(",");
  Serial.print("gy4:");
  Serial.print(gVal4.y);
  Serial.print(",");
  Serial.print("gz4:");
  Serial.println(gVal4.z);

  delay(10);
}
