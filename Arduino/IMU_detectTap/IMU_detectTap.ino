#include <Wire.h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x69 //default I2C address, if solder jumper bridged: 0x68
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR); //create an ICM20948 object

int ESP_version =  2; //specify the version of the ESP here (1 or 2)

// I2C pins
#define I2C_SDA 23
#define I2C_SCL 22

// Tap detection function
unsigned long lastTapTime = 0;
const unsigned long debounceDelay = 250; // ms
const float tapThresholdSq = 2.10f;
bool detectTap(xyzFloat aVal) {
  float magSq = (aVal.x * aVal.x) + (aVal.y * aVal.y) + (aVal.z * aVal.z);

  if (magSq > tapThresholdSq && (millis() - lastTapTime) > debounceDelay) {
    lastTapTime = millis();
    return true;
  }
  return false;
}

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
  xyzFloat aVal = myIMU.getGValues();
  xyzFloat gVal = myIMU.getGyrValues();

//  Serial.print("ax:");
//  Serial.print(aVal.x);
//  Serial.print(",");
//  Serial.print("ay:");
//  Serial.print(aVal.y);
//  Serial.print(",");
//  Serial.print("az:");
//  Serial.print(aVal.z);
//  Serial.print(",");
//  Serial.print("gx:");
//  Serial.print(gVal.x);
//  Serial.print(",");
//  Serial.print("gy:");
//  Serial.print(gVal.y);
//  Serial.print(",");
//  Serial.print("gz:");
//  Serial.println(gVal.z);

  if (detectTap(aVal)) {
    Serial.println("\n========================");
    Serial.println("      TAP DETECTED!     ");
    Serial.println("========================\n");
  }
 
  delay(10);
}
