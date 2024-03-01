//include necessary libraries
#include <Wire.h>
#include <ICM20948_WE.h>
#include <Adafruit_DPS310.h>
#include <Encoder.h>
#include <SPIMemory.h>
#include <SD.h>

//I2C addresses for barometer and IMU
#define DPS310_ADDR 0x77
#define ICM20948_ADDR 0x69

//#define FILE_SIZE_512K 524288L
//#define FILE_SIZE_1M   1048576L
//#define PROG_FLASH_SIZE 1024 * 1024 * 1

//initialise the IMU
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

//initialise the barometer and its temp and pressure sensors
Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp = dps.getTemperatureSensor();
Adafruit_Sensor *dps_pressure = dps.getPressureSensor();


//define motor and encoder pins
#define DIR1 37
#define PWM1 36
#define encoderPinA 22
#define encoderPinB 23

//define limit switch
#define limitSwitchPin 9

Encoder myEnc(encoderPinA, encoderPinB);
//OBSOLETE WITH NEW ENCODER LIBRARY
//create variables to help with PID controller
//volatile long encoderCount = 0; 
//long previousTime = 0;
//float ePrevious = 0;
//float eIntegral = 0;
//const int target = 1000;

float seaLevelPressure = 0; //current pressure at sea level, to be defined by user

boolean isCalibrated = false; //boolean flag to prevent action before calibration completes

unsigned int startFlashAddr, curFlashAddr = 0;

SPIFlash flash;

struct dataList {
  uint16_t recordNumber;
  uint32_t timeStamp;
  float dpsTemperature;
  float imuTemperature;
  float pressure;
  float accelX;
  float accelY;
  float accelZ;
  float resAcceleration;
  float altitude;
  float magValueX;
  float magValueY;
  float magValueZ;
  float gyrX;
  float gyrY;
  float gyrZ;
  bool launch;
  bool burnout;
  bool extending;
  bool retracting;
  bool touchdown;
} oneRecord;

bool logToFlash(){
  bool res = flash.writeAnything(curFlashAddr, oneRecord);
  if(res) curFlashAddr += sizeof(oneRecord);
  return res;
}

// Dumps all data written to the flash chip over the lifetime of the program to 
// the SD card. IMPORTANT: This function is meant to be called only once, at or
// near the end of execution.
void dumpToSD(){
  File file = SD.open("data.csv");
  if(file) {
    unsigned int _addr = 0;
    while(_addr < curFlashAddr, file){
      structToCSV(_addr, file);
      _addr += sizeof(dataList);
    }
  }
}

void structToCSV(unsigned int _addr, File fileName){
  fileName.print(flash.readWord(_addr));
  _addr += 2;
  fileName.print(flash.readULong(_addr));
  _addr += 4;
  // loop over the 14 consecutive floats in the data struct
  for(int i = 0; i < 14; i++){
    fileName.print(flash.readFloat(_addr));
    _addr += 4;
  }
  for(int i = 0; i < 5; i++){
    bool data;
    flash.readAnything(_addr, data);
    fileName.print(data);
    _addr += 1;

  }
}

void setup() {
  // put your setup code here, to run once:

  Wire.begin();
  Serial.begin(115200);
  if(!myIMU.init()){
    Serial.println(F("ICM20948 does not respond"));
  }
  else{
    Serial.println(F("ICM20948 is connected"));
  }
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);

}

void loop() { 

  //main menu
  
  int option = 0;
  Serial.println(F("Please enter the number that corresponds with the menu option you would like to choose.\n 1. Calibrate \n 2. Test Motor \n 3. Test Sensors \n 4. Pad-Idle Mode"));
  
  while(option == 0){
    option = Serial.parseInt();
     switch(option){
      
      case 1:
        calibrate(); //required upon startup
        break;
        
      case 2:
        if(isCalibrated){
          motorTest();
        }
        else{
          Serial.println(F("Sensor and motor calibration not completed. Please calibrate and try again."));
          option = 0;
        }
        break;
        
      case 3:
        if(isCalibrated){
          sensorTest();
        }
        else{
          Serial.println(F("Sensor and motor calibration not completed. Please calibrate and try again."));
          option = 0;
        }
        break;
        
      case 4:
        if(isCalibrated){
          padIdle();
        }
        else{
          Serial.println(F("Sensor and motor calibration not completed. Please calibrate and try again."));
          option = 0;
        }
        break;
        
      default:
        Serial.println(F("Invalid Input. Please try again."));
        option = 0;
      
    }
  } 
}


//calibrates sensors and motor before use
void calibrate(){
  Serial.println(F("Calibrating IMU..."));
  delay(1000);
  myIMU.autoOffsets();
  myIMU.setAccRange(ICM20948_ACC_RANGE_16G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setAccSampleRateDivider(10);

  Serial.println(F("Please enter the current pressure at sea level in hPa."));
  while(seaLevelPressure == 0){
    seaLevelPressure = Serial.parseFloat();
    if(seaLevelPressure == 0){
      Serial.println(F("Invalid Input. Please enter a floating point number greater than zero."));
    }
  }
  //TODO: Motor calibration??? DPS Calibration???
 
  isCalibrated = true; 
}


//allows user to extend airbrakes by any angle 
void motorTest(){
  int option = 0;
  Serial.println(F("Please enter the number that corresponds with the menu option you would like to choose. \n 1. User Input Mode \n 2. One Cycle \n 3. Full Deploy \n 4. Retract \n 5.Return to Main Menu"));

  while(option == 0){
    option = Serial.parseInt();
    switch(option){
      
      case 1: {
        Serial.println(F("Please enter the angle you wish to deploy to."));
        float deployAngle = 0.0;
        while(deployAngle == 0){
          deployAngle = Serial.parseFloat();
          if(deployAngle <= 0 || deployAngle > 56){ //TODO: Replace 56 with full extend angle
            Serial.println(F("Invalid Input. Please enter a floating point number greater than zero and less than or equal to (INSERT TRUE MAX ANGLE HERE)."));
          }
        }

        extendAirbrakes(calculateTargetCount(deployAngle), 0);
        break;
      }
        
      case 2:{
        Serial.println(F("Retracting"));
        retractAirbrakes(0);
        Serial.println(F("Extending"));
        extendAirbrakes(100,0); //TODO: Replace 1000 with full extend count
        Serial.println(F("Delaying"));
        delay(500);
        Serial.println(F("Retracting"));
        retractAirbrakes(0);
        option = 0;
        break;
      }
        
      case 3:{
        retractAirbrakes(0);
        extendAirbrakes(100,0); //TODO: Replace 1000 with full extend count
        option = 0;
        break;
      }
        
      case 4: {
        retractAirbrakes(0);
        option = 0;
        break;
      }
        
      case 5: {
        break;
      }
      default:
        Serial.println(F("Invalid Input. Please try again."));
        option = 0;
      
    }
  }
  
  
}


//allows user to receive sensor data
void sensorTest(){
  Serial.println(F("CURRENT DATA:"));
  Serial.println(F("BAROMETER (DPS310) DATA:"));
  sensors_event_t temp_event, pressure_event;
  if (dps.temperatureAvailable()) {
    dps_temp->getEvent(&temp_event);
    Serial.print(F("Temperature = "));
    Serial.print(temp_event.temperature);
    Serial.println(F(" *C"));
    Serial.println();
  }

  if (dps.pressureAvailable()) {
    dps_pressure->getEvent(&pressure_event);
    Serial.print(F("Pressure = "));
    Serial.print(pressure_event.pressure);
    Serial.println(F(" hPa"));
    Serial.println();
  }

  Serial.print(F("Altitude: "));
  Serial.print(dps.readAltitude(seaLevelPressure));
  Serial.println();
  
  

  Serial.println(F("---------------"));
  Serial.println(F("IMU (ICM20948) DATA:"));

  myIMU.readSensor();
  xyzFloat accRaw = myIMU.getAccRawValues();
  xyzFloat corrAccRaw = myIMU.getCorrectedAccRawValues();
  xyzFloat gVal = myIMU.getGValues();
  float resultantG = myIMU.getResultantG(gVal);
  
  Serial.println(F("Raw acceleration values (x,y,z):"));
  Serial.print(accRaw.x);
  Serial.print(F("   "));
  Serial.print(accRaw.y);
  Serial.print(F("   "));
  Serial.println(accRaw.z);

  Serial.println(F("Corrected raw acceleration values (x,y,z):"));
  Serial.print(corrAccRaw.x);
  Serial.print(F("   "));
  Serial.print(corrAccRaw.y);
  Serial.print(F("   "));
  Serial.println(corrAccRaw.z);

  Serial.println(F("g-values (x,y,z):"));
  Serial.print(gVal.x);
  Serial.print(F("   "));
  Serial.print(gVal.y);
  Serial.print(F("   "));
  Serial.println(gVal.z);

  Serial.print(F("Resultant g: "));
  Serial.println(resultantG);

  Serial.println(F("Magnetic field data (x,y,z)"));
  
}


//launch ready state
void padIdle(){
  oneRecord.launch = detectLaunch();
  while(!oneRecord.launch){
    delay(10);
    oneRecord.launch = detectLaunch();
  }
  int timeStart = micros();
  int currentTime = micros();
  while(!oneRecord.burnout){
    //LOG DATA HERE: TO BE IMPLEMENTED
    currentTime = micros();
    recordData(timeStart, currentTime);
    logToFlash();
    if(currentTime-timeStart > 1000000){
      oneRecord.burnout = detectBurnout();
      if(currentTime-timeStart > 1800000){
        oneRecord.burnout = true;
      }
    }
  }

  delay(1000);
  oneRecord.extending = true;
  extendAirbrakes(400, timeStart);
  delay(1000);
  oneRecord.retracting = true;
  retractAirbrakes(timeStart);
  while(!oneRecord.touchdown){
    oneRecord.touchdown = detectTouchdown();
    currentTime = micros();
    recordData(timeStart, currentTime);
    logToFlash();
  }
  delay(10000);
  dumpToSD();
  int option = 0;
  Serial.println(F("PLEASE ENTER 1234567890 ONCE YOU HAVE EJECTED THE SD CARD AND COMPLETED OPERATIONS."));
  while(option == 0){
    option = Serial.parseInt();
    if(option == 1234567890){
      option = 1;
    }
    else{
      option = 0;
    }
  }
  
   
  }

  


//helper method to calculate motor count required to extend airbrakes by angle passed in
int calculateTargetCount(float angle){
  return 0; //NOT YET IMPLEMENTED, LOWER PRIORITY
}


//helper method to retract airbrakes
void retractAirbrakes(int startTime){
  
  while(digitalRead(limitSwitchPin)==LOW){
    Serial.println(myEnc.read());
    recordData(startTime, micros());
    logToFlash();
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, 255);
  }
  digitalWrite(DIR1, LOW);
  analogWrite(PWM1, 0);
  
}

//helper method to extend airbrakes
void extendAirbrakes(int target, int startTime){
  while(myEnc.read() < target){
    digitalWrite(PWM1, LOW);
    analogWrite(DIR1, 255);
    recordData(startTime, micros()); //TODO: Pass in startTime eventually
    logToFlash();
    Serial.println(myEnc.read());
  }
  digitalWrite(PWM1, LOW);
  analogWrite(DIR1, 0);
}

boolean detectLaunch(){
  boolean launch = false;
  myIMU.readSensor();
  if(abs(myIMU.getResultantG(myIMU.getGValues())) > 3.0){
    launch = true;
  }
  return launch;
    
}

boolean detectBurnout(){
  boolean burnout = false;
  myIMU.readSensor();
  if(abs(myIMU.getResultantG(myIMU.getGValues())) < 4){
    burnout = true;
    }
  return burnout;
}

boolean detectTouchdown(){
  int ctr = 0;
  boolean touchdown = false;
  if(oneRecord.resAcceleration >=0.7 && oneRecord.resAcceleration <= 1.3 ){
    while(ctr<10){
      if((myIMU.getResultantG(myIMU.getGValues())) >=0.7 && myIMU.getResultantG(myIMU.getGValues()) <= 1.3 ){
        ctr++;
      }
      else{
        return false;
      }
    }
    return true;
  }
  return false;
}

void recordData(int startTime, int currentTime){
  myIMU.readSensor();
  oneRecord.recordNumber++;
  oneRecord.timeStamp = currentTime-startTime;
  sensors_event_t temp_event, pressure_event;
  if (dps.temperatureAvailable()) {
    dps_temp->getEvent(&temp_event);
    oneRecord.dpsTemperature = temp_event.temperature;
  }
  if (dps.pressureAvailable()) {
    dps_pressure->getEvent(&pressure_event);
    oneRecord.pressure = pressure_event.pressure;
  }

  oneRecord.accelX = myIMU.getGValues().x;
  oneRecord.accelY = myIMU.getGValues().y;
  oneRecord.accelZ = myIMU.getGValues().z;
  oneRecord.resAcceleration = myIMU.getResultantG(myIMU.getGValues());
  oneRecord.imuTemperature = myIMU.getTemperature();
  oneRecord.magValueX = myIMU.getMagValues().x;
  oneRecord.magValueY = myIMU.getMagValues().y;
  oneRecord.magValueZ = myIMU.getMagValues().z;
  oneRecord.gyrX = myIMU.getGyrValues().x;
  oneRecord.gyrY = myIMU.getGyrValues().y;
  oneRecord.gyrZ = myIMU.getGyrValues().z;
}
