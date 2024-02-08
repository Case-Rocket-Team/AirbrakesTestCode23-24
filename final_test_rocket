//include necessary libraries
#include <Wire.h>
#include <ICM20948_WE.h>
#include <Adafruit_DPS310.h>
#include <Encoder.h>
#include <LittleFS.h>

//I2C addresses for barometer and IMU
#define DPS310_ADDR 0x77
#define ICM20948_ADDR 0x69

#define FILE_SIZE_512K 524288L
#define FILE_SIZE_1M   1048576L
#define PROG_FLASH_SIZE 1024 * 1024 * 1

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
#define limitSwitchPin 10


//OBSOLETE WITH NEW ENCODER LIBRARY
//create variables to help with PID controller
//volatile long encoderCount = 0; 
//long previousTime = 0;
//float ePrevious = 0;
//float eIntegral = 0;
//const int target = 1000;

float seaLevelPressure = 0; //current pressure at sea level, to be defined by user

float altitudes[] = {0, 0, 0, 0, 0}; //stores recent altitudes
float accelerations[] = {0, 0, 0, 0, 0}; //stores recent acceleration values

boolean isCalibrated = false; //boolean flag to prevent action before calibration completes




void setup() {
  // put your setup code here, to run once:

  Wire.begin();
  Serial.begin(115200);
  if(!myIMU.init()){
    Serial.println("ICM20948 does not respond");
  }
  else{
    Serial.println("ICM20948 is connected");
  }

  

}

void loop() { 

  //main menu
  
  int option = 0;
  Serial.println("Please enter the number that corresponds with the menu option you would like to choose.\n 1. Calibrate \n 2. Test Motor \n 3. Test Sensors \n 4. Pad-Idle Mode");
  
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
          Serial.println("Sensor and motor calibration not completed. Please calibrate and try again.");
          option = 0;
        }
        break;
        
      case 3:
        if(isCalibrated){
          sensorTest();
        }
        else{
          Serial.println("Sensor and motor calibration not completed. Please calibrate and try again.");
          option = 0;
        }
        break;
        
      case 4:
        if(isCalibrated){
          padIdle();
        }
        else{
          Serial.println("Sensor and motor calibration not completed. Please calibrate and try again.");
          option = 0;
        }
        break;
        
      default:
        Serial.println("Invalid Input. Please try again.");
        option = 0;
      
    }
  } 
}


//calibrates sensors and motor before use
void calibrate(){
  Serial.println("Calibrating IMU...");
  delay(1000);
  myIMU.autoOffsets();
  myIMU.setAccRange(ICM20948_ACC_RANGE_16G);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setAccSampleRateDivider(10);
  Serial.println("IMU Calibrated");

  Serial.println("Please enter the current pressure at sea level in hPa.");
  while(seaLevelPressure == 0){
    seaLevelPressure = Serial.parseFloat();
    if(seaLevelPressure == 0){
      Serial.println("Invalid Input. Please enter a floating point number greater than zero.");
    }
  }
  

  
  //TODO: Motor calibration??? DPS Calibration???
 
  isCalibrated = true; 
}


//allows user to extend airbrakes by any angle 
void motorTest(){
  int option = 0;
  Serial.println("Please enter the number that corresponds with the menu option you would like to choose. \n 1. User Input Mode \n 2. One Cycle \n 3. Full Deploy \n 4. Retract \n 5.Return to Main Menu");

  while(option == 0){
    option = Serial.parseInt();
    switch(option){
      
      case 1: {
        Serial.println("Please enter the angle you wish to deploy to.");
        float deployAngle = 0.0;
        while(deployAngle == 0){
          deployAngle = Serial.parseFloat();
          if(deployAngle <= 0 || deployAngle > 56){ //TODO: Replace 56 with full extend angle
            Serial.println("Invalid Input. Please enter a floating point number greater than zero and less than or equal to ('INSERT TRUE MAX ANGLE HERE).");
          }
        }

        extendAirbrakes(calculateTargetCount(deployAngle));
        break;
      }
        
      case 2:{
        retractAirbrakes();
        extendAirbrakes(1000); //TODO: Replace 1000 with full extend count
        delay(5000);
        retractAirbrakes();
        option = 0;
        break;
      }
        
      case 3:{
        retractAirbrakes();
        extendAirbrakes(1000); //TODO: Replace 1000 with full extend count
        option = 0;
        break;
      }
        
      case 4: {
        retractAirbrakes();
        option = 0;
        break;
      }
        
      case 5: {
        break;
      }
      default:
        Serial.println("Invalid Input. Please try again.");
        option = 0;
      
    }
  }
  
  
}


//allows user to receive sensor data
void sensorTest(){
  Serial.println("CURRENT DATA:");
  Serial.println("BAROMETER (DPS310) DATA:");
  sensors_event_t temp_event, pressure_event;
  if (dps.temperatureAvailable()) {
    dps_temp->getEvent(&temp_event);
    Serial.print(F("Temperature = "));
    Serial.print(temp_event.temperature);
    Serial.println(" *C");
    Serial.println();
  }

  if (dps.pressureAvailable()) {
    dps_pressure->getEvent(&pressure_event);
    Serial.print(F("Pressure = "));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa");
    Serial.println();
  }

  Serial.print("Altitude: ");
  Serial.print(dps.readAltitude(seaLevelPressure));
  Serial.println();
  
  

  Serial.println("---------------");
  Serial.println("IMU (ICM20948) DATA:");

  myIMU.readSensor();
  xyzFloat accRaw = myIMU.getAccRawValues();
  xyzFloat corrAccRaw = myIMU.getCorrectedAccRawValues();
  xyzFloat gVal = myIMU.getGValues();
  float resultantG = myIMU.getResultantG(gVal);
  
  Serial.println("Raw acceleration values (x,y,z):");
  Serial.print(accRaw.x);
  Serial.print("   ");
  Serial.print(accRaw.y);
  Serial.print("   ");
  Serial.println(accRaw.z);

  Serial.println("Corrected raw acceleration values (x,y,z):");
  Serial.print(corrAccRaw.x);
  Serial.print("   ");
  Serial.print(corrAccRaw.y);
  Serial.print("   ");
  Serial.println(corrAccRaw.z);

  Serial.println("g-values (x,y,z):");
  Serial.print(gVal.x);
  Serial.print("   ");
  Serial.print(gVal.y);
  Serial.print("   ");
  Serial.println(gVal.z);

  Serial.print("Resultant g: ");
  Serial.println(resultantG);

  Serial.println("Magnetic field data (x,y,z)");
  
}


//launch ready state
void padIdle(){
  launchDetected = detectLaunch();
  while(!launchDetected){
    delay(10);
    launchDetected = detectLaunch();
  }
   
  }

  
}

//helper method to calculate motor count required to extend airbrakes by angle passed in
int calculateTargetCount(float angle){
  return 0;
}


//helper method to extend airbrakes
void extendAirbrakes(int target){
  
}


//fully retracts airbrakes from any position
void retractAirbrakes(){
  
}

boolean detectLaunch(){
  boolean launch = false;
  
  while(!launch){
    myIMU.readSensor();
    xyzFloat accRaw = myIMU.getAccRawValues();
    xyzFloat corrAccRaw = myIMU.getCorrectedAccRawValues();
    xyzFloat gVal = myIMU.getGValues();
    float resultantG = myIMU.getResultantG(gVal);
    if(abs(resultantG) > 3.0){
      launch = true;
    }
    return launch;
    
  }
}
