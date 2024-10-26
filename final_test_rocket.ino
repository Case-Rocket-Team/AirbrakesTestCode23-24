// Basic demo for accelerometer readings from Adafruit ICM20948

#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_DPS310.h>
#include <Encoder.h>
#include <SerialFlash.h>
#include <SD.h>

Adafruit_ICM20948 myIMU;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp;
Adafruit_Sensor *dps_pressure;

#define DIR1 6
#define PWM1 5

#define limitSwitchOne 3
#define limitSwitchTwo 4
#define SDChipSelect 53
#define flashChipSelect 10
#define encoderPinA 7
#define encoderPinB 8
#define burnoutTime1 1000000
#define burnoutTime2 1800000
sensors_event_t pressure_event, accel, gyro, temp, mag;

Encoder myEnc(encoderPinA, encoderPinB);

float seaLevelPressure = 0;  //current pressure at sea level, to be defined by user

boolean isCalibrated = false;  //boolean flag to prevent action before calibration completes

unsigned long curAddr = 0;  //counters to track current flash addresses being written to

unsigned long recordNum = 0;

boolean deployAirbrakes = false;

SerialFlashFile sff;

File file;

struct dataList {
  unsigned long recordNumber;
  unsigned long timeStamp;
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
  double encoderCounts;
  bool launch;
  bool burnout;
  bool extending;
  bool retracting;
  bool touchdown;
} oneRecord;

bool logToFlash() {
  sff.write(&oneRecord, sizeof(oneRecord));
  sff.seek(sff.position() + sizeof(oneRecord));
  curAddr++;
  
  return true;
}

void dumpToSD(){
  Serial.println("Dumping to SD");
  file = SD.open("data.bin", FILE_WRITE);
  
  sff.seek(0);
  char buf[sizeof(oneRecord)];
  for(int i = 0; i < curAddr; i++){
    sff.read(buf, sizeof(oneRecord));
    file.write(buf, sizeof(oneRecord));
  }

  file.close();
}

void padIdle() {
  oneRecord.launch = detectLaunch();
  while (!oneRecord.launch) {
    delay(10);
    oneRecord.launch = detectLaunch();
  }
  Serial.println("Launched!");
  unsigned long timeStart = micros();
  unsigned long currentTime = micros();
  while (!oneRecord.burnout) {
    currentTime = micros();
    recordData(timeStart, currentTime);
    logToFlash();
    if (currentTime - timeStart > burnoutTime1) {
      
      oneRecord.burnout = detectBurnout();
      if (currentTime - timeStart > burnoutTime2) {
        oneRecord.burnout = true;
      }
    }
  }
  Serial.println("Burnout detected");
  

  delay(1000);
  if(deployAirbrakes){
    oneRecord.extending = true;
    extendAirbrakes(timeStart);
  }
  
  delay(1000);
  if(deployAirbrakes){
    oneRecord.retracting = true;
    retractAirbrakes(timeStart);
  }
  
  while (currentTime - timeStart < 300000000) {
    currentTime = micros();
    recordData(timeStart, currentTime);
    logToFlash();
  }

  Serial.println("Touchdown detected");
  delay(10000);
  dumpToSD();
  int option = 0;
  Serial.println(F("ENTER 1234567890 ONCE YOU HAVE EJECTED THE SD CARD."));
  while (option == 0) {
    option = Serial.parseInt();
    if (option == 1234567890) {
      option = 1;
    } else {
      option = 0;
    }
  }
}


// Helper method to calculate motor count required to extend airbrakes from angle parameter
int calculateTargetCount(float angle) {
  return 0;  //NOT YET IMPLEMENTED, LOWER PRIORITY
}


// Retracts the airbrakes until the limit switch is triggered while logging
// to flash.
void retractAirbrakes(int startTime) {
  Serial.println(digitalRead(limitSwitchOne));
  while(digitalRead(limitSwitchOne)==LOW){
  recordData(startTime, micros());
  logToFlash();
  digitalWrite(DIR1, LOW);
  analogWrite(PWM1, 255);
  }
  digitalWrite(DIR1, LOW);
  analogWrite(PWM1, 0);
}

// Extends the airbrakes until the inputted target value is reached by the
// encoder while logging to flash.
void extendAirbrakes(int startTime) {
  Serial.println(digitalRead(limitSwitchTwo));
  while(digitalRead(limitSwitchTwo) == LOW){
  digitalWrite(PWM1, LOW);
  analogWrite(DIR1, 255);
  recordData(startTime, micros());
  logToFlash();
  }
  digitalWrite(PWM1, LOW);
  analogWrite(DIR1, 0);
}

// Detects if a launch event has occurred based on accelerometer
// readings exceeding a threshold value of 3 Gs.
boolean detectLaunch() {
  myIMU.getEvent(&accel, &gyro, &temp);
  Serial.println(accel.acceleration.z);
  return abs(accel.acceleration.z) > 30;
}

// Detects if a burnout event has occurred based on accelerometer
// readings falling below a threshold value. of 4 Gs.
boolean detectBurnout() {
  myIMU.getEvent(&accel, &gyro, &temp);
  return abs(accel.acceleration.z) < 40;
}

// Updates the datalist struct with new values from sensors
void recordData(int startTime, int currentTime) {
  myIMU.getEvent(&accel, &gyro, &temp);
  oneRecord.recordNumber = recordNum++;
  oneRecord.timeStamp = currentTime - startTime;
  if (dps.temperatureAvailable()) {
    dps_temp->getEvent(&temp);
    oneRecord.dpsTemperature = temp.temperature;
  }
  if (dps.pressureAvailable()) {
    dps_pressure->getEvent(&pressure_event);
    oneRecord.pressure = pressure_event.pressure;
  }
  oneRecord.accelX = accel.acceleration.x;
  oneRecord.accelY = accel.acceleration.y;
  oneRecord.accelZ = accel.acceleration.z;
  oneRecord.resAcceleration = accel.acceleration.z;
  oneRecord.imuTemperature = temp.temperature;
  oneRecord.magValueX = 0;
  oneRecord.magValueY = 0;
  oneRecord.magValueZ = 0;
  oneRecord.gyrX = gyro.gyro.x;
  oneRecord.gyrY = gyro.gyro.y;
  oneRecord.gyrZ = gyro.gyro.z;
  oneRecord.encoderCounts = myEnc.read();
  Serial.println(oneRecord.encoderCounts);
}

void motorTest() {
  int option = 0;
  Serial.println(F("Enter: \n 1  for One Cycle \n 3 for Full Deploy \n 4 for Retract \n 5 for Main Menu"));

  while (option == 0) {
    option = Serial.parseInt();
    switch (option) {

      case 1:
        {
          Serial.println(F("Retracting"));
          retractAirbrakes(0);
          Serial.println(F("Extending"));
          extendAirbrakes(0);  //TODO: Replace 1000 with full extend count
          Serial.println(F("Delaying"));
          delay(500);
          Serial.println(F("Retracting"));
          retractAirbrakes(0);
          option = 0;
          break;
        }

      case 2:
        {
          //retractAirbrakes(0);
          extendAirbrakes(0);  //TODO: Replace 1000 with full extend count
          option = 0;
          break;
        }

      case 3:
        {
          retractAirbrakes(0);
          option = 0;
          break;
        }

      case 4:
        {
          break;
        }
      default:
        Serial.println(F("Invalid Input. Please try again."));
        option = 0;
    }
  }
}

void calibrate() {
  Serial.println(F("Calibrating IMU..."));
  delay(1000);
  myIMU.setAccelRange(ICM20948_ACCEL_RANGE_16_G);

  Serial.println(F("Please enter the current pressure in hPa."));
  while (seaLevelPressure == 0) {
    seaLevelPressure = Serial.parseFloat();
    if (seaLevelPressure == 0) {
      Serial.println(F("Invalid Input. Please enter a floating point number greater than zero."));
    }
  }
  //TODO: Motor calibration??? DPS Calibration???
  Serial.println("Should the airbrakes deploy? T or F");
  while(!deployAirbrakes){
    String selection = Serial.readString();
    if (selection == "F\n"){
      break;
    } else if (selection == "T\n") {
      deployAirbrakes = true;
    } else {
      Serial.println(F("Invalid Input. Please enter T or F"));
    }
  }
  
  isCalibrated = true;
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Try to initialize!
  if (!myIMU.begin_I2C(0x69)) {

    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");

  dps.begin_I2C();
  dps_temp = dps.getTemperatureSensor();
  dps_pressure = dps.getPressureSensor();

  if (!SerialFlash.begin(flashChipSelect)){
    Serial.println("Failed to init flash");
  }
  sff = SerialFlash.open("flash.bin");

  if(!SD.begin(53)){
    Serial.println("Failed to init SD");
  }
  
  
  myIMU.setAccelRange(ICM20948_ACCEL_RANGE_16_G);


  myIMU.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);


  myIMU.setAccelRateDivisor(4095);


  myIMU.setGyroRateDivisor(255);


  myIMU.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);

  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(limitSwitchOne, INPUT);
  pinMode(limitSwitchTwo, INPUT);
  digitalWrite(DIR1, LOW);
  analogWrite(PWM1, 0);
}

void loop() {

  //  /* Get a new normalized sensor event */
  myIMU.getEvent(&accel, &gyro, &temp, &mag);

  int option = 0;
  Serial.println(F("Please enter the number that corresponds with the menu option you would like to choose.\n 1. Calibrate \n 2. Test Motor \n 3. Test Sensors \n 4. Pad-Idle Mode"));

  while (option == 0) {
    option = Serial.parseInt();
    switch (option) {

      case 1:
        calibrate();  //required upon startup
        break;

      case 2:
        if (isCalibrated) {
          motorTest();
        } else {
          Serial.println(F("Sensor and motor calibration not completed. Please calibrate and try again."));
          option = 0;
        }
        break;

      case 3:
        if (isCalibrated) {
          //sensorTest();
        } else {
          Serial.println(F("Sensor and motor calibration not completed. Please calibrate and try again."));
          option = 0;
        }
        break;

      case 4:
        if (isCalibrated) {
          padIdle();
        } else {
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
