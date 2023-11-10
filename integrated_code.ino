#include <SerialFlash.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <ICM20948_WE.h>


#define ICM20948_ADDR 0x69

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (37)

#define FILE_SIZE_512K 524288L
#define FILE_SIZE_1M   1048576L


Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
ICM20948_WE myIMU = ICM20948_WE(BMP_CS, true);

float altitudes[] = {0, 0, 0, 0, 0}; //stores recent altitudes
float accelerations[] = {0, 0, 0, 0, 0}; //stores recent acceleration values

SerialFlashFile file;

const uint32_t sampleInterval = 10;
const byte PIN_FLASH_CS = 10;

float groundPressure = -1;



//list of variables to store different kinds of data to flash to the CSV
struct dataList {
  uint32_t recordNumber;
  uint32_t timeStamp;
  float bmpTemperature;
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
  bool deploy;
  bool retract;
  bool apogee;
  bool touchdown;
} oneRecord;

char filename[16];

uint8_t dumpFileNumber = 1;

uint8_t isRecording = false;

uint32_t prevMillis, currentMillis = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  if (!SerialFlash.begin( PIN_FLASH_CS )) {
    Serial.println(F("SPI Flash not detected. Freezing..."));
    while (1);
  }

  if(!myIMU.init()){
    Serial.println("ICM20948 is not responding. Freezing...");
    while(1);
  }

  unsigned status = bmp.begin();
  if(!status){
    Serial.println("Could not find valid BMP. Freezing...");
    while(1);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setGyrSampleRateDivider(10);
  myIMU.setTempDLPF(ICM20948_DLPF_6);
  myIMU.setMagOpMode(AK09916_CONT_MODE_20HZ);
  

  Serial.println(F("ERASING FLASH CHIP"));
  SerialFlash.eraseAll();

  // create a dummy file so we can look for it
  // this seems the only way of knowing if there is an actual
  // filesystem present when we startup ...
  SerialFlash.create( "dummy.txt", 16 );
  file = SerialFlash.open("dummy.txt");
  file.write( "Hello", 6 );
  file.close();
  Serial.println(F("Done"));

  if ( !SerialFlash.exists( "dummy.txt" )) {
    Serial.println(F("Flash doesn't appear to hold a file system despite a file system just being created. Most likely software issue, but try a different flash chip to be sure. Freezing."));
    while(1);
  }
  else {
    Serial.println(F("Files currently in flash:"));
    SerialFlash.opendir();
    while (1) {
      uint32_t filesize;
      if (SerialFlash.readdir(filename, sizeof(filename), filesize)) {
        Serial.print(filename);
        Serial.print(F("  "));
        Serial.print(filesize);
        Serial.print(F(" bytes"));
        Serial.println();
      }
      else {
        break; // no more files
      }
    }
  }
  isRecording = true;
  groundPressure = bmp.readPressure();

}

void loop() {
  // put your main code here, to run repeatedly:

  currentMillis = millis();
  if (isRecording == true) {
    // is it time for another sample?
    if ( currentMillis - prevMillis > sampleInterval ) {
      prevMillis = currentMillis;

      // prepare the record for writing to the flash chip
      // read in the data from sensors
      myIMU.readSensor();
      oneRecord.recordNumber++;
      oneRecord.timeStamp = currentMillis;
      oneRecord.bmpTemperature = bmp.readTemperature();
      oneRecord.imuTemperature = myIMU.getTemperature();
      oneRecord.pressure = bmp.readPressure();
      oneRecord.accelX = myIMU.getGValues().x;
      oneRecord.accelY = myIMU.getGValues().y;
      oneRecord.accelZ = myIMU.getGValues().z;
      oneRecord.resAcceleration = myIMU.getResultantG(myIMU.getGValues());
      oneRecord.altitude = updateAltitudes();
      oneRecord.magValueX = myIMU.getMagValues().x;
      oneRecord.magValueY = myIMU.getMagValues().y;
      oneRecord.magValueZ = myIMU.getMagValues().z;
      oneRecord.gyrX = myIMU.getGyrValues().x;
      oneRecord.gyrY = myIMU.getGyrValues().y;
      oneRecord.gyrZ = myIMU.getGyrValues().z;
      if(!oneRecord.launch){
        oneRecord.launch = launchDetection();
      }
      if(oneRecord.launch &&(!oneRecord.burnout)){
        oneRecord.burnout = burnoutDetection();
      }
      
     

      // write the record to the flash chip
      file.write( (uint8_t *)&oneRecord, sizeof(oneRecord) );
    }
  }

}

float updateAltitudes(){
  altitudes[0] = altitudes[1];
  altitudes[1] = altitudes[2];
  altitudes[2] = altitudes[3];
  altitudes[3] = altitudes[4];
  altitudes[4] = bmp.readAltitude(groundPressure);
  return altitudes[4];
}

bool launchDetection(){
  if(altitudes[4]>altitudes[3] && altitudes[3]>altitudes[2] && altitudes[2]>altitudes[1] && altitudes[1]>altitudes[0] && oneRecord.resAcceleration < -1){
    return true;
  }
  return false;
}

bool burnoutDetection(){
  if(oneRecord.resAcceleration > 0){
    return true;
  }
  return false;
}

bool apogeeDetection(){
  if(altitudes[4]<altitudes[3] && altitudes[3]>altitudes[2] && altitudes[2]>altitudes[1]){
    return true;
  }
  return false;
}

bool touchdownDetection(){
  
}

