#include <SPIMemory.h>
#define CS_PIN 10
SPIFlash flash(CS_PIN);

float fakeTemperature = 12.0;
float fakePressure = 1013.25;
float fakeAcceleration = 1.0;
float a = -1;
float p = -1;
float temp = -1;
float t = -1;
int freeAddress = -1;
int currentAddress = -1;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
  flash.begin();



  int id = flash.getJEDECID();
  if(id){
    Serial.println("Comms established");
  }else{
    
    Serial.println("Unable to communicate with flash chip. Freezing...");
    while(1);
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  a = getFakeAcceleration();
  p = getFakePressure();
  temp = getFakeTemperature();
  t = (float) millis();
  
  freeAddress = flash.getAddress(sizeof(a));
  if(flash.writeByteArray(freeAddress, ((byte *)&a), sizeof(a))){
    Serial.print(a);
    Serial.print(" m, ");
  } else{
    Serial.println("Write Failed.");
  }

  freeAddress = flash.getAddress(sizeof(p));
  if(flash.writeByteArray(freeAddress, ((byte *)&p), sizeof(p))){
    Serial.print(p);
    Serial.print(" hPa, ");
  } else{
    Serial.println("Write Failed.");
  }

 freeAddress = flash.getAddress(sizeof(temp));
  if(flash.writeByteArray(freeAddress, ((byte *)&temp), sizeof(temp))){
    Serial.print(temp);
    Serial.print(" degrees Celcius, ");
  } else{
    Serial.println("Write Failed.");
  }

  freeAddress = flash.getAddress(sizeof(t));
  if(flash.writeByteArray(freeAddress, ((byte *)&t), sizeof(t))){
    Serial.print(t);
    Serial.print(" ms, ");
  } else{
    Serial.println("Write Failed.");
  }

  currentAddress = freeAddress;
  Serial.println(currentAddress);
  
  

}
  
float getFakeAcceleration() {
  fakeAcceleration = fakeAcceleration + 0.01;
  return fakeAcceleration;
}

float getFakePressure() {
  fakePressure = fakePressure - 0.01;
  return fakePressure;
}

float getFakeTemperature() {
  fakeTemperature = fakeTemperature + 0.01;
  return fakeTemperature;
}
