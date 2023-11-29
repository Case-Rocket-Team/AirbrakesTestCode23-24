#include <Wire.h>
#include <SPIMemory.h>
#include <Arduino.h>

#define BAUD_RATE 9600
#define CS_PIN 7

struct flight_counts{
 int num_flights;
 int flight_addr_strt[10];
 int flight_length[10];
} flights;

SPIFlash flash(CS_PIN);
// Program Related variables
float a,P; // altitude and pressure
float t; // time
double baseline; //baseline pressure used for calculations

int poll = 1000 / 10;
int current_address = -1;
long flights_adr = 256 * 35000; //page size * page number


double getPressure() {
  return 5;
}

// Real Code Time
void setup() {
  Serial.begin(BAUD_RATE);
  flash.begin();
  Serial.println(flash.error());
  
  /** Testing for Serial Connection **/
  bool is_serial = false;
  char do_read = 'q';
  char do_erase = 'q';
  for(int i = 0; i < 5; i++) {
    delay(500);
    if(Serial) {
      is_serial = true;
      i = 6;
    }
  }

  /**making sure we get and ID off of the chip **/
  int id = flash.getJEDECID();
  //Serial.println(id);
  if (id) {
    
    delay(50);
    delay(50);
    delay(50);
  } else {
    Serial.println("No comms. Check wiring. Is chip supported? If unable to fix, raise an issue on Github");
    Serial.println(id);
  }


  bool can_read = flash.readAnything(flights_adr,flights);
  Serial.print(flights.num_flights);
  Serial.print(" flights recorded\n");

  if(is_serial) {
    Serial.println("Would you like to read data y/n");
    while(do_read != 'y' && do_read != 'n') {
      delay(1000);
      do_read = Serial.read();
    }
    Serial.println(do_read);
  }
  bool reading = false;
  //Serial.println(can_read);
  if(do_read == 'y') {
    reading = true;
   
    for(int i = 0; i < flights.num_flights; i++) {
      
      int fl_strt = flights.flight_addr_strt[i];
      int lgth = flights.flight_length[i];
      Serial.print("reading flight starting at: ");
      Serial.println(fl_strt);
      Serial.print("reading flight with length: ");
      Serial.println(lgth);
      byte data[lgth];
      if(lgth > 0 && fl_strt >= 0 && flash.readByteArray(fl_strt,data,lgth)) {
        float * float_data = (float *)(data);
        Serial.print(" altitude, ");
        Serial.print(" Pressure, ");
        Serial.println(" Time, ");
        for(int j = 0; j < lgth / 8; j+=3) {
          Serial.print(float_data[j]);
          Serial.print("\t");
          Serial.print(float_data[j+1]);
          Serial.print("\t");
          Serial.println(float_data[j+2]);
        }
      } else {
        Serial.println("Unable to read from chip!");
      }
    }
    reading = false;
  
  } else {
    flights.num_flights++; //update to a new flight
    flights.flight_length[flights.num_flights-1] = 0; //setting the length from -1 to 0
  }
  while(reading);
  if(is_serial) {
    Serial.println("Would you like to erase chip y/n");
    while(do_erase != 'y' && do_erase != 'n') {
        delay(1000);
        do_erase = Serial.read();
    }
    Serial.println(do_erase);
  }

  if(do_erase == 'y') {
    Serial.println("erasing will begin in 2 seconds...");
    delay(2000);
    Serial.println("erasing...");
    if(flash.eraseChip()) {
    Serial.print("erase successful in ");
    Serial.print("min and ");
    Serial.println("sec");
    flights.num_flights = 0;
    flash.eraseSector(flights_adr);
    flash.writeAnything(flights_adr, flights);
    } else {
      Serial.println("could not erase!");
      while(1);
    }
  }

  if(Serial) {
    while(1);
  }


  
  baseline = getPressure();
  flash.eraseSector(flights_adr);
  flash.writeAnything(flights_adr,flights);
}



void loop() {
   // find the next empty spot
  if((int) millis() - (int) t >= poll) {
    P = (float) getPressure();
    t = (float) millis();
    
    int free_add = flash.getAddress(sizeof(a));
    if(current_address < 0)
      flights.flight_addr_strt[flights.num_flights - 1] = free_add;
    if (flash.writeByteArray(free_add, ((byte *)&a), sizeof(a))) {
      Serial.print(a);
      Serial.print(" m, ");
    } else {
      Serial.println("write failed :(");
    }
    
    free_add = flash.getAddress(sizeof(P));
    if (flash.writeByteArray(free_add, ((byte *)&P), sizeof(P))) {
      Serial.print(P);
      Serial.print(" atm, ");
    } else {
      Serial.println("write failed :(");
    }

    free_add = flash.getAddress(sizeof(t));
    if (flash.writeByteArray(free_add, ((byte *)&t), sizeof(t))) {
      Serial.print(t);
      Serial.println(" ms, ");
    } else {
      Serial.println("write failed :(");
    }
    current_address = free_add;
    Serial.println(current_address);
    flights.flight_length[flights.num_flights - 1] += (4 * 3);
    Serial.println("writing flight length...");
    flash.eraseSector(flights_adr);
    flash.writeAnything(flights_adr, flights);
  }
}
