#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 23 // YELLOW
#define ENCB 22 // WHITE
#define IN2 36
#define IN1 37

volatile int posi = 0; // specify posi as volatile
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int a = digitalRead(ENCA);
int b = digitalRead(ENCB);

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");

  digitalWrite(IN1,LOW);
  analogWrite(IN2,255);
}

void loop() {
  
  // set target position
  int target = 20;
  //int target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 20;
  float kd = 0.0;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // error
  int e = target - pos;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  //eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e; //kd*dedt; + ki*eintegral;
  Serial.println("pos");
  Serial.println(pos);

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = 0;
    
  }


  // signal the motor
  //setMotor(dir,pwr,IN1,IN2);
  //digitalWrite(IN1,LOW);
  //analogWrite(IN2,255);
  // store previous error
  eprev = e;


  
  //Serial.print(target);
  //Serial.print(" ");
  //Serial.print(pos);
  //Serial.println();
  delay(10);
}

void setMotor(int dir, int pwmVal, int in1, int in2){
  //Serial.println(dir);
  //Serial.println(pwmVal);
  if(dir == 1){
    analogWrite(in1,pwmVal);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    analogWrite(in2,pwmVal);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int newA = digitalRead(ENCA);
  int newB = digitalRead(ENCB);
  Serial.println("Reading Encoder");
  Serial.println(posi);
  Serial.print(digitalRead(ENCA));
  Serial.print(" ");
  int b = digitalRead(ENCB);
  Serial.println(b);
  Serial.println(" ");
  
  if(newA == 1 && a == 0){
    if(b == 1)
      posi++;
    else
      posi--;
  }
  a = newA;
  b = newB;
}
