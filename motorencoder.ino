#define DIR1 37
#define PWM1 36

#define encoderPinA 23
#define encoderPinB 22

volatile long encoderCount = 0;

long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

void setup(){
  Serial.begin(9600);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, IMPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
}

void loop(){

  int target = 1000;
  
  float kp = 0.0;
  float kd = 0.0;
  float ki = 0.0;
  float u = pidController(target, kp, kd, ki);
  
  moveMotor(DIR1, PWM1, u);
  
  Serial.print(target);
  Serial.print(", ");
  Serial.print(encoderCount);
}

void handleEncoder(){
  if(digitalRead(encoderPinA) > digitalRead(encoderPinB)){
    encoderCount++;
  }
  else{
    encoderCount--;
  }
}


void moveMotor(int dirPin, int pwmPin, float u){
  float speed = fabs(u);
  if(speed > 255){
    speed = 255;
  }
  int direction = 1;
  if(u<0){
    direction = 0;
  }
  digitalWrite(dirPin, direction);
  analogWrite(pwmPin, speed);
}

float pidController(int target, //to be implemented
