#define DIR1 37
#define PWM1 36

#define encoderPinA 22
#define encoderPinB 23

volatile long encoderCount = 0;

long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;
const int target = 2000000;
int direction = 1;

void setup(){
  Serial.begin(11520);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  //attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
}

void loop(){
  float kp = 0.0;
  float kd = 0.0;
  float ki = 0.0;
  float u = pidController(target, kp, kd, ki);
  
  moveMotor(DIR1, PWM1, u);
  int a = digitalRead(encoderPinA);
  int b = digitalRead(encoderPinB);
  Serial.print(target);
  Serial.print(", ");
  Serial.print(encoderCount);
  Serial.print(" ");
  Serial.println(direction);
  
  if(digitalRead(encoderPinA) < digitalRead(encoderPinB)){
    encoderCount++;
  }
  else if(digitalRead(encoderPinA) > digitalRead(encoderPinB)){
    encoderCount--;
  }

}

void handleEncoder(){
  Serial.begin(11520);
  int a = digitalRead(encoderPinA);
  int b = digitalRead(encoderPinB);
  Serial.print(target);
  Serial.print(", ");
  Serial.print(encoderCount);
  Serial.println(" from interrupt");

  if(digitalRead(encoderPinA) < digitalRead(encoderPinB)){
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
  if(encoderCount > target){
    direction = 0;
  }
  if(direction == 1){
    digitalWrite(dirPin, direction);
    analogWrite(pwmPin, speed);
  } else {
    Serial.println("second block");
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, speed);
  }
  
}

float pidController(int target, float kp, float kd, float ki){
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTime))/1.0e6;

  int e = encoderCount-target;
  float eDerivative = (e-ePrevious)/deltaT;
  eIntegral = eIntegral + e*deltaT;

  float u = (kp*e) + (kd*eDerivative) + (ki*eIntegral);

  previousTime = currentTime;
  ePrevious = e;

  return u;
}
