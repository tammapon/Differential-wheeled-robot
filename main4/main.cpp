#include <Arduino.h>
//right
#define ENCA_R 18 // YELLOW
#define ENCB_R 19 // WHITE
#define PWM_R 12
#define IN2_R 13
#define IN1_R 14
//left
#define ENCA_L 33 // YELLOW
#define ENCB_L 32 // WHITE
#define PWM_L 15
#define IN2_L 2
#define IN1_L 4

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void readEncoder_R();
void readEncoder_L();

volatile int posi_R = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev_R = 0;
float eintegral_R = 0;

volatile int posi_L = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
// long prevT_L = 0;
float eprev_L = 0;
float eintegral_L = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA_R,INPUT);
  pinMode(ENCB_R,INPUT);
  pinMode(ENCA_L,INPUT);
  pinMode(ENCB_L,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_R),readEncoder_R,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_L),readEncoder_L,RISING);
  
  pinMode(PWM_R,OUTPUT);
  pinMode(IN1_R,OUTPUT);
  pinMode(IN2_R,OUTPUT);
  pinMode(PWM_L,OUTPUT);
  pinMode(IN1_L,OUTPUT);
  pinMode(IN2_L,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
//   int target = 1200;
//   int target_R = 250*sin(prevT/1e6);
//   int target_L = 250*sin(prevT/1e6);
    int target_R = 1200;
    int target_L = 1200;

  // PID constants
  float kp_R = 1;
  float kd_R = 0.025;
  float ki_R = 0.0;

  float kp_L = 1;
  float kd_L = 0.025;
  float ki_L = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos_R = 0; 
  int pos_L = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos_R = posi_R;
  pos_L = posi_L;
  interrupts(); // turn interrupts back on
  

  // error
  int e_R = pos_R - target_R;
  int e_L = pos_L - target_L;

  // derivative
  float dedt_R = (e_R-eprev_R)/(deltaT);
  float dedt_L = (e_L-eprev_L)/(deltaT);

  // integral
  eintegral_R = eintegral_R + e_R*deltaT;
  eintegral_L = eintegral_L + e_L*deltaT;

  // control signal
  float u_R = kp_R*e_R + kd_R*dedt_R + ki_R*eintegral_R;
  float u_L = kp_L*e_L + kd_L*dedt_L + ki_L*eintegral_L;

  // motor power
  float pwr_R = fabs(u_R);
  if( pwr_R > 255 ){
    pwr_R = 255;
  }
  float pwr_L = fabs(u_L);
  if( pwr_L > 255 ){
    pwr_L = 255;
  }

  // motor direction
  int dir_R = 1;
  int dir_L = 1;
  if(u_R<0){
    dir_R = -1;
  }
  if(u_L<0){
    dir_L = -1;
  }

  // signal the motor
  setMotor(dir_R,pwr_R,PWM_R,IN1_R,IN2_R);
  setMotor(dir_L,pwr_L,PWM_L,IN1_L,IN2_L);


  // store previous error
  eprev_R = e_R;
  eprev_L = e_L;

  Serial.print(" targetR :");
  Serial.print(target_R);
  Serial.print(" posR :");
  Serial.print(pos_R);
  Serial.print(" targetL :");
  Serial.print(target_L);
  Serial.print(" posL :");
  Serial.print(pos_L);
//   Serial.print(" u:");
//   Serial.print(u);
//   Serial.print(" dir:");
//   Serial.print(dir);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder_R(){
  int bR = digitalRead(ENCB_R);
  if(bR > 0){
    posi_R++;
  }
  else{
    posi_R--;
  }
}
void readEncoder_L(){
    int bL = digitalRead(ENCB_L);
    if(bL > 0){
      posi_L++;
    }
    else{
      posi_L--;
    }
  }