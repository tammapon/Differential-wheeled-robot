#include <Arduino.h>
#include <math.h>

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

// Parameters
const float wheelRadius = 0.0335;  // [m]
const float wheelBase = 0.09;    // [m]

// Pose [x, y, theta]
float pose[3] = {0.0, 0.0, PI / 4};  // initial pose

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void readEncoder_R();
void readEncoder_L();

float target_velocity_L = 0;
  float target_velocity_R = 0;
  #define FILTER_SIZE 100
  float velocityBuffer_R[FILTER_SIZE];  // buffer สำหรับเก็บค่า velocity
int bufferIndex = 0;
float velocityFiltered_R = 0;
float velocityBuffer_L[FILTER_SIZE];  // buffer สำหรับเก็บค่า velocity
float velocityFiltered_L = 0;

float k_feedforward = 0.7;

volatile int posi_R = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev_R = 0;
float eintegral_R = 0;
float prevPos_R=1;

volatile int posi_L = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
// long prevT_L = 0;
float eprev_L = 0;
float eintegral_L = 0;
float prevPos_L=1;

void setup() {
  Serial.begin(115200);


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

  // Reference velocities
  float vRef = 0.2;
  float wRef = 0;

  // Inverse kinematics
  float wL = (vRef - wRef * wheelBase / 2.0) / wheelRadius;
  float wR = (vRef + wRef * wheelBase / 2.0) / wheelRadius;

  // Forward kinematics (Body frame)
  float vB = 0.5 * wheelRadius * (wL + wR);
  float wB = (wR - wL) * wheelRadius / wheelBase;

  // Transform to world frame
  float theta = pose[2];
  float vx = vB * cos(theta);
  float vy = vB * sin(theta);
  float omega = wB;

//   // Discrete integration
//   pose[0] += vx * sampleTime;
//   pose[1] += vy * sampleTime;
//   pose[2] += omega * sampleTime;

  Serial.print("vRef : ");
  Serial.print(vRef);
  Serial.print(" wRef : ");
  Serial.print(wRef);
  Serial.print(" wL : ");
  Serial.print(wL);
  Serial.print(" wR : ");
  Serial.print(wR);
  Serial.print(" vB : ");
  Serial.print(vB);
  Serial.print(" wB : ");
  Serial.println(wB);

  //note
  //target velocity pulse/s
  //236 pulse = 1 rev = 360 degree = 2pi rad
  //ref velocity rad/s
  //ref -> target
  //target = ref*(236/2pi)

  target_velocity_L = wL*(236/(2*PI));
  target_velocity_R = -1*wR*(236/(2*PI));
  Serial.print(" target_velocity_L : ");
  Serial.print(target_velocity_L);
  Serial.print(" target_velocity_R : ");
  Serial.println(target_velocity_R);

  //max 255 -> v ~360
  //max 255 -> v ~1080

}

void loop() {


  // PID constants
  float kp_R = 1;
  float kd_R = 0.025;
  float ki_R = 0.01;

  float kp_L = 1;
  float kd_L = 0.025;
  float ki_L = 0.01;

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
  
  // Calculate velocity (change in position over time)
  float velocity_R = (pos_R - prevPos_R) / deltaT;  // velocity = change in position / time difference
    prevPos_R = pos_R;  // store the previous position for next iteration
    float velocity_L = (pos_L - prevPos_L) / deltaT;  // velocity = change in position / time difference
    prevPos_L = pos_L;  // store the previous position for next iteration

    // บันทึกค่าใน buffer
    velocityBuffer_R[bufferIndex] = velocity_R;
    velocityBuffer_L[bufferIndex] = velocity_L;
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;

    // คำนวณค่าเฉลี่ย
    velocityFiltered_R = 0;
    velocityFiltered_L = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        velocityFiltered_R += velocityBuffer_R[i];
        velocityFiltered_L += velocityBuffer_L[i];
    }
    velocityFiltered_R /= FILTER_SIZE;
    velocityFiltered_L /= FILTER_SIZE;

  // error
  int e_R = target_velocity_R - velocityFiltered_R;
  int e_L = target_velocity_L - velocityFiltered_L;

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
//   float pwr_R = fabs(u_R);
  float pwr_R = fabs(k_feedforward*target_velocity_R);
  if( pwr_R > 255 ){
    pwr_R = 255;
  }
//   float pwr_L = fabs(u_L);
  float pwr_L = fabs(k_feedforward*target_velocity_L);
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
  Serial.print(target_velocity_R);
  Serial.print(" velocity_R :");
  Serial.print(velocityFiltered_R);
  Serial.print(" velocity_R :");
  Serial.print(velocity_R);
  Serial.println();
  Serial.print(" targetL :");
  Serial.print(target_velocity_L);
  Serial.print(" velocity_L :");
  Serial.print(velocityFiltered_L);
  Serial.print(" velocity_L :");
  Serial.print(velocity_L);
// //   Serial.print(" u:");
// //   Serial.print(u);
// //   Serial.print(" dir:");
// //   Serial.print(dir);
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