#include <Arduino.h>

#define LED_PIN 2

int time_H;
int time_L;
int Frequency;
int encoder0PinA = 18;
int encoder0PinB = 19;
int encoder0Pos = 0;
int encoder0PinALast = LOW;
int n = LOW;
float t_period;

double Setpoint, Input, Output;
// put function declarations here:

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN,OUTPUT);
  Serial.begin(9600);
  pinMode (encoder0PinA,INPUT);
  pinMode (encoder0PinB,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // delay(500);
  // digitalWrite(LED_PIN,HIGH);
  // delay(500);
  // digitalWrite(LED_PIN,LOW);
  n = digitalRead(encoder0PinA);
  if ((encoder0PinALast == LOW) && (n == HIGH)) { // Test For Signal A and B Direction
    if (digitalRead(encoder0PinB) == LOW) {
      // encoder0Pos--; // Recommend not to use
      Serial.print (" CW, "); // Mark Direction CW
    } else {
      // encoder0Pos++; // Recommend not to use
      Serial.print (" CCW, "); // Mark Direction CCW
    }
    time_H = pulseIn(encoder0PinA,HIGH);
    time_L = pulseIn(encoder0PinA,LOW);
    t_period = time_H+time_L;
    t_period = t_period/1000;
    Frequency = 1000/t_period;
    if(Frequency>3000){ // Set error Variable for t_period more
      Frequency = 0;
    }
    Input = Frequency;
    Serial.print("PV = "); // Recommend not to use
    Serial.print(Input); // Recommend not to use
  }
  encoder0PinALast = n;
}
