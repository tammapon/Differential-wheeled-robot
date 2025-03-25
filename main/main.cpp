//Pulses encoder แยกล้อ

#include <Arduino.h>

// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT_A 18

void right_wheel_pulse();
 
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
 
void setup() {
 
  // Open the serial port at 9600 bps
  Serial.begin(115200); 
 
  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
 
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
   
}
 
void loop() {
    Serial.print(" Pulses: ");
    Serial.println(right_wheel_pulse_count);  
}
 
// 360 degree = 240 pulse 236

// Increment the number of pulses by 1
void right_wheel_pulse() {
  right_wheel_pulse_count++;
}
