#include <Arduino.h>
#include <math.h>

// กำหนดค่าพารามิเตอร์ที่ต้องใช้
const float D = 0.5; // ระยะห่างระหว่างล้อซ้ายและขวา (เมตร)
const float R_L = 0.1; // รัศมีของล้อซ้าย (เมตร)
const float R_R = 0.1; // รัศมีของล้อขวา (เมตร)
const float d = 2.0; // ระยะทางที่หุ่นยนต์เคลื่อนที่ (เมตร)
const float theta = 0.785; // มุมที่หุ่นยนต์ต้องหันไป (เรเดียน) (45 องศา)

// ฟังก์ชันคำนวณมุมที่ล้อซ้ายและขวาต้องหมุน
void calculateWheelAngles() {
  // คำนวณระยะทางที่ล้อซ้ายและขวาหมุนตามมุม theta
  float arcLength_L = (d - D * tan(theta)) / 2.0; // ระยะทางของล้อซ้าย (ถ้าหมุนในวงกลม)
  float arcLength_R = (d + D * tan(theta)) / 2.0; // ระยะทางของล้อขวา (ถ้าหมุนในวงกลม)

  // คำนวณมุมที่ล้อซ้ายและขวาหมุน (มุมในหน่วยเรเดียน)
  float theta_L = arcLength_L / R_L;
  float theta_R = arcLength_R / R_R;

  // แสดงผลลัพธ์
  Serial.print("Theta Left (rad): ");
  Serial.println(theta_L);
  Serial.print("Theta Right (rad): ");
  Serial.println(theta_R);
}

void setup() {
  // เริ่มต้นการสื่อสารทางซีเรียล
  Serial.begin(115200);
  while (!Serial); // รอการเริ่มต้นของ Serial
  
  // คำนวณมุมที่ล้อซ้ายและขวาต้องหมุน
  calculateWheelAngles();
}

void loop() {
  // ไม่มีการทำงานใน loop
}