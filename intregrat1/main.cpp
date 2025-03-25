#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h> // For parsing JSON
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

// const char *ssid = "Robotics";
// const char *password = "Robotics";
const char *ssid = "KluiTMP_LAB_2.4G";
const char *password = "0872449955";

AsyncWebServer server(80);
WebSocketsServer webSocket(81); // WebSocket server on port 81

int MODE = 0;

// set target position
//   int target = 1200;
//   int target_R = 250*sin(prevT/1e6);
//   int target_L = 250*sin(prevT/1e6);
int target_R = 0;
int target_L = 0;

// PID constants
float kp_R = 1;
float kd_R = 0.025;
float ki_R = 0.0;

float kp_L = 1;
float kd_L = 0.025;
float ki_L = 0.0;

//UI param
float realTimeSpeed = 0;
float posX = 0;
float posY = 0;
float currentAngle = 0; // Add a variable to store the current angle
float vL = 0;    // ความเร็วล้อซ้าย
float vR = 0;    // ความเร็วล้อขวา
float v = 0;     // ความเร็วเชิงเส้น
float omega = 0; // ความเร็วเชิงมุม

volatile int posi_R = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev_R = 0;
float eintegral_R = 0;

volatile int posi_L = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
// long prevT_L = 0;
float eprev_L = 0;
float eintegral_L = 0;

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void readEncoder_R();
void readEncoder_L();
// Function to handle WebSocket events
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void sendRealTimeData();

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

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    if (!SPIFFS.begin(true)) {
        Serial.println("An error occurred while mounting SPIFFS");
        return;
    }

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });
    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/script.js", "application/javascript");
    });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/style.css", "text/css");
    });

    // รับค่าความเร็วและตำแหน่ง X, Y ผ่าน HTTP GET (ยังคงไว้)
    server.on("/updateSpeed", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("speed")) {
        realTimeSpeed = request->getParam("speed")->value().toFloat();
        }
        if (request->hasParam("x")) {
        posX = request->getParam("x")->value().toFloat();
        }
        if (request->hasParam("y")) {
        posY = request->getParam("y")->value().toFloat();
        }
        Serial.printf("Speed: %.2f px/s | X: %.2f | Y: %.2f\n", realTimeSpeed, posX, posY);
        request->send(200, "text/plain", "Data Updated");
    });

    server.begin();

    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}

void loop() {
    webSocket.loop(); // Keep WebSocket server running
    if(MODE == 1) {
        Serial.println("control mode");
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
        if(pwr_L<0 && pwr_R<0){
            Serial.println("control mode off");
            MODE=0;
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

        // Serial.print(" targetR :");
        // Serial.print(target_R);
        // Serial.print(" posR :");
        // Serial.print(pos_R);
        // Serial.print(" targetL :");
        // Serial.print(target_L);
        // Serial.print(" posL :");
        // Serial.print(pos_L);
        // //   Serial.print(" u:");
        // //   Serial.print(u);
        // //   Serial.print(" dir:");
        // //   Serial.print(dir);
        // Serial.println();
    }
    else{
        delay(10);
    }
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

// Function to handle WebSocket events
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
        Serial.printf("[%u] Disconnected!\n", num);
        break;
        case WStype_CONNECTED: {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        } break;
        case WStype_TEXT: {
        Serial.printf("[%u] get Text: %s\n", num, payload);

        // Parse JSON data
        StaticJsonDocument<1024> doc; // Use StaticJsonDocument instead of DynamicJsonDocument
        DeserializationError error = deserializeJson(doc, payload);

        if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
        }

        const char* messageType = doc["type"];

        if (strcmp(messageType, "direction") == 0) {
            Serial.println("control mode off");
            MODE=0;
            const char* direction = doc["data"];
            float v = doc["v"].as<float>();
            float omega = doc["omega"].as<float>();
            float vL = doc["vL"].as<float>();
            float vR = doc["vR"].as<float>();
            currentAngle = doc["angle"].as<float>(); // Get the current angle
            Serial.printf("Received Direction: %s, v: %.2f, omega: %.2f, vL: %.2f, vR: %.2f, angle: %.2f\n", direction, v, omega, vL, vR, currentAngle);
            // Process direction command here (e.g., move motors)
            if (strcmp(direction, "Forward") == 0) {
            Serial.println("Moving Forward");
            // Calculate motor speeds for forward movement
            // Example:
            // setMotorSpeed(vL, vR);
            setMotor(-1,255,PWM_R,IN1_R,IN2_R);
            setMotor(1,255,PWM_L,IN1_L,IN2_L);
            } else if (strcmp(direction, "Backward") == 0) {
            Serial.println("Moving Backward");
            // Calculate motor speeds for backward movement
            // Example:
            // setMotorSpeed(-vL, -vR);
            setMotor(1,255,PWM_R,IN1_R,IN2_R);
            setMotor(-1,255,PWM_L,IN1_L,IN2_L);
            } else if (strcmp(direction, "Turn Left") == 0) {
            Serial.println("Turning Left");
            // Calculate motor speeds for turning left
            // Example:
            // setMotorSpeed(-vL, vR);
            setMotor(-1,255,PWM_R,IN1_R,IN2_R);
            setMotor(-1,255,PWM_L,IN1_L,IN2_L);
            } else if (strcmp(direction, "Turn Right") == 0) {
            Serial.println("Turning Right");
            // Calculate motor speeds for turning right
            // Example:
            // setMotorSpeed(vL, -vR);
            setMotor(1,255,PWM_R,IN1_R,IN2_R);
            setMotor(1,255,PWM_L,IN1_L,IN2_L);
            } else if (strcmp(direction, "Stop") == 0) {
            Serial.println("Stopping");
            // Stop motors
            // Example:
            // setMotorSpeed(0, 0);
            setMotor(1,0,PWM_R,IN1_R,IN2_R);
            setMotor(-1,0,PWM_L,IN1_L,IN2_L);
            } else if (strcmp(direction, "Forward Right") == 0) {
            Serial.println("Moving Forward Right");
            // Add your motor control code here
            } else if (strcmp(direction, "Forward Left") == 0) {
            Serial.println("Moving Forward Left");
            // Add your motor control code here
            } else if (strcmp(direction, "Backward Right") == 0) {
            Serial.println("Moving Backward Right");
            // Add your motor control code here
            } else if (strcmp(direction, "Backward Left") == 0) {
            Serial.println("Moving Backward Left");
            // Add your motor control code here
            }
        } else if (strcmp(messageType, "params") == 0) {
            JsonObject params = doc["data"].as<JsonObject>();
            // Use obj[key].is<T>() instead of containsKey()
            if (params["x"].is<float>() && params["y"].is<float>() && params["theta"].is<float>()) {
            float x = params["x"].as<float>();
            float y = params["y"].as<float>();
            float theta = params["theta"].as<float>();
            Serial.printf("Received Params: X=%.2f, Y=%.2f, Theta=%.2f\n", x, y, theta);
            // Process parameters here (e.g., update target position)
            if (params["time"].is<float>()) {
                float time = params["time"].as<float>();
                Serial.printf("Time=%.2f\n", time);
            }
            Serial.println("control mode on");
            MODE=1;
            target_R = -1*x;
            target_L = y;
            }
        }
        } break;
        case WStype_BIN:
        Serial.printf("[%u] get binary length: %u\n", num, length);
        break;
        case WStype_ERROR:
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
        break;
    }
}

void sendRealTimeData()
{
    if (webSocket.connectedClients() > 0)
    {
        StaticJsonDocument<256> doc;
        doc["type"] = "realtime";
        doc["speed"] = realTimeSpeed;
        doc["x"] = posX;
        doc["y"] = posY;
        doc["angle"] = currentAngle; // Include the current angle
        doc["v"] = v;
        doc["omega"] = omega;
        doc["vL"] = vL;
        doc["vR"] = vR;
        char buffer[256];
        serializeJson(doc, buffer);
        webSocket.broadcastTXT(buffer);
    }
}