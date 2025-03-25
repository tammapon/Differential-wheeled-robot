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

// set wifi
// const char *ssid = "Robotics";
// const char *password = "Robotics";
// const char *ssid = "KluiTMP_LAB_2.4G";
// const char *password = "0872449955";
const char *ssid = "klui";
const char *password = "123456789";

AsyncWebServer server(80);
WebSocketsServer webSocket(81); // WebSocket server on port 81

// UI param
float realTimeSpeed = 0;
float posX = 0;
float posY = 0;
float currentAngle = 0; // Add a variable to store the current angle
float vL = 0;    // ความเร็วล้อซ้าย
float vR = 0;    // ความเร็วล้อขวา
float v = 0;     // ความเร็วเชิงเส้น
float omega = 0; // ความเร็วเชิงมุม

// robot define
// Parameters
const float wheelRadius = 0.0335;  // [m]
const float wheelBase = 0.09;    // [m]


// mode 0:manual 1:position control 2:velocity feed forward
int MODE = 0;

// position control param
int target_R = 0;
int target_L = 0;

// PID constants
float kp_R = 1;
float kd_R = 0.025;
float ki_R = 0.0;

float kp_L = 1;
float kd_L = 0.025;
float ki_L = 0.0;

long prevT = 0;
volatile int posi_R = 0;
float eprev_R = 0;
float eintegral_R = 0;

volatile int posi_L = 0;
float eprev_L = 0;
float eintegral_L = 0;

// velocity control param
// Reference velocities
float target_velocity_L = 0;
float target_velocity_R = 0;
//max 255 -> v ~360 ->0.7
//max 255 -> v ~1080 ->0.236
float k_feedforward = 0.236;
int interval_time = 0;


// comunicate function
void sendRealTimeData();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
// control function
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void readEncoder_R();
void readEncoder_L();

void setup()
{
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
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    if (!SPIFFS.begin(true))
    {
        Serial.println("An error occurred while mounting SPIFFS");
        return;
    }

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                { request->send(SPIFFS, "/index.html", "text/html"); });
    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request)
                { request->send(SPIFFS, "/script.js", "application/javascript"); });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
                { request->send(SPIFFS, "/style.css", "text/css"); });

    // รับค่าความเร็วและตำแหน่ง X, Y ผ่าน HTTP GET (ยังคงไว้)
    server.on("/updateSpeed", HTTP_GET, [](AsyncWebServerRequest *request)
                {
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
    request->send(200, "text/plain", "Data Updated"); });

    server.begin();

    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}

void loop()
{
    webSocket.loop();   // Keep WebSocket server running
    sendRealTimeData(); // Send real-time data
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
    else if(MODE == 2){

        // time difference
        long currT = micros();
        // float deltaT = ((float) (currT - prevT))/( 1.0e6 );
        Serial.println(currT - prevT);
        
        // motor power
        float u_R = k_feedforward*target_velocity_R;
        float pwr_R = fabs(u_R);
        if( pwr_R > 255 ){
        pwr_R = 255;
        }
        float u_L = k_feedforward*target_velocity_L;
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
        Serial.print(" targetR :");
        Serial.print(target_velocity_R);
        // Serial.print(" velocity_R :");
        // Serial.print(velocityFiltered_R);
        // Serial.print(" velocity_R :");
        // Serial.print(velocity_R);
        // Serial.println();
        Serial.print(" targetL :");
        Serial.print(target_velocity_L);
        // Serial.print(" velocity_L :");
        // Serial.print(velocityFiltered_L);
        // Serial.print(" velocity_L :");
        // Serial.print(velocity_L);
        Serial.println();
        
        if (currT - prevT > interval_time*1000000) {
            Serial.println("time-out");
            prevT = currT;
            setMotor(1,0,PWM_R,IN1_R,IN2_R);
            setMotor(-1,0,PWM_L,IN1_L,IN2_L);
            MODE = 0;
        }

    }
    else{
        delay(10);
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

// Function to handle WebSocket events
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_DISCONNECTED:
        Serial.printf("[%u] Disconnected!\n", num);
        break;
    case WStype_CONNECTED:
    {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
    }
    break;
    case WStype_TEXT:
    {
        Serial.printf("[%u] get Text: %s\n", num, payload);

        // Parse JSON data
        StaticJsonDocument<1024> doc; // Use StaticJsonDocument instead of DynamicJsonDocument
        DeserializationError error = deserializeJson(doc, payload);

        if (error)
        {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
        }

        const char *messageType = doc["type"];

        if (strcmp(messageType, "direction") == 0)
        {
            Serial.println("manual mode on");
            MODE=0;
            const char *direction = doc["data"];
            v = doc["v"].as<float>();
            omega = doc["omega"].as<float>();
            vL = doc["vL"].as<float>();
            vR = doc["vR"].as<float>();
            currentAngle = doc["angle"].as<float>(); // Get the current angle
            Serial.printf("Received Direction: %s, v: %.2f, omega: %.2f, vL: %.2f, vR: %.2f, angle: %.2f\n", direction, v, omega, vL, vR, currentAngle);
            // Process direction command here (e.g., move motors)
            if (strcmp(direction, "Forward") == 0)
            {
                Serial.println("Moving Forward");
                // Calculate motor speeds for forward movement
                setMotor(-1,155,PWM_R,IN1_R,IN2_R);
                setMotor(1,155,PWM_L,IN1_L,IN2_L);
            }
            else if (strcmp(direction, "Backward") == 0)
            {
                Serial.println("Moving Backward");
                // Calculate motor speeds for backward movement
                setMotor(1,155,PWM_R,IN1_R,IN2_R);
                setMotor(-1,155,PWM_L,IN1_L,IN2_L);
            }
            else if (strcmp(direction, "Turn Left") == 0)
            {
                Serial.println("Turning Left");
                // Calculate motor speeds for turning left
                setMotor(-1,155,PWM_R,IN1_R,IN2_R);
                setMotor(-1,155,PWM_L,IN1_L,IN2_L);
            }
            else if (strcmp(direction, "Turn Right") == 0)
            {
                Serial.println("Turning Right");
                // Calculate motor speeds for turning right
                setMotor(1,155,PWM_R,IN1_R,IN2_R);
                setMotor(1,155,PWM_L,IN1_L,IN2_L);
            }
            else if (strcmp(direction, "Stop") == 0)
            {
                Serial.println("Stopping");
                // Stop motors
                setMotor(1,0,PWM_R,IN1_R,IN2_R);
                setMotor(-1,0,PWM_L,IN1_L,IN2_L);
                prevT = 0;
                posi_R = 0;
                eprev_R = 0;
                eintegral_R = 0;
                posi_L = 0;
                eprev_L = 0;
                eintegral_L = 0;
            }
        }
        else if (strcmp(messageType, "params") == 0)
        {
            JsonObject params = doc["data"].as<JsonObject>();
            // Use obj[key].is<T>() instead of containsKey()
            if (params.containsKey("v") && params.containsKey("w") && params.containsKey("time"))
            {
                float v = params["v"].as<float>();
                float w = params["w"].as<float>();
                float time = params["time"].as<float>();
                Serial.printf("Received Velocity Params: v=%.2f, w=%.2f, Time=%.2f\n", v, w, time);
                Serial.println(" ");
                Serial.println("velocity control mode on");
                MODE=2;
                long currT = micros();
                prevT = currT;
                // Process parameters here (e.g., update target position)

                // Inverse kinematics
                float wL = (v - w * wheelBase / 2.0) / wheelRadius;
                float wR = (v + w * wheelBase / 2.0) / wheelRadius;

                target_velocity_L = wL*(236/(2*PI));
                target_velocity_R = -1*wR*(236/(2*PI));
                interval_time = time;

                Serial.print("target_velocity_L : ");
                Serial.print(target_velocity_L);
                Serial.print(" target_velocity_R : ");
                Serial.print(target_velocity_R);
                Serial.print(" interval_time : ");
                Serial.println(interval_time);

            }
            else if (params.containsKey("distance") && params.containsKey("theta"))
            {
                float distance = params["distance"].as<float>();
                float theta = params["theta"].as<float>();
                Serial.printf("Received Position Params: Distance=%.2f, Theta=%.2f\n", distance, theta);
                Serial.println(" ");
                Serial.println("position control mode on");
                MODE=1;
                float arcLength_L = distance - theta * wheelBase / 2.0;//(distance - wheelBase * tan(theta)) / 2.0;
                float arcLength_R = distance + theta * wheelBase / 2.0;//(distance + wheelBase * tan(theta)) / 2.0;
                float theta_L = arcLength_L / wheelRadius;
                float theta_R = arcLength_R / wheelRadius;
                //note
                //236 pulse = 1 rev = 360 degree = 2pi rad
                target_R = theta_R*(236/(2*PI));
                target_L = -1*theta_L*(236/(2*PI));

                Serial.print("distance : ");
                Serial.print(distance);
                Serial.print(" theta : ");
                Serial.println(theta);

                Serial.print("arcLength_L : ");
                Serial.print(arcLength_L);
                Serial.print(" arcLength_R : ");
                Serial.println(arcLength_R);

                Serial.print("theta_R : ");
                Serial.print(theta_R);
                Serial.print(" theta_L : ");
                Serial.print(theta_L);
                Serial.print(" target_R : ");
                Serial.print(target_R);
                Serial.print(" target_L : ");
                Serial.println(target_L);

            }
        }
    }
    break;
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