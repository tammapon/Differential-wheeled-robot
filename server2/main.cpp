#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h> // For parsing JSON

// const char *ssid = "Robotics";
// const char *password = "Robotics";
const char *ssid = "KluiTMP_LAB_2.4G";
const char *password = "0872449955";

AsyncWebServer server(80);
WebSocketsServer webSocket(81); // WebSocket server on port 81

float realTimeSpeed = 0;
float posX = 0;
float posY = 0;
float currentAngle = 0; // Add a variable to store the current angle

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
        } else if (strcmp(direction, "Backward") == 0) {
          Serial.println("Moving Backward");
          // Calculate motor speeds for backward movement
          // Example:
          // setMotorSpeed(-vL, -vR);
        } else if (strcmp(direction, "Turn Left") == 0) {
          Serial.println("Turning Left");
          // Calculate motor speeds for turning left
          // Example:
          // setMotorSpeed(-vL, vR);
        } else if (strcmp(direction, "Turn Right") == 0) {
          Serial.println("Turning Right");
          // Calculate motor speeds for turning right
          // Example:
          // setMotorSpeed(vL, -vR);
        } else if (strcmp(direction, "Stop") == 0) {
          Serial.println("Stopping");
          // Stop motors
          // Example:
          // setMotorSpeed(0, 0);
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

void setup() {
  Serial.begin(115200);

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
  delay(10);
}