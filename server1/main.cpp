#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

// const char *ssid = "Robotics";
// const char *password = "Robotics";
const char *ssid = "Wipay_IOT";
const char *password = "W!payIOT1234!";

AsyncWebServer server(80);
float realTimeSpeed = 0;
float posX = 0;
float posY = 0;

void setup()
{
  Serial.begin(115200);

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

  // รับค่าความเร็วและตำแหน่ง X, Y ผ่าน HTTP GET
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
}

void loop()
{
  delay(1000);
}
