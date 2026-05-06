#include <WiFi.h>               // A1
#include <ESPAsyncWebServer.h>   // A2
#include <ArduinoJson.h>         // A3

const char* ssid     = "RobotSumo";   // A4
const char* password = "12345678";    // A4

#define IN1  27    // A5
#define IN2  26
#define ENA  14
#define IN3  25
#define IN4  33
#define ENB  32

#define PWM_FREQ  1000   // A6
#define PWM_RES   8

AsyncWebServer server(80);   // A7
AsyncWebSocket ws("/ws");    // A7