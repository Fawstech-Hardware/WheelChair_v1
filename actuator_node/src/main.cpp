#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

// --- 1. NETWORK CONFIGURATION ---
const char* ssid = "Fawstech Innovations Pvt Ltd";
const char* password = "Jersarfaws";

// STATIC IP SETUP
IPAddress local_ACT(192, 168, 0, 118); 
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

// --- 2. HARDWARE PINOUT ---
const int PIN_L_FORW = 4;   const int PIN_L_BACK = 2;  const int PIN_L_SPD = 14;
const int PIN_R_FORW = 32;  const int PIN_R_BACK = 33; const int PIN_R_SPD = 27;
const int PIN_TRIG = 5;     const int PIN_ECHO = 34;
const int PIN_BUZZER = 12;
const int PIN_LED = 13;     // Connection Status LED

// --- 3. SAFETY SETTINGS ---
const int SAFETY_STOP_CM = 25;
const unsigned long HEARTBEAT_TIMEOUT = 1500; 

// --- 4. GLOBALS ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
unsigned long lastHeartbeatTime = 0;
long currentDistance = 999;
bool isBrainLinked = false;

// --- 5. MOTOR ACTUATION LOGIC ---
void physicalStop() {
    digitalWrite(PIN_L_FORW, LOW); digitalWrite(PIN_L_BACK, LOW);
    digitalWrite(PIN_R_FORW, LOW); digitalWrite(PIN_R_BACK, LOW);
    analogWrite(PIN_L_SPD, 0);      analogWrite(PIN_R_SPD, 0);
}

void physicalMove(char dir, int spd) {
    if (dir == 'F' && currentDistance < SAFETY_STOP_CM) {
        physicalStop();
        return;
    }

    if (dir == 'S') { physicalStop(); return; }

    digitalWrite(PIN_L_FORW, (dir == 'F' || dir == 'R'));
    digitalWrite(PIN_L_BACK, (dir == 'B' || dir == 'L'));
    digitalWrite(PIN_R_FORW, (dir == 'F' || dir == 'L'));
    digitalWrite(PIN_R_BACK, (dir == 'B' || dir == 'R'));

    analogWrite(PIN_L_SPD, spd);
    analogWrite(PIN_R_SPD, spd);
}

// --- 6. SENSOR UPDATE ---
void updateSensor() {
    digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    long duration = pulseIn(PIN_ECHO, HIGH, 25000);
    currentDistance = (duration == 0) ? 999 : duration * 0.034 / 2;
}

// --- 7. COMMUNICATION HANDLER ---
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("Brain Linked: Client #%u\n", client->id());
        isBrainLinked = true;
    } 
    else if (type == WS_EVT_DISCONNECT) {
        isBrainLinked = false; // Fixed: lowercase f
        physicalStop();
    }
    else if (type == WS_EVT_DATA) {
        // Updated for ArduinoJson v7
        JsonDocument doc; 
        DeserializationError error = deserializeJson(doc, data, len);
        if (error) return;

        const char* cmd = doc["cmd"];

        if (cmd && strcmp(cmd, "H") == 0) {
            lastHeartbeatTime = millis();
            digitalWrite(PIN_LED, !digitalRead(PIN_LED)); 
        } 
        else if (cmd && strcmp(cmd, "M") == 0) {
            const char* dir = doc["dir"];
            int spd = doc["spd"];
            if (dir) physicalMove(dir[0], spd);
        }
    }
}

// --- 8. INITIALIZATION ---
void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_L_FORW, OUTPUT); pinMode(PIN_L_BACK, OUTPUT); pinMode(PIN_L_SPD, OUTPUT);
    pinMode(PIN_R_FORW, OUTPUT); pinMode(PIN_R_BACK, OUTPUT); pinMode(PIN_R_SPD, OUTPUT);
    pinMode(PIN_TRIG, OUTPUT);   pinMode(PIN_ECHO, INPUT);
    pinMode(PIN_BUZZER, OUTPUT); pinMode(PIN_LED, OUTPUT);

    physicalStop();
    
    WiFi.config(local_ACT, gateway, subnet);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }

    ws.onEvent(onEvent);
    server.addHandler(&ws);
    server.begin();
    
    lastHeartbeatTime = millis();
    Serial.println("\n✅ ACTUATOR HAL v4.2.2 ONLINE");
}

void loop() {
    ws.cleanupClients();
    updateSensor();

    if (currentDistance < SAFETY_STOP_CM) {
        physicalStop();
        digitalWrite(PIN_BUZZER, HIGH);
    } else {
        digitalWrite(PIN_BUZZER, LOW);
    }

    if (millis() - lastHeartbeatTime > HEARTBEAT_TIMEOUT) {
        physicalStop();
        digitalWrite(PIN_LED, LOW);
    }
    
    delay(20); 
}