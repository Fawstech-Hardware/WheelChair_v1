#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

// --- 1. NETWORK CONFIGURATION ---
const char *ssid = "Fawstech R&D";
const char *password = "R&D@Fawstech";

IPAddress local_ACT(172, 20, 11, 118);
IPAddress gateway(172, 20, 11, 1);
IPAddress subnet(255, 255, 255, 0);

// --- 2. HARDWARE PINOUT ---
const int PIN_L_FORW = 4;
const int PIN_L_BACK = 2;
const int PIN_L_SPD = 14;
const int PIN_R_FORW = 32;
const int PIN_R_BACK = 33;
const int PIN_R_SPD = 27;
const int PIN_TRIG = 5;
const int PIN_ECHO = 34;
const int PIN_BUZZER = 12;
const int PIN_LED = 13;

// --- 3. SAFETY SETTINGS ---
const int SAFETY_STOP_CM = 25;
const unsigned long HEARTBEAT_TIMEOUT = 1500;

// --- 4. GLOBALS ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
unsigned long lastHeartbeatTime = 0;
long currentDistance = 999;
bool isBrainLinked = false;

// --- 5. MOTOR CONTROL FUNCTIONS ---
void physicalStop() {
  digitalWrite(PIN_L_FORW, LOW);
  digitalWrite(PIN_L_BACK, LOW);
  digitalWrite(PIN_R_FORW, LOW);
  digitalWrite(PIN_R_BACK, LOW);
  analogWrite(PIN_L_SPD, 0);
  analogWrite(PIN_R_SPD, 0);
}

// Set individual motor (simplified from your current logic)
void setLeftMotor(char dir, int spd) {
  dir = toupper(dir);
  if (dir == 'F') {
    digitalWrite(PIN_L_FORW, HIGH);
    digitalWrite(PIN_L_BACK, LOW);
  } else if (dir == 'B') {
    digitalWrite(PIN_L_FORW, LOW);
    digitalWrite(PIN_L_BACK, HIGH);
  } else {
    digitalWrite(PIN_L_FORW, LOW);
    digitalWrite(PIN_L_BACK, LOW);
  }
  analogWrite(PIN_L_SPD, spd);
}

void setRightMotor(char dir, int spd) {
  dir = toupper(dir);
  if (dir == 'F') {
    digitalWrite(PIN_R_FORW, HIGH);
    digitalWrite(PIN_R_BACK, LOW);
  } else if (dir == 'B') {
    digitalWrite(PIN_R_FORW, LOW);
    digitalWrite(PIN_R_BACK, HIGH);
  } else {
    digitalWrite(PIN_R_FORW, LOW);
    digitalWrite(PIN_R_BACK, LOW);
  }
  analogWrite(PIN_R_SPD, spd);
}

// NEW: Differential drive control (for PID system)
void differentialMove(char left_dir, int left_spd, char right_dir,
                      int right_spd) {
  // Safety check: Don't move forward if obstacle is near
  if (left_dir == 'F' && right_dir == 'F' && currentDistance < SAFETY_STOP_CM) {
    physicalStop();
    return;
  }

  setLeftMotor(left_dir, left_spd);
  setRightMotor(right_dir, right_spd);
}

// OLD: Kept for backward compatibility (your current protocol)
void simpleMove(char dir, int spd) {
  if (dir == 'F' && currentDistance < SAFETY_STOP_CM) {
    physicalStop();
    return;
  }

  if (dir == 'S') {
    physicalStop();
    return;
  }

  // Your original differential steering logic
  digitalWrite(PIN_L_FORW, (dir == 'F' || dir == 'R'));
  digitalWrite(PIN_L_BACK, (dir == 'B' || dir == 'L'));
  digitalWrite(PIN_R_FORW, (dir == 'F' || dir == 'L'));
  digitalWrite(PIN_R_BACK, (dir == 'B' || dir == 'R'));

  analogWrite(PIN_L_SPD, spd);
  analogWrite(PIN_R_SPD, spd);
}

// --- 6. SENSOR UPDATE ---
void updateSensor() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duration = pulseIn(PIN_ECHO, HIGH, 25000);
  currentDistance = (duration == 0) ? 999 : duration * 0.034 / 2;
}

// --- 7. COMMUNICATION HANDLER ---
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("Brain Linked: Client #%u\n", client->id());
    isBrainLinked = true;
    digitalWrite(PIN_LED, HIGH); // LED on when connected
  } else if (type == WS_EVT_DISCONNECT) {
    isBrainLinked = false;
    physicalStop();
    digitalWrite(PIN_LED, LOW); // LED off when disconnected
  } else if (type == WS_EVT_DATA) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, data, len);
    if (error) {
      Serial.print("JSON Error: ");
      Serial.println(error.c_str());
      return;
    }

    const char *cmd = doc["cmd"];

    if (cmd && strcmp(cmd, "H") == 0) {
      lastHeartbeatTime = millis();
    } else if (cmd && strcmp(cmd, "M") == 0) {
      // NEW PROTOCOL: Check if new format fields exist
      if (doc.containsKey("left_dir") && doc.containsKey("left_spd") &&
          doc.containsKey("right_dir") && doc.containsKey("right_spd")) {

        // Differential drive (PID mode)
        const char *left_dir = doc["left_dir"];
        const char *right_dir = doc["right_dir"];
        int left_spd = doc["left_spd"];
        int right_spd = doc["right_spd"];

        differentialMove(left_dir[0], left_spd, right_dir[0], right_spd);

        Serial.printf("DIFF: L=%c%d R=%c%d\n", left_dir[0], left_spd,
                      right_dir[0], right_spd);
      }
      // OLD PROTOCOL: Check if old format fields exist
      else if (doc.containsKey("dir") && doc.containsKey("spd")) {
        // Simple move (backward compatibility)
        const char *dir = doc["dir"];
        int spd = doc["spd"];

        simpleMove(dir[0], spd);

        Serial.printf("SIMPLE: %c @ %d\n", dir[0], spd);
      } else {
        Serial.println("⚠️ Unknown motor command format");
      }
    }
  }
}

// --- 8. INITIALIZATION ---
void setup() {
  Serial.begin(115200);

  pinMode(PIN_L_FORW, OUTPUT);
  pinMode(PIN_L_BACK, OUTPUT);
  pinMode(PIN_L_SPD, OUTPUT);
  pinMode(PIN_R_FORW, OUTPUT);
  pinMode(PIN_R_BACK, OUTPUT);
  pinMode(PIN_R_SPD, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  physicalStop();

  // Configure WiFi with static IP
  WiFi.config(local_ACT, gateway, subnet);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✅ ACTUATOR HAL v5.0 ONLINE");
  Serial.print("📡 IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("🤖 Support: Simple & Differential Drive");

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();

  lastHeartbeatTime = millis();
}

void loop() {
  ws.cleanupClients();
  updateSensor();

  // Safety: Stop if obstacle too close (any forward movement)
  if (currentDistance < SAFETY_STOP_CM) {
    digitalWrite(PIN_BUZZER, HIGH);
    physicalStop(); // 🛑 Actually stop motors!
  } else {
    digitalWrite(PIN_BUZZER, LOW);
  }

  // Safety: Stop if heartbeat lost
  if (millis() - lastHeartbeatTime > HEARTBEAT_TIMEOUT) {
    physicalStop();
    digitalWrite(PIN_LED, LOW); // Indicate connection lost
  }

  // 📡 Send distance to Brain every 100ms
  static unsigned long lastDistSend = 0;
  if (isBrainLinked && millis() - lastDistSend > 100) {
    String msg = "{\"dist\":" + String(currentDistance) + "}";
    ws.textAll(msg);
    lastDistSend = millis();
  }

  delay(20);
}