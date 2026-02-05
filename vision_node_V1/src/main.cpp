#include "esp_camera.h"
#include "esp_http_server.h"
#include <Arduino.h>
#include <WiFi.h>

// ========================================================================
// VISUAL NODE v3.0 - DEDICATED CAMERA
// ========================================================================
// Function: MJPEG Streaming over HTTP
// Hardware: Seeed Studio XIAO ESP32S3 Sense
// Note: No sensors attached, purely for Vision (ArUco detection by Brain)
// ========================================================================

// ========================================================================
// 1. CONFIGURATION
// ========================================================================

// Network Settings
const char *ssid = "Fawstech R&D";
const char *password = "R&D@Fawstech";

// Static IP Configuration
IPAddress local_IP(172, 20, 11, 126);
IPAddress gateway(172, 20, 11, 1);
IPAddress subnet(255, 255, 255, 0);

// Status LED (Yellow LED on XIAO S3 is GPIO 21, Active LOW)
#define STATUS_LED 21

// ========================================================================
// 2. CAMERA PINS (XIAO ESP32S3 Sense)
// ========================================================================
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39
#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

// ========================================================================
// 3. GLOBAL VARIABLES
// ========================================================================
httpd_handle_t stream_httpd = NULL;

// ========================================================================
// 4. HTTP STREAM HANDLER
// ========================================================================
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  static const char *_STREAM_CONTENT_TYPE =
      "multipart/x-mixed-replace;boundary=frame";
  static const char *_STREAM_BOUNDARY = "\r\n--frame\r\n";
  static const char *_STREAM_PART =
      "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
    return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      delay(10);
      continue;
    }

    size_t hlen = snprintf(part_buf, 64, _STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY,
                                  strlen(_STREAM_BOUNDARY));
    }

    esp_camera_fb_return(fb);

    if (res != ESP_OK)
      break;

    // Optional: Limit framerate if needed
    // delay(1);
  }
  return res;
}

// ========================================================================
// 5. INITIALIZATION
// ========================================================================

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Resolution: QVGA (320x240) for high framerate, VGA (640x480) for quality
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12; // 0-63 (lower is higher quality)
  config.fb_count = 2;      // Double buffering
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAMERA] ❌ Init failed: 0x%x\n", err);
    return false;
  }

  Serial.println("[CAMERA] ✅ Initialized");
  return true;
}

bool initWiFi() {
  Serial.println("[Wi-Fi] Connecting...");

  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet);
  // Disable power save for low latency streaming
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    digitalWrite(STATUS_LED, (attempts % 2) ? HIGH : LOW); // Blink
    delay(250);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[Wi-Fi] ❌ Connection failed!");
    return false;
  }

  Serial.println();
  Serial.printf("[Wi-Fi] ✅ Connected! IP: %s\n",
                WiFi.localIP().toString().c_str());
  Serial.printf("[Wi-Fi] 📶 RSSI: %d dBm\n", WiFi.RSSI());

  return true;
}

bool initHttpServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t stream_uri = {.uri = "/",
                            .method = HTTP_GET,
                            .handler = stream_handler,
                            .user_ctx = NULL};

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.printf("[HTTP] ✅ Server started at http://%s/\n",
                  WiFi.localIP().toString().c_str());
    return true;
  }

  Serial.println("[HTTP] ❌ Server start failed!");
  return false;
}

// ========================================================================
// 6. SETUP
// ========================================================================
void setup() {
  Serial.begin(115200);

  // Wait a moment for serial
  unsigned long start = millis();
  while (!Serial && (millis() - start) < 3000)
    ;

  Serial.println("\n\n========================================");
  Serial.println("🎥 VISUAL NODE v3.0 - CLEAN STREAMER");
  Serial.println("   XIAO ESP32S3 Sense");
  Serial.println("========================================\n");

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH); // LED OFF initially

  // Initialize PSRAM
  if (!psramInit()) {
    Serial.println("[PSRAM] ❌ Init failed!");
    // Blink fast forever if PSRAM fails
    while (1) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(100);
    }
  } else {
    Serial.println("[PSRAM] ✅ Initialized");
  }

  // Initialize Camera
  if (!initCamera()) {
    // Blink slow if camera fails
    while (1) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(500);
    }
  }

  // Initialize Wi-Fi
  if (!initWiFi()) {
    // Blink SOS ... --- ... if WiFi fails
    while (1) {
      for (int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED, LOW);
        delay(100);
        digitalWrite(STATUS_LED, HIGH);
        delay(100);
      }
      delay(300);
      for (int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED, LOW);
        delay(400);
        digitalWrite(STATUS_LED, HIGH);
        delay(100);
      }
      delay(300);
      for (int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED, LOW);
        delay(100);
        digitalWrite(STATUS_LED, HIGH);
        delay(100);
      }
      delay(1000);
    }
  }

  // Initialize HTTP Server
  initHttpServer();

  // Ready indicator
  Serial.println("\n========================================");
  Serial.println("🚀 SYSTEM READY");
  Serial.printf("   Stream: http://%s/\n", WiFi.localIP().toString().c_str());
  Serial.println("========================================\n");

  digitalWrite(STATUS_LED, LOW); // LED ON = Ready (Active LOW)
}

// ========================================================================
// 7. LOOP
// ========================================================================
void loop() {
  // Check Wi-Fi connection periodically
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(STATUS_LED, HIGH); // LED OFF
    Serial.println("[Wi-Fi] ⚠️ Connection lost, reconnecting...");
    WiFi.reconnect();
    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(200);
    }
    digitalWrite(STATUS_LED, LOW); // LED ON
    Serial.println("[Wi-Fi] ✅ Reconnected");
  }

  delay(2000); // Low activity loop, HTTP is interrupt driven
}