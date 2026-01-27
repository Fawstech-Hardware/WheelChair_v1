#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

// --- 1. CONFIGURATION ---
const char* ssid = "Fawstech Innovations Pvt Ltd";
const char* password = "Jersarfaws";

IPAddress local_IP(192, 168, 0, 126);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// LED Pin (Yellow LED on XIAO S3 is GPIO 21, Active LOW)
#define STATUS_LED 21 

httpd_handle_t stream_httpd = NULL;

// --- 2. STREAM HANDLER ---
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  char * part_buf[64];
  static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
  static const char* _STREAM_BOUNDARY = "\r\n--frame\r\n";
  static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) { delay(10); continue; }
    size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
    delay(1); 
  }
  return res;
}

// --- 3. SETUP ---
void setup() {
  Serial.begin(115200);
  // Wait for Serial but continue after 3 seconds if not connected
  unsigned long start = millis();
  while (!Serial && (millis() - start) < 3000);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH); // LED OFF

  Serial.println("\n\n--- XIAO S3 BOOTING ---");

  // A. Camera Configuration
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA; 
  config.jpeg_quality = 12;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  // B. Hardware Initialization
  if (!psramInit()) {
    Serial.println("❌ PSRAM Init Failed!");
    while(1) { digitalWrite(STATUS_LED, LOW); delay(100); digitalWrite(STATUS_LED, HIGH); delay(100); }
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera Init Failed: 0x%x\n", err);
    while(1) { digitalWrite(STATUS_LED, LOW); delay(100); digitalWrite(STATUS_LED, HIGH); delay(100); }
  }
  Serial.println("✅ Hardware Ready");

  // C. WiFi Connection
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(STATUS_LED, LOW); // Blink while connecting
    delay(250);
    digitalWrite(STATUS_LED, HIGH);
    delay(250);
    Serial.print(".");
  }

  // D. Start Server
  httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
  server_config.server_port = 80;
  httpd_uri_t stream_uri = { .uri = "/", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };

  if (httpd_start(&stream_httpd, &server_config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    digitalWrite(STATUS_LED, LOW); // Solid ON means Connected
    Serial.println("\n🚀 Vision Node Online at 192.168.0.126");
  }
}

void loop() {
  // If WiFi lost, blink and try to reconnect every 10s
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 10000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      digitalWrite(STATUS_LED, HIGH);
      WiFi.reconnect();
    } else {
      digitalWrite(STATUS_LED, LOW); // Stay solid ON
    }
  }
  delay(10);
}