// ESP32-CAM WebSocket-Controlled Image Capture with SPIFFS Storage

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <esp_camera.h>
#include <FS.h>
#include <SPIFFS.h>

// Wi-Fi Credentials
const char* ssid = "MSPWifiBridge";
const char* password = "123456789";

WebSocketsServer webSocket(81);

bool capturing = false;
unsigned long lastCaptureTime = 0;
const unsigned long captureInterval = 1000;  // Capture every 1 second
int imageCount = 0;

#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27

#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22
#define FRAMESIZE FRAMESIZE_XGA

void startCamera() {
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: %s\n", esp_err_to_name(err));
    return;
  }
}

void saveImageToSPIFFS() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  String path = "/img" + String(imageCount) + ".jpg";
  File file = SPIFFS.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in write mode");
  } else {
    file.write(fb->buf, fb->len);
    Serial.println("Saved: " + path);
    imageCount++;
  }
  file.close();
  esp_camera_fb_return(fb);
}

void handleCommand(String cmd, uint8_t client_id) {
  if (cmd == "start") {
    capturing = true;
    imageCount = 0;
    webSocket.sendTXT(client_id, "[ESP] Capturing started.");
  } else if (cmd == "stop") {
    capturing = false;
    webSocket.sendTXT(client_id, "[ESP] Capturing stopped.");
  } else if (cmd == "clear") {
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
      SPIFFS.remove(file.name());
      file = root.openNextFile();
    }
    webSocket.sendTXT(client_id, "[ESP] SPIFFS cleared.");
  } else if (cmd == "download") {
    for (int i = 0; i < imageCount; i++) {
      String path = "/img" + String(i) + ".jpg";
      File file = SPIFFS.open(path, FILE_READ);
      if (file) {
        size_t len = file.size();
        uint8_t* buffer = (uint8_t*)malloc(len);
        file.read(buffer, len);
        webSocket.sendBIN(client_id, buffer, len);
        free(buffer);
        file.close();
      }
    }
    webSocket.sendTXT(client_id, "[ESP] Download complete.");
  }
}

void onWebSocketEvent(uint8_t client_id, WStype_t type, uint8_t* payload, size_t len) {
  if (type == WStype_TEXT) {
    String cmd = String((char*)payload);
    handleCommand(cmd, client_id);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected: " + WiFi.localIP().toString());

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
    return;
  }

  startCamera();
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void loop() {
  webSocket.loop();

  if (capturing && millis() - lastCaptureTime >= captureInterval) {
    lastCaptureTime = millis();
    saveImageToSPIFFS();
  }
}
