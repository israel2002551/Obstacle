/*
ESP32-CAM Autonomous Explorer - Final Unified Version
Integrated WebSockets, Camera Drivers, Autonomous Logic, and Dynamic Speed Control
*/

#include <WebSocketsServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_camera.h"

// ==========================================
// 1. NETWORK SETTINGS
// ==========================================
const char* ssid = "YOUR_WIFI_NAME";      // <<< Change this
const char* password = "YOUR_WIFI_PASS";  // <<< Change this

// ==========================================
// 2. CAMERA PIN DEFINITIONS (AI-THINKER)
// ==========================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ==========================================
// 3. ROBOT PIN DEFINITIONS
// ==========================================
const int PIN_SERVO_PAN       = 2;  
const int ECHO_PIN            = 3;  // RX Pin (DISCONNECT WHEN UPLOADING)
const int TRIG_PIN            = 4;  // Shared with onboard LED
const int PINDC_RIGHT_BACK    = 12; // IN4
const int PINDC_LEFT_BACK     = 13; // IN2
const int PINDC_RIGHT_FORWARD = 14; // IN3
const int PINDC_LEFT_FORWARD  = 15; // IN1

const int LEFT_CHANNEL        = 2;
const int RIGHT_CHANNEL       = 3;
const int SERVO_PAN_CHANNEL   = 4;
const int SERVO_RESOLUTION    = 16;
unsigned long previousMillisServo = 0;
const unsigned long intervalServo = 10;

// ==========================================
// 4. GLOBAL VARIABLES & STATES
// ==========================================
int cameraInitState = -1;
uint8_t* jpgBuff = new uint8_t[68123];
size_t   jpgLength = 0;
uint8_t camNo = 0;
bool clientConnected = false;

// Servo manual sweep
bool servoRotateLeft = false;
bool servoRotateRight = false;
int posServo = 90; // Center

// AI Navigation Variables
enum AutoState { AUTO_FORWARD, AUTO_SCAN_LEFT, AUTO_SCAN_RIGHT, AUTO_TURN, AUTO_RECENTER };
bool isAutonomous = false;
AutoState robotState = AUTO_FORWARD;
unsigned long autoTimer = 0;
int distLeft = 0;
int distRight = 0;
const int DIST_THRESHOLD = 25; // cm

// --- NEW: Global Speed Control ---
int currentSpeedPWM = 190; // Defaults to slider level 3

// Servers
WebSocketsServer webSocket = WebSocketsServer(86);
WiFiUDP UDPServer;
const int UDPPort = 6868;
byte packetBuffer[16];

// ==========================================
// 5. CAMERA FUNCTIONS
// ==========================================
int initCamera(){
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
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return -1;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  s->set_framesize(s, FRAMESIZE_VGA);
  return 0;
}

esp_err_t grabImage(size_t& jpg_buf_len, uint8_t *jpg_buf){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  fb = esp_camera_fb_get();
  uint8_t *jpg_buf_tmp = NULL;
  
  if (!fb) {
      res = ESP_FAIL;
  } else {
    if(fb->format != PIXFORMAT_JPEG){
        bool jpeg_converted = frame2jpg(fb, 80, &jpg_buf_tmp, &jpg_buf_len);
        memcpy(jpg_buf, jpg_buf_tmp, jpg_buf_len);
        fb = NULL;
        if(!jpeg_converted) res = ESP_FAIL;
    } else {
        jpg_buf_len = fb->len;
        memcpy(jpg_buf, fb->buf, jpg_buf_len);
    }
    esp_camera_fb_return(fb);
  }
  return res;
}

// ==========================================
// 6. ROBOT MOTOR & SENSOR FUNCTIONS
// ==========================================
void servoWrite(uint8_t channel, uint8_t angle) {
  uint32_t maxDuty = (pow(2,SERVO_RESOLUTION)-1)/10; 
  uint32_t minDuty = (pow(2,SERVO_RESOLUTION)-1)/20; 
  uint32_t duty = (maxDuty-minDuty)*angle/180 + minDuty;
  ledcWrite(channel, duty);
}

void controlServoManual(){
  if(servoRotateLeft && posServo < 180) posServo += 2;
  if(servoRotateRight && posServo > 0) posServo -= 2;
  servoWrite(SERVO_PAN_CHANNEL, posServo);
}

void controlDC(int leftDir, int leftPWM_flag, int rightDir, int rightPWM_flag){
  digitalWrite(PINDC_LEFT_BACK, leftDir);
  // Apply dynamic speed instead of max 255
  ledcWrite(LEFT_CHANNEL, leftPWM_flag == HIGH ? currentSpeedPWM : 0);
  
  digitalWrite(PINDC_RIGHT_BACK, rightDir);
  // Apply dynamic speed instead of max 255
  ledcWrite(RIGHT_CHANNEL, rightPWM_flag == HIGH ? currentSpeedPWM : 0);
}

int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

// ==========================================
// 7. AUTONOMOUS LOGIC
// ==========================================
void processAutonomous() {
  if (!isAutonomous) return;
  unsigned long currentMillis = millis();
  
  switch(robotState) {
    case AUTO_FORWARD:
      controlDC(LOW, HIGH, LOW, HIGH); 
      if (currentMillis - autoTimer > 100) {
        autoTimer = currentMillis;
        if (getDistance() < DIST_THRESHOLD) {
          controlDC(LOW, LOW, LOW, LOW); // Stop
          robotState = AUTO_SCAN_LEFT;
          autoTimer = currentMillis;
        }
      }
      break;
      
    case AUTO_SCAN_LEFT:
      servoWrite(SERVO_PAN_CHANNEL, 150); 
      if (currentMillis - autoTimer > 500) {
        distLeft = getDistance();
        robotState = AUTO_SCAN_RIGHT;
        autoTimer = currentMillis;
      }
      break;
      
    case AUTO_SCAN_RIGHT:
      servoWrite(SERVO_PAN_CHANNEL, 30); 
      if (currentMillis - autoTimer > 500) {
        distRight = getDistance();
        robotState = AUTO_TURN;
        autoTimer = currentMillis;
      }
      break;
      
    case AUTO_TURN:
      if (distLeft > distRight) {
        controlDC(LOW, LOW, LOW, HIGH); 
      } else {
        controlDC(LOW, HIGH, LOW, LOW); 
      }
      if (currentMillis - autoTimer > 600) { 
        controlDC(LOW, LOW, LOW, LOW);
        robotState = AUTO_RECENTER;
        autoTimer = currentMillis;
      }
      break;
      
    case AUTO_RECENTER:
      servoWrite(SERVO_PAN_CHANNEL, 90); 
      if (currentMillis - autoTimer > 500) {
        robotState = AUTO_FORWARD; 
      }
      break;
  }
}

// ==========================================
// 8. WEBSOCKET COMMAND ROUTER
// ==========================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
      case WStype_DISCONNECTED:
          camNo = num;
          clientConnected = false;
          controlDC(LOW,LOW,LOW,LOW); 
          break;
      case WStype_CONNECTED:
          clientConnected = true;
          break;
      case WStype_TEXT:
      {
          String cmd = String((char*)payload);
          
          // --- NEW: Catch speed commands from HTML slider ---
          if(cmd.startsWith("speed:")){
              int speedLevel = cmd.substring(6).toInt();
              // Map slider (1-5) to PWM duty cycle (130-255)
              if (speedLevel <= 1) currentSpeedPWM = 130;
              else if (speedLevel == 2) currentSpeedPWM = 160;
              else if (speedLevel == 3) currentSpeedPWM = 190;
              else if (speedLevel == 4) currentSpeedPWM = 220;
              else if (speedLevel >= 5) currentSpeedPWM = 255;
              
          }else if(cmd.equals("auto")){
              isAutonomous = true;
              robotState = AUTO_FORWARD;
              servoWrite(SERVO_PAN_CHANNEL, 90);
          }else if(cmd.equals("manual")){
              isAutonomous = false;
              controlDC(LOW,LOW,LOW,LOW); 
              servoWrite(SERVO_PAN_CHANNEL, 90);
          }else if(!isAutonomous) {
              if(cmd.equals("forward")) controlDC(LOW,HIGH,LOW,HIGH);
              else if(cmd.equals("backward")) controlDC(HIGH,LOW,HIGH,LOW);
              else if(cmd.equals("left")) controlDC(LOW,LOW,LOW,HIGH);
              else if(cmd.equals("right")) controlDC(LOW,HIGH,LOW,LOW);
              else if(cmd.equals("stop")) controlDC(LOW,LOW,LOW,LOW);
              else if(cmd.equals("camleft")) servoRotateLeft = true;
              else if(cmd.equals("camright")) servoRotateRight = true;
              else if(cmd.equals("camstill")) { servoRotateLeft = false; servoRotateRight = false; }
              else if(cmd.equals("camcenter")) { posServo = 90; servoWrite(SERVO_PAN_CHANNEL, posServo); }
          }
          break;
      }
  }
}

void processUDPData(){
  if (UDPServer.parsePacket()) {
      UDPServer.read(packetBuffer, 16);
      if(String((const char*)packetBuffer).equals("whoami")){
          UDPServer.beginPacket(UDPServer.remoteIP(), UDPServer.remotePort()-1);
          String res = "ESP32-CAM";
          UDPServer.write((const uint8_t*)res.c_str(),res.length());
          UDPServer.endPacket();
      }
      memset(packetBuffer, 0, 16);
  }
}

// ==========================================
// 9. SETUP & MAIN LOOP
// ==========================================
void setup(void) {
  Serial.begin(115200);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(PINDC_LEFT_BACK, OUTPUT);
  ledcSetup(LEFT_CHANNEL, 100, 8);
  ledcAttachPin(PINDC_LEFT_FORWARD, LEFT_CHANNEL);
  
  pinMode(PINDC_RIGHT_BACK, OUTPUT);
  ledcSetup(RIGHT_CHANNEL, 100, 8);
  ledcAttachPin(PINDC_RIGHT_FORWARD, RIGHT_CHANNEL);

  ledcSetup(SERVO_PAN_CHANNEL, 50, 16);
  ledcAttachPin(PIN_SERVO_PAN, SERVO_PAN_CHANNEL);
  servoWrite(SERVO_PAN_CHANNEL, posServo);
  controlDC(LOW,LOW,LOW,LOW);

  cameraInitState = initCamera();
  if(cameraInitState != 0) {
    Serial.println("Camera Init Failed. Check Power/Pins.");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  UDPServer.begin(UDPPort); 
}

void loop(void) {
  webSocket.loop();
  
  if(clientConnected == true){
    grabImage(jpgLength, jpgBuff);
    webSocket.sendBIN(camNo, jpgBuff, jpgLength);
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisServo >= intervalServo) {
    previousMillisServo = currentMillis;
    processUDPData();
    if(!isAutonomous) controlServoManual();
  }

  processAutonomous();
}
