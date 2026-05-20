#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>


//----------------------Joints----------------------

// Home position --> 150 degree, 0 rad 
float base_joint_lim[2] = {-2.616, +2.616}; // New limits 
// Home position --> 150 degree, 0 rad
float shoulder_joint_lim[2] = {-1.57, +1.57};
// Home position --> 150 degree, 0 rad
float elbow_joint_lim[2] = {-2.094, +2.094};

float pos[3] = {0.0f, 0.0f, 0.0f};

//----------------------Servos----------------------
// Servos --> ASME-05B, 380kg.cm, 0-300 or +-150° limit
Servo base_servo;
Servo shoulder_servo;
Servo elbow_servo;
#define SERVO0_PIN 18
#define SERVO1_PIN 19
#define SERVO2_PIN 21

// Pulses in microseconds --> 500 for 0°, 2500 for 300°
#define PULSE_MIN 500   // = 0°
#define PULSE_MAX 2500  // = 300°

//-----------------------Wifi-----------------------

#define USE_WIFI 1 // Upload as 1 if using wifi, 0 if not

#if USE_WIFI
  // Wifi Credentials
  const char* WIFI_SSID     = "pi";
  const char* WIFI_PASSWORD = "12345678";
  const int   TCP_PORT      = 8888;
  WiFiServer wifiServer(TCP_PORT);
  WiFiClient wifiClient;
#endif

//--------------------Transport--------------------

int transportAvailable() {
// Check connection of either transports
#if USE_WIFI
  if (!wifiClient || !wifiClient.connected()) {
    wifiClient = wifiServer.available();
  }
  return (wifiClient && wifiClient.connected()) ? wifiClient.available() : 0;
#else
  return Serial.available();
#endif
}

char transportRead() {
//Read using either transports
#if USE_WIFI
  return (char)wifiClient.read();
#else
  return (char)Serial.read();
#endif
}

void transportPrint(const char* buf) {
// Print using either transports
#if USE_WIFI
  if (wifiClient && wifiClient.connected()){
    wifiClient.print(buf);
  }
#else
  Serial.print(buf);
#endif
}

//----------------------Helpers----------------------

float clamp(float val, float low, float high){
  // Clamp angles to prevent exceeding the joints' limits
  if (val > high) return high;
  if (val < low) return low;
  return val;
}

void writeServo(int id, float angle){
  /* 
  Takes servo id and angles in rad
  Converts angles to degree and map the angles to signals
  Uses servo.writeMicroseconds() in order to exceed the 180° limited by servo.write()
  */
  float angle_degree = angle*(180.0f/PI);
  int sig;
  sig = int(map(angle_degree, -150, 150, PULSE_MIN, PULSE_MAX));
  if (id == 0){
    base_servo.writeMicroseconds(sig);
  }
  else if(id == 1){
    shoulder_servo.writeMicroseconds(sig);
  }
  else if(id == 2){
    elbow_servo.writeMicroseconds(sig);
  }
}

bool got_cmd = false;
String inputBuffer = "";
unsigned long lastSendTime = 0;

#define LED_PIN 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(500);
  pinMode(LED_PIN, OUTPUT);

  base_servo.attach(SERVO0_PIN, PULSE_MIN, PULSE_MAX);
  shoulder_servo.attach(SERVO1_PIN, PULSE_MIN, PULSE_MAX);
  elbow_servo.attach(SERVO2_PIN, PULSE_MIN, PULSE_MAX);

#if USE_WIFI
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // blink while connecting
  }
  digitalWrite(LED_PIN, HIGH);
  Serial.print("\nWiFi connected. IP: ");
  Serial.println(WiFi.localIP());
  wifiServer.begin();
  Serial.printf("TCP server listening on port %d\n", TCP_PORT);
#endif
}

void loop() {
  while (transportAvailable()){
    char c = transportRead();
    if (c == '\n'){
      inputBuffer.trim();
      if (inputBuffer.startsWith("CMD")){
        float a0, a1, a2;
        if (sscanf(inputBuffer.c_str(), "CMD %f %f %f", &a0, &a1, &a2) == 3){
          pos[0] = clamp(a0, base_joint_lim[0], base_joint_lim[1]);
          pos[1] = clamp(a1, shoulder_joint_lim[0], shoulder_joint_lim[1]);
          pos[2] = clamp(a2, elbow_joint_lim[0], elbow_joint_lim[1]);
          
          writeServo(0, pos[0]);
          writeServo(1, pos[1]);
          writeServo(2, pos[2]);

          got_cmd = true;
        }
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  unsigned long now = millis();
  if (got_cmd && (now - lastSendTime > 10)){ // Match the ROS2 node 100Hz
    // send data
    char buf[64];
    snprintf(buf, sizeof(buf), "STATE %.4f %.4f %.4f\n", pos[0], pos[1], pos[2]);
    transportPrint(buf);
    lastSendTime = now;
  }
}