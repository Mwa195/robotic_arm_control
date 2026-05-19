#include <Arduino.h>
#include <ESP32Servo.h>

// Home position --> 150 degree, 0 rad 
float base_joint_lim[2] = {-2.616, +2.616}; // New limits 
// Home position --> 150 degree, 0 rad
float shoulder_joint_lim[2] = {-1.57, +1.57};
// Home position --> 150 degree, 0 rad
float elbow_joint_lim[2] = {-2.094, +2.094};

float pos[3] = {0.0f, 0.0f, 0.0f};

float cmd[3] = {0.0f, 0.0f, 0.0f};


// Servos
Servo myServo0;
Servo myServo1;
Servo myServo2;
#define SERVO0_PIN 18
#define SERVO1_PIN 19
#define SERVO2_PIN 21

#define PULSE_MIN 500   // = 0°
#define PULSE_MAX 2500  // = 300°

float clamp(float val, float low, float high){
  if (val > high) return high;
  if (val < low) return low;
  return val;
}

void writeServo(int id, float angle){
  float angle_degree = angle*(180.0f/PI);
  int sig;
  sig = int(map(angle_degree, -150, 150, PULSE_MIN, PULSE_MAX));
  if (id == 0){
    myServo0.writeMicroseconds(sig);
  }
  else if(id == 1){
    myServo1.writeMicroseconds(sig);
  }
  else if(id == 2){
    myServo2.writeMicroseconds(sig);
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

  myServo0.attach(SERVO0_PIN, PULSE_MIN, PULSE_MAX);
  myServo1.attach(SERVO1_PIN, PULSE_MIN, PULSE_MAX);
  myServo2.attach(SERVO2_PIN, PULSE_MIN, PULSE_MAX);
}

void loop() {
  while (Serial.available()){
    char c = Serial.read();
    if (c == '\n'){
      inputBuffer.trim();
      if (inputBuffer.startsWith("CMD")){
        float a0, a1, a2, a0_clamped, a1_clamped, a2_clamped;
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
  if (got_cmd && (now - lastSendTime > 10)){
    // send data
    char buf[64];
    snprintf(buf, sizeof(buf), "STATE %.4f %.4f %.4f\n", pos[0], pos[1], pos[2]);
    Serial.print(buf);
    lastSendTime = now;
  }

}