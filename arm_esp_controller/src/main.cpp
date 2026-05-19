// #include <Arduino.h>

// // put function declarations here:
// int myFunction(int, int);

// void setup() {
//   // put your setup code here, to run once:
//   int result = myFunction(2, 3);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
// }

// // put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }

#include <Arduino.h>

float base_joint_lim[2] = {-3.14159, +3.14159};
float shoulder_joint_lim[2] = {-1.57, +1.57};
float elbow_joint_lim[2] = {-2.094, +2.094};

float pos[3] = {0.0f, 0.0f, 0.0f};

float cmd[3] = {0.0f, 0.0f, 0.0f};

float clamp(float val, float low, float high){
  if (val > high) return high;
  if (val < low) return low;
  return val;
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
}

void loop() {
  // bool got_cmd = false;
  // // put your main code here, to run repeatedly:
  // digitalWrite(LED_PIN, LOW);
  // if (Serial.available()){
  //   String line = Serial.readStringUntil('\n');
  //   line.trim();
  //   // Serial.println("RECV: " + line); // Just for debugging 

  //   if (line.startsWith("CMD")){
  //     // Parse: "CMD a0 a1 a2"
  //     float a0, a1, a2;
  //     int parsed = sscanf(line.c_str(), "CMD %f %f %f", &a0, &a1, &a2);

  //     if (parsed == 3){
  //       cmd[0] = clamp(a0, base_joint_lim[0], base_joint_lim[1]);
  //       cmd[1] = clamp(a1, shoulder_joint_lim[0], shoulder_joint_lim[1]);
  //       cmd[2] = clamp(a2, elbow_joint_lim[0], elbow_joint_lim[1]);

  //       pos[0] = cmd[0];
  //       pos[1] = cmd[1];
  //       pos[2] = cmd[2];
        
  //       got_cmd = true;
  //     }
  //   }
  // }

  // // send data
  // // if (got_cmd){
  //   char buf[64];
  //   snprintf(buf, sizeof(buf), "STATE %.6f %.6f %.6f\n",
  //           pos[0], pos[1], pos[2]);
  //   Serial.print(buf);
  // // }

  // delay(10);  // 100 Hz


  while (Serial.available()){
    char c = Serial.read();
    if (c == '\n'){
      inputBuffer.trim();
      if (inputBuffer.startsWith("CMD")){
        float a0, a1, a2;
        if (sscanf(inputBuffer.c_str(), "CMD %f %f %f", &a0, &a1, &a2) == 3){
          pos[0] = clamp(a0, base_joint_lim[0], base_joint_lim[1]);
          pos[1] = clamp(a1, shoulder_joint_lim[0], shoulder_joint_lim[1]);
          pos[2] = clamp(a2, elbow_joint_lim[0], elbow_joint_lim[1]);
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