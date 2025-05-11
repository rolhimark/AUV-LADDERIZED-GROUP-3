#include <Arduino.h>
#include "camera.h"
#include "sonar.h"
#include "motor.h"
#include "wifi_server.h"

void setup() {
  Serial.begin(115200);
  initCamera();
  initSonar();
  initMotors();
  connectWiFi();
  startWebServer();
}

void loop() {
  float distance = readSonar();
  captureAndDetectObjects();

  if (distance < 30.0) {
    stopMotors();
  } else {
    moveForward();
  }

  sendDataToDashboard(distance);
  delay(200);
}
