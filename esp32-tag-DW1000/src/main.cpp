#include <Arduino.h>
#include "rtls.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  xTaskCreatePinnedToCore(RTLS_Task, "RTLS_Task", 8192, NULL, 1, NULL, 1);
}

void loop() {
  // put your main code here, to run repeatedly:
}