#include <Arduino.h>
#include "rtls.h"

void setup()
{
    Serial.begin(115200);

    /* Create FreeRTOS Tasks Begin */
    xTaskCreatePinnedToCore(RTLS_Task, "RTLS_Task", 4096, NULL, 1, NULL, 1);
}

void loop()
{
    
}
