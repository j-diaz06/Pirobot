#include <Arduino.h>
#include "sensor.h"
#include <Wire.h>

sensor misensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  misensor.iniciar();

}

void loop() {
misensor.medir();
delay(500);
}
