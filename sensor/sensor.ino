#include "sensor.h"
#include <Wire.h>

sensor sensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  sensor.iniciar();

}

void loop() {
sensor.medir();
delay(500);
}
