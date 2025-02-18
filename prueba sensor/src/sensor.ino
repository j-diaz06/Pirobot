#include "sensor.h"
#include <Wire.h>

// Definir sensores con sus direcciones I2C y pines XSHUT
Sensor sensor1(0x30, 4);  // Sensor 1 en dirección 0x30, XSHUT en GPIO 4
Sensor sensor2(0x31, 5);  // Sensor 2 en dirección 0x31, XSHUT en GPIO 5

void setup() {
    Serial.begin(115200);
    Wire.begin();  // Iniciar comunicación I2C
    
    // Configurar cada sensor
    sensor1.iniciar();
    sensor2.iniciar();
}

void loop() {
    sensor1.medirYEnviar();
    sensor2.medirYEnviar();
    delay(500);  // Esperar 500ms antes de repetir el ciclo
}
