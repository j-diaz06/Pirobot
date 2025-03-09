#include <Arduino.h>
#include <Wire.h>
#include "sensor.h"

Sensor::Sensor(uint8_t direccion, uint8_t xshut) {
    this->direccion = direccion;
    pinXSHUT = xshut;
    pinMode(pinXSHUT, OUTPUT);
}

bool Sensor::iniciar() {
    // Reiniciar sensor mediante XSHUT
    digitalWrite(pinXSHUT, LOW);
    delay(10);
    digitalWrite(pinXSHUT, HIGH);
    delay(10);

    Serial.printf("[Sensor] Inicializando en dirección 0x%02X...\n", direccion);

    // Inicializar sensor con dirección personalizada
    if (!lox.begin(direccion)) {  // Usar begin() con dirección
        Serial.println("[Sensor] Error: Fallo en begin()");
        return false;
    }

    // Configurar modo de alta velocidad (opcional)
    lox.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
    //lox.setMeasurementTimingBudgetMicroSeconds(20000);
    Serial.println("[Sensor] Configuración exitosa");
    
    return true;
}

int Sensor::medir() {
    VL53L0X_RangingMeasurementData_t medida;
    lox.rangingTest(&medida, false); // Medición con debug
    
    if (medida.RangeStatus != 4) { // 4 = Out of bounds
        Medida = medida.RangeMilliMeter;
        return Medida;
    }
    return -1;
}