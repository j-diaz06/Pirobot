#include "sensor.h"
#include <Arduino.h>

Adafruit_VL53L0X lox;

void sensor::iniciar(){
  if (!lox.begin()) {
        Serial.println("¡Error! No se pudo encontrar el VL53L0X.");
        return;
    }
  Serial.println("VL53L0X inicializado correctamente.");
  }

void sensor::medir(){
    VL53L0X_RangingMeasurementData_t measure;

    // Obtener la medición de distancia
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) { // Estado 4 significa fuera de rango
        Serial.println("Distancia: " + String(measure.RangeMilliMeter) + " mm");
        return;
    }
    Serial.println("Fuera de rango");
  }
