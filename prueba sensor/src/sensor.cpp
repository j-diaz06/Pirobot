#include "sensor.h"
#include <Arduino.h>

<<<<<<< HEAD
Sensor::Sensor(uint8_t direccion, uint8_t xshut) {
    direccionI2C = direccion;
    pinXSHUT = xshut;
}

void Sensor::iniciar() {
    pinMode(pinXSHUT, OUTPUT);
    digitalWrite(pinXSHUT, LOW);  // Apagar sensor
    delay(10);
    digitalWrite(pinXSHUT, HIGH); // Encender sensor
    delay(10);

    if (!sensor.begin(direccionI2C)) {
      Serial.println("Error: No se encontró el sensor en la dirección 0x" + String(direccionI2C, HEX));
        return;
    }
    Serial.println("Sensor en dirección 0x" + String(direccionI2C, HEX) + " inicializado correctamente.");
}

int Sensor::medir() {
    VL53L0X_RangingMeasurementData_t medicion;
    sensor.rangingTest(&medicion, false);

    if (medicion.RangeStatus != 4) { // Estado 4 significa fuera de rango
        return medicion.RangeMilliMeter;
    }
    return -1;  // Retorna -1 si está fuera de rango
}

void Sensor::medirYEnviar() {
    int distancia = medir();
    Serial.print("Sensor en dirección 0x" + String(direccionI2C, HEX) + ": ");
    if (distancia != -1) {
      Serial.println(String(distancia) + " mm");
    } else {
        Serial.println("Fuera de rango");
    }
}
=======
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
>>>>>>> bab335048e1c06f2bd1e658b8a751e1b6eaf5ac7
