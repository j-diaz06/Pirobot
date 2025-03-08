#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"  // Biblioteca de Adafruit

class Sensor {
private:
    uint8_t pinXSHUT;
    uint8_t direccion;
    Adafruit_VL53L0X lox;  // Objeto de Adafruit

public:
    Sensor(uint8_t direccion, uint8_t xshut);
    bool iniciar();
    int medir();
};

#endif