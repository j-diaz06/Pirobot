#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>  // Biblioteca para MPU6050

class Giro_acel{
private:
    MPU6050 mpu;
    float dt;
    float yaw = 0.0;
    float accelAngleYaw = 0.0;
    float alpha = 0.98; // Factor de filtro complementario
    unsigned long lastUpdate = 0;
public:
    // Constructor
    Giro_acel(int MPU_AD0_PIN);

    // Inicializar el MPU6050
    void iniciar();

    void actualizarFiltro();
    float obtenerYaw();
    void calibrarGiroscopio(int muestras = 200);
};

#endif