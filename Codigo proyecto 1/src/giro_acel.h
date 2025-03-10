#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define GYRO_FULL_SCALE_250_DPS  0x00
#define GYRO_FULL_SCALE_500_DPS  0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

//#define G_R 131 //250
#define G_R 65.536 //500        // 32768/500 (corregido para ±500 DPS)
//#define G_R 32.8 //1000
//#define G_R 16.4 //2000

#define MPU 0x68           // Dirección I2C de la IMU

#define MUESTRAS_CALIBRACION 500  // Reducido para calibración periódica

class Giro_acel{
private:
    float promedio = 0;
    unsigned long lastCalibration = 0;
    int16_t GyZ;
    long tiempo_prev = 0;
    SemaphoreHandle_t yawMutex;
    volatile float yaw; // Variable atómica para lectura/escritura
    float currentYaw;
public:
    // Constructor
    Giro_acel();

    float getYaw();

    int16_t leerRawGyroZ();
    void calibrateGyro();
    int16_t leerGiroscopio();
    void calcularYaw(int16_t giroscopio);
    void iniciarMPU();
};

#endif