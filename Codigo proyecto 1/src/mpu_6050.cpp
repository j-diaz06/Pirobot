#include <Arduino.h>
#include "mpu_6050.h"

// Constructor
Giro_acel::Giro_acel(int MPU_AD0_PIN) {
    // Configurar el pin AD0 del MPU6050
    pinMode(MPU_AD0_PIN, OUTPUT);
    digitalWrite(MPU_AD0_PIN, LOW);  // AD0 en LOW (dirección I2C: 0x68)
}

// Inicializar el MPU6050
void Giro_acel::iniciar() {
    Serial.println("Inicializando MPU6050...");
    mpu.initialize();

    // Verificar la conexión con el MPU6050
    if (mpu.testConnection()) {
        Serial.println("MPU6050 conectado correctamente.");
    } else {
        Serial.println("Error al conectar con el MPU6050.");
    }
}

void Giro_acel::calibrarGiroscopio(int muestras) {
    float sum = 0;
    for(int i=0; i<muestras; i++){
        sum += mpu.getRotationZ();
        delay(10);
    }
    mpu.setZGyroOffset(sum/muestras);
}

void Giro_acel::actualizarFiltro() {
    unsigned long now = micros();
    dt = (now - lastUpdate) / 1000000.0;
    lastUpdate = now;

    // Leer datos del acelerómetro
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    
    // Calcular ángulo del acelerómetro
    accelAngleYaw = atan2(ay, az) * 180/M_PI;

    // Leer giroscopio
    float gyroZ = mpu.getRotationZ() / 131.0; // 131 LSB/°/s
    
    // Filtro complementario
    yaw = alpha * (yaw + gyroZ * dt) + (1 - alpha) * accelAngleYaw;
    
    // Mantener yaw en rango 0-360
    if(yaw < 0) yaw += 360;
    if(yaw >= 360) yaw -= 360;
}

float Giro_acel::obtenerYaw() {
    return yaw;
}