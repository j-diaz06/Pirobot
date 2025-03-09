#include <Arduino.h>
#include "giro_acel.h"
#include <Wire.h>



// Constructor
Giro_acel::Giro_acel() {
    // Configurar el pin AD0 del MPU6050
    //pinMode(MPU_AD0_PIN, OUTPUT);
    //digitalWrite(MPU_AD0_PIN, LOW);  // AD0 en LOW (dirección I2C: 0x68)

    yawMutex = xSemaphoreCreateMutex(); // Crear el mutex
    if (yawMutex == NULL) {
        Serial.println("Error: No se pudo crear el mutex");
    }
}

/*
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
    Serial.printf("acelerometro=%d %d %d ",ax,ay,az);
    
    // Calcular ángulo del acelerómetro
    accelAngleYaw = atan2(ay, az) * 180/M_PI;

    // Leer giroscopio
    float gyroZ = mpu.getRotationZ() / 131.0; // 131 LSB/°/s
    Serial.printf("giroscopio=%.4f ",gyroZ);
    
    // Filtro complementario
    yaw = alpha * (yaw + gyroZ * dt) + (1 - alpha) * accelAngleYaw;

    Serial.printf("Yaw=%.4f\n ",yaw);
    
    // Mantener yaw en rango 0-360
    if(yaw < 0) yaw += 360;
    if(yaw >= 360) yaw -= 360;
}

float Giro_acel::obtenerYaw() {
    return yaw;
}*/

int16_t Giro_acel::leerRawGyroZ() {
    Wire.beginTransmission(MPU);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2);
    return Wire.read() << 8 | Wire.read();
  }
  
  void Giro_acel::calibrateGyro() {
    long suma = 0;
    for (int i = 0; i < MUESTRAS_CALIBRACION; i++) {
      suma += leerRawGyroZ();
      delay(1);
    }
    promedio = suma / (float)MUESTRAS_CALIBRACION;
    Serial.print("Bias calibrado: ");
    Serial.println(promedio);
  }
  
  int16_t Giro_acel::leerGiroscopio() {
    GyZ = leerRawGyroZ() - promedio;
    GyZ = GyZ / G_R; // Convertir a °/s
    return GyZ;
  }

  float Giro_acel::getYaw() {
    if (xSemaphoreTake(yawMutex, portMAX_DELAY) == pdTRUE) {
      float currentYaw = yaw;
      xSemaphoreGive(yawMutex);
      return currentYaw;
  }
  return 0.0; // Valor por defecto en caso de error
}
  
  void Giro_acel::calcularYaw(int16_t giroscopio) {
    if (xSemaphoreTake(yawMutex, portMAX_DELAY) == pdTRUE) {
      float dt = (millis() - tiempo_prev) / 1000.0;
      tiempo_prev = millis();
      yaw += giroscopio * dt;
      xSemaphoreGive(yawMutex);
    }
  }
  
  void Giro_acel::iniciarMPU() {
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0); // Inicializar MPU
    Wire.endTransmission(true);
  
    Wire.beginTransmission(MPU);
    Wire.write(0x1B);
    Wire.write(GYRO_FULL_SCALE_500_DPS); // Configurar escala
    Wire.endTransmission(true);
  
    calibrateGyro();
    tiempo_prev = millis();
    lastCalibration = millis();
  }