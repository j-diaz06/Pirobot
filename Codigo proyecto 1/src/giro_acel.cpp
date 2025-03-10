#include <Arduino.h>
#include "giro_acel.h"
#include <Wire.h>

// Constructor
Giro_acel::Giro_acel()
{
  yawMutex = xSemaphoreCreateMutex(); // Crear el mutex
  if (yawMutex == NULL)
  {
    Serial.println("Error: No se pudo crear el mutex");
  }
}

int16_t Giro_acel::leerRawGyroZ()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2);
  return Wire.read() << 8 | Wire.read();
}

void Giro_acel::calibrateGyro()
{
  long suma = 0;
  for (int i = 0; i < MUESTRAS_CALIBRACION; i++)
  {
    suma += leerRawGyroZ();
    delay(1);
  }
  promedio = suma / (float)MUESTRAS_CALIBRACION;
  Serial.print("Bias calibrado: ");
  Serial.println(promedio);
}

int16_t Giro_acel::leerGiroscopio()
{
  GyZ = leerRawGyroZ() - promedio;
  GyZ = GyZ / G_R; // Convertir a °/s
  return GyZ;
}

float Giro_acel::getYaw()
{
  if (xSemaphoreTake(yawMutex, portMAX_DELAY) == pdTRUE)
  {
    currentYaw = yaw;
    xSemaphoreGive(yawMutex);
  }
  //Serial.printf("[ERROR] yaw no se pudo leer\n");
  return currentYaw;
}

void Giro_acel::calcularYaw(int16_t giroscopio)
{
  float dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();
  if (xSemaphoreTake(yawMutex, portMAX_DELAY) == pdTRUE)
  {
    yaw += giroscopio * dt;
    xSemaphoreGive(yawMutex);
  }
  Serial.printf("Yaw actualizado: %.2f\n", yaw);
}

void Giro_acel::iniciarMPU()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0); // Inicializar MPU
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(GYRO_FULL_SCALE_500_DPS); // Configurar escala
  Wire.endTransmission(true);

  // Configurar la frecuencia de muestreo a 1 kHz
  Wire.beginTransmission(MPU);
  Wire.write(0x19); // Dirección del registro de la frecuencia de muestreo
  Wire.write(0x00); // Configuración para 1 kHz (1 kHz / (1 + 0))
  Wire.endTransmission(true);

  calibrateGyro();
  tiempo_prev = millis();
  lastCalibration = millis();
}