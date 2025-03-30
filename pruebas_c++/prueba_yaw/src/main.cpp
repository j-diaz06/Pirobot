#include <Arduino.h>
#include <Wire.h>

#define GYRO_FULL_SCALE_250_DPS  0x00
#define GYRO_FULL_SCALE_500_DPS  0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define MPU 0x68           // Dirección I2C de la IMU
//#define A_R 16384.0        // 32768/2 (escala del acelerómetro)
//#define G_R 131 //250
//#define G_R 65.536 //500        // 32768/500 (corregido para ±500 DPS)
#define G_R 32.8 //1000
//#define G_R 16.4 //2000
//#define RAD_A_DEG 57.295779 // Conversión radianes a grados

#define MUESTRAS_CALIBRACION 500  // Reducido para calibración periódica

// Variables para calibración
float promedio = 0;       // Bias del giroscopio Z
unsigned long lastCalibration = 0;
const unsigned long intervaloCalibracion = 60000; // 60 segundos

// Valores RAW
int16_t GyZ;

// Ángulos y variables de tiempo
//float Gy[3];
float yaw;
long tiempo_prev = 0;
//float dt = 0;

// Lee el valor RAW del giroscopio Z (sin escalar)
int16_t leerRawGyroZ() {
  Wire.beginTransmission(MPU);
  Wire.write(0x47);         // Registro del giroscopio Z
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2);
  return Wire.read() << 8 | Wire.read();
}

// Calibra el giroscopio tomando muestras en reposo
void calibrateGyro() {
  long suma = 0;
  for (int i = 0; i < MUESTRAS_CALIBRACION; i++) {
    suma += leerRawGyroZ();
    delay(1);
  }
  promedio = suma / (float)MUESTRAS_CALIBRACION; // hace un promedio
  Serial.print("Nuevo bias: "); Serial.println(promedio);
}

void leerGiroscopio() {
  // Lee y corrige el bias
  GyZ = leerRawGyroZ() - promedio;
  GyZ = GyZ / G_R;       // Convierte a °/s con escala correcta
}

void calcularYaw() {
  float dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();
  
  // Integración para obtener el ángulo YAW
  yaw += GyZ * dt;  // Yaw = ∫ velocidad angular
}

void imprimirAngulo() {
  Serial.print("YAW: ");
  Serial.println(yaw, 2); // 2 decimales
}

void setup() {
  Wire.begin();             // Inicia I2C (pines D2 SDA, D1 SCL)
  Serial.begin(115200);
  
  // Configuración inicial del MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);            // Pone el MPU6050 en modo operativo
  Wire.endTransmission(true);

  // Configura rango del giroscopio a ±500 °/s
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(GYRO_FULL_SCALE_500_DPS);
  Wire.endTransmission(true);

  calibrateGyro(); // Calibración inicial
  lastCalibration = millis();
  tiempo_prev = millis();
}

void loop() {
  if (millis() - lastCalibration >= intervaloCalibracion) {
    calibrateGyro();
    tiempo_prev = millis();  // Reinicia contador de tiempo
    lastCalibration = millis();
  }
  
  leerGiroscopio();
  calcularYaw();
  imprimirAngulo();
  delay(10);                // Ajusta según necesidad
}

