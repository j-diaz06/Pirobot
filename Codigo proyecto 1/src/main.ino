#include <Arduino.h>
//#include <Wire.h>
//#include <MPU6050.h>  // Biblioteca para MPU6050
//#include "motor.h"
//#include "sensor.h"
//#include "mpu_6050.h"
#include "maquina.h"

#define ENA 25
#define IN1 27
#define IN2 32
#define IN3 14
#define IN4 33
#define ENB 26

// Pines I2C para MPU6050 (21 y 22 por defecto)
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_AD0_PIN 15  // Pin D0 del ESP conectado a AD0 del MPU

#define XSHUT_SENSOR_DELANTERO 4
#define DIRECCION_SENSOR_DELANTERO 0x30
#define XSHUT_SENSOR_TRASERO 5
#define DIRECCION_SENSOR_TRASERO 0x35

#define DISTANCIA_DETECCION 200

Giro_acel giro_acel(MPU_AD0_PIN);  // Objeto MPU6050

bool aceleracion_adelante = true; // Valor inicial

//Motor motores(ENA, IN1, IN2, ENB, IN3, IN4);

//Sensor sensor_trasero(DIRECCION_SENSOR_DELANTERO, XSHUT_SENSOR_DELANTERO);  // Sensor trasero
//Sensor sensor_delantero(DIRECCION_SENSOR_TRASERO, XSHUT_SENSOR_TRASERO);  // Sensor delantero

Maquina maquina(ENA, IN1, IN2, ENB, IN3, IN4,
    DIRECCION_SENSOR_TRASERO, XSHUT_SENSOR_TRASERO,
    DIRECCION_SENSOR_DELANTERO, XSHUT_SENSOR_DELANTERO, DISTANCIA_DETECCION,
    giro_acel, aceleracion_adelante);

void setup() {
    Serial.begin(115200);

    // Iniciar I2C con los pines definidos
    Wire.begin(SDA_PIN, SCL_PIN);
/*
    // Configurar el pin AD0 del MPU6050
    pinMode(MPU_AD0_PIN, OUTPUT);
    digitalWrite(MPU_AD0_PIN, LOW);  // AD0 en LOW (dirección I2C: 0x68)
*/

    maquina.iniciar();

    giro_acel.iniciar();
    giro_acel.calibrarGiroscopio();
/*
    // Inicializar sensores VL53L0X
    if (sensor_trasero.iniciar()) {Serial.println("[Sensor] Trasero inicializado correctamente");}
        else{Serial.println("[Sensor] Error: Trasero no inicializado");}
    if (sensor_delantero.iniciar()) {Serial.println("[Sensor] delantero inicializado correctamente");}
        else{Serial.println("[Sensor] Error: delantero no inicializado");}
*/
    // Configurar motores
    //motores.configurar();
    
    maquina.acelerarAdelante(true);// por ahora solo acelerara hacia adelante

}

void loop() {
    static unsigned long lastMPURead = 0;
    
    // Actualizar MPU cada 50ms
    if(millis() - lastMPURead > 50) {
        giro_acel.actualizarFiltro();
        lastMPURead = millis();
    }
    
    maquina.ejecutar();
}