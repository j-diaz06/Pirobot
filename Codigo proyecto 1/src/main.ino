#include <Arduino.h>
//#include <Wire.h>
//#include <MPU6050.h>  // Biblioteca para MPU6050
//#include "motor.h"
//#include "sensor.h"
//#include "mpu_6050.h"
#include "maquina.h"
#include "webcontrol.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void yawTask(void *pvParameters) {
    Maquina* maquina = (Maquina*)pvParameters;
    while(true) {
        int16_t gyroZ = maquina->giro_acel.leerGiroscopio();
        maquina->giro_acel.calcularYaw(gyroZ);
        vTaskDelay(1 / portTICK_PERIOD_MS); // Mínima espera
    }
}

#define ENA 25
#define IN1 27
#define IN2 32
#define IN3 14
#define IN4 33
#define ENB 26

// Pines I2C para MPU6050 (21 y 22 por defecto)
#define SDA_PIN 21
#define SCL_PIN 22
//#define MPU_AD0_PIN 15

#define XSHUT_SENSOR_DELANTERO 5
#define DIRECCION_SENSOR_DELANTERO 0x30
#define XSHUT_SENSOR_TRASERO 4
#define DIRECCION_SENSOR_TRASERO 0x35

#define DISTANCIA_DETECCION 150

#define TIEMPO_CAMBIO_MARCHA 1500 // Tiempo de detención configurable

bool aceleracion_adelante = true; // Valor inicial

#define intervaloCalibracion_yaw 60000

#define ssid "Conectando..."
#define pass "casa2023"
#define host "esp32"

//Motor motores(ENA, IN1, IN2, ENB, IN3, IN4);

//Sensor sensor_trasero(DIRECCION_SENSOR_DELANTERO, XSHUT_SENSOR_DELANTERO);  // Sensor trasero
//Sensor sensor_delantero(DIRECCION_SENSOR_TRASERO, XSHUT_SENSOR_TRASERO);  // Sensor delantero

Maquina maquina(ENA, IN1, IN2, ENB, IN3, IN4,
    DIRECCION_SENSOR_TRASERO, XSHUT_SENSOR_TRASERO,
    DIRECCION_SENSOR_DELANTERO, XSHUT_SENSOR_DELANTERO, DISTANCIA_DETECCION,
    aceleracion_adelante, intervaloCalibracion_yaw, TIEMPO_CAMBIO_MARCHA);

WebControl webControl(maquina.motores, maquina); // Crear instancia de WebControl

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

    xTaskCreatePinnedToCore(
        yawTask,             // Función de la tarea
        "Yaw Processor",     // Nombre
        10000,               // Tamaño de stack
        &maquina,            // Parámetro (instancia de Maquina)
        1,                   // Prioridad
        NULL,                // Task handle
        1                    // Núcleo 1
    );

    //giro_acel.iniciar();
    //giro_acel.calibrarGiroscopio();
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

    webControl.begin(ssid, pass, host);

    delay(50);
}

void loop() {
    
    maquina.ejecutar();
    webControl.handleClient(); // Manejar clientes web
}