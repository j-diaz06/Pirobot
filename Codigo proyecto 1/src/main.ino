#include <Arduino.h>
#include "maquina.h"
#include "webcontrol.h"
//#include "segundo_nucleo.h" // Agregar esta línea
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define ENA 25
#define IN1 27
#define IN2 32
#define IN3 14
#define IN4 33
#define ENB 26

// Pines I2C para MPU6050 (21 y 22 por defecto)
#define SDA_PIN 21
#define SCL_PIN 22

#define XSHUT_SENSOR_DELANTERO 5
#define DIRECCION_SENSOR_DELANTERO 0x30
#define XSHUT_SENSOR_TRASERO 4
#define DIRECCION_SENSOR_TRASERO 0x35

#define DISTANCIA_DETECCION 150

#define TIEMPO_CAMBIO_MARCHA 1500 // Tiempo de detención configurable

bool aceleracion_adelante = true; // Valor inicial

#define intervaloCalibracion_yaw 30000

#define ssid "Redmis"
#define pass "asdfg123"
#define host "esp32"

Maquina maquina(ENA, IN1, IN2, ENB, IN3, IN4,
    DIRECCION_SENSOR_TRASERO, XSHUT_SENSOR_TRASERO,
    DIRECCION_SENSOR_DELANTERO, XSHUT_SENSOR_DELANTERO, DISTANCIA_DETECCION,
    aceleracion_adelante, intervaloCalibracion_yaw, TIEMPO_CAMBIO_MARCHA);

WebControl webControl(maquina.motores, maquina); // Crear instancia de WebControl

void yawTask(void *pvParameters) {
    // Convertir el parámetro al tipo Maquina*
    Maquina* maquina = (Maquina*)pvParameters;
    while(true) {
        // Leer el giroscopio y calcular el Yaw
        int16_t gyroZ = maquina->giro_acel.leerGiroscopio();
        maquina->giro_acel.calcularYaw(gyroZ);
        // Esperar 1 ms
        vTaskDelay(1 / portTICK_PERIOD_MS); // Mínima espera
    }
}

void setup() {
    Serial.begin(115200);

    // Iniciar I2C con los pines definidos
    Wire.begin(SDA_PIN, SCL_PIN);
    
    maquina.acelerarAdelante(true);// por ahora solo acelerara hacia adelante

    webControl.begin(ssid, pass, host);

    maquina.iniciar();

    xTaskCreatePinnedToCore(
        yawTask, // Función a ejecutar
        "yawTask", // Nombre de la tarea
        10000, // Tamaño de la pila
        (void*)&maquina, // Parámetro
        1, // Prioridad
        NULL, // Handle
        1); // Núcleo
}

void loop() {
    maquina.loop();
    webControl.handleClient(); // Manejar clientes web
}