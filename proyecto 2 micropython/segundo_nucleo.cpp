#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
//#include "maquina.h"
#include "segundo_nucleo.h"
#include <Arduino.h>

Segundo_nucleo::Segundo_nucleo()
{
    
}

void Segundo_nucleo::yawTask(void *pvParameters) {
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

void Segundo_nucleo::iniciar(Maquina& maquina) {
    Serial.println("Iniciando segundo núcleo...");
    maquinaPtr = &maquina;  // Guardar el puntero a Maquina
    xTaskCreatePinnedToCore(
        Segundo_nucleo::yawTask, // Función a ejecutar
        "yawTask", // Nombre de la tarea
        10000, // Tamaño de la pila
        (void*)maquinaPtr, // Parámetro
        1, // Prioridad
        NULL, // Handle
        1); // Núcleo
    Serial.println("Segundo núcleo iniciado");
}
