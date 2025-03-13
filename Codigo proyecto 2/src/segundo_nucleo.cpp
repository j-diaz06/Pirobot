#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "maquina.h"
#include "segundo_nucleo.h"
#include <Arduino.h>

segundo_nucleo::segundo_nucleo()
{
    xTaskCreatePinnedToCore(
        segundo_nucleo::yawTask, // Función a ejecutar
        "yawTask", // Nombre de la tarea
        10000, // Tamaño de la pila
        (void*)&maquina, // Parámetro
        1, // Prioridad
        NULL, // Handle
        1); // Núcleo
}


