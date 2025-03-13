#include "maquina.h"

class segundo_nucleo
{
private:
    /* data */
public:
    segundo_nucleo(/* args */);
    static void yawTask(void *pvParameters);
    
};


void segundo_nucleo::yawTask(void *pvParameters) {
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
