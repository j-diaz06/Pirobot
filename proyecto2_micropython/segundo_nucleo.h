#ifndef SEGUNDO_NUCLEO_H
#define SEGUNDO_NUCLEO_H

#include "maquina.h"

class Segundo_nucleo
{
private:
    Maquina* maquinaPtr;
public:
    Segundo_nucleo(/* args */);
    static void yawTask(void *pvParameters);
    void iniciar(Maquina& maquina);
};

#endif