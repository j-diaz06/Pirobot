#ifndef MAQUINA_H
#define MAQUINA_H

#include <Arduino.h>
#include "motor.h"
#include "sensor.h"
#include "giro_acel.h"

// Estados del sistema (usando #define)
#define ESTADO_ADELANTE 0
#define ESTADO_DERECHA 1
#define ESTADO_IZQUIERDA 2
#define ESTADO_DETENIDO 3
#define ESTADO_ESPERA_ADELANTE 4
#define ESTADO_ESPERA_DERECHA 5

// Direcciones de giro (usando #define)
#define GIRO_NINGUNO 0
#define GIRO_DERECHA 1

class Maquina {
private:
    // Variables de control
    int estado_actual;

    bool aceleracion_adelante; // true = adelante, false = atrás

    Sensor sensor_trasero;
    Sensor sensor_delantero;

    short DISTANCIA_DETECCION;

    float yawActual;
    float yawObjetivo=-90;
    bool girando;
    //Giro_acel& giro;

    unsigned long conteo_espera;
    short tiempo_espera;
    

    unsigned long intervaloCalibracion;

    unsigned long lastMPURead = 0;
    unsigned long lastCalCheck = 0;

    int distancia_trasero=0;
    int distancia_delantero=0;

    // Funciones de control de motores
    void mover_adelante();
    void girar_derecha();
    void detener();

    // Lectura de sensores
    bool obstaculo_enfrente;
    bool obstaculo_atras;

    //Sensores s;
    void leer_sensores();
    
    // Lógica de giro
    bool puede_girar(int direccion);

    // Máquina de estados
    void maquina_estados();

    // Ejecutar acción según estado
    void ejecutar_accion();

public:
    // Constructor
    Maquina(int ENA, int IN1, int IN2, int ENB, int IN3, int IN4,
        uint8_t direccion_trasero, uint8_t xshut_trasero,
        uint8_t direccion_delantero, uint8_t xshut_delantero, short DISTANCIA_MAXIMA,
        bool direccion_inicial, const unsigned long IntervaloCalibracion, short TIEMPO_ESPERA);

    Sensor& getSensorDelantero() { return sensor_delantero; }
    Sensor& getSensorTrasero() { return sensor_trasero; }
    float getYaw() { return yawActual; }

    Motor motores;

    Giro_acel giro_acel;  // Objeto MPU6050

    //Sensores leer_sensores(int trasero,int delantero,short DISTANCIA_DETECCION);

    // Inicialización
    void iniciar();

    // Bucle principal
    void ejecutar();

    //void actualizarOrientacion();

    void acelerarAdelante(bool nuevaDireccion);
};

#endif