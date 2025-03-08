#ifndef MAQUINA_H
#define MAQUINA_H

#include <Arduino.h>
#include "motor.h"
#include "sensor.h"
#include "mpu_6050.h"

// Estados del sistema (usando #define)
#define ESTADO_ADELANTE 0
#define ESTADO_DERECHA 1
#define ESTADO_IZQUIERDA 2
#define ESTADO_BLOQUEO 3

// Direcciones de giro (usando #define)
#define GIRO_NINGUNO 0
#define GIRO_DERECHA 1

class Maquina {
private:
    // Variables de control
    int estado_actual;

    bool aceleracion_adelante; // true = adelante, false = atrás

    Motor motores;

    Sensor sensor_trasero;
    Sensor sensor_delantero;

    short DISTANCIA_DETECCION;

    float yawActual;
    float yawObjetivo;
    bool girando;
    Giro_acel& mpu;

    // Funciones de control de motores
    void mover_adelante();
    void girar_derecha();
    void detener();

    // Lectura de sensores
    struct Sensores {
        bool front;
        bool rear;
    };

    //Sensores s;
    Sensores leer_sensores();
    
    // Lógica de giro
    bool puede_girar(int direccion);

    // Máquina de estados
    void maquina_estados(bool front, bool rear);

    // Ejecutar acción según estado
    void ejecutar_accion();

public:
    // Constructor
    Maquina(int ENA, int IN1, int IN2, int ENB, int IN3, int IN4,
        uint8_t direccion_trasero, uint8_t xshut_trasero,
        uint8_t direccion_delantero, uint8_t xshut_delantero, short DISTANCIA_MAXIMA,
        Giro_acel& sensorMpu, bool direccion_inicial);

    //Sensores leer_sensores(int trasero,int delantero,short DISTANCIA_DETECCION);

    // Inicialización
    void iniciar();

    // Bucle principal
    void ejecutar();

    void actualizarOrientacion();

    void acelerarAdelante(bool nuevaDireccion);
};

#endif