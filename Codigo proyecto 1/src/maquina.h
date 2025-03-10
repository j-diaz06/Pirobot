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

class Maquina
{
private:
    
    Sensor sensor_trasero;
    Sensor sensor_delantero;

    bool aceleracion_adelante; // true = adelante, false = atrás
    bool girando;
    // Lectura de sensores
    bool obstaculo_enfrente;
    bool obstaculo_atras;
    int estado_actual;
    float yawActual;
    float yawObjetivo = -90;
    unsigned long conteo_espera;
    unsigned long tiempo_vl53 = 0;
    unsigned long lastCalCheck = 0;
    unsigned long intervaloCalibracion;
    short DISTANCIA_DETECCION;
    bool bypass = true;
    short tiempo_espera;
    int distancia_trasero = 0;
    int distancia_delantero = 0;

public:
    // Constructor
    Maquina(int ENA, int IN1, int IN2, int ENB, int IN3, int IN4,
            uint8_t direccion_trasero, uint8_t xshut_trasero,
            uint8_t direccion_delantero, uint8_t xshut_delantero, short DISTANCIA_MAXIMA,
            bool direccion_inicial, const unsigned long IntervaloCalibracion, short TIEMPO_ESPERA);
    void iniciar();
    void loop();
    void mover();
    void detener();
    void girar_derecha();
    void obstaculo_detectado();
    void leer_sensores();
    // Ejecutar acción según estado
    void ejecutar_accion();
    void acelerarAdelante(bool nuevaDireccion);
    Sensor &getSensorDelantero() { return sensor_delantero; }
    Sensor &getSensorTrasero() { return sensor_trasero; }
    float getYaw() { return yawActual; }

    Motor motores;

    Giro_acel giro_acel; // Objeto MPU6050

    // void actualizarOrientacion(); para uso futuro
};

#endif