#include <Arduino.h>
#include "maquina.h"

//#include "sensor.h"

// Constructor
Maquina::Maquina(int ENA, int IN1, int IN2, int ENB, int IN3, int IN4,
    uint8_t direccion_trasero, uint8_t xshut_trasero,
    uint8_t direccion_delantero, uint8_t xshut_delantero,short DISTANCIA_MAXIMA,
    Giro_acel& sensorMpu, bool direccion_inicial)
: motores(ENA, IN1, IN2, ENB, IN3, IN4),
sensor_trasero(direccion_trasero, xshut_trasero),
sensor_delantero(direccion_delantero, xshut_delantero),
mpu(sensorMpu), aceleracion_adelante(direccion_inicial) {
    estado_actual = ESTADO_ADELANTE;
    //direccion_adelante = true; // Por defecto, la dirección es adelante
    DISTANCIA_DETECCION=DISTANCIA_MAXIMA;
    //Motor motores(ENA, IN1, IN2, ENB, IN3, IN4);
}

// Funciones de control de motores
void Maquina::mover_adelante() {
    if (aceleracion_adelante) {
        motores.avanzar(motores.getVelocidad());
    } else {
        motores.retroceder(motores.getVelocidad());
    }
}
/*
void Maquina::girar_derecha() {
    if (direccion_adelante) {
        motores.girarDerecha(motores.getVelocidad());
    } else {
        //falta este giro
    }
}*/

void Maquina::detener() {
    motores.detener();
}

// Lectura de sensores
Maquina::Sensores Maquina::leer_sensores() {
    Sensores s;
    int distancia_trasero = sensor_trasero.medir();
    int distancia_delantero = sensor_delantero.medir();

    s.front = (distancia_delantero <= DISTANCIA_DETECCION && distancia_delantero != -1);
    s.rear = (distancia_trasero <= DISTANCIA_DETECCION && distancia_trasero != -1);
    return s;
}

// Lógica de giro
bool Maquina::puede_girar(int direccion) {
    return true;
}

void Maquina::actualizarOrientacion() {
    mpu.actualizarFiltro();
    yawActual = mpu.obtenerYaw();
}

// Máquina de estados
void Maquina::maquina_estados(bool front, bool rear) {
    actualizarOrientacion();
    
    if(girando) {
        if(fabs(yawActual - yawObjetivo) < 2.0) { // Margen de 2°
            girando = false;
            estado_actual = ESTADO_ADELANTE;
        }
        return;
    }

    // Lógica original de detección
    if((aceleracion_adelante && front) || (!aceleracion_adelante && rear)) {
        estado_actual = ESTADO_DERECHA;
        yawObjetivo = yawActual + 90.0;
        if(yawObjetivo >= 360) yawObjetivo -= 360;
        girando = true;
    }

/* LOGICA DE GIRO ANTERIOR: (IGNORAR)
    switch (estado_actual) {
        case ESTADO_ADELANTE:
            if (direccion_adelante) {
                // Movimiento hacia adelante
                if (front) {
                    // Si el sensor delantero detecta algo, girar a la derecha
                    if (puede_girar(GIRO_DERECHA)) estado_actual = ESTADO_DERECHA;
                } else if (rear) {
                    // Si el sensor trasero detecta algo, no hacer nada
                    // (se mantiene en estado adelante)
                }
            } else {
                // Movimiento hacia atrás
                if (rear) {
                    // Si el sensor trasero detecta algo, girar a la derecha
                    if (puede_girar(GIRO_DERECHA)) estado_actual = ESTADO_DERECHA;
                } else if (front) {
                    // Si el sensor delantero detecta algo, no hacer nada
                    // (se mantiene en estado adelante)
                }
            }
            break;

        case ESTADO_DERECHA:
            if (!front && !rear) {
                // Si no hay obstáculos, volver al estado adelante
                estado_actual = ESTADO_ADELANTE;
            }
            break;
    }*/
}

// Ejecutar acción según estado
void Maquina::ejecutar_accion() {
    switch (estado_actual) {
        case ESTADO_ADELANTE: mover_adelante(); break;
        case ESTADO_DERECHA:  motores.girarDerecha(motores.getVelocidad()); break;
    }
}

// Inicialización
void Maquina::iniciar() {
    //pinMode(PIN_DIRECCION, INPUT_PULLUP); // Configura el pin de dirección como entrada con pull-up
    motores.configurar();

    if (sensor_trasero.iniciar()) {Serial.println("[Sensor] Trasero inicializado correctamente");}
        else{Serial.println("[Sensor] Error: Trasero no inicializado");}
    if (sensor_delantero.iniciar()) {Serial.println("[Sensor] delantero inicializado correctamente");}
        else{Serial.println("[Sensor] Error: delantero no inicializado");}

        Serial.println("[Maquina] Inicializada.");
}

// Bucle principal
void Maquina::ejecutar() {
    // Leer el estado del pin de dirección
    //direccion_adelante = digitalRead(PIN_DIRECCION) == LOW; // LOW = adelante, HIGH = atrás (pull-up)

    Sensores s = leer_sensores();
    maquina_estados(s.front, s.rear);
    ejecutar_accion();

    // Monitorización
    Serial.print("Dirección: ");
    //Serial.print(direccion_adelante ? "Adelante" : "Atrás");
    Serial.print(" | Estado: ");
    switch (estado_actual) {
        case ESTADO_ADELANTE: Serial.print("Adelante"); break;
        case ESTADO_DERECHA: Serial.print("Derecha"); break;
    }
    Serial.println();

    delay(100);
}

void Maquina::acelerarAdelante(bool nuevaDireccion) {
    aceleracion_adelante = nuevaDireccion;
}