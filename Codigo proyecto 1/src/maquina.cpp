#include <Arduino.h>
#include "maquina.h"
#include "giro_acel.h"

//#include "sensor.h"

// Constructor
Maquina::Maquina(int ENA, int IN1, int IN2, int ENB, int IN3, int IN4,
    uint8_t direccion_trasero, uint8_t xshut_trasero,
    uint8_t direccion_delantero, uint8_t xshut_delantero,short DISTANCIA_MAXIMA,
    bool direccion_inicial, const unsigned long IntervaloCalibracion, short TIEMPO_ESPERA)
: motores(ENA, IN1, IN2, ENB, IN3, IN4),
sensor_trasero(direccion_trasero, xshut_trasero),
sensor_delantero(direccion_delantero, xshut_delantero),
aceleracion_adelante(direccion_inicial){
    estado_actual = ESTADO_ADELANTE;
    //direccion_adelante = true; // Por defecto, la dirección es adelante
    DISTANCIA_DETECCION=DISTANCIA_MAXIMA;
    //Motor motores(ENA, IN1, IN2, ENB, IN3, IN4);
    intervaloCalibracion=IntervaloCalibracion;

    tiempo_espera = TIEMPO_ESPERA;
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
void Maquina::leer_sensores() {
    //Obstaculo obstaculo;
    do
    {
        distancia_trasero = sensor_trasero.medir();
        Serial.printf("[sensor] trasero=%d ",distancia_trasero);
        delay(5);
    }while(distancia_trasero==8);

    do
    {
        distancia_delantero = sensor_delantero.medir();
        Serial.printf("[sensor] delantero=%d\n",distancia_delantero);
        delay(5);
    } while (distancia_delantero==8);
    
    if(distancia_delantero <= DISTANCIA_DETECCION && distancia_delantero != -1){
        Serial.printf("[enfrente] Detectado obstaculo\n");
        obstaculo_enfrente=true;
    }else {obstaculo_enfrente=false;}
    
    if(distancia_trasero <= DISTANCIA_DETECCION && distancia_trasero != -1){
        Serial.printf("[atras] Detectado obstaculo\n");
        obstaculo_atras = true;
    }else {obstaculo_atras = false;}
    //return obstaculo;
}

// Lógica de giro
bool Maquina::puede_girar(int direccion) {
    return true;
}
/*
void Maquina::actualizarOrientacion() {
    mpu.actualizarFiltro();
    yawActual = mpu.obtenerYaw();

    yawActual = giro_acel.calcularYaw(giro_acel.leerGiroscopio()); // Usar variable global yaw
}*/

// Máquina de estados
void Maquina::maquina_estados() {
    yawActual = giro_acel.getYaw(); // Lectura directa del valor actualizado
    Serial.printf("Yaw=%.2f",yawActual);
    lastMPURead = millis();
    
    if(girando) {
        if(yawActual - yawObjetivo < 10.0) { // Margen de 2°
            girando = false;
            estado_actual = ESTADO_ESPERA_ADELANTE;
        }
        return;
    }

    // Lógica original de detección
    if((aceleracion_adelante && obstaculo_enfrente) || (!aceleracion_adelante && obstaculo_atras)) {

        estado_actual = ESTADO_ESPERA_DERECHA;
        
        yawObjetivo = yawActual - 90.0;// al girar hacia la derecha el mpu entrega valores del giroscopio negativos
        //if(yawObjetivo >= 360) yawObjetivo -= 360;
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
        case ESTADO_ADELANTE:
        if(millis() - conteo_espera >= tiempo_espera) {
            mover_adelante();
        }
        break;
        case ESTADO_DERECHA:
        if(millis() - conteo_espera >= tiempo_espera) {
            motores.girarDerecha(motores.getVelocidad());
        }
        break;
        //case ESTADO_IZQUIERDA: break;
        //case ESTADO_DETENIDO: break;
        case ESTADO_ESPERA_ADELANTE:
        motores.detener();
        conteo_espera = millis();
        estado_actual=ESTADO_ADELANTE;
        break;
        case ESTADO_ESPERA_DERECHA:
        motores.detener();
        conteo_espera = millis();
        estado_actual=ESTADO_DERECHA;
        break;
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

    giro_acel.iniciarMPU(); // Inicializar y calibrar MPU

    leer_sensores();
}

// Bucle principal
void Maquina::ejecutar() {
    // Leer el estado del pin de dirección
    //direccion_adelante = digitalRead(PIN_DIRECCION) == LOW; // LOW = adelante, HIGH = atrás (pull-up)

    // Actualizar MPU cada 50ms
    if(millis() - lastMPURead > 50) {//minimo 30
    //giro_acel.actualizarFiltro();

    leer_sensores();
    maquina_estados();
    ejecutar_accion();

    Serial.print(" | Estado: ");
    switch (estado_actual) {
        case ESTADO_ADELANTE: Serial.print("Adelante"); break;
        case ESTADO_DERECHA: Serial.print("Derecha"); break;
    }
    Serial.println();

    //giro_acel.calcularYaw(giro_acel.leerGiroscopio());
    //lastMPURead = millis();
    }

    // Monitorización
    //Serial.print("Dirección: ");
    //Serial.print(direccion_adelante ? "Adelante" : "Atrás");

    if (millis() - lastCalCheck >= intervaloCalibracion) {
        giro_acel.calibrateGyro();
        lastCalCheck = millis();
      }
}

void Maquina::acelerarAdelante(bool nuevaDireccion) {
    aceleracion_adelante = nuevaDireccion;
}