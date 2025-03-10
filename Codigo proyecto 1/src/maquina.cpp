#include <Arduino.h>
#include "maquina.h"
#include "giro_acel.h"

// Constructor
Maquina::Maquina(int ENA, int IN1, int IN2, int ENB, int IN3, int IN4,
                 uint8_t direccion_trasero, uint8_t xshut_trasero,
                 uint8_t direccion_delantero, uint8_t xshut_delantero, short DISTANCIA_MAXIMA,
                 bool direccion_inicial, const unsigned long IntervaloCalibracion, short TIEMPO_ESPERA)
    : motores(ENA, IN1, IN2, ENB, IN3, IN4),
      sensor_trasero(direccion_trasero, xshut_trasero),
      sensor_delantero(direccion_delantero, xshut_delantero),
      aceleracion_adelante(direccion_inicial), giro_acel()
{

    estado_actual = ESTADO_ADELANTE;
    DISTANCIA_DETECCION = DISTANCIA_MAXIMA;
    intervaloCalibracion = IntervaloCalibracion;
    tiempo_espera = TIEMPO_ESPERA;
}

// Funciones de control de motores
void Maquina::mover()
{
    if (aceleracion_adelante)
    {
        motores.avanzar(motores.getVelocidad());
    }
    else
    {
        motores.retroceder(motores.getVelocidad());
    }
}

void Maquina::detener()
{
    motores.detener();
}

void Maquina::obstaculo_detectado()
{
    motores.detener();
    conteo_espera = millis();
    estado_actual = ESTADO_DERECHA;
    yawObjetivo = yawActual - 90.0; // al girar hacia la derecha el mpu entrega valores del giroscopio negativos
    girando = true;
}

// Lectura de sensores
void Maquina::leer_sensores()
{
    distancia_trasero = sensor_trasero.medir();
    Serial.printf("[sensor] trasero=%d ", distancia_trasero);
    delay(10);

    distancia_delantero = sensor_delantero.medir();
    Serial.printf("[sensor] delantero=%d\n", distancia_delantero);

    if (bypass) // los sensores siempre dan una lectura erronea de 8 al encender el vehiculo, por lo que se omite
    {
        if (distancia_delantero == 8 || distancia_trasero == 8)
        {
            distancia_trasero = sensor_trasero.medir();
            Serial.printf("[sensor] trasero=%d ", distancia_trasero);
            delay(10);
            distancia_delantero = sensor_delantero.medir();
            Serial.printf("[sensor] delantero=%d\n", distancia_delantero);
            bypass = false;
        }
    }

    if (!girando && aceleracion_adelante && (distancia_delantero <= DISTANCIA_DETECCION && distancia_delantero != -1))
    {
        obstaculo_detectado();
        Serial.printf("[enfrente] Detectado obstaculo\n");
        obstaculo_enfrente = true;
    }
    else if (!girando && !aceleracion_adelante && (distancia_trasero <= DISTANCIA_DETECCION && distancia_trasero != -1))
    {
        obstaculo_detectado();
        Serial.printf("[atras] Detectado obstaculo\n");
        obstaculo_atras = true;
    }
    else
    {
        obstaculo_enfrente = false;
        obstaculo_atras = false;
    }
}

// Ejecutar acción según estado
void Maquina::ejecutar_accion()
{
    switch (estado_actual)
    {
    case ESTADO_ADELANTE:
        if (millis() - conteo_espera >= tiempo_espera)
        {
            mover();
        }
        break;
    case ESTADO_DERECHA:
        if (millis() - conteo_espera >= tiempo_espera)
        {
            motores.girarDerecha(motores.getVelocidad());
        }
        break;
    }
}

// Bucle principal
void Maquina::loop()
{
    // Actualizar MPU cada 30ms
    if (millis() - tiempo_vl53 > 30)
    { // minimo 30

        leer_sensores();
        // maquina_estados();

        tiempo_vl53 = millis(); // contador para el loop
    }

    yawActual = giro_acel.getYaw(); // Lectura directa del valor actualizado
    Serial.printf("Yaw=%.2f", yawActual);

    if (girando && (yawActual - yawObjetivo < 10.0))
    {
        motores.detener();
        conteo_espera = millis();
        girando = false;
        estado_actual = ESTADO_ADELANTE;
        if (millis() - lastCalCheck >= intervaloCalibracion)
        {
            giro_acel.calibrateGyro();
            lastCalCheck = millis();
        }
    }

    ejecutar_accion();

    Serial.print("Estado: ");
    switch (estado_actual)
    {
    case ESTADO_ADELANTE:
        Serial.print("Adelante");
        break;
    case ESTADO_DERECHA:
        Serial.print("Derecha");
        break;
    }
    Serial.println();
}

// Inicialización
void Maquina::iniciar()
{
    // pinMode(PIN_DIRECCION, INPUT_PULLUP); // Configura el pin de dirección como entrada con pull-up
    motores.configurar();

    if (sensor_trasero.iniciar())
    {
        Serial.println("[Sensor] Trasero inicializado correctamente");
    }
    else
    {
        Serial.println("[Sensor] Error: Trasero no inicializado");
    }
    if (sensor_delantero.iniciar())
    {
        Serial.println("[Sensor] delantero inicializado correctamente");
    }
    else
    {
        Serial.println("[Sensor] Error: delantero no inicializado");
    }

    Serial.println("[Maquina] Inicializada.");

    giro_acel.iniciarMPU(); // Inicializar y calibrar MPU

    mover();
    tiempo_vl53 = millis(); // inicia por si necesita detenerse al iniciar
}

void Maquina::acelerarAdelante(bool nuevaDireccion)
{
    aceleracion_adelante = nuevaDireccion;
}