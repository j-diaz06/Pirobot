#include <Arduino.h>
#include "motor.h"

// Definir pines del L298N
#define ENA 25 // PWM Motor A
#define IN1 27
#define IN2 32
#define IN3 14
#define IN4 33
#define ENB 26 // PWM Motor B

Motor motores(ENA, IN1, IN2, ENB, IN3, IN4);  // Crear objeto Motor

void setup() {
    Serial.begin(115200);  // Iniciar comunicación serial
    motores.configurar();  // Configurar los pines del motor
    Serial.println("Motores configurados correctamente.");
    Serial.println("Enviar comandos: adelante, atras, izquierda, derecha, detener");
}

void loop() {
    // Verificar si hay datos disponibles en el puerto serial
    if (Serial.available() > 0) {
        String comando = Serial.readStringUntil('\n');  // Leer el comando enviado
        comando.trim();  // Eliminar espacios en blanco al inicio y final

        // Ejecutar la acción correspondiente al comando
        if (comando == "adelante") {
            Serial.println("Avanzando");
            motores.avanzar(100);  // Avanzar a velocidad 200
        } else if (comando == "atras") {
            Serial.println("Retrocediendo");
            motores.retroceder(100);  // Retroceder a velocidad 200
        } else if (comando == "izquierda") {
            Serial.println("Girando Izquierda");
            motores.girarIzquierda(100);  // Girar a la izquierda a velocidad 200
        } else if (comando == "derecha") {
            Serial.println("Girando Derecha");
            motores.girarDerecha(100);  // Girar a la derecha a velocidad 200
        } else if (comando == "detener") {
            Serial.println("Deteniendo");
            motores.detener();  // Detener los motores
        } else {
            Serial.println("Comando no reconocido. Usar: adelante, atras, izquierda, derecha, detener");
        }
    }
}
