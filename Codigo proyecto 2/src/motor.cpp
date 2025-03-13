#include <Arduino.h>
#include "motor.h"

Motor::Motor(int _enA, int _in1, int _in2, int _enB, int _in3, int _in4) {
    enA = _enA; in1 = _in1; in2 = _in2;
    enB = _enB; in3 = _in3; in4 = _in4;
    velocidad = 50;  // Velocidad inicial 110
    estado = 0;       // Estado inicial: detenido
}

void Motor::configurar() {
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    Serial.println("[Motor] Motores configurados correctamente.");
}

void Motor::avanzar(int vel) {
    if (vel < 1 || vel > 255) {
        Serial.printf("[Motor] Error: Velocidad inválida (Avanzar) %d\n", vel);
        return;
    }
    Serial.printf("[Motor] Avanzar: ENA=%d, ENB=%d\n", vel, vel);
    
    //velocidad = constrain(vel, 110, 255);
    estado = 1;
    
    analogWrite(enA, velocidad);
    analogWrite(enB, velocidad);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
}

void Motor::retroceder(int vel) {
    if (vel < 110 || vel > 255) {
        Serial.printf("[Motor] Error: Velocidad inválida (Retroceder) %d\n", vel);
        return;
    }
    Serial.printf("[Motor] Retroceder: ENA=%d, ENB=%d\n", vel, vel);
    
    //velocidad = constrain(vel, 110, 255);
    estado = 2;
    analogWrite(enA, velocidad);
    analogWrite(enB, velocidad);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
}

void Motor::girarIzquierda(int vel) {
    if (vel < 110 || vel > 255) {
        Serial.printf("[Motor] Error: Velocidad inválida (Girar Izquierda) %d\n", vel);
        return;
    }
    int velDer = vel;
    int velIzq = vel;  // Reducción del 40%
    Serial.printf("[Motor] Girar Izquierda: ENA=%d, ENB=%d\n", velDer, velIzq);
    
    //velocidad = constrain(vel, 110, 255);
    estado = 3;
    analogWrite(enA, velDer);
    analogWrite(enB, velIzq);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
}

void Motor::girarDerecha(int vel) {
    if (vel < 1 || vel > 255) {
        Serial.printf("[Motor] Error: Velocidad inválida (Girar Derecha) %d\n", vel);
        return;
    }
    int velIzq = vel;
    int velDer = vel;  // Reducción del 40%
    Serial.printf("[Motor] Girar Derecha: ENA=%d, ENB=%d\n", velDer, velIzq);
    
    //velocidad = constrain(vel, 110, 255);
    estado = 4;
    analogWrite(enA, velDer);
    analogWrite(enB, velIzq);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
}

void Motor::detener() {
    Serial.println("[Motor] Deteniendo motores...");
    estado = 0;
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

void Motor::setVelocidad(int vel) {
    if (vel < 110 || vel > 255) {
        Serial.printf("[Motor] Error: Velocidad inválida (Set Velocidad) %d\n", vel);
        return;
    }
    //velocidad = constrain(vel, 110, 255);
    Serial.printf("[Motor] Nueva velocidad: %d\n", velocidad);
    actualizarVelocidad();
}

int Motor::getVelocidad() {
    return velocidad;
}

void Motor::actualizarVelocidad() {
    if (estado != 0) {  // Si los motores están en movimiento
        Serial.printf("[Motor] Actualizando velocidad: Estado=%d, Velocidad=%d\n", estado, velocidad);
        switch (estado) {
            case 1: avanzar(velocidad); break;
            case 2: retroceder(velocidad); break;
            case 3: girarIzquierda(velocidad); break;
            case 4: girarDerecha(velocidad); break;
        }
    }
}