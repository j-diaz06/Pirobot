#include <Arduino.h>
#include "motor.h"

Motor::Motor(int _enA, int _in1, int _in2, int _enB, int _in3, int _in4) {
    enA = _enA; in1 = _in1; in2 = _in2;
    enB = _enB; in3 = _in3; in4 = _in4;
    velocidad = 255;  // Velocidad máxima por defecto
}

void Motor::configurar() {
    // Configurar los pines de control como salidas
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
}

void Motor::avanzar(int vel) {
    velocidad = constrain(vel, 0, 255);
    analogWrite(enA, velocidad);  // Usar analogWrite para ENA
    analogWrite(enB, velocidad);  // Usar analogWrite para ENB
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
}

void Motor::retroceder(int vel) {
    velocidad = constrain(vel, 0, 255);
    analogWrite(enA, velocidad);  // Usar analogWrite para ENA
    analogWrite(enB, velocidad);  // Usar analogWrite para ENB
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
}

void Motor::girarIzquierda(int vel) {
    velocidad = constrain(vel, 0, 255);
    analogWrite(enA, velocidad);  // Usar analogWrite para ENA
    analogWrite(enB, velocidad);  // Usar analogWrite para ENB
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
}

void Motor::girarDerecha(int vel) {
    velocidad = constrain(vel, 0, 255);
    analogWrite(enA, velocidad);  // Usar analogWrite para ENA
    analogWrite(enB, velocidad);  // Usar analogWrite para ENB
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
}

void Motor::detener() {
    analogWrite(enA, 0);  // Detener Motor A
    analogWrite(enB, 0);  // Detener Motor B
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}