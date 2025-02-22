#include "dual.h"

// Definir pines del L298N
#define ENA 25 // PWM Motor A
#define IN1 27
#define IN2 32
#define IN3 14
#define IN4 33
#define ENB 26 // PWM Motor B

Motor motor(ENA, IN1, IN2, ENB, IN3, IN4);  // Crear objeto Motor

Sensor sensor1(0x30, 9); // Dirección 0x30, pin XSHUT 9
Sensor sensor2(0x31, 10); // Dirección 0x31, pin XSHUT 10

void setup() {
  Serial.begin(9600);
  motor.configurar();
  sensor1.iniciar();
  sensor2.iniciar();
  delay(1000); // Espera inicial para estabilizar
}

void loop() {
  int dist1 = sensor1.medir();
  int dist2 = sensor2.medir();

  // Lógica de movimiento basada en sensores
  if (dist1 != -1 && dist2 != -1) { // Ambos detectan objeto
    motor.avanzar(200); // Avanzar a velocidad 200
  } else if (dist1 != -1) { // Solo sensor1 detecta
    motor.girarDerecha(200); // Girar derecha
  } else if (dist2 != -1) { // Solo sensor2 detecta
    motor.girarIzquierda(200); // Girar izquierda
  } else { // Ninguno detecta
    motor.detener(); // Detener motores
  }

  delay(100); // Pequeña pausa para estabilidad
}