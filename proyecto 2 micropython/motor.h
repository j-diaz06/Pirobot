#ifndef MOTOR_H
#define MOTOR_H

class Motor {
private:
    int enA, in1, in2;  // Pines para Motor A
    int enB, in3, in4;  // Pines para Motor B
    int velocidad;       // Velocidad de los motores (110-255)
    int estado;          // 0: detenido, 1: avanzar, 2: retroceder, 3: izquierda, 4: derecha

public:
    Motor(int enA, int in1, int in2, int enB, int in3, int in4);  // Constructor
    void configurar();  // Configura los pines
    void avanzar(int vel);
    void retroceder(int vel);
    void girarIzquierda(int vel);
    void girarDerecha(int vel);
    void detener();
    void setVelocidad(int vel);  // Establecer la velocidad
    int getVelocidad();  // Obtener la velocidad actual
    void actualizarVelocidad();  // Actualizar la velocidad en tiempo real
};

#endif