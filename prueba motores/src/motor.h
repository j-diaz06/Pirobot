class Motor {
private:
    int enA, in1, in2;  // Pines para Motor A
    int enB, in3, in4;  // Pines para Motor B
    int velocidad;       // Velocidad de los Motor (0-255)

public:
    Motor(int enA, int in1, int in2, int enB, int in3, int in4);  // Constructor
    void configurar();  // Configura los pines
    void avanzar(int vel);
    void retroceder(int vel);
    void girarIzquierda(int vel);
    void girarDerecha(int vel);
    void detener();
};