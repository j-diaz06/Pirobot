#include "Adafruit_VL53L0X.h"

class Sensor {
    Adafruit_VL53L0X sensor;
    uint8_t direccionI2C;  // Dirección única para cada sensor
    uint8_t pinXSHUT;      // Pin XSHUT para habilitar/deshabilitar el sensor

    public:
    Sensor(uint8_t direccion, uint8_t xshut);  // Constructor con dirección y XSHUT
    void iniciar();
    int medir();  // Devuelve la distancia en mm
    void medirYEnviar();  // Mide la distancia y la envía por Serial
};

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
