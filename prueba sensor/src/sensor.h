#include "Adafruit_VL53L0X.h"
<<<<<<< HEAD

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
=======
extern Adafruit_VL53L0X lox;

class sensor{
  
  public:
  void iniciar();
  void medir();
  };
>>>>>>> bab335048e1c06f2bd1e658b8a751e1b6eaf5ac7
