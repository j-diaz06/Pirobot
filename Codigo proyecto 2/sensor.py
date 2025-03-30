from machine import Pin, I2C
import time
import vl53l0x

class SensorVL53L0X:
    def __init__(self, direccion, pin_xshut, i2c_bus=0, sda=21, scl=22):
        self.direccion = direccion
        self.pin_xshut = Pin(pin_xshut, Pin.OUT)
        self.i2c = I2C(i2c_bus, sda=Pin(sda), scl=Pin(scl))
        self.lox = None
        self.medida = -1
        
        # Secuencia de reinicio
        self.pin_xshut.value(0)
        time.sleep_ms(10)
        self.pin_xshut.value(1)
        time.sleep_ms(10)
        
        self._inicializar_sensor()

    def _inicializar_sensor(self):
        try:
            self.lox = vl53l0x.VL53L0X(self.i2c)
            self.lox._register(0x8A, self.direccion)  # Cambiar dirección
            self.lox.address = self.direccion
            print(f"[Sensor] Inicializado en 0x{self.direccion:02X}")
        except Exception as e:
            print(f"[Sensor] Error: {str(e)}")

    def medir(self):
        if self.lox:
            try:
                self.medida = self.lox.read()
                return self.medida
            except OSError:
                return -1
        return -1

    def obtener_medida(self):
        return self.medida
"""
#/ Ejemplo de uso con dos sensores
if __name__ == "__main__":
    # Configurar sensores
    sensor_delantero = SensorVL53L0X(0x30, pin_xshut=5)
    sensor_trasero = SensorVL53L0X(0x31, pin_xshut=4)
    
    while True:
        # Leer y mostrar medidas
        print(f"Delantero: {sensor_delantero.medir()} mm")
        print(f"Trasero: {sensor_trasero.medir()} mm")
        time.sleep(1)"""