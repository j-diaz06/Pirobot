# sensor.py (CORREGIDO)
from machine import Pin, I2C
import time
import vl53l0x # Asegúrate que este archivo está en el mismo directorio

class SensorVL53L0X:
    # --- CORRECCIÓN AQUÍ: Cambiar parámetros para aceptar un objeto I2C ---
    # def __init__(self, direccion, pin_xshut, i2c_bus=0, sda=21, scl=22): # <-- Línea original
    def __init__(self, direccion, pin_xshut, i2c_object): # <-- Línea Corregida
        self.direccion = direccion
        self.pin_xshut = Pin(pin_xshut, Pin.OUT)
        # --- CORRECCIÓN AQUÍ: Usar el objeto I2C pasado directamente ---
        # self.i2c = I2C(i2c_bus, sda=Pin(sda), scl=Pin(scl)) # <-- Línea original
        self.i2c = i2c_object # <-- Línea Corregida
        # --- FIN CORRECCIONES ---
        self.lox = None
        self.medida = -1

        # Secuencia de reinicio (igual)
        self.pin_xshut.value(0)
        time.sleep_ms(10)
        self.pin_xshut.value(1)
        time.sleep_ms(10)

        self._inicializar_sensor()

    def _inicializar_sensor(self):
        try:
            # Ahora usa self.i2c que es el objeto pasado desde Maquina
            self.lox = vl53l0x.VL53L0X(self.i2c)
            # Forzar la dirección correcta después de la inicialización general
            # (Puede que la librería vl53l0x intente usar la dirección por defecto 0x29 inicialmente)
            # Es importante intentar cambiarla ANTES de interactuar mucho con ella si hay conflicto.
            # Esta secuencia puede necesitar ajustes dependiendo de cómo vl53l0x.py maneje el cambio.
            try:
                 self.lox._register(0x8A, self.direccion)  # Cambiar registro de dirección I2C
                 self.lox.address = self.direccion         # Actualizar la dirección en el objeto Python
                 print(f"[Sensor] Dirección establecida a 0x{self.direccion:02X}")
            except Exception as e:
                 print(f"[Sensor Warn] No se pudo cambiar dirección I2C a 0x{self.direccion:02X} post-init: {e}")
                 # Podría ya estar en esa dirección si la secuencia de XSHUT funcionó bien.

            print(f"[Sensor] Intentando usar sensor en 0x{self.lox.address:02X}")
            # Realizar una lectura de prueba para confirmar
            test_read = self.lox.read()
            print(f"[Sensor] Lectura inicial en 0x{self.lox.address:02X}: {test_read} mm")

        except Exception as e:
            # Imprimir el error completo puede ayudar a depurar
            import sys
            sys.print_exception(e)
            print(f"[Sensor Fatal] Error inicializando VL53L0X en dirección deseada 0x{self.direccion:02X}")
            self.lox = None # Asegurarse que no se use si falló

    def medir(self):
        if self.lox:
            try:
                self.medida = self.lox.read()
                # Añadir un pequeño retardo si las lecturas son inestables
                # time.sleep_ms(1)
                return self.medida
            except Exception as e: # Capturar cualquier error, no solo OSError
                print(f"[Sensor Error] midiendo en 0x{self.direccion:02X}: {e}")
                # Devolver un valor consistente en caso de error
                self.medida = -1
                return -1
        return -1 # Retornar -1 si self.lox no está inicializado

    def obtener_medida(self):
        # Este método puede ser redundante si medir() ya retorna la medida
        return self.medida