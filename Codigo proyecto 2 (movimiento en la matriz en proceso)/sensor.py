# sensor.py
from machine import Pin, I2C
import time
# Asegúrate de que la librería vl53l0x.py esté en el sistema de archivos del ESP32
# Puedes encontrarla buscando "micropython vl53l0x"
try:
    import vl53l0x
except ImportError:
    print("*******************************************************")
    print("Error: Falta la librería vl53l0x.py.")
    print("Descárgala (e.g., desde GitHub) y súbela al ESP32.")
    print("*******************************************************")
    # Clase Dummy para evitar errores fatales si falta la librería
    class vl53l0x:
        class VL53L0X:
            def __init__(self, i2c, address=0x29): self.address=address; print(f"Dummy VL53L0X creado en {hex(address)}")
            def read(self): return 8190 # Valor fuera de rango típico
            def _register(self, reg, val): pass
            def set_address(self, new_addr): self.address = new_addr; print(f"Dummy VL53L0X address set to {hex(new_addr)}")


class SensorVL53L0X:
    DEFAULT_ADDR = 0x29 # Dirección I2C por defecto del VL53L0X

    def __init__(self, i2c_bus_obj, direccion_deseada, pin_xshut):
        """
        Inicializa el sensor VL53L0X usando un pin XSHUT para establecer la dirección.

        Args:
            i2c_bus_obj: El objeto I2C ya inicializado (ej: I2C(0, sda=Pin(21), scl=Pin(22)))
            direccion_deseada: La dirección I2C que se le asignará al sensor.
            pin_xshut: El número del pin GPIO conectado al XSHUT del sensor.
        """
        self.i2c = i2c_bus_obj
        self.direccion_asignada = direccion_deseada
        self.pin_xshut = Pin(pin_xshut, Pin.OUT)
        self.lox = None
        self._last_measurement = -1
        self.initialized = False
        self.sensor_id_for_log = f"Sensor(Pin:{pin_xshut}, Addr:{hex(direccion_deseada)})"

        print(f"[{self.sensor_id_for_log}] Configurando XSHUT...")
        # Secuencia de reinicio y asignación de dirección:
        # 1. Poner XSHUT en bajo para desactivar el sensor (todos si hay varios)
        self.pin_xshut.value(0)
        # Esperar un poco más para asegurar que esté apagado
        time.sleep_ms(20)

        # 2. Poner XSHUT en alto para activar ESTE sensor
        print(f"[{self.sensor_id_for_log}] Activando (XSHUT=1)...")
        self.pin_xshut.value(1)
        # Esperar tiempo suficiente para que el sensor arranque completamente
        time.sleep_ms(50)

        # 3. Intentar inicializar la librería con la dirección POR DEFECTO (0x29)
        #    La librería intentará comunicarse en 0x29 para cambiarla.
        print(f"[{self.sensor_id_for_log}] Intentando inicializar en dirección por defecto ({hex(self.DEFAULT_ADDR)})...")
        try:
            # Crear instancia temporal en la dirección por defecto
            temp_lox = vl53l0x.VL53L0X(self.i2c, address=self.DEFAULT_ADDR)

            # 4. Cambiar la dirección I2C del sensor activo a la deseada
            print(f"[{self.sensor_id_for_log}] Cambiando dirección a {hex(direccion_deseada)}...")
            # Usar el método set_address si existe en tu librería vl53l0x.py
            if hasattr(temp_lox, 'set_address'):
                 temp_lox.set_address(direccion_deseada)
            else:
                 # Si no, usar el acceso directo al registro (menos portable)
                 # El registro 0x8A es I2C_SLAVE_DEVICE_ADDRESS según datasheet VL53L0X
                 print(f"[{self.sensor_id_for_log}] Usando acceso a registro 0x8A (asegúrate que sea correcto).")
                 temp_lox._register(0x8A, direccion_deseada)

            # Pequeña pausa después de cambiar la dirección
            time.sleep_ms(10)

            # 5. Ahora que la dirección ha sido cambiada en el hardware,
            #    creamos la instancia final del objeto apuntando a la NUEVA dirección.
            print(f"[{self.sensor_id_for_log}] Creando instancia final en {hex(direccion_deseada)}...")
            self.lox = vl53l0x.VL53L0X(self.i2c, address=direccion_deseada)

            # (Opcional) Configurar modos/rangos si es necesario. Ejemplo:
            # self.lox.set_measurement_timing_budget(40000) # Microsegundos
            # self.lox.set_signal_rate_limit(0.5) # MCPS
            # self.lox.start_continuous() # Si quieres modo continuo

            print(f"[{self.sensor_id_for_log}] Inicializado correctamente en {hex(self.direccion_asignada)}")
            self.initialized = True

        except OSError as e:
             print(f"[{self.sensor_id_for_log}] Error de comunicación I2C durante inicialización: {e}.")
             print(f"    ¿Conflicto de dirección o sensor no conectado/alimentado?")
             print(f"    Asegúrate que no haya otro dispositivo en {hex(self.DEFAULT_ADDR)} o {hex(direccion_deseada)} antes de activar XSHUT.")
             # Escanear bus puede ayudar a depurar
             try:
                 devices = self.i2c.scan()
                 print("    Dispositivos I2C encontrados AHORA:", [hex(device) for device in devices])
             except Exception as scan_e:
                 print(f"    Error escaneando I2C: {scan_e}")

        except AttributeError as e:
            print(f"[{self.sensor_id_for_log}] Error de atributo: {e}. ¿Método set_address o _register no existe en tu librería vl53l0x.py?")

        except Exception as e:
            print(f"[{self.sensor_id_for_log}] Error inesperado durante inicialización: {e}")
            import sys
            sys.print_exception(e)


    def medir(self):
        """
        Realiza una medición de distancia. Es una operación BLOQUEANTE.
        Retorna la distancia en mm o -1 si hay error o no está inicializado.
        """
        if self.lox and self.initialized:
            try:
                # La lectura puede tardar ~30ms o más dependiendo de la configuración
                # Si usas modo continuo, .read() podría ser más rápido.
                # Si no, .read() realiza una medición completa.
                measurement = self.lox.read()

                # Validar rango básico (VL53L0X típico ~2m max, a veces menos)
                if measurement > 2000 or measurement < 0: # Rango inválido o error
                    self._last_measurement = -1 # Considerar inválido
                    # print(f"[{self.sensor_id_for_log}] Lectura fuera de rango: {measurement}")
                else:
                    self._last_measurement = measurement
                return self._last_measurement
            except OSError as e:
                # Errores comunes son timeout o NACK si el sensor se desconecta
                print(f"[{self.sensor_id_for_log}] Error I2C al leer: {e}")
                self._last_measurement = -1
                self.initialized = False # Marcar como no inicializado si hay error I2C?
                return -1
            except Exception as e:
                print(f"[{self.sensor_id_for_log}] Error inesperado al leer: {e}")
                self._last_measurement = -1
                return -1
        else:
            # print(f"[{self.sensor_id_for_log}] Intento de medir sin inicializar.")
            return -1

    def obtener_ultima_medida(self):
        """Retorna la última medición realizada sin realizar una nueva lectura."""
        return self._last_measurement

