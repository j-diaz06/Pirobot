# giro_acel.py
from machine import I2C, Pin
import time

class GiroAcel:
    # Constantes de Registros MPU6050
    MPU_ADDR = 0x68         # Dirección I2C estándar
    PWR_MGMT_1_REG = 0x6B   # Registro de gestión de energía
    GYRO_CONFIG_REG = 0x1B  # Registro de configuración del giroscopio
    GYRO_ZOUT_H_REG = 0x47  # Registro de lectura del eje Z (Byte Alto)

    # Valores de Configuración
    GYRO_FS_SEL_500DPS = 0x08 # Valor para configurar ±500 grados/segundo
    # Factor de escala LSB/(°/s) para ±500 dps
    SCALE_FACTOR_500DPS = 65.5

    def __init__(self, i2c_bus_obj, intervalo_calibracion_ms):
        """
        Inicializa el MPU6050 y la lógica de cálculo de Yaw.

        Args:
            i2c_bus_obj: El objeto I2C ya inicializado.
            intervalo_calibracion_ms: Cada cuánto tiempo (en ms) se debe recalibrar el giroscopio.
        """
        self.i2c = i2c_bus_obj
        self.intervalo_calibracion = intervalo_calibracion_ms
        self.offset_gyro_z = 0.0 # Offset calculado durante la calibración
        self.yaw = 0.0 # Ángulo Yaw acumulado en grados
        self.ultima_actualizacion_ticks = time.ticks_ms()
        self.ultima_calibracion_ticks = 0
        self.initialized = False
        self.sensor_id_for_log = f"Giroscopio(Addr:{hex(self.MPU_ADDR)})"

        # Intentar inicializar al crear el objeto
        self.iniciar_mpu()


    def iniciar_mpu(self):
        """Despierta el MPU6050, configura el rango y realiza la calibración inicial."""
        print(f"[{self.sensor_id_for_log}] Iniciando MPU6050...")
        try:
            # Verificar si el dispositivo responde en la dirección esperada
            devices = self.i2c.scan()
            if self.MPU_ADDR not in devices:
                print(f"[{self.sensor_id_for_log}] ERROR: MPU6050 no encontrado en {hex(self.MPU_ADDR)}")
                print("    Dispositivos encontrados:", [hex(d) for d in devices])
                return False

            # Despertar el MPU6050 (escribir 0 en PWR_MGMT_1)
            print(f"[{self.sensor_id_for_log}] Despertando MPU...")
            self.i2c.writeto_mem(self.MPU_ADDR, self.PWR_MGMT_1_REG, bytes([0]))
            time.sleep_ms(100) # Esperar a que estabilice

            # Configurar rango del giroscopio a ±500°/s
            print(f"[{self.sensor_id_for_log}] Configurando rango a +/- 500 dps...")
            self.i2c.writeto_mem(self.MPU_ADDR, self.GYRO_CONFIG_REG, bytes([self.GYRO_FS_SEL_500DPS]))
            time.sleep_ms(10)

            # Calibración inicial
            print(f"[{self.sensor_id_for_log}] Realizando calibración inicial (no mover)...")
            if self._realizar_calibracion():
                print(f"[{self.sensor_id_for_log}] Calibración inicial completa. Offset Z: {self.offset_gyro_z:.4f}")
                self.ultima_actualizacion_ticks = time.ticks_ms() # Resetear tiempo para yaw
                self.initialized = True
                return True
            else:
                print(f"[{self.sensor_id_for_log}] ERROR: Falló la calibración inicial.")
                return False

        except OSError as e:
            print(f"[{self.sensor_id_for_log}] Error I2C durante inicialización: {e}")
            return False
        except Exception as e:
             print(f"[{self.sensor_id_for_log}] Error inesperado durante inicialización: {e}")
             return False

    def _realizar_calibracion(self):
        """Mide el offset del giroscopio en reposo."""
        num_lecturas = 500
        suma_raw_z = 0
        print(f"[{self.sensor_id_for_log}] Calibrando con {num_lecturas} lecturas...")
        try:
            for i in range(num_lecturas):
                suma_raw_z += self._leer_raw_z()
                time.sleep_ms(3) # Pausa un poco más larga entre lecturas de calibración
                if i % 50 == 0: print(".", end="") # Progreso
            print(f"\n[{self.sensor_id_for_log}] Lecturas de calibración completadas.")
            self.offset_gyro_z = suma_raw_z / num_lecturas
            self.ultima_calibracion_ticks = time.ticks_ms()
            return True
        except OSError as e:
            print(f"\n[{self.sensor_id_for_log}] Error I2C durante calibración: {e}")
            return False
        except Exception as e:
            print(f"\n[{self.sensor_id_for_log}] Error inesperado durante calibración: {e}")
            return False

    def _calibrar_periodicamente(self):
        """Verifica si es tiempo de recalibrar y lo hace si es necesario."""
        ahora_ticks = time.ticks_ms()
        # Usar ticks_diff para manejar posible desbordamiento de ticks_ms
        if self.initialized and time.ticks_diff(ahora_ticks, self.ultima_calibracion_ticks) >= self.intervalo_calibracion:
            print(f"[{self.sensor_id_for_log}] Recalibrando periódicamente (intervalo {self.intervalo_calibracion} ms)...")
            if self._realizar_calibracion():
                print(f"[{self.sensor_id_for_log}] Recalibración completa. Nuevo offset Z: {self.offset_gyro_z:.4f}")
            else:
                print(f"[{self.sensor_id_for_log}] ERROR: Falló la recalibración periódica.")
            # Resetear Yaw después de recalibrar? Podría ser problemático si el robot estaba girando.
            # Mejor no resetear aquí a menos que se sepa que está parado.
            # self.reset_yaw(0.0)


    def _leer_raw_z(self):
        """
        Lee el valor raw del giroscopio en el eje Z.
        Retorna el valor entero de 16 bits con signo.
        Lanza OSError en caso de fallo de lectura I2C persistente.
        """
        max_retries = 3
        for attempt in range(max_retries):
            try:
                # Leer 2 bytes (High y Low) del registro GYRO_ZOUT_H
                data = self.i2c.readfrom_mem(self.MPU_ADDR, self.GYRO_ZOUT_H_REG, 2)
                # Combinar los bytes en un entero de 16 bits con signo
                value = (data[0] << 8) | data[1]
                # Convertir a valor con signo (complemento a dos manual)
                if value >= 32768: # 0x8000
                    value -= 65536 # 0x10000
                return value
            except OSError as e:
                print(f"[{self.sensor_id_for_log}] Error I2C al leer raw Z (intento {attempt+1}/{max_retries}): {e}")
                if attempt == max_retries - 1: # Si es el último intento, lanzar excepción
                     # Podríamos marcar el sensor como no inicializado aquí?
                     # self.initialized = False
                     raise e # Relanzar la excepción para que el llamador la maneje
                time.sleep_ms(10) # Esperar antes de reintentar

        # Si sale del bucle sin éxito (aunque no debería por el raise)
        raise OSError(f"[{self.sensor_id_for_log}] Fallo al leer MPU6050 Z después de múltiples intentos.")


    def actualizar_yaw(self):
        """
        Calcula el cambio en Yaw desde la última llamada y lo acumula.
        Debe llamarse periódicamente y frecuentemente desde un hilo o bucle rápido.
        """
        if not self.initialized:
            # print("Warning: Gyro not initialized, cannot update Yaw.")
            return

        ahora_ticks = time.ticks_ms()
        # Calcular delta tiempo en segundos, usando ticks_diff para seguridad
        dt = time.ticks_diff(ahora_ticks, self.ultima_actualizacion_ticks) / 1000.0
        self.ultima_actualizacion_ticks = ahora_ticks

        # Evitar dt muy grandes o negativos si hubo pausa larga o error de tiempo
        if dt > 0.5 or dt <= 0:
            # print(f"[{self.sensor_id_for_log}] Warning: dt inválido o muy grande ({dt:.4f}s), omitiendo actualización de Yaw.")
            return

        try:
            # Leer valor raw, restar offset y escalar a grados/segundo
            raw_z = self._leer_raw_z()
            # Usar el factor de escala correcto para 500dps
            gyro_z_dps = (raw_z - self.offset_gyro_z) / self.SCALE_FACTOR_500DPS

            # Integración simple (Euler) para obtener el cambio en Yaw (grados)
            delta_yaw = gyro_z_dps * dt

            # Acumular el cambio en el Yaw total
            # El signo depende de la orientación física del MPU6050 en el robot.
            # Si gira a la derecha y el Yaw disminuye, invertir el signo aquí.
            self.yaw += delta_yaw
            # Ejemplo de inversión: self.yaw -= delta_yaw

            # (Opcional) Normalizar Yaw a un rango específico si se desea
            # self.yaw = self.yaw % 360
            # if self.yaw > 180: self.yaw -= 360

        except OSError as e:
             # El error ya se imprime en _leer_raw_z
             # print(f"[{self.sensor_id_for_log}] Error I2C omitiendo actualización de Yaw.")
             pass # Simplemente omitir esta actualización si falla la lectura
        except Exception as e:
             print(f"[{self.sensor_id_for_log}] Error inesperado durante actualización de Yaw: {e}")

    def get_yaw(self):
        """Retorna el ángulo Yaw acumulado actual en grados."""
        return self.yaw

    def reset_yaw(self, nuevo_valor=0.0):
        """Resetea el ángulo Yaw acumulado a un valor específico (por defecto 0)."""
        print(f"[{self.sensor_id_for_log}] Reseteando Yaw a {nuevo_valor:.2f}")
        self.yaw = nuevo_valor
