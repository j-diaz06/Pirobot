# giro_acel.py
import time
from machine import I2C, Pin, UART # <-- Importar UART si pasas el objeto

#ALPHA_LPF = 0.05
ALPHA_LPF = 0.5
num_muestras_cal = 2000 # Mantenemos las muestras altas de la iteración anterior

class GiroAcel:
    def __init__(self, i2c, debug_uart_obj=None): # Modificado
        
        self.debug_uart = debug_uart_obj # Guardar referencia
        
        self.i2c = i2c
        self.MPU_ADDR = 0x68
        self.G_R = 65.536
        self.GYRO_FULL_SCALE_500_DPS = 0x08
        self.promedio = 0.0
        self.yaw = 0.0
        self.ultima_lectura = time.ticks_ms()
        self.filtered_gyro_z = 0.0
        # --- NUEVO: Bandera para controlar la ejecución de actualizar_yaw ---
        self.calibrando = False
        # --- FIN NUEVO ---
        
        self.ultima_depuracion_yaw_ms = time.ticks_ms()
        
    def _debug(self, mensaje):
        if self.debug_uart:
            try:
                self.debug_uart.write(f"[Giro] {mensaje}\n")
            except: # Ignorar errores de UART aquí
                pass
        else: # Fallback a print si no hay UART
             self._debug(f"[Giro NO UART] {mensaje}")

    def iniciar_mpu(self):
        """Inicializa el MPU6050 y realiza la calibración inicial."""
        self.calibrando = True # Bloquear actualizaciones durante inicio
        try:
            self.i2c.writeto_mem(self.MPU_ADDR, 0x6B, bytes([0])) # Wake up
            time.sleep_ms(100)
            self.i2c.writeto_mem(self.MPU_ADDR, 0x1B, bytes([self.GYRO_FULL_SCALE_500_DPS])) # Set range
            time.sleep_ms(100)

#             self._debug("-----------Calibrando Giroscopio Inicial (No mover)------------")
#             suma = 0
#             self._debug(f"    Tomando {num_muestras_cal} muestras iniciales...")
#             tiempo_cal_inicio = time.ticks_ms()
#             for i in range(num_muestras_cal):
#                 suma += self._leer_raw_z()
#                 time.sleep_ms(2)
#             duracion_cal = time.ticks_diff(time.ticks_ms(), tiempo_cal_inicio)
# 
#             self.promedio = suma / num_muestras_cal
#             self._debug(f"Calibración inicial completa ({duracion_cal} ms). Offset Promedio: {self.promedio:.2f}")

            # --- Reiniciar estado después de calibración inicial ---
#             self.yaw = 0.0
#             self.filtered_gyro_z = 0.0
#             self.ultima_lectura = time.ticks_ms()
            # --- FIN Reinicio ---

        except Exception as e:
            self._debug(f"[Error Fatal] Iniciando MPU6050: {e}")
            raise
        finally:
            self.calibrando = False # Desbloquear actualizaciones

    def _calibrar(self):
        """Pausa actualizaciones, reinicia MPU, recalibra y resetea estado."""
        # --- NUEVO: Bloquear actualizaciones ---
        if self.calibrando: # Evitar llamadas recursivas o paralelas si ya está calibrando
             self._debug("Advertencia: Intento de recalibrar mientras ya se estaba calibrando.")
             return
        self.calibrando = True
        # --- FIN NUEVO ---
        self._debug(f"--- Iniciando Recalibración con Reinicio MPU (Offset Actual: {self.promedio:.2f}) ---")
        try:
            # --- Secuencia de Reinicio y Reconfiguración ---
            self._debug("    Enviando comando de reinicio al MPU...")
            self.i2c.writeto_mem(self.MPU_ADDR, 0x6B, bytes([0x80])) # Reset
            time.sleep_ms(100)
            self._debug("    Re-despertando MPU y configurando rango...")
            self.i2c.writeto_mem(self.MPU_ADDR, 0x6B, bytes([0])) # Wake up
            time.sleep_ms(100)
            self.i2c.writeto_mem(self.MPU_ADDR, 0x1B, bytes([self.GYRO_FULL_SCALE_500_DPS])) # Set range
            time.sleep_ms(100)

            self._debug("    Tomando muestras para nuevo offset...")
            suma = 0
            self._debug(f"    Tomando {num_muestras_cal} muestras...")
            tiempo_cal_inicio = time.ticks_ms()
            for _ in range(num_muestras_cal):
                # _leer_raw_z se llamará aquí, pero actualizar_yaw está bloqueado
                suma += self._leer_raw_z()
                time.sleep_ms(2)
            duracion_cal = time.ticks_diff(time.ticks_ms(), tiempo_cal_inicio)

            nuevo_promedio = suma / num_muestras_cal
            self.promedio = nuevo_promedio
            self._debug(f"--- Recalibración (post-reinicio) completa ({duracion_cal} ms). Nuevo Offset: {self.promedio:.2f} ---")

            # --- NUEVO: Reiniciar estado después de recalibración ---
            self._debug("    Reseteando Yaw acumulado, filtro y timestamp.")
            self.yaw = 0.0
            self.filtered_gyro_z = 0.0
            self.ultima_lectura = time.ticks_ms()
            # --- FIN NUEVO ---

        except Exception as e:
            self._debug(f"[Error] Durante recalibración con reinicio: {e}")
            # ... (manejo de error como antes) ...
            try:
                self.i2c.writeto_mem(self.MPU_ADDR, 0x6B, bytes([0]))
                time.sleep_ms(50)
                self.i2c.writeto_mem(self.MPU_ADDR, 0x1B, bytes([self.GYRO_FULL_SCALE_500_DPS]))
            except Exception as e_recovery:
                 print(f"[Error] Intentando recuperar MPU post-fallo: {e_recovery}")
        finally:
            # --- NUEVO: Asegurar desbloqueo de actualizaciones ---
            self.calibrando = False
            # --- FIN NUEVO ---


    def _leer_raw_z(self):
        # ... (sin cambios) ...
        max_retries = 3
        for attempt in range(max_retries):
            try:
                data = self.i2c.readfrom_mem(self.MPU_ADDR, 0x47, 2)
                value = (data[0] << 8) | data[1]
                if value > 32767:
                    value -= 65536
                return value
            except OSError as e:
                # No imprimir error si estamos calibrando, es esperado durante reinicio
                if not self.calibrando:
                    self._debug(f"Error I2C leyendo Giro Z (intento {attempt+1}): {e}")
                time.sleep_ms(10)
        # Si falla después de reintentos, devolver 0 para no romper actualizar_yaw,
        # pero imprimir error si NO estamos calibrando
        if not self.calibrando:
            self._debug("ERROR: Fallo al leer Giro Z después de varios intentos, devolviendo 0.")
        return 0 # Devolver 0 en caso de fallo persistente


    def actualizar_yaw(self):
        """Lee el giroscopio, aplica filtro y actualiza el ángulo Yaw, si no se está calibrando."""
        # --- NUEVO: Comprobar bandera antes de hacer nada ---
        if self.calibrando:
            return # No hacer nada si se está calibrando/reiniciando
        # --- FIN NUEVO ---

        ahora = time.ticks_ms()
        dt = time.ticks_diff(ahora, self.ultima_lectura) / 1000.0
        #if dt <= 0 or dt > 0.5:
        #     dt = 0.01

        self.ultima_lectura = ahora

        try:
            # _leer_raw_z ahora maneja errores y devuelve 0 si falla persistentemente
            raw_z = self._leer_raw_z()
            gyro_z = (raw_z - self.promedio) / self.G_R

            self.filtered_gyro_z = ALPHA_LPF * gyro_z + (1.0 - ALPHA_LPF) * self.filtered_gyro_z
            incremento_yaw = self.filtered_gyro_z * dt
            self.yaw += incremento_yaw

            # --- DEBUG DETALLADO (sin cambios) ---
            if time.ticks_diff(ahora, self.ultima_depuracion_yaw_ms) >= 100000000:
                self._debug(f"[YawDebug] dt={dt:.3f} rawZ={raw_z} offs={self.promedio:.1f} gyroZ={gyro_z:.2f} filtZ={self.filtered_gyro_z:.2f} incrYaw={incremento_yaw:.3f} TOTAL_YAW={self.yaw:.1f}")
                self.ultima_depuracion_yaw_ms = ahora

        # except OSError as e: # _leer_raw_z ahora maneja OSError internamente
        #     print(f"Error OS en actualizar_yaw omitiendo lectura: {e}")
        except Exception as e:
             self._debug(f"Error inesperado en actualizar_yaw: {e}")