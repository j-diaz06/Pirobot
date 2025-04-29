# maquina.py (Versión Completa con debug_serial y correcciones)
import time
from machine import Pin, PWM, I2C, UART # Asegúrate de importar UART
import _thread
import json

from config import *

# --- Importar Librerías ---
from giro_acel import GiroAcel
from motor import Motor # Asume que tiene set_pwm_duties(a,b)
from sensor import SensorVL53L0X
from mqtt_pirobot import mqtt_pirobot
from matriz import MatrizWS2812B


class Maquina:
    # --- MODIFICADO: Añadir debug_uart_obj al constructor ---
    def __init__(self, ENA, IN1, IN2, ENB, IN3, IN4, # Motor
                 direccion_trasero, xshut_trasero,      # Sensor T.
                 direccion_delantero, xshut_delantero,  # Sensor D.
                 SSID, PASSWORD, MQTT_BROKER, CLIENT_ID_BASE, # Red/MQTT
                 MI_ID, # Identificación
                 debug_uart_obj): # <--- AÑADIDO

        self.debug_uart = debug_uart_obj

        self.debug_serial("[Init] Configurando hardware...") # <-- Usar debug_serial
        self.motores = Motor(ENA, IN1, IN2, ENB, IN3, IN4)
        self.motores.velocidad = VELOCIDAD_MOVIMIENTO # Usar constante definida arriba
        self.i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
        self.debug_serial(f"[Init] Bus I2C creado: {self.i2c}") # <-- Usar debug_serial
        self.debug_serial("[Init] Creando sensor trasero...") # <-- Usar debug_serial
        self.sensor_trasero = SensorVL53L0X(direccion_trasero, xshut_trasero, i2c_object=self.i2c)
        self.debug_serial("[Init] Creando sensor delantero...") # <-- Usar debug_serial
        self.sensor_delantero = SensorVL53L0X(direccion_delantero, xshut_delantero, i2c_object=self.i2c)
        self.debug_serial("[Init] Creando Giroscopio/Acelerómetro...") # <-- Usar debug_serial
        # --- CORREGIDO: Pasar self.debug_uart a GiroAcel ---
        self.giro_acel = GiroAcel(self.i2c, self.debug_uart) # <--- PASAR UART
        # --- FIN CORRECCIÓN ---
        self.debug_serial("[Init] Hardware configurado.") # <-- Usar debug_serial
        self.MI_ID = MI_ID
        self.posicion_actual = None
        self.ultimo_comando_ejecutado = {}
        self.estado_sistema = ESTADO_INICIALIZANDO
        self.comando_actual_pendiente = None
        mqtt_client_id = f"{CLIENT_ID_BASE}_{MI_ID}"
        self.debug_serial(f"[Init] Configurando MQTT con ClientID: {mqtt_client_id}")
        # Pasar self.debug_uart si mqtt_pirobot lo necesita para depurar internamente
        self.mqtt = mqtt_pirobot(SSID, PASSWORD, MQTT_BROKER, mqtt_client_id)
        self.girando = False
        self.motores_activos = False
        self.espera_2nucleo = 0
        self.debug_timer = 0

        self.matriz = MatrizWS2812B(pin_matriz, brillo=0)
        self.debug_serial(f"[Init] Matriz configurada en pin {pin_matriz}.")

    # --- NUEVO: Función de Depuración Serial ---
    def debug_serial(self, mensaje):#revisado
        """Envía un mensaje de depuración por el UART configurado."""
        if self.debug_uart:
            try:
                # Añadir timestamp opcional
                # timestamp = f"[{time.ticks_ms()}] "
                # self.debug_uart.write(timestamp + str(mensaje) + '\n')
                self.debug_uart.write(str(mensaje) + '\n')
            except Exception as e:
                 # Fallback a print si falla el UART (solo para errores críticos de debug)
                 print(f"[UART DEBUG FAIL] {mensaje} (Error: {e})")
        else:
             # Fallback si no se pasó UART al inicializar Maquina
             print(f"[DEBUG NO UART] {mensaje}")
    """
    # --- Métodos de Movimiento (Usando debug_serial) ---
    def mover_hacia(self, direccion):#revisado
        self.debug_serial(f"    Moviendo hacia: {direccion}") # <-- Usar debug_serial
        self.motores_activos = True
        if direccion == "adelante": self.motores.avanzar(VELOCIDAD_MOVIMIENTO) # Usar constante
        elif direccion == "atras": self.motores.retroceder(VELOCIDAD_MOVIMIENTO) # Usar constante"""

    def detener(self):#revisado
        # self.debug_serial("   Intentando detener motores...") # Opcional
        if not self.motores_activos: return
        self.motores.detener()
        self.motores_activos = False

    def normalizar_angulo(self, angulo):
        """Asegura que un ángulo esté en el rango (-180, 180]."""
        while angulo <= -180: angulo += 360
        while angulo > 180: angulo -= 360
        return angulo

    def girar(self, grados_giro):
        self.debug_serial(f"   Iniciando giro PID relativo: {grados_giro:.1f}°")

        # --- IMPORTANTE: Calcular el Yaw Objetivo Absoluto ---
        yaw_inicial = self.giro_acel.yaw # Leer yaw actual ANTES de empezar
        yaw_objetivo_absoluto = self.normalizar_angulo(yaw_inicial + grados_giro)
        self.debug_serial(f"      Yaw Inicial: {yaw_inicial:.1f}°, Objetivo Absoluto: {yaw_objetivo_absoluto:.1f}°")

        self.motores_activos = True
        self.girando = True

        # --- Inicialización PID ---
        pid_integral = 0.0
        ultimo_error = 0.0
        ultima_vez = time.ticks_ms()

        # Bucle de control PID
        while True:
            ahora = time.ticks_ms()
            dt = time.ticks_diff(ahora, ultima_vez) / 1000.0 # Tiempo en segundos
            # Evitar dt inválido en el primer ciclo o si hay pausas largas
            if dt <= 0:
                dt = 0.01 # Usar un dt pequeño por defecto si es necesario
            ultima_vez = ahora

            # --- Leer Yaw Actual ---
            yaw_actual = self.giro_acel.yaw # Ya está normalizado por la tarea yaw_task (asumiendo)

            # --- Calcular Error (manejando el cruce -180/180) ---
            error = self.normalizar_angulo(yaw_objetivo_absoluto - yaw_actual)

            # --- Condición de Parada ---
            # Detener si el error es muy pequeño Y la velocidad angular es baja (opcional pero recomendado)
            velocidad_angular_actual = self.giro_acel.filtered_gyro_z # Usar el valor filtrado
            if abs(error) <= PID_GIRO_TOLERANCIA_DEG and abs(velocidad_angular_actual) < 1.0: # Umbral de velocidad angular (ajustar)
                self.debug_serial(f"   ¡Giro PID detenido! Error: {error:.1f}°, VelAng: {velocidad_angular_actual:.1f} deg/s")
                break

            # --- Calcular Términos PID ---
            # Integral (con anti-windup)
            pid_integral += error * dt
            pid_integral = max(PID_GIRO_MIN_INTEGRAL, min(PID_GIRO_MAX_INTEGRAL, pid_integral))

            # Derivativo
            derivativo = 0
            if dt > 0:
                derivativo = (error - ultimo_error) / dt

            # --- Calcular Salida PID ---
            pid_output = (KP_GIRO * error) + (KI_GIRO * pid_integral) + (KD_GIRO * derivativo)

            # --- Mapear Salida PID a Velocidad de Motores ---
            # La salida PID indica la "fuerza" y dirección del giro.
            # Positivo podría ser girar izquierda, negativo derecha (o viceversa, depende de tu robot)

            # Limitar la salida PID para que no genere velocidades excesivas
            velocidad_giro_calculada = int(abs(pid_output))
            velocidad_giro_calculada = max(VELOCIDAD_GIRO_MIN, min(VELOCIDAD_GIRO_MAX, velocidad_giro_calculada))

            # Determinar dirección y aplicar velocidad
            if pid_output > 0: # Asumimos que positivo es girar IZQUIERDA
                self.motores.girar_izquierda(velocidad_giro_calculada)
                direccion_giro = "Izquierda"
            elif pid_output < 0: # Negativo es girar DERECHA
                self.motores.girar_derecha(velocidad_giro_calculada)
                direccion_giro = "Derecha"
            else: # Error cero, intentar detenerse (aunque la condición de parada debería actuar antes)
                self.detener()
                direccion_giro = "Detenido (error 0)"
                # Podríamos salir aquí también si el error es exactamente 0
                # if abs(error) <= PID_GIRO_TOLERANCIA_DEG: break

            # --- Actualizar estado para la siguiente iteración ---
            ultimo_error = error

            # --- Debugging (Opcional, pero útil para tuning) ---
            # No imprimir en cada ciclo, hacerlo cada cierto tiempo
            if time.ticks_diff(time.ticks_ms(), self.debug_timer) > 200: # Imprimir cada 200ms
               self.debug_serial(f"      PID Giro: Err={error:.1f}, P={KP_GIRO*error:.0f}, I={KI_GIRO*pid_integral:.0f}, D={KD_GIRO*derivativo:.0f} -> Out={pid_output:.0f} -> Vel={velocidad_giro_calculada} ({direccion_giro})")
               self.debug_timer = time.ticks_ms() # Resetear timer de debug

            time.sleep_ms(1) # Pequeña pausa para no saturar CPU y permitir otras tareas (ajustar)

        # --- Fin del Bucle PID ---
        self.detener()
        self.girando = False
        yaw_final = self.giro_acel.yaw
        error_final = self.normalizar_angulo(yaw_objetivo_absoluto - yaw_final)
        self.debug_serial(f"   Giro PID completado. Yaw final: {yaw_final:.1f}°, Error final: {error_final:.1f}°")
        # --- IMPORTANTE: Recalibrar después de un giro puede ser necesario ---
        # self.giro_acel._calibrar() # Considera si necesitas esto aquí o antes del siguiente movimiento

    def ejecutar_movimiento(self, destino_str):# revisado
        """Mueve el robot a la celda destino, asegurando estabilidad yaw antes de cada tramo."""
        self.debug_serial(f"[Accion] Iniciando secuencia de movimiento hacia: {destino_str}") # <-- Usar debug_serial
        if not self.posicion_actual:
            self.debug_serial("[Error] Posición actual desconocida.") # <-- Usar debug_serial
            return
        try:
            current_x, current_y = map(float, self.posicion_actual.split(','))
            target_x, target_y = map(float, destino_str.split(','))
        except Exception as e:
            self.debug_serial(f"[Error] Formato coordenadas inválido: {e}") # <-- Usar debug_serial
            return
        self.debug_serial(f"        Desde: ({current_x:.1f}, {current_y:.1f}) Hacia: ({target_x:.1f}, {target_y:.1f})") # <-- Usar debug_serial
        dx_celdas = target_x - current_x
        dy_celdas = target_y - current_y
        movimiento_exitoso_x = True
        movimiento_exitoso_y = True
        hubo_movimiento_y = False


        # 1. Mover en Eje Y
        if abs(dy_celdas) > 0.01:
            hubo_movimiento_y = True
            distancia_mm_y = dy_celdas * CELDA_SIZE_MM
            self.debug_serial(f"--- Movimiento Eje Y ---") # <-- Usar debug_serial

            # --- Bucle de verificación de estabilidad Yaw Y ---
            self.debug_serial("   Verificando estabilidad Yaw antes de mover en Y...") # <-- Usar debug_serial
            self._verificar_estabilidad_yaw(UMBRAL_DERIVA_YAW_GPS, DURACION_CHECK_YAW_MS)
            # --- FIN Bucle ---

            movimiento_exitoso_y = self.mover_distancia_con_sensor(distancia_mm_y, self.giro_acel.yaw) # mover_distancia usa debug_serial
            if not movimiento_exitoso_y: self.debug_serial("   Fallo en movimiento Y.") # <-- Usar debug_serial

            """"
            # --- Bucle de verificación de estabilidad ---
            self.debug_serial("   Verificando estabilidad Yaw antes de giro en Y...") # <-- Usar debug_serial
            self._verificar_estabilidad_yaw(UMBRAL_DERIVA_YAW_GPS, DURACION_CHECK_YAW_MS)
            # --- FIN Bucle ---

            # Objetivo Yaw: Adelante (+Y) -> 0 grados; Atrás (-Y) -> 180 grados
            yaw_objetivo_recto_y = 0.0 if dy_celdas > 0 else 180.0
            yaw_absoluto_estimado_actual = self.giro_acel.yaw
            angulo_giro_y = yaw_objetivo_recto_y - yaw_absoluto_estimado_actual

            # Ajuste para el caso 180/-180 (ya estaba en tu snippet)
            if abs(yaw_objetivo_recto_y) == 180.0:
                 if yaw_absoluto_estimado_actual < -GIRO_TOLERANCIA_DEG : # Objetivo -180
                     angulo_giro_y = -180.0 - yaw_absoluto_estimado_actual
                 else: # Objetivo +180
                     angulo_giro_y = 180.0 - yaw_absoluto_estimado_actual
                 # Normalizar ángulo +/- 180
            while angulo_giro_y > 180: angulo_giro_y -= 360
            while angulo_giro_y <= -180: angulo_giro_y += 360
#             else: # Objetivo 0.0
#                  while angulo_giro_y > 180: angulo_giro_y -= 360
#                  while angulo_giro_y <= -180: angulo_giro_y += 360

            if angulo_giro_y > 0: # Si el giro más corto es a la izquierda...
                self.debug_serial(f"   Forzando giro a la derecha para ángulo izquierdo: {angulo_giro_y:.1f}°")
                angulo_giro_y -= 90.0 # ...convertir a giro largo a la derecha
                self.debug_serial(f"   Nuevo ángulo (derecha): {angulo_giro_y:.1f}°")

            self.debug_serial(f"   DEBUG: Checking Giro Y: abs({angulo_giro_y:.2f}) > {GIRO_TOLERANCIA_DEG:.2f} ?") # <-- Usar debug_serial
            if abs(angulo_giro_y) > GIRO_TOLERANCIA_DEG:
                self.girar(angulo_giro_y) # girar usa debug_serial
                self.debug_serial(f"   Pausa post-giro Y ({PAUSA_ENTRE_MOVIMIENTOS_MS} ms)...") # <-- Usar debug_serial
                time.sleep_ms(PAUSA_ENTRE_MOVIMIENTOS_MS)
            else:
                self.debug_serial(f"   Giro Y no necesario (Ángulo: {angulo_giro_y:.1f}° <= Tolerancia: {GIRO_TOLERANCIA_DEG:.1f}°)") # <-- Usar debug_serial
            """
            
        # Pausa post-movimiento Y (si hubo)
        if hubo_movimiento_y:
             self.debug_serial(f"   Pausa post-movimiento X ({PAUSA_ENTRE_MOVIMIENTOS_MS} ms)...") # <-- Usar debug_serial
             time.sleep_ms(PAUSA_ENTRE_MOVIMIENTOS_MS)

        # 2. Mover en Eje X
        if abs(dx_celdas) > 0.01 and movimiento_exitoso_y:
            # --- Bucle de verificación de estabilidad ---
            self.debug_serial("   Verificando estabilidad Yaw antes de giro en X...") # <-- Usar debug_serial
            self._verificar_estabilidad_yaw(UMBRAL_DERIVA_YAW_GPS, DURACION_CHECK_YAW_MS)
            # --- FIN Bucle ---
            
            # Objetivo Yaw: Derecha (+X) -> -90 grados; Izquierda (-X) -> +90 grados
            yaw_objetivo_recto_x = -90.0 if dx_celdas > 0 else 90.0
            yaw_absoluto_actual_antes_giro = self.giro_acel.yaw
            angulo_giro_x = yaw_objetivo_recto_x - yaw_absoluto_actual_antes_giro
            # Normalizar ángulo de giro a +/- 180 grados
            #while angulo_giro_x > 180: angulo_giro_x -= 360
            #while angulo_giro_x <= -180: angulo_giro_x += 360
            
            if angulo_giro_x > 0: # Si el giro más corto es a la izquierda...
                self.debug_serial(f"   Forzando giro a la derecha para ángulo izquierdo: {angulo_giro_x:.1f}°")
                angulo_giro_x -= 180.0 # ...convertir a giro largo a la derecha
                self.debug_serial(f"   Nuevo ángulo (derecha): {angulo_giro_x:.1f}°")

            self.debug_serial(f"   DEBUG: Checking Giro X: abs({angulo_giro_x:.2f}) > {GIRO_TOLERANCIA_DEG:.2f} ?") # <-- Usar debug_serial
            if abs(angulo_giro_x) > GIRO_TOLERANCIA_DEG:
                self.girar(angulo_giro_x) # girar usa debug_serial internamente
                self.debug_serial(f"   Pausa post-giro X ({PAUSA_ENTRE_MOVIMIENTOS_MS} ms)...") # <-- Usar debug_serial
                time.sleep_ms(PAUSA_ENTRE_MOVIMIENTOS_MS)
            else:
                self.debug_serial(f"   Giro X no necesario (Ángulo: {angulo_giro_x:.1f}° <= Tolerancia: {GIRO_TOLERANCIA_DEG:.1f}°)") # <-- Usar debug_serial
            
            
            distancia_mm_x = dx_celdas * CELDA_SIZE_MM
            self.debug_serial(f"--- Movimiento Eje X ---") # <-- Usar debug_serial

            # --- Bucle de verificación de estabilidad ---
            self.debug_serial("   Verificando estabilidad Yaw antes de giro en X...") # <-- Usar debug_serial
            self._verificar_estabilidad_yaw(UMBRAL_DERIVA_YAW_GPS, DURACION_CHECK_YAW_MS)
            # --- FIN Bucle ---

            movimiento_exitoso_x = self.mover_distancia_con_sensor(distancia_mm_x, self.giro_acel.yaw) # mover con yaw 0 ya que despues de la recalibracion este se reinicia a 0
            if not movimiento_exitoso_x: self.debug_serial("   Fallo en movimiento X.") # <-- Usar debug_serial   

            # --- Bucle de verificación de estabilidad ---
            self.debug_serial("   Verificando estabilidad Yaw antes de giro en X...") # <-- Usar debug_serial
            self._verificar_estabilidad_yaw(UMBRAL_DERIVA_YAW_GPS, DURACION_CHECK_YAW_MS)
            # --- FIN Bucle ---

            self.girar(-270)# alinear con el eje Y


        # 3. Actualizar posición final
        if movimiento_exitoso_x and movimiento_exitoso_y:
            self.posicion_actual = destino_str
            self.debug_serial(f"[Accion] Secuencia hacia {destino_str} completada.") # <-- Usar debug_serial
            self.debug_serial(f"[Estado] Nueva posición: {self.posicion_actual}") # <-- Usar debug_serial
        else:
            self.debug_serial(f"[Accion] Secuencia hacia {destino_str} NO completada exitosamente.") # <-- Usar debug_serial
        #self.controlar_led("apagado") # controlar_led usa debug_serial


    def mover_distancia_con_sensor(self, distancia_mm, yaw_setpoint):# revisado
        
        adelante = distancia_mm > 0
        sensor_a_usar = self.sensor_delantero if adelante else self.sensor_trasero
        distancia_absoluta_mm = abs(distancia_mm)

        self.debug_serial(f"   Moviendo {'adelante' if adelante else 'atras'} por {distancia_absoluta_mm} mm manteniendo Yaw={yaw_setpoint:.1f}°...") # <-- Usar debug_serial
        distancia_inicial_promedio = self._leer_sensor_promedio(sensor_a_usar, 5) # promedio de 5 lecturas iniciales ya que no afecta al pid
        self.debug_serial(f"      Lectura INICIAL PROMEDIO sensor {'frontal' if adelante else 'trasero'}: {distancia_inicial_promedio} mm") # <-- Usar debug_serial
        
        if distancia_inicial_promedio < 0:
            self.debug_serial(f"[Error Sensor] Lecturas iniciales inválidas. Abortando.")
            self.detener()
            return False
        
        distancia_objetivo_sensor = distancia_inicial_promedio - distancia_absoluta_mm #esto valido para ambos sentidos de movimiento
        
        if adelante and distancia_objetivo_sensor < MIN_TARGET_DIST_MM:
             distancia_objetivo_sensor = MIN_TARGET_DIST_MM
             self.debug_serial(f"     Objetivo distancia ajustado por seguridad a: {distancia_objetivo_sensor} mm") # <-- Usar debug_serial
        
        self.debug_serial(f"      Distancia objetivo del sensor: {distancia_objetivo_sensor:.0f} mm") # <-- Usar debug_serial
        # Inicializar PID
        pid_integral = 0.0; pid_last_error = 0.0; pid_last_time = time.ticks_ms()

        # Establecer dirección inicial
        if adelante: self.motores.avanzar(VELOCIDAD_MOVIMIENTO)#self.motores.in1.value(0); self.motores.in2.value(1); self.motores.in3.value(1); self.motores.in4.value(0)
        else: self.motores.retroceder(VELOCIDAD_MOVIMIENTO)#self.motores.in1.value(1); self.motores.in2.value(0); self.motores.in3.value(0); self.motores.in4.value(1)
        self.motores_activos = True

        movimiento_completado = False

        # Bucle PID y distancia
        while True:
            ahora = time.ticks_ms()

            # --- 1. Leer Sensor (Rápido) ---
            distancia_actual = self._leer_sensor_promedio(sensor_a_usar, NUM_LECTURAS_SENSOR_MOVE)

            if distancia_actual < 0:
                # Si hay error de lectura, podríamos saltar este ciclo PID o detenernos
                self.debug_serial("[Warn] Error lectura sensor durante movimiento, saltando ciclo PID.")
                time.sleep_ms(1) # Pequeña pausa antes de reintentar
                pid_last_time = ahora # Evitar dt grande en el siguiente ciclo
                continue # Saltar el resto del bucle

            # --- 2. Comprobar Condición de Parada ---
            detener_movimiento = False

            if distancia_actual <= distancia_objetivo_sensor:
                detener_movimiento = True
            

            if detener_movimiento:
                self.debug_serial(f"\n   ¡Detenido por sensor! Dist final: {distancia_actual} mm (Obj: {distancia_objetivo_sensor:.0f})")
                movimiento_completado = True
                break # Salir del bucle while

            # --- 3. Calcular PID para Corrección de Yaw ---
            dt_pid = time.ticks_diff(ahora, pid_last_time) / 1000.0
            # Validar dt para evitar cálculos erróneos
            if dt_pid <= 1e-6 or dt_pid > 0.5: # Si dt es cero, negativo o muy grande
                 dt_pid = 0.02 # Usar un valor por defecto razonable (ej. 20ms)
            pid_last_time = ahora

            yaw_actual = self.giro_acel.yaw
            error = yaw_setpoint - yaw_actual
            # Normalizar error a +/- 180 grados
            while error > 180.0: error -= 360.0
            while error <= -180.0: error += 360.0

            # Término Integral con anti-windup
            pid_integral += error * dt_pid
            pid_integral = max(PID_MOVE_MIN_INTEGRAL, min(PID_MOVE_MAX_INTEGRAL, pid_integral))

            # Término Derivativo (solo si dt es válido)
            derivative = 0
            if dt_pid > 1e-6: # Evitar división por cero
                derivative = (error - pid_last_error) / dt_pid
            pid_last_error = error

            # Calcular corrección PID usando las ganancias de movimiento
            pid_correction = (KP_MOVE * error) + (KI_MOVE * pid_integral) + (KD_MOVE * derivative)

            # --- 4. Aplicar Corrección a los Motores ---
            base_pwm_duty = VELOCIDAD_MOVIMIENTO

            # Calcular PWMs finales (lógica de avance/retroceso mantenida)
            if adelante:
                pwm_A = base_pwm_duty - pid_correction # Izquierda
                pwm_B = base_pwm_duty + pid_correction # Derecha
            else: # Hacia atrás
                pwm_A = base_pwm_duty + pid_correction # Izquierda
                pwm_B = base_pwm_duty - pid_correction # Derecha

            # Limitar PWM (manteniendo mínimo y máximo)
            pwm_A = max(MIN_PWM_MOVE, min(65535, int(pwm_A)))
            pwm_B = max(MIN_PWM_MOVE, min(65535, int(pwm_B)))

            self.motores.set_pwm_duties(pwm_A, pwm_B)

            # --- 5. Debugging (simplificado a una línea por ciclo) ---
            debug_msg = (f"     Dist:{distancia_actual}(Obj:{distancia_objetivo_sensor:.0f})"
                         f"|YawErr:{error:.1f},PID:{pid_correction:.0f}"
                         f"|PWM A:{pwm_A},B:{pwm_B}|dt:{dt_pid:.3f}")
            # CUIDADO: Imprimir en cada ciclo puede ralentizar o saturar la comunicación.
            # Considera imprimir cada N ciclos si es necesario.
            self.debug_serial(debug_msg)

        self.detener()
        return movimiento_completado


    # --- Método Verificar estabilidad Yaw (Usando debug_serial) ---
    def _verificar_estabilidad_yaw(self, umbral_gps, duracion_ms, pausa_reintento_ms=500):# revisado

        self.giro_acel._calibrar()

        self.debug_serial("--- Iniciando proceso de estabilidad Yaw ---")

        while True: # Bucle hasta que confirmemos estabilidad
            # --- Verificación inicial (igual que en la función original) ---
            if self.motores_activos:
                self.debug_serial("  Advertencia: Intentando verificar estabilidad Yaw con motores activos.")
                break # Salir del bucle (y de la función) si los motores están activos

            self.debug_serial(f"  Verificando deriva Yaw por {duracion_ms} ms (Umbral: {umbral_gps} deg/s)...")
            yaw_inicio = self.giro_acel.yaw
            time_inicio = time.ticks_ms()

            # --- Esperar ---
            while time.ticks_diff(time.ticks_ms(), time_inicio) < duracion_ms:
                time.sleep_ms(10) # Espera mínima

            yaw_fin = self.giro_acel.yaw
            tiempo_transcurrido_ms = time.ticks_diff(time.ticks_ms(), time_inicio)
            # Usar duracion_ms como respaldo si el tiempo medido es inválido o cero
            tiempo_transcurrido_s = (tiempo_transcurrido_ms / 1000.0) if tiempo_transcurrido_ms > 0 else (duracion_ms / 1000.0)

            # --- Calcular deriva ---
            deriva_abs = abs(yaw_fin - yaw_inicio)
            # Manejar el cruce de 180/-180 grados
            if deriva_abs > 180.0:
                deriva_abs = 360.0 - deriva_abs

            tasa_deriva_gps = deriva_abs / tiempo_transcurrido_s if tiempo_transcurrido_s > 0 else 0

            self.debug_serial(f"  Check Yaw: Inicio={yaw_inicio:.1f}, Fin={yaw_fin:.1f}, Deriva={deriva_abs:.1f}° en {tiempo_transcurrido_s:.2f}s -> Tasa={tasa_deriva_gps:.2f} deg/s")

            # --- Evaluar estabilidad ---
            if tasa_deriva_gps <= umbral_gps:
                # ¡Estable! Salir del bucle y de la función
                self.debug_serial("  Yaw estable.")
                break # Rompe el bucle while
            else:
                # ¡Inestable! Recalibrar y preparar reintento
                self.debug_serial(f"  ¡ALERTA! Deriva de Yaw detectada ({tasa_deriva_gps:.2f} deg/s > {umbral_gps:.2f} deg/s). Recalibrando...")
                self.giro_acel._calibrar()

                # Mensaje y pausa antes del siguiente intento
                self.debug_serial(f"  Inestabilidad detectada. Re-verificando estabilidad en {pausa_reintento_ms} ms...")
                time.sleep_ms(pausa_reintento_ms)
                # El bucle 'while True' continuará con la siguiente verificación

        # --- Fin del proceso ---
        self.debug_serial("--- Verificación de estabilidad Yaw completada ---")

    def controlar_matriz(self, figura=None):
        self.debug_serial(f"[Matriz] Controlando figura: {figura}")
        if figura == "cara_feliz":
            self.matriz.ajustar_brillo(brillo_matriz)
            self.matriz.mostrar_matriz_grafica(cara_feliz, color) # verde encendido
            self.matriz.mostrar()
        elif figura == "identificar":
            self.matriz.ajustar_brillo(brillo_matriz)
            self.matriz.llenar_matriz(color)  # verde
            self.matriz.mostrar()
        elif figura == "apagar":
            self.matriz.apagar_matriz()
            self.matriz.mostrar()
        else:
            self.debug_serial(f"[Error] Figura no reconocida: {figura}")

    def yaw_task(self):# revisado
        """Tarea en segundo plano SOLO para actualizar el Yaw."""
        self.debug_serial("[Yaw Task] Iniciada.") # <-- Usar debug_serial
        while True:
            #if time.ticks_ms() - self.espera_2nucleo > 10: # Intervalo mínimo entre lecturas
            try:
                # actualizar_yaw está protegido contra ejecución durante calibración
                self.giro_acel.actualizar_yaw()
            except Exception as e:
                    self.debug_serial(f"[Error Yaw Task] {e}") # <-- Usar debug_serial
                    # Considera si el hilo debe detenerse o solo esperar en caso de error grave
                    time.sleep_ms(100)
            #self.espera_2nucleo = time.ticks_ms()
            time.sleep_ms(5) # Ceder tiempo a otros procesos/hilos

    def iniciar(self):# revisado
        self.debug_serial(f"[Maquina] Inicializando vehículo ID: {self.MI_ID}") # <-- Usar debug_serial

        self.controlar_matriz("apagar") # Apagar matriz al iniciar

        try:
            # iniciar_mpu usará el debug_uart pasado en __init__
            self.giro_acel.iniciar_mpu()
            self.debug_serial("[Giro] MPU inicializado y calibrado.") # <-- Usar debug_serial
        except Exception as e:
             # El MPU es crítico, detener si falla
             self.debug_serial(f"[Error Fatal] MPU6050: {e}. Deteniendo.") # <-- Usar debug_serial
             # sys.print_exception(e, self.debug_uart_stream) # Opcional
             return
            
        try:
             _thread.start_new_thread(self.yaw_task, ())
             self.debug_serial("[Init] Hilo para lectura de Yaw iniciado.")
        except Exception as e:
             self.debug_serial(f"[Error Fatal] Iniciando hilo Yaw: {e}")
             return # No continuar si el hilo falla

        # Conexión WiFi y MQTT (asume que mqtt_pirobot imprime sus propios mensajes)
        if self.mqtt.connect_wifi():
            self.debug_serial("[WiFi] Conectado.") # <-- Usar debug_serial
        else:
            self.debug_serial("[Error] Fallo conexión WiFi. Deteniendo.") # <-- Usar debug_serial
            return

        if self.mqtt.connect_mqtt():
            self.debug_serial("[MQTT] Conectado al broker.") # <-- Usar debug_serial
        else:
            self.debug_serial("[Error] Fallo conexión MQTT. Deteniendo.") # <-- Usar debug_serial
            return

        # Suscripciones MQTT
        topic_inicial = b'vehiculos/posicion_inicial'
        topic_comando = b'vehiculos/comando'
        try:
            self.mqtt.set_callback(topic_inicial, self)
            self.mqtt.set_callback(topic_comando, self)
            self.debug_serial(f"[MQTT] Suscrito inicialmente a {topic_inicial.decode()} y {topic_comando.decode()}") # <-- Usar debug_serial
        except Exception as e:
            self.debug_serial(f"[Error MQTT] Suscribiendose: {e}") # <-- Usar debug_serial
            return

        self.estado_sistema = ESTADO_INICIALIZANDO
        #self.controlar_led("encender", color="verde")
        self.debug_serial("[Estado] Esperando posición inicial...") # <-- Usar debug_serial
        self.debug_timer = time.ticks_ms()


    def mqtt_recibido(self, topic_str, msg_str):# revisado
        self.debug_serial(f"[MQTT Rx] Topic: '{topic_str}', Msg: '{msg_str}'") # <-- Usar debug_serial
        try:
            comando_actual = json.loads(msg_str) # Decodificar JSON
            if 'id_vehiculo' not in comando_actual:
                self.debug_serial("[MQTT Warn] Mensaje sin 'id_vehiculo', ignorando.") # <-- Usar debug_serial
                return
            id_vehiculo_msg = comando_actual['id_vehiculo']

            # Ignorar mensajes para otros vehículos
            if id_vehiculo_msg != self.MI_ID:
                self.debug_serial(f"[MQTT Info] Mensaje para otro ID ({id_vehiculo_msg}), ignorando.") # Opcional
                return
            
            # extraer el comando de acción, excluyendo 'id_vehiculo'
            comando_accion = {k: v for k, v in comando_actual.items() if k != 'id_vehiculo'}

            # --- Fase Inicialización ---
            if self.estado_sistema == ESTADO_INICIALIZANDO:
                if topic_str == 'vehiculos/posicion_inicial':
                    if 'ubicacion' in comando_actual:
                        self.posicion_actual = comando_actual['ubicacion']
                        self.debug_serial(f"[Init] Posición inicial recibida: {self.posicion_actual}") # <-- Usar debug_serial
                        
                        self.estado_sistema = ESTADO_OPERANDO
                        self.ultimo_comando_ejecutado = {} # Resetear último comando
                        self.debug_serial("[Estado] Cambiado a modo OPERACIÓN.") # <-- Usar debug_serial
                    else:
                        self.debug_serial("[MQTT Warn] Mensaje de posición inicial sin 'ubicacion'.") # <-- Usar debug_serial
            
                elif topic_str == 'vehiculos/comando':
                    # Aceptar solo comandos de matriz en este estado
                    accion = comando_accion.get('accion')
                    if accion == 'mostrar':
                        fig = comando_accion.get('figura', "identificar") # Usar por defecto si no se especifica
                        self.controlar_matriz(fig)
                        self.debug_serial(f"[Init-Comando] Ejecución visualización '{fig}' finalizada.")
                        # No guardar como último comando ejecutado para permitir repetirlo si es necesario
                    elif accion == 'apagar':
                        self.controlar_matriz("apagar")
                        self.debug_serial(f"[Init-Comando] Ejecución apagar matriz finalizada.")
                    elif 'destino' in comando_accion:
                        self.debug_serial("[Init-Comando Warn] Comando de movimiento ignorado en estado INICIALIZANDO.")
                    else:
                        self.debug_serial(f"[Init-Comando Error] Acción no reconocida o no permitida: {comando_accion}")

            # --- Fase Operación ---
            elif self.estado_sistema == ESTADO_OPERANDO:
                if topic_str == 'vehiculos/comando':
                    # Evitar ejecutar el mismo comando dos veces seguidas
                    if comando_accion == self.ultimo_comando_ejecutado:
                        self.debug_serial("[Comando Info] Comando repetido, ignorando.") # <-- Usar debug_serial
                        return
                    self.debug_serial(f"[Comando] Procesando: {comando_accion}") # <-- Usar debug_serial
                    self.ultimo_comando_ejecutado = comando_accion # Guardar como último ejecutado ANTES de empezar
                    
                    accion = comando_accion.get('accion')

                    if 'destino' in comando_accion:
                        # Bloquearía aquí hasta que termine, lo cual es bueno para secuencia
                        self.ejecutar_movimiento(comando_accion['destino'])
                        self.debug_serial(f"[Comando] Ejecución movimiento hacia {comando_accion['destino']} finalizada.") # <-- Usar debug_serial
                    elif accion == 'mostrar':# si no es un destino, entonces es un comando de visualización
                        fig = comando_accion.get('figura',"identificar")
                        self.controlar_matriz(fig)
                        self.debug_serial(f"[Comando] Ejecución visualización '{fig}' finalizada.") # <-- Usar debug_serial
                    elif accion == 'apagar':
                        self.controlar_matriz("apagar") # Usar el método unificado
                        self.debug_serial(f"[Comando] Ejecución apagar matriz finalizada.")
                    else:
                        self.debug_serial(f"[Comando Error] Acción no reconocida: {comando_accion}") # <-- Usar debug_serial
                        self.ultimo_comando_ejecutado = {} # Resetear si el comando no es válido
                elif topic_str == 'vehiculos/posicion_inicial':
                     self.debug_serial("[Comando Warn] Mensaje de posición inicial recibido en modo OPERANDO, ignorando.")

        except ValueError:
            self.debug_serial(f"[MQTT Error] JSON inválido recibido: '{msg_str}'") # <-- Usar debug_serial
        except Exception as e:
            self.debug_serial(f"[Error Inesperado] Callback MQTT: {e}") # <-- Usar debug_serial

    def loop(self):# revisado
        """Bucle principal para verificar mensajes MQTT y realizar tareas periódicas."""
        try:
            if self.mqtt.client:
                # check_msg() procesará mensajes pendientes y llamará a mqtt_recibido si hay algo
                self.mqtt.client.check_msg()
        except Exception as e:
             self.debug_serial(f"[Error MQTT Loop] Verificando mensajes: {e}") # <-- Usar debug_serial

        # --- Debug Periódico (Intervalo largo para no saturar) ---
        if time.ticks_ms() - self.debug_timer > 10000000000: # Reducido a 10 segundos para ver algo más a menudo
            try:
                d_del = self._leer_sensor_promedio(self.sensor_delantero, 3)
                d_tra = self._leer_sensor_promedio(self.sensor_trasero, 3)
                # Este mensaje periódico SÍ va por serial ahora
                self.debug_serial(f"[Sensores Loop] Del:{d_del}mm Tra:{d_tra}mm | Yaw:{self.giro_acel.yaw:.1f}°") # <-- Usar debug_serial (más corto)
            except Exception as e:
                 self.debug_serial(f"[Error lectura sensores loop: {e}") # <-- Usar debug_serial
            self.debug_timer = time.ticks_ms() # Resetear timer


    def _leer_sensor_promedio(self, sensor_obj, num_lecturas):# revisado
        """Lee el sensor varias veces y devuelve el promedio de lecturas válidas."""
        lecturas_validas = []; suma = 0
        for i in range(num_lecturas):
            lectura = sensor_obj.medir() # Asume que medir() devuelve -1 en error
            lecturas_validas.append(lectura)
            suma += lectura
            time.sleep_ms(1) # Pequeña pausa entre lecturas
        if not lecturas_validas:
            self.debug_serial("[Sensor Error] Ninguna lectura válida en promedio.") # Opcional
            return -1 # Indicar error
        else:
             promedio = suma // len(lecturas_validas) # División entera
             # self.debug_serial(f"[Sensor Info] Promedio de {len(lecturas_validas)} lecturas: {promedio} mm") # Opcional
             return promedio
