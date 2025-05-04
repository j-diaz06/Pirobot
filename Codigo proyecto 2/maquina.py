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
        #self.motores.velocidad = VELOCIDAD_MOVIMIENTO # Usar constante definida arriba
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
        self.alineacion_actual = 'Y' # <--- NUEVA VARIABLE: Inicia alineado en Y
        self.ultimo_comando_ejecutado = {}
        self.estado_sistema = ESTADO_INICIALIZANDO
        #self.comando_actual_pendiente = None
        mqtt_client_id = f"{CLIENT_ID_BASE}_{MI_ID}"
        self.debug_serial(f"[Init] Configurando MQTT con ClientID: {mqtt_client_id}")
        # Pasar self.debug_uart si mqtt_pirobot lo necesita para depurar internamente
        self.mqtt = mqtt_pirobot(SSID, PASSWORD, MQTT_BROKER, mqtt_client_id)
        self.girando = False
        self.motores_activos = False
        #self.espera_2nucleo = 0
        self.debug_timer = 0
        self.permitir_actualizacion_pos_operando = False # Nuevo flag

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
        #self.debug_serial(f"   Iniciando giro incremental: {grados_giro:.1f}° (pasos de {GRADOS_POR_MINIGIRO}°)")

        yaw_inicial = self.giro_acel.yaw
        yaw_objetivo_total_absoluto = self.normalizar_angulo(yaw_inicial + grados_giro)
        #self.debug_serial(f"      Yaw Inicial: {yaw_inicial:.1f}°, Objetivo Total: {yaw_objetivo_total_absoluto:.1f}°")

        grados_restantes = grados_giro
        paso_actual = 0

        while abs(grados_restantes) > PID_GIRO_TOLERANCIA_DEG:
            paso_actual += 1
            yaw_actual_inicio_paso = self.giro_acel.yaw # Leer yaw al inicio de ESTE paso

            # Determinar cuánto girar en este paso
            if abs(grados_restantes) >= GRADOS_POR_MINIGIRO:
                giro_paso_actual = GRADOS_POR_MINIGIRO * (1 if grados_restantes > 0 else -1)
            else:
                giro_paso_actual = grados_restantes # Girar lo que queda

            yaw_objetivo_paso_absoluto = self.normalizar_angulo(yaw_actual_inicio_paso + giro_paso_actual)

            #self.debug_serial(f"--- Paso {paso_actual}: Girando {giro_paso_actual:.1f}° ---")
            #self.debug_serial(f"      Desde Yaw: {yaw_actual_inicio_paso:.1f}° -> Objetivo Paso: {yaw_objetivo_paso_absoluto:.1f}°")

            # --- EJECUTAR PID PARA ESTE PASO ---
            if not self._ejecutar_giro_pid_paso(yaw_objetivo_paso_absoluto):
                 self.debug_serial(f"[Error] Fallo en el PID del paso {paso_actual}. Abortando giro total.")
                 self.detener() # Asegurar que esté detenido
                 return # Salir de la función girar si un paso falla

            # --- FIN PID PASO ---

            yaw_final_paso = self.giro_acel.yaw # Leer yaw real después del paso
            #error_real_paso = self.normalizar_angulo(yaw_objetivo_paso_absoluto - yaw_final_paso)
            #self.debug_serial(f"      Paso {paso_actual} completado. Yaw final: {yaw_final_paso:.1f}°, Error paso: {error_real_paso:.1f}°")

            # Recalcular grados restantes basados en el objetivo TOTAL
            grados_restantes = self.normalizar_angulo(yaw_objetivo_total_absoluto - yaw_final_paso)
            #self.debug_serial(f"      Grados restantes para objetivo total ({yaw_objetivo_total_absoluto:.1f}°): {grados_restantes:.1f}°")


            # Pausar si quedan más pasos
            if abs(grados_restantes) > PID_GIRO_TOLERANCIA_DEG:
                #self.debug_serial(f"      Pausando {PAUSA_ENTRE_MINIGIROS_MS} ms antes del siguiente paso...")
                self.detener() # Asegurarse de detener motores durante la pausa
                time.sleep_ms(PAUSA_ENTRE_MINIGIROS_MS)
            #else:
                #self.debug_serial("      Objetivo total alcanzado dentro de la tolerancia.")


        # --- Fin del bucle de pasos ---
        self.detener() # Asegurar que los motores están detenidos al final
        self.girando = False # Aunque el PID interno ya lo haga, asegurar estado
        yaw_final_total = self.giro_acel.yaw
        error_final_total = self.normalizar_angulo(yaw_objetivo_total_absoluto - yaw_final_total)
        #self.debug_serial(f"--- Giro incremental completado ---")
        #self.debug_serial(f"   Yaw final total: {yaw_final_total:.1f}°, Error final total: {error_final_total:.1f}°")

    # --- NUEVO: Función auxiliar para ejecutar el PID de un solo paso de giro ---
    def _ejecutar_giro_pid_paso(self, yaw_objetivo_absoluto):
        """Ejecuta el bucle PID para alcanzar un yaw_objetivo_absoluto específico. Devuelve True si éxito, False si falla."""
        self.motores_activos = True
        self.girando = True # Marcar como girando durante el paso

        # --- Inicialización PID (para este paso) ---
        pid_integral = 0.0
        ultimo_error = 0.0
        ultima_vez = time.ticks_ms()
        debug_timer_paso = time.ticks_ms() # Timer de debug para este paso

        # Bucle de control PID para este paso
        # Añadir un timeout para evitar bucles infinitos si algo va mal
        inicio_timeout = time.ticks_ms()
        TIMEOUT_PASO_GIRO_MS = 15000 # Ejemplo: 15 segundos máximo por paso

        while time.ticks_diff(time.ticks_ms(), inicio_timeout) < TIMEOUT_PASO_GIRO_MS:
            ahora = time.ticks_ms()
            dt = time.ticks_diff(ahora, ultima_vez) / 1000.0
            if dt <= 0: dt = 0.01 # Evitar dt inválido
            ultima_vez = ahora

            yaw_actual = self.giro_acel.yaw
            error = self.normalizar_angulo(yaw_objetivo_absoluto - yaw_actual)
            velocidad_angular_actual = self.giro_acel.filtered_gyro_z

            # --- Condición de Parada del Paso ---
            if abs(error) <= PID_GIRO_TOLERANCIA_DEG: # Simplificado: solo mirar error para parada fina
                 # Considerar también velocidad angular baja si es necesario refinar más
                 # if abs(velocidad_angular_actual) < 0.5: # Ejemplo
                 #self.debug_serial(f"      ¡Paso detenido por error bajo! Error: {error:.1f}° <= {PID_GIRO_TOLERANCIA_DEG}°")
                 self.detener() # Detener motores al final del paso exitoso
                 self.girando = False
                 return True # Paso completado con éxito

            # --- NUEVO: Modo Pulsos para errores pequeños ---
            if abs(error) < UMBRAL_ERROR_GIRO_PULSO:
                #self.debug_serial(f"         Modo Pulso (Error: {error:.1f}° < {UMBRAL_ERROR_GIRO_PULSO}°)")
                if error > 0: # Necesita girar izquierda
                    self.motores.girar_izquierda(VELOCIDAD_GIRO_PULSO)
                    accion_pulso = "Izquierda"
                else: # Necesita girar derecha
                    self.motores.girar_derecha(VELOCIDAD_GIRO_PULSO)
                    accion_pulso = "Derecha"

                #self.debug_serial(f"            Pulso {accion_pulso} ({DURACION_PULSO_GIRO_MS}ms @ {VELOCIDAD_GIRO_PULSO})")
                self.motores_activos = True # Asegurar que esté activo para el pulso
                time.sleep_ms(DURACION_PULSO_GIRO_MS)
                self.detener()
                #self.debug_serial(f"            Pausa post-pulso ({PAUSA_ENTRE_PULSOS_GIRO_MS}ms)")
                time.sleep_ms(PAUSA_ENTRE_PULSOS_GIRO_MS)
                ultima_vez = time.ticks_ms() # Resetear timer dt para evitar saltos en PID si vuelve a él
                ultimo_error = error # Actualizar último error conocido antes de continuar
                continue # Saltar el cálculo PID y reevaluar error en el siguiente ciclo
            # --- FIN MODO PULSOS ---

            # --- Seleccionar Constantes PID (Solo si error >= UMBRAL_ERROR_GIRO_PULSO) ---
            if error > 0:
                kp_actual, ki_actual, kd_actual = KP_GIRO_IZQ, KI_GIRO_IZQ, KD_GIRO_IZQ
                direccion_necesaria = "Izquierda"
            elif error < 0:
                kp_actual, ki_actual, kd_actual = KP_GIRO_DER, KI_GIRO_DER, KD_GIRO_DER
                direccion_necesaria = "Derecha"
            else:
                kp_actual, ki_actual, kd_actual = KP_GIRO_IZQ, KI_GIRO_IZQ, KD_GIRO_IZQ
                direccion_necesaria = "Ninguna"

            # --- Calcular Términos PID ---
            pid_integral += error * dt
            pid_integral = max(PID_GIRO_MIN_INTEGRAL, min(PID_GIRO_MAX_INTEGRAL, pid_integral))
            derivativo = 0
            if dt > 0: derivativo = (error - ultimo_error) / dt
            pid_output = (kp_actual * error) + (ki_actual * pid_integral) + (kd_actual * derivativo)

            # --- Mapear Salida PID a Velocidad de Motores ---
            velocidad_giro_calculada = int(abs(pid_output))
            velocidad_giro_calculada = max(VELOCIDAD_GIRO_MIN, min(VELOCIDAD_GIRO_MAX, velocidad_giro_calculada))

            if pid_output > 0:
                self.motores.girar_izquierda(velocidad_giro_calculada)
                direccion_giro = "Izquierda"
            elif pid_output < 0:
                self.motores.girar_derecha(velocidad_giro_calculada)
                direccion_giro = "Derecha"
            else:
                # No debería detenerse aquí a menos que el error sea 0 y la velocidad angular también (manejado por la condición de parada)
                # Si llega aquí con error 0 pero aún moviéndose, podría mantener la última acción o detenerse suavemente.
                # Por ahora, mantenemos la lógica anterior: detener si output es 0.
                self.detener()
                direccion_giro = "Detenido (error 0)"


            ultimo_error = error

            # --- Debugging (Opcional, pero útil para tuning) ---
            #if time.ticks_diff(time.ticks_ms(), debug_timer_paso) > 200:
            #   self.debug_serial(f"         PID Paso ({direccion_necesaria}): Err={error:.1f}, P={kp_actual*error:.0f}, I={ki_actual*pid_integral:.0f}, D={kd_actual*derivativo:.0f} -> Out={pid_output:.0f} -> Vel={velocidad_giro_calculada} ({direccion_giro})")
            #   debug_timer_paso = time.ticks_ms()

            time.sleep_ms(1) # Pequeña pausa

        # --- Si salimos del bucle por Timeout ---
        self.debug_serial(f"[Error PID Paso] Timeout de {TIMEOUT_PASO_GIRO_MS} ms alcanzado. Yaw actual: {self.giro_acel.yaw:.1f}°, Objetivo: {yaw_objetivo_absoluto:.1f}°")
        self.detener()
        self.girando = False
        return False # Indicar fallo del paso

    def ejecutar_movimiento(self, destino_str):# revisado
        """Mueve el robot a la celda destino, priorizando el eje actual y calibrando al inicio."""
        self.debug_serial(f"[Accion] Iniciando secuencia de movimiento hacia: {destino_str}")
        if not self.posicion_actual:
            self.debug_serial("[Error] Posición actual desconocida.")
            return
        try:
            current_x, current_y = map(float, self.posicion_actual.split(','))
            target_x, target_y = map(float, destino_str.split(','))
        except Exception as e:
            self.debug_serial(f"[Error] Formato coordenadas inválido: {e}")
            return

        self.debug_serial(f"        Desde: ({current_x:.1f}, {current_y:.1f}) Hacia: ({target_x:.1f}, {target_y:.1f})")
        dx_celdas = target_x - current_x
        dy_celdas = target_y - current_y

        movimiento_y_necesario = abs(dy_celdas) > 0.01
        movimiento_x_necesario = abs(dx_celdas) > 0.01

        resultado_parcial_y = True
        resultado_parcial_x = True
        movimiento_exitoso_total = True
        primer_eje_movido = False

        # --- NUEVO: Verificar estabilidad y calibrar al INICIO de la secuencia ---
        self.debug_serial("--- Verificando estabilidad Yaw INICIAL (calibrará si es necesario) ---")
        self._verificar_estabilidad_yaw(UMBRAL_DERIVA_YAW_GPS, DURACION_CHECK_YAW_MS)
        self.debug_serial(f"--- Estabilidad inicial verificada. Yaw actual (relativo post-calibración): {self.giro_acel.yaw:.1f}° ---")
        # A partir de aquí, el yaw relativo es 0.0 si hubo calibración.

        if self.alineacion_actual == 'Y':
            self.debug_serial(f"Priorizando Eje Y (Alineación actual: {self.alineacion_actual})")
            # Movimiento en Y (si es necesario)
            if movimiento_y_necesario:
                self.debug_serial("--- Intentando Movimiento Eje Y (Priorizado) ---")
                distancia_mm_y = dy_celdas * CELDA_SIZE_MM
                #self.debug_serial("   Alineación requerida para Y: 0.0° (Relativo)")
                # Como ya calibramos al inicio, el yaw actual debería ser cercano a 0 si estábamos en Y.
                # Un giro aquí solo corregiría deriva ocurrida DESDE la calibración inicial.
                # yaw_actual_rel = self.giro_acel.yaw
                # angulo_giro_y = self.normalizar_angulo(0.0 - yaw_actual_rel)

                #if abs(angulo_giro_y) > GIRO_TOLERANCIA_DEG:
                #    self.debug_serial(f"      ¡Requiere micro-corrección para realinear con Y! Giro Relativo: {angulo_giro_y:.1f}°") # Mensaje ajustado
                    # NO necesitamos otra verificación de estabilidad aquí
                #    self.girar(angulo_giro_y)
                #    time.sleep_ms(PAUSA_ENTRE_MOVIMIENTOS_MS)
                #else:
                #    self.debug_serial(f"      Ya alineado con Y (Relativo: {yaw_actual_rel:.1f}°)")
                self.alineacion_actual = 'Y'
                #self.debug_serial(f"      Alineación Y confirmada.")

                # Leer yaw actual justo antes de moverse
                yaw_actual_para_movimiento = self.giro_acel.yaw
                self.debug_serial(f"   Moviendo en Y manteniendo Yaw actual: {yaw_actual_para_movimiento:.1f}°")
                resultado_parcial_y = self.mover_distancia_con_sensor(distancia_mm_y, yaw_actual_para_movimiento)
                if not resultado_parcial_y:
                    self.debug_serial("   Fallo en movimiento Y.")
                    movimiento_exitoso_total = False
                else:
                     self.debug_serial("   Movimiento Y completado.")
                     primer_eje_movido = True


            # Movimiento en X (si es necesario y Y fue exitoso)
            if movimiento_exitoso_total and movimiento_x_necesario:
                if primer_eje_movido:
                    self.debug_serial(f"   Pausa post-movimiento Y ({PAUSA_ENTRE_MOVIMIENTOS_MS} ms)...")
                    time.sleep_ms(PAUSA_ENTRE_MOVIMIENTOS_MS)
                self.debug_serial("--- Intentando Movimiento Eje X (Secundario) ---")
                distancia_mm_x = dx_celdas * CELDA_SIZE_MM

                yaw_objetivo_recto_x_rel = -90.0
                self.debug_serial(f"   Alineación requerida para X: {yaw_objetivo_recto_x_rel:.1f}° (Relativo)")
                # --- CAMBIO: Usar directamente el ángulo objetivo para el giro de 90° ---
                # yaw_actual_rel = self.giro_acel.yaw # Ya no es necesario leerlo aquí
                angulo_giro_x = yaw_objetivo_recto_x_rel
                self.debug_serial(f"   Giro Relativo necesario para alinear con X: {angulo_giro_x:.1f}°")

                # Comprobamos si *ya* estamos orientados a X (la tolerancia del giro lo gestiona)
                # if abs(angulo_giro_x) > GIRO_TOLERANCIA_DEG: # La función girar ya tiene esta lógica
                self.debug_serial(f"      Realizando giro para alinearse con X...")
                self.girar(angulo_giro_x) # Gira al ángulo RELATIVO +/-90
                time.sleep_ms(PAUSA_ENTRE_MOVIMIENTOS_MS)
                # else:
                    # Esto no debería ocurrir si venimos de Y=0, a menos que la tolerancia sea muy grande
                #    self.debug_serial(f"      Giro X no necesario (¿Ya alineado con X relativo?).")
                self.alineacion_actual = 'X'
                self.debug_serial(f"      Alineación X confirmada.")

                # Leer yaw actual justo antes de moverse
                yaw_actual_para_movimiento = self.giro_acel.yaw
                self.debug_serial(f"   Moviendo en X manteniendo Yaw actual: {yaw_actual_para_movimiento:.1f}°")
                resultado_parcial_x = self.mover_distancia_con_sensor(distancia_mm_x, yaw_actual_para_movimiento)
                if not resultado_parcial_x:
                    self.debug_serial("   Fallo en movimiento X.")
                    movimiento_exitoso_total = False
                else:
                     self.debug_serial("   Movimiento X completado.")


        elif self.alineacion_actual == 'X':
            self.debug_serial(f"Priorizando Eje X (Alineación actual: {self.alineacion_actual})")
             # Movimiento en X (si es necesario)
            if movimiento_x_necesario:
                self.debug_serial("--- Intentando Movimiento Eje X (Priorizado) ---")
                distancia_mm_x = dx_celdas * CELDA_SIZE_MM
                # El objetivo RELATIVO después de la calibración inicial (que nos dejó en X=0 relativo) es 0.0
                #yaw_objetivo_recto_x_rel = 0.0
                #self.debug_serial(f"   Alineación requerida para X: {yaw_objetivo_recto_x_rel:.1f}° (Relativo)")
                # --- CAMBIO: Usar directamente el ángulo objetivo para el giro de 90° ---
                # yaw_actual_rel = self.giro_acel.yaw # Ya no es necesario leerlo aquí
                #angulo_giro_x = yaw_objetivo_recto_x_rel
                #self.debug_serial(f"   Giro Relativo necesario para alinear con X: {angulo_giro_x:.1f}°")

                # Comprobamos si *ya* estamos orientados a X (la tolerancia del giro lo gestiona)
                # if abs(angulo_giro_x) > GIRO_TOLERANCIA_DEG: # La función girar ya tiene esta lógica
                #self.debug_serial(f"      Realizando giro para alinearse con X...")
                #self.girar(angulo_giro_x) # Gira al ángulo RELATIVO +/-90
                #time.sleep_ms(PAUSA_ENTRE_MOVIMIENTOS_MS)
                # else:
                    # Esto no debería ocurrir si venimos de Y=0, a menos que la tolerancia sea muy grande
                #    self.debug_serial(f"      Giro X no necesario (¿Ya alineado con X relativo?).")
                self.alineacion_actual = 'X'
                #self.debug_serial(f"      Alineación X confirmada.")

                # Leer yaw actual justo antes de moverse
                yaw_actual_para_movimiento = self.giro_acel.yaw
                self.debug_serial(f"   Moviendo en X manteniendo Yaw actual: {yaw_actual_para_movimiento:.1f}°")
                resultado_parcial_x = self.mover_distancia_con_sensor(distancia_mm_x, yaw_actual_para_movimiento)
                if not resultado_parcial_x:
                    self.debug_serial("   Fallo en movimiento X.")
                    movimiento_exitoso_total = False
                else:
                    self.debug_serial("   Movimiento X completado.")
                    primer_eje_movido = True

            # Movimiento en Y (si es necesario y X fue exitoso)
            if movimiento_exitoso_total and movimiento_y_necesario:
                if primer_eje_movido:
                    self.debug_serial(f"   Pausa post-movimiento X ({PAUSA_ENTRE_MOVIMIENTOS_MS} ms)...")
                    time.sleep_ms(PAUSA_ENTRE_MOVIMIENTOS_MS)
                self.debug_serial("--- Intentando Movimiento Eje Y (Secundario) ---")
                distancia_mm_y = dy_celdas * CELDA_SIZE_MM

                yaw_objetivo_recto_y_rel = 90.0
                self.debug_serial(f"   Alineación requerida para Y: {yaw_objetivo_recto_y_rel:.1f}° (Relativo)")
                # --- CAMBIO: Usar directamente el ángulo objetivo para el giro de 90° ---
                # yaw_actual_rel = self.giro_acel.yaw # Ya no es necesario leerlo aquí
                angulo_giro_y = yaw_objetivo_recto_y_rel
                self.debug_serial(f"   Giro Relativo necesario para alinear con Y: {angulo_giro_y:.1f}°")

                # if abs(angulo_giro_y) > GIRO_TOLERANCIA_DEG:
                self.debug_serial(f"      Realizando giro para alinearse con Y...")
                # NO necesitamos otra verificación de estabilidad aquí
                self.girar(angulo_giro_y)
                time.sleep_ms(PAUSA_ENTRE_MOVIMIENTOS_MS)
                # else:
                    # Esto no debería ocurrir si venimos de X=0
                #    self.debug_serial(f"      Giro Y no necesario (¿Ya alineado con Y relativo?).")
                self.alineacion_actual = 'Y'
                self.debug_serial(f"      Alineación Y confirmada.")

                # Leer yaw actual justo antes de moverse
                yaw_actual_para_movimiento = self.giro_acel.yaw
                self.debug_serial(f"   Moviendo en Y manteniendo Yaw actual: {yaw_actual_para_movimiento:.1f}°")
                resultado_parcial_y = self.mover_distancia_con_sensor(distancia_mm_y, yaw_actual_para_movimiento)
                if not resultado_parcial_y:
                    self.debug_serial("   Fallo en movimiento Y.")
                    movimiento_exitoso_total = False
                else:
                    self.debug_serial("   Movimiento Y completado.")
        else:
            self.debug_serial(f"[Error Fatal] Alineación desconocida: {self.alineacion_actual}. Abortando movimiento.")
            movimiento_exitoso_total = False

        # --- Finalización ---
        if movimiento_exitoso_total:
            if movimiento_x_necesario or movimiento_y_necesario:
                self.posicion_actual = destino_str
                self.debug_serial(f"[Accion] Secuencia hacia {destino_str} completada exitosamente.")
                self.debug_serial(f"[Estado] Nueva posición: {self.posicion_actual}, Alineación final: {self.alineacion_actual}")
            else:
                self.debug_serial(f"[Accion] No se requirió movimiento para ir a {destino_str} (ya estaba allí).")
                self.debug_serial(f"[Estado] Posición: {self.posicion_actual}, Alineación: {self.alineacion_actual}")
        else:
            self.debug_serial(f"[Accion] Secuencia hacia {destino_str} NO completada exitosamente.")
            self.debug_serial(f"[Estado] Posición NO actualizada: {self.posicion_actual}, Alineación actual: {self.alineacion_actual}")

        #self.controlar_led("apagado")

    def mover_distancia_con_sensor(self, distancia_mm, yaw_setpoint):

        # el concepto de "adelante" o "atrás" es relativo a la intención del movimiento actual,
        # y el sensor correspondiente (delantero o trasero) se usa como referencia principal para ese movimiento
        
        adelante = distancia_mm > 0
        sensor_a_usar = self.sensor_delantero if adelante else self.sensor_trasero
        distancia_absoluta_mm = abs(distancia_mm)

        self.debug_serial(f"   Moviendo {'adelante' if adelante else 'atras'} por {distancia_absoluta_mm} mm manteniendo Yaw={yaw_setpoint:.1f}°...") # <-- Usar debug_serial
        distancia_inicial_promedio = self._leer_sensor_promedio(sensor_a_usar, 15) # promedio de 15 lecturas iniciales ya que no afecta al pid
        self.debug_serial(f"      Lectura INICIAL PROMEDIO sensor {'frontal' if adelante else 'trasero'}: {distancia_inicial_promedio} mm") # <-- Usar debug_serial
        
        if distancia_inicial_promedio < 0:
            self.debug_serial(f"[Error Sensor] Lecturas iniciales inválidas. Abortando.")
            self.detener()
            return False
        
        # --- NUEVO: Seleccionar offset basado en la distancia inicial promedio ---
        # REFACORTADO: Usar función auxiliar
        offset_config = self._obtener_offset_actual(distancia_inicial_promedio, adelante)
        self.debug_serial(f"      Usando offset {'delantero' if adelante else 'trasero'} para distancia inicial {distancia_inicial_promedio}mm: {offset_config}mm")
            
        # Aplicar offset a la lectura inicial
        distancia_inicial_ajustada = distancia_inicial_promedio - offset_config
        self.debug_serial(f"      Lectura INICIAL AJUSTADA: {distancia_inicial_ajustada} mm (Original: {distancia_inicial_promedio} mm, Offset: {offset_config} mm)")
        
        # Calcular objetivo basado en la distancia inicial ajustada
        distancia_objetivo_base = distancia_inicial_ajustada - distancia_absoluta_mm
        
        # El objetivo no se modifica con el offset, ya que el offset se aplicará a la lectura actual
        self.debug_serial(f"      Distancia objetivo del sensor: {distancia_objetivo_base:.0f} mm") # <-- Usar debug_serial
        # Inicializar PID
        pid_integral = 0.0; pid_last_error = 0.0; pid_last_time = time.ticks_ms()

        # Establecer dirección inicial
        if adelante: self.motores.avanzar(VELOCIDAD_MOVIMIENTO)#self.motores.in1.value(0); self.motores.in2.value(1); self.motores.in3.value(1); self.motores.in4.value(0)
        else: self.motores.retroceder(VELOCIDAD_MOVIMIENTO)#self.motores.in1.value(1); self.motores.in2.value(0); self.motores.in3.value(0); self.motores.in4.value(1)
        self.motores_activos = True

        movimiento_completado = False
        # Añadir un temporizador para debug similar al de girar
        debug_timer_move = time.ticks_ms()

        # Bucle PID y distancia
        while True:
            ahora = time.ticks_ms()

            # --- 1. Leer Sensor (Rápido) ---
            # OPTIMIZACIÓN: Leer solo 1 vez en el bucle para mínima latencia
            distancia_actual = self._leer_sensor_promedio(sensor_a_usar, NUM_LECTURAS_SENSOR_MOVE)

            if distancia_actual < 0:
                # Si hay error de lectura, podríamos saltar este ciclo PID o detenernos
                self.debug_serial("[Warn] Error lectura sensor durante movimiento, saltando ciclo PID.")
                pid_last_time = ahora # Evitar dt grande en el siguiente ciclo
                continue # Saltar el resto del bucle

            # --- 2. Comprobar Condición de Parada ---
            
            # --- CORREGIDO: Seleccionar offset basado en la lectura ACTUAL del sensor ---
            # REFACTORIZADO: Usar función auxiliar
            offset_actual = self._obtener_offset_actual(distancia_actual, adelante)
            
            # Imprimir debug sólo si cambia el offset (comparado con el inicial o el anterior)
            if offset_actual != offset_config: # Compara con el offset inicial o el último usado
                offset_config = offset_actual  # Actualizar el offset actual para la próxima comparación
            
            # Aplicar offset a la lectura actual
            distancia_actual_ajustada = distancia_actual - offset_actual
            
            # Comprobar si hemos llegado al objetivo comparando la distancia ajustada
            if distancia_actual_ajustada <= distancia_objetivo_base:
                self.detener()
                self.debug_serial(f"\n   ¡Detenido por sensor! Dist final: {distancia_actual} mm, Dist ajustada: {distancia_actual_ajustada:.0f} mm (Obj: {distancia_objetivo_base:.0f})")
                movimiento_completado = True
                break # Salir del bucle while

            # --- 3. Calcular PID para Corrección de Yaw ---
            dt_pid = time.ticks_diff(ahora, pid_last_time) / 1000.0
            pid_last_time = ahora # Actualizar pid_last_time siempre

            yaw_actual = self.giro_acel.yaw
            error = yaw_setpoint - yaw_actual
            # Normalizar error a +/- 180 grados
            while error > 180.0: error -= 360.0
            while error <= -180.0: error += 360.0

            # --- Seleccionar constantes PID según dirección ---
            if adelante:
                kp_actual, ki_actual, kd_actual = KP_MOVE, KI_MOVE, KD_MOVE
            else:
                # Asumir que KP_MOVE_BACK, KI_MOVE_BACK, KD_MOVE_BACK están definidas en config.py
                kp_actual, ki_actual, kd_actual = KP_MOVE_BACK, KI_MOVE_BACK, KD_MOVE_BACK

            # Término Derivativo (solo si dt es válido)
            derivative = 0
            pid_correction = (kp_actual * error) # Inicializar solo con P, usando constante seleccionada

            # Solo calcular I y D si dt_pid es válido y positivo
            if dt_pid > 0:
                # Término Integral con anti-windup
                pid_integral += error * dt_pid
                pid_integral = max(PID_MOVE_MIN_INTEGRAL, min(PID_MOVE_MAX_INTEGRAL, pid_integral))

                # Término Derivativo
                derivative = (error - pid_last_error) / dt_pid

                # Añadir términos I y D a la corrección
                pid_correction += (ki_actual * pid_integral) + (kd_actual * derivative)

            pid_last_error = error

            # --- 4. Aplicar Corrección a los Motores ---
            
            # OPTIMIZACIÓN: Reducir velocidad cerca del objetivo
            distancia_al_objetivo = abs(distancia_actual_ajustada - distancia_objetivo_base)
            if distancia_al_objetivo < DISTANCIA_REDUCCION_MM:
                # Reducción lineal (ejemplo simple)
                factor_reduccion = max(0, distancia_al_objetivo / DISTANCIA_REDUCCION_MM)
                base_pwm_duty_actual = int(VELOCIDAD_REDUCCION + (VELOCIDAD_MOVIMIENTO - VELOCIDAD_REDUCCION) * factor_reduccion)
                base_pwm_duty_actual = max(VELOCIDAD_REDUCCION, base_pwm_duty_actual) # Asegurar mínimo
                # Imprimir solo si cambia significativamente para no saturar
                #if abs(base_pwm_duty_actual - VELOCIDAD_MOVIMIENTO) > 100: 
                #    self.debug_serial(f"     Reduciendo velocidad base a: {base_pwm_duty_actual} (Dist obj: {distancia_al_objetivo:.0f}mm)")
            else:
                base_pwm_duty_actual = VELOCIDAD_MOVIMIENTO
            
            # Calcular PWMs finales usando la velocidad base actual (potencialmente reducida)
            if adelante:
                pwm_A = base_pwm_duty_actual - pid_correction # Izquierda
                pwm_B = base_pwm_duty_actual + pid_correction # Derecha
            else: # Hacia atrás
                pwm_A = base_pwm_duty_actual + pid_correction # Izquierda
                pwm_B = base_pwm_duty_actual - pid_correction # Derecha

            # Limitar PWM (manteniendo mínimo y máximo)
            pwm_A = max(MIN_PWM_MOVE, min(65535, int(pwm_A)))
            pwm_B = max(MIN_PWM_MOVE, min(65535, int(pwm_B)))

            self.motores.set_pwm_duties(pwm_A, pwm_B)

            # --- 5. Debugging (simplificado a una línea por ciclo) ---
            # Imprimir solo cada cierto tiempo
#             if time.ticks_diff(time.ticks_ms(), debug_timer_move) > 100: # Imprimir cada 100ms (ajustable)
#                 # Calcular distancia recorrida y restante para el mensaje de debug
#                 distancia_recorrida_mm = distancia_inicial_ajustada - distancia_actual_ajustada # Usar ajustadas
#                 distancia_restante_mm = distancia_absoluta_mm - distancia_recorrida_mm
#                 debug_msg = (f"     Dist:{distancia_actual}|Ajust:{distancia_actual_ajustada:.0f}(Obj:{distancia_objetivo_base:.0f})"
#                              f"|Rest:{distancia_restante_mm:.0f}mm|Off:{offset_actual}mm|VBase:{base_pwm_duty_actual}"
#                              f"|YawErr:{error:.1f},PID:{pid_correction:.0f}"
#                              f"|PWM A:{pwm_A},B:{pwm_B}")
#                 self.debug_serial(debug_msg)
#                 debug_timer_move = time.ticks_ms() # Resetear timer de debug

        # --- Corrección de Ángulo Final (si el movimiento principal se completó) ---
        if movimiento_completado:
            # EL DETENER SE MOVIÓ DENTRO DEL BUCLE PARA PARADA INSTANTÁNEA
            self.debug_serial(f"   Movimiento principal completado. Verificando ángulo final...")
            yaw_final_mov = self.giro_acel.yaw
            error_final = self.normalizar_angulo(yaw_setpoint - yaw_final_mov) # Normalizar aquí también

            self.debug_serial(f"      Yaw Final: {yaw_final_mov:.1f}°, Setpoint: {yaw_setpoint:.1f}°, Error Final: {error_final:.1f}°")

            # Usar la misma tolerancia que el giro normal
            if abs(error_final) > PID_GIRO_TOLERANCIA_DEG:
                self.debug_serial(f"      Corrigiendo error angular final ({error_final:.1f}°)...")
                self.girar(error_final) # Llamar a la función de giro PID (esta detiene al final)
                self.debug_serial(f"      Corrección angular final completada.")
            else:
                self.debug_serial(f"      Ángulo final dentro de la tolerancia ({PID_GIRO_TOLERANCIA_DEG}°). No se requiere corrección.")
                # Asegurarse de que los motores estén detenidos si no hubo giro de corrección
                self.detener()
        else:
             # Si el movimiento principal falló (ej. por error de sensor inicial), detener y salir
             self.detener()
             return False

        # --- NUEVO: Fase de Corrección de Distancia Final ---
        self.debug_serial(f"--- Iniciando verificación/corrección de distancia final ---")
        time.sleep_ms(1000) # Pequeña pausa para asegurar que el robot está quieto

        intento_correccion = 0
        while intento_correccion < MAX_INTENTOS_CORRECCION:
            intento_correccion += 1
            self.debug_serial(f"--- Intento de corrección #{intento_correccion}/{MAX_INTENTOS_CORRECCION} ---")

            # --- VERIFICACIÓN/CORRECCIÓN DE YAW ANTES DE MEDIR/CORREGIR DISTANCIA ---
            #self.debug_serial(f"   Check Yaw (Antes Intento #{intento_correccion}): Verificando ángulo...")
            yaw_actual_correccion = self.giro_acel.yaw
            error_yaw_correccion = self.normalizar_angulo(yaw_setpoint - yaw_actual_correccion)
            #self.debug_serial(f"      Yaw Actual (Pre-Corrección): {yaw_actual_correccion:.1f}°, Setpoint: {yaw_setpoint:.1f}°, Error: {error_yaw_correccion:.1f}°")
            if abs(error_yaw_correccion) > PID_GIRO_TOLERANCIA_DEG:
                #self.debug_serial(f"      ¡Requiere corrección de Yaw ({error_yaw_correccion:.1f}°)!...")
                self.girar(error_yaw_correccion)
                #self.debug_serial(f"      Corrección de Yaw (Antes Intento #{intento_correccion}) completada. Pausando...")
                time.sleep_ms(200) # Pausa después de la corrección de ángulo
            #else:
                #self.debug_serial(f"      Ángulo dentro de tolerancia ({PID_GIRO_TOLERANCIA_DEG}°). No se requiere corrección de Yaw.")
            # --- FIN VERIFICACIÓN/CORRECCIÓN DE YAW ---


            distancia_actual_promedio = self._leer_sensor_promedio(sensor_a_usar, 15)
            if distancia_actual_promedio < 0:
                self.debug_serial(f"[Error Corrección Intento #{intento_correccion}] Lectura inválida. Abortando corrección.")
                # Considerar si el movimiento principal debe marcarse como fallido aquí
                movimiento_completado = False # Opcional: Marcar como fallo si la lectura es inválida
                break # Salir del bucle de intentos

            # Seleccionar offset para la lectura actual
            offset_actual = self._obtener_offset_actual(distancia_actual_promedio, adelante)
            distancia_actual_ajustada = distancia_actual_promedio - offset_actual
            error_distancia_mm = distancia_actual_ajustada - distancia_objetivo_base

            self.debug_serial(f"   Check Intento #{intento_correccion}: Lectura={distancia_actual_promedio}mm, Offset={offset_actual}mm -> Ajustada={distancia_actual_ajustada:.0f}mm")
            #self.debug_serial(f"   Objetivo Base: {distancia_objetivo_base:.0f}mm -> Error Actual: {error_distancia_mm:.0f}mm")

            # Comprobar si estamos dentro de la tolerancia
            if abs(error_distancia_mm) <= TOLERANCIA_DISTANCIA_FINAL_MM:
                self.detener()
                self.debug_serial(f"   Distancia final dentro de la tolerancia ({TOLERANCIA_DISTANCIA_FINAL_MM}mm) en intento #{intento_correccion}.")
                break # Salir del bucle de intentos, corrección exitosa o no necesaria

            # Si el error es mayor que la tolerancia, proceder con la corrección
            #self.debug_serial(f"   ¡Requiere corrección! Error ({error_distancia_mm:.0f}mm) > Tolerancia ({TOLERANCIA_DISTANCIA_FINAL_MM}mm).")

            # --- CORRECCIÓN POR IMPULSO TEMPORIZADO ---
            # Determinar dirección
            # Si error > 0 (dist_ajustada > obj_base -> se quedó corto), mover ADELANTE.
            # Si error < 0 (dist_ajustada < obj_base -> se pasó), mover ATRÁS.
            adelante_correccion = error_distancia_mm > 0 # True si se quedó corto y necesita avanzar

            # --- Seleccionar duración del pulso según el error ---
            error_abs = abs(error_distancia_mm)
            # --- NUEVO: Seleccionar constantes según dirección ORIGINAL del movimiento ---
            if adelante: # Si el movimiento principal fue hacia adelante
                pulse_large = CORRECTION_PULSE_FWD_MS_LARGE_ERROR
                pulse_small = CORRECTION_PULSE_FWD_MS_SMALL_ERROR
                direccion_str = "ADELANTE"
            else: # Si el movimiento principal fue hacia atrás
                pulse_large = CORRECTION_PULSE_BWD_MS_LARGE_ERROR
                pulse_small = CORRECTION_PULSE_BWD_MS_SMALL_ERROR
                direccion_str = "ATRÁS"
            # --- FIN NUEVO ---

            if error_abs > 3:
                pulso_ms = pulse_large # Usar variable seleccionada
                #self.debug_serial(f"      Error grande ({error_abs:.0f}mm > 3mm) mov. {direccion_str}, usando pulso de {pulso_ms}ms.") # Actualizar debug
            else:
                pulso_ms = pulse_small # Usar variable seleccionada
                #self.debug_serial(f"      Error pequeño ({error_abs:.0f}mm <= 3mm) mov. {direccion_str}, usando pulso de {pulso_ms}ms.") # Actualizar debug
            # ---------------------------------------------------

            # --- APLICAR PULSO DE CORRECCIÓN (con inversión para BWD) ---
            #self.debug_serial(f"      Corrección necesaria: Mover {'adelante' if adelante_correccion else 'atras'}. Pulso: {pulso_ms}ms.")

            # Establecer dirección y velocidad de corrección
            if adelante: # Movimiento original FWD
                if adelante_correccion: # Error > 0 (pasado de largo), corregir AVANZANDO
                    self.motores.avanzar(VELOCIDAD_CORRECCION_FINAL)
                    accion_motor = "AVANZAR (Fwd)"
                else: # Error < 0 (quedado corto), corregir RETROCEDIENDO
                    self.motores.retroceder(VELOCIDAD_CORRECCION_FINAL)
                    accion_motor = "RETROCEDER (Fwd)"
            else: # Movimiento original BWD
                if adelante_correccion: # Error > 0 (demasiado cerca de pared trasera), corregir AVANZANDO
                    # Original: self.motores.avanzar(). Los logs sugieren invertir.
                    self.motores.retroceder(VELOCIDAD_CORRECCION_FINAL) # <-- INVERTIDO
                    accion_motor = "RETROCEDER (Bwd - Corrección FWD Invertida)"
                else: # Error < 0 (demasiado lejos de pared trasera), corregir RETROCEDIENDO
                    # Original: self.motores.retroceder(). Los logs sugieren invertir.
                    self.motores.avanzar(VELOCIDAD_CORRECCION_FINAL) # <-- INVERTIDO
                    accion_motor = "AVANZAR (Bwd - Corrección BWD Invertida)"

            #self.debug_serial(f"      Aplicando acción motor: {accion_motor} por {pulso_ms}ms")
            self.motores_activos = True

            # Mover durante el pulso calculado
            time.sleep_ms(pulso_ms)

            # Detener inmediatamente después del pulso
            self.detener()

            # Pausa breve antes del siguiente intento de verificación/corrección
            time.sleep_ms(100) # Permitir que el robot se asiente antes de medir de nuevo

        # --- FIN Bucle de Intentos de Corrección ---

        # --- VERIFICACIÓN/CORRECCIÓN DE YAW FINAL (DESPUÉS DEL BUCLE DE CORRECCIÓN DE DISTANCIA) ---
        self.debug_serial(f"--- Verificación/Corrección de Yaw Final (Post-Corrección Distancia) ---")
        yaw_final_post_correccion = self.giro_acel.yaw
        error_yaw_final_post = self.normalizar_angulo(yaw_setpoint - yaw_final_post_correccion)
        self.debug_serial(f"   Yaw Final (Post-Corrección): {yaw_final_post_correccion:.1f}°, Setpoint: {yaw_setpoint:.1f}°, Error: {error_yaw_final_post:.1f}°")
        if abs(error_yaw_final_post) > PID_GIRO_TOLERANCIA_DEG:
            self.debug_serial(f"   ¡Requiere corrección de Yaw final ({error_yaw_final_post:.1f}°)!...")
            self.girar(error_yaw_final_post)
            self.debug_serial(f"   Corrección de Yaw final completada.")
        else:
            self.debug_serial(f"   Ángulo final dentro de tolerancia ({PID_GIRO_TOLERANCIA_DEG}°).")
        # --- FIN VERIFICACIÓN/CORRECCIÓN DE YAW FINAL ---


        # Verificar el resultado final después de los intentos
        if intento_correccion >= MAX_INTENTOS_CORRECCION and abs(error_distancia_mm) > TOLERANCIA_DISTANCIA_FINAL_MM:
             self.debug_serial(f"[Error Corrección Final] No se alcanzó la tolerancia después de {MAX_INTENTOS_CORRECCION} intentos. Error final: {error_distancia_mm:.0f}mm")
             movimiento_completado = False # Marcar el movimiento principal como fallido si la corrección final no tuvo éxito

        # Este detener final es redundante si la corrección se hizo y detuvo,
        # pero es seguro dejarlo por si no hubo corrección o el bucle terminó.
        self.detener()
        return movimiento_completado


    # --- Método Verificar estabilidad Yaw (Usando debug_serial) ---
    def _verificar_estabilidad_yaw(self, umbral_gps, duracion_ms, pausa_reintento_ms=500):# revisado

        self.debug_serial("--- Iniciando proceso de estabilidad Yaw ---")
        # Calibrar una vez antes de empezar el bucle de verificación
        self.giro_acel._calibrar()
        self.debug_serial("  Calibración inicial completada.")

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

    # --- NUEVO: Función para rotar una matriz (figura) ---
    def _rotar_matriz(self, matriz, grados):
        """Rota una matriz cuadrada 90 o 180 grados en sentido horario."""
        if not matriz or not matriz[0]:
            return matriz # Devuelve vacía o inválida si lo es
        
        N = len(matriz)
        if N != len(matriz[0]):
            self.debug_serial("[Error Rotación] La matriz no es cuadrada.")
            return matriz # No se puede rotar si no es cuadrada

        # Crear una nueva matriz vacía del mismo tamaño
        matriz_rotada = [[0 for _ in range(N)] for _ in range(N)]

        if grados == 90:
            for i in range(N):
                for j in range(N):
                    matriz_rotada[j][N - 1 - i] = matriz[i][j]
        elif grados == 180:
            for i in range(N):
                for j in range(N):
                    matriz_rotada[N - 1 - i][N - 1 - j] = matriz[i][j]
        else: # Si grados no es 90 ni 180, devolver original
            self.debug_serial(f"[Warn Rotación] Grados no soportados ({grados}), devolviendo original.")
            return matriz

        return matriz_rotada
    # --- FIN NUEVO ---

    def controlar_matriz(self, figura=None):
        self.debug_serial(f"[Matriz] Controlando figura: {figura}")

        # Determinar grados de rotación según la alineación actual
        grados_rotacion = 0
        if self.alineacion_actual == 'Y':
            grados_rotacion = 180
        elif self.alineacion_actual == 'X':
            grados_rotacion = 90

        if figura == "cara_feliz":
            self.matriz.ajustar_brillo(brillo_matriz)
            # Obtener la matriz original de la configuración
            matriz_original = cara_feliz # Asume que cara_feliz está importada o definida globalmente
            # Rotar la matriz si es necesario (grados != 0)
            matriz_a_mostrar = matriz_original
            if grados_rotacion != 0:
                 self.debug_serial(f"      Rotando {grados_rotacion}° por alineación '{self.alineacion_actual}'")
                 matriz_a_mostrar = self._rotar_matriz(matriz_original, grados_rotacion)
            else:
                 # Si no hay rotación (alineación inicial desconocida o no Y/X)
                 self.debug_serial(f"      No se requiere rotación (Alineación: {self.alineacion_actual})")

            self.matriz.mostrar_matriz_grafica(matriz_a_mostrar, color)
            self.matriz.mostrar()
        elif figura == "identificar":
            # El llenado no necesita rotación
            self.matriz.ajustar_brillo(brillo_matriz)
            self.matriz.llenar_matriz(color)
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
            time.sleep_ms(3) # Ceder tiempo a otros procesos/hilos (Reducido de 5ms)

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

    # --- NUEVO: Método para habilitar la escucha de posición inicial en modo operando ---
    def habilitar_escucha_posicion(self):
        """Permite que el robot acepte una nueva posición desde MQTT mientras opera."""
        self.permitir_actualizacion_pos_operando = True
        self.debug_serial("[CMD ESCUCHARPOS] Habilitada la recepción de nueva posición inicial vía MQTT.")

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

            # Procesar basado en el estado y el topic
            if topic_str == 'vehiculos/posicion_inicial':
                # Acepta posición inicial si está inicializando O si se habilitó explícitamente
                if self.estado_sistema == ESTADO_INICIALIZANDO or self.permitir_actualizacion_pos_operando:
                    if 'ubicacion' in comando_actual:
                        ubicacion_anterior = self.posicion_actual
                        self.posicion_actual = comando_actual['ubicacion']
                        self.ultimo_comando_ejecutado = {} # Resetear último comando

                        if self.estado_sistema == ESTADO_INICIALIZANDO:
                            self.estado_sistema = ESTADO_OPERANDO
                            self.debug_serial(f"[Init] Posición inicial recibida: {self.posicion_actual}. Estado cambiado a OPERACIÓN.")
                        else: # Estaba en OPERANDO y permitió la actualización
                            self.permitir_actualizacion_pos_operando = False # Deshabilitar escucha tras actualizar
                            self.debug_serial(f"[OPERANDO-Repos] Posición actualizada por comando UART: {ubicacion_anterior} -> {self.posicion_actual}")
                    else:
                        self.debug_serial("[MQTT Warn] Mensaje de posición inicial sin 'ubicacion'.")
                        # Si estábamos esperando la actualización forzada y falla, mantenemos el flag?
                        # self.permitir_actualizacion_pos_operando = False # Quizás resetear aquí también
                else:
                    # Ignorar si estamos en OPERANDO y no se habilitó la escucha
                    self.debug_serial("[Comando Warn] Mensaje de posición inicial MQTT ignorado en modo OPERANDO (escucha no habilitada).")

            elif topic_str == 'vehiculos/comando':
                # Procesar comandos solo si estamos en el estado correcto
                if self.estado_sistema == ESTADO_INICIALIZANDO:
                    # Aceptar solo comandos de matriz en este estado
                    # ... (código existente para mostrar/apagar matriz en init) ...
                    accion = comando_accion.get('accion')
                    if accion == 'mostrar':
                        fig = comando_accion.get('figura', "identificar")
                        self.controlar_matriz(fig)
                        self.debug_serial(f"[Init-Comando] Ejecución visualización '{fig}' finalizada.")
                    elif accion == 'apagar':
                        self.controlar_matriz("apagar")
                        self.debug_serial(f"[Init-Comando] Ejecución apagar matriz finalizada.")
                    elif 'destino' in comando_accion:
                         self.debug_serial("[Init-Comando Warn] Comando de movimiento ignorado en estado INICIALIZANDO.")
                    else:
                         self.debug_serial(f"[Init-Comando Error] Acción no reconocida o no permitida: {comando_accion}")

                elif self.estado_sistema == ESTADO_OPERANDO:
                     # ... (código existente para procesar comandos de movimiento y matriz en operando) ...
                    if comando_accion == self.ultimo_comando_ejecutado:
                        self.debug_serial("[Comando Info] Comando repetido, ignorando.")
                        return
                    self.debug_serial(f"[Comando] Procesando: {comando_accion}")
                    self.ultimo_comando_ejecutado = comando_accion
                    
                    accion = comando_accion.get('accion')
                    if 'destino' in comando_accion:
                        self.ejecutar_movimiento(comando_accion['destino'])
                        self.debug_serial(f"[Comando] Ejecución movimiento hacia {comando_accion['destino']} finalizada.")
                    elif accion == 'mostrar':
                        fig = comando_accion.get('figura',"identificar")
                        self.controlar_matriz(fig)
                        self.debug_serial(f"[Comando] Ejecución visualización '{fig}' finalizada.")
                    elif accion == 'apagar':
                        self.controlar_matriz("apagar")
                        self.debug_serial(f"[Comando] Ejecución apagar matriz finalizada.")
                    else:
                        self.debug_serial(f"[Comando Error] Acción no reconocida: {comando_accion}")
                        self.ultimo_comando_ejecutado = {}

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
        # if time.ticks_ms() - self.debug_timer > 1000: # Intervalo original del usuario
        #     try:
        #         d_del = self._leer_sensor_promedio(self.sensor_delantero, 3)
        #         d_tra = self._leer_sensor_promedio(self.sensor_trasero, 3)
        #         # Este mensaje periódico SÍ va por serial ahora
        #         self.debug_serial(f"[Sensores Loop] Del:{d_del}mm Tra:{d_tra}mm | Yaw:{self.giro_acel.yaw:.1f}°") # <-- Usar debug_serial (más corto)
        #     except Exception as e:
        #         self.debug_serial(f"[Error lectura sensores loop: {e}") # <-- Usar debug_serial
        #     self.debug_timer = time.ticks_ms() # Resetear timer

        time.sleep_ms(5)


    def _leer_sensor_promedio(self, sensor_obj, num_lecturas):# revisado
        """Lee el sensor varias veces y devuelve el promedio de lecturas válidas."""
        suma_lecturas = 0
        lecturas_validas_count = 0
        errores_consecutivos = 0 # Para detectar fallos persistentes
        MAX_ERRORES_CONSECUTIVOS = 3 # Ajustable

        for i in range(num_lecturas):
            lectura = sensor_obj.medir() # Asume que medir() devuelve -1 en error
            if lectura >= 0: # Considerar solo lecturas válidas (>= 0 mm)
                suma_lecturas += lectura
                lecturas_validas_count += 1
                errores_consecutivos = 0 # Resetear contador de errores
            else:
                # Opcional: Log específico del error si medir() devolviera más detalles
                # self.debug_serial(f"[Sensor Warn] Lectura inválida ({i+1}/{num_lecturas})")
                errores_consecutivos += 1
                if errores_consecutivos >= MAX_ERRORES_CONSECUTIVOS:
                    self.debug_serial(f"[Sensor Error] {MAX_ERRORES_CONSECUTIVOS} errores consecutivos. Abortando promedio.")
                    return -1 # Abortar si hay demasiados errores seguidos


            # --- AJUSTE: Pausa sólo si hay más de una lectura ---
            if num_lecturas > 1:
                 time.sleep_ms(1) # Pequeña pausa entre lecturas
            # ---------------------------------------------------

        if lecturas_validas_count == 0: # Si no hay lecturas válidas, retornar -1
            # Mensaje ya se imprime si hubo errores consecutivos
            # self.debug_serial("[Sensor Error] Ninguna lectura válida en promedio.")
            return -1 # Indicar error
        else:
            promedio = suma_lecturas // lecturas_validas_count # División entera
            # self.debug_serial(f"[Sensor Info] Promedio de {lecturas_validas_count} lecturas: {promedio} mm") # Opcional
            return promedio

    def _obtener_offset_actual(self, distancia_lectura, es_movimiento_adelante):
        """Devuelve el offset del sensor apropiado basado en la distancia y dirección."""
        offset = 0
        if es_movimiento_adelante: # Usar offsets delanteros
            if distancia_lectura <= 100: offset = SENSOR_TARGET_OFFSET_DELANTERO_100MM
            elif distancia_lectura <= 200: offset = SENSOR_TARGET_OFFSET_DELANTERO_200MM
            elif distancia_lectura <= 300: offset = SENSOR_TARGET_OFFSET_DELANTERO_300MM
            elif distancia_lectura <= 400: offset = SENSOR_TARGET_OFFSET_DELANTERO_400MM
            elif distancia_lectura <= 500: offset = SENSOR_TARGET_OFFSET_DELANTERO_500MM
            elif distancia_lectura <= 600: offset = SENSOR_TARGET_OFFSET_DELANTERO_600MM
            elif distancia_lectura <= 700: offset = SENSOR_TARGET_OFFSET_DELANTERO_700MM # Asumiendo que tienes hasta 700
            else: offset = SENSOR_TARGET_OFFSET_DELANTERO_700MM # O el último offset aplicable
        else: # Usar offsets traseros
            if distancia_lectura <= 100: offset = SENSOR_TARGET_OFFSET_TRASERO_100MM
            elif distancia_lectura <= 200: offset = SENSOR_TARGET_OFFSET_TRASERO_200MM
            elif distancia_lectura <= 300: offset = SENSOR_TARGET_OFFSET_TRASERO_300MM
            elif distancia_lectura <= 400: offset = SENSOR_TARGET_OFFSET_TRASERO_400MM
            elif distancia_lectura <= 500: offset = SENSOR_TARGET_OFFSET_TRASERO_500MM
            elif distancia_lectura <= 600: offset = SENSOR_TARGET_OFFSET_TRASERO_600MM
            elif distancia_lectura <= 700: offset = SENSOR_TARGET_OFFSET_TRASERO_700MM # Asumiendo que tienes hasta 700
            else: offset = SENSOR_TARGET_OFFSET_TRASERO_700MM # O el último offset aplicable
        return offset

