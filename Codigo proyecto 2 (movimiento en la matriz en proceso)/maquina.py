# maquina.py
import time
from machine import Pin, I2C
import _thread
import ujson

# Importar las clases de hardware
from motor import Motor
from sensor import SensorVL53L0X
from giro_acel import GiroAcel
# Importar la clase MQTT
from mqtt_pirobot import mqtt_pirobot

# --- Constantes Internas ---
CELL_SIZE_MM = 80.0 # Tamaño de la celda en mm
HALF_CELL_SIZE_MM = CELL_SIZE_MM / 2.0
TOLERANCIA_CENTRO_CELDA = 5.0 # +/- 5mm del centro ideal
DISTANCIA_MINIMA_AVANCE = 10.0 # Distancia mínima (mm) para considerar que ha llegado a la pared/línea

# --- Estados de la Máquina de Movimiento ---
class EstadosMovimiento:
    IDLE = 0
    PROCESSING_PLAN = 1
    TURNING_START = 2
    TURNING_WAIT = 3
    ADVANCING_START = 4
    ADVANCING_MOVE = 5
    ADVANCING_CENTERING_START = 6
    ADVANCING_CENTERING_ADJUST = 7
    FINISHING = 8
    ERROR = 9

# --- Constantes de Tiempo y Velocidad para FSM ---
TIMEOUT_GIRO_MS = 5000       # Timeout máximo para un giro (5s)
TIMEOUT_AVANCE_CELDA_MS = 7000 # Timeout máximo para avanzar una celda (7s)
TIMEOUT_CENTRADO_MS = 4000   # Timeout máximo para centrar (4s)
VELOCIDAD_LENTA_DUTY = 15000 # Duty cycle para ajustes finos de centrado (~23%)
PULSO_AJUSTE_CENTRADO_MS = 50 # Duración del pulso de motor para ajustar centrado


class Maquina:
    def __init__(self, mi_id,
                 ena_pin, in1_pin, in2_pin, enb_pin, in3_pin, in4_pin, # Motor
                 i2c_bus, # I2C Bus obj
                 xshut_trasero, dir_trasero, xshut_delantero, dir_delantero, # Sensores VL53L0X
                 distancia_max, distancia_centro,
                 intervalo_calib, # Giroscopio
                 # pin_matriz_led, # Descomentar si se implementa LED
                 ssid, password, mqtt_broker, mqtt_port, # Red/MQTT
                 topic_pos_inicial, topic_comando,
                 velocidad_base, velocidad_giro, tolerancia_angulo # Movimiento
                 ):

        self.MI_ID = mi_id
        print(f"[Maquina] ID={self.MI_ID} inicializando...")

        # --- Guardar parámetros ---
        self.i2c = i2c_bus
        self.DISTANCIA_DETECCION_OBSTACULO = distancia_max
        self.DISTANCIA_CENTRO_CELDA_IDEAL = distancia_centro
        self.INTERVALO_CALIBRACION_GIRO = intervalo_calib
        self.VELOCIDAD_BASE = velocidad_base
        self.VELOCIDAD_GIRO = velocidad_giro
        self.TOLERANCIA_ANGULO = tolerancia_angulo

        # --- Inicialización Hardware ---
        self.motores = Motor(ena_pin, in1_pin, in2_pin, enb_pin, in3_pin, in4_pin)
        self.sensor_delantero = SensorVL53L0X(self.i2c, dir_delantero, xshut_delantero)
        time.sleep_ms(50)
        self.sensor_trasero = SensorVL53L0X(self.i2c, dir_trasero, xshut_trasero)
        self.giro_acel = GiroAcel(self.i2c, self.INTERVALO_CALIBRACION_GIRO)

        # --- Placeholder Matriz LED ---
        self.matriz_led_disponible = False
        print(f"[Hardware] Matriz LED: {'Implementada (placeholder)' if self.matriz_led_disponible else 'No implementada'}")

        # --- Estado Interno del Robot ---
        self.posicion_actual = None
        self.orientacion_actual = 0.0 # Se actualiza en _yaw_update_task
        self.ultimo_comando_ejecutado = None
        self.comando_pendiente = None
        self.fase_inicializacion = True
        self.accion_en_curso = False
        self.tipo_accion_actual = None
        self.destino_movimiento_actual = None

        # --- Estado de la Máquina de Estados de Movimiento (FSM) ---
        self.movimiento_estado = EstadosMovimiento.IDLE
        self.plan_actual = []           # Lista de pasos: [('girar', angulo_rel), ('avanzar', num_celdas)]
        self.paso_actual_idx = 0        # Índice del paso actual en self.plan_actual
        self.avance_celdas_recorridas = 0 # Celdas completadas en el paso 'avanzar' actual
        self.avance_meta_celdas = 0     # Celdas objetivo para el paso 'avanzar' actual
        self.giro_angulo_objetivo = 0.0 # Ángulo Yaw absoluto objetivo para el giro actual
        self.tiempo_inicio_paso = 0     # ticks_ms al iniciar un estado con timeout
        self.ultimo_error_movimiento = "" # Mensaje del último error

        # --- Configuración Comunicación ---
        self.mqtt = mqtt_pirobot(ssid, password, mqtt_broker, mqtt_port, self.MI_ID)
        self.TOPIC_POSICION_INICIAL = topic_pos_inicial
        self.TOPIC_COMANDO = topic_comando

        # --- Tiempos para lógica no bloqueante ---
        self.tiempo_ultima_lectura_sensores = 0
        self.intervalo_lectura_sensores = 60


    # --- Funciones de Control LED (Placeholders - sin cambios) ---
    def _encender_led_verde(self):
        if self.matriz_led_disponible: print("[LED] Encendiendo VERDE")
        else: print("[LED] (Simulado) Encendiendo VERDE")

    def _apagar_led(self):
        if self.matriz_led_disponible: print("[LED] Apagando")
        else: print("[LED] (Simulado) Apagando")

    def _mostrar_figura_led_async(self, figura):
        """Inicia la visualización (usando FSM simplificada)."""
        if self.accion_en_curso:
            print("[Accion] Ignorado (mostrar LED): Ya hay otra acción en curso.")
            return

        if self.matriz_led_disponible: print(f"[LED] Mostrando '{figura}' (Async)")
        else: print(f"[LED] (Simulado) Mostrando '{figura}'")

        self.accion_en_curso = True
        self.tipo_accion_actual = 'visualizacion'
        self.comando_pendiente = {"accion": "mostrar", "figura": figura}
        # Simulación: Finalizar inmediatamente
        print("[Visualizacion] Placeholder: Acción completada inmediatamente.")
        self._finalizar_accion_actual(exitoso=True)


    # --- Funciones de Movimiento (Planificación - sin cambios) ---
    def _calcular_movimiento(self, destino_str):
        # (Misma lógica que la versión anterior)
        if not self.posicion_actual:
            print("Error: No se puede calcular movimiento sin posición actual.")
            return None
        try:
            current_x, current_y = map(int, self.posicion_actual.split(','))
            target_x, target_y = map(int, destino_str.split(','))
        except ValueError:
            print(f"Error: Formato de coordenadas inválido (Actual:'{self.posicion_actual}' o Destino:'{destino_str}')")
            return None
        except TypeError:
             print(f"Error: Posición actual ({self.posicion_actual}) no es válida para split.")
             return None

        delta_x = target_x - current_x
        delta_y = target_y - current_y
        pasos = []
        orientacion_temporal = self.giro_acel.get_yaw() # Usar yaw actual real

        print(f"[Movimiento] Calculando desde {self.posicion_actual} ({orientacion_temporal:.1f}°) hacia {destino_str}")
        if delta_x == 0 and delta_y == 0:
             print("[Movimiento] Destino es la posición actual. No se requieren pasos.")
             return [], destino_str # Devuelve plan vacío

        # Mover en X primero
        if delta_x != 0:
            orientacion_deseada = 90.0 if delta_x > 0 else 270.0 # Este o Oeste
            giro_necesario = self._calcular_angulo_giro(orientacion_deseada, orientacion_temporal)
            if abs(giro_necesario) > self.TOLERANCIA_ANGULO:
                 pasos.append(("girar", giro_necesario))
                 orientacion_temporal += giro_necesario # Actualizar orientación para el siguiente paso
            num_pasos_x = abs(delta_x)
            if num_pasos_x > 0: pasos.append(("avanzar", num_pasos_x))

        # Mover en Y después
        if delta_y != 0:
            orientacion_deseada = 0.0 if delta_y > 0 else 180.0 # Norte o Sur
            giro_necesario = self._calcular_angulo_giro(orientacion_deseada, orientacion_temporal)
            if abs(giro_necesario) > self.TOLERANCIA_ANGULO:
                 pasos.append(("girar", giro_necesario))
            num_pasos_y = abs(delta_y)
            if num_pasos_y > 0: pasos.append(("avanzar", num_pasos_y))

        print(f"[Movimiento] Plan calculado para ir a {destino_str}: {pasos}")
        return pasos, destino_str

    def _calcular_angulo_giro(self, angulo_deseado, angulo_actual):
        # (Misma lógica que la versión anterior)
        angulo_actual_norm = (angulo_actual + 180) % 360 - 180
        angulo_deseado_norm = (angulo_deseado + 180) % 360 - 180
        delta = angulo_deseado_norm - angulo_actual_norm
        if delta > 180: delta -= 360
        elif delta <= -180: delta += 360
        return delta

    # --- Funciones de Movimiento (Ejecución con FSM) ---

    def _ejecutar_movimiento_async(self, plan_movimiento):
        """Inicia la máquina de estados para ejecutar el plan de movimiento."""
        pasos, destino_final = plan_movimiento

        if self.accion_en_curso:
            print("[Accion] Ignorado (movimiento): Ya hay otra acción en curso.")
            return

        if not pasos:
             print("[Movimiento] No hay pasos a ejecutar (destino es actual).")
             # Considerar éxito inmediato si no hay pasos
             self._finalizar_accion_actual(exitoso=True, destino_alcanzado=destino_final)
             return

        print(f"[Movimiento FSM] Iniciando ejecución para {destino_final} con plan: {pasos}")
        self.accion_en_curso = True
        self.tipo_accion_actual = 'movimiento'
        self.destino_movimiento_actual = destino_final
        self.comando_pendiente = {"destino": destino_final}
        self._apagar_led()

        # Inicializar FSM
        self.plan_actual = pasos
        self.paso_actual_idx = 0
        self.ultimo_error_movimiento = ""
        self.movimiento_estado = EstadosMovimiento.PROCESSING_PLAN # Estado inicial

    def _procesar_maquina_estados_movimiento(self):
        """Ejecuta un paso de la máquina de estados de movimiento. Llamar desde loop()."""
        if not self.accion_en_curso or self.tipo_accion_actual != 'movimiento':
            return # No hacer nada si no hay movimiento activo

        ahora = time.ticks_ms()
        estado_actual = self.movimiento_estado # Copia local para claridad

        # --- Procesamiento de Estados ---
        if estado_actual == EstadosMovimiento.IDLE:
            # No debería estar aquí si accion_en_curso es True, pero por seguridad:
            self._finalizar_accion_actual(exitoso=False, error="Estado IDLE inesperado")
            return

        elif estado_actual == EstadosMovimiento.PROCESSING_PLAN:
            print(f"[FSM] Estado: PROCESSING_PLAN (Paso Idx: {self.paso_actual_idx})")
            if self.paso_actual_idx >= len(self.plan_actual):
                # Plan completado
                print("[FSM] Plan completado.")
                self.movimiento_estado = EstadosMovimiento.FINISHING
            else:
                accion, valor = self.plan_actual[self.paso_actual_idx]
                print(f"[FSM] Siguiente paso: {accion}, Valor: {valor}")
                if accion == "girar":
                    # Calcular ángulo absoluto objetivo
                    self.giro_angulo_objetivo = self.giro_acel.get_yaw() + valor
                    print(f"[FSM] Giro objetivo Yaw: {self.giro_angulo_objetivo:.1f}° (Relativo: {valor:.1f}°)")
                    self.movimiento_estado = EstadosMovimiento.TURNING_START
                elif accion == "avanzar":
                    self.avance_meta_celdas = valor
                    self.avance_celdas_recorridas = 0
                    print(f"[FSM] Avance objetivo: {valor} celdas")
                    if valor <= 0: # Si el plan pide avanzar 0 celdas, saltar paso
                         self.paso_actual_idx += 1
                         self.movimiento_estado = EstadosMovimiento.PROCESSING_PLAN # Volver a procesar
                    else:
                         self.movimiento_estado = EstadosMovimiento.ADVANCING_START
                else:
                    self._finalizar_accion_actual(exitoso=False, error=f"Acción desconocida en plan: {accion}")

        # --- Estados de Giro ---
        elif estado_actual == EstadosMovimiento.TURNING_START:
            print("[FSM] Estado: TURNING_START")
            # Determinar dirección del giro
            angulo_relativo = self._calcular_angulo_giro(self.giro_angulo_objetivo, self.giro_acel.get_yaw())
            if abs(angulo_relativo) <= self.TOLERANCIA_ANGULO:
                 print("[FSM] Giro innecesario o ya completado.")
                 self.paso_actual_idx += 1
                 self.movimiento_estado = EstadosMovimiento.PROCESSING_PLAN
            else:
                 print(f"[FSM] Iniciando giro ({angulo_relativo:.1f}°)...")
                 if angulo_relativo > 0: # Girar Derecha (Yaw aumenta) - AJUSTAR SIGNO SI ES NECESARIO
                     self.motores.girar_derecha(self.VELOCIDAD_GIRO)
                 else: # Girar Izquierda (Yaw disminuye) - AJUSTAR SIGNO SI ES NECESARIO
                     self.motores.girar_izquierda(self.VELOCIDAD_GIRO)
                 self.tiempo_inicio_paso = ahora
                 self.movimiento_estado = EstadosMovimiento.TURNING_WAIT

        elif estado_actual == EstadosMovimiento.TURNING_WAIT:
            # Ya está girando, comprobar si llegó al ángulo
            yaw_actual = self.giro_acel.get_yaw()
            error_angulo = self._calcular_angulo_giro(self.giro_angulo_objetivo, yaw_actual)
            # print(f"[FSM] Estado: TURNING_WAIT (Actual: {yaw_actual:.1f}, Objetivo: {self.giro_angulo_objetivo:.1f}, Error: {error_angulo:.1f})") # Debug

            if abs(error_angulo) <= self.TOLERANCIA_ANGULO:
                print("[FSM] Giro completado.")
                self.motores.detener(modo_freno=True)
                # Actualizar orientación precisa al finalizar? O confiar en el Yaw del thread?
                # self.orientacion_actual = self.giro_angulo_objetivo # Podría introducir error acumulado
                self.paso_actual_idx += 1
                self.movimiento_estado = EstadosMovimiento.PROCESSING_PLAN
            elif time.ticks_diff(ahora, self.tiempo_inicio_paso) > TIMEOUT_GIRO_MS:
                self._finalizar_accion_actual(exitoso=False, error="Timeout durante giro")
            # else: Sigue girando, no cambiar estado

        # --- Estados de Avance ---
        elif estado_actual == EstadosMovimiento.ADVANCING_START:
            print(f"[FSM] Estado: ADVANCING_START (Celda {self.avance_celdas_recorridas + 1}/{self.avance_meta_celdas})")
            # Aquí asumimos que la orientación ya es correcta por un giro previo
            # TODO: Determinar si avanzar o retroceder basado en la orientación actual
            # Por simplicidad, asumimos que 'avanzar' siempre es hacia adelante relativo al robot
            print("[FSM] Iniciando avance...")
            self.motores.avanzar(self.VELOCIDAD_BASE)
            self.tiempo_inicio_paso = ahora
            self.movimiento_estado = EstadosMovimiento.ADVANCING_MOVE

        elif estado_actual == EstadosMovimiento.ADVANCING_MOVE:
            # Moviendo hacia adelante, comprobar sensores
            dist_f, dist_t = self._leer_sensores_distancia()
            # print(f"[FSM] Estado: ADVANCING_MOVE (F: {dist_f}, T: {dist_t})") # Debug

            # Comprobar obstáculo inmediato
            if dist_f != -1 and dist_f < self.DISTANCIA_DETECCION_OBSTACULO:
                print(f"[FSM] ¡Obstáculo detectado adelante a {dist_f} mm!")
                self._finalizar_accion_actual(exitoso=False, error="Obstáculo detectado")
                return # Salir inmediatamente

            # Comprobar si hemos llegado a la línea/pared de la siguiente celda
            # Condición simple: distancia frontal muy corta
            if dist_f != -1 and dist_f < DISTANCIA_MINIMA_AVANCE:
                print(f"[FSM] Línea/Pared detectada adelante (dist: {dist_f}). Deteniendo para centrar.")
                self.motores.detener(modo_freno=True)
                self.avance_celdas_recorridas += 1
                print(f"[FSM] Celda {self.avance_celdas_recorridas}/{self.avance_meta_celdas} alcanzada.")
                self.movimiento_estado = EstadosMovimiento.ADVANCING_CENTERING_START
            elif time.ticks_diff(ahora, self.tiempo_inicio_paso) > TIMEOUT_AVANCE_CELDA_MS:
                 self._finalizar_accion_actual(exitoso=False, error="Timeout avanzando celda")
            # else: Sigue avanzando

        # --- Estados de Centrado ---
        elif estado_actual == EstadosMovimiento.ADVANCING_CENTERING_START:
             print("[FSM] Estado: ADVANCING_CENTERING_START")
             # Preparar para ajustar posición
             self.tiempo_inicio_paso = ahora
             self.movimiento_estado = EstadosMovimiento.ADVANCING_CENTERING_ADJUST

        elif estado_actual == EstadosMovimiento.ADVANCING_CENTERING_ADJUST:
            dist_f, dist_t = self._leer_sensores_distancia()
            # print(f"[FSM] Estado: ADVANCING_CENTERING_ADJUST (F: {dist_f}, T: {dist_t})") # Debug

            if dist_f == -1: # Error sensor delantero
                 self._finalizar_accion_actual(exitoso=False, error="Error sensor delantero durante centrado")
                 return

            error_centro = dist_f - self.DISTANCIA_CENTRO_CELDA_IDEAL
            # print(f"[FSM] Error centrado: {error_centro:.1f} mm") # Debug

            if abs(error_centro) <= TOLERANCIA_CENTRO_CELDA:
                 print("[FSM] Centrado completado.")
                 self.motores.detener(modo_freno=True)
                 # Verificar si hemos completado todas las celdas de este paso 'avanzar'
                 if self.avance_celdas_recorridas >= self.avance_meta_celdas:
                     print("[FSM] Paso 'avanzar' completado.")
                     self.paso_actual_idx += 1
                     self.movimiento_estado = EstadosMovimiento.PROCESSING_PLAN
                 else:
                     # Aún quedan celdas por avanzar en este paso
                     print("[FSM] Necesita avanzar más celdas...")
                     self.movimiento_estado = EstadosMovimiento.ADVANCING_START # Volver a avanzar
            elif time.ticks_diff(ahora, self.tiempo_inicio_paso) > TIMEOUT_CENTRADO_MS:
                 self._finalizar_accion_actual(exitoso=False, error="Timeout durante centrado")
            else:
                 # Necesita ajuste
                 if error_centro > 0: # Demasiado lejos, avanzar un poco
                     # print("[FSM] Ajuste: Avanzar...") # Debug
                     self.motores.avanzar(VELOCIDAD_LENTA_DUTY)
                     time.sleep_ms(PULSO_AJUSTE_CENTRADO_MS) # Mover por un corto tiempo
                     self.motores.detener(modo_freno=False) # Detener sin freno brusco
                 else: # Demasiado cerca, retroceder un poco
                     # print("[FSM] Ajuste: Retroceder...") # Debug
                     self.motores.retroceder(VELOCIDAD_LENTA_DUTY)
                     time.sleep_ms(PULSO_AJUSTE_CENTRADO_MS) # Mover por un corto tiempo
                     self.motores.detener(modo_freno=False)
                 # Permanecer en ADVANCING_CENTERING_ADJUST para re-evaluar en la próxima iteración
                 # Añadir una pequeña pausa para que el robot se asiente?
                 # time.sleep_ms(50)

        # --- Estados Finales ---
        elif estado_actual == EstadosMovimiento.FINISHING:
            print("[FSM] Estado: FINISHING")
            # La acción se finaliza aquí
            self._finalizar_accion_actual(exitoso=True, destino_alcanzado=self.destino_movimiento_actual)
            # El estado se cambiará a IDLE dentro de _finalizar_accion_actual

        elif estado_actual == EstadosMovimiento.ERROR:
            print(f"[FSM] Estado: ERROR ({self.ultimo_error_movimiento})")
            # La acción ya se finalizó como fallida
            # Asegurarse de estar en IDLE
             self.movimiento_estado = EstadosMovimiento.IDLE
             self.accion_en_curso = False # Doble check

        else:
             # Estado desconocido
             self._finalizar_accion_actual(exitoso=False, error=f"Estado FSM desconocido: {estado_actual}")


    def _finalizar_accion_actual(self, exitoso, destino_alcanzado=None, error=""):
        """Rutina común para finalizar la acción actual (movimiento o visualización)."""
        tipo_accion = self.tipo_accion_actual if self.tipo_accion_actual else "Desconocida"
        print(f"[{tipo_accion.capitalize()}] Acción finalizada. Éxito: {exitoso}")

        if tipo_accion == 'movimiento':
            self.motores.detener(modo_freno=True) # Asegurar motores detenidos
            if not exitoso:
                 self.ultimo_error_movimiento = error
                 print(f"ERROR Movimiento: {error}")
                 # No actualizamos posición si falló
            elif destino_alcanzado:
                # Actualizar posición SOLO si el destino coincide con el esperado
                if destino_alcanzado == self.destino_movimiento_actual:
                    self.posicion_actual = destino_alcanzado
                    print(f"[Estado] Nueva posición: {self.posicion_actual}")
                    # La orientación la obtenemos del thread de Yaw
                    self.orientacion_actual = self.giro_acel.get_yaw()
                    print(f"[Estado] Orientación final: {self.orientacion_actual:.1f}°")
                else:
                    print(f"ERROR LÓGICO: Movimiento exitoso pero destino final ({destino_alcanzado}) no coincide con el esperado ({self.destino_movimiento_actual})")
                    exitoso = False # Marcar como fallo
                    self.ultimo_error_movimiento = "Discrepancia en destino final"

        elif tipo_accion == 'visualizacion':
            self._apagar_led()

        # Resetear flags y estado FSM
        self.accion_en_curso = False
        self.tipo_accion_actual = None
        self.destino_movimiento_actual = None
        self.movimiento_estado = EstadosMovimiento.IDLE # Volver a IDLE
        self.plan_actual = []
        self.paso_actual_idx = 0

        # Actualizar último comando ejecutado SI la acción tuvo éxito
        if exitoso and self.comando_pendiente:
            self.ultimo_comando_ejecutado = self.comando_pendiente.copy()
            print(f"[Estado] ultimo_comando_ejecutado actualizado a: {self.ultimo_comando_ejecutado}")
        elif not exitoso and self.comando_pendiente:
             print(f"[Estado] Acción fallida, ultimo_comando_ejecutado NO actualizado (era {self.ultimo_comando_ejecutado}).")

        # Limpiar comando pendiente
        self.comando_pendiente = None


    # --- Lógica Principal y Comunicación (sin cambios significativos) ---

    def iniciar(self):
        """Inicializa conexiones y espera la posición inicial."""
        if not self.giro_acel or not self.giro_acel.initialized:
             print("ERROR CRÍTICO: Giroscopio no inicializado. Abortando.")
             raise SystemExit("Fallo al inicializar Giroscopio")
        if not self.sensor_delantero or not self.sensor_delantero.initialized:
             print("ERROR CRÍTICO: Sensor delantero no inicializado. Abortando.")
             raise SystemExit("Fallo al inicializar Sensor Delantero")
        if not self.sensor_trasero or not self.sensor_trasero.initialized:
             print("ERROR CRÍTICO: Sensor trasero no inicializado. Abortando.")
             raise SystemExit("Fallo al inicializar Sensor Trasero")

        print("[Sistema] Iniciando hilo para actualización de Yaw...")
        try:
            _thread.start_new_thread(self._yaw_update_task, ())
        except Exception as e:
            print(f"Error CRÍTICO iniciando hilo de Yaw: {e}")
            raise SystemExit("Fallo al iniciar hilo de Yaw")

        print("[Sistema] Conectando a red y MQTT...")
        if self.mqtt.connect_wifi():
            if self.mqtt.connect_mqtt():
                self.mqtt.set_callback_handler(self._procesar_mensaje_mqtt)
                self.mqtt.subscribe(self.TOPIC_POSICION_INICIAL)
                self._encender_led_verde()
                self.fase_inicializacion = True
                print("[Sistema] Conectado. Esperando posición inicial...")
            else: print("ERROR CRÍTICO: No se pudo conectar a MQTT.")
        else: print("ERROR CRÍTICO: No se pudo conectar a WiFi.")


    def _procesar_mensaje_mqtt(self, topic, payload):
        """Callback principal que recibe mensajes."""
        # (Lógica interna sin cambios respecto a la versión anterior)
        # ... (código _procesar_mensaje_mqtt omitido por brevedad) ...
        # --- Fase de Inicialización: Esperando Posición ---
        if self.fase_inicializacion:
            if topic == self.TOPIC_POSICION_INICIAL.decode('utf-8'):
                if isinstance(payload, dict) and 'id_vehiculo' in payload and 'ubicacion' in payload:
                    id_recibido = payload['id_vehiculo']
                    if id_recibido == self.MI_ID:
                        print(f"[Init] ¡Mensaje para mí ({self.MI_ID})!")
                        try:
                            x_str, y_str = payload['ubicacion'].split(',')
                            x, y = int(x_str), int(y_str)
                            self.posicion_actual = payload['ubicacion']
                            print(f"[Init] Posición inicial válida recibida: {self.posicion_actual}")
                            self._apagar_led()
                            self.mqtt.unsubscribe(self.TOPIC_POSICION_INICIAL)
                            self.mqtt.subscribe(self.TOPIC_COMANDO)
                            self.fase_inicializacion = False
                            self.ultimo_comando_ejecutado = None
                            self.comando_pendiente = None
                            self.giro_acel.reset_yaw(0.0)
                            print("[System] Fase de inicialización completada. Orientación reseteada a 0. Listo para comandos.")
                        except (ValueError, TypeError): print(f"[Init] Ignorado: Ubicación '{payload['ubicacion']}' no tiene formato X,Y válido.")
                        except Exception as e: print(f"[Init] Error procesando ubicación: {e}")
            return

        # --- Fase de Operación: Procesando Comandos ---
        if topic == self.TOPIC_COMANDO.decode('utf-8'):
            if isinstance(payload, dict) and 'id_vehiculo' in payload:
                id_recibido = payload['id_vehiculo']
                if id_recibido == self.MI_ID:
                    comando_actual = {k: v for k, v in payload.items() if k != 'id_vehiculo'}
                    if comando_actual == self.ultimo_comando_ejecutado: return # Ignorar repetido
                    if self.accion_en_curso:
                         print(f"[Comando] Ignorado: Acción '{self.tipo_accion_actual}' aún en curso. Comando: {comando_actual}")
                         return

                    print(f"[Comando] Procesando nuevo comando: {comando_actual}")
                    if "destino" in comando_actual:
                        destino = comando_actual["destino"]
                        try:
                            x_str, y_str = destino.split(',')
                            x, y = int(x_str), int(y_str)
                            plan = self._calcular_movimiento(destino)
                            if plan is not None: # Devuelve None si hay error, o ([], dest) si ya está ahí
                                self._ejecutar_movimiento_async(plan) # Inicia la FSM
                            else: print("[Comando] No se pudo calcular plan. Comando ignorado.")
                        except (ValueError, TypeError): print(f"[Comando] Ignorado: Destino '{destino}' no tiene formato X,Y válido.")
                        except Exception as e: print(f"[Comando] Error procesando destino: {e}")
                    elif "accion" in comando_actual and comando_actual["accion"] == "mostrar":
                        if "figura" in comando_actual: self._mostrar_figura_led_async(comando_actual["figura"])
                        else: print("[Comando] Ignorado: Acción 'mostrar' sin clave 'figura'.")
                    else: print(f"[Comando] Ignorado: Tipo de comando no reconocido: {comando_actual}")


    def _leer_sensores_distancia(self):
        """Lee ambos sensores y retorna (dist_f, dist_t). Retorna (-1, -1) en error."""
        # (Sin cambios)
        dist_f = -1; dist_t = -1
        try:
            dist_f = self.sensor_delantero.medir()
            dist_t = self.sensor_trasero.medir()
        except Exception as e: print(f"Error leyendo sensores de distancia: {e}")
        return dist_f, dist_t

    def _check_obstaculos(self):
        """Verifica si hay obstáculos cercanos."""
        # (Sin cambios)
        dist_f, dist_t = self._leer_sensores_distancia()
        obstaculo_delante = (dist_f != -1 and dist_f <= self.DISTANCIA_DETECCION_OBSTACULO)
        obstaculo_atras = (dist_t != -1 and dist_t <= self.DISTANCIA_DETECCION_OBSTACULO)
        return obstaculo_delante, obstaculo_atras

    def _yaw_update_task(self):
        """Tarea en hilo para actualizar Yaw y recalibrar giroscopio."""
        # (Sin cambios)
        print("[Thread Yaw] Iniciado.")
        while True:
            try:
                if self.giro_acel and self.giro_acel.initialized:
                    self.giro_acel.actualizar_yaw()
                    # Actualizar la variable de instancia directamente aquí
                    self.orientacion_actual = self.giro_acel.get_yaw()
                    self.giro_acel._calibrar_periodicamente()
                else: time.sleep_ms(100)
                time.sleep_ms(10)
            except Exception as e:
                print(f"Error CRÍTICO irrecuperable en hilo de Yaw: {e}")
                import sys
                sys.print_exception(e)
                break


    def loop(self):
        """Bucle principal de operación del robot."""
        # --- Intentar reconectar MQTT ---
        if not self.mqtt.connected:
             self.mqtt.connect_wifi()
             time.sleep_ms(100)
             self.mqtt.connect_mqtt()

        # --- Revisar mensajes MQTT ---
        if self.mqtt.connected:
            self.mqtt.check_msg() # Llama a _procesar_mensaje_mqtt si hay mensajes

        # --- Procesar Máquina de Estados de Movimiento ---
        if self.accion_en_curso and self.tipo_accion_actual == 'movimiento':
            try:
                self._procesar_maquina_estados_movimiento()
            except Exception as e:
                 print(f"Error CRÍTICO procesando FSM de movimiento: {e}")
                 import sys
                 sys.print_exception(e)
                 self._finalizar_accion_actual(exitoso=False, error="Excepción en FSM")

        # --- Lectura de sensores en idle (opcional) ---
        # current_time = time.ticks_ms()
        # if not self.accion_en_curso and time.ticks_diff(current_time, self.tiempo_ultima_lectura_sensores) > self.intervalo_lectura_sensores:
        #      self._check_obstaculos() # Solo informativo
        #      self.tiempo_ultima_lectura_sensores = current_time


    def detener_emergencia(self):
        """Detiene los motores inmediatamente y cancela acción en curso."""
        # (Sin cambios)
        print("¡DETENIENDO MOTORES POR EMERGENCIA!")
        self.motores.detener(modo_freno=True)
        if self.accion_en_curso:
            print(f"Cancelando acción en curso: {self.tipo_accion_actual}")
            self._finalizar_accion_actual(exitoso=False, error="Detención de emergencia")


    def apagar_led(self):
        """Interfaz pública para apagar LED (placeholder)."""
        # (Sin cambios)
        self._apagar_led()

    def desconectar_mqtt(self):
        """Desconecta limpiamente de MQTT."""
        # (Sin cambios)
        self.mqtt.disconnect()

    def __del__(self):
        """Limpieza al destruir el objeto."""
        # (Sin cambios)
        print("[Maquina] Desinicializando...")
        self.detener_emergencia()
        self.desconectar_mqtt()
        if hasattr(self.motores, 'deinit'): self.motores.deinit()

