# config.py

# --- Constantes Generales ---
ESTADO_INICIALIZANDO = 0
ESTADO_OPERANDO = 1
CELDA_SIZE_MM = 70
MIN_TARGET_DIST_MM = 20
PAUSA_ENTRE_MOVIMIENTOS_MS = 1

UMBRAL_DERIVA_YAW_GPS = 0.2
DURACION_CHECK_YAW_MS = 5000

# --- Constantes PID para Movimiento Recto ---
VELOCIDAD_MOVIMIENTO = 40000 # Velocidad base PWM (0-65535)
MIN_PWM_MOVE = 30000 # PWM mínimo para evitar que los motores se detengan

KP_MOVE = 2000  # Ganancia Proporcional para corrección Yaw durante movimiento
KI_MOVE = 1500     # Ganancia Integral
KD_MOVE = 0     # Ganancia Derivativa
# --- NUEVAS CONSTANTES PARA MOVIMIENTO HACIA ATRÁS ---
KP_MOVE_BACK = 2000 # Ajustar experimentalmente
KI_MOVE_BACK = 450 # Ajustar experimentalmente
KD_MOVE_BACK = 100  # Ajustar experimentalmente

PID_MOVE_MAX_INTEGRAL = 2000.0 # Límite anti-windup Integral
PID_MOVE_MIN_INTEGRAL = -2000.0
NUM_LECTURAS_SENSOR_MOVE = 2 # Lecturas de sensor por ciclo PID de movimiento

DISTANCIA_REDUCCION_MM = 50 #distancia a la que se reduce la velocidad
VELOCIDAD_REDUCCION = MIN_PWM_MOVE
TOLERANCIA_DISTANCIA_FINAL_MM = 1 # Margen de error aceptable en mm
VELOCIDAD_CORRECCION_FINAL = 20000 # Velocidad lenta para ajuste (reducida)
TIMEOUT_CORRECCION_MS = 5000 # Timeout para la corrección final
MAX_INTENTOS_CORRECCION = 20 # Máximos intentos para la corrección final de distancia
CORRECTION_PULSE_FWD_MS_LARGE_ERROR = 80 # Duración ms pulso corrección ADELANTE para errores > 3mm
CORRECTION_PULSE_FWD_MS_SMALL_ERROR = 25  # Duración ms pulso corrección ADELANTE para errores <= 3mm
CORRECTION_PULSE_BWD_MS_LARGE_ERROR = 80 # Duración ms pulso corrección ATRAS para errores > 3mm
CORRECTION_PULSE_BWD_MS_SMALL_ERROR = 50  # Duración ms pulso corrección ATRAS para errores <= 3mm

# --- Constantes para Offset de Sensores en Movimiento ---
# Ajuste experimental para compensar desfase del sensor.
# POSITIVO (+X): Si el robot se detiene ANTES de lo esperado (necesita ir más lejos).
# NEGATIVO (-X): Si el robot se detiene DESPUÉS de lo esperado (necesita ir menos lejos).
#SENSOR_TARGET_OFFSET_DELANTERO_MM = 25 # A cortas distancias es 40
#SENSOR_TARGET_OFFSET_TRASERO_MM = 50   # Ejemplo: Poner +5 si se detiene 5mm antes al retroceder. Poner -8 si se detiene 8mm después.
#positivo si el sensor mide mas distancia que la real
#negativo si el sensor mide menos distancia que la real

# --- NUEVOS OFFSETS PARA DIFERENTES DISTANCIAS ---
# Sensor delantero
SENSOR_TARGET_OFFSET_DELANTERO_100MM = 28  # Para distancias hasta 100mm
SENSOR_TARGET_OFFSET_DELANTERO_200MM = 15  # Para distancias hasta 200mm
SENSOR_TARGET_OFFSET_DELANTERO_300MM = 24  # Para distancias hasta 300mm
SENSOR_TARGET_OFFSET_DELANTERO_400MM = 29  # Para distancias hasta 400mm
SENSOR_TARGET_OFFSET_DELANTERO_500MM = 28  # Para distancias hasta 500mm
SENSOR_TARGET_OFFSET_DELANTERO_600MM = 28  # Para distancias hasta 600mm
SENSOR_TARGET_OFFSET_DELANTERO_700MM = 25  # Para distancias de 700mm o más

# Sensor trasero
SENSOR_TARGET_OFFSET_TRASERO_100MM = 30   # Para distancias hasta 100mm
SENSOR_TARGET_OFFSET_TRASERO_200MM = 34   # Para distancias hasta 200mm
SENSOR_TARGET_OFFSET_TRASERO_300MM = 41   # Para distancias hasta 300mm
SENSOR_TARGET_OFFSET_TRASERO_400MM = 44   # Para distancias hasta 400mm
SENSOR_TARGET_OFFSET_TRASERO_500MM = 47   # Para distancias hasta 500mm
SENSOR_TARGET_OFFSET_TRASERO_600MM = 49   # Para distancias hasta 600mm
SENSOR_TARGET_OFFSET_TRASERO_700MM = 44   # Para distancias de 700mm o más

# --- NUEVAS CONSTANTES PID POR DIRECCIÓN DE GIRO ---
# Giro a la Izquierda (error positivo -> necesita girar más a la izquierda)
KP_GIRO_IZQ = 1000
KI_GIRO_IZQ = 0
KD_GIRO_IZQ = 100

# Giro a la Derecha (error negativo -> necesita girar más a la derecha)
KP_GIRO_DER = 1000 # Inicialmente iguales, ajustar si es necesario
KI_GIRO_DER = 0  # Inicialmente iguales, ajustar si es necesario
KD_GIRO_DER = 100   # Inicialmente iguales, ajustar si es necesario
# --- FIN NUEVAS CONSTANTES ---

GIRO_TOLERANCIA_DEG = 0.1
PID_GIRO_MAX_INTEGRAL = 1000.0 # Límite anti-windup Integral
PID_GIRO_MIN_INTEGRAL = -1000.0
PID_GIRO_TOLERANCIA_DEG = 0.1 # Tolerancia en grados para considerar el giro completado
VELOCIDAD_GIRO_MAX = 50000 # Velocidad máxima PWM durante el giro PID (0-65535)
VELOCIDAD_GIRO_MIN = 40000  # Velocidad mínima PWM para vencer fricción (0-65535)

# --- NUEVO: Constante para giros incrementales ---
GRADOS_POR_MINIGIRO = 22.5       # Tamaño de cada paso de giro incremental
PAUSA_ENTRE_MINIGIROS_MS = 200   # Pausa en ms entre cada paso

# --- NUEVO: Constantes para corrección de giro por pulsos ---
UMBRAL_ERROR_GIRO_PULSO = 2.0      # Error (deg) por debajo del cual usar pulsos
VELOCIDAD_GIRO_PULSO = VELOCIDAD_GIRO_MIN # Usar la velocidad mínima de giro para los pulsos
DURACION_PULSO_GIRO_MS = 10        # Duración de cada pulso de corrección (ms)
PAUSA_ENTRE_PULSOS_GIRO_MS = 100   # Pausa después de un pulso antes de reevaluar (ms)

COLORES = {
    "rojo": (255, 0, 0),
    "verde": (0, 255, 0),
    "azul": (0, 0, 255),
    "blanco": (255, 255, 255),
    "amarillo": (255, 255, 0),
    "cyan": (0, 255, 255),
    "magenta": (255, 0, 255),
    "apagado": (0, 0, 0) # Útil para apagar o color por defecto
}
COLOR_POR_DEFECTO_NOMBRE = "blanco" # Color si no se especifica uno válido
FIGURA_POR_DEFECTO = "cara_feliz" # Figura si no se especifica una válida
pin_matriz = 18 # Pin GPIO para la matriz de LEDs
brillo_matriz = 1 # Brillo de la matriz (0.0 a 1.0)
color = (0, 255, 0)
cara_feliz = [
  [0, 0, 0, 0, 0, 0, 0, 0],
  [0, 0, 1, 0, 0, 1, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0],
  [0, 1, 0, 0, 0, 0, 1, 0],
  [0, 1, 0, 0, 0, 0, 1, 0],
  [0, 0, 1, 0, 0, 1, 0, 0],
  [0, 0, 0, 1, 1, 0, 0, 0],
  [0, 0, 0, 0, 0, 0, 0, 0],
]


