# config.py

# --- Constantes Generales ---
ESTADO_INICIALIZANDO = 0
ESTADO_OPERANDO = 1
CELDA_SIZE_MM = 80
GIRO_TOLERANCIA_DEG = 20 #20 mejor opcion
MIN_TARGET_DIST_MM = 10
# VELOCIDAD_GIRO = 30000 #120 mejor opcion # Comentada o eliminada si no se usa directamente
# VELOCIDAD_MOVIMIENTO = 40000 # Redefinida abajo para PID # Comentada o eliminada
PAUSA_ENTRE_MOVIMIENTOS_MS = 100

UMBRAL_DERIVA_YAW_GPS = 0.5
DURACION_CHECK_YAW_MS = 5000

# --- Constantes PID para Movimiento Recto ---
VELOCIDAD_MOVIMIENTO = 40000 # Velocidad base PWM (0-65535)
KP_MOVE = 2000  # Ganancia Proporcional para corrección Yaw durante movimiento
KI_MOVE = 30     # Ganancia Integral
KD_MOVE = 0     # Ganancia Derivativa
PID_MOVE_MAX_INTEGRAL = 1000.0 # Límite anti-windup Integral
PID_MOVE_MIN_INTEGRAL = -1000.0
MIN_PWM_MOVE = 30000 # PWM mínimo para evitar que los motores se detengan
NUM_LECTURAS_SENSOR_MOVE = 2 # Lecturas de sensor por ciclo PID de movimiento

# --- Constantes PID para Giros sobre Eje ---
KP_GIRO = 1000   # Ganancia Proporcional (¡AJUSTAR!)
KI_GIRO = 400      # Ganancia Integral (¡AJUSTAR!)
KD_GIRO = 0      # Ganancia Derivativa (¡AJUSTAR!)
PID_GIRO_MAX_INTEGRAL = 5000.0 # Límite anti-windup Integral
PID_GIRO_MIN_INTEGRAL = -5000.0
PID_GIRO_TOLERANCIA_DEG = 0.5 # Tolerancia en grados para considerar el giro completado
VELOCIDAD_GIRO_MAX = 50000 # Velocidad máxima PWM durante el giro PID (0-65535)
VELOCIDAD_GIRO_MIN = 20000  # Velocidad mínima PWM para vencer fricción (0-65535)

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
brillo_matriz = 0.1 # Brillo de la matriz (0.0 a 1.0)
color = (150, 156, 18)
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