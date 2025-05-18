# --- Configuraciones MQTT (Basado en prueba_mostrar.txt) ---
MQTT_BROKER_IP = "192.168.22.157" # IP de tu Broker MQTT [cite: 5]
MQTT_PORT = 1883                   # Puerto estándar de MQTT [cite: 5]
CLIENT_ID_PUB = "raspberry_pi_tester" # ID para este script publicador [cite: 5]
TARGET_VEHICLE_ID = "vehiculo_1"   # ID del vehículo a controlar [cite: 5]

# --- Topics MQTT (Basado en prueba_mostrar.txt) ---
TOPIC_POSICION_INICIAL = "vehiculos/posicion_inicial" # [cite: 5]
TOPIC_COMANDO = "vehiculos/comando"                 # [cite: 5]

# --- Comandos para la Matriz LED (Basado en comentarios de prueba_mostrar.txt) ---
COMANDO_IDENTIFICAR = "identificar" # Comando para encender la matriz para detección [cite: 13]
COMANDO_APAGAR = "apagar"         # Comando para apagar la matriz
COMANDO_CARA_FELIZ = "cara_feliz" # Comando para mostrar cara feliz [cite: 12]

# --- Configuraciones de Visión (Basado en deteccion_matriz.txt) ---
# Rango de color HSV a detectar
HSV_BAJO = (40, 100, 100) # Límite inferior de Hue, Saturation, Value [cite: 1]
HSV_ALTO = (60, 200, 200) # Límite superior de Hue, Saturation, Value [cite: 1]

# --- Configuraciones de Cámara (Basado en deteccion_matriz.txt) ---
CAM_WIDTH = 640                     # [cite: 1]
CAM_HEIGHT = 480                    # [cite: 1]
CAM_FORMAT = 'XRGB8888'             # [cite: 1]
# --- Configuración específica para DETECCIÓN ---
CAM_SATURATION_DETECCION = 3        # Valor alto para resaltar color [cite: 1]
CAM_EXPOSURE_TIME_DETECCION = 50    # Valor fijo para consistencia [cite: 1]
CAM_SATURATION_BASE = 1.0           # Valor de saturación normal/base

# --- Configuración de la Cuadrícula ---
# Área de interés en la imagen (en píxeles)
ROI_X_INICIO = 100  # Píxel X de inicio de la región de interés
ROI_Y_INICIO = 100  # Píxel Y de inicio de la región de interés
ROI_X_FIN = 540     # Píxel X de fin de la región de interés
ROI_Y_FIN = 380     # Píxel Y de fin de la región de interés

# Dimensiones de las celdas
CELDA_CM = 7        # Tamaño de cada celda en cm
FILAS_CUADRICULA = 6  # Número de filas en la cuadrícula
COLUMNAS_CUADRICULA = 8  # Número de columnas en la cuadrícula

# --- Otros ---
# Puedes mantener estos o ajustarlos según necesites para la nueva posición
x = 0
y = 0

MORPH_DILATE_ITER = 2  # Número de iteraciones para dilatar la máscara
MORPH_CLOSE_ITER = 1   # Número de iteraciones para la operación de cierre morfológico
MIN_AREA_DETECCION = 100 # Umbral de área mínima para considerar una detección válida
POSICION_NUEVA_X = 0
POSICION_NUEVA_Y = 1

# --- Configuración para Formación ---
COLOR_PATRON_OBJETIVO = (255, 0, 255) # Magenta (BGR) para el patrón objetivo
COLOR_CELDA_ASIGNADA_PATRON = (0, 255, 255) # Amarillo (BGR) para la celda asignada en el patrón

# Colores para los patrones del vehículo (RGB)
COLOR_ROJO_VEHICULO = (255, 0, 0)
COLOR_VERDE_VEHICULO = (0, 255, 0)
COLOR_AZUL_VEHICULO = (0, 0, 255)


# PATRON_OBJETIVO: Matriz de 0s y 1s.
# Las dimensiones deben ser FILAS_CUADRICULA x COLUMNAS_CUADRICULA.

# El patrón se define visualmente (fila 0 del array es la fila superior de la figura).
# Se convertirá para que la fila 0 del grid (abajo) corresponda.
# Ejemplo: una 'L' simple para una cuadrícula 6x8 (6 filas, 8 columnas)
# PATRON_OBJETIVO = [
#     [1, 0, 0, 0, 0, 0, 0, 0],  # Fila superior visual
#     [1, 0, 0, 0, 0, 0, 0, 0],
#     [1, 0, 0, 0, 0, 0, 0, 0],
#     [1, 0, 0, 0, 0, 0, 0, 0],
#     [1, 1, 1, 1, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0]   # Fila inferior visual (corresponderá a fila 0 del grid)
# ]

PATRON_OBJETIVO = [
    [0,0,0,0,0,0,0,0], # fila 5 del grid
    [0,0,0,0,0,0,0,0], # fila 4 del grid
    [0,0,0,1,1,0,0,0], # 3,3 y 3,4
    [0,0,0,1,1,0,0,0], # 2,3 y 2,4
    [0,0,0,0,0,0,0,0], # fila 1 del grid
    [0,0,0,0,0,0,0,0]  # fila 0 del grid
]

posicion3_3 = [
    [1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0]
]

posicion3_4 = [
    [1,1,1,1,1,1,1,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1]
]

posicion2_3 = [
    [0,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,0,0,0,0,0,0,0],
    [1,1,1,1,1,1,1,1]
]

posicion2_4 = [
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1]
]

# Mapeo de celda objetivo (col_grid, fila_grid_logica_desde_abajo_0) a
# (nombre_del_patron_en_config, nombre_del_color_en_config)
# Estas celdas deben coincidir con las generadas por traducir_patron_a_celdas_objetivo
# a partir de PATRON_OBJETIVO.
CELDA_A_PATRON_COLOR = {
    (3, 3): ("posicion3_3", "COLOR_ROJO_VEHICULO"),   # Corresponde a PATRON_OBJETIVO[2][3]
    (4, 3): ("posicion3_4", "COLOR_VERDE_VEHICULO"), # Corresponde a PATRON_OBJETIVO[2][4]
    (3, 2): ("posicion2_3", "COLOR_AZUL_VEHICULO"),   # Corresponde a PATRON_OBJETIVO[3][3]
    (4, 2): ("posicion2_4", "COLOR_ROJO_VEHICULO"),  # Corresponde a PATRON_OBJETIVO[3][4] (ej. reusar color)
}
# Validar que el patrón tenga las dimensiones correctas (opcional pero recomendado)
if len(PATRON_OBJETIVO) != FILAS_CUADRICULA or \
   (FILAS_CUADRICULA > 0 and len(PATRON_OBJETIVO[0]) != COLUMNAS_CUADRICULA):
    raise ValueError(
        f"PATRON_OBJETIVO debe tener dimensiones {FILAS_CUADRICULA}x{COLUMNAS_CUADRICULA}."
    )
