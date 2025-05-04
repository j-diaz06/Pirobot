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

# Dimensiones físicas de la caja (en cm)
CAJA_ANCHO_CM = 56  # Ancho de la caja en cm (8 celdas de 7cm)
CAJA_ALTO_CM = 42   # Alto de la caja en cm (8 celdas de 7cm)

# Dimensiones de las celdas
CELDA_CM = 7        # Tamaño de cada celda en cm
FILAS_CUADRICULA = 6  # Número de filas en la cuadrícula
COLUMNAS_CUADRICULA = 8  # Número de columnas en la cuadrícula

# --- Otros ---
# Puedes mantener estos o ajustarlos según necesites para la nueva posición
POSICION_NUEVA_X = 0
POSICION_NUEVA_Y = 1

x = 0
y = 0

MORPH_DILATE_ITER = 2  # Número de iteraciones para dilatar la máscara
MORPH_CLOSE_ITER = 1   # Número de iteraciones para la operación de cierre morfológico
