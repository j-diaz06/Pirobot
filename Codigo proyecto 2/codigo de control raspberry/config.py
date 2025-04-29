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
CAM_SATURATION = 3                  # [cite: 1]
CAM_EXPOSURE_TIME = 50              # [cite: 1]

# --- Otros ---
# Puedes mantener estos o ajustarlos según necesites para la nueva posición
POSICION_NUEVA_X = 0
POSICION_NUEVA_Y = 1

x = 0
y = 0
