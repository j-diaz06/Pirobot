# main.py
from machine import Pin, I2C
import time
import ujson # Necesario para parsear JSON
# Importar la librería del sensor si es necesaria globalmente (normalmente no)
# import vl53l0x

# --- Configuración del Vehículo ---
MI_ID = "vehiculo_1"  # <<< DEFINE EL ID UNICO DE ESTE VEHICULO AQUI

# --- Configuración de Hardware ---
# Pines Motores
ENA = 25
IN1 = 27
IN2 = 32
IN3 = 14 # OJO: En tu maquina.txt original usabas IN4 aquí, revisa tu cableado
IN4 = 33 # OJO: En tu maquina.txt original usabas IN3 aquí, revisa tu cableado
ENB = 26

# Pines I2C (Por defecto para ESP32 comunes)
SDA_PIN = 21
SCL_PIN = 22

# Pines Sensores Distancia (Asegúrate que coincidan con tu hardware)
# Los XSHUT permiten cambiar la dirección I2C si es necesario y evitar conflictos
XSHUT_SENSOR_DELANTERO = 5
DIRECCION_SENSOR_DELANTERO = 0x30 # Dirección I2C deseada para el sensor delantero
XSHUT_SENSOR_TRASERO = 4
DIRECCION_SENSOR_TRASERO = 0x35 # Dirección I2C deseada para el sensor trasero
DISTANCIA_MAXIMA_DETECCION = 150 # En mm, ajustar según necesidad (para obstáculos)
DISTANCIA_CENTRO_CELDA = 40 # En mm (80mm / 2), para la lógica de centrado

# Pines Matriz LED (DEFINIR SEGÚN TU HARDWARE)
# Ejemplo: PIN_MATRIZ_LED = 23

# --- Configuración de Operación ---
# Intervalo calibración Giroscopio (ms)
INTERVALO_CALIBRACION_YAW = 30000 # Calibrar cada 30 segundos
# Velocidades (ajustar según motores y voltaje)
# La clase Motor usa duty_u16 (0-65535), así que definimos en ese rango
VELOCIDAD_BASE = 30000 # Ejemplo: ~45% de velocidad
VELOCIDAD_GIRO = 20000 # Ejemplo: ~30% de velocidad
TOLERANCIA_ANGULO = 3.0 # Grados de tolerancia para giros

# --- Configuración de Red y MQTT ---
SSID = "Redmis"
PASSWORD = "asdfg123"
MQTT_BROKER = "192.168.202.157" # IP del Broker MQTT
MQTT_PORT = 1883 # Puerto estándar MQTT

# Topics MQTT (Según especificación v3)
TOPIC_POSICION_INICIAL = b"vehiculos/posicion_inicial" # b'' para bytes
TOPIC_COMANDO = b"vehiculos/comando"

# --- Importar Clases del Vehículo ---
# Asegúrate que estos archivos .py estén en el dispositivo
from maquina import Maquina
# Las clases de hardware (Motor, SensorVL53L0X, GiroAcel) se importan dentro de maquina.py
# La clase mqtt_pirobot también se importa dentro de maquina.py

# --- Inicialización ---
print(f"Inicializando vehículo con ID: {MI_ID}")

# Inicializar I2C una sola vez
print(f"Configurando I2C: SDA={SDA_PIN}, SCL={SCL_PIN}")
# Comprobar si ya existe una instancia I2C (útil en algunos entornos)
try:
    i2c = I2C(0)
    print("Usando instancia I2C(0) existente.")
except ValueError:
    print("Creando nueva instancia I2C(0).")
    i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)


# Escanear bus I2C para depuración (opcional)
try:
    print("Escaneando bus I2C...")
    devices = i2c.scan()
    if devices:
        print("Dispositivos I2C encontrados:", [hex(device) for device in devices])
    else:
        print("No se encontraron dispositivos I2C.")
except Exception as e:
    print(f"Error escaneando I2C: {e}")

# Crear instancia de la Máquina (Robot)
try:
    robot = Maquina(
        mi_id=MI_ID,
        # Pines Motores
        ena_pin=ENA, in1_pin=IN1, in2_pin=IN2, enb_pin=ENB, in3_pin=IN3, in4_pin=IN4,
        # Configuración Sensores Distancia (Pasar el bus I2C)
        i2c_bus=i2c,
        xshut_trasero=XSHUT_SENSOR_TRASERO, dir_trasero=DIRECCION_SENSOR_TRASERO,
        xshut_delantero=XSHUT_SENSOR_DELANTERO, dir_delantero=DIRECCION_SENSOR_DELANTERO,
        distancia_max=DISTANCIA_MAXIMA_DETECCION,
        distancia_centro=DISTANCIA_CENTRO_CELDA,
        # Configuración Giroscopio
        intervalo_calib=INTERVALO_CALIBRACION_YAW,
        # Configuración LED (Pasar pines/objeto si se implementa)
        # pin_matriz_led=PIN_MATRIZ_LED, # Ejemplo
        # Configuración Red/MQTT
        ssid=SSID, password=PASSWORD,
        mqtt_broker=MQTT_BROKER, mqtt_port=MQTT_PORT,
        topic_pos_inicial=TOPIC_POSICION_INICIAL,
        topic_comando=TOPIC_COMANDO,
        # Parámetros de movimiento
        velocidad_base=VELOCIDAD_BASE,
        velocidad_giro=VELOCIDAD_GIRO,
        tolerancia_angulo=TOLERANCIA_ANGULO
    )
except Exception as e:
    print("Error CRÍTICO creando la instancia de Maquina:")
    import sys
    sys.print_exception(e)
    # Detener ejecución si la máquina no se puede crear
    raise SystemExit("Fallo al inicializar Maquina")


# --- Bucle Principal ---
try:
    robot.iniciar() # Conecta a WiFi, MQTT, inicializa sensores y espera posición
    print("Robot inicializado. Esperando posición inicial...")
    while True:
        # El método loop ahora maneja la lógica principal:
        # - Comprueba mensajes MQTT
        # - Actualiza estado interno (sensores, yaw - vía thread)
        # - Progresa en acciones asíncronas (si estuvieran implementadas)
        robot.loop()
        # Pequeña pausa para ceder control si no se usa asyncio
        # En una implementación con asyncio, el bucle sería manejado por asyncio.run()
        # o el planificador de tareas tomaría control aquí.
        time.sleep_ms(20) # Un respiro pequeño, ajustar si afecta reactividad

except KeyboardInterrupt:
    print("Interrupción por teclado detectada.")
except Exception as e:
    print(f"Error inesperado en el bucle principal: {e}")
    # Podrías intentar guardar el estado o reiniciar aquí si es crítico
    import sys
    sys.print_exception(e)
finally:
    # Acciones de limpieza al salir
    print("Deteniendo sistema...")
    if 'robot' in locals() and robot: # Asegurarse que robot existe
        robot.detener_emergencia() # Detiene motores
        robot.apagar_led()         # Apaga LED (placeholder)
        robot.desconectar_mqtt()   # Desconecta de MQTT
    print("Sistema detenido.")
