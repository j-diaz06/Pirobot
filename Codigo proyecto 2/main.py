# main.py
from machine import Pin, I2C, UART
import time
import _thread

# --- Importar Clases ---
from maquina import Maquina

# --- Configuración de Pines y Constantes ---
# ... (igual que antes) ...
ENA = 25
IN1 = 27
IN2 = 32
IN4 = 33
IN3 = 14
ENB = 26
SDA_PIN = 21
SCL_PIN = 22
XSHUT_SENSOR_DELANTERO = 4
DIRECCION_SENSOR_DELANTERO = 0x30
XSHUT_SENSOR_TRASERO = 5
DIRECCION_SENSOR_TRASERO = 0x35

# --- Configuración UART para HC-05 ---
UART_ID = 2
UART_TX_PIN = 17
UART_RX_PIN = 16
UART_BAUDRATE = 115200

# --- Configuración Wi-Fi y MQTT ---
# ... (igual que antes) ...
SSID = "Redmis"
PASSWORD = "asdfg123"
MQTT_BROKER = "192.168.22.157"
CLIENT_ID_BASE = "esp32_vehicle"
MI_ID = "vehiculo_1"

# --- Inicializar UART para Debug ---
debug_uart = UART(UART_ID, baudrate=UART_BAUDRATE, tx=UART_TX_PIN, rx=UART_RX_PIN)
# Usamos print aquí porque la conexión serial de debug aún no está confirmada
print(f"UART para Debug inicializado en pines TX={UART_TX_PIN}, RX={UART_RX_PIN}")

# --- Función de Handshake Serial MODIFICADA ---
def esperar_conexion_debug(uart, timeout_ms=1000000):
    """
    Espera recibir 'OK' (sin necesidad de \\n) por UART para confirmar conexión.
    Imprime lo recibido en la consola USB para depuración.
    """
    print("Esperando conexión de depuración serial (envía 'ok')...")
    uart.write("ESP32_READY-------------------------------------------------------------------\n") # Avisa que el ESP32 está listo
    start_time = time.ticks_ms()
    buffer = ""
    while time.ticks_diff(time.ticks_ms(), start_time) < timeout_ms:
        if uart.any():
            try:
                data_in_bytes = uart.read() # Leer los bytes disponibles
                # --- DEBUG: Imprimir bytes crudos recibidos en consola USB ---
                print(f"Bytes recibidos: {data_in_bytes}")
                # --- FIN DEBUG ---

                if data_in_bytes: # Solo si se leyó algo
                    decoded_data = data_in_bytes.decode('utf-8')
                    # --- DEBUG: Imprimir string decodificado en consola USB ---
                    print(f"Decodificado como: '{decoded_data}'")
                    # --- FIN DEBUG ---
                    buffer += decoded_data

                    # --- Lógica de comprobación MODIFICADA ---
                    # Comprobar si el buffer acumulado (limpio y en mayúsculas) es "OK"
                    # Usamos strip() para eliminar espacios/saltos de línea accidentales al inicio/final
                    if buffer.strip() == "ok":
                        print("¡Conexión de depuración establecida! (Recibido 'ok')") # Mensaje en consola USB
                        uart.write("DEBUG_CONNECTED\n") # Confirmación por HC-05
                        return True
                    else:
                        # Opcional: Limpiar el buffer si no es "OK" para evitar concatenaciones no deseadas
                        # Si esperas recibir "OK" de golpe, puedes limpiar aquí:
                        # print(f"Buffer actual: '{buffer}', no es 'OK'. Limpiando buffer.")
                        # buffer = ""
                        # O si "OK" puede llegar fragmentado, simplemente sigue acumulando
                        print(f"Buffer actual: '{buffer}', esperando 'ok'...") # Mensaje en consola USB
                        buffer = ""
                        # No reenvíes ESP32_READY constantemente para no llenar el buffer del otro lado
                        # uart.write("ESP32_READY\n")

            except UnicodeError as e:
                print(f"Error de decodificación Unicode: {e}, bytes: {data_in_bytes}") # Mensaje en consola USB
                # Limpiar buffer si hay error de decodificación
                buffer = ""
            except Exception as e:
                print(f"Error leyendo UART: {e}") # Mensaje en consola USB
                buffer = "" # Limpiar en caso de otro error

        time.sleep_ms(100) # Pequeña pausa

    print("Timeout esperando conexión de depuración.") # Mensaje en consola USB
    return False

# --- Ejecutar Handshake ---
if not esperar_conexion_debug(debug_uart):
     print("ERROR FATAL: No se pudo establecer conexión de depuración serial.")
     # Detener ejecución
     while True:
         time.sleep(1)

# --- Pasar el objeto UART a la máquina ---
debug_uart.write(f"--- Iniciando Vehículo Autónomo [{MI_ID}] ---") # Consola USB

maquina = Maquina(
    ENA, IN1, IN2, ENB, IN3, IN4,
    DIRECCION_SENSOR_TRASERO, XSHUT_SENSOR_TRASERO,
    DIRECCION_SENSOR_DELANTERO, XSHUT_SENSOR_DELANTERO,
    SSID, PASSWORD, MQTT_BROKER, CLIENT_ID_BASE,
    MI_ID,
    debug_uart # Pasar el objeto UART
)

# Iniciar la lógica principal del robot
maquina.iniciar()

# --- Bucle Principal ---
debug_uart.write("--- Bucle Principal Iniciado (Depuración por Serial) ---") # Consola USB
maquina.debug_serial("--- Bucle Principal Iniciado (Depuración por Serial HC05) ---") # HC-05

while True:
    try:
        maquina.loop()
    except KeyboardInterrupt:
        debug_uart.write("Programa detenido por el usuario.") # Consola USB
        maquina.detener()
        break
    except Exception as e:
        error_msg = f"Error en el bucle principal: {e}"
        debug_uart.write(error_msg) # Consola USB
        maquina.debug_serial(error_msg) # HC-05
        maquina.detener()
        time.sleep(5)