import paho.mqtt.client as mqtt
import json
import time
import threading
from datetime import datetime

from rich.console import Console
from rich.table import Table
from rich.live import Live
from rich.text import Text

# Declarar live globalmente para que sea accesible desde on_message y main
live = None

# Intenta importar la configuración del broker.
# Asegúrate de que config.py esté en la misma carpeta o en PYTHONPATH.
try:
    import config
except ImportError:
    print("Error: No se pudo importar 'config.py'. Asegúrate de que exista y sea accesible.")
    print("Usando valores predeterminados para el broker MQTT (localhost:1883).")
    class MockConfig:
        MQTT_BROKER_IP = "localhost"
        MQTT_PORT = 1883
    config = MockConfig()


# --- Configuración del Monitor ---
VEHICLE_ID_TO_MONITOR = "vehiculo_1"  # Cambia esto al ID del vehículo que quieres monitorear
MQTT_BROKER = config.MQTT_BROKER_IP
MQTT_PORT = config.MQTT_PORT
# ID de cliente único para este script de monitoreo
CLIENT_ID_MONITOR = f"raspberry_monitor_{VEHICLE_ID_TO_MONITOR}_{int(time.time())}"
STATUS_TOPIC = f"vehiculos/estado_actual/{VEHICLE_ID_TO_MONITOR}"
LOG_FILE = f"log_estado_{VEHICLE_ID_TO_MONITOR}.txt"

console = Console()

# Datos compartidos entre el hilo MQTT y el hilo principal (para Rich Live)
latest_data_lock = threading.Lock()
latest_data_payload = {
    "connection_status": Text("Inicializando...", style="yellow"),
    "subscribed_topic": "N/A",
    "id_vehiculo_monitoreado": VEHICLE_ID_TO_MONITOR,
    "id_vehiculo_reportado": "N/A",
    "angulo": "N/A",
    "distancia_sensor": "N/A",
    "estado_movimiento": "N/A",
    "timestamp_vehiculo": "N/A",
    "timestamp_mensaje_vehiculo_ms": "N/A", # Para el valor crudo de timestamp_ms
    "ultima_actualizacion_monitor": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
}

def on_connect(client, userdata, flags, rc):
    """Callback cuando se conecta al broker MQTT."""
    global latest_data_payload
    with latest_data_lock:
        if rc == 0:
            latest_data_payload["connection_status"] = Text(f"Conectado a {MQTT_BROKER}:{MQTT_PORT}", style="green")
            try:
                client.subscribe(STATUS_TOPIC)
                latest_data_payload["subscribed_topic"] = Text(STATUS_TOPIC, style="cyan")
                latest_data_payload["estado_movimiento"] = "Esperando datos..."
            except Exception as e:
                latest_data_payload["subscribed_topic"] = Text(f"Error al suscribir: {e}", style="red")
        else:
            latest_data_payload["connection_status"] = Text(f"Fallo al conectar, código={rc}", style="red")
            latest_data_payload["subscribed_topic"] = "N/A"

def on_disconnect(client, userdata, rc):
    """Callback cuando se desconecta del broker MQTT."""
    global latest_data_payload
    with latest_data_lock:
        latest_data_payload["connection_status"] = Text(f"Desconectado (código={rc}). Reintentando...", style="bold red")
        latest_data_payload["subscribed_topic"] = "N/A"
        # Aquí podrías añadir lógica para reintentar la conexión si no es un cierre intencional

def on_message(client, userdata, msg):
    """Callback cuando se recibe un mensaje MQTT."""
    global latest_data_payload, live
    try:
        payload_str = msg.payload.decode('utf-8')
        data = json.loads(payload_str)

        # Guardar en archivo de log
        log_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write(f"{log_timestamp} | {payload_str}\n")

        # Actualizar datos para la tabla de Rich
        with latest_data_lock:
            latest_data_payload["id_vehiculo_reportado"] = data.get("id_vehiculo", "N/A")
            latest_data_payload["angulo"] = data.get("angulo", "N/A")
            latest_data_payload["distancia_sensor"] = data.get("distancia_sensor", "N/A") # Puede ser -1
            latest_data_payload["estado_movimiento"] = data.get("estado_movimiento", "N/A")
            latest_data_payload["timestamp_mensaje_vehiculo_ms"] = data.get("timestamp_ms", "N/A")
            latest_data_payload["ultima_actualizacion_monitor"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            latest_data_payload["timestamp_vehiculo"] = f"{data.get("timestamp_ms")} ms"

        # Solo actualizar si live ha sido inicializada
        if live:
            live.update(generate_status_table())


    except json.JSONDecodeError:
        print(f"[Error] No se pudo decodificar JSON: {msg.payload.decode('utf-8', errors='replace')}")
    except Exception as e:
        print(f"[Error] Procesando mensaje: {e}")

def generate_status_table() -> Table:
    """Genera la tabla de Rich con los datos más recientes."""
    with latest_data_lock:
        # Usar el ID reportado por el vehículo para el título si está disponible
        display_id = latest_data_payload["id_vehiculo_reportado"]
        if display_id == "N/A" or not display_id:
            display_id = latest_data_payload["id_vehiculo_monitoreado"]

        table = Table(
            title=f"Estado del Vehículo: [bold cyan]{display_id}[/] ([italic]{datetime.now().strftime('%H:%M:%S')}[/])",
            show_header=True,
            header_style="bold magenta",
            border_style="blue",
            caption=f"Log en: {LOG_FILE}"
        )
        table.add_column("Parámetro", style="dim", width=30, overflow="fold")
        table.add_column("Valor", overflow="fold")

        # Estado de la Conexión MQTT
        table.add_row("Estado Conexión MQTT", latest_data_payload["connection_status"])
        table.add_row("Cliente ID (Monitor)", Text(CLIENT_ID_MONITOR, style="yellow"))
        table.add_row("Suscrito a Tópico", latest_data_payload["subscribed_topic"])
        table.add_section()

        # Datos del Vehículo
        angulo = latest_data_payload['angulo']
        distancia = latest_data_payload['distancia_sensor']
        estado_mov = latest_data_payload['estado_movimiento']

        table.add_row("ID Vehículo (Reportado)", str(latest_data_payload['id_vehiculo_reportado']))
        table.add_row("Ángulo Actual (°)", f"{angulo:.2f}" if isinstance(angulo, float) else str(angulo))
        table.add_row("Última Distancia (mm)", str(distancia) if distancia != -1 else Text("N/A o inválida", style="yellow"))
        table.add_row("Estado Movimiento", Text(str(estado_mov).capitalize(), style="bold green" if estado_mov == "moviendo" else "bold red"))
        table.add_row("Timestamp Vehículo", str(latest_data_payload['timestamp_vehiculo']))
        table.add_row("Última Actualización (Monitor)", str(latest_data_payload['ultima_actualizacion_monitor']))

    return table

def main():
    """Función principal para conectar y mostrar datos."""
    global live # Indicar que vamos a asignar a la variable global live
    client = mqtt.Client(client_id=CLIENT_ID_MONITOR)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message

    console.print(f"Iniciando monitor para [bold cyan]{VEHICLE_ID_TO_MONITOR}[/]...")
    console.print(f"Intentando conectar al broker MQTT: [bold yellow]{MQTT_BROKER}:{MQTT_PORT}[/]")

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    except ConnectionRefusedError:
        console.print(f"[bold red]Error Crítico: Conexión rechazada por el broker.[/]")
        console.print("Verifica que el broker MQTT esté corriendo y accesible.")
        return
    except OSError as e: # Errores como "No route to host"
        console.print(f"[bold red]Error Crítico de Red/OS: {e}[/]")
        console.print("Verifica la configuración de red y la IP del broker.")
        return
    except Exception as e:
        console.print(f"[bold red]Error Crítico al conectar: {e}[/]")
        return

    # `Live` se encarga de refrescar la tabla llamando a `generate_status_table`
    with Live(generate_status_table(), console=console, refresh_per_second=2, vertical_overflow="visible", screen=False) as live_instance:
        client.loop_start()  # Inicia el bucle de red MQTT en un hilo separado
        live = live_instance # Asignar la instancia de Live a la variable global
        try:
            while True:
                # El hilo principal se mantiene vivo aquí.
                # `Live` y el hilo MQTT hacen su trabajo.
                time.sleep(1)
        except KeyboardInterrupt:
            console.print("\n[yellow]Cerrando aplicación por el usuario...[/yellow]")
        finally:
            if live: # Usar la variable global live
                live.stop() # Detiene la actualización de Live antes de imprimir más
            console.print("[yellow]Deteniendo cliente MQTT...[/yellow]")
            client.loop_stop()
            client.disconnect()
            live = None # Limpiar la referencia global, buena práctica

    console.print("[green]Aplicación cerrada limpiamente.[/green]")
    console.print(f"El log de estados se ha guardado en: [bold cyan]{LOG_FILE}[/]")

if __name__ == "__main__":
    main()
