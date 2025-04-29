import paho.mqtt.client as mqtt
import json
import time
import sys
import config # Importa la configuración actualizada

client = None # Variable global para el cliente MQTT

def on_connect(client, userdata, flags, rc):
    """Callback de conexión."""
    if rc == 0:
        print(f"MQTT Conectado exitosamente a {config.MQTT_BROKER_IP}:{config.MQTT_PORT}") #
    else:
        print(f"MQTT Fallo al conectar, código={rc}")

def on_disconnect(client, userdata, rc):
    """Callback de desconexión."""
    print(f"MQTT Desconectado con código={rc}") #

def conectar_mqtt():
    """Crea, configura e intenta conectar el cliente MQTT."""
    global client
    # Usa el CLIENT_ID_PUB de config.py que ahora es "raspberry_pi_tester"
    client = mqtt.Client(client_id=config.CLIENT_ID_PUB) # [cite: 5]
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect

    try:
        print(f"Intentando conectar a {config.MQTT_BROKER_IP}...") #
        client.connect(config.MQTT_BROKER_IP, config.MQTT_PORT, keepalive=60) #
        client.loop_start()
        time.sleep(2)
        if not client.is_connected():
            print("No se pudo establecer la conexión MQTT.")
            return None
        return client
    except Exception as e:
        print(f"Error al conectar al broker MQTT: {e}") #
        return None

def desconectar_mqtt():
    """Detiene el bucle y desconecta el cliente MQTT."""
    global client
    if client:
        client.loop_stop()
        client.disconnect()
        print("MQTT Desconectado.")

def enviar_comando_matriz(vehicle_id, figura):
    """Envia un comando genérico para la matriz LED."""
    global client
    if not client or not client.is_connected():
        print("Error: Cliente MQTT no conectado.")
        return

    topic = config.TOPIC_COMANDO # Usa el topic de config.py [cite: 5]
    payload_dict = {
        "id_vehiculo": vehicle_id,
        "accion": "mostrar",
        "figura": figura
    }
    payload_json = json.dumps(payload_dict)
    print(f"Enviando comando matriz a [{topic}]: {payload_json}") #
    client.publish(topic, payload_json, qos=0)

def enviar_posicion_inicial(vehicle_id, x, y):
    """Envia la posición inicial detectada al vehículo."""
    global client
    if not client or not client.is_connected():
        print("Error: Cliente MQTT no conectado.")
        return

    topic = config.TOPIC_POSICION_INICIAL # Usa el topic de config.py [cite: 5]
    payload_dict = {
        "id_vehiculo": vehicle_id,
        "ubicacion": f"{int(x)},{int(y)}" #
    }
    payload_json = json.dumps(payload_dict) #
    print(f"Enviando posición inicial a [{topic}]: {payload_json}") #
    result = client.publish(topic, payload_json, qos=1) # QoS 1 como en el ejemplo original [cite: 7]

def enviar_comando_movimiento(vehicle_id, x_destino, y_destino):
    """Envia un comando de movimiento al vehículo."""
    global client
    if not client or not client.is_connected():
        print("Error: Cliente MQTT no conectado.")
        return

    topic = config.TOPIC_COMANDO # Usa el topic de config.py [cite: 5]
    payload_dict = {
        "id_vehiculo": vehicle_id,
        "destino": f"{int(x_destino)},{int(y_destino)}" #
    }
    payload_json = json.dumps(payload_dict) #
    print(f"Enviando comando movimiento a [{topic}]: {payload_json}") #
    client.publish(topic, payload_json, qos=0) # QoS 0 como en el ejemplo original [cite: 8]