from umqtt.robust import MQTTClient
import network
import time

class mqtt_pirobot:
    def __init__(self, wifi_ssid, wifi_password, mqtt_broker, client_id):
        """
        Inicializa el suscriptor MQTT
        
        Args:
            wifi_ssid (str): Nombre de la red WiFi
            wifi_password (str): Contraseña de la red WiFi
            mqtt_broker (str): Dirección IP del broker MQTT
            client_id (str): ID único para el cliente MQTT
        """
        self.ssid = wifi_ssid
        self.password = wifi_password
        self.broker = mqtt_broker
        self.client_id = client_id
        self.client = None
        self.callback = None
        self.subscribed_topic = None

    def connect_wifi(self):
        """Conecta a la red WiFi"""
        sta_if = network.WLAN(network.STA_IF)
        if not sta_if.isconnected():
            print("Conectando a Wi-Fi...")
            sta_if.active(True)
            sta_if.connect(self.ssid, self.password)
            while not sta_if.isconnected():
                time.sleep(1)
        print("Conectado a Wi-Fi")
        return sta_if.isconnected()

    def connect_mqtt(self):
        """Conecta al broker MQTT"""
        try:
            self.client = MQTTClient(self.client_id, self.broker)
            self.client.connect()
            print("Conectado al broker MQTT")
            return True
        except OSError as e:
            print(f"Error al conectar al broker MQTT: {e}")
            return False

    def set_callback(self, topic, callback_instance):
        """
        Configura el callback usando una instancia de clase y su método
        
        Args:
            topic: Tópico MQTT a suscribirse
            callback_instance: Instancia de la clase que contiene el método mqtt_recibido
        """
        self.subscribed_topic = topic
        self.callback_instance = callback_instance
        
        # Asigna el método interno que hará de puente
        self.client.set_callback(self._internal_callback)
        self.client.subscribe(topic)
        print(f"Suscripto a tópico: {topic}")

    def _internal_callback(self, topic, msg):
        """Manejador interno que redirige al callback de la otra clase"""
        try:
            decoded_topic = topic.decode('utf-8')
            decoded_msg = msg.decode('utf-8')
            if hasattr(self.callback_instance, 'mqtt_recibido'):
                # Llama al método mqtt_recibido de la instancia de Maquina
                self.callback_instance.mqtt_recibido(decoded_topic, decoded_msg)
        except Exception as e:
            print(f"Error procesando mensaje: {e}")

    def wait_for_messages(self):
        """Espera activamente por mensajes MQTT
        if not self.client:
            raise RuntimeError("MQTT no conectado")
        
        print("Esperando mensajes...")
        while True:"""
        try:
            self.client.check_msg()
            #time.sleep(0.1)  # Pequeña pausa para evitar saturación
        except OSError as e:
            print(f"Error en la conexión MQTT: {e}")
            # Podrías añadir reconexión automática aquí

    def disconnect(self):
        """Desconecta del broker MQTT"""
        if self.client:
            if self.subscribed_topic:
                self.client.unsubscribe(self.subscribed_topic)
            self.client.disconnect()
            print("Desconectado del broker MQTT")