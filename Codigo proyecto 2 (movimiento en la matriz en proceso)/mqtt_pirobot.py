# mqtt_pirobot.py
from umqtt.robust import MQTTClient
import network
import time
import ujson # Necesario para decodificar JSON

class mqtt_pirobot:
    def __init__(self, wifi_ssid, wifi_password, mqtt_broker, mqtt_port, client_id):
        """
        Inicializa el cliente MQTT y la conexión WiFi.

        Args:
            wifi_ssid (str): Nombre de la red WiFi.
            wifi_password (str): Contraseña de la red WiFi.
            mqtt_broker (str): Dirección IP o nombre del broker MQTT.
            mqtt_port (int): Puerto del broker MQTT.
            client_id (str): ID único para este cliente MQTT.
        """
        self.ssid = wifi_ssid
        self.password = wifi_password
        self.broker = mqtt_broker
        self.port = mqtt_port
        self.client_id = client_id
        self.client = None
        self._callback_handler = None # Referencia al método de la clase Maquina
        self.connected = False
        self._subscribed_topics = set() # Para llevar registro de suscripciones deseadas

    def connect_wifi(self):
        """Intenta conectar a la red WiFi. Retorna True si está conectado."""
        sta_if = network.WLAN(network.STA_IF)
        if not sta_if.isconnected():
            print(f"Conectando a Wi-Fi ({self.ssid})...")
            sta_if.active(True)
            # Limpiar conexiones anteriores si existen
            try:
                if sta_if.isconnected(): # Solo desconectar si estaba conectado a algo
                    sta_if.disconnect()
                    time.sleep_ms(500)
            except OSError as e:
                print(f"Info: Error al desconectar WiFi previo (puede ser normal): {e}")
                pass # Puede dar error si no estaba conectado, continuar

            sta_if.connect(self.ssid, self.password)
            connect_timeout = time.ticks_add(time.ticks_ms(), 20000) # Timeout 20s
            while not sta_if.isconnected():
                if time.ticks_diff(connect_timeout, time.ticks_ms()) < 0:
                    print("\nError: Timeout conectando a WiFi.")
                    sta_if.active(False) # Desactivar si falla
                    return False
                print(".", end="")
                time.sleep_ms(500)
            print("\nConectado a Wi-Fi:", sta_if.ifconfig())
        else:
             # print("Ya conectado a Wi-Fi:", sta_if.ifconfig()) # Comentado para reducir spam
             pass
        return True

    def connect_mqtt(self):
        """
        Intenta conectar (o reconectar) al broker MQTT.
        Retorna True si la conexión es exitosa.
        """
        if not self.client or not self.connected: # Intentar solo si no hay cliente o no está conectado
            try:
                # Asegurarse que el cliente anterior esté limpio si existía
                if self.client:
                    try:
                        self.client.disconnect()
                    except Exception:
                        pass # Ignorar errores al desconectar el viejo cliente
                self.client = None
                self.connected = False

                print(f"Conectando a MQTT Broker ({self.broker}:{self.port}) con Client ID: {self.client_id}")
                self.client = MQTTClient(self.client_id, self.broker, port=self.port, keepalive=60) # Keepalive 60s
                # Asignar callback ANTES de conectar si es posible
                if self._callback_handler:
                    self.client.set_callback(self._internal_callback)
                self.client.connect()
                print("Conectado a MQTT Broker.")
                self.connected = True
                # Re-suscribirse a topics deseados después de conectar/reconectar
                print(f"Resuscribiendo a tópicos: {self._subscribed_topics}")
                for topic in list(self._subscribed_topics): # Iterar sobre copia
                    self.subscribe(topic, resubscribing=True) # Llama a self.client.subscribe internamente
                return True
            except OSError as e:
                print(f"Error al conectar al broker MQTT: {e}")
                self.client = None
                self.connected = False
                # Podríamos añadir un backoff aquí antes de reintentar en el siguiente ciclo
                time.sleep_ms(5000) # Esperar 5s antes del próximo reintento automático
                return False
            except Exception as e: # Captura otras posibles excepciones
                print(f"Error inesperado al conectar a MQTT: {e}")
                self.client = None
                self.connected = False
                time.sleep_ms(5000)
                return False
        return self.connected # Ya estaba conectado

    def set_callback_handler(self, handler_method):
        """Asigna el método de la clase Maquina que procesará los mensajes."""
        self._callback_handler = handler_method
        # Si el cliente ya existe, reasignar el callback
        if self.client:
            self.client.set_callback(self._internal_callback)

    def _internal_callback(self, topic, msg):
        """
        Callback interno que recibe de umqtt, decodifica, parsea JSON
        y llama al handler de la clase Maquina.
        """
        if self._callback_handler:
            try:
                # Decodificar topic y mensaje (asumiendo UTF-8)
                decoded_topic = topic.decode('utf-8')
                decoded_msg = msg.decode('utf-8')
                # print(f"MQTT Raw Msg | Topic: {decoded_topic} | Msg: {decoded_msg}") # Debug Raw

                # Intentar parsear como JSON
                try:
                    payload = ujson.loads(decoded_msg)
                except ValueError:
                    # Si no es JSON válido, pasar el mensaje decodificado tal cual
                    # print(f"Advertencia: Mensaje en {decoded_topic} no es JSON válido: {decoded_msg}. Se pasará como string.")
                    payload = decoded_msg # Pasar como string si no es JSON

                # Llamar al handler de Maquina pasando topic y payload (dict o string)
                self._callback_handler(decoded_topic, payload)

            except Exception as e:
                print(f"Error CRÍTICO procesando mensaje MQTT: {e}")
                import sys
                sys.print_exception(e)
        else:
            print("Advertencia: Mensaje MQTT recibido pero no hay handler asignado.")

    def subscribe(self, topic, resubscribing=False):
        """
        Añade un tópico al conjunto de suscripciones deseadas
        e intenta suscribirse si está conectado.
        """
        is_new_subscription = topic not in self._subscribed_topics
        self._subscribed_topics.add(topic) # Añadir al registro de deseadas

        if self.client and self.connected:
            try:
                self.client.subscribe(topic)
                if not resubscribing and is_new_subscription:
                    print(f"Suscrito a tópico: {topic.decode('utf-8')}")
                return True
            except Exception as e:
                print(f"Error al suscribir a {topic.decode('utf-8')}: {e}")
                # Si falla, permanecerá en _subscribed_topics para reintentar en reconexión
                return False
        else:
            # Si no está conectado, ya se guardó en _subscribed_topics
            if is_new_subscription:
                 print(f"MQTT no conectado. Tópico {topic.decode('utf-8')} añadido a suscripciones pendientes.")
            return False # Indicar que no se suscribió ahora

    def unsubscribe(self, topic):
        """
        Elimina un tópico de las suscripciones deseadas
        e intenta desuscribirse si está conectado.
        """
        self._subscribed_topics.discard(topic) # Quitar de las deseadas

        if self.client and self.connected:
            try:
                # umqtt.robust puede no tener un método unsubscribe explícito.
                # Intentarlo por si acaso. Si falla, la lógica es no resuscribirse.
                self.client.unsubscribe(topic)
                print(f"Desuscripción intentada para: {topic.decode('utf-8')}")
                return True
            except AttributeError:
                 print(f"Nota: umqtt.robust no soporta unsubscribe explícito. Se dejará de procesar {topic.decode('utf-8')}.")
                 return True # Considerar éxito ya que se eliminó del registro
            except Exception as e:
                print(f"Error al intentar desuscribir de {topic.decode('utf-8')}: {e}")
                return False
        else:
            # print(f"Tópico {topic.decode('utf-8')} eliminado de suscripciones pendientes (MQTT no conectado).")
            return True


    def check_msg(self):
        """Revisa si hay mensajes pendientes. Llamar periódicamente."""
        if self.client and self.connected:
            try:
                self.client.check_msg() # Puede levantar OSError si la conexión se pierde
            except OSError as e:
                print(f"Error en conexión MQTT durante check_msg: {e}. Marcando como desconectado.")
                # Marcar como desconectado para forzar reconexión en el siguiente loop
                self.connected = False
                # No limpiar self.client aquí, connect_mqtt lo manejará
            except Exception as e:
                 print(f"Error inesperado durante check_msg: {e}")
                 # Considerar marcar como desconectado también?
                 # self.connected = False


    def disconnect(self):
        """Desconecta limpiamente del broker MQTT."""
        if self.client:
            try:
                print("Desconectando de MQTT...")
                self.client.disconnect()
                print("Desconectado.")
            except Exception as e:
                 print(f"Error al desconectar de MQTT: {e}")
            finally:
                self.client = None
                self.connected = False
                # Mantener _subscribed_topics para posible reconexión
