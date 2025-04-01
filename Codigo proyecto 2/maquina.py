# maquina.py
import time
from machine import Pin, PWM, I2C
import _thread
# Importar las clases desde los archivos proporcionados
from giro_acel import GiroAcel
from motor import Motor
from sensor import SensorVL53L0X
from mqtt_pirobot import mqtt_pirobot

# Estados del sistema
ESTADO_ADELANTE = 0
ESTADO_DERECHA = 1
ESTADO_IZQUIERDA = 2
ESTADO_DETENER = 3
#ESTADO_ESPERA_ADELANTE = 4
#ESTADO_ESPERA_DERECHA = 5
ESTADO_ESPERAR_CAMBIO = 4

class Maquina:
    def __init__(self, ENA, IN1, IN2, ENB, IN3, IN4,
                 direccion_trasero, xshut_trasero,
                 direccion_delantero, xshut_delantero, DISTANCIA_MAXIMA,
                 direccion_inicial, IntervaloCalibracion, TIEMPO_ESPERA,
                 SSID,PASSWORD,MQTT_BROKER,CLIENT_ID):

        self.motores = Motor(ENA, IN1, IN2, ENB, IN3, IN4)
        self.sensor_trasero = SensorVL53L0X(direccion_trasero, xshut_trasero)
        self.sensor_delantero = SensorVL53L0X(direccion_delantero, xshut_delantero)
        self.aceleracion_adelante = direccion_inicial
        self.i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000) # Pines por defecto, ajustar si es necesario
        self.giro_acel = GiroAcel(self.i2c, IntervaloCalibracion)  # Inicializar GiroAcel
        self.mqtt = mqtt_pirobot(SSID,PASSWORD,MQTT_BROKER,CLIENT_ID)
        

        self.estado_actual = ESTADO_ESPERAR_CAMBIO
        self.DISTANCIA_DETECCION = DISTANCIA_MAXIMA
        #self.intervaloCalibracion = IntervaloCalibracion
        self.tiempo_espera = TIEMPO_ESPERA
        self.girando = False
        self.yawObjetivo = 0.0
        self.conteo_espera = 0
        #self.lastCalCheck = 0
        #self.bypass = True
        self.obstaculo_enfrente = False
        self.obstaculo_atras = False
        self.distancia_trasero = 0
        self.distancia_delantero = 0
        self.tiempo_vl53 = 0
        self.espera_2nucleo = 0
        
        self.debug=0

        # Iniciar el hilo para actualizar el yaw
        _thread.start_new_thread(self.yaw_task, ())

    def mover(self):
        if self.aceleracion_adelante:
            self.motores.avanzar(self.motores.velocidad)
        else:
            self.motores.retroceder(self.motores.velocidad)

    def detener(self):
        self.motores.detener()

    def obstaculo_detectado(self):
        self.motores.detener()
        self.conteo_espera = time.ticks_ms()
        self.estado_actual = ESTADO_DERECHA
        self.yawObjetivo = self.giro_acel.yaw - 90.0  # Ajustar según la orientación del giroscopio
        self.girando = True

    def leer_sensores(self):
        self.distancia_trasero = self.sensor_trasero.medir()
        #print("[sensor] trasero=", self.distancia_trasero, end=" ")
        time.sleep_ms(5)

        self.distancia_delantero = self.sensor_delantero.medir()
        #print("[sensor] delantero=", self.distancia_delantero)

        """if self.bypass:
            if self.distancia_delantero == 8 or self.distancia_trasero == 8:
                self.distancia_trasero = self.sensor_trasero.medir()
                print("[sensor] trasero=", self.distancia_trasero, end=" ")
                time.sleep_ms(5)
                self.distancia_delantero = self.sensor_delantero.medir()
                print("[sensor] delantero=", self.distancia_delantero)
                self.bypass = False"""

        if not self.girando and self.aceleracion_adelante and (self.distancia_delantero <= self.DISTANCIA_DETECCION and self.distancia_delantero != -1):
            self.obstaculo_detectado()
            print("[enfrente] Detectado obstaculo")
            self.obstaculo_enfrente = True
        elif not self.girando and not self.aceleracion_adelante and (self.distancia_trasero <= self.DISTANCIA_DETECCION and self.distancia_trasero != -1):
            self.obstaculo_detectado()
            print("[atras] Detectado obstaculo")
            self.obstaculo_atras = True
        else:
            self.obstaculo_enfrente = False
            self.obstaculo_atras = False

    def ejecutar_accion(self):
        #print("H")
        if self.estado_actual == ESTADO_ADELANTE:
            print("B")
            if time.ticks_ms() - self.conteo_espera >= self.tiempo_espera:
                self.estado_actual = ESTADO_ESPERAR_CAMBIO
                print("C")
                self.giro_acel._calibrar()
                print("D")
                self.mover()
        elif self.estado_actual == ESTADO_DERECHA:
            print("E")
            if time.ticks_ms() - self.conteo_espera >= self.tiempo_espera:
                self.estado_actual = ESTADO_ESPERAR_CAMBIO
                print("F")
                self.giro_acel._calibrar()
                print("G")
                self.motores.girar_derecha(self.motores.velocidad)
        elif self.estado_actual == ESTADO_DETENER:
            self.estado_actual = ESTADO_ESPERAR_CAMBIO
            print("H")
            self.motores.detener()

    def loop(self):
        if time.ticks_ms() - self.tiempo_vl53 > 10:
            self.leer_sensores()
            self.tiempo_vl53 = time.ticks_ms()

        yawActual = self.giro_acel.yaw  # Lectura directa del valor actualizado
        
        if self.girando and (abs(yawActual - self.yawObjetivo) < 5.0):
            self.motores.detener()
            print("A")
            self.conteo_espera = time.ticks_ms()
            self.girando = False
            self.estado_actual = ESTADO_ADELANTE
            
        if time.ticks_ms() - self.debug > 100:
            print("Yaw=", yawActual)
            self.debug = time.ticks_ms()
            
        self.mqtt.wait_for_messages()
            
        self.ejecutar_accion()

        """print("Estado: ", end="")
        if self.estado_actual == ESTADO_ADELANTE:
            print("Adelante")
        elif self.estado_actual == ESTADO_DERECHA:
            print("Derecha")
        print()"""

    def iniciar(self):

        print("[Maquina] Inicializada.")
        
        if self.mqtt.connect_wifi():
            print("[Wifi] conectado.")
            if self.mqtt.connect_mqtt():
                # Suscribirse a un tópico y configurar la función de callback
                self.mqtt.set_callback("esp32/movimiento", self)

        self.giro_acel.iniciar_mpu()  # Inicializar y calibrar MPU

        #self.mover()
        
        self.tiempo_vl53 = time.ticks_ms()

    def acelerarAdelante(self, nuevaDireccion):
        self.aceleracion_adelante = nuevaDireccion

    def yaw_task(self):
        while True:
            if time.ticks_ms() - self.espera_2nucleo > 5:
                self.giro_acel.actualizar_yaw()
                self.espera_2nucleo = time.ticks_ms()
                #time.sleep_ms(5)
            
    def mqtt_recibido(self, topic, mensaje):
        print(f"Mensaje recibido en {topic}: {mensaje}")
        # Aquí pones la acción que quieres ejecutar
        if mensaje == "adelante":
            print("[mqtt] adelante")
            self.aceleracion_adelante = True
            self.estado_actual = ESTADO_ADELANTE
        elif mensaje == "atras":
            print("[mqtt] atras")
            self.aceleracion_adelante = False
            self.estado_actual = ESTADO_ADELANTE
        elif mensaje == "detener":
            print("[mqtt] detener")
            self.estado_actual = ESTADO_DETENER
        elif mensaje == "matriz_encender":
            print("[mqtt] matriz_encender")
            #llamar funcion que enciende la matriz
        elif mensaje == "matriz_apagar":
            print("[mqtt] matriz_apagar")
            #llamar funcion que apaga la matriz
