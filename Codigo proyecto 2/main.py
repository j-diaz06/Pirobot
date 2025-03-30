# main.py
from machine import Pin, I2C
import time
import _thread

# Importar las clases desde los archivos proporcionados
from maquina import Maquina
from giro_acel import GiroAcel
from motor import Motor
from sensor import SensorVL53L0X

ENA = 25
IN1 = 27
IN2 = 32
IN3 = 14
IN4 = 33
ENB = 26

# Pines I2C para MPU6050 (21 y 22 por defecto)
SDA_PIN = 21
SCL_PIN = 22

XSHUT_SENSOR_DELANTERO = 5
DIRECCION_SENSOR_DELANTERO = 0x30
XSHUT_SENSOR_TRASERO = 4
DIRECCION_SENSOR_TRASERO = 0x35

DISTANCIA_DETECCION = 150

TIEMPO_CAMBIO_MARCHA = 2000  # Tiempo de detención configurable

aceleracion_adelante = True  # Valor inicial

intervaloCalibracion_yaw = 15000

# Inicializar I2C con los pines definidos
i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN))

maquina = Maquina(ENA, IN1, IN2, ENB, IN4, IN3,
                  DIRECCION_SENSOR_TRASERO, XSHUT_SENSOR_TRASERO,
                  DIRECCION_SENSOR_DELANTERO, XSHUT_SENSOR_DELANTERO, DISTANCIA_DETECCION,
                  aceleracion_adelante, intervaloCalibracion_yaw, TIEMPO_CAMBIO_MARCHA)

maquina.acelerarAdelante(True)  # Por ahora solo acelerará hacia adelante

maquina.iniciar()

while True:
    maquina.loop()
    time.sleep_ms(1)  # Añadir un pequeño delay para evitar el bloqueo