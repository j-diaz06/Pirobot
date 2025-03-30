from machine import Pin, I2C
import time
import vl53l0x

# Configuración de pines XSHUT para los dos sensores
xshut_delantero = Pin(5, Pin.OUT)  # Pin XSHUT del sensor delantero
xshut_trasero = Pin(4, Pin.OUT)   # Pin XSHUT del sensor trasero

# Inicializar ambos pines en alto para habilitar los sensores
xshut_delantero.value(0)
xshut_trasero.value(0)

# Configuración del bus I2C
i2c = I2C(0, scl=Pin(22), sda=Pin(21))

# Configurar el sensor delantero
xshut_delantero.value(1)
time.sleep(0.1)  # Esperar un momento
sensor_delantero = vl53l0x.VL53L0X(i2c)
sensor_delantero._register(0x8A, 0x30)  # Cambiar dirección I2C del sensor delantero a 0x30
#xshut_delantero.value(0) 

# Configurar el sensor trasero
xshut_trasero.value(1)  # Apagar el sensor delantero
time.sleep(0.1)  # Esperar un momento
sensor_trasero = vl53l0x.VL53L0X(i2c)
sensor_trasero._register(0x8A, 0x31)  # Cambiar dirección I2C del sensor trasero a 0x31
#xshut_trasero.value(0)  # Encender el sensor delantero

# Actualizar las direcciones en los objetos de los sensores
sensor_delantero.address = 0x30
sensor_trasero.address = 0x31

# Bucle principal para leer ambos sensores
while True:
    distancia_delantera = sensor_delantero.read()
    time.sleep(0.1)
    distancia_trasera = sensor_trasero.read()
    print("Distancia del sensor delantero: {} mm".format(distancia_delantera))
    print("Distancia del sensor trasero: {} mm".format(distancia_trasera))

