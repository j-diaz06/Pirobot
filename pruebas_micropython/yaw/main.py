from machine import I2C, Pin
import time

# Constantes del MPU6050
MPU_ADDR = 0x68
GYRO_FULL_SCALE_500_DPS = 0x08
G_R = 65.536  # Escala para ±500 DPS (32768/500)

# Configuración I2C (pines SDA=21, SCL=22 comunes en ESP32)
i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400000)

# Variables globales
muestras_calibracion = 500
promedio = 0.0
yaw = 0.0
last_calibration = time.ticks_ms()
intervalo_calibracion = 60000  # 60 segundos
tiempo_prev = time.ticks_ms()

def configurar_mpu():
    # Despertar el MPU6050
    i2c.writeto_mem(MPU_ADDR, 0x6B, bytes([0]))
    # Configurar rango del giroscopio a ±500°/s
    i2c.writeto_mem(MPU_ADDR, 0x1B, bytes([GYRO_FULL_SCALE_500_DPS]))

def leer_raw_gyro_z():
    data = i2c.readfrom_mem(MPU_ADDR, 0x47, 2)
    value = (data[0] << 8) | data[1]
    if value > 32767:  # Ajustar para valores negativos
        value -= 65536
    return value

def calibrar_giro():
    global promedio, last_calibration
    suma = 0
    for _ in range(muestras_calibracion):
        suma += leer_raw_gyro_z()
        time.sleep_ms(1)
    promedio = suma / muestras_calibracion
    print("Nuevo bias:", promedio)
    last_calibration = time.ticks_ms()

def leer_giroscopio():
    global GyZ
    raw = leer_raw_gyro_z() - promedio
    return raw / G_R  # Convertir a °/s

def calcular_yaw():
    global yaw, tiempo_prev
    dt = time.ticks_diff(time.ticks_ms(), tiempo_prev) / 1000.0
    tiempo_prev = time.ticks_ms()
    yaw += GyZ * dt  # Integración del ángulo

# Configuración inicial
configurar_mpu()
calibrar_giro()

while True:
    # Calibración periódica
    if time.ticks_diff(time.ticks_ms(), last_calibration) > intervalo_calibracion:
        calibrar_giro()
    
    # Lectura y cálculo
    GyZ = leer_giroscopio()
    calcular_yaw()
    
    # Mostrar resultados
    print("YAW: {:.2f}°".format(yaw))
    
    time.sleep_ms(10)