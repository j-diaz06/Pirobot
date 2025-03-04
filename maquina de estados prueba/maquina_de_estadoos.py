from machine import Pin
from time import sleep

# Definir pines de los motores
m1a = Pin(26, Pin.OUT)
m1b = Pin(25, Pin.OUT)
m2a = Pin(19, Pin.OUT)
m2b = Pin(18, Pin.OUT)

# Definir pines de los sensores con pull-up
sens_adelante = Pin(4, Pin.IN, Pin.PULL_UP) 
sens_derecha = Pin(0, Pin.IN, Pin.PULL_UP)
sens_izquierda = Pin(2, Pin.IN, Pin.PULL_UP)

# Definir los estados como constantes
ADELANTE = 0
GIRO_DERECHA = 1
GIRO_IZQUIERDA = 2
DETENIDO = 3

# Estado inicial
estado = ADELANTE

# Funciones de movimiento
def forward():  
    m1a.value(0)
    m1b.value(1)
    m2a.value(0)
    m2b.value(1)

def right():  
    m1a.value(0)
    m1b.value(1)
    m2a.value(1)
    m2b.value(0)

def left():  
    m1a.value(1)
    m1b.value(0)
    m2a.value(0)
    m2b.value(1)

def stop():
    m1a.value(0)
    m1b.value(0)
    m2a.value(0)
    m2b.value(0)

# Función para leer sensores
def read_sensors():
    return sens_adelante.value(), sens_derecha.value(), sens_izquierda.value()

# Bucle principal con máquina de estados
while True:
    adelante, derecha, izquierda = read_sensors()

    # Invertimos la lógica: 0 significa obstáculo, 1 significa libre
    if estado == ADELANTE:
        if adelante:  # Si hay un obstáculo adelante
            if not derecha:  # Si la derecha está libre
                estado = GIRO_DERECHA
            elif not izquierda:  # Si la izquierda está libre
                estado = GIRO_IZQUIERDA
            else:  # Si ambos lados están bloqueados
                estado = DETENIDO
    elif estado == GIRO_DERECHA:
        if not adelante:  # Si ya no hay obstáculo adelante, avanzar
            estado = ADELANTE
    elif estado == GIRO_IZQUIERDA:
        if not adelante:  # Si ya no hay obstáculo adelante, avanzar
            estado = ADELANTE
        elif not derecha:  # Si la derecha se libera antes que adelante
            estado = GIRO_DERECHA
    elif estado == DETENIDO:
        if not adelante:
            estado = ADELANTE
        elif not derecha:
            estado = GIRO_DERECHA
        elif not izquierda:
            estado = GIRO_IZQUIERDA

    # Ejecutar el movimiento según el estado
    if estado == ADELANTE:
        forward()
    elif estado == GIRO_DERECHA:
        right()
    elif estado == GIRO_IZQUIERDA:
        left()
    elif estado == DETENIDO:
        stop()
    
    print(estado)
    sleep(0.1)
