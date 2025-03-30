import machine
import time
from motor import Motor

# Configuración de pines
ENA = 25
IN1 = 27
IN2 = 32
ENB = 26
IN3 = 33#14
IN4 = 14#33

motores = Motor(ENA, IN1, IN2, ENB, IN3, IN4)

# Configuración UART
# uart = machine.UART(1, baudrate=115200)  # UART0, baudrate 115200

def process_command(cmd):
    if cmd == '1':
        motores.avanzar(motores.get_velocidad())
        print("Adelante")
    elif cmd == '2':
        motores.retroceder(motores.get_velocidad())
        print("Atras")
    elif cmd == '3':
        motores.girar_izquierda(motores.get_velocidad())
        print("Izquierda")
    elif cmd == '4':
        motores.girar_derecha(motores.get_velocidad())
        print("Derecha")
    elif cmd == '0':
        motores.detener()
        print("Detener")
    else:
        print("Comando inválido")

while True:
    # if uart.any():
    #     command = uart.readline()
    #     command = command.decode('utf-8').strip()
    #     process_command(command)
    command = input()
    process_command(command)
    time.sleep(0.1)