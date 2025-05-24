# motor.py (Modificado para usar PWM de 16 bits directamente)
from machine import Pin, PWM

class Motor:
    def __init__(self, en_a, in1, in2, en_b, in3, in4):
        # Configuración de pines PWM y de dirección
        self.enA = PWM(Pin(en_a))
        self.enA.freq(1000)  # Frecuencia PWM
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)

        self.enB = PWM(Pin(en_b))
        self.enB.freq(1000)  # Frecuencia PWM
        self.in3 = Pin(in3, Pin.OUT)
        self.in4 = Pin(in4, Pin.OUT)

        # Inicializar motores detenidos
        self.detener()

    def set_pwm_duties(self, duty_A, duty_B):
        """
        Establece directamente los ciclos de trabajo PWM (0-65535) para los motores A y B.
        """
        # Asegurarse que los valores están en el rango correcto (0-65535 para duty_u16)
        duty_A_u16 = max(0, min(65535, int(duty_A)))
        duty_B_u16 = max(0, min(65535, int(duty_B)))
        # Aplicar los ciclos de trabajo
        self.enA.duty_u16(duty_A_u16)
        self.enB.duty_u16(duty_B_u16)

    def avanzar(self, duty_cycle):
        """
        Mueve ambos motores hacia adelante con el ciclo de trabajo especificado (0-65535).
        """
        # Establecer dirección para avanzar
        self.in1.value(0)
        self.in2.value(1)
        self.in3.value(1) # Asumiendo cableado estándar para avanzar
        self.in4.value(0)
        # Establecer la velocidad (ciclo de trabajo)
        self.set_pwm_duties(duty_cycle, duty_cycle)

    def retroceder(self, duty_cycle):
        """
        Mueve ambos motores hacia atrás con el ciclo de trabajo especificado (0-65535).
        """
        # Establecer dirección para retroceder
        self.in1.value(1)
        self.in2.value(0)
        self.in3.value(0) # Asumiendo cableado estándar para retroceder
        self.in4.value(1)
        # Establecer la velocidad (ciclo de trabajo)
        self.set_pwm_duties(duty_cycle, duty_cycle)

    def girar_izquierda(self, duty_cycle):
        """
        Gira el robot hacia la izquierda (motor A atrás, motor B adelante)
        con el ciclo de trabajo especificado (0-65535).
        """
        # Establecer dirección para girar izquierda
        self.in1.value(1); self.in2.value(0) # Motor A atrás
        self.in3.value(1); self.in4.value(0) # Motor B adelante
        # Establecer la velocidad (ciclo de trabajo)
        self.set_pwm_duties(duty_cycle, duty_cycle)

    def girar_derecha(self, duty_cycle):
        """
        Gira el robot hacia la derecha (motor A adelante, motor B atrás)
        con el ciclo de trabajo especificado (0-65535).
        """
        # Establecer dirección para girar derecha
        self.in1.value(0); self.in2.value(1) # Motor A adelante
        self.in3.value(0); self.in4.value(1) # Motor B atrás
        # Establecer la velocidad (ciclo de trabajo)
        self.set_pwm_duties(duty_cycle, duty_cycle)

    def detener(self):
        """
        Detiene ambos motores (PWM a 0 y pines de dirección a 0).
        """
        self.set_pwm_duties(0, 0) # Poner ciclo de trabajo a 0
        # Poner pines de dirección a bajo (puede ayudar como freno o simplemente apagar)
        self.in1.value(0); self.in2.value(0)
        self.in3.value(0); self.in4.value(0)

