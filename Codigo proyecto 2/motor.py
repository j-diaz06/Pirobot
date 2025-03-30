from machine import Pin, PWM

class Motor:
    def __init__(self, en_a, in1, in2, en_b, in3, in4):
        self.enA = PWM(Pin(en_a))
        self.enA.freq(1000)
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        
        self.enB = PWM(Pin(en_b))
        self.enB.freq(1000)
        self.in3 = Pin(in3, Pin.OUT)
        self.in4 = Pin(in4, Pin.OUT)
        
        self.velocidad = 130  # Velocidad máxima por defecto

    def set_velocidad(self, vel):
        self.velocidad = max(0, min(255, vel))

    def get_velocidad(self):
        return self.velocidad

    def avanzar(self, vel):
        self.set_velocidad(vel)
        self.enA.duty_u16(self.velocidad * 257)
        self.enB.duty_u16(self.velocidad * 257)
        self.in1.value(0)
        self.in2.value(1)
        self.in3.value(1)
        self.in4.value(0)

    def retroceder(self, vel):
        self.set_velocidad(vel)
        self.enA.duty_u16(self.velocidad * 257)
        self.enB.duty_u16(self.velocidad * 257)
        self.in1.value(1)
        self.in2.value(0)
        self.in3.value(0)
        self.in4.value(1)

    def girar_izquierda(self, vel):
        self.set_velocidad(vel)
        self.enA.duty_u16(self.velocidad * 257)
        self.enB.duty_u16(self.velocidad * 257)
        self.in1.value(1)
        self.in2.value(0)
        self.in3.value(1)
        self.in4.value(0)

    def girar_derecha(self, vel):
        self.set_velocidad(vel)
        self.enA.duty_u16(self.velocidad * 257)
        self.enB.duty_u16(self.velocidad * 257)
        self.in1.value(0)
        self.in2.value(1)
        self.in3.value(0)
        self.in4.value(1)

    def detener(self):
        self.enA.duty_u16(0)
        self.enB.duty_u16(0)
        self.in1.value(0)
        self.in2.value(0)
        self.in3.value(0)
        self.in4.value(0)