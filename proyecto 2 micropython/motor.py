from machine import Pin, PWM

class Motor:
    def __init__(self, enA, in1, in2, enB, in3, in4):
        self.enA = PWM(Pin(enA), Pin(in1, Pin.OUT), Pin(in2, Pin.OUT)
        self.enB = PWM(Pin(enB), Pin(in3, Pin.OUT), Pin(in4, Pin.OUT)
        self.velocidad = 140  # Velocidad inicial
        self.estado = 0  # Estado inicial: detenido

    def configurar(self):
        for pin in self.enA + self.enB:
            if isinstance(pin, Pin):
                pin.init(Pin.OUT)
        print("[Motor] Motores configurados correctamente.")

    def avanzar(self, vel):
        if vel < 1 or vel > 255:
            print(f"[Motor] Error: Velocidad inválida (Avanzar) {vel}")
            return
        print(f"[Motor] Avanzar: ENA={vel}, ENB={vel}")
        self.estado = 1
        self.enA[0].duty(vel)
        self.enB[0].duty(vel)
        self.enA[1].value(0)
        self.enA[2].value(1)
        self.enB[1].value(1)
        self.enB[2].value(0)

    def retroceder(self, vel):
        if vel < 110 or vel > 255:
            print(f"[Motor] Error: Velocidad inválida (Retroceder) {vel}")
            return
        print(f"[Motor] Retroceder: ENA={vel}, ENB={vel}")
        self.estado = 2
        self.enA[0].duty(vel)
        self.enB[0].duty(vel)
        self.enA[1].value(1)
        self.enA[2].value(0)
        self.enB[1].value(0)
        self.enB[2].value(1)

    def girar_izquierda(self, vel):
        if vel < 110 or vel > 255:
            print(f"[Motor] Error: Velocidad inválida (Girar Izquierda) {vel}")
            return
        velDer = vel
        velIzq = vel  # Reducción del 40%
        print(f"[Motor] Girar Izquierda: ENA={velDer}, ENB={velIzq}")
        self.estado = 3
        self.enA[0].duty(velDer)
        self.enB[0].duty(velIzq)
        self.enA[1].value(1)
        self.enA[2].value(0)
        self.enB[1].value(1)
        self.enB[2].value(0)

    def girar_derecha(self, vel):
        if vel < 1 or vel > 255:
            print(f"[Motor] Error: Velocidad inválida (Girar Derecha) {vel}")
            return
        velIzq = vel
        velDer = vel  # Reducción del 40%
        print(f"[Motor] Girar Derecha: ENA={velDer}, ENB={velIzq}")
        self.estado = 4
        self.enA[0].duty(velDer)
        self.enB[0].duty(velIzq)
        self.enA[1].value(0)
        self.enA[2].value(1)
        self.enB[1].value(0)
        self.enB[2].value(1)

    def detener(self):
        print("[Motor] Deteniendo motores...")
        self.estado = 0
        self.enA[0].duty(0)
        self.enB[0].duty(0)
        self.enA[1].value(0)
        self.enA[2].value(0)
        self.enB[1].value(0)
        self.enB[2].value(0)

    def set_velocidad(self, vel):
        if vel < 110 or vel > 255:
            print(f"[Motor] Error: Velocidad inválida (Set Velocidad) {vel}")
            return
        self.velocidad = vel
        print(f"[Motor] Nueva velocidad: {self.velocidad}")
        self.actualizar_velocidad()

    def get_velocidad(self):
        return self.velocidad

    def actualizar_velocidad(self):
        if self.estado != 0:  # Si los motores están en movimiento
            print(f"[Motor] Actualizando velocidad: Estado={self.estado}, Velocidad={self.velocidad}")
            if self.estado == 1:
                self.avanzar(self.velocidad)
            elif self.estado == 2:
                self.retroceder(self.velocidad)
            elif self.estado == 3:
                self.girar_izquierda(self.velocidad)
            elif self.estado == 4:
                self.girar_derecha(self.velocidad)