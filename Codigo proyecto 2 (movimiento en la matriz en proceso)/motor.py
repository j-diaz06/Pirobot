# motor.py
from machine import Pin, PWM

class Motor:
    # Rango PWM en MicroPython con duty_u16
    PWM_MAX_DUTY = 65535 # 16 bits
    PWM_MIN_DUTY = 0

    def __init__(self, en_a_pin, in1_pin, in2_pin, en_b_pin, in3_pin, in4_pin):
        """
        Inicializa los pines para controlar un driver de motor dual tipo L298N o similar.

        Args:
            en_a_pin: Pin PWM para la velocidad del Motor A.
            in1_pin, in2_pin: Pines digitales para la dirección del Motor A.
            en_b_pin: Pin PWM para la velocidad del Motor B.
            in3_pin, in4_pin: Pines digitales para la dirección del Motor B.
        """
        print(f"[Motor] Inicializando pines: ENA={en_a_pin}, IN1={in1_pin}, IN2={in2_pin}, ENB={en_b_pin}, IN3={in3_pin}, IN4={in4_pin}")
        try:
            # Motor A
            self.enA = PWM(Pin(en_a_pin))
            self.enA.freq(1000) # Frecuencia PWM (1kHz es común)
            self.in1 = Pin(in1_pin, Pin.OUT)
            self.in2 = Pin(in2_pin, Pin.OUT)

            # Motor B
            self.enB = PWM(Pin(en_b_pin))
            self.enB.freq(1000)
            self.in3 = Pin(in3_pin, Pin.OUT)
            self.in4 = Pin(in4_pin, Pin.OUT)

            # Guardar velocidad (duty cycle)
            self._velocidad_duty_u16 = 0 # Duty cycle actual (0-65535)

             # Detener motores al inicio
            self.detener()
            print("[Motor] Pines configurados y motores detenidos.")

        except Exception as e:
            print(f"Error CRÍTICO inicializando pines de motor: {e}")
            # Podrías lanzar una excepción aquí para detener la inicialización del robot
            raise RuntimeError("Fallo al inicializar los motores")


    def _set_pwm_duty(self, duty_u16):
        """Establece el duty cycle (0-65535) para ambos motores."""
        # Limitar el valor entre 0 y 65535
        duty = max(self.PWM_MIN_DUTY, min(self.PWM_MAX_DUTY, int(duty_u16)))
        if duty != self._velocidad_duty_u16: # Actualizar solo si cambia
            self._velocidad_duty_u16 = duty
            try:
                self.enA.duty_u16(duty)
                self.enB.duty_u16(duty)
            except Exception as e:
                print(f"Error estableciendo PWM duty: {e}")


    def set_velocidad_duty(self, duty_u16):
         """Establece la velocidad directamente como duty cycle (0-65535)."""
         # print(f"[Motor] Estableciendo velocidad duty: {duty_u16}") # Debug
         self._set_pwm_duty(duty_u16)


    def avanzar(self, velocidad_duty_u16):
        """Mueve ambos motores hacia adelante."""
        # print(f"[Motor] Avanzando - Duty: {velocidad_duty_u16}") # Debug
        self.in1.value(0)
        self.in2.value(1) # Motor A Forward (Común)
        # La dirección del Motor B depende del cableado y montaje
        # Asumimos que IN3=1, IN4=0 es Forward para Motor B
        self.in3.value(1)
        self.in4.value(0)
        self._set_pwm_duty(velocidad_duty_u16)

    def retroceder(self, velocidad_duty_u16):
        """Mueve ambos motores hacia atrás."""
        # print(f"[Motor] Retrocediendo - Duty: {velocidad_duty_u16}") # Debug
        self.in1.value(1) # Motor A Backward
        self.in2.value(0)
        # Asumimos que IN3=0, IN4=1 es Backward para Motor B
        self.in3.value(0)
        self.in4.value(1)
        self._set_pwm_duty(velocidad_duty_u16)

    def girar_izquierda(self, velocidad_duty_u16):
        """Gira sobre el eje: Motor A hacia atrás, Motor B hacia adelante."""
        # print(f"[Motor] Girando Izquierda - Duty: {velocidad_duty_u16}") # Debug
        self.in1.value(1) # Motor A Backward
        self.in2.value(0)
        self.in3.value(1) # Motor B Forward
        self.in4.value(0)
        self._set_pwm_duty(velocidad_duty_u16)

    def girar_derecha(self, velocidad_duty_u16):
        """Gira sobre el eje: Motor A hacia adelante, Motor B hacia atrás."""
        # print(f"[Motor] Girando Derecha - Duty: {velocidad_duty_u16}") # Debug
        self.in1.value(0)
        self.in2.value(1) # Motor A Forward
        self.in3.value(0) # Motor B Backward
        self.in4.value(1)
        self._set_pwm_duty(velocidad_duty_u16)

    def detener(self, modo_freno=True):
        """
        Detiene ambos motores.

        Args:
            modo_freno (bool): True para frenado activo (IN1=IN2=0),
                               False para rueda libre (PWM duty=0).
        """
        # print(f"[Motor] Deteniendo (Freno: {modo_freno})") # Debug
        if modo_freno:
            # Frenado activo (puede ser más brusco y consumir más)
            self.in1.value(0)
            self.in2.value(0)
            self.in3.value(0)
            self.in4.value(0)
            self._set_pwm_duty(self.PWM_MAX_DUTY) # Asegurar que el freno esté activo si EN está alto
            # O poner duty a 0 también funciona con IN1=IN2=0
            # self._set_pwm_duty(self.PWM_MIN_DUTY)
        else:
            # Rueda libre (coasting)
             self._set_pwm_duty(self.PWM_MIN_DUTY)
             # El estado de IN1/IN2/IN3/IN4 no importa mucho si EN=0

    def deinit(self):
        """Desactiva los PWMs para liberar los pines."""
        print("[Motor] Desinicializando PWMs...")
        try:
            self.enA.deinit()
            self.enB.deinit()
        except Exception as e:
            print(f"Error durante deinit de motores: {e}")

