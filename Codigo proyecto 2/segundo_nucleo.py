# segundo_nucleo.py
import _thread
import time

class Segundo_nucleo:
    def __init__(self, maquina):
        self.maquina = maquina

    def yaw_task(self):
        while True:
            # Leer el giroscopio y calcular el Yaw
            gyro_z = self.maquina.giro_acel.gyro.z # Ajustar según la biblioteca MPU6050
            # Suponiendo que la clase MPU6050 tiene un método para calcular el Yaw
            # self.maquina.giro_acel.calcular_yaw(gyro_z)
            time.sleep_ms(1)  # Esperar 1 ms

    def iniciar(self):
        print("Iniciando segundo núcleo...")
        _thread.start_new_thread(self.yaw_task, ())
        print("Segundo núcleo iniciado")