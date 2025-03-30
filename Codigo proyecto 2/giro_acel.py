from machine import I2C, Pin
import time

class GiroAcel:
    def __init__(self, i2c, IntervaloCalibracion):
        #self.i2c = I2C(i2c_bus, sda=Pin(sda), scl=Pin(scl), freq=400000)
        self.i2c = i2c
        self.MPU_ADDR = 0x68
        self.G_R = 65.536  # Para 1000 DPS
        self.GYRO_FULL_SCALE_500_DPS = 0x08
        self.promedio = 0.0
        self.yaw = 0.0
        self.ultima_lectura = time.ticks_ms()
        
        self.intervaloCalibracion = IntervaloCalibracion
        self.lastCalCheck = 0
        
        #self.iniciar_mpu()

    """def _escribir_registro(self, reg, valor):
        self.i2c.writeto_mem(self.MPU_ADDR, reg, bytes([valor]))"""

    """def _leer_registro(self, reg, n_bytes=2):
        return self.i2c.readfrom_mem(self.MPU_ADDR, reg, n_bytes)"""

    def iniciar_mpu(self):
        # Despertar el MPU6050
        self.i2c.writeto_mem(self.MPU_ADDR, 0x6B, bytes([0]))
        # Configurar rango del giroscopio a ±500°/s
        self.i2c.writeto_mem(self.MPU_ADDR, 0x1B, bytes([self.GYRO_FULL_SCALE_500_DPS]))
        print("-----------Calibrando------------")
        suma = 0
        for _ in range(500):
            suma += self._leer_raw_z()
            time.sleep_ms(1)
        self.promedio = suma / 500
        self.lastCalCheck = time.ticks_ms()

    def _calibrar(self):
        if time.ticks_ms() - self.lastCalCheck >= self.intervaloCalibracion:
            print("-----------Calibrando------------")
            suma = 0
            for _ in range(500):
                suma += self._leer_raw_z()
                time.sleep_ms(1)
            self.promedio = suma / 500
            self.lastCalCheck = time.ticks_ms()
            #print("promedio=", self.promedio)

    def _leer_raw_z(self):
        max_retries = 3
        for _ in range(max_retries):
            try:
                data = self.i2c.readfrom_mem(self.MPU_ADDR, 0x47, 2)
                value = (data[0] << 8) | data[1]
                if value > 32767:
                    value -= 65536
                return value
            except OSError as e:
                print(f"I2C read error: {e}")
                time.sleep_ms(10)  # Wait a bit before retrying
        raise OSError("Failed to read from MPU6050 after multiple retries")

    def actualizar_yaw(self):
        ahora = time.ticks_ms()
        dt = time.ticks_diff(ahora, self.ultima_lectura) / 1000
        self.ultima_lectura = ahora
        
        raw = self._leer_raw_z() - self.promedio
        gyro_z = raw / self.G_R
        self.yaw += gyro_z * dt