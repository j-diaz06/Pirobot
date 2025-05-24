import machine, neopixel
import time

class MatrizWS2812B:
    def __init__(self, pin, brillo=1.0):
        """
        Inicializa la matriz WS2812B.

        Args:
            pin (int): El pin GPIO al que está conectada la matriz.
            num_pixeles (int): El número de píxeles en la matriz.
            ancho (int): El ancho de la matriz en píxeles.
            alto (int): El alto de la matriz en píxeles.
            brillo (float): El brillo de la matriz (0.0 a 1.0).
        """
        self.pin = pin
        self.ancho = 8
        self.alto = 8
        self.num_pixeles = self.ancho * self.alto
        self.brillo = brillo
        self.np = neopixel.NeoPixel(machine.Pin(pin), self.num_pixeles)
        self.apagar_matriz()
        
    def establecer_pixel(self, pixel, color):
        """
        Establece el color de un píxel individual con el brillo ajustado.

        Args:
            pixel (int): El índice del píxel.
            color (tuple): El color RGB del píxel (R, G, B).
        """
        r, g, b = color
        r = int(r * self.brillo)
        g = int(g * self.brillo)
        b = int(b * self.brillo)
        self.np[pixel] = (r, g, b)
    
    def apagar_matriz(self):
        """
        Apaga todos los pixeles de la matriz.
        """
        self.llenar_matriz((0,0,0))

    def llenar_matriz(self, color):
        """
        Llena toda la matriz con un color específico y el brillo ajustado.

        Args:
            color (tuple): El color RGB para llenar la matriz (R, G, B).
        """
        r, g, b = color
        r = int(r * self.brillo)
        g = int(g * self.brillo)
        b = int(b * self.brillo)
        for i in range(self.num_pixeles):
            self.np[i] = (r, g, b)

    def mostrar(self):
        """
        Muestra los cambios en la matriz.
        """
        self.np.write()

    def ajustar_brillo(self, brillo):
        """
        Ajusta el brillo de la matriz.

        Args:
            brillo (float): El nuevo brillo de la matriz (0.0 a 1.0).
        """
        self.brillo = brillo
    
    def mostrar_matriz_grafica(self, matriz_grafica, color_encendido=(255, 255, 255), color_apagado=(0, 0, 0)):
        """
        Muestra una matriz gráfica en la matriz WS2812B.

        Args:
            matriz_grafica (list): Una matriz de unos y ceros que representa la imagen.
            color_encendido (tuple): El color RGB para los píxeles encendidos.
            color_apagado (tuple): El color RGB para los píxeles apagados.
        """
        for y in range(self.alto):
            for x in range(self.ancho):
                pixel = y * self.ancho + x
                if matriz_grafica[y][x] == 1:
                    self.establecer_pixel(pixel, color_encendido)
                else:
                    self.establecer_pixel(pixel, color_apagado)
        self.mostrar()







# Llenar la matriz con un color
#matriz.ajustar_brillo(1)  # Brillo al 100%
#matriz.llenar_matriz((150, 156, 18))  # verde
#matriz.llenar_matriz((255, 0, 0))  # rojo
#matriz.llenar_matriz((0, 0, 255))  # rojo
#matriz.mostrar_matriz_grafica(cara_feliz, color_encendido=(150, 156, 18)) # verde encendido, apagado negro
#matriz.mostrar()

#matriz.apagar_matriz()
#matriz.mostrar()
""""
while True:
    matriz.llenar_matriz((0, 0, 255))
    matriz.mostrar()
    time.sleep(1)
    matriz.llenar_matriz((0, 255, 0))
    matriz.mostrar()
    time.sleep(1)
    matriz.llenar_matriz((255, 0, 0))
    matriz.mostrar()
    time.sleep(1)"""
