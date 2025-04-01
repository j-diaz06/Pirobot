import cv2
import numpy as np
from picamera2 import Picamera2

# Inicializar la cmara
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.set_controls({"Saturation": 3})
picam2.set_controls({"ExposureTime": 3000})
picam2.start()

# Variables globales para la calibraci
rango_hsv_bajo = np.array([30, 80, 80])  # Ajusta estos valores segn sea necesario
rango_hsv_alto = np.array([80, 255, 255])  # Ajusta estos valores segn sea necesario

def click_evento(evento, x, y, flags, param):
    """
    Funcin para manejar eventos de clic del ratn.
    """
    if evento == cv2.EVENT_LBUTTONDOWN:
        hsv_valor = hsv_imagen[y, x]
        print("Valor HSV:", hsv_valor)
        # Ajusta los rangos HSV segn el valor calibrado
        rango_hsv_bajo = np.array([hsv_valor[0] - 10, 100, 100])
        rango_hsv_alto = np.array([hsv_valor[0] + 10, 255, 255])

# Crear ventanas para mostrar la imagen y la mscara
cv2.namedWindow("Imagen")
cv2.namedWindow("Mscara")
cv2.setMouseCallback("Imagen", click_evento)

while True:
    imagen = picam2.capture_array()
    hsv_imagen = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
    mascara = cv2.inRange(hsv_imagen, rango_hsv_bajo, rango_hsv_alto)

    # Operaciones morfolgicas
    kernel = np.ones((5, 5), np.uint8)
    mascara = cv2.dilate(mascara, kernel, iterations=5)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)

    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contornos:
        contorno_mas_grande = max(contornos, key=cv2.contourArea)
        x, y, ancho, alto = cv2.boundingRect(contorno_mas_grande)
        cv2.rectangle(imagen, (x, y), (x + ancho, y + alto), (0, 255, 0), 2)
        centro_x = x + ancho // 2
        centro_y = y + alto // 2
        cv2.circle(imagen, (centro_x, centro_y), 5, (0, 0, 255), -1)
        cv2.putText(imagen, f"({centro_x}, {centro_y})", (centro_x + 10, centro_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow("Imagen", imagen)
    cv2.imshow("Mscara", mascara)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.close()
cv2.destroyAllWindows()
