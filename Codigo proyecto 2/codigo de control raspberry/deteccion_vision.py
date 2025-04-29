import cv2
import numpy as np
from picamera2 import Picamera2
import time
import config # Importa la configuración actualizada

picam2 = None # Variable global para la cámara

def inicializar_camara():
    """Inicializa y configura la cámara PiCamera2."""
    global picam2
    try:
        picam2 = Picamera2()
        # Configura la cámara con los valores de config.py
        cam_config = picam2.create_preview_configuration(
            main={"format": config.CAM_FORMAT, "size": (config.CAM_WIDTH, config.CAM_HEIGHT)}
        ) # [cite: 1]
        picam2.configure(cam_config)
        # Añadir controles de cámara basados en deteccion_matriz.txt
        picam2.set_controls({"Saturation": config.CAM_SATURATION}) # [cite: 1]
        picam2.set_controls({"ExposureTime": config.CAM_EXPOSURE_TIME}) # [cite: 1]
        picam2.start()
        print("Cámara inicializada correctamente.")
        time.sleep(2) # Dar tiempo a la cámara para que se ajuste
        return picam2
    except Exception as e:
        print(f"Error al inicializar la cámara: {e}")
        picam2 = None
        return None

def detectar_color(mostrar_ventana=False):
    """
    Captura un frame, busca el color definido en config.py y devuelve
    True si se detecta, junto con las coordenadas del centro.
    """
    global picam2
    if not picam2:
        print("Error: Cámara no inicializada.")
        return False, None, None

    imagen = picam2.capture_array()
    if imagen is None:
        print("Error al capturar imagen.")
        return False, None, None

    # Convertir la imagen a espacio de color HSV
    hsv_imagen = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)

    # Crear la máscara usando los rangos de config.py
    # Asegurarse que los valores de config son tuplas numpy
    hsv_bajo_np = np.array(config.HSV_BAJO) # [cite: 1]
    hsv_alto_np = np.array(config.HSV_ALTO) # [cite: 1]
    mascara = cv2.inRange(hsv_imagen, hsv_bajo_np, hsv_alto_np)

    # --- Operaciones morfológicas para limpiar la máscara ---
    # Ajustadas para coincidir con deteccion_matriz.txt
    kernel = np.ones((5, 5), np.uint8) # [cite: 2]
    # Dilatar con 5 iteraciones
    mascara = cv2.dilate(mascara, kernel, iterations=5) # [cite: 2]
    # Cerrar (Close)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel) # [cite: 3]
    # Nota: El número de iteraciones para MORPH_CLOSE no estaba especificado
    # en el fragmento, se asume 1 o se puede ajustar si es necesario.
    # Si necesitas más iteraciones para CLOSE, añade ", iterations=X" al final.

    # Encontrar contornos en la máscara
    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # [cite: 3]

    detectado = False
    centro_x = None
    centro_y = None

    if contornos:
        contorno_mas_grande = max(contornos, key=cv2.contourArea) # [cite: 3]
        area = cv2.contourArea(contorno_mas_grande)
        umbral_area_minima = 100 # Mantener un umbral para evitar ruido

        if area > umbral_area_minima:
            detectado = True
            x, y, ancho, alto = cv2.boundingRect(contorno_mas_grande) # [cite: 3]
            centro_x = x + ancho // 2 # [cite: 3]
            centro_y = y + alto // 2 # [cite: 3]

            if mostrar_ventana:
                cv2.rectangle(imagen, (x, y), (x + ancho, y + alto), (0, 255, 0), 2) # [cite: 3]
                cv2.circle(imagen, (centro_x, centro_y), 5, (0, 0, 255), -1) # [cite: 4]
                cv2.putText(imagen, f"Detectado ({centro_x}, {centro_y})", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2) # [cite: 4]
    else:
        if mostrar_ventana:
             cv2.putText(imagen, "No detectado", (20, 30),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    if mostrar_ventana:
        cv2.imshow("Imagen Original", imagen) # [cite: 4]
        cv2.imshow("Mascara HSV", mascara) # [cite: 4]

    return detectado, centro_x, centro_y

def liberar_camara():
    """Detiene y libera la cámara."""
    global picam2
    if picam2:
        picam2.stop()
        picam2.close()
        print("Cámara liberada.")
    cv2.destroyAllWindows() # [cite: 4]