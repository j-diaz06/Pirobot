import cv2
import numpy as np
from picamera2 import Picamera2
import time
import config # Importa la configuración actualizada

picam2 = None # Variable global para la cámara
# Variables globales para coordenadas del ratón (no se usan directamente en este archivo pero se importan en main)
mouse_x, mouse_y = -1, -1

def inicializar_camara():
    """Inicializa y configura la cámara PiCamera2 (configuración base) si no está ya inicializada."""
    global picam2
    # Si ya existe una instancia, retornarla
    if picam2 is not None:
        print("Cámara ya inicializada.")
        return picam2

    try:
        print("Inicializando cámara...")
        picam2 = Picamera2()
        # Configura la cámara con los valores de config.py (formato y tamaño)
        cam_config = picam2.create_preview_configuration(
            main={"format": config.CAM_FORMAT, "size": (config.CAM_WIDTH, config.CAM_HEIGHT)}
        )
        picam2.configure(cam_config)
        picam2.start()
        print("Cámara inicializada correctamente (configuración base).")
        time.sleep(2)
        return picam2
    except Exception as e:
        print(f"Error al inicializar la cámara: {e}")
        picam2 = None # Asegurarse que es None en caso de error
        return None

def aplicar_configuracion_deteccion():
    """Aplica los controles específicos de cámara para la detección de color usando la instancia global."""
    global picam2
    if not picam2:
        print("Error: Intento de aplicar configuración a cámara no inicializada.")
        return
    try:
        print("Aplicando configuración de cámara para detección (Saturación, Exposición)...")
        # Usar los nuevos nombres de variables de config.py
        picam2.set_controls({"Saturation": config.CAM_SATURATION_DETECCION})
        picam2.set_controls({"ExposureTime": config.CAM_EXPOSURE_TIME_DETECCION})
        time.sleep(0.5)
        print("Configuración de detección aplicada.")
    except Exception as e:
        print(f"Error al aplicar configuración de detección: {e}")

def aplicar_configuracion_base():
    """Restablece los controles de la cámara a valores base/automáticos."""
    global picam2
    if not picam2:
        print("Error: Intento de aplicar configuración base a cámara no inicializada.")
        return
    try:
        print("Aplicando configuración de cámara base (Saturación normal, Exposición Auto)...")
        picam2.set_controls({"Saturation": config.CAM_SATURATION_BASE})
        # Poner ExposureTime a 0 generalmente activa el modo automático en picamera2
        picam2.set_controls({"ExposureTime": 0})
        time.sleep(0.5)
        print("Configuración base aplicada.")
    except Exception as e:
        print(f"Error al aplicar configuración base: {e}")

def calcular_celda(x, y):
    """
    Calcula la celda de la cuadrícula donde se encuentra un punto (x, y) en píxeles.
    Retorna (fila, columna) de la celda en la cuadrícula (fila 0 abajo).
    """
    # Calcular el ancho y alto de la ROI (Region of Interest)
    roi_ancho = config.ROI_X_FIN - config.ROI_X_INICIO
    roi_alto = config.ROI_Y_FIN - config.ROI_Y_INICIO
    
    # Calcular el ancho y alto de cada celda en píxeles
    celda_ancho_px = roi_ancho / config.COLUMNAS_CUADRICULA
    celda_alto_px = roi_alto / config.FILAS_CUADRICULA
    
    # Ajustar las coordenadas relativas a la ROI
    x_rel = x - config.ROI_X_INICIO
    y_rel = y - config.ROI_Y_INICIO
    
    # Calcular la columna y fila (asegurarse de que estén dentro de los límites)
    columna = min(max(int(x_rel / celda_ancho_px), 0), config.COLUMNAS_CUADRICULA - 1)
    
    # Calcular la fila de arriba a abajo (0 arriba, N-1 abajo)
    fila_arriba_abajo = min(max(int(y_rel / celda_alto_px), 0), config.FILAS_CUADRICULA - 1)
    
    # Invertir la fila para que 0 sea abajo y N-1 sea arriba
    fila = config.FILAS_CUADRICULA - 1 - fila_arriba_abajo
    
    return fila, columna

def dibujar_cuadricula(imagen, celdas_patron_objetivo=None, celda_asignada_patron=None):
    """
    Dibuja una cuadrícula en la imagen.
    Opcionalmente, resalta las celdas de un patrón objetivo y una celda asignada en ese patrón.
    Retorna la imagen con la cuadrícula dibujada.
    """
    # Obtener dimensiones de la ROI
    roi_ancho = config.ROI_X_FIN - config.ROI_X_INICIO
    roi_alto = config.ROI_Y_FIN - config.ROI_Y_INICIO

    # Calcular tamaño de celda en píxeles
    celda_ancho_px = roi_ancho / config.COLUMNAS_CUADRICULA
    celda_alto_px = roi_alto / config.FILAS_CUADRICULA

    # Color de las líneas (azul claro)
    color_grid = (255, 100, 0)  # BGR - Azul claro para la cuadrícula

    # Dibujar el rectángulo de la ROI
    cv2.rectangle(imagen, (config.ROI_X_INICIO, config.ROI_Y_INICIO), 
                  (config.ROI_X_FIN, config.ROI_Y_FIN), (0, 0, 255), 2) # ROI en rojo

    # Dibujar líneas verticales
    for i in range(1, config.COLUMNAS_CUADRICULA):
        x = int(config.ROI_X_INICIO + i * celda_ancho_px)
        cv2.line(imagen, (x, config.ROI_Y_INICIO), (x, config.ROI_Y_FIN), color_grid, 1)

    # Dibujar líneas horizontales
    for i in range(1, config.FILAS_CUADRICULA):
        y = int(config.ROI_Y_INICIO + i * celda_alto_px)
        cv2.line(imagen, (config.ROI_X_INICIO, y), (config.ROI_X_FIN, y), color_grid, 1)

    # Opcional: añadir coordenadas de celdas (col, fila) con (0,0) abajo a la izquierda
    for fila_idx_fisica in range(config.FILAS_CUADRICULA): # fila_idx_fisica va de 0 (arriba) a N-1 (abajo)
        for col_idx in range(config.COLUMNAS_CUADRICULA):
            # Calcular la posición central de la celda física (fila_idx, col)
            x_centro = int(config.ROI_X_INICIO + col_idx * celda_ancho_px + celda_ancho_px / 2)
            y_centro = int(config.ROI_Y_INICIO + fila_idx_fisica * celda_alto_px + celda_alto_px / 2)
            # Calcular la etiqueta de fila lógica (0 abajo, N-1 arriba)
            fila_etiqueta_logica = config.FILAS_CUADRICULA - 1 - fila_idx_fisica # (0 abajo)
            cv2.putText(imagen, f"{col_idx},{fila_etiqueta_logica}", (x_centro - 15, y_centro + 5), # Ajustar posición para mejor visibilidad
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # Dibujar el patrón objetivo si se proporciona
    if celdas_patron_objetivo:
        for col_target, fila_target in celdas_patron_objetivo:
            # La fila_target es lógica (0 abajo), resaltar_celda espera fila lógica
            imagen = resaltar_celda(imagen, fila_target, col_target, color=config.COLOR_PATRON_OBJETIVO, grosor=1)
    
    # Resaltar la celda asignada en el patrón objetivo, si se proporciona
    if celda_asignada_patron:
        col_asignada, fila_asignada = celda_asignada_patron
        imagen = resaltar_celda(imagen, fila_asignada, col_asignada, color=config.COLOR_CELDA_ASIGNADA_PATRON, grosor=3)

    return imagen

def resaltar_celda(imagen, fila_logica, columna, color=(0, 255, 0), grosor=2):
    """
    Resalta una celda específica en la cuadrícula.
    La fila 0 es la inferior, la columna 0 es la izquierda.
    """
    # Obtener dimensiones de la ROI
    roi_ancho = config.ROI_X_FIN - config.ROI_X_INICIO
    roi_alto = config.ROI_Y_FIN - config.ROI_Y_INICIO

    # Calcular tamaño de celda en píxeles
    celda_ancho_px = roi_ancho / config.COLUMNAS_CUADRICULA
    celda_alto_px = roi_alto / config.FILAS_CUADRICULA

    # Calcular la fila física (0 arriba, N-1 abajo) a partir de la fila lógica (0 abajo)
    # Validar que la fila lógica y columna estén en rango
    if not (0 <= fila_logica < config.FILAS_CUADRICULA and 0 <= columna < config.COLUMNAS_CUADRICULA):
        # print(f"Advertencia: Intento de resaltar celda fuera de rango ({fila_logica}, {columna})")
        return imagen

    fila_fisica = config.FILAS_CUADRICULA - 1 - fila_logica

    # Calcular las coordenadas de la celda física
    x1 = int(config.ROI_X_INICIO + columna * celda_ancho_px)
    y1 = int(config.ROI_Y_INICIO + fila_fisica * celda_alto_px) # Usar fila física
    x2 = int(x1 + celda_ancho_px)
    y2 = int(y1 + celda_alto_px)

    # Dibujar un rectángulo resaltado (verde, grosor 2 por defecto)
    cv2.rectangle(imagen, (x1, y1), (x2, y2), color, grosor)

    return imagen

def detectar_color(mostrar_ventana=False, celdas_patron_objetivo=None, celda_asignada_patron=None):
    """
    Captura un frame de la cámara global, busca el color definido en config.py dentro de la cuadrícula
    y devuelve True si se detecta, junto con las coordenadas de la celda (col, fila).
    Opcionalmente dibuja el patrón objetivo y la celda asignada en él.
    Muestra ventanas internamente si mostrar_ventana=True.
    """
    global picam2
    if not picam2:
        print("Error: Cámara no inicializada.")
        return False, None, None

    imagen = picam2.capture_array()
    if imagen is None:
        print("Error al capturar imagen.")
        return False, None, None

    # Convertir la imagen a espacio de color HSV (intentar BGR y luego RGBA)
    try:
        hsv_imagen = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
    except cv2.error:
        try:
             hsv_imagen = cv2.cvtColor(imagen, cv2.COLOR_RGBA2HSV)
        except cv2.error as e2:
             print(f"Error irrecuperable convirtiendo a HSV: {e2}")
             # Si mostramos ventana, al menos mostrar el frame original con error
             if mostrar_ventana:
                 frame_error = imagen.copy()
                 cv2.putText(frame_error, "Error ColorSpace", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                 # Dibujar cuadrícula aquí también para contexto
                 frame_error_con_cuad = dibujar_cuadricula(frame_error, celdas_patron_objetivo, celda_asignada_patron)
                 cv2.imshow("Imagen con Cuadricula", frame_error_con_cuad)
                 cv2.waitKey(1) # Necesario para que imshow funcione
             return False, None, None

    # Crear la máscara usando los rangos de config.py
    hsv_bajo_np = np.array(config.HSV_BAJO)
    hsv_alto_np = np.array(config.HSV_ALTO)
    mascara = cv2.inRange(hsv_imagen, hsv_bajo_np, hsv_alto_np)

    # --- Operaciones morfológicas para limpiar la máscara (valores originales) ---
    kernel = np.ones((5, 5), np.uint8)
    # Usar valores de config.py para operaciones morfológicas
    mascara_proc = cv2.dilate(mascara, kernel, iterations=config.MORPH_DILATE_ITER)
    mascara_proc = cv2.morphologyEx(mascara_proc, cv2.MORPH_CLOSE, kernel, iterations=config.MORPH_CLOSE_ITER)

    # Encontrar contornos en la máscara procesada
    contornos, _ = cv2.findContours(mascara_proc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detectado = False
    centro_x_px, centro_y_px = None, None # Coordenadas en píxeles
    celda_fila_detectada, celda_col_detectada = None, None # Coordenadas en celdas (fila 0 abajo)

    imagen_con_cuadricula = None # Inicializar para evitar UnboundLocalError si no se entra al if
    if mostrar_ventana:
        # Convertir imagen original a 3 canales si tiene 4, para dibujar y mostrar
        if imagen.shape[2] == 4:
             imagen_para_dibujar = cv2.cvtColor(imagen, cv2.COLOR_BGRA2BGR)
        else:
             imagen_para_dibujar = imagen.copy()
        # Dibujar cuadrícula Y patrón objetivo ANTES de dibujar detección
        imagen_con_cuadricula = dibujar_cuadricula(imagen_para_dibujar, celdas_patron_objetivo, celda_asignada_patron)


    if contornos:
        contorno_mas_grande = max(contornos, key=cv2.contourArea)
        area = cv2.contourArea(contorno_mas_grande)

        if area > config.MIN_AREA_DETECCION:
            x_rect, y_rect, ancho_rect, alto_rect = cv2.boundingRect(contorno_mas_grande)
            centro_x_px = x_rect + ancho_rect // 2
            centro_y_px = y_rect + alto_rect // 2

            # Verificar si el punto está dentro de la ROI
            if (config.ROI_X_INICIO <= centro_x_px < config.ROI_X_FIN and
                config.ROI_Y_INICIO <= centro_y_px < config.ROI_Y_FIN):

                detectado = True
                # Calcular en qué celda está el objeto (fila lógica, columna)
                celda_fila_detectada, celda_col_detectada = calcular_celda(centro_x_px, centro_y_px)

                if mostrar_ventana and imagen_con_cuadricula is not None:
                    # Dibujar sobre imagen_con_cuadricula
                    cv2.rectangle(imagen_con_cuadricula, (x_rect, y_rect), (x_rect + ancho_rect, y_rect + alto_rect), (0, 255, 0), 2) # Verde para detección
                    cv2.circle(imagen_con_cuadricula, (centro_x_px, centro_y_px), 5, (0, 0, 255), -1) # Rojo para centro
                    # Resaltar la celda detectada
                    imagen_con_cuadricula = resaltar_celda(imagen_con_cuadricula, celda_fila_detectada, celda_col_detectada, color=(0,255,0), grosor=2) # Celda detectada en verde
                    # Mostrar texto de detección
                    cv2.putText(imagen_con_cuadricula, f"Detectado: ({celda_col_detectada},{celda_fila_detectada})",
                                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # --- Mostrar ventanas (si aplica) ---
    if mostrar_ventana:
        if imagen_con_cuadricula is None: # Si no se pudo preparar imagen_para_dibujar antes
            # Este bloque es redundante si el bloque anterior siempre se ejecuta cuando mostrar_ventana es True
            if imagen.shape[2] == 4:
                imagen_para_dibujar = cv2.cvtColor(imagen, cv2.COLOR_BGRA2BGR)
            else:
                imagen_para_dibujar = imagen.copy()
            imagen_con_cuadricula = dibujar_cuadricula(imagen_para_dibujar, celdas_patron_objetivo, celda_asignada_patron)

        # Si no se detectó nada, mostrar texto "Buscando..."
        if not detectado:
            cv2.putText(imagen_con_cuadricula, "Buscando...", (20, 30),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        # Mostrar imagen principal y máscara
        cv2.imshow("Imagen con Cuadricula", imagen_con_cuadricula)
        cv2.imshow("Mascara HSV", mascara_proc) # Mostrar máscara procesada

    # Devolver estado y celda lógica (col, fila)
    return detectado, celda_col_detectada, celda_fila_detectada

def liberar_camara():
    """Detiene y libera la cámara global y la establece a None."""
    global picam2
    if picam2:
        try:
            picam2.stop()
            picam2.close()
            print("Cámara liberada.")
        except Exception as e:
             print(f"Error al liberar cámara: {e}")
        finally:
             picam2 = None # Asegurarse que es None incluso si hay error al cerrar
    cv2.destroyAllWindows()

# Callback para el evento del ratón
def _callback_raton(event, x, y, flags, param):
    """Actualiza las coordenadas globales del ratón."""
    global mouse_x, mouse_y
    mouse_x, mouse_y = x, y

def mostrar_vista_previa_con_coordenadas(celdas_patron_objetivo=None):
    """
    Muestra una ventana con la vista de la cámara global, la cuadrícula dibujada
    y las coordenadas del ratón en tiempo real. Opcionalmente, dibuja el patrón objetivo.
    Permite salir presionando 'q'. (Función de ayuda para calibración/visualización)
    ASUME que inicializar_camara() ya fue llamada exitosamente.
    """
    global picam2, mouse_x, mouse_y

    if not picam2:
        print("Error crítico: Cámara no inicializada antes de llamar a vista previa.")
        return

    print("\n--- Iniciando Vista Previa (Patrón/Coords) ---")
    print("Mueve el ratón para ver coordenadas. Pulsa 'q' para salir.")
    print("Presiona 'q' en la ventana de OpenCV para cerrar esta vista y continuar.")

    nombre_ventana = "Vista Previa Camara (Pulsa 'q' para salir)"
    cv2.namedWindow(nombre_ventana)
    cv2.setMouseCallback(nombre_ventana, _callback_raton)

    while True:
        frame = picam2.capture_array()
        if frame is None:
            print("Error al capturar frame para vista previa.")
            time.sleep(0.5) # Pequeña pausa antes de reintentar
            continue

        frame_para_dibujar = frame.copy()
        if frame_para_dibujar.shape[2] == 4: # Asegurar 3 canales para dibujar
            frame_para_dibujar = cv2.cvtColor(frame_para_dibujar, cv2.COLOR_BGRA2BGR)

        # Dibujar cuadrícula y, opcionalmente, el patrón objetivo
        frame_con_cuadricula = dibujar_cuadricula(frame_para_dibujar, celdas_patron_objetivo=celdas_patron_objetivo)

        # Mostrar coordenadas del ratón
        texto_coords = f"Pixel: ({mouse_x}, {mouse_y})"
        cv2.putText(frame_con_cuadricula, texto_coords, (10, config.CAM_HEIGHT - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # Mostrar la imagen
        cv2.imshow(nombre_ventana, frame_con_cuadricula)

        # Esperar tecla 'q' para salir
        key = cv2.waitKey(30) & 0xFF # Espera 30ms
        if key == ord('q'):
            break

    cv2.destroyWindow(nombre_ventana) # Cerrar solo esta ventana
    print("--- Vista Previa Finalizada ---")