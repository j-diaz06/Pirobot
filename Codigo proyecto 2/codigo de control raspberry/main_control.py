import time
import cv2
import config # Importa la configuración global
import mqtt_comunicacion as mqtt # Importa las funciones MQTT
import deteccion_vision as vision # Importa las funciones de visión
from deteccion_vision import mouse_x, mouse_y, _callback_raton # Importar para coords del ratón
def get_user_choice(prompt, options):
    """Muestra un prompt y opciones, y devuelve la elección válida del usuario."""
    print(prompt)
    # Mapear números a claves originales para mantener la lógica interna
    numeric_options = {}
    for i, (key, desc) in enumerate(options.items(), 1):
        print(f"  [{i}] {desc}")
        numeric_options[str(i)] = key # Guardar '1' -> 'e', '2' -> 's', etc.

    while True:
        # Ajustar el prompt para ser más genérico
        choice = input("Elige una opción (número): ").strip()
        if choice in numeric_options:
            return numeric_options[choice] # Devolver la clave original ('e', 's', etc.)
        else:
            print("Opción no válida. Inténtalo de nuevo.")

def run_detection_loop(last_detected_coords):
    """
    Ejecuta el bucle de detección visual interactivo.
    Permite confirmar y enviar ('e'), reintentar ('r'), pausar ('p') o salir ('q'/'s').
    Devuelve las coordenadas confirmadas (x, y) o None si no se confirmó.
    """
    print("\n--- Iniciando Detección Visual Interactiva ---")
    print("Ventana OpenCV:")
    print("  [E] Confirmar y Guardar Posición Detectada | [R] Reintentar Búsqueda")
    print("  [P] Pausar/Reanudar | [Q/S] Volver al Menú Principal")

    # Aplicar configuración de cámara para detección
    print("Aplicando configuración de cámara para detección...")
    vision.aplicar_configuracion_deteccion()
    time.sleep(1) # Pausa para que se apliquen los ajustes

    coords_confirmadas = None
    centro_x_actual, centro_y_actual = None, None # Coords de la detección MÁS RECIENTE en el frame

    while True:
        detectado_ahora, x_detectado, y_detectado = vision.detectar_color(mostrar_ventana=True)
        if detectado_ahora:
            centro_x_actual, centro_y_actual = x_detectado, y_detectado # Actualizar coords actuales

        key = cv2.waitKey(50) & 0xFF

        if key == ord('e') and centro_x_actual is not None:
            coords_confirmadas = (centro_x_actual, centro_y_actual)
            print(f"Posición ({coords_confirmadas[0]}, {coords_confirmadas[1]}) confirmada.")
            # --- ENVIAR POSICIÓN INMEDIATAMENTE ---
            mqtt.enviar_posicion_inicial(config.TARGET_VEHICLE_ID, coords_confirmadas[0], coords_confirmadas[1])
            print(f"Posición inicial ({coords_confirmadas[0]}, {coords_confirmadas[1]}) enviada.")
            break # Salir del bucle de detección
        elif key == ord('r'):
            print("Reintentando detección...")
            centro_x_actual, centro_y_actual = None, None # Olvidar detección actual para la confirmación 'e'
            # El bucle continúa automáticamente
        elif key == ord('p'):
            print("Pausado. Presiona cualquier tecla en la ventana para continuar...")
            cv2.waitKey(-1)
            print("Reanudando...")
        elif key == ord('q') or key == ord('s'):
            print("Volviendo al menú principal...")
            break # Salir del bucle de detección
        elif key != 255 and key == ord('e') and centro_x_actual is None:
             print("Tecla 'e' presionada, pero no hay detección activa para confirmar.")

    # Cerrar ventanas de detección al salir de esta función
    cv2.destroyWindow("Imagen con Cuadricula")
    cv2.destroyWindow("Mascara HSV")
    return coords_confirmadas

def show_preview_with_grid_and_coords():
    """
    Muestra una vista previa de la cámara con cuadrícula y coordenadas del ratón.
    Usa la configuración base de la cámara, sin ajustes de detección ni máscara HSV.
    """
    # Acceder a la cámara global a través del módulo vision
    if not vision.picam2: # Accede a la variable global a través del módulo vision
        print("Error: Cámara no inicializada.")
        return

    # --- Asegurar configuración base antes de mostrar ---
    print("Restableciendo configuración de cámara a base para vista previa...")
    vision.aplicar_configuracion_base()

    print("\n--- Mostrando Vista Previa con Cuadrícula y Coordenadas ---")
    print("Mueve el ratón sobre la ventana para ver las coordenadas.")
    print("Presiona 'q' en la ventana para volver al menú.")
    nombre_ventana = "Vista Previa (Cuadricula/Coords) (Pulsa 'q')"
    cv2.namedWindow(nombre_ventana)
    cv2.setMouseCallback(nombre_ventana, _callback_raton) # Configurar callback del ratón

    while True:
        frame = vision.picam2.capture_array()
        if frame is None:
            print("Error capturando frame.")
            time.sleep(0.1)
            continue

        # Dibujar cuadrícula sobre una copia del frame
        # La función dibujar_cuadricula maneja la conversión de color si es necesario internamente
        frame_con_cuadricula = vision.dibujar_cuadricula(frame.copy())

        # Mostrar coordenadas del ratón (usando las variables importadas)
        texto_coords = f"Pixel: ({mouse_x}, {mouse_y})"
        cv2.putText(frame_con_cuadricula, texto_coords, (10, config.CAM_HEIGHT - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow(nombre_ventana, frame_con_cuadricula)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyWindow(nombre_ventana)
    print("Vista previa simple cerrada.")

def get_manual_coordinates():
    """Solicita y valida coordenadas X, Y del usuario."""
    while True:
        try:
            coord_x_str = input("Introduce la coordenada X de destino: ").strip()
            coord_x = int(coord_x_str)
            # Opcional: Añadir validación de rango si es necesario
            if not (0 <= coord_x < config.COLUMNAS_CUADRICULA):
                print(f"Error: La coordenada X debe estar entre 0 y {config.COLUMNAS_CUADRICULA - 1}.")
                continue
            coord_y_str = input("Introduce la coordenada Y de destino: ").strip()
            coord_y = int(coord_y_str)
            if not (0 <= coord_y < config.FILAS_CUADRICULA):
                print(f"Error: La coordenada Y debe estar entre 0 y {config.FILAS_CUADRICULA - 1}.")
                continue
            return coord_x, coord_y
        except ValueError:
            print("Error: Introduce números enteros válidos para las coordenadas.")

def run_interactive_menu():
    """Ejecuta la secuencia interactiva de detección y comandos."""

    # 1. Inicializar MQTT
    print("--- Iniciando Conexión MQTT ---")
    cliente_mqtt = mqtt.conectar_mqtt()
    if not cliente_mqtt:
        print("Error crítico: No se pudo conectar a MQTT. Saliendo.")
        return # Salir si no hay conexión MQTT

    # 2. Inicializar Cámara
    print("\n--- Iniciando Cámara ---")
    camara = vision.inicializar_camara()
    if not camara:
        print("Error crítico: No se pudo inicializar la cámara. Saliendo.")
        mqtt.desconectar_mqtt() # Desconectar MQTT antes de salir
        return # Salir si no hay cámara

    # Variables para almacenar estado
    salir_programa = False
    last_detected_x = None
    last_detected_y = None

    try:
        # Bucle del Menú Principal
        while not salir_programa:
            print("\n--- MENÚ PRINCIPAL ---")
            # Mostrar última posición detectada si existe
            pos_text = "Ninguna"
            if last_detected_x is not None:
                pos_text = f"({last_detected_x}, {last_detected_y})"

            main_menu_options = {
                'identificar': f"Enviar Comando '{config.COMANDO_IDENTIFICAR}'",
                'detectar': "Iniciar Detección Visual Interactiva",
                'enviar_inicial_manual': "Enviar Posición Inicial (solicitará coords)", # Nueva opción
                # 'enviar_ultima': f"Enviar Última Posición Detectada: {pos_text}", # Opción eliminada
                # 'enviar_manual': f"Enviar Posición Manual ({config.x}, {config.y})", # Opción eliminada
                'mover': "Enviar Comando Mover (solicitará coords)", # Descripción actualizada
                'apagar': f"Enviar Comando '{config.COMANDO_APAGAR}'",
                'feliz': f"Enviar Comando '{config.COMANDO_CARA_FELIZ}'",
                'preview_grid': "Mostrar Vista Previa (Cuadrícula/Coords)",
                # 'preview_roi_original': "Mostrar Vista Previa (Ajuste ROI Original)", # Opción original, comentada por ahora
                'salir': "Salir del Programa"
            }

            choice = get_user_choice("Elige una acción:", main_menu_options)

            # --- Ejecutar Acción Seleccionada ---
            if choice == 'identificar':
                mqtt.enviar_comando_matriz(config.TARGET_VEHICLE_ID, config.COMANDO_IDENTIFICAR) # Usar clave original
                print(f"Comando '{config.COMANDO_IDENTIFICAR}' enviado.")
                time.sleep(0.5) # Pequeña pausa

            elif choice == 'detectar':
                # Ejecutar el bucle de detección y obtener coordenadas confirmadas
                coords = run_detection_loop((last_detected_x, last_detected_y))
                # run_detection_loop ahora envía la posición si se confirma con 'e'
                # Actualizamos las coords locales por si el usuario quiere reenviarlas manualmente
                if coords:
                    last_detected_x, last_detected_y = coords
                # Al volver, se mostrará el menú principal de nuevo

            elif choice == 'enviar_inicial_manual':
                print("\n--- Solicitando Coordenadas de Posición Inicial ---")
                inicial_x, inicial_y = get_manual_coordinates() # Obtener coords del usuario
                mqtt.enviar_posicion_inicial(config.TARGET_VEHICLE_ID, inicial_x, inicial_y)
                print(f"Posición inicial manual ({inicial_x}, {inicial_y}) enviada.")
                time.sleep(0.5)

            elif choice == 'mover':
                print("\n--- Solicitando Coordenadas de Movimiento ---")
                destino_x, destino_y = get_manual_coordinates() # Obtener coords del usuario
                mqtt.enviar_comando_movimiento(config.TARGET_VEHICLE_ID, destino_x, destino_y)
                print(f"Comando de movimiento a ({destino_x}, {destino_y}) enviado.")
                time.sleep(0.5)

            elif choice == 'apagar':
                mqtt.enviar_comando_matriz(config.TARGET_VEHICLE_ID, config.COMANDO_APAGAR)
                print(f"Comando '{config.COMANDO_APAGAR}' enviado.")
                time.sleep(0.5)

            elif choice == 'feliz':
                mqtt.enviar_comando_matriz(config.TARGET_VEHICLE_ID, config.COMANDO_CARA_FELIZ)
                print(f"Comando '{config.COMANDO_CARA_FELIZ}' enviado.")
                time.sleep(0.5)

            elif choice == 'preview_grid':
                show_preview_with_grid_and_coords()

            # elif choice == 'preview_roi_original':
                # Esta función ya maneja su propio bucle y salida con 'q'
                # Asegúrate de que esta función use la config base si la llamas
                # vision.mostrar_vista_previa_con_coordenadas() # Llamada a la función original si la descomentas

            elif choice == 'salir':
                salir_programa = True

    except (KeyboardInterrupt, SystemExit) as e:
        print(f"\nProceso interrumpido o salida solicitada: {e}")
        # Intentar apagar la matriz si se interrumpe y MQTT está conectado
        # Comprobar si cliente_mqtt existe y está conectado
        if 'cliente_mqtt' in locals() and cliente_mqtt and mqtt.client and mqtt.client.is_connected():
             print("Intentando enviar comando 'apagar' antes de salir...")
             try:
                 # Usar un timeout corto para no bloquear si hay problemas
                 # Nota: La función enviar_comando_matriz no tiene argumento timeout, lo omitimos
                 mqtt.enviar_comando_matriz(config.TARGET_VEHICLE_ID, config.COMANDO_APAGAR)
                 print("Comando 'apagar' enviado (o intento realizado).")
             except Exception as mqtt_err:
                 print(f"No se pudo enviar comando 'apagar' al salir: {mqtt_err}")


    finally:
        # 6. Limpieza final
        print("\n--- Limpiando recursos ---")
        vision.liberar_camara() # Cierra ventanas de OpenCV también
        if 'cliente_mqtt' in locals() and cliente_mqtt:
            mqtt.desconectar_mqtt()
        print("Secuencia finalizada.")

# --- Punto de Entrada Principal ---
if __name__ == "__main__":
    run_interactive_menu()
