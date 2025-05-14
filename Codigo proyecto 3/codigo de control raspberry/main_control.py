import time
import cv2
import numpy as np
import config
import mqtt_comunicacion as mqtt # Importa las funciones MQTT
import deteccion_vision as vision # Importa las funciones de visión
from deteccion_vision import mouse_x, mouse_y, _callback_raton # Importar para coords del ratón
def get_user_choice(prompt, options):
    """Muestra un prompt y opciones, y devuelve la elección válida del usuario."""
    print(prompt)
    numeric_options = {}
    for i, (key, desc) in enumerate(options.items(), 1):
        print(f"  [{i}] {desc}")
        numeric_options[str(i)] = key

    while True:
        choice = input("Elige una opción (número): ").strip()
        if choice in numeric_options:
            return numeric_options[choice]
        else:
            print("Opción no válida. Inténtalo de nuevo.")

def traducir_patron_a_celdas_objetivo(patron_matriz):
    """
    Convierte la matriz PATRON_OBJETIVO (donde la fila 0 es la superior visual)
    a una lista de celdas objetivo (col, fila_logica) donde fila_logica 0 es la inferior del grid.
    """
    celdas_objetivo = []
    num_filas_patron = len(patron_matriz)
    if num_filas_patron == 0:
        return celdas_objetivo
    num_cols_patron = len(patron_matriz[0])

    for r_idx_patron in range(num_filas_patron):
        for c_idx_patron in range(num_cols_patron):
            if patron_matriz[r_idx_patron][c_idx_patron] == 1:
                # Convertir fila del patrón (0=arriba) a fila lógica del grid (0=abajo)
                fila_logica_grid = (config.FILAS_CUADRICULA - 1) - r_idx_patron
                # Asegurar que la fila lógica calculada esté dentro de los límites del grid
                if 0 <= fila_logica_grid < config.FILAS_CUADRICULA and \
                   0 <= c_idx_patron < config.COLUMNAS_CUADRICULA:
                    celdas_objetivo.append((c_idx_patron, fila_logica_grid))
                else:
                    print(f"Advertencia: La celda del patrón ({c_idx_patron}, {r_idx_patron} desde arriba) está fuera de los límites del grid al convertir.")
    
    print(f"Celdas del patrón objetivo generadas (col, fila_abajo=0): {celdas_objetivo}")
    return celdas_objetivo

def encontrar_celda_mas_cercana_en_patron(celda_vehiculo_actual, celdas_patron_objetivo):
    if not celdas_patron_objetivo: return None
    vehiculo_np = np.array(celda_vehiculo_actual)
    min_dist = float('inf')
    celda_mas_cercana = None
    for celda_obj in celdas_patron_objetivo:
        obj_np = np.array(celda_obj)
        dist = np.linalg.norm(vehiculo_np - obj_np)
        if dist < min_dist:
            min_dist = dist
            celda_mas_cercana = celda_obj
    return celda_mas_cercana

def run_shape_formation_loop(celdas_patron_objetivo):
    """
    Ejecuta un bucle de detección para la formación de una figura.
    Muestra el vehículo detectado y la celda objetivo más cercana en el patrón.
    Permite enviar al vehículo a esa celda objetivo.
    """
    print("\n--- Iniciando Formación de Figura ---")
    print("Ventana OpenCV:")
    print("  [M] Mover vehículo a celda asignada | [R] Reintentar Detección")
    print("  [Q/S] Volver al Menú Principal")

    vision.aplicar_configuracion_deteccion()
    time.sleep(1)

    celda_asignada_actual_patron = None
    vehiculo_celda_actual = None

    if not celdas_patron_objetivo:
        print("Error: No hay celdas en el patrón objetivo. Volviendo al menú.")
        vision.aplicar_configuracion_base()
        return None

    while True:
        detectado_ahora, col_detectado, fila_detectado = vision.detectar_color(
            mostrar_ventana=True,
            celdas_patron_objetivo=celdas_patron_objetivo,
            celda_asignada_patron=celda_asignada_actual_patron
        )

        if detectado_ahora:
            vehiculo_celda_actual = (col_detectado, fila_detectado)
            celda_asignada_actual_patron = encontrar_celda_mas_cercana_en_patron(
                vehiculo_celda_actual, celdas_patron_objetivo
            )
            # if celda_asignada_actual_patron:
            #      print(f"Vehículo en ({col_detectado},{fila_detectado}). Objetivo en patrón: {celda_asignada_actual_patron}")
            # else: # Esto no debería pasar si celdas_patron_objetivo no está vacío
            #     print(f"Vehículo en ({col_detectado},{fila_detectado}), no se asignó celda en patrón.")
        else:
            vehiculo_celda_actual = None
            # celda_asignada_actual_patron = None # Opcional: limpiar si no hay detección


        key = cv2.waitKey(50) & 0xFF
        if key == ord('m'):
            if detectado_ahora and celda_asignada_actual_patron:
                print(f"Confirmado: Mover vehículo a celda de patrón {celda_asignada_actual_patron}.")
                mqtt.enviar_comando_movimiento(config.TARGET_VEHICLE_ID, celda_asignada_actual_patron[0], celda_asignada_actual_patron[1])
                print(f"Comando de movimiento a ({celda_asignada_actual_patron[0]}, {celda_asignada_actual_patron[1]}) enviado.")
            elif not detectado_ahora: print("No se ha detectado ningún vehículo para mover.")
            elif not celda_asignada_actual_patron: print("No hay celda asignada en el patrón para el vehículo.")
        elif key == ord('r'):
            print("Reintentando detección...")
            celda_asignada_actual_patron = None
            vehiculo_celda_actual = None
        elif key == ord('q') or key == ord('s'):
            print("Volviendo al menú principal..."); break
    
    cv2.destroyWindow("Imagen con Cuadricula")
    cv2.destroyWindow("Mascara HSV")
    vision.aplicar_configuracion_base()

def run_detection_loop(last_detected_coords):
    """
    Ejecuta el bucle de detección visual interactivo para posición inicial.
    """
    print("\n--- Iniciando Detección Visual Interactiva (Posición Inicial) ---")
    print("Ventana OpenCV: [E] Confirmar y Enviar | [R] Reintentar | [P] Pausar | [Q/S] Menú")
    vision.aplicar_configuracion_deteccion()
    time.sleep(1)

    coords_confirmadas = None
    celda_detectada_actual = None

    while True:
        detectado_ahora, x_detectado, y_detectado = vision.detectar_color(mostrar_ventana=True)
        if detectado_ahora: celda_detectada_actual = (x_detectado, y_detectado)
        key = cv2.waitKey(50) & 0xFF
        if key == ord('e') and celda_detectada_actual:
            coords_confirmadas = celda_detectada_actual
            print(f"Posición ({coords_confirmadas[0]}, {coords_confirmadas[1]}) confirmada.")
            mqtt.enviar_posicion_inicial(config.TARGET_VEHICLE_ID, coords_confirmadas[0], coords_confirmadas[1])
            print(f"Posición inicial ({coords_confirmadas[0]}, {coords_confirmadas[1]}) enviada."); break
        elif key == ord('r'):
            print("Reintentando..."); celda_detectada_actual = None
        elif key == ord('p'):
            print("Pausado..."); cv2.waitKey(-1); print("Reanudando...")
        elif key == ord('q') or key == ord('s'):
            print("Volviendo al menú..."); break
        elif key != 255 and key == ord('e') and not celda_detectada_actual:
             print("Tecla 'e' presionada, pero no hay detección activa.")
    cv2.destroyWindow("Imagen con Cuadricula"); cv2.destroyWindow("Mascara HSV")
    vision.aplicar_configuracion_base()
    return coords_confirmadas

def show_preview_with_grid_and_coords(celdas_patron_objetivo_global):
    """
    Muestra una vista previa de la cámara con cuadrícula, coords del ratón y el patrón objetivo.
    """
    if not vision.picam2: print("Error: Cámara no inicializada."); return
    print("Restableciendo config. cámara a base para vista previa...")
    vision.aplicar_configuracion_base()
    vision.mostrar_vista_previa_con_coordenadas(celdas_patron_objetivo=celdas_patron_objetivo_global)
    print("Vista previa cerrada.")


def get_manual_coordinates():
    """Solicita y valida coordenadas X, Y del usuario."""
    while True:
        try:
            coord_x = int(input(f"Columna (0-{config.COLUMNAS_CUADRICULA - 1}): ").strip())
            if not (0 <= coord_x < config.COLUMNAS_CUADRICULA): raise ValueError("Columna fuera de rango.")
            coord_y = int(input(f"Fila (0-{config.FILAS_CUADRICULA - 1}, 0=abajo): ").strip())
            if not (0 <= coord_y < config.FILAS_CUADRICULA): raise ValueError("Fila fuera de rango.")
            return coord_x, coord_y
        except ValueError:
            print(f"Error: {e}. Introduce números enteros válidos.")

def run_interactive_menu():
    """Ejecuta la secuencia interactiva de detección y comandos."""
    print("--- Iniciando Conexión MQTT ---")
    cliente_mqtt = mqtt.conectar_mqtt()
    if not cliente_mqtt:
        print("Error crítico: No se pudo conectar a MQTT. Saliendo.")
        return

    print("\n--- Iniciando Cámara ---")
    camara = vision.inicializar_camara()
    if not camara:
        print("Error crítico: No se pudo inicializar la cámara. Saliendo.")
        if cliente_mqtt: mqtt.desconectar_mqtt()
        return

    # Traducir el patrón una vez al inicio
    celdas_patron_objetivo_global = traducir_patron_a_celdas_objetivo(config.PATRON_OBJETIVO)

    salir_programa = False
    last_detected_x, last_detected_y = None, None

    try:
        while not salir_programa:
            print("\n--- MENÚ PRINCIPAL ---")
            pos_text = "Ninguna"
            if last_detected_x is not None:
                pos_text = f"({last_detected_x}, {last_detected_y})"

            main_menu_options = {
                'identificar': f"Enviar '{config.COMANDO_IDENTIFICAR}'",
                'detectar_pos_inicial': "Detectar Posición Inicial Vehículo",
                'enviar_inicial_manual': "Enviar Posición Inicial (Manual)",
                'mover_manual': "Mover Vehículo (Manual)",
                'formar_figura': "Formar Figura Definida (Interactivo)", # Cambiado
                'apagar': f"Enviar '{config.COMANDO_APAGAR}'",
                'feliz': f"Enviar '{config.COMANDO_CARA_FELIZ}'",
                'preview_grid': "Mostrar Vista Previa (Patrón/Coords)", # Cambiado
                'salir': "Salir"
            }

            choice = get_user_choice("Elige una acción:", main_menu_options)

            if choice == 'identificar':
                mqtt.enviar_comando_matriz(config.TARGET_VEHICLE_ID, config.COMANDO_IDENTIFICAR)
                print(f"Comando '{config.COMANDO_IDENTIFICAR}' enviado."); time.sleep(0.5)
            elif choice == 'detectar_pos_inicial':
                coords = run_detection_loop((last_detected_x, last_detected_y))
                if coords: last_detected_x, last_detected_y = coords
            elif choice == 'formar_figura': # Cambiado
                run_shape_formation_loop(celdas_patron_objetivo_global)
            elif choice == 'enviar_inicial_manual':
                print("\n--- Coordenadas Posición Inicial ---")
                x, y = get_manual_coordinates()
                mqtt.enviar_posicion_inicial(config.TARGET_VEHICLE_ID, x, y)
                print(f"Posición inicial manual ({x}, {y}) enviada."); time.sleep(0.5)
            elif choice == 'mover_manual':
                print("\n--- Coordenadas Movimiento ---")
                x, y = get_manual_coordinates()
                mqtt.enviar_comando_movimiento(config.TARGET_VEHICLE_ID, x, y)
                print(f"Comando movimiento a ({x}, {y}) enviado."); time.sleep(0.5)
            elif choice == 'apagar':
                mqtt.enviar_comando_matriz(config.TARGET_VEHICLE_ID, config.COMANDO_APAGAR)
                print(f"Comando '{config.COMANDO_APAGAR}' enviado."); time.sleep(0.5)
            elif choice == 'feliz':
                mqtt.enviar_comando_matriz(config.TARGET_VEHICLE_ID, config.COMANDO_CARA_FELIZ)
                print(f"Comando '{config.COMANDO_CARA_FELIZ}' enviado."); time.sleep(0.5)
            elif choice == 'preview_grid': # Cambiado
                show_preview_with_grid_and_coords(celdas_patron_objetivo_global)
            elif choice == 'salir':
                salir_programa = True

    except (KeyboardInterrupt, SystemExit) as e:
        print(f"\nInterrupción: {e}")
        if 'cliente_mqtt' in locals() and cliente_mqtt and mqtt.client and mqtt.client.is_connected():
             try:
                 print("Intentando enviar 'apagar'...")
                 mqtt.enviar_comando_matriz(config.TARGET_VEHICLE_ID, config.COMANDO_APAGAR)
                 print("Comando 'apagar' (intento) enviado.")
             except Exception as mqtt_err:
                 print(f"Error al enviar 'apagar': {mqtt_err}")


    finally:
        print("\n--- Limpiando recursos ---")
        vision.liberar_camara()
        if 'cliente_mqtt' in locals() and cliente_mqtt:
            mqtt.desconectar_mqtt()
        print("Secuencia finalizada.")

if __name__ == "__main__":
    run_interactive_menu()
