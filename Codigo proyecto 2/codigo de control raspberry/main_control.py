import time
import cv2
import config # Importa la configuración global
import mqtt_comunicacion as mqtt # Importa las funciones MQTT
import deteccion_vision as vision # Importa las funciones de visión

def run_sequence():
    """Ejecuta la secuencia completa de detección y comandos."""

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

    vehiculo_detectado = False
    centro_x_detectado = None
    centro_y_detectado = None

    try:
        # 3. Enviar comando inicial para encender matriz de identificación
        #    (Se envía sólo una vez antes de empezar a buscar)
        print(f"\n--- Paso 1: Enviando comando '{config.COMANDO_IDENTIFICAR}' para iniciar detección ---")
        mqtt.enviar_comando_matriz(config.TARGET_VEHICLE_ID, config.COMANDO_IDENTIFICAR)
        time.sleep(1) # Dar tiempo al vehículo para reaccionar

        # 4. Bucle de detección infinito
        print("\n--- Paso 2: Buscando el vehículo por color (Bucle infinito hasta detección)... ---")
        print("(Presiona 'q' en la ventana de OpenCV si se muestra para salir)")
        while not vehiculo_detectado:
            # Llama a la función de detección. Pasa True para ver ventanas de depuración.
            # Cambia a False si no necesitas las ventanas visuales.
            detectado, centro_x, centro_y = vision.detectar_color(mostrar_ventana=True)

            if detectado:
                print(f"\n¡Vehículo detectado en ({centro_x}, {centro_y})!")
                vehiculo_detectado = True
                centro_x_detectado = centro_x # Guardar coordenadas para usarlas después
                centro_y_detectado = centro_y
                # No salimos del bucle aquí, sólo cambiamos la bandera
            else:
                # Pequeña pausa y manejo de salida si se muestran ventanas
                if vision.detectar_color(mostrar_ventana=True): # Si se muestra ventana
                     # waitKey(1) es esencial para que OpenCV procese eventos y muestre la ventana
                     # Un valor más alto (e.g., 50) da más tiempo de pausa
                     key = cv2.waitKey(50) & 0xFF
                     if key == ord('q'): # Permite salir con 'q'
                        print("Salida manual solicitada durante detección.")
                        # Forzar salida del bucle while si se presiona 'q'
                        break
                     elif key == ord('p'): # Ejemplo: tecla 'p' para pausar/reanudar (opcional)
                         print("Pausado. Presiona cualquier tecla en la ventana para continuar...")
                         cv2.waitKey(-1) # Espera indefinidamente hasta que se presione una tecla

                else: # Si no se muestra ventana, igual hacemos una pausa
                    time.sleep(0.05) # Pausa de 50ms para no saturar CPU

        # --- Fin del Bucle de Detección ---

        # 5. Si el vehículo fue detectado, proceder con comandos manuales
        if vehiculo_detectado:
            print("\n--- Vehículo Detectado. Esperando confirmación para comandos ---")

            # Usar las coordenadas guardadas
            #x = centro_x_detectado
            #y = centro_y_detectado

            input(f"\n--- Paso 3: Presiona Enter para enviar comando '{config.COMANDO_APAGAR}' ---")
            mqtt.enviar_comando_matriz(config.TARGET_VEHICLE_ID, config.COMANDO_APAGAR)
            print(f"Comando '{config.COMANDO_APAGAR}' enviado.")
            time.sleep(0.5) # Pequeña pausa

            input(f"\n--- Paso 4: Presiona Enter para enviar posición inicial ({config.x}, {config.y}) ---")
            mqtt.enviar_posicion_inicial(config.TARGET_VEHICLE_ID, config.x, config.y)
            print(f"Posición inicial ({config.x}, {config.y}) enviada.")
            time.sleep(0.5)

            input(f"\n--- Paso 5: Presiona Enter para enviar comando de movimiento a ({config.POSICION_NUEVA_X}, {config.POSICION_NUEVA_Y}) ---")
            mqtt.enviar_comando_movimiento(config.TARGET_VEHICLE_ID, config.POSICION_NUEVA_X, config.POSICION_NUEVA_Y)
            print(f"Comando de movimiento a ({config.POSICION_NUEVA_X}, {config.POSICION_NUEVA_Y}) enviado.")
            # Puedes añadir una espera mayor si quieres ver el movimiento antes del siguiente paso
            # time.sleep(5)

            input(f"\n--- Paso 6: Presiona Enter para enviar comando '{config.COMANDO_CARA_FELIZ}' ---")
            mqtt.enviar_comando_matriz(config.TARGET_VEHICLE_ID, config.COMANDO_CARA_FELIZ)
            print(f"Comando '{config.COMANDO_CARA_FELIZ}' enviado.")
            time.sleep(1)

        else:
            # Esto sólo se alcanzaría si salimos del bucle con 'q'
            print("\nProceso detenido antes de completar la secuencia de comandos.")
            # Opcional: enviar comando de apagar por si acaso quedó encendido
            mqtt.enviar_comando_matriz(config.TARGET_VEHICLE_ID, config.COMANDO_APAGAR)

    except KeyboardInterrupt:
        print("\nProceso interrumpido por el usuario (Ctrl+C).")
        # Intentar apagar la matriz si se interrumpe
        if cliente_mqtt and cliente_mqtt.is_connected():
             mqtt.enviar_comando_matriz(config.TARGET_VEHICLE_ID, config.COMANDO_APAGAR)

    finally:
        # 6. Limpieza final
        print("\n--- Limpiando recursos ---")
        vision.liberar_camara() # Cierra ventanas de OpenCV también
        mqtt.desconectar_mqtt()
        print("Secuencia finalizada.")

# --- Punto de Entrada Principal ---
if __name__ == "__main__":
    run_sequence()
