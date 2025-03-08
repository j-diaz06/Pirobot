#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>  // Incluir biblioteca para mDNS
#include "motor.h"

#define ENA 25
#define IN1 27
#define IN2 32
#define IN3 14
#define IN4 33
#define ENB 26

Motor motores(ENA, IN1, IN2, ENB, IN3, IN4);

const char* ssid = "Conectando...";
const char* password = "";
const char* hostname = "esp32";  // Nombre para mDNS (accesible como esp32.local)

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    if (type == WStype_TEXT) {
        String message = String((char*)payload);
        
        if (message.startsWith("speed:")) {
            int speed = message.substring(6).toInt();
            motores.setVelocidad(speed);
        }
        else {
            if (message == "adelante") motores.avanzar(motores.getVelocidad());
            else if (message == "atras") motores.retroceder(motores.getVelocidad());
            else if (message == "izquierda") motores.girarIzquierda(motores.getVelocidad());
            else if (message == "derecha") motores.girarDerecha(motores.getVelocidad());
            else if (message == "detener") motores.detener();
        }
    }
}

void handleRoot() {
    String html = R"(
    <!DOCTYPE html>
    <html>
    <head>
        <title>Control Motores WS</title>
        <style>
            body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }
            .slider { width: 80%; margin: 20px auto; }
            .button { padding: 10px 20px; font-size: 16px; margin: 10px; }
        </style>
    </head>
    <body>
        <h1>Control Motores (WebSocket)</h1>
        <div>
            <button class="button" id="btnAdelante">Adelante</button>
            <button class="button" id="btnAtras">Atrás</button>
            <button class="button" id="btnIzquierda">Izquierda</button>
            <button class="button" id="btnDerecha">Derecha</button>
            <button class="button" id="btnDetener">Detener</button>
        </div>
        <div>
            <label>Velocidad: </label>
            <input type="range" id="sliderVelocidad" class="slider" min="0" max="255" value="150">
            <span id="velocidadValue">150</span>
        </div>
        <script>
            // Usar el nombre mDNS en lugar de la IP
            var ws = new WebSocket('ws://esp32.local:81/');
            
            document.getElementById('btnAdelante').addEventListener('click', () => ws.send('adelante'));
            document.getElementById('btnAtras').addEventListener('click', () => ws.send('atras'));
            document.getElementById('btnIzquierda').addEventListener('click', () => ws.send('izquierda'));
            document.getElementById('btnDerecha').addEventListener('click', () => ws.send('derecha'));
            document.getElementById('btnDetener').addEventListener('click', () => ws.send('detener'));
            
            document.getElementById('sliderVelocidad').addEventListener('input', function() {
                document.getElementById('velocidadValue').textContent = this.value;
                ws.send('speed:' + this.value);
            });
        </script>
    </body>
    </html>
    )";
    server.send(200, "text/html", html);
}

void setup() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) delay(1000);
    
    // Iniciar mDNS
    if (!MDNS.begin(hostname)) {
        // Si falla, puedes agregar un mensaje de depuración
    }
    
    server.on("/", handleRoot);
    server.begin();
    
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    
    motores.configurar();
}

void loop() {
    webSocket.loop();
    server.handleClient();
    //MDNS.update(); // Mantener mDNS activo
}