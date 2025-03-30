#include "webcontrol.h"

WebControl::WebControl(/*Motor& m,*/ Maquina& maq) 
    : server(80), webSocket(81), /*motores(m),*/ maquina(maq), lastUpdate(0) {}

void WebControl::begin(const char* ssid, const char* password, const char* hostname) {
    Serial.println("[WiFi] Iniciando conexión...");
    WiFi.begin(ssid, password);
    
    int intentos = 0;
    while (WiFi.status() != WL_CONNECTED && intentos < 15) {
        delay(1000);
        Serial.printf("[WiFi] Intento %d: Conectando...\n", intentos + 1);
        intentos++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] Conectado! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("[WiFi] Error: Fallo de conexión");
        return;
    }

    if (!MDNS.begin(hostname)) {
        Serial.println("[mDNS] Error: No se pudo iniciar");
    }
    
    server.on("/", [this]() { handleRoot(); });
    server.begin();
    
    webSocket.begin();
    webSocket.onEvent([this](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
        webSocketEvent(num, type, payload, length);
    });
}

void WebControl::handleRoot() {
    String html = R"=====(
        <!DOCTYPE html>
        <html>
        <head>
            <title>Datos de Sensores</title>
            <style>
                body { font-family: Arial, sans-serif; text-align: center; }
                .sensor-box { 
                    margin: 20px; padding: 15px; 
                    border: 2px solid #007bff; 
                    border-radius: 10px; 
                    display: inline-block;
                }
                .sensor-label { color: #2c3e50; font-size: 1.2em; }
                .sensor-value { color: #e74c3c; font-size: 1.4em; font-weight: bold; }
            </style>
        </head>
        <body>
            <h1>Monitor de Sensores</h1>
            
            <div class="sensor-box">
                <h2 class="sensor-label">Sensor Delantero</h2>
                <p><span class="sensor-value" id="delantero">---</span> mm</p>
            </div>
        
            <div class="sensor-box">
                <h2 class="sensor-label">Sensor Trasero</h2>
                <p><span class="sensor-value" id="trasero">---</span> mm</p>
            </div>
        
            <div class="sensor-box">
                <h2 class="sensor-label">Orientación (Yaw)</h2>
                <p><span class="sensor-value" id="yaw">---</span>°</p>
            </div>
        
            <script>
                const ws = new WebSocket('ws://' + location.hostname + ':81/');
                
                ws.onmessage = function(e) {
                    const data = JSON.parse(e.data);
                    if(data.delantero !== undefined) 
                        document.getElementById('delantero').textContent = data.delantero;
                    if(data.trasero !== undefined) 
                        document.getElementById('trasero').textContent = data.trasero;
                    if(data.yaw !== undefined) 
                        document.getElementById('yaw').textContent = data.yaw.toFixed(1);
                };
            </script>
        </body>
        </html>
        )=====";
    server.send(200, "text/html", html);
}

void WebControl::webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[WebSocket] Cliente %u desconectado\n", num);
            break;
            
        case WStype_CONNECTED:
            Serial.printf("[WebSocket] Cliente %u conectado\n", num);
            break;
            
        case WStype_TEXT:
            String msg((char*)payload);
            Serial.printf("[WebSocket] Comando recibido: %s\n", msg.c_str());
            /*
            if (msg.startsWith("speed:")) {
                motores.setVelocidad(msg.substring(6).toInt());
            } else if (msg == "adelante") {
                motores.avanzar(motores.getVelocidad());
            } else if (msg == "atras") {
                motores.retroceder(motores.getVelocidad());
            } else if (msg == "izquierda") {
                motores.girarIzquierda(motores.getVelocidad());
            } else if (msg == "derecha") {
                motores.girarDerecha(motores.getVelocidad());
            } else if (msg == "detener") {
                motores.detener();
            }*/
            break;
    }
}

void WebControl::sendSensorData() {
    if (millis() - lastUpdate >= 50) { // Actualizar cada 30ms
        // Leer datos de los sensores y MPU
        int delantero = maquina.getSensorDelantero().medida();
        int trasero = maquina.getSensorTrasero().medida();
        float yaw = maquina.getYaw();

        // Crear JSON con los datos
        String data = "{\"delantero\":" + String(delantero) + 
                     ",\"trasero\":" + String(trasero) + 
                     ",\"yaw\":" + String(yaw, 1) + "}";
        
        webSocket.broadcastTXT(data);
        lastUpdate = millis();
    }
}

void WebControl::handleClient() {
    server.handleClient();
    webSocket.loop();
    sendSensorData();
}