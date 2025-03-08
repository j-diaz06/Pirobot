#include "webcontrol.h"

WebControl::WebControl(Motor& m, Sensor& s1, Sensor& s2) 
    : server(80), webSocket(81), motores(m), sensor1(s1), sensor2(s2), lastSensor1Update(0), lastSensor2Update(0), alternateSensor(false) {}

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
        <title>Control Motores</title>
        <style>
            body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }
            .control-group { margin: 20px; padding: 15px; border: 1px solid #ddd; border-radius: 8px; }
            .sensor-data { color: #2c3e50; font-size: 1.2em; }
        </style>
    </head>
    <body>
        <h1>Control Motores</h1>
        
        <div class="control-group">
            <h2>Motores</h2>
            <button class="btn" onclick="send('adelante')">▲ Adelante</button><br>
            <button class="btn" onclick="send('izquierda')">◄ Izquierda</button>
            <button class="btn" onclick="send('detener')">■ Detener</button>
            <button class="btn" onclick="send('derecha')">► Derecha</button><br>
            <button class="btn" onclick="send('atras')">▼ Atrás</button>
            <br><br>
            <input type="range" id="velocidad" min="110" max="255" value="110" oninput="updateSpeed(this.value)">
            <span id="speedValue">110</span>
        </div>

        <div class="control-group">
            <h2>Sensores</h2>
            <p>Sensor 1: <span id="sensor1" class="sensor-data">---</span> mm</p>
            <p>Sensor 2: <span id="sensor2" class="sensor-data">---</span> mm</p>
        </div>

        <script>
            const ws = new WebSocket('ws://' + location.hostname + ':81/');
            
            function send(cmd) { ws.send(cmd); }
            function updateSpeed(val) {
                document.getElementById('speedValue').textContent = val;
                ws.send('speed:' + val);
            }
            
            ws.onmessage = function(e) {
                const data = JSON.parse(e.data);
                if(data.sensor1) document.getElementById('sensor1').textContent = data.sensor1;
                if(data.sensor2) document.getElementById('sensor2').textContent = data.sensor2;
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
            }
            break;
    }
}

void WebControl::sendSensorData() {
    static unsigned long lastReadTime = 0;
    const unsigned long readInterval = 1000; // 500ms

    if (millis() - lastReadTime >= readInterval) {
        int sensor1Val = sensor1.medir();
        delay(50);
        int sensor2Val = sensor2.medir();

        if (sensor1Val >= 0) webSocket.broadcastTXT("{\"sensor1\":" + String(sensor1Val) + "}");
        if (sensor2Val >= 0) webSocket.broadcastTXT("{\"sensor2\":" + String(sensor2Val) + "}");

        lastReadTime = millis();
    }
}

void WebControl::handleClient() {
    server.handleClient();
    webSocket.loop();
    sendSensorData();
}