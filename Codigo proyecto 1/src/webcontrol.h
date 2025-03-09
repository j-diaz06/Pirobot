#ifndef WEBCONTROL_H
#define WEBCONTROL_H

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include "motor.h"
#include "sensor.h"
#include "maquina.h"

class WebControl {
private:
    WebServer server;
    WebSocketsServer webSocket;
    Motor& motores;
    Maquina& maquina; // Referencia a Maquina
    unsigned long lastUpdate = 0;
    //Sensor& sensor1;
    //Sensor& sensor2;
    //unsigned long lastSensor1Update = 0;
    //unsigned long lastSensor2Update = 0;
    //bool alternateSensor = false;
    
    void handleRoot();
    void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
    void sendSensorData();

public:
    WebControl(Motor& m, Maquina& maq);
    void begin(const char* ssid, const char* password, const char* hostname);
    void handleClient();
};

#endif