#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>

#include "MPU6050.hpp"
#include "RobotServer.hpp"


RobotServer::RobotServer():PORT(80), server(PORT), ws("/ws") {
    ws.onEvent(wsHandler.eventHandler());
}

void RobotServer::connect_to_wifi() {
    WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);
    
    unsigned long start_attempt_time = millis();

    Serial.print("\nConnecting to wifi");
    while(WiFi.status() != WL_CONNECTED && (millis() - start_attempt_time < WIFI_TIMEOUT_MS)) {
        Serial.print(".");
        delay(50);
    }

    if(WiFi.status() != WL_CONNECTED) {
        Serial.print("\nFailed!");
        // Take Action after failed
    } else {
        Serial.print("\nConnected: ");
        Serial.print(WiFi.localIP());
    }
}

void RobotServer::configure() {
    /* CORS */
    String allowed_origins[] = {"http://127.0.0.1:5500", "http://" + WiFi.localIP().toString()};
    cors.setOrigin(allowed_origins->c_str());
    server.addMiddleware(&cors);

    connect_to_wifi();
    setup_routes();
    setup_socket();
    server.begin();
}

void RobotServer::setup_routes() {
    /* GET / - Index route */
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/html", htmlContent);
    });

    /* GET /api/getPIDConstants - PID constants api route */
    server.on("/api/getPIDConstants", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncJsonResponse *response = new AsyncJsonResponse();
        JsonObject root = response->getRoot().to<JsonObject>();
        root["kp"] = kp;
        root["ki"] = ki;
        root["kd"] = kd;
        response->setLength();
        request->send(response);
    });

    /* GET /api/updatePIDConstants - PID constants api route */
    server.on("/api/updatePIDConstants", HTTP_GET, [](AsyncWebServerRequest *request) {
        if(request->hasParam("kp")) {
            kp = request->getParam("kp")->value().toFloat();
            Serial.print("\nkp updated: ");
            Serial.print(kp);
        }
        if(request->hasParam("ki")) {
            ki = request->getParam("ki")->value().toFloat();
            Serial.print("\nki updated: ");
            Serial.print(ki);
        }
        if(request->hasParam("kd")) {
            kd = request->getParam("kd")->value().toFloat();
            Serial.print("\nkd updated: ");
            Serial.print(kd);
        }
        request->send(200, "text/plain", "ok!");
    });
}

void RobotServer::setup_socket() {

    wsHandler.onConnect([](AsyncWebSocket *s, AsyncWebSocketClient *c) {
        Serial.printf("\nWeb socket client %" PRIu32 " connected", c->id());
    });

    wsHandler.onDisconnect([](AsyncWebSocket *s, uint32_t clientId) {
        Serial.printf("\nWeb socket client %" PRIu32 " disconnected", clientId);
    });

    wsHandler.onError([](AsyncWebSocket *s, AsyncWebSocketClient *c, uint16_t errorCode, const char *reason, size_t len) {
        Serial.printf("\nWeb socket client %" PRIu32 " error: %" PRIu16 ": %s\n", c->id(), errorCode, reason);
    });

    server.addHandler(&ws);
}

void RobotServer::send_data(JsonDocument& doc, String& json) {
    serializeJson(doc, json);
    if(ws.count() > 0) {
        ws.textAll(json);
    }
}