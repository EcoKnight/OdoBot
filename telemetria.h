#ifndef TELEMETRIA_H
#define TELEMETRIA_H

#include <Arduino.h>
#include <WiFi.h>

// Datos del HotSpot
const char* ap_ssid = "Chrome_Robot";
const char* ap_password = "chrome1234"; 
const uint16_t tcp_port = 8888;
extern WiFiServer server;
extern WiFiClient dashboard_client;

inline void init_telemetria() {
    Serial.println("\n--- Configurando Hotspot WiFi ---");
    
    // Configura la ESP32 en modo Access Point
    WiFi.softAP(ap_ssid, ap_password);
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("Hotspot Activo. SSID: ");
    Serial.println(ap_ssid);
    Serial.print("IP del Robot (Servidor): ");
    Serial.println(IP); // Por defecto suele ser 192.168.4.1
    
    // Inicia el servidor TCP en el puerto especificado
    server.begin();
    Serial.print("Servidor TCP escuchando en el puerto: ");
    Serial.println(tcp_port);
}

inline void verificar_conexion_dashboard() {
    // Si no hay un cliente de Python conectado, revisa si hay alguna solicitud entrante
    if (!dashboard_client || !dashboard_client.connected()) {
        dashboard_client = server.available();
        if (dashboard_client) {
            Serial.println("\n[WiFi] ¡Dashboard de Python conectado exitosamente!");
        }
    }
}

/*inline void enviar_telemetria(float setpoint, float vel_der, float vel_izq, float yaw_val, int pwm_der, int pwm_izq, float dist) {
    // Solo transmite si Python está conectado y escuchando
    if (dashboard_client && dashboard_client.connected()) {
        // Enviamos los datos en una sola línea formateada en CSV terminando con '\n'
        dashboard_client.printf("%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f\n", 
                                setpoint, vel_der, vel_izq, yaw_val, pwm_der, pwm_izq, dist);
    }
}*/
inline void enviar_telemetria(float pos_x, float pos_y, float vel_der, float vel_izq, int pwm_der, int pwm_izq, float yaw_val) {
    if (dashboard_client && dashboard_client.connected()) {
        dashboard_client.printf("%.4f,%.4f,%.2f,%.2f,%d,%d,%.2f\n", 
                                pos_x, pos_y, vel_der, vel_izq, pwm_der, pwm_izq, yaw_val);
    }
}
#endif