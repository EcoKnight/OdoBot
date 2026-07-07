#ifndef SENSORES_H
#define SENSORES_H

#include <Arduino.h>
#include <Wire.h>
#include "FastIMU.h"
#include <math.h>

// FACTOR DE CORRECCIÓN GYRO (Falla de escala de FastIMU)
const float GYRO_SCALE_CORRECTION = 1.0; 

// FILTRO DE MEDIA MÓVIL (FIR)
struct FiltroMediaMovil {
    int num_taps;
    float* buffer;
    int indice;
    float suma;
    int muestras_actuales;

    FiltroMediaMovil(int taps) {
        num_taps = taps;
        buffer = new float[taps];
        for (int i = 0; i < taps; i++) buffer[i] = 0.0;
        indice = 0;
        suma = 0.0;
        muestras_actuales = 0;
    }

    float filtrar(float nuevo_valor) {
        suma -= buffer[indice];
        buffer[indice] = nuevo_valor;
        suma += buffer[indice];
        indice = (indice + 1) % num_taps;
        if (muestras_actuales < num_taps) muestras_actuales++;
        return suma / muestras_actuales;
    }
};

// FILTRO EMA
struct FiltroEMA {
    float alpha, valor;
    bool primera;
    FiltroEMA(float a) : alpha(a), valor(0), primera(true) {}
    float filtrar(float lectura) {
        if (primera) { valor = lectura; primera = false; }
        else { valor = (alpha * lectura) + ((1.0 - alpha) * valor); }
        return valor;
    }
};

// INSTANCIAS DE FILTROS — IMU Y ULTRASONIDO
extern FiltroMediaMovil filtro_gyrX;
extern FiltroMediaMovil filtro_gyrY;
extern FiltroMediaMovil filtro_gyrZ;
extern FiltroEMA filtro_accX;
extern FiltroEMA filtro_accY;
extern FiltroEMA filtro_accZ;
extern FiltroEMA filtro_us;

// PINES Y CONSTANTES ULTRASONIDO
const int TRIGGER_PIN = 18;
const int ECHO_PIN    = 19;
const unsigned long US_TIMEOUT_US  = 25000UL;
const float         US_MAX_CM      = 400.0;
const float         US_FACTOR      = 0.01715;

// CONFIGURACIÓN IMU (FastIMU)
const int IMU_ADDRESS = 0x68;
extern MPU6500 IMU;
extern calData calib;
extern AccelData accelData;
extern GyroData gyroData;

// OFFSETS DE CALIBRACIÓN
extern float offset_aX, offset_aY, offset_aZ;
extern float offset_gX, offset_gY, offset_gZ;
const int          NUM_MUESTRAS_CALIB = 500;
const unsigned int DELAY_CALIB_MS     = 10;

// ÁNGULOS ACUMULADOS
extern float angulo_X;
extern float angulo_Y;
extern float angulo_Z;
extern float roll;
extern float pitch;
extern float yaw;
extern float distancia_cm;
const float ALPHA_COMP = 0.98;

// FUNCIONES DE SENSORES
inline float aplicar_banda_muerta(float valor, float umbral) {
    if (abs(valor) < umbral) return 0.0;
    return valor;
}

inline float leer_ultrasonido() {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    unsigned long duracion = pulseIn(ECHO_PIN, HIGH, US_TIMEOUT_US);
    if (duracion == 0) return -1.0;

    float distancia = duracion * US_FACTOR;
    if (distancia > US_MAX_CM) return -1.0;

    return distancia;
}

inline void calibrar_IMU() {
    Serial.println("============================================");
    Serial.println("   CALIBRACIÓN IMU — No muevas el sensor...");
    Serial.println("============================================");

    for (int i = 3; i > 0; i--) {
        Serial.print("   Iniciando en "); Serial.print(i); Serial.println("...");
        delay(1000);
    }
    Serial.println("   Tomando muestras...");

    double suma_aX = 0, suma_aY = 0, suma_aZ = 0;
    double suma_gX = 0, suma_gY = 0, suma_gZ = 0;

    for (int i = 0; i < NUM_MUESTRAS_CALIB; i++) {
        IMU.update();
        IMU.getAccel(&accelData);
        IMU.getGyro(&gyroData);

        suma_aX += accelData.accelX;
        suma_aY += accelData.accelY;
        suma_aZ += accelData.accelZ;
        suma_gX += gyroData.gyroX;
        suma_gY += gyroData.gyroY;
        suma_gZ += gyroData.gyroZ;

        delay(DELAY_CALIB_MS);
    }

    offset_aX = suma_aX / NUM_MUESTRAS_CALIB;
    offset_aY = suma_aY / NUM_MUESTRAS_CALIB;
    
    // Auto-detectar si la librería devuelve Gs (1.0) o m/s2 (9.81) para el offset en Z
    float acc_Z_promedio = suma_aZ / NUM_MUESTRAS_CALIB;
    if (acc_Z_promedio > 5.0) { offset_aZ = acc_Z_promedio - 9.81; } 
    else { offset_aZ = acc_Z_promedio - 1.0; }

    offset_gX = suma_gX / NUM_MUESTRAS_CALIB;
    offset_gY = suma_gY / NUM_MUESTRAS_CALIB;
    offset_gZ = suma_gZ / NUM_MUESTRAS_CALIB;

    Serial.println("   Calibración completa.");
    Serial.println("============================================");
}

inline void init_sensores() {
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    Serial.println("Inicializando MPU6500 (FastIMU)...");
    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0) {
        Serial.print("Error IMU: "); Serial.println(err);
        while (true) delay(10);
    }
    
    IMU.setGyroRange(500);  // ±500 DPS
    IMU.setAccelRange(2);   // ±2g
    
    Serial.println("MPU6500 OK. Rangos ajustados.");
    delay(500);
    calibrar_IMU();
}

inline void actualizar_sensores(float dt) {
    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);

    // Filtros Accel
    float aX_f = filtro_accX.filtrar(accelData.accelX - offset_aX);
    float aY_f = filtro_accY.filtrar(accelData.accelY - offset_aY);
    float aZ_f = filtro_accZ.filtrar(accelData.accelZ - offset_aZ);

    // Filtros Gyro (Con el factor de corrección por si FastIMU sigue entregando datos inflados)
    float gX_f = filtro_gyrX.filtrar(gyroData.gyroX - offset_gX) * GYRO_SCALE_CORRECTION;
    float gY_f = filtro_gyrY.filtrar(gyroData.gyroY - offset_gY) * GYRO_SCALE_CORRECTION;
    float gZ_f = filtro_gyrZ.filtrar(gyroData.gyroZ - offset_gZ) * GYRO_SCALE_CORRECTION;

    // Banda muerta
    gX_f = aplicar_banda_muerta(gX_f, 1.5);
    gY_f = aplicar_banda_muerta(gY_f, 1.5);
    gZ_f = aplicar_banda_muerta(gZ_f, 1.5);

    // dps a Rad/s
    float gX_Rad = gX_f * (PI / 180.0);
    float gY_Rad = gY_f * (PI / 180.0);
    float gZ_Rad = gZ_f * (PI / 180.0);

    // --- BLINDAJE ACELERÓMETRO ---
    float acc_angulo_X = atan2(aY_f, aZ_f);
    float acc_angulo_Y = atan2(-aX_f, sqrt(aY_f * aY_f + aZ_f * aZ_f));

    // Filtro complementario
    angulo_X = ALPHA_COMP * (angulo_X + gX_Rad * dt) + (1.0 - ALPHA_COMP) * acc_angulo_X;
    angulo_Y = ALPHA_COMP * (angulo_Y + gY_Rad * dt) + (1.0 - ALPHA_COMP) * acc_angulo_Y;
    
    // Yaw por integración pura
    angulo_Z += gZ_Rad * dt; 

    // Conversión final a grados
    roll  = angulo_X * (180.0 / PI);
    pitch = angulo_Y * (180.0 / PI);
    yaw   = angulo_Z * (180.0 / PI);

    // Lectura Ultrasonido
    float distancia_raw = leer_ultrasonido();
    if (distancia_raw > 0) {
        distancia_cm = filtro_us.filtrar(distancia_raw);
    } else {
        distancia_cm = -1.0;
    }
}
#endif