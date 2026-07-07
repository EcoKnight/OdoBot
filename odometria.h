#ifndef ODOMETRIA_H
#define ODOMETRIA_H

#include <Arduino.h>
#include <math.h>
 
// CONSTANTES FÍSICAS DEL ROBOT 
const float R_RUEDA         = 0.034025;   // Radio de la rueda en metros 
const float L_EJE           = 0.22;       // Distancia entre ruedas en metros 
const float TICKS_POR_REV   = 2472.8;     // Ticks del encoder por vuelta completa de rueda

// Conversión de Ticks/s a metros/s
const float TICKS_A_METROS = (2.0 * PI * R_RUEDA) / TICKS_POR_REV;
 
// VECTOR DE ESTADO Y MATRICES DEL EKF           Estado: [0]=X (m), [1]=Y (m), [2]=Theta (rad)
extern float x_est[3]; 

// Matriz de Covarianza del Error P (3x3)
extern float P_est[3][3];

// Matrices de Ruido
const float Q_pos   = 0.001;   // Ruido de proceso en posición (X, Y)
const float Q_angle = 0.002;   // Ruido de proceso en el ángulo por encoders
const float R_imu   = 0.001;   // Ruido de medición de la IMU (entre menor sea, más confía en el Yaw)

// INICIALIZACIÓN DE ODOMETRÍA (Home en 90°)
inline void init_odometria() {
    x_est[0] = 0.0;                 // X inicial = 0 metros
    x_est[1] = 0.0;                 // Y inicial = 0 metros
    x_est[2] = 90.0 * (PI / 180.0); // Theta inicial = 90 grados (en radianes)

    // Inicializar P como una matriz identidad multiplicada por una incertidumbre inicial baja
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_est[i][j] = (i == j) ? 0.01 : 0.0;
        }
    }
}

// Normalizar ángulos entre -PI y PI 
inline float normalizar_angulo(float angulo) {
    while (angulo > PI)  angulo -= 2.0 * PI;
    while (angulo < -PI) angulo += 2.0 * PI;
    return angulo;
}
 
// CICLO DE FUSIÓN EKF (Predicción + Corrección)
inline void ejecutar_EKF(float vel_der_ticks, float vel_izq_ticks, float yaw_grados, float dt) {
    
    // --- 1. CONVERSIÓN DE CINEMÁTICA LINEAL (m/s) ---
    float v_R = vel_der_ticks * TICKS_A_METROS;
    float v_L = vel_izq_ticks * TICKS_A_METROS;
    
    float v     = (v_R + v_L) / 2.0;       // Velocidad lineal central (m/s)
    float w_enc = (v_R - v_L) / L_EJE;     // Velocidad angular teórica por encoders (rad/s)

    // Desplazamientos en el espacio local de este ciclo
    float delta_s     = v * dt;
    float delta_theta = w_enc * dt;

    // Guardar el ángulo anterior para el Jacobiano
    float theta_ant = x_est[2];

    // --- 2. ETAPA DE PREDICCIÓN (Modelo Cinemático Diferencial) ---
    x_est[0] = x_est[0] + delta_s * cos(theta_ant + delta_theta / 2.0);
    x_est[1] = x_est[1] + delta_s * sin(theta_ant + delta_theta / 2.0);
    x_est[2] = normalizar_angulo(x_est[2] + delta_theta);

    // Jacobiano de la función de transición Fx (3x3) respecto al estado anterior
    float Fx[3][3] = {
        {1.0, 0.0, -delta_s * sin(theta_ant + delta_theta / 2.0)},
        {0.0, 1.0,  delta_s * cos(theta_ant + delta_theta / 2.0)},
        {0.0, 0.0,  1.0}
    };

    // Propagación de la Covarianza: P_priori = Fx * P_anterior * Fx^T + Q
    float P_aux[3][3] = {0};
    // P_aux = Fx * P_est
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                P_aux[i][j] += Fx[i][k] * P_est[k][j];
            }
        }
    }
    // P_est = P_aux * Fx^T + Q
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_est[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                P_est[i][j] += P_aux[i][k] * Fx[j][k]; // Fx[j][k] es Fx^T[k][j]
            }
            // Sumar ruido de proceso Q en la diagonal
            if (i == j) {
                P_est[i][j] += (i == 2) ? Q_angle : Q_pos;
            }
        }
    }

    // --- 3. ETAPA DE CORRECCIÓN ---
    float z_imu = yaw_grados * (PI / 180.0);    // Medición
    
    // Innovación (Error entre lo medido por la IMU y lo predicho por el encoder)
    float innovacion = normalizar_angulo(z_imu - x_est[2]);

    // Covarianza de la innovación: S = H * P_priori * H^T + R
    // Como el vector de medición H es [0, 0, 1] (solo mide ángulo), H*P*H^T se reduce a P_est[2][2]
    float S = P_est[2][2] + R_imu;

    // Ganancia de Kalman: K = P_priori * H^T / S
    float K[3];
    K[0] = P_est[0][2] / S; // Corrección aplicable a X
    K[1] = P_est[1][2] / S; // Corrección aplicable a Y
    K[2] = P_est[2][2] / S; // Corrección aplicable a Theta

    // Actualización del Vector de Estado con la corrección calculada
    x_est[0] += K[0] * innovacion;
    x_est[1] += K[1] * innovacion;
    x_est[2] = normalizar_angulo(x_est[2] + K[2] * innovacion);

    // Actualización de la Covarianza del Error: P = (I - K * H) * P_priori
    for (int i = 0; i < 3; i++) {
        P_est[i][0] -= K[i] * P_est[2][0];
        P_est[i][1] -= K[i] * P_est[2][1];
        P_est[i][2] -= K[i] * P_est[2][2];
    }
}
#endif