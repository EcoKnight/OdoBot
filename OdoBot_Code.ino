#include <Arduino.h>
#include <Wire.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "rom/ets_sys.h"
#include "sensores.h"
#include "telemetria.h"
#include "odometria.h"

// Filtros para los datos del IMU y del US 
FiltroMediaMovil filtro_gyrX(10); 
FiltroMediaMovil filtro_gyrY(10); 
FiltroMediaMovil filtro_gyrZ(10);
FiltroEMA filtro_accX(0.2); 
FiltroEMA filtro_accY(0.2); 
FiltroEMA filtro_accZ(0.2);
FiltroEMA filtro_us(0.15);

// Calibración del IMU
MPU6500 IMU;
calData calib = { 0 };
AccelData accelData;
GyroData gyroData;

// Declarar valores base
float offset_aX = 0, offset_aY = 0, offset_aZ = 0;
float offset_gX = 0, offset_gY = 0, offset_gZ = 0;
float angulo_X = 0.0, angulo_Y = 0.0, angulo_Z = 0.0;
float roll = 0.0, pitch = 0.0, yaw = 0.0;
float distancia_cm = -1.0;

// Definición física del estado del EKF (Requerido por odometria.h)
float x_est[3] = {0.0, 0.0, 0.0};
float P_est[3][3] = {{0}};

// Instancias de Red (declaradas en telemetria.h)
WiFiServer server(tcp_port);
WiFiClient dashboard_client;

// PINES MOTOR DERECHO (enA - Out1 y 2)
#define ENA_PIN GPIO_NUM_25
#define IN1_PIN GPIO_NUM_33
#define IN2_PIN GPIO_NUM_32
#define C1_der 5
#define C2_der 17

// PINES MOTOR IZQUIERDO (enB - Out3 y 4)
#define ENB_PIN GPIO_NUM_14
#define IN3_PIN GPIO_NUM_26
#define IN4_PIN GPIO_NUM_27
#define C1_izq 15
#define C2_izq 2

// Pines del los sensores IMU (I2C)
#define SDA_PIN 21
#define SCL_PIN 22
#define LED_BUILTIN GPIO_NUM_2

// Signo de los encoders para incrementar o decrementar 
#define ENC_DER_SIGN -1
#define ENC_IZQ_SIGN -1

// VARIABLES GLOBALES CONTROL
volatile long ticks_der = 0;
volatile long ticks_izq = 0;

float vel_der_filtrada = 0;
float vel_izq_filtrada = 0;
const float alpha = 0.3; 

// ESTRUCTURA CONTROL PID 
struct PID {
    float kp, ki, kd;
    float error_anterior, integral;
    float integral_max;

    PID(float p, float i, float d, float imax = 5000)
        : kp(p), ki(i), kd(d), error_anterior(0), integral(0), integral_max(imax) {}

    float calcular(float setpoint, float medido, float dt) {
        float error = setpoint - medido;
        float umbral_integracion = (setpoint > 1000.0) ? (setpoint * 0.20) : 15.0;

        if (abs(error) < umbral_integracion) {
            integral += error * dt;
            integral = constrain(integral, -integral_max, integral_max);
        }
        
        float derivada = (dt > 0) ? (error - error_anterior) / dt : 0;
        error_anterior = error;
        return kp * error + ki * integral + kd * derivada;
    }
    void reset() { error_anterior = 0; integral = 0; }
};

// Declarar los valores de los controles (Kp, Ki, Kd, Correción máx.)
PID pid_motor_der(0.25, 0.4, 0.01, 10000); 
PID pid_motor_izq(0.3167, 0.4, 0.01, 10000); 
PID pd_heading(120.0, 3.5, 12.0); 

int pwm_base = 100; 

// ISRs (Interrupciones de Encoders)
void IRAM_ATTR isr_C1_der() {
    if (gpio_get_level((gpio_num_t)C1_der) == gpio_get_level((gpio_num_t)C2_der))
        ticks_der++; else ticks_der--;
}
void IRAM_ATTR isr_C2_der() {
    if (gpio_get_level((gpio_num_t)C1_der) != gpio_get_level((gpio_num_t)C2_der))
        ticks_der++; else ticks_der--;
}
void IRAM_ATTR isr_C1_izq() {
    if (gpio_get_level((gpio_num_t)C1_izq) == gpio_get_level((gpio_num_t)C2_izq))
        ticks_izq++; else ticks_izq--;
}
void IRAM_ATTR isr_C2_izq() {
    if (gpio_get_level((gpio_num_t)C1_izq) != gpio_get_level((gpio_num_t)C2_izq))
        ticks_izq++; else ticks_izq--;
}

void delay_ms(uint32_t ms) { ets_delay_us(ms * 1000); }

// CONFIGURACIÓN MOTORES
void init_motors() {
    gpio_reset_pin(IN1_PIN); gpio_set_direction(IN1_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN2_PIN); gpio_set_direction(IN2_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN3_PIN); gpio_set_direction(IN3_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(IN4_PIN); gpio_set_direction(IN4_PIN, GPIO_MODE_OUTPUT);

    ledc_timer_config_t timer = {};
    timer.speed_mode      = LEDC_LOW_SPEED_MODE;
    timer.timer_num       = LEDC_TIMER_0;
    timer.duty_resolution = LEDC_TIMER_8_BIT;
    timer.freq_hz         = 1000;
    timer.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&timer);

    ledc_channel_config_t ch1 = {};
    ch1.gpio_num   = ENA_PIN; ch1.speed_mode = LEDC_LOW_SPEED_MODE;
    ch1.channel    = LEDC_CHANNEL_0; ch1.timer_sel = LEDC_TIMER_0;
    ch1.duty = 0; ch1.hpoint = 0;
    ledc_channel_config(&ch1);

    ledc_channel_config_t ch2 = {};
    ch2.gpio_num   = ENB_PIN; ch2.speed_mode = LEDC_LOW_SPEED_MODE;
    ch2.channel    = LEDC_CHANNEL_1; ch2.timer_sel = LEDC_TIMER_0;
    ch2.duty = 0; ch2.hpoint = 0;
    ledc_channel_config(&ch2);
}

void move_motors(int speed_der, int dir_der, int speed_izq, int dir_izq) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed_der);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed_izq);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    if (dir_der == 1)       { gpio_set_level(IN1_PIN,1); gpio_set_level(IN2_PIN,0); }
    else if (dir_der == -1) { gpio_set_level(IN1_PIN,0); gpio_set_level(IN2_PIN,1); }
    else                    { gpio_set_level(IN1_PIN,0); gpio_set_level(IN2_PIN,0); }

    if (dir_izq == 1)       { gpio_set_level(IN3_PIN,1); gpio_set_level(IN4_PIN,0); }
    else if (dir_izq == -1) { gpio_set_level(IN3_PIN,0); gpio_set_level(IN4_PIN,1); }
    else                    { gpio_set_level(IN3_PIN,0); gpio_set_level(IN4_PIN,0); }
}

// LAZO DE CONTROL PID CON ENCODER E IMU

void prueba_pd(float target_setpoint, float target_angle, unsigned long duracion_ms) {
    bool filtro_inicializado = false;
    unsigned long t_anterior = millis();
    long ticks_der_anterior = ticks_der;
    long ticks_izq_anterior = ticks_izq;

    unsigned long start = millis();
    while (millis() - start < duracion_ms) {
        unsigned long ahora = millis();
        float dt = (ahora - t_anterior) / 1000.0;
        if (dt < 0.02) continue; // 50 Hz

        // --- 1. LECTURA DE ENCODERS ---
        long t_der = ticks_der, t_izq = ticks_izq;
        float vel_der_raw = ENC_DER_SIGN * (t_der - ticks_der_anterior) / dt;
        float vel_izq_raw = ENC_IZQ_SIGN * (t_izq - ticks_izq_anterior) / dt;
        ticks_der_anterior = t_der;
        ticks_izq_anterior = t_izq;

        // --- 2. FILTRADO EMA DE VELOCIDADES ---
        if (!filtro_inicializado) {
            vel_der_filtrada = vel_der_raw;
            vel_izq_filtrada = vel_izq_raw;
            filtro_inicializado = true;
        } else {
            vel_der_filtrada = alpha * vel_der_raw + (1.0 - alpha) * vel_der_filtrada;
            vel_izq_filtrada = alpha * vel_izq_raw + (1.0 - alpha) * vel_izq_filtrada;
        }

        // --- 3. ACTUALIZACIÓN DE SENSORES ---
        actualizar_sensores(dt);
        
        // --- EJECUCIÓN DEL FILTRO DE KALMAN EXTENDIDO ---
        ejecutar_EKF(vel_der_filtrada, vel_izq_filtrada, yaw, dt);

        // --- 4. CONTROL DE ORIENTACIÓN CON ÁNGULO OBJETIVO Y ZONA MUERTA ---
        float error_angulo = target_angle - yaw;
        float corr_heading = 0.0;
        
        if (abs(error_angulo) > 1.0) {
            corr_heading = pd_heading.calcular(target_angle, yaw, dt);
        } else {
            pd_heading.reset(); 
            corr_heading = 0.0;
        }

        // --- 5. COMPENSACIÓN MECÁNICA BASE (FeedForward) ---
        float target_der_base = target_setpoint * 0.9603;  // Se frena matemáticamente al motor derecho por ser el más fuerte
        float target_izq_base = target_setpoint;

        // --- 6. APLICACIÓN DE CORRECCIÓN DE RUMBO ---
        float setpoint_m_der = target_der_base + corr_heading; 
        float setpoint_m_izq = target_izq_base - corr_heading; 

        // --- 7. CÁLCULO DE PIDs DE VELOCIDAD INDIVIDUALES ---
        float corr_der = pid_motor_der.calcular(setpoint_m_der, vel_der_filtrada, dt);
        float corr_izq = pid_motor_izq.calcular(setpoint_m_izq, vel_izq_filtrada, dt);

        // --- 8. SATURACIÓN Y SALIDA DE ACTUADORES ---
        int pwm_der = constrain((int)(pwm_base + corr_der), 0, 255);
        int pwm_izq = constrain((int)(pwm_base + corr_izq), 0, 255);
        
        move_motors(pwm_der, 1, pwm_izq, 1);

        // --- 9. TELEMETRÍA SERIE ---
        Serial.print(target_setpoint);         Serial.print(" ");
        Serial.print(vel_der_filtrada);        Serial.print(" ");
        Serial.print(vel_izq_filtrada);        Serial.print(" ");
        Serial.print(yaw);                     Serial.print(" "); 
        Serial.print(pwm_der);                 Serial.print(" ");
        Serial.print(pwm_izq);                 Serial.print(" ");
        if(distancia_cm > 0) { Serial.println(distancia_cm); } else { Serial.println(0); }

        // --- 10. TELEMETRÍA INALÁMBRICA TCP (Enviando Coordenadas del EKF) ---
        enviar_telemetria(x_est[0], x_est[1], vel_der_filtrada, vel_izq_filtrada, pwm_der, pwm_izq, yaw);

        t_anterior = ahora;
    }
    
    move_motors(0, 0, 0, 0);
    Serial.println("\n--- Prueba PID terminada. Robot detenido. ---");
    if (dashboard_client && dashboard_client.connected()) {
        dashboard_client.println("END"); 
    }
}

// PROCESAMIENTO DE COMANDOS ACTUALIZADO
void procesar_comando(String cmd) {
    cmd.trim(); 
    if (cmd.length() == 0) return;

    // --- COMANDO R: TARA / RESET DE SISTEMA DE REFERENCIA A 90° ---
    if (cmd.startsWith("R") || cmd.startsWith("r")) {
        angulo_X = 0.0;
        angulo_Y = 0.0;
        angulo_Z = 90.0 * (PI / 180.0); // Home en 90° (convertido a radianes)
        roll = 0.0;
        pitch = 0.0;
        yaw = 90.0;                     // Home en 90° absolutos
        ticks_der = 0;
        ticks_izq = 0;

        init_odometria(); // Resetea las coordenadas X, Y a 0.0 y la Matriz P

        for(int i=0; i<3; i++) {
            digitalWrite(LED_BUILTIN, HIGH); delay_ms(100);
            digitalWrite(LED_BUILTIN, LOW);  delay_ms(100);
        }
        digitalWrite(LED_BUILTIN, HIGH); 

        Serial.println("\n[SISTEMA DE REFERENCIA CONFIGURADO]");
        Serial.println("El frente del robot se ha establecido en su Home de 90.0°");
        
        if (dashboard_client && dashboard_client.connected()) {
            dashboard_client.println("SISTEMA DE REFERENCIA OK\n");
        }
        return;
    }

    // --- COMANDO P: MARCHA ---
    if (cmd.startsWith("P") || cmd.startsWith("p")) {
        int c1 = cmd.indexOf(',');
        int c2 = cmd.indexOf(',', c1 + 1);
        int c3 = cmd.indexOf(',', c2 + 1);
        
        if (c1 > 0 && c2 > 0 && c3 > 0) {
            float setpoint_ingresado = cmd.substring(c1 + 1, c2).toFloat();
            float angulo_ingresado   = cmd.substring(c2 + 1, c3).toFloat();
            float duracion_seg       = cmd.substring(c3 + 1).toFloat();
            
            Serial.print("\n[CONFIG] SP_Vel: "); Serial.print(setpoint_ingresado);
            Serial.print(" | Angulo Obj: "); Serial.print(angulo_ingresado);
            Serial.print("° | Tiempo: "); Serial.print(duracion_seg); Serial.println("s");
            
            digitalWrite(LED_BUILTIN, LOW); 
            delay_ms(2000); 

            ticks_der = 0; 
            ticks_izq = 0;
            pid_motor_der.reset(); 
            pid_motor_izq.reset(); 
            pd_heading.reset();

            prueba_pd(setpoint_ingresado, angulo_ingresado, (unsigned long)(duracion_seg * 1000.0)); 
            
            digitalWrite(LED_BUILTIN, HIGH); 
            Serial.println("\n--- CHROME LISTO ---");
        } else {
            Serial.println("Formato incorrecto. Usa: P,velocidad,angulo,duracion (Ej: P,5000,90,10)");
        }
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    init_motors();

    pinMode(C1_der, INPUT_PULLUP); pinMode(C2_der, INPUT_PULLUP);
    pinMode(C1_izq, INPUT_PULLUP); pinMode(C2_izq, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(C1_der), isr_C1_der, CHANGE);
    attachInterrupt(digitalPinToInterrupt(C2_der), isr_C2_der, CHANGE);
    attachInterrupt(digitalPinToInterrupt(C1_izq), isr_C1_izq, CHANGE);
    attachInterrupt(digitalPinToInterrupt(C2_izq), isr_C2_izq, CHANGE);

    init_sensores();
    
    // Establecer HOME Inicial
    angulo_Z = 90.0 * (PI / 180.0); 
    yaw = 90.0;                     
    init_odometria(); 
    
    init_telemetria();
}

void loop() {
    static bool menu_impreso = false;
    verificar_conexion_dashboard();

    if (!menu_impreso) {
        Serial.println("\n--- CHROME LISTO ---");
        Serial.println("Esperando comando desde Python con formato: P,velocidad,angulo,duracion");
        menu_impreso = true;
    }

    if (dashboard_client && dashboard_client.available() > 0) {
        String comando_wifi = dashboard_client.readStringUntil('\n');
        procesar_comando(comando_wifi);
    }

    if (Serial.available() > 0) {
        String comando_serial = Serial.readStringUntil('\n');
        procesar_comando(comando_serial);
    }

    delay_ms(10); 
}