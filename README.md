# Proyecto Chrome 🤖 | Robot Móvil Diferencial con ESP32 y EKF

![ESP32](https://img.shields.io/badge/ESP32-System_on_Chip-blue)
![C++](https://img.shields.io/badge/C++-Arduino_Core-00599C)
![Python](https://img.shields.io/badge/Python-3.x-FFD43B)
![Status](https://img.shields.io/badge/Status-Prototipo_Funcional-success)

Chrome es un robot móvil de tracción diferencial diseñado para pruebas avanzadas de control y navegación autónoma. El sistema integra control PID de bajo nivel, odometría diferencial y un **Filtro de Kalman Extendido (EKF)** ejecutado en tiempo real para la fusión sensorial (IMU + Encoders). Todo esto es monitoreado a través de una arquitectura TCP inalámbrica mediante un dashboard en Python.

[Insertar imagen o GIF del robot Chrome en funcionamiento aquí]

## 🚀 Características Principales

* **Fusión Sensorial (EKF):** Implementación de un Filtro de Kalman Extendido a 50 Hz en la ESP32 para estimar la posición $(X, Y, \theta)$ mitigando el error acumulativo de la odometría pura mediante lecturas absolutas del giroscopio.
* **Controladores PID Independientes:** Lazos de control simultáneos para la velocidad individual de cada motor y para la estabilización activa del rumbo (Heading).
* **Cinemática Inversa:** Capacidad de recibir comandos de desplazamiento en coordenadas cartesianas globales (metros en X e Y) calculando automáticamente trayectorias y tiempos de ejecución.
* **Telemetría Inalámbrica (SoftAP):** Comunicación TCP Socket punto a punto. La ESP32 actúa como un Access Point seguro aislando el tráfico para transmitir datos de telemetría a una tasa estricta de 50 Hz sin latencia de redes externas.
* **Adquisición de Datos Automatizada:** Backend en Python con un menú de máquina de estados que gestiona pruebas de control dinámico, trayectoria cuadrada (error de cierre) y registro directo a archivos `.xlsx` mediante `pandas`.

## 🧰 Arquitectura de Hardware

* **Microcontrolador:** ESP32 (Procesamiento concurrente con FreeRTOS).
* **Sensores Inerciales:** MPU6500 (Acelerómetro y Giroscopio) filtrado mediante medias móviles y EMA.
* **Actuadores:** Motores DC con caja reductora y encoders magnéticos en cuadratura (Resolución: 2472.8 PPR).
* **Driver de Potencia:** Puente H doble canal.
* **Parámetros Cinemáticos:**
  * Radio de la rueda ($r$): 34.025 mm
  * Distancia entre ruedas / Ancho de vía ($L$): 220 mm

## ⚙️ Estructura del Software

El código fuente está modularizado bajo los estándares de C++ para sistemas embebidos:

* `Pruebas_EKF.ino`: Lazo principal, lectura de interrupciones (ISRs) y control PID de actuadores.
* `sensores.h`: Adquisición de la IMU (FastIMU), aplicación de filtros y cálculo de orientación (Yaw).
* `odometria.h`: Matemáticas del modelo diferencial, propagación del Jacobiano y matrices de covarianza del EKF.
* `telemetria.h`: Configuración del servidor TCP y encapsulamiento de datos.
* `dashboard.py`: Backend en Python para el envío de comandos (ASCII), parseo de tramas y exportación a Excel.

## 📡 Protocolo de Comunicación

El flujo de comandos se maneja a través del puerto TCP `8888`.

**Comandos aceptados (Python -> ESP32):**
* `P,velocidad,angulo,duracion\n`: Ejecuta una trayectoria a velocidad constante hacia un rumbo absoluto en grados durante los segundos indicados.
* `R\n`: Resetea el marco de referencia inercial estableciendo la posición actual como origen $(0,0)$ y el frente del robot como el *Home* ($90^\circ$).

**Trama de Telemetría (ESP32 -> Python):**
* Formato continuo a 50 Hz: `X,Y,Vel_Der,Vel_Izq,PWM_Der,PWM_Izq,Yaw\n`

## 🛠️ Instalación y Uso

### Prerrequisitos
1. Instalar el [IDE de Arduino](https://www.arduino.cc/en/software) y el core de ESP32.
2. Instalar dependencias de Python:
   ```bash
   pip install pandas openpyxl
