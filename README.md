# Smart Helmet - Sistema de Monitoreo para Minería Subterránea

Este repositorio contiene la documentación técnica y el código fuente del firmware para el proyecto "Smart Helmet", un sistema de monitoreo portátil con telemetría LoRa diseñado para mejorar la seguridad en minería subterránea.

## 📋 Contenido del Repositorio

*   **Firmware del Smart Helmet:** Código principal para el microcontrolador ESP32-S3 que gestiona los sensores (MQ7, MPU6050, BMP280/AHT20, VL53L0X) y la comunicación LoRa (SX1278).
*   **Lógica del Gateway:** Código para la estación base que recibe y procesa la telemetría.
*   **Máquina de Estados (FSM):** Documentación y código del algoritmo de detección de caídas e impactos.

## 🛠️ Stack Tecnológico

*   **Hardware:** ESP32-S3 N16R8, Módulo LoRa SX1278, Sensores MQ7, MPU6050, BMP280/AHT20, VL53L0X.
*   **Software:** C/C++, Framework Arduino, PlatformIO, ESP-IDF.

## 🚀 Cómo Empezar

Para replicar este proyecto, necesitas tener instalado PlatformIO (como extensión de VS Code) o el IDE de Arduino.

1.  **Clonar el repositorio:**
    ```bash
    git clone https://github.com/[TU-USUARIO]/[NOMBRE-DEL-REPO].git
