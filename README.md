# Multi-Sensor IoT Tracker (Heltec WiFi LoRa 32 V3)

Este proyecto implementa un nodo LoRaWAN avanzado utilizando la placa **Heltec WiFi LoRa 32 V3**. Integra geolocalización por GPS, conteo de pasos mediante IMU de alta precisión, monitorización de pulso cardíaco y visualización de datos en tiempo real en una pantalla OLED.

![Heltec V3]([https://resource.heltec.cn/download/WiFi_LoRa_32_V3/HTIT-WB32_V3.png](https://heltec.org/wp-content/uploads/2023/09/2.png))

## 🚀 Características Principales

* **Conectividad LoRaWAN**: Configurado para la región **EU868** (Europa) con soporte para activación OTAA (Over-The-Air Activation).
* **Rastreo GPS**: Obtención de coordenadas (latitud, longitud) y número de satélites visibles mediante el módulo **NEO-M8N**.
* **Contador de Pasos**: Implementado con el sensor **BNO085** utilizando el algoritmo de fusión de sensores integrado (SH2) para una alta precisión.
* **Monitor Cardíaco**: Lectura analógica en tiempo real de pulsaciones mediante un sensor de pulso óptico.
* **Interfaz OLED**: Visualización local del estado de la red, recuento de pasos, coordenadas GPS y pulso.

## 🛠️ Hardware y Conexiones (Pinout)

La placa base es un **Heltec WiFi LoRa 32 V3** (ESP32-S3 + SX1262). A continuación se detalla el mapa de conexiones para los sensores externos:

| Dispositivo | Pin del Sensor | Pin Heltec V3 | Función / Protocolo |
| :--- | :--- | :--- | :--- |
| **GPS NEO-M8N** | VCC | 3V3 | Alimentación |
| | GND | GND | Tierra |
| | TX | **GPIO 19** | UART RX (ESP32) |
| | RX | **GPIO 20** | UART TX (ESP32) |
| **BNO085 IMU** | VCC | 3V3 | Alimentación |
| | GND | GND | Tierra |
| | SDA | **GPIO 47** | I2C SDA (Bus 1) |
| | SCL | **GPIO 48** | I2C SCL (Bus 1) |
| **Pulse Sensor** | VCC (+) | 3V3 | Alimentación |
| | GND (-) | GND | Tierra |
| | Signal (S) | **GPIO 1** | Entrada Analógica (ADC1_CH0) |

> **Nota:** La pantalla OLED está integrada en la placa y utiliza los pines `GPIO 17` (SDA) y `GPIO 18` (SCL) en el bus I2C interno.

## 📦 Estructura del Payload LoRaWAN

El dispositivo envía un paquete de datos hexadecimal de **18 bytes** con la siguiente estructura para optimizar el tiempo en aire:

| Byte(s) | Dato | Tipo | Descripción |
| :--- | :--- | :--- | :--- |
| **0-3** | Pasos Totales | `uint32` | Contador acumulativo histórico. |
| **4-5** | Pasos Nuevos | `uint16` | Pasos realizados desde el último paquete enviado. |
| **6-9** | Latitud | `int32` | Coordenada multiplicada por 1,000,000. |
| **10-13** | Longitud | `int32` | Coordenada multiplicada por 1,000,000. |
| **14** | Satélites | `uint8` | Número de satélites GPS visibles. |
| **15** | GPS Fix | `uint8` | `1` si hay señal válida, `0` si no. |
| **16-17** | Pulso | `uint16` | Valor analógico crudo del sensor (0-4095). |

### Payload Formatter (Javascript para TTN)
Puedes usar este código en la consola de The Things Network para decodificar los datos:

```javascript
function Decoder(bytes, port) {
  var decoded = {};
  
  // Pasos
  decoded.steps_total = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
  decoded.steps_new = (bytes[4] << 8) | bytes[5];
  
  // GPS
  decoded.latitude = ((bytes[6] << 24) | (bytes[7] << 16) | (bytes[8] << 8) | bytes[9]) / 1000000;
  decoded.longitude = ((bytes[10] << 24) | (bytes[11] << 16) | (bytes[12] << 8) | bytes[13]) / 1000000;
  decoded.satellites = bytes[14];
  decoded.gps_fixed = bytes[15] ? true : false;
  
  // Pulso
  decoded.pulse_raw = (bytes[16] << 8) | bytes[17];
  
  return decoded;
}
