#include "LoRaWan_APP.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "HT_SSD1306Wire.h"
#include "HT_TinyGPS++.h"

/* * CÓDIGO COMPLETO PARA HELTEC WIFI LORA 32 V3 (ESP32-S3 + SX1262)
 * Conexión a TTN + BNO085 Step Counter + GPS NEO-M8N + Pantalla OLED
 * 
 * CONEXIONES BNO085 (modo I2C) - USA UN SEGUNDO BUS I2C:
 * ======================================================
 * VCC_3V3  -> 3.3V
 * GND      -> GND
 * SCL      -> GPIO 48
 * SDA      -> GPIO 47
 * PS0      -> -
 * PS1      -> -
 * CS       -> -
 * ADDR     -> -
 * RST      -> -
 * 
 * CONEXIONES GPS NEO-M8N (UART):
 * ==============================
 * VCC      -> 3.3V
 * GND      -> GND
 * TX       -> GPIO 19 (RX del ESP32)
 * RX       -> GPIO 20 (TX del ESP32)
 * 
 * NOTA: La pantalla OLED interna usa GPIO 17 (SDA) y GPIO 18 (SCL)
 */

// --- CORRECCIÓN DE ERRORES DE COMPILACIÓN ---
#ifndef HELTEC_BOARD
  #define HELTEC_BOARD 0
#endif
#ifndef SLOW_CLK_TPYE
  #define SLOW_CLK_TPYE 0 
#endif
// --------------------------------------------

// ---------------------------------------------------------
// CONFIGURACIÓN PANTALLA OLED (Heltec WiFi LoRa 32 V3)
// ---------------------------------------------------------
static SSD1306Wire oledDisplay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// ---------------------------------------------------------
// CONFIGURACIÓN BNO085 (Segundo bus I2C)
// ---------------------------------------------------------
#define BNO085_SDA 47
#define BNO085_SCL 48
#define BNO08X_RESET 26

TwoWire Wire1_BNO = TwoWire(1);
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// ---------------------------------------------------------
// CONFIGURACIÓN GPS NEO-M8N (UART2)
// ---------------------------------------------------------
#define GPS_RX_PIN 19  // GPIO19 recibe datos del TX del GPS
#define GPS_TX_PIN 20  // GPIO20 envía datos al RX del GPS
#define GPS_BAUD 9600

HardwareSerial GPSSerial(2);  // UART2
TinyGPSPlus gps;

// Variables GPS
double latitude = 0.0;
double longitude = 0.0;
uint32_t satellites = 0;
bool gpsFixed = false;

// Variables para el contador de pasos
uint32_t stepCount = 0;
uint32_t lastSentSteps = 0;
bool bno085Connected = false;
String loraStatus = "Iniciando...";

// ---------------------------------------------------------
// FUNCIONES DE PANTALLA OLED
// ---------------------------------------------------------
void VextON() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(10);
}

void initDisplay() {
  VextON();
  delay(100);
  
  oledDisplay.init();
  oledDisplay.flipScreenVertically();
  oledDisplay.setContrast(255);
  oledDisplay.clear();
  oledDisplay.setFont(ArialMT_Plain_10);
  oledDisplay.drawString(0, 0, "Iniciando...");
  oledDisplay.display();
  
  Serial.println("Pantalla OLED inicializada");
}

void updateDisplay() {
  oledDisplay.clear();
  oledDisplay.setFont(ArialMT_Plain_10);
  
  // Línea 1: Pasos y satélites
  oledDisplay.drawString(0, 0, "Pasos: " + String(stepCount));
  oledDisplay.drawString(80, 0, "Sat: " + String(satellites));
  
  // Línea 2: Estado GPS
  if (gpsFixed) {
    oledDisplay.drawString(0, 12, "GPS: OK");
  } else {
    oledDisplay.drawString(0, 12, "GPS: Buscando...");
  }
  
  // Línea 3: Latitud
  oledDisplay.drawString(0, 24, "Lat: " + String(latitude, 6));
  
  // Línea 4: Longitud
  oledDisplay.drawString(0, 36, "Lon: " + String(longitude, 6));
  
  // Línea 5: Estado LoRa y BNO
  String statusLine = "LoRa: " + loraStatus;
  if (!bno085Connected) {
    statusLine += " [!BNO]";
  }
  oledDisplay.drawString(0, 52, statusLine);
  
  oledDisplay.display();
}

// ---------------------------------------------------------
// TUS CREDENCIALES
// ---------------------------------------------------------
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x44, 0x27 };
uint8_t appKey[] = { 0x01, 0x2F, 0xF1, 0x1A, 0xFC, 0x34, 0x95, 0x35, 
                     0xD7, 0xCD, 0x25, 0xC3, 0x16, 0x24, 0xA8, 0x66 };

uint8_t nwkSKey[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t appSKey[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;

uint32_t license[4] = { 0x00000000, 0x00000000, 0x00000000, 0x00000000 };

// ---------------------------------------------------------
// CONFIGURACIÓN LORAWAN (Europa EU868)
// ---------------------------------------------------------
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t  loraWanClass = CLASS_A;
uint32_t appTxDutyCycle = 10000;
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = false;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

// ---------------------------------------------------------
// FUNCIONES GPS
// ---------------------------------------------------------
void initGPS() {
  Serial.println("Inicializando GPS NEO-M8N...");
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(100);
  Serial.println("GPS UART inicializado en pines RX:" + String(GPS_RX_PIN) + " TX:" + String(GPS_TX_PIN));
}

void readGPS() {
  // Leer todos los datos disponibles del GPS
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    gps.encode(c);
  }
  
  // Actualizar variables si hay datos válidos
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    gpsFixed = true;
  }
  
  if (gps.satellites.isValid()) {
    satellites = gps.satellites.value();
  }
  
  // Si no hay fix después de un tiempo, marcar como no fijado
  if (gps.location.age() > 5000) {
    gpsFixed = false;
  }
}

// ---------------------------------------------------------
// FUNCIONES BNO085
// ---------------------------------------------------------
void setReports(void) {
  Serial.println("Configurando reportes del BNO085...");
  
  if (!bno08x.enableReport(SH2_ACCELEROMETER, 10000)) {
    Serial.println("ERROR: No se pudo habilitar Acelerómetro");
  } else {
    Serial.println("  - Acelerómetro habilitado");
  }
  
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) {
    Serial.println("ERROR: No se pudo habilitar Giroscopio");
  } else {
    Serial.println("  - Giroscopio habilitado");
  }
  
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000)) {
    Serial.println("ERROR: No se pudo habilitar Game Rotation Vector");
  } else {
    Serial.println("  - Game Rotation Vector habilitado");
  }
  
  if (!bno08x.enableReport(SH2_STEP_COUNTER, 500000)) {
    Serial.println("ERROR: No se pudo habilitar Step Counter");
  } else {
    Serial.println("  - Step Counter habilitado");
  }
  
  if (!bno08x.enableReport(SH2_STEP_DETECTOR, 0)) {
    Serial.println("ERROR: No se pudo habilitar Step Detector");
  } else {
    Serial.println("  - Step Detector habilitado");
  }
  
  Serial.println("Reportes configurados.");
  delay(100);
}

bool initBNO085() {
  Serial.println("Inicializando BNO085...");
  
  Wire1_BNO.begin(BNO085_SDA, BNO085_SCL);
  
  if (!bno08x.begin_I2C(0x4A, &Wire1_BNO)) {
    Serial.println("ERROR: No se encontró el BNO085 en 0x4A");
    if (!bno08x.begin_I2C(0x4B, &Wire1_BNO)) {
      Serial.println("ERROR: No se encontró el BNO085 en 0x4B tampoco");
      return false;
    }
    Serial.println("BNO085 encontrado en dirección 0x4B");
  } else {
    Serial.println("BNO085 encontrado en dirección 0x4A");
  }
  
  Serial.println("Información del sensor:");
  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("  Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version ");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.println(bno08x.prodIds.entry[n].swVersionPatch);
  }
  
  setReports();
  return true;
}

void readStepCounter() {
  if (!bno085Connected) return;
  
  while (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_STEP_COUNTER:
        stepCount = sensorValue.un.stepCounter.steps;
        Serial.print(">>> Pasos: ");
        Serial.println(stepCount);
        break;
      case SH2_STEP_DETECTOR:
        Serial.println(">>> Paso detectado!");
        break;
      default:
        break;
    }
  }
}

// ---------------------------------------------------------
// FUNCIÓN PARA PREPARAR EL MENSAJE LORAWAN
// ---------------------------------------------------------
void prepareTxFrame( uint8_t port ) {
    readStepCounter();
    readGPS();
    
    uint32_t newSteps = stepCount - lastSentSteps;
    
    // Convertir lat/lon a enteros (multiplicar por 1000000 para 6 decimales)
    int32_t latInt = (int32_t)(latitude * 1000000);
    int32_t lonInt = (int32_t)(longitude * 1000000);
    
    // Payload: pasos(4) + nuevos(2) + lat(4) + lon(4) + satellites(1) + gpsFixed(1) = 16 bytes
    appDataSize = 16;
    
    // Pasos totales (4 bytes)
    appData[0] = (stepCount >> 24) & 0xFF;
    appData[1] = (stepCount >> 16) & 0xFF;
    appData[2] = (stepCount >> 8) & 0xFF;
    appData[3] = stepCount & 0xFF;
    
    // Pasos nuevos (2 bytes)
    appData[4] = (newSteps >> 8) & 0xFF;
    appData[5] = newSteps & 0xFF;
    
    // Latitud (4 bytes, signed)
    appData[6] = (latInt >> 24) & 0xFF;
    appData[7] = (latInt >> 16) & 0xFF;
    appData[8] = (latInt >> 8) & 0xFF;
    appData[9] = latInt & 0xFF;
    
    // Longitud (4 bytes, signed)
    appData[10] = (lonInt >> 24) & 0xFF;
    appData[11] = (lonInt >> 16) & 0xFF;
    appData[12] = (lonInt >> 8) & 0xFF;
    appData[13] = lonInt & 0xFF;
    
    // Satélites (1 byte)
    appData[14] = satellites & 0xFF;
    
    // GPS Fixed (1 byte)
    appData[15] = gpsFixed ? 1 : 0;
    
    lastSentSteps = stepCount;
    
    Serial.println("=== ENVIANDO POR LORAWAN ===");
    Serial.print("Pasos: "); Serial.print(stepCount);
    Serial.print(" | Nuevos: "); Serial.println(newSteps);
    Serial.print("Lat: "); Serial.print(latitude, 6);
    Serial.print(" | Lon: "); Serial.println(longitude, 6);
    Serial.print("Satélites: "); Serial.print(satellites);
    Serial.print(" | GPS Fix: "); Serial.println(gpsFixed ? "Sí" : "No");
}

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);
  
  VextON();
  delay(100);
  
  Serial.println("\n\n========================================");
  Serial.println("   HELTEC V3 + BNO085 + GPS NEO-M8N");
  Serial.println("========================================\n");

  // Inicializar pantalla
  initDisplay();
  
  // Inicializar GPS
  oledDisplay.clear();
  oledDisplay.setFont(ArialMT_Plain_10);
  oledDisplay.drawString(0, 0, "Iniciando GPS...");
  oledDisplay.display();
  initGPS();
  
  // Inicializar BNO085
  oledDisplay.clear();
  oledDisplay.drawString(0, 0, "Iniciando BNO085...");
  oledDisplay.display();
  bno085Connected = initBNO085();
  
  if (!bno085Connected) {
    Serial.println("*** ADVERTENCIA: BNO085 no detectado ***");
    oledDisplay.clear();
    oledDisplay.drawString(0, 0, "BNO085 NO detectado!");
    oledDisplay.display();
    delay(2000);
  } else {
    oledDisplay.clear();
    oledDisplay.drawString(0, 0, "BNO085 OK!");
    oledDisplay.display();
    delay(1000);
  }

  // Inicializar LoRaWAN
  oledDisplay.clear();
  oledDisplay.drawString(0, 0, "Iniciando LoRaWAN...");
  oledDisplay.display();
  
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  Serial.println("Sistema LoRaWAN inicializado.");
  
  updateDisplay();
}

// ---------------------------------------------------------
// LOOP PRINCIPAL
// ---------------------------------------------------------
unsigned long lastDisplayUpdate = 0;
unsigned long lastSendTime = 0;
bool loraJoined = false;

void loop() {
  // Leer sensores constantemente
  readStepCounter();
  readGPS();
  
  // Actualizar pantalla cada 500ms
  if (millis() - lastDisplayUpdate >= 500) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
  
  // Máquina de estados LoRaWAN
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
#if(LORAWAN_DEVEUI_AUTO)
      LoRaWAN.generateDeveuiByChipID();
#endif
      LoRaWAN.init(loraWanClass, loraWanRegion);
      LoRaWAN.setDefaultDR(3);
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      loraStatus = "Conectando...";
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      if (!loraJoined) {
        loraJoined = true;
        lastSendTime = millis();
      }
      loraStatus = "Enviando...";
      prepareTxFrame(appPort);
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      loraStatus = "Conectado";
      lastSendTime = millis();
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.sleep(loraWanClass);
      
      if (loraJoined && (millis() - lastSendTime >= appTxDutyCycle)) {
        deviceState = DEVICE_STATE_SEND;
      }
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}
