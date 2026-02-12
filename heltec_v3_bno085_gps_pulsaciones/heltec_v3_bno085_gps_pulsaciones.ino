#include "LoRaWan_APP.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "HT_SSD1306Wire.h"
#include "HT_TinyGPS++.h"

// --- CORRECCIÓN DE ERRORES DE COMPILACIÓN ---
#ifndef HELTEC_BOARD
  #define HELTEC_BOARD 0
#endif
#ifndef SLOW_CLK_TPYE
  #define SLOW_CLK_TPYE 0 
#endif
// --------------------------------------------

/* * CONEXIONES ACTUALIZADAS HELTEC V3:
 * GPS: TX->19, RX->20
 * BNO085: SDA->47, SCL->48
 * PULSE SENSOR: S->GPIO 1, +->3.3V, ->GND
 */

// --- CONFIGURACIÓN SENSORES ---
#define PULSE_PIN 1  // Pin analógico para pulsaciones
static SSD1306Wire oledDisplay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

#define BNO085_SDA 47
#define BNO085_SCL 48
#define BNO08X_RESET 26

TwoWire Wire1_BNO = TwoWire(1);
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#define GPS_RX_PIN 19
#define GPS_TX_PIN 20
#define GPS_BAUD 9600

HardwareSerial GPSSerial(2);
TinyGPSPlus gps;

// Variables de estado
double latitude = 0.0, longitude = 0.0;
uint32_t satellites = 0, stepCount = 0, lastSentSteps = 0;
int pulseValue = 0; 
bool gpsFixed = false, bno085Connected = false;
String loraStatus = "Iniciando...";
unsigned long lastDisplayUpdate = 0;
unsigned long lastSendTime = 0;
bool loraJoined = false;

// --- CREDENCIALES LORAWAN (Asegúrate de que coincidan con TTN) ---
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x44, 0x27 };
uint8_t appKey[] = { 0x01, 0x2F, 0xF1, 0x1A, 0xFC, 0x34, 0x95, 0x35, 0xD7, 0xCD, 0x25, 0xC3, 0x16, 0x24, 0xA8, 0x66 };

uint8_t nwkSKey[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t appSKey[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint32_t devAddr = (uint32_t)0x007e6ae1;

uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = CLASS_A;
uint32_t appTxDutyCycle = 10000;
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = false;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

// --- FUNCIONES ---

void VextON() { 
  pinMode(Vext, OUTPUT); 
  digitalWrite(Vext, LOW); 
  delay(10); 
}

void updateDisplay() {
  oledDisplay.clear();
  oledDisplay.setFont(ArialMT_Plain_10);
  
  oledDisplay.drawString(0, 0, "P:" + String(stepCount));
  oledDisplay.drawString(65, 0, "Sat:" + String(satellites));
  
  // PULSO: Muy pequeño en la esquina
  oledDisplay.drawString(108, 0, String(pulseValue)); 
  
  oledDisplay.drawString(0, 12, gpsFixed ? "GPS: OK" : "GPS: Bsq...");
  oledDisplay.drawString(0, 24, "Lat:" + String(latitude, 5));
  oledDisplay.drawString(0, 36, "Lon:" + String(longitude, 5));
  
  String status = "LoRa:" + loraStatus + (!bno085Connected ? " !BNO" : "");
  oledDisplay.drawString(0, 52, status);
  oledDisplay.display();
}

void readGPS() {
  while (GPSSerial.available() > 0) gps.encode(GPSSerial.read());
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    gpsFixed = true;
  }
  if (gps.satellites.isValid()) satellites = gps.satellites.value();
  if (gps.location.age() > 5000) gpsFixed = false;
}

bool initBNO085() {
  Wire1_BNO.begin(BNO085_SDA, BNO085_SCL);
  if (!bno08x.begin_I2C(0x4A, &Wire1_BNO)) return false;
  bno08x.enableReport(SH2_STEP_COUNTER, 500000);
  return true;
}

void readStepCounter() {
  if (!bno085Connected) return;
  while (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_STEP_COUNTER) stepCount = sensorValue.un.stepCounter.steps;
  }
}

void prepareTxFrame(uint8_t port) {
    readStepCounter();
    readGPS();
    pulseValue = analogRead(PULSE_PIN); 

    int32_t latInt = (int32_t)(latitude * 1000000);
    int32_t lonInt = (int32_t)(longitude * 1000000);
    uint32_t newSteps = stepCount - lastSentSteps;

    appDataSize = 18; 
    appData[0] = (stepCount >> 24) & 0xFF;
    appData[1] = (stepCount >> 16) & 0xFF;
    appData[2] = (stepCount >> 8) & 0xFF;
    appData[3] = stepCount & 0xFF;
    appData[4] = (newSteps >> 8) & 0xFF;
    appData[5] = newSteps & 0xFF;
    appData[6] = (latInt >> 24) & 0xFF;
    appData[7] = (latInt >> 16) & 0xFF;
    appData[8] = (latInt >> 8) & 0xFF;
    appData[9] = latInt & 0xFF;
    appData[10] = (lonInt >> 24) & 0xFF;
    appData[11] = (lonInt >> 16) & 0xFF;
    appData[12] = (lonInt >> 8) & 0xFF;
    appData[13] = lonInt & 0xFF;
    appData[14] = satellites & 0xFF;
    appData[15] = gpsFixed ? 1 : 0;
    appData[16] = (pulseValue >> 8) & 0xFF;
    appData[17] = pulseValue & 0xFF;

    lastSentSteps = stepCount;
}

void setup() {
  Serial.begin(115200);
  VextON();
  
  oledDisplay.init();
  oledDisplay.flipScreenVertically();
  oledDisplay.clear();
  oledDisplay.display();
  
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  bno085Connected = initBNO085();
  
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  pinMode(PULSE_PIN, INPUT); 
}

void loop() {
  readStepCounter();
  readGPS();
  pulseValue = analogRead(PULSE_PIN); 

  if (millis() - lastDisplayUpdate >= 500) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  switch(deviceState) {
    case DEVICE_STATE_INIT: LoRaWAN.init(loraWanClass, loraWanRegion); break;
    case DEVICE_STATE_JOIN: loraStatus = "Join..."; LoRaWAN.join(); break;
    case DEVICE_STATE_SEND:
      loraStatus = "Env...";
      prepareTxFrame(appPort);
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    case DEVICE_STATE_CYCLE: 
      loraStatus = "OK"; 
      lastSendTime = millis(); 
      deviceState = DEVICE_STATE_SLEEP; 
      break;
    case DEVICE_STATE_SLEEP:
      LoRaWAN.sleep(loraWanClass);
      if (millis() - lastSendTime >= appTxDutyCycle) deviceState = DEVICE_STATE_SEND;
      break;
    default: deviceState = DEVICE_STATE_INIT; break;
  }
}