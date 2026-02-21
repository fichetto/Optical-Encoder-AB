#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include "calibration.h"

// ===================================================================
//  MAPPA REGISTRI MODBUS RTU (Slave ID: 1)
//  USB CDC nativo ESP32-S3 - Nessun reset da DTR
// -------------------------------------------------------------------
//  FC 03/04 - Read Holding/Input Registers:
//    0x0000 : X_HIGH  - Bit 31-16 del contatore X (int16, signed)
//    0x0001 : X_LOW   - Bit 15-0  del contatore X (uint16)
//    0x0002 : Y_HIGH  - Bit 31-16 del contatore Y (int16, signed)
//    0x0003 : Y_LOW   - Bit 15-0  del contatore Y (uint16)
//    0x0004 : SQUAL   - Surface Quality PMW3901 (uint16)
//    0x0005 : ENC_HIGH- Bit 31-16 posizione encoder (int16, signed)
//    0x0006 : ENC_LOW - Bit 15-0  posizione encoder (uint16)
//
//  FC 06 - Write Single Register:
//    Reg 0x0010, Val 0x0001 : Reset contatori X, Y, encoder
//    Reg 0x0011, Val 0x0001 : LED PMW3901 ON
//    Reg 0x0011, Val 0x0000 : LED PMW3901 OFF
//
//  Ricostruzione int32 lato PC (Python):
//    import struct
//    x = struct.unpack('>i', struct.pack('>HH', x_high, x_low))[0]
// ===================================================================

// Pin definitions per ESP32-S3-Nano Waveshare
#define PMW3901_CS    11
#define PMW3901_SCK   12
#define PMW3901_MOSI  13
#define PMW3901_MISO  14

// Pin encoder AB output (simulano encoder tradizionale)
#define ENCODER_A_PIN  2
#define ENCODER_B_PIN  3

// LED RGB WS2812 integrato sulla scheda ESP32-S3
#define RGB_LED_PIN    48
#define NUM_LEDS       1

// Modbus RTU
#define MODBUS_SLAVE_ID  1

// PMW3901 registers
#define PMW3901_PRODUCT_ID        0x00
#define PMW3901_REVISION_ID       0x01
#define PMW3901_MOTION            0x02
#define PMW3901_DELTA_X_L         0x03
#define PMW3901_DELTA_X_H         0x04
#define PMW3901_DELTA_Y_L         0x05
#define PMW3901_DELTA_Y_H         0x06
#define PMW3901_SQUAL             0x07
#define PMW3901_RAW_DATA_SUM      0x08
#define PMW3901_MAXIMUM_RAW_DATA  0x09
#define PMW3901_MINIMUM_RAW_DATA  0x0A
#define PMW3901_SHUTTER_UPPER     0x0B
#define PMW3901_SHUTTER_LOWER     0x0C

// NeoPixel
Adafruit_NeoPixel rgb_led(NUM_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// SPI
SPIClass *spi;

// Registri accumulatori (thread-safe via mutex)
volatile long registerX     = 0;
volatile long registerY     = 0;
volatile long encoderPosition = 0;
volatile uint8_t lastSQUAL  = 0;
volatile bool encoderA_state = false;
volatile bool encoderB_state = false;

// Filtro outlier - Finestra mobile per rilevamento anomalie
#define OUTLIER_WINDOW_SIZE 10
#define OUTLIER_MAD_THRESHOLD 3.0
struct OutlierFilter {
  int16_t windowX[OUTLIER_WINDOW_SIZE];
  int16_t windowY[OUTLIER_WINDOW_SIZE];
  uint8_t windowIndex;
  uint32_t outlierCountX;
  uint32_t outlierCountY;
  uint32_t totalSamples;

  OutlierFilter() : windowIndex(0), outlierCountX(0), outlierCountY(0), totalSamples(0) {
    memset(windowX, 0, sizeof(windowX));
    memset(windowY, 0, sizeof(windowY));
  }

  int16_t calculateMedian(int16_t* values, uint8_t size) {
    int16_t sorted[OUTLIER_WINDOW_SIZE];
    memcpy(sorted, values, size * sizeof(int16_t));
    for (uint8_t i = 0; i < size - 1; i++) {
      for (uint8_t j = 0; j < size - i - 1; j++) {
        if (sorted[j] > sorted[j + 1]) {
          int16_t temp = sorted[j];
          sorted[j] = sorted[j + 1];
          sorted[j + 1] = temp;
        }
      }
    }
    return (size % 2 == 0) ? (sorted[size/2 - 1] + sorted[size/2]) / 2 : sorted[size/2];
  }

  float calculateMAD(int16_t* values, uint8_t size, int16_t median) {
    int16_t deviations[OUTLIER_WINDOW_SIZE];
    for (uint8_t i = 0; i < size; i++) {
      deviations[i] = abs(values[i] - median);
    }
    return (float)calculateMedian(deviations, size);
  }

  bool filterSample(int16_t deltaX, int16_t deltaY, int16_t* filteredX, int16_t* filteredY) {
    totalSamples++;
    if (totalSamples <= OUTLIER_WINDOW_SIZE) {
      windowX[windowIndex] = deltaX;
      windowY[windowIndex] = deltaY;
      windowIndex = (windowIndex + 1) % OUTLIER_WINDOW_SIZE;
      *filteredX = deltaX;
      *filteredY = deltaY;
      return true;
    }
    int16_t medianX = calculateMedian(windowX, OUTLIER_WINDOW_SIZE);
    float   madX    = calculateMAD(windowX, OUTLIER_WINDOW_SIZE, medianX);
    int16_t medianY = calculateMedian(windowY, OUTLIER_WINDOW_SIZE);
    float   madY    = calculateMAD(windowY, OUTLIER_WINDOW_SIZE, medianY);
    bool isOutlierX = (madX > 0) && (abs(deltaX - medianX) > OUTLIER_MAD_THRESHOLD * madX);
    bool isOutlierY = (madY > 0) && (abs(deltaY - medianY) > OUTLIER_MAD_THRESHOLD * madY);
    *filteredX = isOutlierX ? medianX : deltaX;
    *filteredY = isOutlierY ? medianY : deltaY;
    windowX[windowIndex] = *filteredX;
    windowY[windowIndex] = *filteredY;
    windowIndex = (windowIndex + 1) % OUTLIER_WINDOW_SIZE;
    return !(isOutlierX || isOutlierY);
  }

  float getOutlierRateX() { return totalSamples > 0 ? (100.0f * outlierCountX / totalSamples) : 0.0f; }
  float getOutlierRateY() { return totalSamples > 0 ? (100.0f * outlierCountY / totalSamples) : 0.0f; }
  void  resetStats()      { outlierCountX = 0; outlierCountY = 0; totalSamples = 0; }
};

OutlierFilter outlierFilter;

// Sistema dual core
TaskHandle_t WiFiTask   = NULL;
TaskHandle_t SensorTask = NULL;
SemaphoreHandle_t registerMutex = NULL;

// WiFi Access Point per calibrazione
WebServer server(80);
volatile bool wifiAPStarted   = false;
volatile bool wifiAPRequested = false;
volatile unsigned long wifiAPStartTime  = 0;
volatile unsigned long lastWiFiActivity = 0;
#define WIFI_AP_TIMEOUT_MS 180000

// Sistema di calibrazione
EncoderCalibration calibration;
volatile bool pmw3901LedsEnabled = false;

// Function declarations
uint8_t readRegister(uint8_t reg);
void    writeRegister(uint8_t reg, uint8_t data);
void    sensorTask(void *pvParameters);
void    wifiTask(void *pvParameters);
void    updateEncoderOutputs(int16_t deltaX);
void    updateRGBLED(bool encoderA, bool encoderB);
void    setupWebServer();
void    setPMW3901LEDs(bool enable);
void    initPMW3901LEDs();
void    initPMW3901Registers();
uint16_t crc16Modbus(uint8_t *buf, uint16_t len);
void    handleModbusRTU();
void    processModbusRequest(uint8_t *frame, uint8_t frameLen);

// ---------------------------------------------------------------------------
void setup() {
  // USB CDC nativo ESP32-S3: il baud rate e' nominale, la velocita' reale e' USB.
  // DTR non causa reset hardware su USB CDC nativo (nessun chip CP210x/CH340).
  // NON usare setTxTimeoutMs(0): con HWCDC fa scartare silenziosamente le write.
  Serial.begin(921600);

  delay(500);

  // Inizializza pin encoder AB
  pinMode(ENCODER_A_PIN, OUTPUT);
  pinMode(ENCODER_B_PIN, OUTPUT);
  digitalWrite(ENCODER_A_PIN, LOW);
  digitalWrite(ENCODER_B_PIN, LOW);

  // Inizializza LED RGB WS2812
  rgb_led.begin();
  rgb_led.setBrightness(50);
  rgb_led.clear();
  rgb_led.show();

  // Inizializza sistema di calibrazione
  calibration.begin();

  // Crea mutex per sincronizzazione registri
  registerMutex = xSemaphoreCreateMutex();
  if (registerMutex == NULL) while(1);

  // Inizializza SPI
  spi = new SPIClass(HSPI);
  spi->begin(PMW3901_SCK, PMW3901_MISO, PMW3901_MOSI, PMW3901_CS);
  pinMode(PMW3901_CS, OUTPUT);
  digitalWrite(PMW3901_CS, HIGH);
  delay(100);

  // Inizializzazione PMW3901 (sequenza Bitcraze)
  uint8_t productID = readRegister(PMW3901_PRODUCT_ID);
  if (productID == 0x49) {
    writeRegister(0x3A, 0x5A);  // Power on reset
    delay(5);
    readRegister(0x02); readRegister(0x03); readRegister(0x04);
    readRegister(0x05); readRegister(0x06);
    delay(1);
    initPMW3901Registers();
  }

  // Re-init SPI dopo la sequenza di inizializzazione
  spi = new SPIClass(HSPI);
  spi->begin(PMW3901_SCK, PMW3901_MISO, PMW3901_MOSI, PMW3901_CS);
  pinMode(PMW3901_CS, OUTPUT);
  digitalWrite(PMW3901_CS, HIGH);

  initPMW3901LEDs();

  // Avvia task sui due core
  xTaskCreatePinnedToCore(sensorTask, "SensorTask",  8192, NULL, 2, &SensorTask, 0);
  xTaskCreatePinnedToCore(wifiTask,   "WiFiTask",   16384, NULL, 1, &WiFiTask,   1);
}

// Loop principale: avvia WiFi AP dopo 5 secondi dal boot
void loop() {
  static bool immediateAPStarted = false;
  if (!immediateAPStarted && millis() > 5000) {
    wifiAPRequested    = true;
    immediateAPStarted = true;
  }
  delay(1000);
}

// ---------------------------------------------------------------------------
// Task Core 0: Sensore PMW3901 @ 100Hz
// ---------------------------------------------------------------------------
void sensorTask(void *pvParameters) {
  delay(100);
  if (readRegister(PMW3901_PRODUCT_ID) != 0x49) vTaskDelete(NULL);

  uint32_t cycleCount      = 0;
  uint32_t poorQualityCount = 0;
  uint32_t lastResetTime   = 0;
  uint32_t totalReadings   = 0;

  while (1) {
    cycleCount++;
    totalReadings++;

    if (cycleCount % 10 == 0) taskYIELD();

    uint8_t motion = readRegister(PMW3901_MOTION);
    vTaskDelay(pdMS_TO_TICKS(1));

    int16_t deltaX = (int16_t)((readRegister(PMW3901_DELTA_X_H) << 8) | readRegister(PMW3901_DELTA_X_L));
    vTaskDelay(pdMS_TO_TICKS(1));
    int16_t deltaY = (int16_t)((readRegister(PMW3901_DELTA_Y_H) << 8) | readRegister(PMW3901_DELTA_Y_L));

    // Applica filtro outlier
    int16_t filteredX, filteredY;
    outlierFilter.filterSample(deltaX, deltaY, &filteredX, &filteredY);
    deltaX = filteredX;
    deltaY = filteredY;

    // Controllo qualita' ogni 40 cicli (~400ms a 100Hz)
    if (cycleCount % 40 == 0) {
      uint8_t squal        = readRegister(PMW3901_SQUAL);
      uint8_t shutterUpper = readRegister(PMW3901_SHUTTER_UPPER);
      uint8_t shutterLower = readRegister(PMW3901_SHUTTER_LOWER);
      uint8_t rawSum       = readRegister(PMW3901_RAW_DATA_SUM);

      lastSQUAL = squal;  // Scrittura atomica (uint8, no mutex necessario)

      bool sensorBlocked = (squal == 0) || (rawSum == 0) ||
                           (shutterUpper == 132 && shutterLower == 3);

      if (sensorBlocked) {
        poorQualityCount++;
        if (poorQualityCount >= 3 && (millis() - lastResetTime) > 10000) {
          writeRegister(0x3A, 0x5A);
          delay(200);
          initPMW3901Registers();
          delay(100);
          if (xSemaphoreTake(registerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (abs(registerX) > 500000 || abs(registerY) > 500000) {
              registerX = 0; registerY = 0; encoderPosition = 0;
            }
            xSemaphoreGive(registerMutex);
          }
          poorQualityCount = 0;
          lastResetTime    = millis();
        }
      } else {
        poorQualityCount = 0;
      }
    }

    // Manutenzione preventiva ogni 10.000 letture
    if (totalReadings > 0 && totalReadings % 10000 == 0) {
      for (int i = 0; i < 5; i++) {
        readRegister(PMW3901_MOTION);
        readRegister(PMW3901_DELTA_X_L); readRegister(PMW3901_DELTA_X_H);
        readRegister(PMW3901_DELTA_Y_L); readRegister(PMW3901_DELTA_Y_H);
        delay(10);
      }
    }

    // Aggiorna registri accumulatori se c'e' movimento
    if ((motion & 0x80) || deltaX != 0 || deltaY != 0) {
      if (deltaX != 0 || deltaY != 0) {
        if (xSemaphoreTake(registerMutex, portMAX_DELAY) == pdTRUE) {
          registerX += deltaX;
          registerY += deltaY;
          xSemaphoreGive(registerMutex);
        }
        updateEncoderOutputs(deltaX);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz (era 20Hz)
  }
}

// ---------------------------------------------------------------------------
// Task Core 1: Modbus RTU slave + WiFi AP
// ---------------------------------------------------------------------------
void wifiTask(void *pvParameters) {
  while (1) {
    // Gestisci Modbus RTU (prima priorita')
    handleModbusRTU();

    // Gestisci richiesta avvio AP
    if (wifiAPRequested && !wifiAPStarted) {
      WiFi.mode(WIFI_AP);
      WiFi.softAP("EncoderCalibration", "12345678");
      setupWebServer();
      server.begin();
      wifiAPStarted   = true;
      wifiAPStartTime = millis();
      lastWiFiActivity = millis();
      wifiAPRequested = false;
    }

    // Gestisci richieste HTTP
    if (wifiAPStarted) {
      server.handleClient();
      if ((millis() - lastWiFiActivity) > WIFI_AP_TIMEOUT_MS) {
        WiFi.mode(WIFI_OFF);
        wifiAPStarted = false;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));  // ~1000Hz max polling Modbus
  }
}

// ---------------------------------------------------------------------------
// Modbus RTU - Implementazione Slave
// ---------------------------------------------------------------------------
uint16_t crc16Modbus(uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  return crc;
}

void handleModbusRTU() {
  static uint8_t buf[64];
  static uint8_t bufLen = 0;

  // Leggi byte disponibili nel buffer USB CDC
  while (Serial.available() && bufLen < (uint8_t)sizeof(buf)) {
    buf[bufLen++] = (uint8_t)Serial.read();
  }

  // Elabora frame con rilevamento a lunghezza fissa
  while (bufLen >= 4) {
    // Scarta byte non indirizzati a questo slave
    if (buf[0] != MODBUS_SLAVE_ID) {
      memmove(buf, buf + 1, --bufLen);
      continue;
    }

    uint8_t fc = buf[1];
    uint8_t expectedLen;

    if (fc == 0x03 || fc == 0x04 || fc == 0x06) {
      expectedLen = 8;
    } else {
      bufLen = 0;  // Function code non supportato: svuota buffer
      break;
    }

    if (bufLen < expectedLen) break;  // Frame incompleto: attendi altri byte

    // Verifica CRC
    uint16_t crc   = crc16Modbus(buf, expectedLen - 2);
    uint16_t rxCrc = buf[expectedLen - 2] | ((uint16_t)buf[expectedLen - 1] << 8);

    if (crc == rxCrc) {
      processModbusRequest(buf, expectedLen);
    }

    // Consuma il frame dal buffer
    bufLen -= expectedLen;
    if (bufLen > 0) memmove(buf, buf + expectedLen, bufLen);
  }
}

void processModbusRequest(uint8_t *frame, uint8_t frameLen) {
  uint8_t fc = frame[1];
  uint8_t resp[20];
  uint8_t ri = 0;

  if (fc == 0x03 || fc == 0x04) {
    uint16_t startReg = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t regCount = ((uint16_t)frame[4] << 8) | frame[5];

    if (regCount == 0 || regCount > 7) {
      // Eccezione 0x03: Illegal Data Value
      resp[ri++] = MODBUS_SLAVE_ID;
      resp[ri++] = fc | 0x80;
      resp[ri++] = 0x03;
      uint16_t crc = crc16Modbus(resp, ri);
      resp[ri++] = crc & 0xFF;
      resp[ri++] = crc >> 8;
      Serial.write(resp, ri);
      Serial.flush();
      return;
    }

    // Leggi valori thread-safe
    int32_t x, y, enc;
    uint8_t squal;
    if (xSemaphoreTake(registerMutex, pdMS_TO_TICKS(5)) != pdTRUE) return;
    x    = (int32_t)registerX;
    y    = (int32_t)registerY;
    enc  = (int32_t)encoderPosition;
    squal = lastSQUAL;
    xSemaphoreGive(registerMutex);

    // Costruisci risposta: header (3) + dati (regCount*2) + CRC (2)
    resp[ri++] = MODBUS_SLAVE_ID;
    resp[ri++] = fc;
    resp[ri++] = (uint8_t)(regCount * 2);

    for (uint16_t i = 0; i < regCount; i++) {
      uint16_t regVal = 0;
      switch (startReg + i) {
        case 0x0000: regVal = (uint16_t)((uint32_t)x   >> 16); break;  // X high
        case 0x0001: regVal = (uint16_t)(x   & 0xFFFF);        break;  // X low
        case 0x0002: regVal = (uint16_t)((uint32_t)y   >> 16); break;  // Y high
        case 0x0003: regVal = (uint16_t)(y   & 0xFFFF);        break;  // Y low
        case 0x0004: regVal = (uint16_t)squal;                 break;  // SQUAL
        case 0x0005: regVal = (uint16_t)((uint32_t)enc >> 16); break;  // Encoder high
        case 0x0006: regVal = (uint16_t)(enc & 0xFFFF);        break;  // Encoder low
        default:     regVal = 0;                                break;
      }
      resp[ri++] = (uint8_t)(regVal >> 8);   // Modbus big-endian
      resp[ri++] = (uint8_t)(regVal & 0xFF);
    }

    uint16_t crc = crc16Modbus(resp, ri);
    resp[ri++] = crc & 0xFF;
    resp[ri++] = crc >> 8;
    Serial.write(resp, ri);
    Serial.flush();

  } else if (fc == 0x06) {
    uint16_t reg = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t val = ((uint16_t)frame[4] << 8) | frame[5];

    if (reg == 0x0010 && val == 0x0001) {
      // Reset contatori X, Y, encoder
      if (xSemaphoreTake(registerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        registerX       = 0;
        registerY       = 0;
        encoderPosition = 0;
        xSemaphoreGive(registerMutex);
      }
      Serial.write(frame, frameLen);
      Serial.flush();
    } else if (reg == 0x0011) {
      // LED PMW3901: 0x0001=ON, 0x0000=OFF
      pmw3901LedsEnabled = (val == 0x0001);
      setPMW3901LEDs(pmw3901LedsEnabled);
      Serial.write(frame, frameLen);
      Serial.flush();
    } else {
      // Eccezione 0x02: Illegal Data Address
      resp[ri++] = MODBUS_SLAVE_ID;
      resp[ri++] = fc | 0x80;
      resp[ri++] = 0x02;
      uint16_t crc = crc16Modbus(resp, ri);
      resp[ri++] = crc & 0xFF;
      resp[ri++] = crc >> 8;
      Serial.write(resp, ri);
      Serial.flush();
    }
  }
}

// ---------------------------------------------------------------------------
// SPI read/write per PMW3901
// ---------------------------------------------------------------------------
uint8_t readRegister(uint8_t reg) {
  if (!spi) return 0;
  digitalWrite(PMW3901_CS, LOW);
  delayMicroseconds(50);
  spi->transfer(reg & 0x7F);
  delayMicroseconds(50);
  uint8_t data = spi->transfer(0x00);
  delayMicroseconds(100);
  digitalWrite(PMW3901_CS, HIGH);
  return data;
}

void writeRegister(uint8_t reg, uint8_t data) {
  if (!spi) return;
  digitalWrite(PMW3901_CS, LOW);
  delayMicroseconds(50);
  spi->transfer(reg | 0x80);
  spi->transfer(data);
  delayMicroseconds(50);
  digitalWrite(PMW3901_CS, HIGH);
  delayMicroseconds(200);
}

// ---------------------------------------------------------------------------
// WiFi Access Point per calibrazione
// ---------------------------------------------------------------------------
void setupWebServer() {
  server.on("/", [](){
    lastWiFiActivity = millis();
    String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Encoder Calibrazione</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 40px; background: #f0f0f0; }
        .container { max-width: 400px; margin: auto; background: white; padding: 20px; border-radius: 10px; }
        h1 { color: #333; text-align: center; }
        .form-group { margin: 15px 0; }
        label { display: block; margin-bottom: 5px; font-weight: bold; }
        input { width: 100%; padding: 10px; border: 1px solid #ddd; border-radius: 5px; box-sizing: border-box; }
        button { width: 100%; padding: 12px; background: #007bff; color: white; border: none; border-radius: 5px; cursor: pointer; margin-top: 5px; }
        button:hover { background: #0056b3; }
        .info { background: #e7f3ff; border: 1px solid #b3d9ff; padding: 10px; border-radius: 5px; margin: 10px 0; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Encoder Calibrazione</h1>
        <div class="info">
            <strong>Distanza:</strong> )" + String(calibration.getDistance(), 1) + R"( mm<br>
            <strong>Scala X:</strong> )" + String(calibration.getScaleX(), 4) + R"(<br>
            <strong>Scala Y:</strong> )" + String(calibration.getScaleY(), 4) + R"(<br>
            <strong>SQUAL:</strong> )" + String((int)lastSQUAL) + R"(
        </div>
        <form action="/calibrate" method="POST">
            <div class="form-group">
                <label>Distanza sensore da superficie (mm):</label>
                <input type="number" name="distance" min="80" max="2000" step="0.1" value=")" + String(calibration.getDistance(), 1) + R"(" required>
            </div>
            <button type="submit">Salva Calibrazione</button>
        </form>
        <div style="margin-top: 20px;">
            <h3>LED PMW3901</h3>
            <button onclick="location.href='/led-on'"  style="background:#28a745">Accendi LED</button>
            <button onclick="location.href='/led-off'" style="background:#6c757d">Spegni LED</button>
        </div>
        <div style="margin-top: 10px;">
            <button onclick="location.href='/reset'"      style="background:#dc3545">Reset Registri</button>
            <button onclick="location.href='/reset-stats'" style="background:#ffc107;color:#000">Reset Outlier Stats</button>
        </div>
    </div>
</body>
</html>)";
    server.send(200, "text/html", html);
  });

  server.on("/calibrate", HTTP_POST, [](){
    lastWiFiActivity = millis();
    if (server.hasArg("distance")) {
      calibration.setDistance(server.arg("distance").toFloat());
    }
    String r = "<html><head><meta http-equiv='refresh' content='2; url=/'></head>";
    r += "<body><h2>Calibrazione salvata!</h2><a href='/'>Torna alla home</a></body></html>";
    server.send(200, "text/html", r);
  });

  server.on("/reset", [](){
    lastWiFiActivity = millis();
    if (xSemaphoreTake(registerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      registerX = 0; registerY = 0; encoderPosition = 0;
      xSemaphoreGive(registerMutex);
    }
    String r = "<html><head><meta http-equiv='refresh' content='2; url=/'></head>";
    r += "<body><h2>Registri resettati!</h2><a href='/'>Torna alla home</a></body></html>";
    server.send(200, "text/html", r);
  });

  server.on("/led-on", [](){
    lastWiFiActivity = millis();
    pmw3901LedsEnabled = true;
    setPMW3901LEDs(true);
    String r = "<html><head><meta http-equiv='refresh' content='2; url=/'></head>";
    r += "<body><h2>LED ACCESI</h2><a href='/'>Torna alla home</a></body></html>";
    server.send(200, "text/html", r);
  });

  server.on("/led-off", [](){
    lastWiFiActivity = millis();
    pmw3901LedsEnabled = false;
    setPMW3901LEDs(false);
    String r = "<html><head><meta http-equiv='refresh' content='2; url=/'></head>";
    r += "<body><h2>LED SPENTI</h2><a href='/'>Torna alla home</a></body></html>";
    server.send(200, "text/html", r);
  });

  server.on("/reset-stats", [](){
    lastWiFiActivity = millis();
    outlierFilter.resetStats();
    String r = "<html><head><meta http-equiv='refresh' content='2; url=/'></head>";
    r += "<body><h2>Statistiche resettate!</h2><a href='/'>Torna alla home</a></body></html>";
    server.send(200, "text/html", r);
  });
}

// ---------------------------------------------------------------------------
// LED PMW3901 integrati
// ---------------------------------------------------------------------------
void setPMW3901LEDs(bool enable) {
  delay(200);
  writeRegister(0x7F, 0x14);
  writeRegister(0x6F, enable ? 0x1C : 0x00);
  writeRegister(0x7F, 0x00);
}

void initPMW3901LEDs() {
  setPMW3901LEDs(pmw3901LedsEnabled);
}

// ---------------------------------------------------------------------------
// Sequenza di inizializzazione Bitcraze (testata e funzionante)
// ---------------------------------------------------------------------------
void initPMW3901Registers() {
  writeRegister(0x7F, 0x00); writeRegister(0x61, 0xAD); writeRegister(0x7F, 0x03);
  writeRegister(0x40, 0x00); writeRegister(0x7F, 0x05); writeRegister(0x41, 0xB3);
  writeRegister(0x43, 0xF1); writeRegister(0x45, 0x14); writeRegister(0x5B, 0x32);
  writeRegister(0x5F, 0x34); writeRegister(0x7B, 0x08); writeRegister(0x7F, 0x06);
  writeRegister(0x44, 0x1B); writeRegister(0x40, 0xBF); writeRegister(0x4E, 0x3F);
  writeRegister(0x7F, 0x08); writeRegister(0x65, 0x20); writeRegister(0x6A, 0x18);
  writeRegister(0x7F, 0x09); writeRegister(0x4F, 0xAF); writeRegister(0x5F, 0x40);
  writeRegister(0x48, 0x80); writeRegister(0x49, 0x80); writeRegister(0x57, 0x77);
  writeRegister(0x60, 0x78); writeRegister(0x61, 0x78); writeRegister(0x62, 0x08);
  writeRegister(0x63, 0x50); writeRegister(0x7F, 0x0A); writeRegister(0x45, 0x60);
  writeRegister(0x7F, 0x00); writeRegister(0x4D, 0x11); writeRegister(0x55, 0x80);
  writeRegister(0x74, 0x1F); writeRegister(0x75, 0x1F); writeRegister(0x4A, 0x78);
  writeRegister(0x4B, 0x78); writeRegister(0x44, 0x08); writeRegister(0x45, 0x50);
  writeRegister(0x64, 0xFF); writeRegister(0x65, 0x1F); writeRegister(0x7F, 0x14);
  writeRegister(0x65, 0x60); writeRegister(0x66, 0x08); writeRegister(0x63, 0x78);
  writeRegister(0x7F, 0x15); writeRegister(0x48, 0x58); writeRegister(0x7F, 0x07);
  writeRegister(0x41, 0x0D); writeRegister(0x43, 0x14); writeRegister(0x4B, 0x0E);
  writeRegister(0x45, 0x0F); writeRegister(0x44, 0x42); writeRegister(0x4C, 0x80);
  writeRegister(0x7F, 0x10); writeRegister(0x5B, 0x02); writeRegister(0x7F, 0x07);
  writeRegister(0x40, 0x41); writeRegister(0x70, 0x00);
  delay(100);
  writeRegister(0x32, 0x44); writeRegister(0x7F, 0x07); writeRegister(0x40, 0x40);
  writeRegister(0x7F, 0x06); writeRegister(0x62, 0xF0); writeRegister(0x63, 0x00);
  writeRegister(0x7F, 0x0D); writeRegister(0x48, 0xC0); writeRegister(0x6F, 0xD5);
  writeRegister(0x7F, 0x00); writeRegister(0x5B, 0xA0); writeRegister(0x4E, 0xA8);
  writeRegister(0x5A, 0x50); writeRegister(0x40, 0x80);
}

// ---------------------------------------------------------------------------
// Genera segnali encoder AB basati sul movimento X
// ---------------------------------------------------------------------------
void updateEncoderOutputs(int16_t deltaX) {
  if (deltaX == 0) return;
  int steps = deltaX * 4;
  if (xSemaphoreTake(registerMutex, portMAX_DELAY) == pdTRUE) {
    encoderPosition += steps;
    xSemaphoreGive(registerMutex);
  }
  for (int i = 0; i < abs(steps); i++) {
    int phase = abs(encoderPosition) & 0x03;
    if (steps > 0) {
      switch (phase) {
        case 0: encoderA_state = false; encoderB_state = false; break;
        case 1: encoderA_state = true;  encoderB_state = false; break;
        case 2: encoderA_state = true;  encoderB_state = true;  break;
        case 3: encoderA_state = false; encoderB_state = true;  break;
      }
    } else {
      switch (phase) {
        case 0: encoderA_state = false; encoderB_state = false; break;
        case 3: encoderA_state = false; encoderB_state = true;  break;
        case 2: encoderA_state = true;  encoderB_state = true;  break;
        case 1: encoderA_state = true;  encoderB_state = false; break;
      }
    }
    digitalWrite(ENCODER_A_PIN, encoderA_state);
    digitalWrite(ENCODER_B_PIN, encoderB_state);
    updateRGBLED(encoderA_state, encoderB_state);
    delayMicroseconds(100);
  }
}

// Aggiorna LED RGB in base allo stato encoder A/B
void updateRGBLED(bool encoderA, bool encoderB) {
  if (encoderA && encoderB)       rgb_led.setPixelColor(0, 255, 255, 0);  // Giallo
  else if (encoderA)              rgb_led.setPixelColor(0, 255, 0,   0);  // Rosso
  else if (encoderB)              rgb_led.setPixelColor(0, 0,   255, 0);  // Verde
  else                            rgb_led.setPixelColor(0, 0,   0,   0);  // Spento
  rgb_led.show();
}
