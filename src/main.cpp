#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_NeoPixel.h>
#include "calibration.h"

// Pin definitions per ESP32-S3-Nano Waveshare
#define PMW3901_CS    11
#define PMW3901_SCK   12  
#define PMW3901_MOSI  13
#define PMW3901_MISO  14

// Pin encoder AB output (simulano encoder tradizionale)
#define ENCODER_A_PIN  2   // Canale A dell'encoder
#define ENCODER_B_PIN  3   // Canale B dell'encoder (90° sfasato)

// LED RGB WS2812 integrato sulla scheda ESP32-S3-DevKitC-1
#define RGB_LED_PIN    48  // GPIO 48 - LED RGB WS2812
#define NUM_LEDS       1   // Un solo LED RGB sulla scheda

// Oggetto NeoPixel
Adafruit_NeoPixel rgb_led(NUM_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// PMW3901 registers
#define PMW3901_PRODUCT_ID        0x00
#define PMW3901_REVISION_ID       0x01
#define PMW3901_MOTION            0x02
#define PMW3901_DELTA_X_L         0x03
#define PMW3901_DELTA_X_H         0x04
#define PMW3901_DELTA_Y_L         0x05
#define PMW3901_DELTA_Y_H         0x06

// PMW3901 quality registers per diagnostica fuoco
#define PMW3901_SQUAL             0x07   // Surface Quality 
#define PMW3901_RAW_DATA_SUM      0x08   // Raw Data Sum
#define PMW3901_MAXIMUM_RAW_DATA  0x09   // Maximum Raw Data
#define PMW3901_MINIMUM_RAW_DATA  0x0A   // Minimum Raw Data
#define PMW3901_SHUTTER_UPPER     0x0B   // Shutter Upper
#define PMW3901_SHUTTER_LOWER     0x0C   // Shutter Lower

// PMW3901 LED control registers (banco 0x14)
#define PMW3901_LED_BANK          0x7F
#define PMW3901_LED_CONTROL       0x6F

SPIClass *spi;

// Registri accumulatori per gli incrementi
volatile long registerX = 0;  // Registro cumulativo incrementi X
volatile long registerY = 0;  // Registro cumulativo incrementi Y

// Filtro outlier - Finestra mobile per rilevamento anomalie
#define OUTLIER_WINDOW_SIZE 10       // Numero campioni per mediana
#define OUTLIER_MAD_THRESHOLD 3.0    // Soglia MAD (3 = ~99.7% dei dati normali)
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

  // Calcola mediana di un array (non modifica l'array originale)
  int16_t calculateMedian(int16_t* values, uint8_t size) {
    int16_t sorted[OUTLIER_WINDOW_SIZE];
    memcpy(sorted, values, size * sizeof(int16_t));

    // Bubble sort semplice (efficiente per array piccoli)
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

  // Calcola MAD (Median Absolute Deviation)
  float calculateMAD(int16_t* values, uint8_t size, int16_t median) {
    int16_t deviations[OUTLIER_WINDOW_SIZE];
    for (uint8_t i = 0; i < size; i++) {
      deviations[i] = abs(values[i] - median);
    }
    return (float)calculateMedian(deviations, size);
  }

  // Filtra un campione - restituisce true se valido, false se outlier
  bool filterSample(int16_t deltaX, int16_t deltaY, int16_t* filteredX, int16_t* filteredY) {
    totalSamples++;

    // Riempi finestra iniziale senza filtrare (primi campioni)
    if (totalSamples <= OUTLIER_WINDOW_SIZE) {
      windowX[windowIndex] = deltaX;
      windowY[windowIndex] = deltaY;
      windowIndex = (windowIndex + 1) % OUTLIER_WINDOW_SIZE;
      *filteredX = deltaX;
      *filteredY = deltaY;
      return true;
    }

    // Calcola mediana e MAD per X
    int16_t medianX = calculateMedian(windowX, OUTLIER_WINDOW_SIZE);
    float madX = calculateMAD(windowX, OUTLIER_WINDOW_SIZE, medianX);

    // Calcola mediana e MAD per Y
    int16_t medianY = calculateMedian(windowY, OUTLIER_WINDOW_SIZE);
    float madY = calculateMAD(windowY, OUTLIER_WINDOW_SIZE, medianY);

    // Rileva outlier usando soglia MAD
    bool isOutlierX = (madX > 0) && (abs(deltaX - medianX) > OUTLIER_MAD_THRESHOLD * madX);
    bool isOutlierY = (madY > 0) && (abs(deltaY - medianY) > OUTLIER_MAD_THRESHOLD * madY);

    // Gestisci outlier
    if (isOutlierX) {
      outlierCountX++;
      *filteredX = medianX;  // Sostituisci con mediana
    } else {
      *filteredX = deltaX;
    }

    if (isOutlierY) {
      outlierCountY++;
      *filteredY = medianY;  // Sostituisci con mediana
    } else {
      *filteredY = deltaY;
    }

    // Aggiorna finestra mobile
    windowX[windowIndex] = *filteredX;
    windowY[windowIndex] = *filteredY;
    windowIndex = (windowIndex + 1) % OUTLIER_WINDOW_SIZE;

    return !(isOutlierX || isOutlierY);  // true se nessun outlier rilevato
  }

  // Statistiche filtro
  float getOutlierRateX() { return totalSamples > 0 ? (100.0f * outlierCountX / totalSamples) : 0.0f; }
  float getOutlierRateY() { return totalSamples > 0 ? (100.0f * outlierCountY / totalSamples) : 0.0f; }
  void resetStats() { outlierCountX = 0; outlierCountY = 0; totalSamples = 0; }
};

OutlierFilter outlierFilter;

// Sistema encoder AB
volatile long encoderPosition = 0;  // Posizione encoder in step
volatile bool encoderA_state = false;
volatile bool encoderB_state = false;
volatile unsigned long lastEncoderUpdate = 0;

// Sistema dual core
TaskHandle_t WiFiTask = NULL;
TaskHandle_t SensorTask = NULL;
SemaphoreHandle_t registerMutex = NULL;

// WiFi Access Point per calibrazione
WebServer server(80);
volatile bool wifiAPStarted = false;
volatile bool wifiAPRequested = false;
volatile unsigned long wifiAPStartTime = 0;
volatile unsigned long lastWiFiActivity = 0;  // Timestamp ultima attività
#define WIFI_AP_TIMEOUT_MS 180000  // 3 minuti timeout

// Sistema di calibrazione
EncoderCalibration calibration;

// LED Status
volatile bool pmw3901LedsEnabled = false;  // Default: LED spenti all'avvio (come richiesto)

// Function declaration
uint8_t readRegister(uint8_t reg);
void writeRegister(uint8_t reg, uint8_t data);
void sensorTask(void *pvParameters);
void wifiTask(void *pvParameters);
void updateEncoderOutputs(int16_t deltaX);
void updateRGBLED(bool encoderA, bool encoderB);
void setupWebServer();
void setPMW3901LEDs(bool enable);
void initPMW3901LEDs();
void initPMW3901Registers();  // Sequenza inizializzazione Bitcraze

void setup() {
  // Initialize both serial ports
  Serial.begin(115200);
  #if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);  // Don't wait for Serial if no USB connection
  #endif
  
  delay(3000);  // Longer delay to ensure serial is ready
  
  Serial.println("=== PMW3901 Flow Sensor Monitor ===");
  Serial.println("Avvio sistema dual-core...");
  Serial.flush();
  
  // Inizializza pin encoder AB
  pinMode(ENCODER_A_PIN, OUTPUT);
  pinMode(ENCODER_B_PIN, OUTPUT);
  digitalWrite(ENCODER_A_PIN, LOW);
  digitalWrite(ENCODER_B_PIN, LOW);
  
  // Inizializza LED RGB WS2812
  // Inizializza LED RGB WS2812
  rgb_led.begin();           // Inizializza il LED RGB
  rgb_led.setBrightness(50); // Luminosità media (0-255) - NEOPIXEL
  rgb_led.clear();           // Assicurati che sia spento all'avvio 
  rgb_led.show();            // Applica (LED spento)
  
  // Test LED RGB all'avvio (PARTIRE CON LED SPENTI come richiesto)
  Serial.println("Test LED RGB encoder...");
  rgb_led.clear(); // LED SPENTO all'avvio
  rgb_led.show();
  Serial.println("LED RGB encoder pronto (spento)!");
  
  // Inizializza sistema di calibrazione
  calibration.begin();
  
  // Crea mutex per sincronizzazione registri
  registerMutex = xSemaphoreCreateMutex();
  if (registerMutex == NULL) {
    Serial.println("ERRORE: Impossibile creare mutex!");
    while(1);
  }
  
  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());
  Serial.flush();
  
  // Initialize SPI
  spi = new SPIClass(HSPI);
  spi->begin(PMW3901_SCK, PMW3901_MISO, PMW3901_MOSI, PMW3901_CS);
  
  pinMode(PMW3901_CS, OUTPUT);
  digitalWrite(PMW3901_CS, HIGH);
  
  Serial.println("SPI initialized");
  Serial.flush();
  delay(100);
  
  // Test communication
  Serial.println("Inizializzazione PMW3901...");
  uint8_t productID = readRegister(PMW3901_PRODUCT_ID);
  uint8_t revisionID = readRegister(PMW3901_REVISION_ID);
  
  if (productID == 0x49) {
    Serial.println("PMW3901 rilevato correttamente");
    Serial.print("Revision ID: 0x");
    Serial.println(revisionID, HEX);
    
    // Sequenza inizializzazione CORRETTA da Bitcraze (testata e funzionante)
    Serial.println("Inizializzazione registri PMW3901 (Bitcraze sequence)...");
    
    // Power on reset (come da Bitcraze)
    writeRegister(0x3A, 0x5A);
    delay(5);
    
    // Leggi registri per svuotare buffer (come da Bitcraze)
    readRegister(0x02);
    readRegister(0x03);
    readRegister(0x04);
    readRegister(0x05);
    readRegister(0x06);
    delay(1);
    
    // INIZIALIZZAZIONE COMPLETA BITCRAZE (Secret Sauce)
    Serial.println("Applicazione secret sauce Bitcraze...");
    initPMW3901Registers();
    
    // Test finale
    uint8_t testShutter = readRegister(PMW3901_SHUTTER_UPPER);
    uint8_t testShutterLow = readRegister(PMW3901_SHUTTER_LOWER);
    uint8_t testSQUAL = readRegister(PMW3901_SQUAL);
    Serial.printf("Dopo init Bitcraze - Shutter: %d/%d, SQUAL: %d\n", testShutter, testShutterLow, testSQUAL);
    
  } else {
    Serial.print("ERRORE: Product ID = 0x");
    Serial.println(productID, HEX);
  }
  
  // Inizializza SPI per PMW3901
  spi = new SPIClass(HSPI);
  spi->begin(PMW3901_SCK, PMW3901_MISO, PMW3901_MOSI, PMW3901_CS);
  pinMode(PMW3901_CS, OUTPUT);
  digitalWrite(PMW3901_CS, HIGH);
  
  // Inizializza LED PMW3901 integrati
  initPMW3901LEDs();
  
  // Avvia task sui due core
  Serial.println("Avvio sistema dual core...");
  
  // Core 0: Task sensore e encoder (alta priorità)
  xTaskCreatePinnedToCore(
    sensorTask,    // Funzione del task
    "SensorTask",  // Nome del task
    8192,          // Stack size RADDOPPIATO per stabilità
    NULL,          // Parametri
    2,             // Priorità alta
    &SensorTask,   // Handle del task
    0              // Core 0
  );
  
  // Core 1: Task WiFi (bassa priorità)
  xTaskCreatePinnedToCore(
    wifiTask,      // Funzione del task
    "WiFiTask",    // Nome del task
    16384,         // Stack size AUMENTATO per WiFi
    NULL,          // Parametri
    1,             // Priorità bassa
    &WiFiTask,     // Handle del task
    1              // Core 1
  );
  
  Serial.println("Sistema avviato - Monitoraggio accumulatori X/Y:");
  Serial.println();  // Riga vuota per separare dall'output dei dati
  
  // Monitor memoria per debug reset
  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());
}

// Nuovo loop principale - gestisce solo trigger richieste WiFi AP
void loop() {
  static unsigned long lastStatus = 0;
  
  // Avvio immediato WiFi AP all'avvio del sistema
  static bool immediateAPStarted = false;
  if (!immediateAPStarted && millis() > 5000) {  // 5 secondi dopo il boot
    wifiAPRequested = true;
    immediateAPStarted = true;
  }
  
  delay(1000);  // Loop lento per risparmiare CPU
}

// Task Core 0: Gestione sensore PMW3901 e generazione segnali encoder AB
void sensorTask(void *pvParameters) {
  
  // Test comunicazione PMW3901
  delay(100);
  uint8_t productID = readRegister(PMW3901_PRODUCT_ID);
  if (productID != 0x49) {
    // Sensore non trovato, ferma il task
    vTaskDelete(NULL);
  }
  
  // Contatori per diagnostica qualità sensore - SISTEMA ANTI-BLOCCO POTENZIATO
  uint32_t cycleCount = 0;
  uint32_t poorQualityCount = 0;
  uint32_t lastResetTime = 0;
  uint32_t totalReadings = 0;  // Contatore totale letture per reset preventivo
  
  while (1) {
    // Incrementa contatori
    cycleCount++;
    totalReadings++;
    
    // Yield every 10 cycles per lasciare CPU ad altri task
    if (cycleCount % 10 == 0) {
      taskYIELD();  // Forza context switch
    }
    
    // Leggi movimento dal sensore
    uint8_t motion = readRegister(PMW3901_MOTION);
    vTaskDelay(pdMS_TO_TICKS(1));  // Piccolo delay tra letture SPI

    int16_t deltaX = (int16_t)((readRegister(PMW3901_DELTA_X_H) << 8) | readRegister(PMW3901_DELTA_X_L));
    vTaskDelay(pdMS_TO_TICKS(1));  // Piccolo delay tra letture SPI

    int16_t deltaY = (int16_t)((readRegister(PMW3901_DELTA_Y_H) << 8) | readRegister(PMW3901_DELTA_Y_L));

    // FILTRO OUTLIER - Rimuovi spike anomali
    int16_t filteredX, filteredY;
    bool isClean = outlierFilter.filterSample(deltaX, deltaY, &filteredX, &filteredY);

    // Log outlier rilevati (solo se significativi)
    if (!isClean && (abs(deltaX) > 10 || abs(deltaY) > 10)) {
      Serial.printf("OUTLIER filtrato: dX=%d->%d, dY=%d->%d\n", deltaX, filteredX, deltaY, filteredY);
    }

    // Usa valori filtrati per il resto dell'elaborazione
    deltaX = filteredX;
    deltaY = filteredY;

    // CONTROLLO ANTI-BLOCCO PIÙ FREQUENTE ogni 40 cicli (circa ogni 2 secondi)
    if (cycleCount % 40 == 0) {
      uint8_t squal = readRegister(PMW3901_SQUAL);
      uint8_t shutterUpper = readRegister(PMW3901_SHUTTER_UPPER);
      uint8_t shutterLower = readRegister(PMW3901_SHUTTER_LOWER);
      uint8_t rawSum = readRegister(PMW3901_RAW_DATA_SUM);
      
      // Rileva condizioni di BLOCCO TOTALE (più aggressivo)
      bool sensorBlocked = (squal == 0) ||  // SQUAL completamente zero = BLOCCO
                           (rawSum == 0) ||   // Nessun dato raw = BLOCCO
                           (shutterUpper == 132 && shutterLower == 3); // Valori fissi = BLOCCO
      
      if (sensorBlocked) {
        poorQualityCount++;
        Serial.printf("BLOCCO RILEVATO (tentativo %d): SQUAL=%d, Raw=%d, Shutter=%d/%d\n", 
                      poorQualityCount, squal, rawSum, shutterUpper, shutterLower);
        
        // RESET IMMEDIATO dopo 3 controlli consecutivi (6 secondi di blocco)
        if (poorQualityCount >= 3 && (millis() - lastResetTime) > 10000) {
          Serial.println("RESET SENSORE: Blocco confermato - reset immediato!");
          
          // Reset completo con power cycle
          writeRegister(0x3A, 0x5A);  // Power up reset
          delay(200);
          
          // Ri-inizializzazione completa Bitcraze
          initPMW3901Registers();
          delay(100);
          
          // Reset accumulatori se overflow
          if (xSemaphoreTake(registerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (abs(registerX) > 500000 || abs(registerY) > 500000) {
              Serial.println("RESET ACCUMULATORI: Overflow rilevato");
              registerX = 0;
              registerY = 0;
              encoderPosition = 0;
            }
            xSemaphoreGive(registerMutex);
          }
          
          poorQualityCount = 0;
          lastResetTime = millis();
          Serial.println("Reset sensore completato - ripresa funzionamento");
        }
      } else {
        // Reset contatore se qualità migliora
        if (poorQualityCount > 0) {
          Serial.println("Qualità sensore ripristinata");
          poorQualityCount = 0;
        }
      }
    }
    
    // RESET PREVENTIVO ogni 10,000 letture (~8 minuti a 20Hz)
    if (totalReadings > 0 && totalReadings % 10000 == 0) {
      Serial.printf("RESET PREVENTIVO: %d letture completate - manutenzione automatica\n", totalReadings);
      
      // Pulizia preventiva buffer
      for(int i = 0; i < 5; i++) {
        readRegister(PMW3901_MOTION);
        readRegister(PMW3901_DELTA_X_L);
        readRegister(PMW3901_DELTA_X_H);
        readRegister(PMW3901_DELTA_Y_L);
        readRegister(PMW3901_DELTA_Y_H);
        delay(10);
      }
      Serial.println("Manutenzione preventiva completata");
    }
    
    // Aggiorna registri se c'è movimento
    if ((motion & 0x80) || (deltaX != 0) || (deltaY != 0)) {
      if (deltaX != 0 || deltaY != 0) {
        // Aggiorna registri accumulatori (thread-safe)
        if (xSemaphoreTake(registerMutex, portMAX_DELAY) == pdTRUE) {
          registerX += deltaX;
          registerY += deltaY;
          xSemaphoreGive(registerMutex);
        }
        
        // Genera segnali encoder AB per l'asse X
        updateEncoderOutputs(deltaX);
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz update rate (molto più conservativo)
  }
}

// Task Core 1: Gestione WiFi Access Point
void wifiTask(void *pvParameters) {
  
  while (1) {
    // Controlla se è richiesto l'avvio dell'AP
    if (wifiAPRequested && !wifiAPStarted) {
      
      WiFi.mode(WIFI_AP);
      WiFi.softAP("EncoderCalibration", "12345678");
      
      IPAddress IP = WiFi.softAPIP();
      
      setupWebServer();
      server.begin();
      
      wifiAPStarted = true;
      wifiAPStartTime = millis();
      lastWiFiActivity = millis();  // Inizializza ultima attività
      wifiAPRequested = false;
    }
    
    // Gestisce richieste HTTP se AP attivo
    if (wifiAPStarted) {
      server.handleClient();
      
      // Auto-shutdown dopo timeout dalla ULTIMA ATTIVITÀ
      unsigned long timeSinceActivity = millis() - lastWiFiActivity;
      if (timeSinceActivity > WIFI_AP_TIMEOUT_MS) {
        WiFi.mode(WIFI_OFF);
        wifiAPStarted = false;
      }
    }
    
    // Output continuo degli accumulatori X e Y + diagnostica qualità (ogni 500ms - meno frequente)
    static unsigned long lastAccumulatorPrint = 0;
    if (millis() - lastAccumulatorPrint > 500) {
      // Leggi i valori accumulatori in modo thread-safe
      int32_t currentX, currentY;
      if (xSemaphoreTake(registerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        currentX = registerX;
        currentY = registerY;
        xSemaphoreGive(registerMutex);

        // Leggi solo i parametri essenziali per diagnostica
        uint8_t motion = readRegister(PMW3901_MOTION);
        uint8_t squal = readRegister(PMW3901_SQUAL);

        // Output essenziale: accumulatori + qualità + diagnostica
        Serial.print("X: ");
        Serial.print(currentX);
        Serial.print(" | Y: ");
        Serial.print(currentY);
        Serial.print(" | SQUAL: ");
        Serial.print(squal);

        // Aggiungi informazioni diagnostiche per debug
        uint8_t rawSum = readRegister(PMW3901_RAW_DATA_SUM);
        uint8_t shutterUpper = readRegister(PMW3901_SHUTTER_UPPER);
        uint8_t shutterLower = readRegister(PMW3901_SHUTTER_LOWER);

        Serial.print(" | Raw: ");
        Serial.print(rawSum);
        Serial.print(" | Shutter: ");
        Serial.print(shutterUpper);
        Serial.print("/");
        Serial.print(shutterLower);

        // Mostra motion solo se significativo
        if (motion & 0x80) {
          Serial.print(" | Motion: ATTIVO");
        } else if (motion != 0) {
          Serial.print(" | Motion: 0x");
          Serial.print(motion, HEX);
        }

        // Statistiche filtro outlier
        Serial.print(" | Outlier: X=");
        Serial.print(outlierFilter.getOutlierRateX(), 1);
        Serial.print("%, Y=");
        Serial.print(outlierFilter.getOutlierRateY(), 1);
        Serial.print("%");

        Serial.println();
      }
      lastAccumulatorPrint = millis();
    }
    
    // Output dei delta istantanei solo per movimenti significativi
    static unsigned long lastDeltaPrint = 0;
    if (millis() - lastDeltaPrint > 500) { // Ogni 500ms (molto meno frequente)
      uint8_t motion = readRegister(PMW3901_MOTION);
      if (motion & 0x80) { // Se c'è movimento
        int16_t deltaX = (int16_t)((readRegister(PMW3901_DELTA_X_H) << 8) | readRegister(PMW3901_DELTA_X_L));
        int16_t deltaY = (int16_t)((readRegister(PMW3901_DELTA_Y_H) << 8) | readRegister(PMW3901_DELTA_Y_L));
        
        // Mostra solo movimenti significativi
        if (abs(deltaX) > 1 || abs(deltaY) > 1) {
          Serial.print("  → Movimento: dX=");
          Serial.print(deltaX);
          Serial.print(", dY=");
          Serial.println(deltaY);
        }
      }
      lastDeltaPrint = millis();
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz update rate per WiFi (meno aggressivo per stabilità)
  }
}

// Genera segnali encoder AB basati sul movimento X
void updateEncoderOutputs(int16_t deltaX) {
  if (deltaX == 0) return;
  
  // Converti pixel in step encoder (esempio: 1 pixel = 4 step)
  int steps = deltaX * 4;
  
  if (xSemaphoreTake(registerMutex, portMAX_DELAY) == pdTRUE) {
    encoderPosition += steps;
    xSemaphoreGive(registerMutex);
  }
  
  // Genera pattern quadrature per ogni step
  for (int i = 0; i < abs(steps); i++) {
    // Pattern encoder quadrature AB
    // Avanti: A=0,B=0 -> A=1,B=0 -> A=1,B=1 -> A=0,B=1 -> repeat
    // Indietro: sequenza inversa
    
    int phase = abs(encoderPosition) & 0x03;  // 4 stati (0-3)
    
    if (steps > 0) {  // Movimento positivo
      switch (phase) {
        case 0: encoderA_state = false; encoderB_state = false; break;
        case 1: encoderA_state = true;  encoderB_state = false; break;
        case 2: encoderA_state = true;  encoderB_state = true;  break;
        case 3: encoderA_state = false; encoderB_state = true;  break;
      }
    } else {  // Movimento negativo
      switch (phase) {
        case 0: encoderA_state = false; encoderB_state = false; break;
        case 3: encoderA_state = false; encoderB_state = true;  break;
        case 2: encoderA_state = true;  encoderB_state = true;  break;
        case 1: encoderA_state = true;  encoderB_state = false; break;
      }
    }
    
    // Aggiorna uscite fisiche
    digitalWrite(ENCODER_A_PIN, encoderA_state);
    digitalWrite(ENCODER_B_PIN, encoderB_state);
    
    // Aggiorna LED RGB per visualizzazione encoder
    updateRGBLED(encoderA_state, encoderB_state);
    
    delayMicroseconds(100);  // Timing per la lettura dell'encoder
  }
}

uint8_t readRegister(uint8_t reg) {
  // Controllo stabilità SPI
  if (!spi) {
    Serial.println("ERRORE: SPI non inizializzato!");
    return 0;
  }
  
  digitalWrite(PMW3901_CS, LOW);
  delayMicroseconds(50);  // Timing Bitcraze
  
  uint8_t cmd = reg & 0x7F;  // Read command (bit 7 = 0)
  spi->transfer(cmd);
  delayMicroseconds(50);      // Timing Bitcraze
  
  uint8_t data = spi->transfer(0x00);
  delayMicroseconds(100);     // Timing Bitcraze
  
  digitalWrite(PMW3901_CS, HIGH);
  
  return data;
}

void writeRegister(uint8_t reg, uint8_t data) {
  // Controllo stabilità SPI
  if (!spi) {
    Serial.println("ERRORE: SPI non inizializzato!");
    return;
  }
  
  digitalWrite(PMW3901_CS, LOW);
  delayMicroseconds(50);      // Timing Bitcraze
  
  uint8_t cmd = reg | 0x80;  // Write command (bit 7 = 1)
  spi->transfer(cmd);
  spi->transfer(data);
  delayMicroseconds(50);      // Timing Bitcraze
  
  digitalWrite(PMW3901_CS, HIGH);
  delayMicroseconds(200);     // Timing Bitcraze (importante!)
}

// Avvia Access Point per calibrazione
void startCalibrationAP() {
  if (wifiAPStarted) return;
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP("EncoderCalibration", "12345678");
  
  IPAddress IP = WiFi.softAPIP();
  
  setupWebServer();
  server.begin();
  
  wifiAPStarted = true;
}

// Configura il web server per la calibrazione
void setupWebServer() {
  // Pagina principale di calibrazione
  server.on("/", [](){
    lastWiFiActivity = millis();  // Aggiorna attività
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
        input { width: 100%; padding: 10px; border: 1px solid #ddd; border-radius: 5px; }
        button { width: 100%; padding: 12px; background: #007bff; color: white; border: none; border-radius: 5px; cursor: pointer; }
        button:hover { background: #0056b3; }
        .status { margin: 10px 0; padding: 10px; border-radius: 5px; }
        .info { background: #e7f3ff; border: 1px solid #b3d9ff; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Encoder Calibrazione</h1>
        
        <div class="status info">
            <strong>Distanza attuale:</strong> <span id="currentDistance">)" + String(calibration.getDistance(), 1) + R"( mm</span><br>
            <strong>Fattore scala X:</strong> <span id="scaleX">)" + String(calibration.getScaleX(), 4) + R"(</span><br>
            <strong>Fattore scala Y:</strong> <span id="scaleY">)" + String(calibration.getScaleY(), 4) + R"(</span><br>
            <strong>Filtro Outlier:</strong> X=)" + String(outlierFilter.getOutlierRateX(), 1) + R"(% | Y=)" + String(outlierFilter.getOutlierRateY(), 1) + R"(%</span>
        </div>
        
        <form action="/calibrate" method="POST">
            <div class="form-group">
                <label for="distance">Distanza sensore da superficie (mm):</label>
                <input type="number" id="distance" name="distance" min="80" max="2000" step="0.1" value=")" + String(calibration.getDistance(), 1) + R"(" required>
            </div>
            
            <button type="submit">Salva Calibrazione</button>
        </form>
        
        <div style="margin-top: 20px;">
            <h3>Controllo LED PMW3901</h3>
            <div class="form-group">
                <label>Stato LED: <strong>)" + String(pmw3901LedsEnabled ? "ACCESI" : "SPENTI") + R"(</strong></label>
                <button onclick="window.location.href='/led-on'" style="background: #28a745; margin-right: 10px;">Accendi LED</button>
                <button onclick="window.location.href='/led-off'" style="background: #6c757d;">Spegni LED</button>
            </div>
        </div>
        
        <div style="margin-top: 20px;">
            <button onclick="window.location.href='/reset'" style="background: #dc3545; margin-right: 10px;">Reset Registri</button>
            <button onclick="window.location.href='/reset-stats'" style="background: #ffc107;">Reset Statistiche Filtro</button>
        </div>
    </div>
</body>
</html>
    )";
    server.send(200, "text/html", html);
  });
  
  // Endpoint per salvare la calibrazione
  server.on("/calibrate", HTTP_POST, [](){
    lastWiFiActivity = millis();  // Aggiorna attività
    if (server.hasArg("distance")) {
      float distance = server.arg("distance").toFloat();
      calibration.setDistance(distance);
      String response = "<html><head><meta http-equiv='refresh' content='3; url=/'></head>";
      response += "<body><h2>Calibrazione salvata!</h2>";
      response += "<p>Distanza: " + String(distance, 1) + " mm</p>";
      response += "<p>Reindirizzamento automatico alla home in 3 secondi...</p>";
      response += "<a href='/'>← Torna subito alla home</a></body></html>";
      server.send(200, "text/html", response);
    } else {
      server.send(400, "text/html", "<h2>Parametro mancante</h2>");
    }
  });
  
  // Endpoint per reset registri
  server.on("/reset", [](){
    lastWiFiActivity = millis();  // Aggiorna attività
    if (xSemaphoreTake(registerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      registerX = 0;
      registerY = 0;
      encoderPosition = 0;
      xSemaphoreGive(registerMutex);
    }
    String response = "<html><head><meta http-equiv='refresh' content='2; url=/'></head>";
    response += "<body><h2>Registri resettati!</h2>";
    response += "<p>Reindirizzamento automatico alla home in 2 secondi...</p>";
    response += "<a href='/'>← Torna subito alla home</a></body></html>";
    server.send(200, "text/html", response);
  });
  
  // Endpoint per accendere LED PMW3901
  server.on("/led-on", [](){
    lastWiFiActivity = millis();  // Aggiorna attività
    pmw3901LedsEnabled = true;
    setPMW3901LEDs(true);
    String response = "<html><head><meta http-equiv='refresh' content='2; url=/'></head>";
    response += "<body><h2>LED PMW3901 ACCESI!</h2>";
    response += "<p>I LED integrati sulla breakout board sono ora attivi.</p>";
    response += "<p>Reindirizzamento automatico alla home in 2 secondi...</p>";
    response += "<a href='/'>← Torna subito alla home</a></body></html>";
    server.send(200, "text/html", response);
  });
  
  // Endpoint per spegnere LED PMW3901
  server.on("/led-off", [](){
    lastWiFiActivity = millis();  // Aggiorna attività
    pmw3901LedsEnabled = false;
    setPMW3901LEDs(false);
    String response = "<html><head><meta http-equiv='refresh' content='2; url=/'></head>";
    response += "<body><h2>LED PMW3901 SPENTI!</h2>";
    response += "<p>I LED integrati sulla breakout board sono ora disattivi.</p>";
    response += "<p>Reindirizzamento automatico alla home in 2 secondi...</p>";
    response += "<a href='/'>← Torna subito alla home</a></body></html>";
    server.send(200, "text/html", response);
  });

  // Endpoint per reset statistiche filtro outlier
  server.on("/reset-stats", [](){
    lastWiFiActivity = millis();  // Aggiorna attività
    outlierFilter.resetStats();
    String response = "<html><head><meta http-equiv='refresh' content='2; url=/'></head>";
    response += "<body><h2>Statistiche filtro resettate!</h2>";
    response += "<p>I contatori degli outlier sono stati azzerati.</p>";
    response += "<p>Reindirizzamento automatico alla home in 2 secondi...</p>";
    response += "<a href='/'>← Torna subito alla home</a></body></html>";
    server.send(200, "text/html", response);
  });
}

// Funzioni per controllo LED PMW3901 integrati
void setPMW3901LEDs(bool enable) {
  // Controllo LED CORRETTO da implementazione Bitcraze
  delay(200);  // Importante: delay prima del cambio LED
  writeRegister(0x7F, 0x14);  // Bank switch
  writeRegister(0x6F, enable ? 0x1C : 0x00);  // LED control (Bitcraze values)
  writeRegister(0x7F, 0x00);  // Torna al bank principale
}

void initPMW3901LEDs() {
  // Inizializza LED al valore di default
  setPMW3901LEDs(pmw3901LedsEnabled);
  Serial.print("LED PMW3901 inizializzati: ");
  Serial.println(pmw3901LedsEnabled ? "ACCESI" : "SPENTI");
}

// Sequenza di inizializzazione COMPLETA da Bitcraze (testata e funzionante!)
void initPMW3901Registers() {
  // Performance optimization registers - ESATTA sequenza Bitcraze
  writeRegister(0x7F, 0x00);
  writeRegister(0x61, 0xAD);
  writeRegister(0x7F, 0x03);
  writeRegister(0x40, 0x00);
  writeRegister(0x7F, 0x05);
  writeRegister(0x41, 0xB3);
  writeRegister(0x43, 0xF1);
  writeRegister(0x45, 0x14);
  writeRegister(0x5B, 0x32);
  writeRegister(0x5F, 0x34);
  writeRegister(0x7B, 0x08);
  writeRegister(0x7F, 0x06);
  writeRegister(0x44, 0x1B);
  writeRegister(0x40, 0xBF);
  writeRegister(0x4E, 0x3F);
  writeRegister(0x7F, 0x08);
  writeRegister(0x65, 0x20);
  writeRegister(0x6A, 0x18);
  writeRegister(0x7F, 0x09);
  writeRegister(0x4F, 0xAF);
  writeRegister(0x5F, 0x40);
  writeRegister(0x48, 0x80);
  writeRegister(0x49, 0x80);
  writeRegister(0x57, 0x77);
  writeRegister(0x60, 0x78);
  writeRegister(0x61, 0x78);
  writeRegister(0x62, 0x08);
  writeRegister(0x63, 0x50);
  writeRegister(0x7F, 0x0A);
  writeRegister(0x45, 0x60);
  writeRegister(0x7F, 0x00);
  writeRegister(0x4D, 0x11);
  writeRegister(0x55, 0x80);
  writeRegister(0x74, 0x1F);
  writeRegister(0x75, 0x1F);
  writeRegister(0x4A, 0x78);
  writeRegister(0x4B, 0x78);
  writeRegister(0x44, 0x08);
  writeRegister(0x45, 0x50);
  writeRegister(0x64, 0xFF);
  writeRegister(0x65, 0x1F);
  writeRegister(0x7F, 0x14);
  writeRegister(0x65, 0x60);
  writeRegister(0x66, 0x08);
  writeRegister(0x63, 0x78);
  writeRegister(0x7F, 0x15);
  writeRegister(0x48, 0x58);
  writeRegister(0x7F, 0x07);
  writeRegister(0x41, 0x0D);
  writeRegister(0x43, 0x14);
  writeRegister(0x4B, 0x0E);
  writeRegister(0x45, 0x0F);
  writeRegister(0x44, 0x42);
  writeRegister(0x4C, 0x80);
  writeRegister(0x7F, 0x10);
  writeRegister(0x5B, 0x02);
  writeRegister(0x7F, 0x07);
  writeRegister(0x40, 0x41);
  writeRegister(0x70, 0x00);
  
  delay(100); // Wait as in Bitcraze implementation
  
  writeRegister(0x32, 0x44);
  writeRegister(0x7F, 0x07);
  writeRegister(0x40, 0x40);
  writeRegister(0x7F, 0x06);
  writeRegister(0x62, 0xF0);
  writeRegister(0x63, 0x00);
  writeRegister(0x7F, 0x0D);
  writeRegister(0x48, 0xC0);
  writeRegister(0x6F, 0xD5);
  writeRegister(0x7F, 0x00);
  writeRegister(0x5B, 0xA0);
  writeRegister(0x4E, 0xA8);
  writeRegister(0x5A, 0x50);
  writeRegister(0x40, 0x80);
  
  Serial.println("Sequenza Bitcraze completata!");
}

// Aggiorna LED RGB in base allo stato encoder A/B
void updateRGBLED(bool encoderA, bool encoderB) {
  if (encoderA && encoderB) {
    // Entrambi attivi = GIALLO (Rosso + Verde)
    rgb_led.setPixelColor(0, 255, 255, 0);
  } else if (encoderA) {
    // Solo A attivo = ROSSO
    rgb_led.setPixelColor(0, 255, 0, 0);
  } else if (encoderB) {
    // Solo B attivo = VERDE  
    rgb_led.setPixelColor(0, 0, 255, 0);
  } else {
    // Nessun movimento = SPENTO
    rgb_led.setPixelColor(0, 0, 0, 0);
  }
  rgb_led.show();  // Aggiorna il LED fisico
}