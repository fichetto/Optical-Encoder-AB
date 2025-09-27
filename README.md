# 🔄 ESP32-S3 Optical Encoder AB

Un encoder ottico digitale basato su ESP32-S3 e sensore PMW3901, con interfaccia WiFi per calibrazione e monitoraggio in tempo reale.

![ESP32-S3](https://img.shields.io/badge/ESP32--S3-Dual%20Core-blue)
![PlatformIO](https://img.shields.io/badge/PlatformIO-Arduino%20Framework-orange)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Working-brightgreen)

## 📋 Descrizione

Questo progetto implementa un **encoder ottico ad alta precisione** utilizzando il sensore PMW3901 (Pimoroni) su una scheda ESP32-S3-Nano Waveshare. Il sistema simula i classici segnali A e B di un encoder incrementale, con interfaccia web per calibrazione e monitoraggio.

### ✨ Caratteristiche Principali

- 🎯 **Alta Precisione**: Tracking ottico fino a 7000 FPS
- 🔄 **Segnali A/B**: Simulazione encoder incrementale classico
- 📡 **WiFi Integrato**: Access Point per calibrazione remota  
- 🎨 **Feedback LED**: Visualizzazione stato tramite LED rosso integrato
- ⚡ **Dual Core**: Task separati per sensore e WiFi
- 🔧 **Auto-tuning**: Ottimizzazione automatica qualità segnale (SQUAL)
- 📊 **Monitoraggio Real-time**: SQUAL, DX/DY, posizione encoder

## 🛠️ Hardware Richiesto

### Componenti Principali
- **ESP32-S3-Nano Waveshare** - Microcontrollore dual-core
- **PMW3901 Breakout (Pimoroni)** - Sensore ottico flow
- **Cavi jumper** - Per collegamenti SPI

### Pinout SPI
```
ESP32-S3    →    PMW3901
GPIO 12     →    SCK  (Clock)
GPIO 11     →    MOSI (Data Out) 
GPIO 13     →    MISO (Data In)
GPIO 10     →    CS   (Chip Select)
3.3V        →    VCC
GND         →    GND
```

### LED Integrati
- **LED Rosso Scheda**: GPIO 48 - Indica attività encoder
- **LED PMW3901**: Controllabili via web interface

## 🚀 Installazione

### 1. Setup PlatformIO
```bash
# Clona il repository
git clone https://github.com/[YOUR_USERNAME]/ESP32-S3-Optical-Encoder-AB.git
cd ESP32-S3-Optical-Encoder-AB

# Compila il progetto
pio run

# Carica sulla scheda
pio run -t upload

# Monitor seriale
pio device monitor
```

### 2. Dipendenze Automatiche
Il file `platformio.ini` installa automaticamente:
- `Adafruit NeoPixel` - Controllo LED
- `WiFi` - Networking ESP32
- `WebServer` - Server HTTP integrato

## 📡 Utilizzo

### 1. Prima Connessione
1. **Alimenta la scheda** - Il LED rosso dovrebbe rimanere spento
2. **Connetti al WiFi**: 
   - SSID: `ESP32-Optical-Encoder`
   - Password: `encoder123`
3. **Apri browser**: http://192.168.4.1

### 2. Interfaccia Web

#### Pagina Principale
```
📊 ESP32 Optical Encoder Monitor
├── Status Sensore: ✅ Attivo
├── SQUAL Quality: 85 (Ottimo: >40)
├── Movimento: DX=+5, DY=-2
├── Posizione Encoder: 1250 passi
└── [Pulsanti controllo LED]
```

#### Calibrazione
1. **Misura distanza nota** (es. righello da 10cm)  
2. **Muovi il sensore** sulla distanza misurata
3. **Inserisci valore** nella pagina di calibrazione
4. **Sistema calcola** automaticamente pixel/mm

### 3. Monitoraggio Seriale
```
Movimento rilevato: DX=+3, DY=-1, SQUAL=67
Encoder A: HIGH, B: LOW → Posizione: +1245
LED: Attivo (movimento rilevato)
```

## ⚙️ Configurazione Avanzata

### Soglie di Movimento
```cpp
#define ENCODER_THRESHOLD    2     // Soglia minima movimento (pixel)
#define SQUAL_GOOD_THRESHOLD 40    // SQUAL minimo accettabile
```

### Task FreeRTOS
```cpp
// Core 0: Sensore PMW3901 (20Hz)
sensorTask (Stack: 8192, Priority: 1)

// Core 1: WiFi + Web Server (10Hz)  
wifiTask (Stack: 16384, Priority: 1)
```

### Watchdog Timer
```cpp
CONFIG_ESP_TASK_WDT_TIMEOUT_S=10    // Timeout 10 secondi
CONFIG_ESP_TASK_WDT_PANIC=y         // Panic on timeout
```

## 🧪 Testing & Debug

### Test Funzionale LED
```cpp
// All'avvio - Test visivo LED
rgb_led.clear();        // LED spento ✅
rgb_led.show();
Serial.println("LED RGB encoder pronto (spento)!");
```

### Qualità Segnale SQUAL
- **SQUAL > 80**: 🟢 Eccellente
- **SQUAL 40-80**: 🟡 Buono  
- **SQUAL < 40**: 🔴 Scarso (auto-ottimizzazione)

### Diagnostica Problemi
```bash
# Monitor debug completo
pio device monitor --baud 115200

# Verifica memoria
ESP.getFreeHeap()     # RAM disponibile
ESP.getFlashChipSize() # Flash size
```

## 📚 API Reference

### Funzioni Principali
```cpp
// Inizializzazione
void initPMW3901Registers();  // Setup sensore Bitcraze
void initPMW3901LEDs();       // LED sensore

// Lettura Sensore  
MotionData readPMW3901();     // Legge DX, DY, SQUAL
void updateEncoderPosition(); // Calcola segnali A/B

// LED Control
void updateRGBLED(bool A, bool B);  // Feedback visivo
void setPMW3901LEDs(bool enable);   // LED sensore
```

### Endpoint Web API
```http
GET  /                 → Dashboard principale
POST /calibrate       → Salva calibrazione  
GET  /led-on          → Accendi LED sensore
GET  /led-off         → Spegni LED sensore
GET  /reset           → Reset registri posizione
```

## 🔧 Troubleshooting

### Problemi Comuni

#### SQUAL Basso (< 40)
```cpp
// Auto-ottimizzazione attiva:
1. Controllo LED massima potenza
2. Reset buffer sensore  
3. Re-inizializzazione completa
```

#### Reset MCU Continui
```cpp
// Soluzioni implementate:
- Stack size aumentati (8192/16384)
- Task frequency ridotte (20Hz/10Hz)  
- Watchdog timeout 10s
- taskYIELD() per context switch
```

#### LED Non Funzionante
```cpp
// Verifica pinout:
#define RGB_LED_PIN 48  // Standard ESP32-S3
// Alternative: GPIO 38, GPIO 21
```

#### Connessione WiFi
```bash
# Verifica Access Point:
SSID: ESP32-Optical-Encoder
IP: 192.168.4.1
Gateway: 192.168.4.1
```

## 📈 Performance

### Specifiche Tecniche
- **Risoluzione**: 30x30 pixel array
- **Frame Rate**: Fino a 7000 FPS  
- **Range Velocità**: 0.3 - 7.2 m/s
- **Precisione**: ±1 pixel
- **Latenza**: < 50ms end-to-end

### Benchmarks Testati
```
✅ Tracking: Eccellente (SQUAL 50-123)
✅ Stabilità: Nessun reset in 24h
✅ Responsività Web: < 200ms  
✅ Memory Usage: 45% RAM, 32% Flash
✅ Task Switching: < 1ms
```

## 🤝 Contribuire

1. **Fork** il repository
2. **Crea branch** feature (`git checkout -b feature/amazing-feature`)
3. **Commit** modifiche (`git commit -m 'Add amazing feature'`)
4. **Push** al branch (`git push origin feature/amazing-feature`)
5. **Apri Pull Request**

### Coding Standards
- Commenti in italiano per logica business
- Variabili in camelCase
- Costanti in UPPER_CASE
- Indentazione 2 spazi

## 📄 Licenza

Questo progetto è rilasciato sotto licenza **MIT** - vedi [LICENSE](LICENSE) per dettagli.

## 👨‍💻 Autore

**Sviluppatore**: [Il Tuo Nome]
- 📧 Email: [mino.m@tecnocons.com]
- 🐙 GitHub: [@tuo-username](https://github.com/fichetto)
- 💼 LinkedIn: [Il Tuo Profilo](https://www.linkedin.com/in/cosimo-massimiliano-84126b34/)

## 🙏 Ringraziamenti

- **Pimoroni** per il fantastico breakout PMW3901
- **Bitcraze** per la sequenza di inizializzazione del sensore
- **Waveshare** per la scheda ESP32-S3-Nano
- **Espressif** per l'eccellente framework Arduino-ESP32

## 📚 Riferimenti

- [PMW3901 Datasheet](https://www.pixart.com/products-detail/10/PMW3901MB-TXQT)
- [ESP32-S3 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)
- [Pimoroni PMW3901 Guide](https://shop.pimoroni.com/products/pmw3901-optical-flow-sensor-breakout)
- [Waveshare ESP32-S3-Nano](https://www.waveshare.com/wiki/ESP32-S3-Nano)

---

⭐ **Se questo progetto ti è stato utile, lascia una stella!** ⭐
