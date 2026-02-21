# 🔄 ESP32-S3 Optical Encoder AB

Encoder ottico digitale basato su **ESP32-S3** e sensore **PMW3901**, con lettura contatori X/Y via **Modbus RTU** su USB e interfaccia WiFi per calibrazione.

![ESP32-S3](https://img.shields.io/badge/ESP32--S3-Dual%20Core-blue)
![PlatformIO](https://img.shields.io/badge/PlatformIO-Arduino%20Framework-orange)
![Modbus RTU](https://img.shields.io/badge/Modbus-RTU%20Slave-purple)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Working-brightgreen)

## 📋 Descrizione

Il sistema legge il flusso ottico dal sensore PMW3901 (100 Hz) e accumula i contatori X/Y. I dati sono accessibili via **Modbus RTU** attraverso la porta USB nativa dell'ESP32-S3, senza driver aggiuntivi. È inclusa un'applicazione Python desktop per il monitoraggio in tempo reale.

### ✨ Caratteristiche Principali

- 📦 **Modbus RTU Slave** — lettura contatori X/Y + SQUAL via USB CDC (TinyUSB)
- 🖥️ **Desktop App Python** — GUI tkinter con grafico in tempo reale
- ⚡ **100 Hz** — frequenza di campionamento del sensore
- 🔄 **Segnali A/B** — simulazione encoder incrementale su GPIO 2/3
- 📡 **WiFi AP** — pagina web per calibrazione distanza sensore
- 🎨 **LED RGB** — stato encoder visualizzato sul LED WS2812 integrato
- 🧮 **Filtro outlier MAD** — rimozione automatica picchi di rumore
- ⚙️ **Dual Core FreeRTOS** — sensorTask (Core 0) + wifiTask + Modbus (Core 1)

## 🛠️ Hardware

### Componenti
| Componente | Modello |
|---|---|
| Microcontrollore | ESP32-S3-DevKitC-1 (8 MB Flash) |
| Sensore ottico | PMW3901 Breakout (Pimoroni / Bitcraze) |

### Pinout SPI (HSPI)
```
ESP32-S3    →    PMW3901
GPIO 11     →    CS   (Chip Select)
GPIO 12     →    SCK  (Clock)
GPIO 13     →    MOSI (Data Out)
GPIO 14     →    MISO (Data In)
3.3V        →    VCC
GND         →    GND
```

### Pin Encoder AB Output
```
GPIO 2  →  Segnale A
GPIO 3  →  Segnale B
GPIO 48 →  LED RGB WS2812 (stato)
```

## 🚀 Installazione Firmware

### Requisiti
- [PlatformIO](https://platformio.org/) (VS Code extension o CLI)
- Python 3.x + `pip install pyserial`

### Build & Upload
```bash
git clone https://github.com/fichetto/Optical-Encoder-AB.git
cd Optical-Encoder-AB

# Compila
pio run

# Carica (auto-reset via USB — non serve premere BOOT)
pio run -t upload
```

> **Nota USB**: Il firmware usa **TinyUSB CDC** (`ARDUINO_USB_MODE=0`).
> L'upload avviene in auto-reset senza premere il pulsante BOOT.
> Dopo il primo avvio la porta può cambiare numero (es. da COM8 a COM10).

### Dipendenze (installate automaticamente)
- `Adafruit NeoPixel` — LED WS2812
- `ArduinoJson` — parsing JSON per WiFi API

## 💻 Desktop App Python

```bash
python encoder_reader.py
```

**Funzionalità:**
- Selezione porta COM, baud rate e frequenza di polling
- Visualizzazione contatori X/Y (pixel) e SQUAL in tempo reale
- Grafico scorrevole X/Y
- Pulsante **Reset X/Y** — azzera i contatori sull'ESP32
- Pulsante **Segna Riferimento** — zero locale senza reset hardware
- Pulsante **LED** — accende/spegne i LED del sensore PMW3901
- Scorciatoie: `R` = reset, `L` = LED, `Q` = esci

## 📡 Protocollo Modbus RTU

**Slave ID**: 1 — **Porta**: USB CDC (TinyUSB) — **Baud**: 921600 (virtuale, velocità reale = USB FS)

### Registri in Lettura — FC 03 / FC 04

| Registro | Nome | Tipo | Descrizione |
|---|---|---|---|
| 0x0000 | X\_HIGH | int16 | Bit 31-16 del contatore X |
| 0x0001 | X\_LOW | uint16 | Bit 15-0 del contatore X |
| 0x0002 | Y\_HIGH | int16 | Bit 31-16 del contatore Y |
| 0x0003 | Y\_LOW | uint16 | Bit 15-0 del contatore Y |
| 0x0004 | SQUAL | uint16 | Surface Quality (0–255) |
| 0x0005 | ENC\_HIGH | int16 | Bit 31-16 posizione encoder AB |
| 0x0006 | ENC\_LOW | uint16 | Bit 15-0 posizione encoder AB |

**Ricostruzione int32 in Python:**
```python
import struct
x = struct.unpack('>i', struct.pack('>HH', x_high, x_low))[0]
```

### Comandi in Scrittura — FC 06

| Registro | Valore | Effetto |
|---|---|---|
| 0x0010 | 0x0001 | Reset contatori X, Y, encoder |
| 0x0011 | 0x0001 | LED PMW3901 ON |
| 0x0011 | 0x0000 | LED PMW3901 OFF |

### Esempio Python Minimale
```python
import serial, struct

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

ser = serial.Serial('COM10', 921600, timeout=1.0)

# Leggi 5 registri (X_H, X_L, Y_H, Y_L, SQUAL)
req = struct.pack('>BBHH', 1, 0x04, 0, 5)
req += struct.pack('<H', crc16(req))
ser.write(req)
raw = ser.read(15)   # 3 header + 10 dati + 2 CRC

regs = [struct.unpack_from('>H', raw, 3 + i*2)[0] for i in range(5)]
x = struct.unpack('>i', struct.pack('>HH', regs[0], regs[1]))[0]
y = struct.unpack('>i', struct.pack('>HH', regs[2], regs[3]))[0]
print(f"X={x}  Y={y}  SQUAL={regs[4]}")
```

## 📡 Calibrazione via WiFi

Dopo 5 secondi dal boot, l'ESP32 avvia un Access Point:

| Parametro | Valore |
|---|---|
| SSID | `EncoderCalibration` |
| Password | `12345678` |
| URL | http://192.168.4.1 |

Dalla pagina web è possibile impostare la distanza sensore-superficie e controllare i LED. L'AP si spegne automaticamente dopo 3 minuti di inattività.

## ⚙️ Configurazione

### Task FreeRTOS
```
Core 0 — sensorTask  (priority 2, stack 8192)  → lettura PMW3901 @ 100 Hz
Core 1 — wifiTask    (priority 1, stack 16384) → Modbus RTU + WiFi AP
```

### Qualità Segnale SQUAL
| SQUAL | Giudizio |
|---|---|
| ≥ 80 | Eccellente |
| 40–79 | Buono |
| < 40 | Scarso (auto-reinizializzazione) |

## 🔧 Troubleshooting

**Porta COM non trovata dopo upload**
Il TinyUSB CDC può assegnare un numero di porta diverso dall'HWCDC originale. Clicca ⟳ nell'app per aggiornare la lista.

**SQUAL basso / nessun movimento rilevato**
Il sensore necessita di una superficie texturizzata a una distanza minima di ~80 mm.

**LED PMW3901 spenti**
Invia `FC06 reg=0x0011 val=0x0001` via Modbus o usa il pulsante LED nell'app.

**Upload fallisce**
Con TinyUSB l'auto-reset funziona senza BOOT. Se non funzionasse, tenere premuto BOOT e premere RESET prima di `pio run -t upload`.

## 📄 Licenza

MIT — vedi [LICENSE](LICENSE)

## 👨‍💻 Autore

- 📧 [mino.m@tecnocons.com](mailto:mino.m@tecnocons.com)
- 🐙 [@fichetto](https://github.com/fichetto)
- 💼 [LinkedIn](https://www.linkedin.com/in/cosimo-massimiliano-84126b34/)

## 🙏 Ringraziamenti

- **Pimoroni / Bitcraze** — breakout PMW3901 e sequenza di inizializzazione
- **Espressif** — framework Arduino-ESP32 e TinyUSB
