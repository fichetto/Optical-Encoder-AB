# 🎯 PMW3901 Optical Encoder - Versione Funzionante (80mm-∞)

## ✅ **Caratteristiche Confermate**

### **Range di Funzionamento**
- **Distanza Minima**: 80mm
- **Distanza Massima**: Infinito (∞)
- **Risoluzione**: Incrementi in pixel del sensore

### **Hardware**
- **Microcontrollore**: ESP32-S3-DevKitC-1
- **Sensore**: PMW3901 Flow Sensor
- **Comunicazione**: SPI

### **Pin Configuration (ESP32-S3)**
```cpp
#define PMW3901_CS    11   // Chip Select
#define PMW3901_SCK   12   // Serial Clock
#define PMW3901_MOSI  13   // Master Out Slave In
#define PMW3901_MISO  14   // Master In Slave Out
```

## 🔧 **Funzionalità Implementate**

### **Registri Accumulatori**
- **registerX**: Accumula incrementi orizzontali
- **registerY**: Accumula incrementi verticali
- **Tipo**: `volatile long` per thread safety

### **Output Seriale (115200 baud)**
- **Incrementi Istantanei**: `ΔX=valor`, `ΔY=valor`
- **Registri Cumulativi**: `REGISTRI: X=total, Y=total`
- **Debug Info**: Motion register, Product ID
- **Stato Periodico**: Ogni 10 secondi

### **Comandi Seriali**
- **`R`** o **`r`**: Reset registri a zero
- **`T`** o **`t`**: Test con incrementi simulati

## 📊 **Esempio Output**
```
🎯 MOVEMENT: ΔX=6, ΔY=-12 | REGISTRI: X=7, Y=-15 (motion=0xB8)

📊 STATO REGISTRI:
   Registro X: 7
   Registro Y: -15
   (R per resettare)

🔍 DEBUG: motion=0x2B, deltaX=0, deltaY=0
```

## 🎛️ **Configurazione SPI**
- **Clock Speed**: Default ESP32 SPI
- **Mode**: SPI Mode 0
- **Bit Order**: MSB First
- **CS Active**: LOW

## ⚡ **Performance**
- **Loop Frequency**: ~10Hz (100ms delay)
- **Debug Output**: Ogni 100 loop
- **Status Report**: Ogni 10 secondi

## 🎯 **Sensibilità**
- **Motion Detection**: Bit 7 del motion register OR deltaX/Y ≠ 0
- **Range Efficace**: 80mm - infinito
- **Superficie Richiesta**: Texture per contrasto ottimale

## 🚀 **Uso Consigliato**
Perfetto per:
- Encoder ottici a lunga distanza
- Tracking di oggetti in movimento
- Applicazioni di navigazione
- Controlli di posizione senza contatto

---
**Data**: 27 Settembre 2025  
**Status**: ✅ VERSIONE TESTATA E FUNZIONANTE
