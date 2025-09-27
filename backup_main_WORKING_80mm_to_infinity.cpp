#include <Arduino.h>
#include <SPI.h>

// Pin definitions per ESP32-S3-Nano Waveshare
#define PMW3901_CS    11
#define PMW3901_SCK   12  
#define PMW3901_MOSI  13
#define PMW3901_MISO  14

// PMW3901 registers
#define PMW3901_PRODUCT_ID        0x00
#define PMW3901_REVISION_ID       0x01
#define PMW3901_MOTION            0x02
#define PMW3901_DELTA_X_L         0x03
#define PMW3901_DELTA_X_H         0x04
#define PMW3901_DELTA_Y_L         0x05
#define PMW3901_DELTA_Y_H         0x06

SPIClass *spi;

// Registri accumulatori per gli incrementi
volatile long registerX = 0;  // Registro cumulativo incrementi X
volatile long registerY = 0;  // Registro cumulativo incrementi Y

// Function declaration
uint8_t readRegister(uint8_t reg);

void setup() {
  // Initialize both serial ports
  Serial.begin(115200);
  #if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);  // Don't wait for Serial if no USB connection
  #endif
  
  delay(3000);  // Longer delay to ensure serial is ready
  
  Serial.println("=== ESP32-S3 BOOT ===");
  Serial.println("PMW3901 Flow Sensor Test");
  Serial.println("========================");
  Serial.flush();  // Force output
  
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
  Serial.println("Testing PMW3901 communication...");
  Serial.flush();
  
  Serial.println("Reading Product ID...");
  uint8_t productID = readRegister(PMW3901_PRODUCT_ID);
  Serial.print("Raw Product ID: 0x");
  Serial.print(productID, HEX);
  Serial.print(" (");
  Serial.print(productID);
  Serial.println(")");
  
  Serial.println("Reading Revision ID...");
  uint8_t revisionID = readRegister(PMW3901_REVISION_ID);
  Serial.print("Raw Revision ID: 0x");
  Serial.print(revisionID, HEX);
  Serial.print(" (");
  Serial.print(revisionID);
  Serial.println(")");
  Serial.flush();
  
  if (productID == 0x49) {
    Serial.println("✓ PMW3901 detected successfully!");
    Serial.println("Initializing PMW3901...");
    
    // Sequenza di inizializzazione PMW3901
    delay(50);
    
    // Reset del sensore e inizializzazione 
    Serial.println("PMW3901 ready for operation!");
  } else if (productID == 0x00 || productID == 0xFF) {
    Serial.println("✗ No response from sensor (check connections/power)");
  } else {
    Serial.println("✗ Unknown sensor detected - check part number");
    Serial.println("Expected Product ID: 0x49 for PMW3901");
  }
  Serial.flush();
}

void loop() {
  static unsigned long lastPrint = 0;
  static int loopCount = 0;
  
  // Print alive message every 5 seconds with more info
  if (millis() - lastPrint > 5000) {
    Serial.println();
    Serial.print("ESP32-S3 alive - Loop #");
    Serial.print(++loopCount);
    Serial.println(" - checking sensor...");
    
    // Test sensor communication periodically
    uint8_t productID = readRegister(PMW3901_PRODUCT_ID);
    Serial.print("Current Product ID: 0x");
    Serial.println(productID, HEX);
    
    lastPrint = millis();
    Serial.flush();
  }
  
  // Read motion data
  uint8_t motion = readRegister(PMW3901_MOTION);
  
  // Debug motion register every 50 loops (5 seconds)
  static int debugCount = 0;
  if (++debugCount >= 50) {
    Serial.print("Motion register: 0x");
    Serial.println(motion, HEX);
    debugCount = 0;
    Serial.flush();
  }
  
  // Leggi sempre i valori delta per diagnostica
  int16_t deltaX = (int16_t)((readRegister(PMW3901_DELTA_X_H) << 8) | readRegister(PMW3901_DELTA_X_L));
  int16_t deltaY = (int16_t)((readRegister(PMW3901_DELTA_Y_H) << 8) | readRegister(PMW3901_DELTA_Y_L));
  
  // Debug: mostra tutti i valori ogni 100 loop per diagnostica
  static int diagCount = 0;
  if (++diagCount >= 100) {
    Serial.print("🔍 DEBUG: motion=0x");
    Serial.print(motion, HEX);
    Serial.print(", deltaX=");
    Serial.print(deltaX);
    Serial.print(", deltaY=");
    Serial.println(deltaY);
    diagCount = 0;
    Serial.flush();
  }
  
  // Controlla movimento (motion bit 7 OR qualsiasi delta diverso da zero)
  if ((motion & 0x80) || (deltaX != 0) || (deltaY != 0)) {
    if (deltaX != 0 || deltaY != 0) {  // Only print if there's actual movement
      // Aggiorna i registri accumulatori
      registerX += deltaX;
      registerY += deltaY;
      
      Serial.print("🎯 MOVEMENT: ΔX=");
      Serial.print(deltaX);
      Serial.print(", ΔY=");
      Serial.print(deltaY);
      Serial.print(" | REGISTRI: X=");
      Serial.print(registerX);
      Serial.print(", Y=");
      Serial.print(registerY);
      Serial.print(" (motion=0x");
      Serial.print(motion, HEX);
      Serial.println(")");
      Serial.flush();
    }
  }
  
  // Mostra i registri ogni 10 secondi anche senza movimento
  static unsigned long lastRegisterPrint = 0;
  if (millis() - lastRegisterPrint > 10000) {
    Serial.println("📊 STATO REGISTRI:");
    Serial.print("   Registro X: ");
    Serial.println(registerX);
    Serial.print("   Registro Y: ");
    Serial.println(registerY);
    Serial.println("   (R per resettare)");
    lastRegisterPrint = millis();
    Serial.flush();
  }
  
  // Controlla se c'è un comando per resettare i registri
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'R' || cmd == 'r') {
      registerX = 0;
      registerY = 0;
      Serial.println("🔄 REGISTRI RESETTATI!");
      Serial.flush();
    } else if (cmd == 'T' || cmd == 't') {
      // Test dei registri con valori simulati
      registerX += 10;
      registerY += 5;
      Serial.println("🧪 TEST: Incrementi simulati aggiunti!");
      Serial.print("   Registro X: ");
      Serial.println(registerX);
      Serial.print("   Registro Y: ");
      Serial.println(registerY);
      Serial.flush();
    }
  }
  
  delay(100);
}

uint8_t readRegister(uint8_t reg) {
  digitalWrite(PMW3901_CS, LOW);
  delayMicroseconds(1);
  
  spi->transfer(reg & 0x7F);  // Read command (bit 7 = 0)
  delayMicroseconds(100);     // tSRAD delay
  
  uint8_t data = spi->transfer(0x00);
  
  delayMicroseconds(1);
  digitalWrite(PMW3901_CS, HIGH);
  delayMicroseconds(19);      // tSCLK-NCS delay
  
  return data;
}