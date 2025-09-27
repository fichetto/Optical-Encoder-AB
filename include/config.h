/*
 * 🎯 PMW3901 OPTICAL ENCODER - CONFIGURAZIONE OTTIMALE
 * ===================================================
 * 
 * VERSIONE TESTATA: 80mm - infinito range
 * DATA: 27 Settembre 2025
 * STATUS: ✅ FUNZIONANTE
 */

#ifndef CONFIG_H
#define CONFIG_H

// ====== CONFIGURAZIONE HARDWARE ======
// Pin ESP32-S3-DevKitC-1
#define PMW3901_CS_PIN      11
#define PMW3901_SCK_PIN     12  
#define PMW3901_MOSI_PIN    13
#define PMW3901_MISO_PIN    14

// ====== CONFIGURAZIONE COMUNICAZIONE ======
#define SERIAL_BAUD_RATE    115200
#define SPI_FREQUENCY       4000000  // 4MHz (default)
#define STARTUP_DELAY_MS    3000     // Delay inizializzazione

// ====== CONFIGURAZIONE TIMING ======
#define LOOP_DELAY_MS       100      // 10Hz loop frequency
#define STATUS_REPORT_MS    10000    // Report ogni 10 secondi
#define DEBUG_EVERY_LOOPS   100      // Debug ogni 100 loop

// ====== RANGE OPERATIVO ======
#define MIN_DISTANCE_MM     80       // Distanza minima funzionamento
#define MAX_DISTANCE_MM     -1       // Infinito (-1 = unlimited)

// ====== REGISTRI PMW3901 ======
#define PMW3901_PRODUCT_ID_EXPECTED  0x49  // ID atteso per PMW3901
#define PMW3901_MOTION_DETECT_MASK   0x80  // Bit 7 per motion detection

// ====== CONFIGURAZIONE DEBUG ======
#define ENABLE_DEBUG_OUTPUT     true
#define ENABLE_MOTION_REGISTER  true  
#define ENABLE_PERIODIC_STATUS  true
#define ENABLE_SERIAL_COMMANDS  true

// ====== COMANDI SERIALI ======
#define CMD_RESET_REGISTERS  'R'  // Reset registri
#define CMD_TEST_INCREMENT   'T'  // Test incrementi
#define TEST_INCREMENT_X     10   // Valore test X
#define TEST_INCREMENT_Y     5    // Valore test Y

#endif // CONFIG_H
