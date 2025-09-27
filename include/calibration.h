/*
 * 🎯 PMW3901 OPTICAL ENCODER - CALIBRATION SYSTEM (SIMPLIFIED)
 * ============================================================
 * 
 * Sistema di calibrazione semplificato per conversione pixel -> mm
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include <Preferences.h>

// ====== CONFIGURAZIONE CALIBRAZIONE ======
#define PREFS_NAMESPACE "encoder_cal"
#define PREFS_DISTANCE_KEY "distance"
#define PREFS_SCALE_X_KEY "scale_x"
#define PREFS_SCALE_Y_KEY "scale_y"

// Default values
#define DEFAULT_DISTANCE_MM 80.0
#define DEFAULT_SCALE_FACTOR 0.1  // pixel to mm conversion base

// ====== CLASSE CALIBRAZIONE ======
class EncoderCalibration {
private:
    Preferences prefs;
    float distanceFromSurface;
    float scaleFactorX;
    float scaleFactorY;
    
    // Calcola il fattore di scala basato sulla distanza
    float calculateScaleFactor(float distance) {
        // Formula empirica: più lontano = meno pixel per mm
        // Calibrazione base: a 80mm, 1mm = 10 pixel circa
        return 10.0 * (80.0 / distance);
    }

public:
    // Inizializzazione
    void begin() {
        prefs.begin(PREFS_NAMESPACE, false);
        loadCalibration();
    }
    
    // Carica calibrazione salvata
    void loadCalibration() {
        distanceFromSurface = prefs.getFloat(PREFS_DISTANCE_KEY, DEFAULT_DISTANCE_MM);
        scaleFactorX = prefs.getFloat(PREFS_SCALE_X_KEY, calculateScaleFactor(distanceFromSurface));
        scaleFactorY = prefs.getFloat(PREFS_SCALE_Y_KEY, calculateScaleFactor(distanceFromSurface));
        
        Serial.println("📊 CALIBRAZIONE CARICATA:");
        Serial.print("   Distanza superficie: ");
        Serial.print(distanceFromSurface);
        Serial.println(" mm");
        Serial.print("   Fattore scala X: ");
        Serial.println(scaleFactorX, 4);
        Serial.print("   Fattore scala Y: ");
        Serial.println(scaleFactorY, 4);
    }
    
    // Salva calibrazione
    void saveCalibration(float distance) {
        if (distance < 80.0 || distance > 2000.0) {
            Serial.println("❌ DISTANZA NON VALIDA (80-2000mm)");
            return;
        }
        
        distanceFromSurface = distance;
        scaleFactorX = calculateScaleFactor(distance);
        scaleFactorY = calculateScaleFactor(distance);
        
        prefs.putFloat(PREFS_DISTANCE_KEY, distanceFromSurface);
        prefs.putFloat(PREFS_SCALE_X_KEY, scaleFactorX);
        prefs.putFloat(PREFS_SCALE_Y_KEY, scaleFactorY);
        
        Serial.println("💾 CALIBRAZIONE SALVATA!");
        Serial.print("   Nuova distanza: ");
        Serial.print(distance);
        Serial.println(" mm");
    }
    
    // Converte pixel in mm
    float pixelsToMmX(int16_t pixels) {
        return pixels / scaleFactorX;
    }
    
    float pixelsToMmY(int16_t pixels) {
        return pixels / scaleFactorY;
    }
    
    // Converte registri pixel in mm
    float registerToMmX(long registerValue) {
        return registerValue / scaleFactorX;
    }
    
    float registerToMmY(long registerValue) {
        return registerValue / scaleFactorY;
    }
    
    // Getter
    float getDistance() { return distanceFromSurface; }
    float getScaleX() { return scaleFactorX; }
    float getScaleY() { return scaleFactorY; }
    
    // Imposta distanza manualmente
    void setDistance(float distance) {
        saveCalibration(distance);
    }
    
    // Reset calibrazione
    void resetToDefaults() {
        prefs.clear();
        distanceFromSurface = DEFAULT_DISTANCE_MM;
        scaleFactorX = calculateScaleFactor(DEFAULT_DISTANCE_MM);
        scaleFactorY = calculateScaleFactor(DEFAULT_DISTANCE_MM);
        Serial.println("🔄 CALIBRAZIONE RESETTATA AI VALORI DEFAULT");
    }
};

#endif // CALIBRATION_H
