#pragma once
/**
 * BLEBridge.h  —  ESP32-S3 side
 *
 * Rôle : péripérique BLE (serveur) qui diffuse tous les logs Serial
 *        via le Nordic UART Service (NUS) en notifications BLE.
 *
 * Architecture :
 *   ESP32 BLE (Peripheral / Server)
 *     └── NUS Service
 *           ├── TX Char  (Notify  6E400003…)  ← logs → PC
 *           └── RX Char  (Write   6E400002…)  ← commandes futures
 *
 * Dépendance PlatformIO : h2zero/NimBLE-Arduino
 *
 * Utilisation minimale dans main.cpp :
 *   #include "BLEBridge.h"
 *   // setup()  → bleBridge.begin();
 *   // loop()   → bleBridge.update();
 *               → bleBridge.sendLog("message");
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// ── UUIDs Nordic UART Service ────────────────────────────────────────────────
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // Notify (ESP32 → PC)
#define NUS_RX_CHAR_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Write  (PC → ESP32)

#define BLE_DEVICE_NAME    "Eurobot"
#define BLE_MTU            512
#define BLE_QUEUE_DEPTH    64    // messages bufferisés avant flush
#define BLE_MAX_MSG_LEN    240   // taille max d'un message (≤ MTU-3)

// ── Classe principale ────────────────────────────────────────────────────────
class BLEBridge : public NimBLEServerCallbacks {
public:
    BLEBridge();
    ~BLEBridge();

    /**
     * Initialise le stack BLE et démarre l'advertising.
     * @param deviceName  Nom BLE visible par le PC (défaut : "Eurobot")
     */
    void begin(const char* deviceName = BLE_DEVICE_NAME);

    /**
     * À appeler dans loop() — flush la file de messages vers la caractéristique TX.
     * Ne bloque pas (< 1 ms si la file est vide).
     */
    void update();

    /**
     * Envoie un message de log via BLE (et le duplique sur Serial).
     * Thread-safe : peut être appelé depuis une ISR ou une tâche FreeRTOS.
     */
    void sendLog(const char* msg);
    void sendLog(const String& msg);

    /** Retourne true si un client PC est connecté. */
    bool isConnected() const { return connected_; }

    // ── NimBLEServerCallbacks ────────────────────────────────────────────────
    void onConnect(NimBLEServer* pServer) override;
    void onDisconnect(NimBLEServer* pServer) override;

private:
    NimBLEServer*           server_   = nullptr;
    NimBLEService*          service_  = nullptr;
    NimBLECharacteristic*   txChar_   = nullptr;   // notifications → PC
    NimBLECharacteristic*   rxChar_   = nullptr;   // write ← PC

    volatile bool       connected_  = false;
    QueueHandle_t       logQueue_   = nullptr;     // file thread-safe

    /** Extrait et transmet tous les messages en attente. */
    void flushQueue_();
};

// ── Singleton global ─────────────────────────────────────────────────────────
extern BLEBridge bleBridge;

// ── TeeSerial : Print wrapper qui envoie vers Serial ET BLE ──────────────────
// Utilisé pour intercepter TOUT Serial.print/println existant sans modifier
// la logique du code. Chaque ligne complète est envoyée via bleBridge.sendLog().
class TeeSerial : public Print {
public:
    size_t write(uint8_t c) override {
        // Toujours écrire vers Serial (USB)
        Serial.write(c);

        // Accumuler dans le buffer ligne
        if (bufPos_ < sizeof(lineBuf_) - 1) {
            lineBuf_[bufPos_++] = static_cast<char>(c);
        }

        // Sur '\n' ou buffer plein → envoyer la ligne complète en BLE
        if (c == '\n' || bufPos_ >= sizeof(lineBuf_) - 1) {
            lineBuf_[bufPos_] = '\0';
            bleBridge.sendLog(lineBuf_);
            bufPos_ = 0;
        }
        return 1;
    }

    size_t write(const uint8_t* buf, size_t size) override {
        for (size_t i = 0; i < size; ++i) {
            write(buf[i]);
        }
        return size;
    }

private:
    char   lineBuf_[BLE_MAX_MSG_LEN] = {};
    size_t bufPos_ = 0;
};

// ── Singleton global TeeSerial ───────────────────────────────────────────────
extern TeeSerial bleSerial;
