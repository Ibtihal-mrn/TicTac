/**
 * BLEBridge.h — ESP32 BLE Bridge (Nordic UART Service)
 *
 * Fournit :
 *   - sendLog()  : envoie un log vers le PC (via BLE notify, thread-safe)
 *   - La réception de commandes (PC → ESP32) dans une queue FreeRTOS
 *
 * Flux :
 *   ESP32 logs ──► logQueue_ ──► BLE notify ──► PC (ui.py)
 *   PC commands ──► BLE write ──► cmdQueue  ──► FSM (Core 1)
 */

#pragma once

#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <Arduino.h>

// ── NUS UUIDs ────────────────────────────────────────────────────────────────
#define NUS_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_TX_CHAR_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // ESP32 → PC
#define NUS_RX_CHAR_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // PC → ESP32

// ── Config ───────────────────────────────────────────────────────────────────
#define BLE_DEVICE_NAME   "Eurobot"
#define BLE_MAX_MSG_LEN   240
#define BLE_QUEUE_DEPTH   32
#define BLE_CMD_QUEUE_DEPTH 16
#define BLE_MTU           247

// ── Types de commandes robot ─────────────────────────────────────────────────
enum class RobotCommandType : uint8_t {
    NONE = 0,
    MOVE,
    ROTATE,
    STOP,
    STATUS,
    RESET,
    PING,
};

struct RobotCommand {
    RobotCommandType type = RobotCommandType::NONE;
    float value = 0.0f;        // paramètre optionnel (distance, angle, …)
    char raw[BLE_MAX_MSG_LEN]; // commande brute reçue
};

// ── BLEBridge ────────────────────────────────────────────────────────────────
class BLEBridge : public NimBLEServerCallbacks {
public:
    BLEBridge();
    ~BLEBridge();

    /// Initialiser le BLE (appeler dans setup, sur Core 0 de préférence)
    void begin(const char* deviceName = BLE_DEVICE_NAME);

    /// Envoyer un log vers le PC (thread-safe, non-bloquant)
    void sendLog(const char* msg);
    void sendLog(const String& msg);

    /// Flush les logs en attente vers BLE (appeler périodiquement)
    void update();

    /// Récupérer la queue de commandes (pour la FSM)
    QueueHandle_t getCommandQueue() const { return cmdQueue_; }

    /// Etat de connexion
    bool isConnected() const { return connected_; }

private:
    // NimBLE callbacks
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override;
    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override;

    void flushQueue_();

public:
    /// Parse et enqueue une commande reçue (appelé par le callback RX)
    void parseCommand_(const char* raw, size_t len);

private:

    NimBLEServer*         server_   = nullptr;
    NimBLEService*        service_  = nullptr;
    NimBLECharacteristic* txChar_   = nullptr;
    NimBLECharacteristic* rxChar_   = nullptr;

    QueueHandle_t logQueue_ = nullptr;   // logs ESP32 → PC
    QueueHandle_t cmdQueue_ = nullptr;   // commandes PC → ESP32

    volatile bool connected_   = false;
    uint16_t      connHandle_  = 0xFFFF;
};

// ── Singleton global ─────────────────────────────────────────────────────────
extern BLEBridge bleBridge;

// ── bleSerial : wrapper pour envoyer facilement des logs ─────────────────────
class BleSerial : public Print {
public:
    size_t write(uint8_t c) override {
        char s[2] = {(char)c, '\0'};
        bleBridge.sendLog(s);
        Serial.write(c);
        return 1;
    }

    size_t write(const uint8_t* buf, size_t size) override {
        // Buffer pour construire le message
        char tmp[BLE_MAX_MSG_LEN];
        size_t chunk = (size < BLE_MAX_MSG_LEN - 1) ? size : (BLE_MAX_MSG_LEN - 1);
        memcpy(tmp, buf, chunk);
        tmp[chunk] = '\0';
        bleBridge.sendLog(tmp);
        Serial.write(buf, size);
        return size;
    }
};

extern BleSerial bleSerial;
