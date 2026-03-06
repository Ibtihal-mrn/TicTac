/**
 * BLEBridge.cpp  —  ESP32-S3 side
 *
 * Implémentation du péripérique BLE (NUS – Nordic UART Service).
 *
 * Flux de données :
 *   Serial.print() (code existant)
 *         │
 *         ▼  (appels manuels via bleBridge.sendLog())
 *   BLEBridge::sendLog()
 *         │  → copie dans logQueue_ (FreeRTOS queue, thread-safe)
 *         ▼
 *   BLEBridge::update()  ← appelé dans loop()
 *         │  → flush queue → notify(txChar_) → PC via BLE
 *         ▼
 *   BluetoothManager (PC Linux/BlueZ)
 *         │  → BackendServer (TCP)
 *         ▼
 *   UI Python
 */

#include "BLEBridge.h"
#include <cstdio>
#include <cstring>

// ── Singleton global ─────────────────────────────────────────────────────────
BLEBridge bleBridge;

// ── Singleton TeeSerial ──────────────────────────────────────────────────────
TeeSerial bleSerial;

// ── Callback RX (PC → ESP32) — pas de commande pour l'instant, log seulement ─
class RxCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) override {
        // Réservé pour les futures commandes
        // Pour l'instant on ignore les données reçues
        (void)pChar;
    }
};
static RxCallback rxCallback;

// ── Constructeur / Destructeur ───────────────────────────────────────────────
BLEBridge::BLEBridge() = default;

BLEBridge::~BLEBridge() {
    if (logQueue_) {
        vQueueDelete(logQueue_);
        logQueue_ = nullptr;
    }
    NimBLEDevice::deinit(true);
}

// ── begin() ─────────────────────────────────────────────────────────────────
void BLEBridge::begin(const char* deviceName) {
    // Créer la file FreeRTOS (messages char[BLE_MAX_MSG_LEN])
    logQueue_ = xQueueCreate(BLE_QUEUE_DEPTH, BLE_MAX_MSG_LEN);
    if (!logQueue_) {
        Serial.println("[BLEBridge] ERREUR: impossible de créer la file de logs");
        return;
    }

    // Initialisation NimBLE
    NimBLEDevice::init(deviceName);
    NimBLEDevice::setMTU(BLE_MTU);

    // Serveur BLE
    server_ = NimBLEDevice::createServer();
    server_->setCallbacks(this);

    // Service NUS
    service_ = server_->createService(NUS_SERVICE_UUID);

    // Caractéristique TX (ESP32 → PC) : NOTIFY + READ (Windows compat)
    txChar_ = service_->createCharacteristic(
        NUS_TX_CHAR_UUID,
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
    );

    // Caractéristique RX (PC → ESP32) : WRITE
    rxChar_ = service_->createCharacteristic(
        NUS_RX_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    rxChar_->setCallbacks(&rxCallback);

    // Démarrage du service
    service_->start();

    // Advertising
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(NUS_SERVICE_UUID);
    adv->setScanResponse(true);
    adv->start();

    Serial.printf("[BLEBridge] Advertising BLE sous le nom \"%s\"\n", deviceName);
}

// ── Callbacks connexion ──────────────────────────────────────────────────────
void BLEBridge::onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
    connected_ = true;
    connHandle_ = desc->conn_handle;
    Serial.printf("[BLEBridge] Client connecté (conn=%d)\n", connHandle_);
}

void BLEBridge::onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
    connected_ = false;
    connHandle_ = 0xFFFF;
    Serial.println("[BLEBridge] Client déconnecté — reprise advertising");
    NimBLEDevice::startAdvertising();
}

// ── sendLog() ────────────────────────────────────────────────────────────────
void BLEBridge::sendLog(const char* msg) {
    if (!logQueue_ || !msg) return;

    char buf[BLE_MAX_MSG_LEN] = {};
    // Tronquer si nécessaire, garantir terminaison '\n' pour l'UI Python
    size_t len = strnlen(msg, BLE_MAX_MSG_LEN - 2);
    memcpy(buf, msg, len);
    if (len > 0 && buf[len - 1] != '\n') {
        buf[len]     = '\n';
        buf[len + 1] = '\0';
    } else {
        buf[len] = '\0';
    }

    // Envoi non bloquant depuis ISR ou tâche (0 = pas d'attente)
    xQueueSendToBack(logQueue_, buf, 0);
}

void BLEBridge::sendLog(const String& msg) {
    sendLog(msg.c_str());
}

// ── update() ─────────────────────────────────────────────────────────────────
void BLEBridge::update() {
    // ── Diagnostic : log transition de connected_ ──
    static bool wasConn = false;
    if (connected_ != wasConn) {
        Serial.printf("[BLE-DBG] connected_ %d -> %d  connHandle_=%d\n",
                      wasConn, (int)connected_, connHandle_);
        wasConn = connected_;
    }

    if (!connected_ || connHandle_ == 0xFFFF) return;

    // ── Heartbeat direct : 1 notification brute toutes les 3 s ──
    //    Teste le chemin BLE SANS la queue.
    static unsigned long lastHB = 0;
    if (millis() - lastHB >= 3000) {
        const char hb[] = "[BLE-HB] heartbeat\n";
        uint16_t attrH = txChar_->getHandle();

        // Méthode 1 : API bas-niveau (bypass m_subscribedVec)
        os_mbuf *om = ble_hs_mbuf_from_flat(
            reinterpret_cast<const uint8_t*>(hb), sizeof(hb) - 1);
        int rc1 = -99;
        if (om) {
            rc1 = ble_gattc_notify_custom(connHandle_, attrH, om);
        }

        // Méthode 2 : API haut-niveau NimBLE-Arduino (utilise m_subscribedVec)
        txChar_->setValue(reinterpret_cast<const uint8_t*>(hb), sizeof(hb) - 1);
        txChar_->notify();

        Serial.printf("[BLE-DBG] HB conn=%d attr=%d rc_low=%d subs=%d\n",
                      connHandle_, attrH, rc1,
                      (int)txChar_->getSubscribedCount());

        lastHB = millis();
    }

    flushQueue_();
}

// ── flushQueue_() ────────────────────────────────────────────────────────────
void BLEBridge::flushQueue_() {
    if (!txChar_ || !logQueue_ || connHandle_ == 0xFFFF) return;

    char buf[BLE_MAX_MSG_LEN];
    uint16_t attrHandle = txChar_->getHandle();
    int count = 0;

    // Vider toute la file en une passe de loop()
    while (xQueueReceive(logQueue_, buf, 0) == pdTRUE) {
        size_t len = strnlen(buf, BLE_MAX_MSG_LEN);

        // Envoi direct via l'API bas-niveau NimBLE.
        os_mbuf *om = ble_hs_mbuf_from_flat(
            reinterpret_cast<uint8_t*>(buf), len);
        if (om) {
            int rc = ble_gattc_notify_custom(connHandle_, attrHandle, om);
            if (rc != 0) {
                Serial.printf("[BLE-DBG] flush notify ERR rc=%d\n", rc);
            }
        }
        count++;

        // Petit délai pour ne pas saturer la liaison BLE
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (count > 0) {
        Serial.printf("[BLE-DBG] flushed %d msgs\n", count);
    }
}
