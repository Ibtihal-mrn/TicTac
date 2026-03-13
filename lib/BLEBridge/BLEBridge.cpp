/**
 * BLEBridge.cpp — ESP32 BLE Bridge (Nordic UART Service)
 *
 * Implémentation du périphérique BLE NUS.
 * - TX (notify) : logs ESP32 → PC
 * - RX (write)  : commandes PC → ESP32, parsées et mises en queue
 */

#include "BLEBridge.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

// ── Singletons ───────────────────────────────────────────────────────────────
BLEBridge bleBridge;
BleSerial bleSerial;

// ── Forward declaration ──────────────────────────────────────────────────────
static void onRxWrite(NimBLECharacteristic* pChar);

// ── Callbacks RX (PC → ESP32) ────────────────────────────────────────────────
class RxCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) override {
        onRxWrite(pChar);
    }
};
static RxCallback rxCallback;

// ── Constructeur / Destructeur ───────────────────────────────────────────────
BLEBridge::BLEBridge() = default;

BLEBridge::~BLEBridge() {
    if (logQueue_) { vQueueDelete(logQueue_); logQueue_ = nullptr; }
    if (cmdQueue_) { vQueueDelete(cmdQueue_); cmdQueue_ = nullptr; }
    NimBLEDevice::deinit(true);
}

// ── begin() ──────────────────────────────────────────────────────────────────
void BLEBridge::begin(const char* deviceName) {
    // Créer les files FreeRTOS
    logQueue_ = xQueueCreate(BLE_QUEUE_DEPTH, BLE_MAX_MSG_LEN);
    cmdQueue_ = xQueueCreate(BLE_CMD_QUEUE_DEPTH, sizeof(RobotCommand));

    if (!logQueue_ || !cmdQueue_) {
        Serial.println("[BLEBridge] ERREUR: impossible de créer les queues");
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

    // Caractéristique TX (ESP32 → PC) : NOTIFY + READ
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
    size_t len = strnlen(msg, BLE_MAX_MSG_LEN - 2);
    memcpy(buf, msg, len);
    // Garantir '\n' pour l'UI Python
    if (len > 0 && buf[len - 1] != '\n') {
        buf[len]     = '\n';
        buf[len + 1] = '\0';
    } else {
        buf[len] = '\0';
    }
    // Non-bloquant (0 = pas d'attente)
    xQueueSendToBack(logQueue_, buf, 0);
}

void BLEBridge::sendLog(const String& msg) {
    sendLog(msg.c_str());
}

// ── update() — appeler dans la tâche BLE (Core 0) ───────────────────────────
void BLEBridge::update() {
    if (!connected_ || connHandle_ == 0xFFFF) return;
    flushQueue_();
}

// ── flushQueue_() ────────────────────────────────────────────────────────────
void BLEBridge::flushQueue_() {
    if (!txChar_ || !logQueue_ || connHandle_ == 0xFFFF) return;

    char buf[BLE_MAX_MSG_LEN];
    uint16_t attrHandle = txChar_->getHandle();
    int count = 0;

    while (xQueueReceive(logQueue_, buf, 0) == pdTRUE) {
        size_t len = strnlen(buf, BLE_MAX_MSG_LEN);

        os_mbuf* om = ble_hs_mbuf_from_flat(
            reinterpret_cast<uint8_t*>(buf), len);
        if (om) {
            int rc = ble_gattc_notify_custom(connHandle_, attrHandle, om);
            if (rc != 0) {
                Serial.printf("[BLE] flush notify ERR rc=%d\n", rc);
            }
        }
        count++;
        vTaskDelay(pdMS_TO_TICKS(2));  // Ne pas saturer la liaison
    }
}

// ── Parsing des commandes reçues du PC ───────────────────────────────────────
void BLEBridge::parseCommand_(const char* raw, size_t len) {
    if (!cmdQueue_ || len == 0) return;

    RobotCommand cmd = {};
    // Copier le texte brut
    size_t cpLen = (len < BLE_MAX_MSG_LEN - 1) ? len : (BLE_MAX_MSG_LEN - 1);
    memcpy(cmd.raw, raw, cpLen);
    cmd.raw[cpLen] = '\0';

    // Trim trailing whitespace/newline
    for (int i = cpLen - 1; i >= 0; i--) {
        if (cmd.raw[i] == '\n' || cmd.raw[i] == '\r' || cmd.raw[i] == ' ')
            cmd.raw[i] = '\0';
        else
            break;
    }

    // Parser la commande (case-insensitive)
    char upper[BLE_MAX_MSG_LEN];
    for (size_t i = 0; i <= cpLen; i++) upper[i] = toupper(cmd.raw[i]);

    // Extraire le premier mot et un éventuel paramètre
    char* space = strchr(upper, ' ');
    float param = 0.0f;
    if (space) {
        *space = '\0';
        param = atof(space + 1);
    }

    if (strcmp(upper, "MOVE") == 0) {
        cmd.type = RobotCommandType::MOVE;
        cmd.value = param;
    } else if (strcmp(upper, "FORWARD") == 0) {
        cmd.type = RobotCommandType::MOVE_FORWARD;
        cmd.value = param;
    } else if (strcmp(upper, "BACKWARD") == 0) {
        cmd.type = RobotCommandType::MOVE_BACKWARD;
        cmd.value = param;
    } else if (strcmp(upper, "ROTATE") == 0) {
        cmd.type = RobotCommandType::ROTATE;
        cmd.value = param;
    } else if (strcmp(upper, "LEFT") == 0) {
        cmd.type = RobotCommandType::ROTATE_LEFT;
        cmd.value = param;
    } else if (strcmp(upper, "RIGHT") == 0) {
        cmd.type = RobotCommandType::ROTATE_RIGHT;
        cmd.value = param;
    } else if (strcmp(upper, "STOP") == 0) {
        cmd.type = RobotCommandType::STOP;
    } else if (strcmp(upper, "DEPLOY") == 0) {
        cmd.type = RobotCommandType::DEPLOY_SERVO;
    } else if (strcmp(upper, "RETRACT") == 0) {
        cmd.type = RobotCommandType::RETRACT_SERVO;
    } else if (strcmp(upper, "STATUS") == 0) {
        cmd.type = RobotCommandType::STATUS;
    } else if (strcmp(upper, "RESET") == 0) {
        cmd.type = RobotCommandType::RESET;
    } else if (strcmp(upper, "PING") == 0) {
        cmd.type = RobotCommandType::PING;
    } else {
        // Commande inconnue — on l'envoie quand même avec NONE
        cmd.type = RobotCommandType::NONE;
        Serial.printf("[BLE] Commande inconnue: '%s'\n", cmd.raw);
        // Envoyer un feedback au PC
        char fb[BLE_MAX_MSG_LEN];
        snprintf(fb, sizeof(fb), "[ESP32] Commande inconnue: '%s'", cmd.raw);
        bleBridge.sendLog(fb);
        return;
    }

    // Enqueue la commande (non-bloquant)
    if (xQueueSendToBack(cmdQueue_, &cmd, 0) != pdTRUE) {
        Serial.println("[BLE] Queue de commandes pleine !");
        bleBridge.sendLog("[ESP32] ERREUR: Queue de commandes pleine !");
    } else {
        char fb[BLE_MAX_MSG_LEN];
        snprintf(fb, sizeof(fb), "[ESP32] CMD reçue: '%s' (val=%.1f)", cmd.raw, cmd.value);
        Serial.println(fb);
        bleBridge.sendLog(fb);
    }
}

// ── Callback RX statique ─────────────────────────────────────────────────────
static void onRxWrite(NimBLECharacteristic* pChar) {
    std::string val = pChar->getValue();
    if (val.empty()) return;

    Serial.printf("[BLE] RX reçu (%d bytes): %s\n", val.length(), val.c_str());
    bleBridge.parseCommand_(val.c_str(), val.length());
}
