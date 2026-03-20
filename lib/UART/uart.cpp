// uart.cpp
#include "uart.h"
#include "../../src/config_coprocessor.h"

static HardwareSerial* _serial = nullptr;

static uint8_t buffer[sizeof(SensorPacket)];
static uint8_t rxIndex = 0;

enum ParseState {
    WAIT_START,
    READ_DATA
};

static ParseState state = WAIT_START;

// ================= INIT =================
void uart_init(HardwareSerial& serial) {
    _serial = &serial;
    serial.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
}

// ================= SEND =================
void sendPacket(const SensorPacket& packetIn) {
    if (!_serial) return;

    SensorPacket p = packetIn;
    p.checksum = computeChecksum((uint8_t*)&p, sizeof(p) - 2);

    _serial->write(PACKET_START);
    _serial->write((uint8_t*)&p, sizeof(p));
}

// ================= RECEIVE =================
bool readPacket(SensorPacket& out) {
    if (!_serial) return false;

    while (_serial->available()) {
        uint8_t b = _serial->read();

        switch (state) {

            case WAIT_START:
                if (b == PACKET_START) {
                    rxIndex = 0;
                    state = READ_DATA;
                }
                break;

            case READ_DATA:
                buffer[rxIndex++] = b;

                if (rxIndex >= sizeof(SensorPacket)) {
                    memcpy(&out, buffer, sizeof(SensorPacket));

                    uint16_t chk = computeChecksum(
                        (uint8_t*)&out,
                        sizeof(SensorPacket) - 2
                    );

                    state = WAIT_START;
                    return (chk == out.checksum);
                }
                break;
        }
    }

    return false;
}