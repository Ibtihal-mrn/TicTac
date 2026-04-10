#pragma once
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

// Feature bits (one byte)
#define DBG_FSM            (1u << 0)
#define DBG_TASKMANAGER    (1u << 1)
#define DBG_MOTORS         (1u << 2)
#define DBG_PID            (1u << 3)
#define DBG_COMMS          (1u << 4)
#define DBG_ENCODER        (1u << 5)
#define DBG_LAUNCH_TGR     (1u << 6)
#define DBG_STEPPER        (1u << 7)
#define DBG_SERVO          (1u << 8)
#define DBG_MAGNET         (1u << 9)
#define DEBUG_TEAM_SWITCH  (1u << 10)
#define DBG_I2C_HUB        (1u << 11)
#define DBG_IMU            (1u << 12)

// #define           (1<<12)
// Global debug mask (0 = none). You can set this at runtime.
extern uint32_t debugMask;

// Initialize Serial and debug mask
inline void debugInit(uint32_t baud, uint32_t mask = 0) {
  Serial.begin(baud);
  debugMask = mask;
}

// Set mask at runtime
inline void debugSetMask(uint32_t mask) { debugMask = mask; }
inline void debugEnable(uint32_t bits) { debugMask |= bits; }
inline void debugDisable(uint32_t bits) { debugMask &= ~bits; }

// Minimum safe formatted print for AVR
inline void debugPrintf(uint32_t feature, const char *fmt, ...) {
  if (!(debugMask & feature)) return;
  char buf[64];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.println(buf);
}

// Simple F() friendly print (no formatting)
inline void debugPrintF(uint8_t feature, const __FlashStringHelper* msg) {
  if (!(debugMask & feature)) return;
  Serial.println(msg);
}


inline void printMillis(uint8_t feature, const char* text, unsigned long t, unsigned long& lastPrint, unsigned long interval = 2000) {
    if (!(debugMask & feature)) return;
    if (t - lastPrint >= interval) {
        Serial.print(text);
        lastPrint = t;
    }
}