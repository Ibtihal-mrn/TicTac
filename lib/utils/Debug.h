#pragma once
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

// Feature bits (one byte)
#define DBG_FSM           (1<<0)
#define DBG_TASKMANAGER   (1<<1)
#define DBG_MOTORS        (1<<2)
#define DBG_SENSORS       (1<<3)
#define DBG_COMMS         (1<<4)
#define DBG_ENCODER       (1<<5)
#define DBG_LAUNCH_TGR    (1<<6)
#define DBG_STEPPER       (1<<7)
#define DBG_SERVO         (1<<8)

// Global debug mask (0 = none). You can set this at runtime.
extern uint8_t debugMask;

// Initialize Serial and debug mask
inline void debugInit(uint32_t baud, uint8_t mask=0) {
  Serial.begin(baud);
  debugMask = mask;
}

// Set mask at runtime
inline void debugSetMask(uint8_t mask) { debugMask = mask; }
inline void debugEnable(uint8_t bits) { debugMask |= bits; }
inline void debugDisable(uint8_t bits) { debugMask &= ~bits; }

// Minimum safe formatted print for AVR
inline void debugPrintf(uint8_t feature, const char *fmt, ...) {
  if (!(debugMask & feature)) return;
  char buf[48]; // keep small — UNO RAM is tight
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