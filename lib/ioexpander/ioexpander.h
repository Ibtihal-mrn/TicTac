#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

struct ExpanderCmd { 
  uint8_t reg; 
  uint8_t val; 
  uint8_t flags; 
};

class IOExpander {
public:
  IOExpander(uint8_t addr);
  
  // Initialization
  bool begin(); // create queue
  
  // I2C Operations
  bool readReg(uint8_t reg, uint8_t &out, TickType_t timeoutMs = pdMS_TO_TICKS(200));
  bool queueWrite(uint8_t reg, uint8_t val, TickType_t wait = 0);
  
  // Accessors
  QueueHandle_t getCmdQueue() const { return _cmdQ; }
  uint8_t getAddress() const { return _addr; }
  
  // FreeRTOS task entry point
  static void taskEntry(void* param); // called by xTaskCreatePinnedToCore
  
  void run(); // main task loop

  private:

  void processWrite(const ExpanderCmd &cmd); // performs I2C write under mutex
  
  uint8_t _addr;
  QueueHandle_t _cmdQ;
};