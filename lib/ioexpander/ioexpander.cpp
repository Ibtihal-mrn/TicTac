#include "IOExpander.h"
#include <Wire.h>

// Forward declarations to avoid circular includes
extern SemaphoreHandle_t i2cMutex;
extern SemaphoreHandle_t ioExpanderMutex;
struct IOExpanderData { bool teamSwitch; bool launchTrigger; bool ready; };
extern IOExpanderData ioExpanderData;

// queue length
static constexpr size_t CMD_Q_LEN = 8;
static constexpr TickType_t I2C_TIMEOUT_MS = 500;  // Increased timeout to avoid conflicts with LIDAR/IMU
static constexpr TickType_t RUN_DELAY_MS = 10;

IOExpander::IOExpander(uint8_t addr) : _addr(addr), _cmdQ(nullptr) {}

bool IOExpander::begin() {
  _cmdQ = xQueueCreate(CMD_Q_LEN, sizeof(ExpanderCmd));
  return (_cmdQ != nullptr);
}

bool IOExpander::queueWrite(uint8_t reg, uint8_t val, TickType_t wait) {
  if (!_cmdQ) return false;
  ExpanderCmd c{reg, val, 0};
  return xQueueSend(_cmdQ, &c, wait) == pdTRUE;
}

bool IOExpander::readReg(uint8_t reg, uint8_t &out, TickType_t timeoutMs) {
  if (i2cMutex == nullptr) return false;
  if (xSemaphoreTake(i2cMutex, timeoutMs) != pdTRUE) return false;
  
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { 
    xSemaphoreGive(i2cMutex); 
    return false; 
  }
  
  size_t r = Wire.requestFrom((int)_addr, (int)1);
  if (r == 0) { 
    xSemaphoreGive(i2cMutex); 
    return false; 
  }
  
  out = Wire.read();
  xSemaphoreGive(i2cMutex);
  return true;
}

void IOExpander::processWrite(const ExpanderCmd &cmd) {
  if (i2cMutex == nullptr) return;
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) return;
  
  Wire.beginTransmission(_addr);
  Wire.write(cmd.reg);
  Wire.write(cmd.val);
  Wire.endTransmission();
  
  xSemaphoreGive(i2cMutex);
}

void IOExpander::run() {
  ExpanderCmd cmd;
  TickType_t lastInputRead = xTaskGetTickCount();
  bool lastReadFailed = false;  // Track last read status to avoid spam
  
  for (;;) {
    // Process queued write commands
    if (_cmdQ && xQueueReceive(_cmdQ, &cmd, 0) == pdTRUE) {
      processWrite(cmd);
    }
    
    // Periodic read of inputs and update ioExpanderData (reduced frequency to avoid I2C conflicts)
    TickType_t now = xTaskGetTickCount();
    if ((now - lastInputRead) >= pdMS_TO_TICKS(200)) {
      uint8_t pinState;
      if (readReg(0x00, pinState)) {  // Read input register
        lastReadFailed = false;
        if (xSemaphoreTake(ioExpanderMutex, 0) == pdTRUE) {
          ioExpanderData.teamSwitch = (pinState >> 1) & 1;      // Pin 1
          ioExpanderData.launchTrigger = (pinState >> 3) & 1;   // Pin 3
          ioExpanderData.ready = true;                          // Signal that data is valid
          xSemaphoreGive(ioExpanderMutex);
        }
      }
      lastInputRead = now;
    }
    
    vTaskDelay(pdMS_TO_TICKS(RUN_DELAY_MS));
  }
}

void IOExpander::taskEntry(void* param) {
  IOExpander* inst = reinterpret_cast<IOExpander*>(param);
  if (inst) inst->run();
  vTaskDelete(nullptr);
}