#include <Arduino.h>
#include <Wire.h>
#include "BLEBridge.h"

void imAlive()
{
  static unsigned long millis_print = 0;
  if (millis() - millis_print >= 2000)
  {
    Serial.println("I'm alive");
    millis_print = millis();
  }
}

void i2c_scanner() {
  bleSerial.println("I2C Scanner");
  byte count = 0;
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      bleSerial.print("Found device at 0x");
      bleSerial.println(i, HEX);
      count++;
      delay(10);
    }
  }
  if(count == 0) bleSerial.println("No I2C devices found");
}

void printEsp32Info() {
    bleSerial.println("=== ESP32 Chip Info ===");
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    bleSerial.printf("Model: ESP32\n");
    bleSerial.printf("Cores: %d\n", chip_info.cores);
    bleSerial.printf("Revision: %d\n", chip_info.revision);
    bleSerial.printf("Features: %s%s%s\n",
        (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi " : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "BLE " : "",
        (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "");

    // Serial.printf("Flash: %dMB %s\n",
    //     spi_flash_get_chip_size() / (1024 * 1024),
    //     (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    bleSerial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
    bleSerial.printf("CPU frequency: %u MHz\n", ESP.getCpuFreqMHz());
    bleSerial.printf("SDK version: %s\n", ESP.getSdkVersion());
    bleSerial.printf("Sketch size: %u bytes\n", ESP.getSketchSize());
    bleSerial.printf("Free sketch space: %u bytes\n", ESP.getFreeSketchSpace());
    bleSerial.printf("Chip ID: %06X\n", (uint32_t)ESP.getEfuseMac());
    // Serial.printf("MAC address: %s\n", WiFi.macAddress().c_str());
    // Serial.printf("Flash chip ID: 0x%X\n", spi_flash_get_id());
    // Serial.printf("Flash speed: %u Hz\n", spi_flash_get_speed());
    // Serial.printf("Flash mode: %u\n", spi_flash_get_mode());
}
