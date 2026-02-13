#include <Arduino.h>
#include <Wire.h>


void i2c_scanner() {
  Serial.println("I2C Scanner");
  byte count = 0;
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(i, HEX);
      count++;
      delay(10);
    }
  }
  if(count == 0) Serial.println("No I2C devices found");
}

void printEsp32Info() {
    Serial.println("=== ESP32 Chip Info ===");
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    Serial.printf("Model: ESP32\n");
    Serial.printf("Cores: %d\n", chip_info.cores);
    Serial.printf("Revision: %d\n", chip_info.revision);
    Serial.printf("Features: %s%s%s\n",
        (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi " : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "BLE " : "",
        (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "");

    // Serial.printf("Flash: %dMB %s\n",
    //     spi_flash_get_chip_size() / (1024 * 1024),
    //     (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("CPU frequency: %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("SDK version: %s\n", ESP.getSdkVersion());
    Serial.printf("Sketch size: %u bytes\n", ESP.getSketchSize());
    Serial.printf("Free sketch space: %u bytes\n", ESP.getFreeSketchSpace());
    Serial.printf("Chip ID: %06X\n", (uint32_t)ESP.getEfuseMac());
    // Serial.printf("MAC address: %s\n", WiFi.macAddress().c_str());
    // Serial.printf("Flash chip ID: 0x%X\n", spi_flash_get_id());
    // Serial.printf("Flash speed: %u Hz\n", spi_flash_get_speed());
    // Serial.printf("Flash mode: %u\n", spi_flash_get_mode());
}
