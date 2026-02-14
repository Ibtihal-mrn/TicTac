#pragma once
#include <Arduino.h>
#include <ESP32Servo.h> // pour ESP32 (S3)
// #include <Servo.h>  // pour Arduino

// Initialisation des bras
void bras_init();

// Déployer les bras
void bras_deployer();

// Rétracter les bras
void bras_retracter();
