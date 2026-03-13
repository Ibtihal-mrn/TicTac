#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ═══════════════════════════════════════════════════════════════════════════
//  I2C Mutex — OBLIGATOIRE pour partager le bus I2C entre plusieurs tâches
// ═══════════════════════════════════════════════════════════════════════════
//  Le bus I2C (Wire) n'est PAS thread-safe. Si l'IMU (Core 1) et l'IO
//  Expander (Core 0) essaient de lire/écrire en même temps → corruption.
//  Le mutex garantit qu'un seul périphérique accède au bus à la fois.
extern SemaphoreHandle_t i2cMutex;

// ═══════════════════════════════════════════════════════════════════════════
//  IO Expander — données partagées entre la tâche IOExp et la FSM
// ═══════════════════════════════════════════════════════════════════════════
//  La tâche IOExpander lit les pins toutes les 200ms et met à jour cette
//  structure. La FSM (ou n'importe quel code) peut la lire via le mutex.
struct IOExpanderData {
    bool teamSwitch;     // Pin P1 : switch de sélection d'équipe
    bool launchTrigger;  // Pin P3 : tirette de départ
    bool ready;          // true dès que la première lecture a réussi
};
extern SemaphoreHandle_t ioExpanderMutex;  // protège ioExpanderData
extern IOExpanderData    ioExpanderData;   // données lues par la tâche IOExp


