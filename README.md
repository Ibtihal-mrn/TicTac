# Eurobot competition : Tictac


### Technical Specifications :

#### Microcontroller = esp32-s3-devkitc-1
```
Model: ESP32
Cores: 2
Revision: 0
Features: WiFi BLE 
Free heap: 369972 bytes
CPU frequency: 240 MHz
SDK version: v4.4.7-dirty
Sketch size: 293056 bytes
Free sketch space: 3342336 bytes
Chip ID: C31DB410
```

### Motors - GB37-100

Source : polulu.com/product/4695

Red : Motor Power Terminal + .
Black : Motor Power Terminal -.
Green : Encoder GND.
Yellow : Encoder A output. 
White : Encoder B output.


Todo :

- Encoders ok
- Steppers ok
- Ultrasonic sensors x3
- LiDars


- Gyro (ok detects)
- Motors : change Driver

- freeRtos
- Commands Queue




## TODO:

- US Health Routine.
- add freinage to stop.



clarifions control.cpp
1. je vais le renommer par pid.cpp
2. je vais introduire une structure du genre : 
struct PID {
    float kp, ki, kd;
    float integral, previousError;

    PID(float _kp=0, float _ki=0, float _kd=0) : kp(_kp), ki(_ki), kd(_kd), integral(0), previousError(0) {}
};

const PID DISTANCE_PID_DEFAULT(1.5f, 0.0f, 0.2f); // KP, KI, KD initiaux pour la distance
const PID ANGLE_PID_DEFAULT   (2.0f, 0.0f, 0.3f); // KP, KI, KD initiaux pour l'angle

