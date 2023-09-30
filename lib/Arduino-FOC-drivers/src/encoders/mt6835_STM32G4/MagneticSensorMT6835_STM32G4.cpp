
#include "MagneticSensorMT6835_STM32G4.h"


MagneticSensorMT6835_STM32G4::MagneticSensorMT6835_STM32G4(int nCS) : Sensor(), MT6835_STM32G4(nCS) {
    // nix
};


MagneticSensorMT6835_STM32G4::~MagneticSensorMT6835_STM32G4() {
    // nix
};


float MagneticSensorMT6835_STM32G4::getSensorAngle() {
    return getCurrentAngle();
};


void MagneticSensorMT6835_STM32G4::init() {
    this->MT6835_STM32G4::init();
    this->Sensor::init();
};

