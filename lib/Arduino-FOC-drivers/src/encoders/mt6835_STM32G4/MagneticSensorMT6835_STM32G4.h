
#pragma once

#include "common/base_classes/Sensor.h"
#include "./MT6835_STM32G4.h"

class MagneticSensorMT6835_STM32G4 : public Sensor, public MT6835_STM32G4 {
public:
	MagneticSensorMT6835_STM32G4(int nCS = -1);
	virtual ~MagneticSensorMT6835_STM32G4();

    virtual float getSensorAngle() override;

	virtual void init();
};


