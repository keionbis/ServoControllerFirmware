#ifndef MA702_H
#define MA702_H

#include <mbed.h>
#include "main.h"
#include <SPI.h>

//SPI Mode: MagAlpha Gen3 support SPI mode 3 and 0 [SPI_MODE3, SPI_MODE0]
#define MA_SPI_MODE_0       SPI_MODE0
#define MA_SPI_MODE_3       SPI_MODE3

class MA702 {
public:
    int rotations;

	MA702();
	void  begin();
	void end();
	double readAngle();
	uint16_t readAngleRaw();
	uint16_t readAngleRaw(bool* error);
	uint16_t readAngleRaw16();
	uint8_t readAngleRaw8();
	uint8_t readRegister(uint8_t address);
	uint8_t writeRegister(uint8_t address, uint8_t value);
	void setSpiClockFrequency(uint32_t speedMaximum);
	void setSpiDataMode(uint8_t spiMode);
	void setSpiChipSelectPin(uint8_t spiChipSelectPin);
	double convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle);
    long int totalAngle();

private:
	int _last_angle;
	int _init_angle;
};

#endif //MAGALPHA_H
