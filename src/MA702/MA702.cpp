#include "MA702.h"
#include "mbed.h"
#include "SPI.h"
/***************************************************
  Mbed library for the MPS MA702 magnetic angle sensor
  Supports MA702 3rd generation Sensors. MA702 sensor detects the
  absolute angular position of a permanent magnet, typically a diametrically
  magnetized cylinder on the rotating shaft.
  ----> http://www.monolithicpower.com/Products/Position-Sensors/Products-Overview
  Written by Mathieu Kaelin for Monolithic Power Systems.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
//MA702 Read/Write Register Command
#define READ_REG_COMMAND    (0b010 << 13)
#define WRITE_REG_COMMAND   (0b100 << 13)

#define SSI_MODE            spi_MODE1

SPI spi(PA_7, PA_6, PA_5);
DigitalOut spiChipSelectPin(PA_1);

MA702::MA702(){
}

void  MA702::begin(){
	spiChipSelectPin = 1;
	spi.format(16,3);
	spi.frequency(1000000);
}

double MA702::readAngle(){
	uint16_t angle;
	double angleInDegree;
	angle = readAngleRaw16();
	angleInDegree = (angle*360.0)/65536.0;
	return angleInDegree;
}


uint16_t MA702::readAngleRaw16(){
	uint16_t angle;
	angle = spi.write(0x0000); //Read 16-bit angle
	spiChipSelectPin = 1;
	return angle;
}

uint8_t MA702::readAngleRaw8(){
	uint8_t angle;
	spiChipSelectPin = 0;
	angle = spi.write(0x00);     //Read 8-bit angle
	spiChipSelectPin = 1;
	return angle;
}

uint16_t MA702::readAngleRaw(bool* error){
	uint16_t angle;
	uint8_t parity;
	uint8_t highStateCount = 0;

	spiChipSelectPin = 0;
	angle = spi.write(0x0000);
	parity = spi.write(0x00);
	spiChipSelectPin = 1;

	parity = ((parity & 0x80) >> 7);
	//Count the number of 1 in the angle binary value
	for (int i=0;i<16;++i){
		if ((angle & (1 << i)) != 0){
			highStateCount++;
		}
	}
	//check if parity bit is correct
	if ((highStateCount % 2) == 0){
		if (parity == 0){
			*error = false;
		}
		else{
			*error = true;
		}
	}
	else{
		if (parity == 1){
			*error = false;
		}
		else{
			*error = true;
		}
	}
	double boundForWrap = 4096/6;
	  double maxForWrap =(4096-boundForWrap);
	  if(_last_angle>maxForWrap && angle<=boundForWrap)
	    rotations+=1;
	  else if(_last_angle<boundForWrap && angle>=maxForWrap)
	    rotations-=1;
	  _last_angle=angle;

	return angle;
}
long int MA702::totalAngle(){
    return readAngle()+rotations*4096 ;
}
uint8_t MA702::readRegister(uint8_t address){
	uint8_t readbackRegisterValue;
	spiChipSelectPin = 0;
	spi.write(READ_REG_COMMAND | ((address & 0x1F) << 8) | 0x00);
	spiChipSelectPin = 1;
	spiChipSelectPin = 0;
	readbackRegisterValue = ((spi.write(0x0000) & 0xFF00) >> 8);
	spiChipSelectPin = 1;
	return readbackRegisterValue;
}

uint8_t MA702::writeRegister(uint8_t address, uint8_t value){
	uint8_t readbackRegisterValue;
	spiChipSelectPin = 0;
	spi.write(WRITE_REG_COMMAND | ((address & 0x1F) << 8) | value);
	spiChipSelectPin = 1;
	wait(20);                      //Wait for 20ms
	spiChipSelectPin = 0;
	readbackRegisterValue = ((spi.write(0x0000) & 0xFF00) >> 8);
	spiChipSelectPin = 1;
	//readbackRegisterValue should be equal to the written value
	return readbackRegisterValue;
}

double MA702::convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle){
	double angleInDegree;
	angleInDegree = (rawAngle*360.0)/((double)pow(2, rawAngleDataBitLength));
	return angleInDegree;
}

