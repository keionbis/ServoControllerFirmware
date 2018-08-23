#include "main.h"
#include "PIDController.h"
#include "MA702.h"
typedef unsigned char Byte;

DigitalOut myled(signaling);
Mutex stdio_mutex;
Ticker pidTimer;           // implements a timer


SPISlave Comms(MOSIPin, MISOPin, SCKPin, CommsCSPin); // mosi, miso, sclk, ssel
InterruptIn CS (CommsCSPin);
int SetPoint[3];
float i = 0;
Byte CommsValues;
static PIDimp * pid;
float Feeedback[3];
void runPid() {
	// update all positions fast and together

	pid->updatePosition();
	// next update all control outputs
	pid->updateControl();
}



void COMMSStart(){
	stdio_mutex.lock();
	if(Comms.receive()) {
		CommsValues = Comms.read();   // Read byte from master
        SetPoint[1] = CommsValues>>24&0xFF;
        SetPoint[2] = CommsValues>>16&0xFF;
        SetPoint[3] = CommsValues>>8&0xFF;
        CommsValues = CommsValues>>24&0xFF;
        CommsValues = CommsValues|Feeedback[1]<<16|Feeedback[2]<<8|Feeedback[3];
        Comms.reply(CommsValues);


	}
}
void COMMSStop(){
	stdio_mutex.unlock();

}

int main() {
	pid = new PIDimp(new PwmOut(In2Pin),
			new PwmOut(In2Pin),
			new MA702(),
			new AnalogIn(FBPin));


	pid->state.config.Enabled = false;

	wait_ms(500);
	pidTimer.attach(&runPid, 0.0025);

	// capture 100 ms of encoders before starting
	wait_ms(100);

	Comms.reply(0x00);

	CS.rise(COMMSStart);
	CS.fall(COMMSStop);


	while(1) {
		Feeedback[0] = pid->getPosition();
		Feeedback[1] = pid->getVelocity();
		Feeedback[2] = pid->getTorque();
	}

}
