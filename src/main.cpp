#include "main.h"
#include "PIDController.h"
#include "MA702.h"
DigitalOut myled(signaling);
Mutex stdio_mutex;
Ticker pidTimer;           // implements a timer


SPISlave Comms(MOSIPin, MISOPin, SCKPin, CommsCSPin); // mosi, miso, sclk, ssel
InterruptIn CS (CommsCSPin);
float i = 0;
int v;
static PIDimp * pid;

void runPid() {
	// update all positions fast and together

		pid->updatePosition();
	// next update all control outputs
		pid->updateControl();
}


void COMMSStart(){
    stdio_mutex.lock();
	if(Comms.receive()) {
		v = Comms.read();   // Read byte from master

		Comms.reply(v);         // Make this the next reply
	}
}
void COMMSStop(){
    stdio_mutex.unlock();

}

int main() {
	pid = new PIDimp(new PwmOut(In2Pin), new PwmOut(In2Pin),  new MA702(),
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

	}

}
