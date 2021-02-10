// SSRelay.h

#ifndef _SSRELAY_h
#define _SSRELAY_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define RELAY_PIN 3

void _setupRelay();
int switchRelayState(int state);


#endif

