// Name:								SSRelay.cpp
// Author:						Jesse Smedberg
// Description:	Handles the escooter's soft starter relay.

#include "SSRelay.h"

bool _RelaySetup = 0;


void _setupRelay() {
				pinMode(RELAY_PIN, OUTPUT);
				_RelaySetup = 1;
}


int switchRelayState(int state) {

				if (!_RelaySetup) {
								_setupRelay();
				} 
				
				if (digitalRead(RELAY_PIN) == state) {
								return 1;
				}
				else {
								digitalWrite(RELAY_PIN, state);
								return 1;
				}

				return 0;
}

