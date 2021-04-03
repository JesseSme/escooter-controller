// Name:								SSRelay.cpp
// Author:						Jesse Smedberg
// Description:	Handles the escooter's soft starter relay.

#include "SSRelay.h"

bool _RelaySetup = 0;
bool _PowerState = 0;


void _setupRelay() {
				pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
				_RelaySetup = 1;
}


int switchRelayState(int state) {

				if (!_RelaySetup) {
								_setupRelay();
				} 
				
				if (_PowerState == state) {
								return 1;
				}
				else if (state == 1) {
								digitalWrite(RELAY_PIN, HIGH);
        delay(700);
        digitalWrite(RELAY_PIN, LOW);
								_PowerState = state;
								return 2;
				}
    else if (state != _PowerState) {
        digitalWrite(RELAY_PIN, HIGH);
        delay(2000);
        digitalWrite(RELAY_PIN, LOW);
								_PowerState = state;
        return 3;
    }

				return 0;
}

