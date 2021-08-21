#include "Arduino.h"
#include "Utilities.h"

static volatile mode MODE = IDLE;	// Operating Mode

// Set new operating mode
void setMODE(mode newMODE) {
	MODE = newMODE;
}

// Get operating mode
mode getMODE() {
	return MODE;
}