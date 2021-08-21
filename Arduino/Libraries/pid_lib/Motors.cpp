#include "Motors.h"
#include "Arduino.h"

// Pins for driving motors
#define Motor1Dir1 50
#define Motor1Dir2 51
#define Motor1pwm 10

#define Motor2Dir1 48
#define Motor2Dir2 49
#define Motor2pwm 9

#define Motor3Dir1 46
#define Motor3Dir2 47
#define Motor3pwm 8

void motor_setup(void)
{
	// Setup Motor 1 pins
	pinMode(Motor1Dir1, OUTPUT);
	pinMode(Motor1Dir2, OUTPUT);
	pinMode(Motor1pwm, OUTPUT);
	digitalWrite(Motor1Dir1, LOW);
	digitalWrite(Motor1Dir2, LOW);

	// Setup Motor 2 pins
	pinMode(Motor2Dir1, OUTPUT);
	pinMode(Motor2Dir2, OUTPUT);
	pinMode(Motor2pwm, OUTPUT);
	digitalWrite(Motor2Dir1, LOW);
	digitalWrite(Motor2Dir2, LOW);

	// Setup Motor 3 pins
	pinMode(Motor3Dir1, OUTPUT);
	pinMode(Motor3Dir2, OUTPUT);
	pinMode(Motor3pwm, OUTPUT);
	digitalWrite(Motor3Dir1, LOW);
	digitalWrite(Motor3Dir2, LOW);
}

// Set control effort
void setPWM(int value, int motor)
{
	static int pin;
	if (motor == 1)
	{
		pin = Motor1pwm;
	}
	else if (motor == 2)
	{
		pin = Motor2pwm;
	}
	else if (motor == 3)
	{
		pin = Motor3pwm;
	}

	//account for inverting amplifying circuitry
	value = 255 - value;

	analogWrite(pin, value);
}

// Set motor spinning direction
void motor_direction(int direction, int motor)
{
	static int pin1, pin2;

	// Determine correct pins based on motor selection
	if (motor == 1)
	{
		pin1 = Motor1Dir1;
		pin2 = Motor1Dir2;
	}
	else if (motor == 2)
	{
		pin1 = Motor2Dir1;
		pin2 = Motor2Dir2;
	}
	else if (motor == 3)
	{
		pin1 = Motor3Dir1;
		pin2 = Motor3Dir2;
	}

	// Update motor direction
	if (direction == 0)
	{
		// forward
		digitalWrite(pin1, LOW);
		digitalWrite(pin2, HIGH);
	} 
	else if (direction == 1)
	{
		// backward
		digitalWrite(pin1, HIGH);
		digitalWrite(pin2, LOW);
	}
	else
	{
		// stop
		digitalWrite(Motor1Dir1, LOW);
		digitalWrite(Motor1Dir2, LOW);
	}
}