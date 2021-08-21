#include "Encoder.h"
#include "Arduino.h"

/*

Port registers allow for lower-level and faster manipulation of the i/o pins of the microcontroller 
on an Arduino board. Using port registers to update encoder values in interrupts is the only method 
which is fast enough using a single microcontroller. For more information, see: 
	
	https://www.arduino.cc/en/Reference/PortManipulation
	http://forum.arduino.cc/index.php?topic=260731.0
	https://www.arduino.cc/en/Hacking/PinMappingSAM3X

	Pin Number 	SAM3X Pin Name
	5			PC25
	6			PC24
	18			PA11
	19			PA10
	20			PB12
	21			PB13

Encoder Code used reduces resolution of any encoder by 4. This is done becuase of the tradeoff between 
encoder resolution and computational load.

Pololu 100:1 Metal Gearmotor 37Dx73L mm with 64 CPR Encoder. 

Encoder is located on output shaft and precise gearratio is 102.083:1 leaving full encoder resolution of
6533 counts per revolution. Divide this by four for this version of the code -> 1633.25 counts per revolution. 
This corresponds to 0.22041940915 counts per degree of rotation

*/

#define DPC 0.220419409	// degrees per count

static volatile float SAMPLE_NUMBER; // Number of samples for moving average

// Motor1
#define Motor1PORT PIOC							// Port register for motor 1 interrupt pins
static int Motor1pinA = 5;						// Interrupt pin definitions
static int Motor1pinB = 6;
static int Motor1pinAmask = 0x02000000;			// PC25
static int Motor1pinBmask = 0x01000000;			// PC24
static int Motor1mask = 0x03000000;				// PC24 + PC25
static volatile bool Motor1aFlag = 0;			// Flag that were expecting a rising edge on Motor1pinA to signal that the encoder has arrived at a detent
static volatile bool Motor1bFlag = 0;			// Flag that were expecting a rising edge on Motor1pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
static volatile int Motor1encoderPos = 0;		// Current encoder position
static volatile int Motor1encoderPosOld = 0;	// Previous encoder position
static volatile float Motor1velocityTotal = 0; 	// Velocity total for moving average

// Motor2
#define Motor2PORT PIOA						// Port register containing for motor 2 interrupt pins
static int Motor2pinA = 18;					
static int Motor2pinB = 19;
static int Motor2pinAmask =  0x00000800;	// PA11
static int Motor2pinBmask =  0x00000400;	// PA10
static int Motor2mask = 0x00000C00;			// PA11 + PA10
static volatile bool Motor2aFlag = 0;		
static volatile bool Motor2bFlag = 0;		
static volatile int Motor2encoderPos = 0;
static volatile int Motor2encoderPosOld = 0;
static volatile float Motor2velocityTotal = 0;

// Motor3
#define Motor3PORT PIOB						// Port register containing for motor 3 interrupt pins
static int Motor3pinA = 20;					
static int Motor3pinB = 21;
static int Motor3pinAmask =  0x00001000;	// PB12
static int Motor3pinBmask =  0x00002000;	// PB13
static int Motor3mask = 0x00003000;			// PB12 + PB13
static volatile bool Motor3aFlag = 0;		
static volatile bool Motor3bFlag = 0;		
static volatile int Motor3encoderPos = 0;	
static volatile int Motor3encoderPosOld = 0;
static volatile float Motor3velocityTotal = 0;

// Setup encoder pins for all three motors
void encoder_setup(float sample_number) {

	SAMPLE_NUMBER = sample_number;

	// Motor1
	pinMode(Motor1pinA, INPUT_PULLUP); 
	pinMode(Motor1pinB, INPUT_PULLUP); 
	attachInterrupt(digitalPinToInterrupt(Motor1pinA), Motor1pinA_ISR, RISING); 
	attachInterrupt(digitalPinToInterrupt(Motor1pinB), Motor1pinB_ISR, RISING); 
	
	// Motor2
	pinMode(Motor2pinA, INPUT_PULLUP);
	pinMode(Motor2pinB, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(Motor2pinA), Motor2pinA_ISR, RISING);
	attachInterrupt(digitalPinToInterrupt(Motor2pinB), Motor2pinB_ISR, RISING);

	// Motor3
	pinMode(Motor3pinA, INPUT_PULLUP);
	pinMode(Motor3pinB, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(Motor3pinA), Motor3pinA_ISR, RISING);
	attachInterrupt(digitalPinToInterrupt(Motor3pinB), Motor3pinB_ISR, RISING);
}

// Read the current motor position in counts
int encoder_counts(int motor)
{
	static int counts;

	if (motor == 1)
	{
		counts = Motor1encoderPos;
	}
	else if (motor == 2)
	{
		counts = Motor2encoderPos;
	}
	else if (motor == 3)
	{
		counts = Motor3encoderPos;
	}

	return counts;
}

// Read the current motor position in degrees
float encoder_degrees(int motor)
{
	static float degrees;
	degrees = ((float)encoder_counts(motor)) * DPC;
	return degrees;
}

// Reset the current motor position back to zero
void encoder_reset(int motor)
{
	if (motor == 1)
	{
		Motor1encoderPos = 0;
	}
	else if (motor == 2)
	{
		Motor2encoderPos = 0;
	}
	else if (motor == 3)
	{
		Motor3encoderPos = 0;
	}
}

// Read previous motor position
int encoder_counts_old(int motor)
{
	static int old;

	if (motor == 1)
	{
		old = Motor1encoderPosOld;
	}
	else if (motor == 2)
	{
		old = Motor2encoderPosOld;
	}
	else if (motor == 3)
	{
		old = Motor3encoderPosOld;
	}

	return old;
}

// Update previous motor position
void encoder_counts_old_update(int motor)
{
	if (motor == 1)
	{
		Motor1encoderPosOld = encoder_counts(1);
	}
	else if (motor == 2)
	{
		Motor2encoderPosOld = encoder_counts(2);
	}
	else if (motor == 3)
	{
		Motor3encoderPosOld = encoder_counts(3);
	}
}

// Read encoder velocity in counts
int encoder_velocity(int motor)
{
	static int velocity;

	velocity = encoder_counts(motor) - encoder_counts_old(motor);

	encoder_counts_old_update(motor);

	return velocity;
}

// Read moving average filtered velocity in counts
float filtered_velocity(int motor)
{
	static float filtered_velocity;

	if (motor == 1)
	{
		Motor1velocityTotal = Motor1velocityTotal + encoder_velocity(motor) - Motor1velocityTotal / SAMPLE_NUMBER;
		filtered_velocity = Motor1velocityTotal / SAMPLE_NUMBER;
	}
	else if (motor == 2)
	{
		Motor2velocityTotal = Motor2velocityTotal + encoder_velocity(motor) - Motor2velocityTotal / SAMPLE_NUMBER;
		filtered_velocity = Motor2velocityTotal / SAMPLE_NUMBER;
	}
	else if (motor == 3)
	{
		Motor3velocityTotal = Motor3velocityTotal + encoder_velocity(motor) - Motor3velocityTotal / SAMPLE_NUMBER;
		filtered_velocity = Motor3velocityTotal / SAMPLE_NUMBER;
	}

	return filtered_velocity;
}

void Motor1pinA_ISR(void) 
{
	static int reading;
	noInterrupts(); // Pause interrupts
	reading = (Motor1PORT -> PIO_PDSR) & Motor1mask;	// Read port and strip away extra pins 
	if (reading == Motor1mask && Motor1aFlag) {			// Both pins at detent (HIGH) and expecting detent on this pin's rising edge
		Motor1encoderPos--;								// Decrement the encoder's position count
		Motor1bFlag = 0;								// Reset flags
		Motor1aFlag = 0;
	}
	else if (reading == Motor1pinAmask) Motor1bFlag = 1; // Set flag expecting Motor1pinB to signal the transition to detent from free rotation
	interrupts(); // Resume interrupts
}

void Motor1pinB_ISR(void) 
{
	static int reading;
	noInterrupts(); /// Pause interrupts
	reading = (Motor1PORT -> PIO_PDSR) & Motor1mask;	// Read port and strip away extra pins
	if (reading == Motor1mask && Motor1bFlag) {			// Both pins at detent (HIGH) and expecting detent on this pin's rising edge
		Motor1encoderPos++;								// Increment the encoder's position count
		Motor1bFlag = 0;								// Reset flags
		Motor1aFlag = 0;
	}
	else if (reading == Motor1pinBmask) Motor1aFlag = 1; // Set flag expecting Motor1pinA to signal the transition to detent from free rotation
	interrupts(); // Resume interrupts
}

void Motor2pinA_ISR(void) 
{
	static int reading;
	noInterrupts(); // Pause interrupts
	reading = (Motor2PORT -> PIO_PDSR) & Motor2mask;	// Read port and strip away extra pins
	if (reading == Motor2mask && Motor2aFlag) {			// Both pins at detent (HIGH) and expecting detent on this pin's rising edge
		Motor2encoderPos--;								// Decrement the encoder's position count
		Motor2bFlag = 0;								// Reset flags
		Motor2aFlag = 0; 
	}
	else if (reading == Motor2pinAmask) Motor2bFlag = 1; // Set flag expecting Motor2pinB to signal the transition to detent from free rotation
	interrupts(); // Resume interrupts
}

void Motor2pinB_ISR(void) 
{
	static int reading;
	noInterrupts(); // Pause interrupts
	reading = (Motor2PORT -> PIO_PDSR) & Motor2mask;	// Read port and strip away extra pins
	if (reading == Motor2mask && Motor2bFlag) {			// Both pins at detent (HIGH) and expecting detent on this pin's rising edge
		Motor2encoderPos++;								// Increment the encoder's position count
		Motor2bFlag = 0;								// Reset flags
		Motor2aFlag = 0;
	}
	else if (reading == Motor2pinBmask) Motor2aFlag = 1; // Set flag expecting Motor2pinA to signal the transition to detent from free rotation
	interrupts(); // Resume interrupts
}

void Motor3pinA_ISR(void) 
{
	static int reading;
	noInterrupts(); // Pause interrupts
	reading = (Motor3PORT -> PIO_PDSR) & Motor3mask;	// Read port and strip away extra pins
	if (reading == Motor3mask && Motor3aFlag) {			// Both pins at detent (HIGH) and expecting detent on this pin's rising edge
		Motor3encoderPos--;								// Decrement the encoder's position count
		Motor3bFlag = 0;								// Reset flags
		Motor3aFlag = 0;
	}
	else if (reading == Motor3pinAmask) Motor3bFlag = 1; // Set flag expecting Motor3pinB to signal the transition to detent from free rotation
	interrupts(); // Resume interrupts
}

void Motor3pinB_ISR(void) 
{
	static int reading;
	noInterrupts(); // Pause interrupts
	reading = (Motor3PORT -> PIO_PDSR) & Motor3mask;	// Read port and strip away extra pins
	if (reading == Motor3mask && Motor3bFlag) {			// Both pins at detent (HIGH) and expecting detent on this pin's rising edge
		Motor3encoderPos++;								// Increment the encoder's position count
		Motor3bFlag = 0;								// Reset flags for the next turn
		Motor3aFlag = 0;
	}
	else if (reading == Motor3pinBmask) Motor3aFlag = 1; // Set flag expecting Motor3pinA to signal the transition to detent from free rotation
	interrupts(); // Resume interrupts
}

