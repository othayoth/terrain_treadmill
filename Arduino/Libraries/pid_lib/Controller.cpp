#include "Arduino.h"
#include "Controller.h"
#include "Utilities.h"
#include "Encoder.h"
#include "DueTimer.h"
#include "Motors.h"
#include "Utilities.h"

// PID gains
static volatile float Kp;
static volatile float Ki;
static volatile float Kd;

// Contains error, desired position
static volatile Controls M1;
static volatile Controls M2;
static volatile Controls M3;

// Velocities
static volatile float v1;
static volatile float v2;
static volatile float v3;

// Setup motor controlller to interrupt at desired frequency
void controller_setup(int ControllerFreq)
{
	Timer3.attachInterrupt(motor_ISR).setFrequency(ControllerFreq).start();
}


// Called at desired frequency
// Behaviour depends on operating mode
void motor_ISR(void)
{
	switch (getMODE())
	{
	case POSITION_HOLD: 
	{
		// Update control effort for each motor
		PID_Controller(M1.desired, encoder_counts(1),1);
		PID_Controller(M2.desired, encoder_counts(2),2);
		PID_Controller(M3.desired, encoder_counts(3),3);
		break;
	}
	case VELOCITY_HOLD:
	{
		// Find velocity
		v1 = filtered_velocity(1);
		v2 = filtered_velocity(2);
		v3 = filtered_velocity(3);

		// Update control effort
		PID_Controller(M1.desired, v1, 1);
		PID_Controller(M2.desired, v2, 2);
		PID_Controller(M3.desired, v3, 3);
	}
	case IDLE:
		// Do nothing
		break;
	}
}

// Return velocity
float get_velocity(int motor)
{
	static float velocity;

	if (motor == 1)
	{
		velocity = v1;
	}
	else if (motor == 2)
	{
		velocity = v2;
	}
	else if (motor == 3)
	{
		velocity = v3;
	}

	return velocity;
}

// Set new desired angle for motor
// takes angle in counts
void set_desired_angle(int angle, int motor)
{
	if (motor == 1)
	{
		M1.desired = angle;
	}
	else if (motor == 2)
	{
		M2.desired = angle;
	}
	else if (motor == 3)
	{
		M3.desired = angle;
	}
}

// Set new desired velocity for motor
// takes velocity in counts/ControllerFreq seconds
void set_desired_velocity(float velocity, int motor)
{
	if (motor == 1)
	{
		M1.desired = velocity;
	}
	else if (motor == 2)
	{
		M2.desired = velocity;
	}
	else if (motor == 3)
	{
		M3.desired = velocity;
	}
}

// Reset error for all of the motors
void reset_controller_error(void)
{
	M1.Eold = 0;
	M1.Enew = 0;
	M1.Eint = 0;
	M1.Edot = 0;

	M2.Eold = 0;
	M2.Enew = 0;
	M2.Eint = 0;
	M2.Edot = 0;

	M3.Eold = 0;
	M3.Enew = 0;
	M3.Eint = 0;
	M3.Edot = 0;
}

// Calculate control effort and set pwm value to control motor
void PID_Controller(float reference, float actual,int motor)
{
	static float u;
	
	if (motor == 1)
	{
		M1.Enew = reference - actual;				// Calculate error
		M1.Eint = M1.Eint + M1.Enew;                // Calculate intergral error
		M1.Edot = M1.Enew - M1.Eold;                // Calculate derivative error
		M1.Eold = M1.Enew;                          // Update old error
		u = Kp*M1.Enew + Ki*M1.Eint + Kd*M1.Edot;   // Calculate effort
	}
	else if (motor == 2)
	{
		M2.Enew = reference - actual;				// Calculate error
		M2.Eint = M2.Eint + M2.Enew;				// Calculate intergral error
		M2.Edot = M2.Enew - M2.Eold;                // Calculate derivative error
		M2.Eold = M2.Enew;                          // Update old error
		u = Kp*M2.Enew + Ki*M2.Eint + Kd*M2.Edot;   // Calculate effort
	}
	else if (motor == 3)
	{
		M3.Enew = reference - actual;				// Calculate error
		M3.Eint = M3.Eint + M3.Enew;                // Calculate intergral error
		M3.Edot = M3.Enew - M3.Eold;                // Calculate derivative error
		M3.Eold = M3.Enew;                          // Update old error
		u = Kp*M3.Enew + Ki*M3.Eint + Kd*M3.Edot;   // Calculate effort
	}

	// Determine correct direction of rotation
	if (u > 0)
	{
		motor_direction(0, motor);
	} 
	else
	{
		motor_direction(1, motor);
		u = abs(u);
	}
	
	// Max effort
	if (u > 255)
	{
		u = 255;
	}
	
	// Set new control effort
	setPWM(u,motor);
}

// Set PID gains used by controller
void set_gains(float kp, float ki, float kd)
{
	noInterrupts();
	Kp = kp;
	Ki = ki;
	Kd = kd;
	interrupts();
}