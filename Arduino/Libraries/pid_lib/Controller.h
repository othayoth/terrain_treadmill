#ifndef CONTROLLER_H
#define CONTROLLER_H

typedef struct {
	float Enew;
	float Eold;
	float Eint;
	float Edot;
	float desired;
} Controls;

void set_desired_angle(int angle, int motor);
void set_desired_velocity(float velocity, int motor);
void reset_controller_error(void);
void controller_setup(int ControllerFreq);
void motor_ISR(void);
void PID_Controller(float reference, float actual, int motor);
float get_velocity(int motor);
void set_gains(float kp, float ki, float kd);

#endif
