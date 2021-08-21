#ifndef MOTORS_H
#define MOTORS_H

void motor_setup(void);
void setPWM(int value, int motor);
void motor_direction(int direction, int motor);

#endif