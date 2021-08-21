#ifndef ENCODER_H
#define ENCODER_H

void encoder_setup(float sample_number);
int encoder_counts(int motor);
float encoder_degrees(int motor);
void encoder_reset(int motor);

int encoder_counts_old(int motor);
void encoder_counts_old_update(int motor);
int encoder_velocity(int motor);
float filtered_velocity(int motor);

void Motor1pinA_ISR(void);
void Motor1pinB_ISR(void);
void Motor2pinA_ISR(void);
void Motor2pinB_ISR(void);
void Motor3pinA_ISR(void);
void Motor3pinB_ISR(void);

#endif
