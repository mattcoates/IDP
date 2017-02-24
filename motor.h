#ifndef MOTOR_H
#define MOTOR_H

#define FORWARD 1
#define REVERSE 0
#define STOP    0

#define RAMP_RATE   64

/* Function Prototypes */
void init_motor(void);
void left_motor(int speed, int direction);
void right_motor(int speed, int direction);
void stop(void);

#endif
