#ifndef MOTOR_H
#define MOTOR_H

/* Direction Macros*/
#define FORWARD 1
#define REVERSE 0

#define UP 1
#define DOWN 0

#define HALT 0

/* Motor Parameters */
#define RAMP_RATE       64
#define LIFT_SPEED_UP   127
#define LIFT_SPEED_DOWN 60

/* Function Prototypes */
void motor_init(void);
void left_motor(int speed, int direction);
void right_motor(int speed, int direction);
void lift(int direction);
void stop(void);

#endif
