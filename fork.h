#ifndef FORK_H
#define FORK_H

/* Direction Macros*/
#define UP 1
#define DOWN 0

/* Motor Parameters */
#define LIFT_SPEED_UP   127
#define LIFT_SPEED_DOWN 70

/* LDR Thresholds */
#define RED_LOW     33
#define RED_HIGH    75
#define GREEN_LOW   28
#define GREEN_HIGH  33
#define BLACK_LOW   0 
#define BLACK_HIGH  27
#define WHITE_LOW   75
#define WHITE_HIGH  255

/* Colour Macros */
#define RED     1
#define GREEN   2
#define BLACK   3
#define WHITE   4

/* Forklift Positions */
#define VERY_BOTTOM 0
#define JUST_BELOW  1
#define JUST_ABOVE  2
#define STACKING    3

/* Function Prototypes */
void lift(int direction);
int pallet_detect(void);
void move_fork(int current_pos, int des_pos);

#endif
