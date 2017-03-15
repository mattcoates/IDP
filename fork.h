#ifndef FORK_H
#define FORK_H

/* Direction Macros*/
#define UP 1
#define DOWN 0

/* Motor Parameters */
#define LIFT_SPEED_UP   127
#define LIFT_SPEED_DOWN 80

/* LDR Thresholds */
#define RED_LOW     36
#define RED_HIGH    75
#define GREEN_LOW   29
#define GREEN_HIGH  36
#define BLACK_LOW   0 
#define BLACK_HIGH  29
#define WHITE_LOW   75
#define WHITE_HIGH  255

/* Colour Macros */
#define RED     1
#define GREEN   2
#define BLACK   3
#define WHITE   4


/* Function Prototypes */
void lift(int direction);
int pallet_detect(void);

#endif
