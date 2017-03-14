#ifndef FORK_H
#define FORK_H

/* Direction Macros*/
#define UP 1
#define DOWN 0

/* Motor Parameters */
#define LIFT_SPEED_UP   127
#define LIFT_SPEED_DOWN 80

/* LDR Thresholds */
#define RED             x
#define RED_TOLERANCE   x
#define GREEN           x
#define GREEN_TOLERANCE x
#define BLACK           x
#define BLACK_TOLERANCE x
#define WHITE           x
#define WHITE_TOLERANCE x

/* Function Prototypes */
void lift(int direction);
int pallet_detect(void);

#endif
