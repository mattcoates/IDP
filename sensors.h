#ifndef SENSORS_H
#define SENSORS_H

/* Colour Macros */
#define GREEN   1
#define WHITE   2
#define RED     3
#define BLACK   4

/* Threshold Macros */
#define WHITE_THRESHOLD	0
#define RED_THRESHOLD 	1
#define GREEN_THRESHOLD	2
#define BLACK_THRESHOLD	3

/* Function Prototypes */
void read_line_sensors(void);
void read_pallet_colour(void);
void read_limit_switches(void);

#endif
