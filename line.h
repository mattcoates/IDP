#ifndef LINE_H
#define LINE_H

/* Heading Macros */
#define NORTH   1
#define EAST    2
#define SOUTH   3   
#define WEST    4

/* Direction Macros */
#define UPWARDS     8
#define DOWNWARDS   9


/* Function Prototypes */
void next_junction(void);
void next_limit(void);
void back_up_from_limit(void);
void change_heading(int old_heading, int new_heading); 
void traverse_ramp(int direction);
void turn_90_anti_clockwise(void);
void turn_90_clockwise(void);

#endif
