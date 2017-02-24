#ifndef NAVIGATION_H
#define NAVIGATION_H

/* Location Macros */
#define P1      2
#define P2      4
#define C1	    0
#define C2	    15
#define D1      13
#define D2      16
#define D3      12

#define START_BOX   6
#define FINISH_BOX  10

/* Function Prototypes */
void travel(int current_location, int current_heading, int destination, int desired_heading);

#endif
