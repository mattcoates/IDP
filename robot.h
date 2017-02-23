#ifndef ROBOT_H
#define ROBOT_H

/* Robot Number */
#define ROBOT_NUM  15

/* Heading Macros */
#define NORTH   1
#define EAST    2
#define SOUTH   3   
#define WEST    4

/* Location Macros */
#define P1      1
#define P2      2
#define D1      13
#define D2      15
#define D3      10

/* Colour Macros */
#define GREEN   1
#define WHITE   2
#define RED     3
#define BLACK   4
 
/* Global Robot Class */
class robot {
    public:
        /* Navigation */
        int heading;
        int location;
        
        /* Line Following */
        bool line[4];
        
        /* Payload Handling */     
        int pallet_colour;
};

#endif
