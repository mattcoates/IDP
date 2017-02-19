#ifndef ROBOT_H
#define ROBOT_H

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
 
/* Function Prototypes */
void next_junction(void);
void change_heading(uint8_t old_heading, uint8_t new_heading); 

/* Global Robot Class */
class robot {
    public:
        /* Navigation */
        uint8_t heading;
        uint8_t location;
        
        /* Line Following */
        bool line_det[4];
        
        /* Payload Handling */     
        uint8_t pallet_colour;
};

#endif
