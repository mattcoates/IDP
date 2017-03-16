#ifndef ROBOT_H
#define ROBOT_H

/* Robot Number */
#define ROBOT_NUM  15

/* Global Robot Class */
class robot_data {
    public:
        /* Navigation */
        int heading;
        int location;
        
        /* IR Sensors */
        int line;
        
        /* Limit Switches */
        int limit;
        
        /* Payload Handling */    
        int pallet_colour;
	    int forklift_position;
	    bool stack_d2;
};

/* Global Robot Link */
extern robot_link rlink;

/* Global Robot Data */
extern robot_data robot;

#endif
