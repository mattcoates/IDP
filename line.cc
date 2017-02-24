#include <iostream>
#include <robot_instr.h>
#include <robot_link.h>
#include <stopwatch.h>
#include <robot_delay.h>

#include "robot.h"
#include "line.h"
#include "motor.h"
#include "sensors.h"
#include "navigation.h"

using namespace std;

stopwatch turning_watch;

/* Prototypes */
void turn_90_anti_clockwise(void);

/* Output Powers */
const int base_power = 60;
const int delta_p = 8;
const int creep_power = 20;
const int turn_power = 50;

void next_limit(void) {
    
    /* 
     * TODO: Drive forwards following the line 
     *       until both limit switches trigger
     */

}
void back_up(void) {

    /* 
     * Use after "next_limit" to get back to track 
     *
     * TODO: Reverse at equal power until the  
     *       junction LED triggers white, then
     *       continue moving back until it
     *       goes black again.
     */

}


void next_junction(void) {

	/* Junction Detection Flag */
	bool junction_detected = false;

	/* Loop until Junction Detected */
	while(!(junction_detected)) { 

        /* Read Line Sensors */
        read_line_sensors();
        
        switch((robot.line & 0x07)) {
            
            /* 0 0 0 - Where's the line? */
            case 0:
                cout << "No line detected" << endl;
                stop(); 
                break;
            
            /* 0 0 1 - Moving Serverly Left */
            case 1:
                left_motor((base_power + (2*delta_p)), FORWARD);
                right_motor((base_power - (2*delta_p)), FORWARD); 
                break;
        
            /* 0 1 0 - Following the Line */
            case 2:
                left_motor(base_power, FORWARD);
                right_motor(base_power, FORWARD); 
                break;
                
            /* 0 1 1 - Moving Slightly Left */
            case 3:
                left_motor((base_power + delta_p), FORWARD);
                right_motor((base_power - delta_p), FORWARD); 
                break;
                
            /* 1 0 0 - Moving Serverly Right */
            case 4:
                left_motor((base_power - (2*delta_p)), FORWARD);
                right_motor((base_power + (2*delta_p)), FORWARD); 
                break;
   
            /* 1 0 1 - What the actual */
            case 5:
                cout << "I am a confused robot" << endl;
                stop(); 
                break;

            /* 1 1 0 - Moving Slightly Right */
            case 6:
                left_motor((base_power - delta_p), FORWARD);
                right_motor((base_power + delta_p), FORWARD); 
                break;
                
            /* 1 1 1 - Junction Detected */
            case 7:
                stop();
                junction_detected = true;
                break;         
	    }      
	}
	
    /* Read Line Sensors */
        read_line_sensors();

	/* Position axel over Junction Centre */
    while((robot.line & 0x08) == 0) {
	    left_motor(creep_power, FORWARD);
        right_motor(creep_power, FORWARD); 
        read_line_sensors();
	}	
}


void turn_90_anti_clockwise(void) {

    /* Start Stopwatch */
    turning_watch.start();

    /* Turn until Junction Sensor goes White w/ 1 Second Grace Period */
    while((turning_watch.read() < 1000)||((robot.line & 0x08) == 0)) {
        read_line_sensors();
        left_motor(turn_power, REVERSE);
        right_motor(turn_power, FORWARD);
    }

    turning_watch.stop();
}


void change_heading(int old_heading, int new_heading) {

    /* NORTH */
    if(old_heading == NORTH) {
        if(new_heading == EAST) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
        }
        if(new_heading == SOUTH) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
                }
        if(new_heading == WEST) {
            turn_90_anti_clockwise();
        }
    }
   
    /* WEST */
    if(old_heading == WEST) {
        if(new_heading == NORTH) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
        }
        if(new_heading == EAST) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
                }
        if(new_heading == SOUTH) {
            turn_90_anti_clockwise();
        }
    }

    /* SOUTH */
    if(old_heading == SOUTH) {
        if(new_heading == WEST) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
        }
        if(new_heading == NORTH) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
                }
        if(new_heading == EAST) {
            turn_90_anti_clockwise();
        }
    }

    /* EAST */
    if(old_heading == EAST) {
        if(new_heading == SOUTH) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
        }
        if(new_heading == WEST) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
                }
        if(new_heading == NORTH) {
            turn_90_anti_clockwise();
        }
    }
}


