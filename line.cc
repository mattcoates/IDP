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
#include "mission.h"

using namespace std;

stopwatch turning_watch;

/* Prototypes */
void turn_90_anti_clockwise(void);

/* Output Powers */
const int base_power = 60;
const int delta_p = 10;
const int creep_power_l = 22;
const int creep_power_r = 25;
const int turn_power = 50;

void next_junction(void) {

	/* Junction Detection Flag */
	bool junction_detected = false;

	/* Loop until Junction Detected */
	while(!(junction_detected)) { 

        /* Read Line Sensors */
        read_line_sensors();
        
        /* DEBUG */
        cout << "L = " << ((robot.line & 0x04) >> 2) << " C = " << ((robot.line & 0x02) >> 1) << " R = " << (robot.line & 0x01) << "    J = " << ((robot.line & 0x08) >> 3);
        
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
                cout << "Serverely Left" << endl;
                break;
        
            /* 0 1 0 - Following the Line */
            case 2:
                left_motor(base_power, FORWARD);
                right_motor(base_power, FORWARD);
                cout << "All Good" << endl; 
                break;
                
            /* 0 1 1 - Moving Slightly Left */
            case 3:
                left_motor((base_power + delta_p), FORWARD);
                right_motor((base_power - delta_p), FORWARD);
                cout << "Slightly Left" << endl;  
                break;
                
            /* 1 0 0 - Moving Serverly Right */
            case 4:
                left_motor((base_power - (2*delta_p)), FORWARD);
                right_motor((base_power + (2*delta_p)), FORWARD); 
                cout << "Serverely Right" << endl; 
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
                cout << "Slightly Right" << endl; 
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
    
    /* DEBUG */
    cout << "Creeping" << endl;

	/* Position axel over Junction Centre */
    while((robot.line & 0x08) == 0) {        
	    left_motor(creep_power_l, FORWARD);
        right_motor(creep_power_r, FORWARD); 
        read_line_sensors();
	}
	
	stop();
}


void next_limit(void) {
     
     /* Object Detection Flag */
	bool object_detected = false;
	
	/* Loop until Junction Detected */
	while(!(object_detected)) { 

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
                
            /* 1 1 1 - Somthing has gone wrong */
            case 7:
                cout << "Junction detected?" << endl;
                stop();
                break;         
	    }
	    
	    /* Read Limit Switches */
	    read_limit_switches();
	    
	    /* Mask & Test for Collision */
	    if (((robot.limit & 0x01) == 0x01) || ((robot.limit & 0x02) == 0x02)) {
	        object_detected = true;
	        /* TODO: Update to ensure it is square using both limit switches */
	    }    
	}
	
	stop();
}


void back_up_from_limit(void) {

    /* Junction Detection Flag */
    bool near_junction_detected = false;
    bool far_junction_detected = false;
    
    /* Reverse at Equal Power until Near Side of Junction Detected */
    while(!(near_junction_detected)) {
    
        /* Read Line Sensors */
        read_line_sensors();
        
        /* Reverse */
        left_motor(creep_power_l, REVERSE);
        right_motor(creep_power_r, REVERSE); 
        
        /* Detect White */
        if((robot.line & 0x08) == 0x08) {
        
            near_junction_detected = true;
        }
    }
    
    /* Reverse until other side of line to position axel */
    while(!(far_junction_detected)) {
        
        /* Read Line Sensors */
        read_line_sensors();
        
        /* Reverse */
        left_motor(creep_power_l, REVERSE);
        right_motor(creep_power_r, REVERSE); 
        
        /* Detect Black */
        if((robot.line & 0x08) == 0) {
        
            far_junction_detected = true;
        }
    }
    
    stop();
}


void turn_90_anti_clockwise(void) {

    /* Start Stopwatch */
    turning_watch.start();

    /* Turn until Junction Sensor goes White w/ 1.5 Second Grace Period */
    while((turning_watch.read() < 1500)||((robot.line & 0x08) == 0)) {
        read_line_sensors();
        left_motor(turn_power, REVERSE);
        right_motor(turn_power, FORWARD);
    }
    
    stop();
    turning_watch.stop();
}


void change_heading(int old_heading, int new_heading) {

    /* Update Heading */
    robot.heading = new_heading;
   
    /* NORTH */
    if(old_heading == NORTH) {
        if(new_heading == EAST) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
        }
        else if(new_heading == SOUTH) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
                }
        else if(new_heading == WEST) {
            turn_90_anti_clockwise();
        }
        else {
            /* Do Nothing */
        }
    }
   
    /* WEST */
    if(old_heading == WEST) {
        if(new_heading == NORTH) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
        }
        else if(new_heading == EAST) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
                }
        else if(new_heading == SOUTH) {
            turn_90_anti_clockwise();
        }
        else {
            /* Do Nothing */
        }
    }

    /* SOUTH */
    if(old_heading == SOUTH) {
        if(new_heading == WEST) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
        }
        else if(new_heading == NORTH) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
                }
        else if(new_heading == EAST) {
            turn_90_anti_clockwise();
        }
        else {
            /* Do Nothing */
        }
    }

    /* EAST */
    if(old_heading == EAST) {
        if(new_heading == SOUTH) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
        }
        else if(new_heading == WEST) {
            turn_90_anti_clockwise();
            turn_90_anti_clockwise();
                }
        else if(new_heading == NORTH) {
            turn_90_anti_clockwise();
        }
        else {
            /* Do Nothing */
        }
    }
}


