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
stopwatch ramp_watch;
stopwatch reversing_watch;

/* Output Powers */
const int base_power = 65;
const int delta_p = 9;

const int base_power_creep = 30;
const int delta_p_creep = 5;

const int ramp_base_power = 80;
const int ramp_delta_p = 12;

const int ramp_power_l = 80;
const int ramp_power_r = 85;

const int reverse_power_l = 36;
const int reverse_power_r = 40;

const int turn_power_l = 36;
const int turn_power_r = 40;

/* Ramp Timings */
const int up_t1 = 4000;
const int up_t2 = 6800;
const int up_t3 = 11500;
const int up_t4 = 15000;

/* Reverse Timings */
const int reverse_watchdog = 3500;

/* Turning Timings */
const int turning_grace = 4000;


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
    cout << "Creeping to Junction" << endl;

	/* Position axel over Junction Centre */
    while((robot.line & 0x08) == 0) {
        
        /* DEBUG */
        cout << "L = " << ((robot.line & 0x04) >> 2) << " C = " << ((robot.line & 0x02) >> 1) << " R = " << (robot.line & 0x01) << "    J = " << ((robot.line & 0x08) >> 3);
        
        switch((robot.line & 0x07)) {
            
            /* 0 0 0 - Where's the line? */
            case 0:
                cout << "No line detected" << endl; 
                break;
            
            /* 0 0 1 - Moving Serverly Left */
            case 1:
                left_motor((base_power_creep + (2*delta_p_creep)), FORWARD);
                right_motor((base_power_creep - (2*delta_p_creep)), FORWARD);
                cout << "Serverely Left" << endl;
                break;
        
            /* 0 1 0 - Following the Line */
            case 2:
                left_motor(base_power_creep, FORWARD);
                right_motor(base_power_creep, FORWARD);
                cout << "All Good" << endl; 
                break;
                
            /* 0 1 1 - Moving Slightly Left */
            case 3:
                left_motor((base_power_creep + delta_p_creep), FORWARD);
                right_motor((base_power_creep - delta_p_creep), FORWARD);
                cout << "Slightly Left" << endl;  
                break;
                
            /* 1 0 0 - Moving Serverly Right */
            case 4:
                left_motor((base_power_creep - (2*delta_p_creep)), FORWARD);
                right_motor((base_power_creep + (2*delta_p_creep)), FORWARD); 
                cout << "Serverely Right" << endl; 
                break;

            /* 1 1 0 - Moving Slightly Right */
            case 6:
                left_motor((base_power_creep - delta_p_creep), FORWARD);
                right_motor((base_power_creep + delta_p_creep), FORWARD); 
                cout << "Slightly Right" << endl; 
                break;
            /* 1 1 1 - Continue */
            case 7:
                cout << "Continuing" << endl;
                left_motor(base_power_creep, FORWARD);
                right_motor(base_power_creep, FORWARD);                 
                break;         
	    }      
    
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
                break;
            
            /* 0 0 1 - Moving Serverly Left */
            case 1:
                left_motor((base_power_creep + (2*delta_p_creep)), FORWARD);
                right_motor((base_power_creep - (2*delta_p_creep)), FORWARD); 
                break;
        
            /* 0 1 0 - Following the Line */
            case 2:
                left_motor(base_power_creep, FORWARD);
                right_motor(base_power_creep, FORWARD); 
                break;
                
            /* 0 1 1 - Moving Slightly Left */
            case 3:
                left_motor((base_power_creep + delta_p_creep), FORWARD);
                right_motor((base_power_creep - delta_p_creep), FORWARD); 
                break;
                
            /* 1 0 0 - Moving Serverly Right */
            case 4:
                left_motor((base_power_creep - (2*delta_p_creep)), FORWARD);
                right_motor((base_power_creep + (2*delta_p_creep)), FORWARD); 
                break;
   
            /* 1 0 1 - What the actual */
            case 5:
                cout << "I am a confused robot" << endl;
                stop(); 
                break;

            /* 1 1 0 - Moving Slightly Right */
            case 6:
                left_motor((base_power_creep - delta_p_creep), FORWARD);
                right_motor((base_power_creep + delta_p_creep), FORWARD); 
                break;
                
            /* 1 1 1 - Somthing has gone wrong */
            case 7:
                cout << "Junction detected?" << endl;
                break;         
	    }
	    
	    /* Read Limit Switches */
	    read_limit_switches();
	    
	    /* Mask & Test for Collision */
	    if (((robot.limit & 0x40) == 0x00) || ((robot.limit & 0x20) == 0x00)) {
	        object_detected = true;
	    }    
	}
	
	/* Ensure Both Pressed */
	read_limit_switches();
	
	while(!(((robot.limit & 0x40) == 0x00) && ((robot.limit & 0x20) == 0x00))) {
	
	    if ((robot.limit & 0x40) == 0x00) {
	    
	        /* Turn Just One Wheel */
	        left_motor(30, FORWARD);
            right_motor(0, FORWARD);
	    
	    } else {
	    
	        /* Turn Other Wheel */
	        left_motor(0, FORWARD);
            right_motor(30, FORWARD);
	 
	    }
	    
	    read_limit_switches();
	}
	
	stop();
}


void back_up_from_limit(void) {

    /* TODO: Reversing from D3 needs timer */

    /* Junction Detection Flag */
    bool junction_detected = false;
    
    /* Reversing Watchdog */
    reversing_watch.start();
    
     while(!junction_detected) {
        
        /* Read Line Sensors */
        read_line_sensors();
        
        /* DEBUG */
        cout << "b0 = " << ((robot.line & 0x01)) << " b1 = " << ((robot.line & 0x02) >> 1) << " b2 = " << ((robot.line & 0x04) >> 2) << "    J = " << ((robot.line & 0x08) >> 3);
        
        /* Reverse until Junction or Watchdog */
        if(((robot.line & 0x07) == 0x07) || (reversing_watch.read() > reverse_watchdog)) {
                      
            /* 1 1 1 - Detected */
            cout << "Detected" << endl;
            stop();   
            junction_detected = true;  
                     
        } else {
        
            /* Reverse */
            left_motor(reverse_power_l, REVERSE);
            right_motor(reverse_power_r, REVERSE);             
	    }           
    }
    
    /* Stop Watch */
    reversing_watch.stop();
    
    /* Read Line Sensors */
    read_line_sensors();
    
    /* DEBUG */
    cout << "Creeping to Junction" << endl;

	/* Position axel over Junction Centre */
    while((robot.line & 0x08) == 0) {
        
        /* DEBUG */
        cout << "L = " << ((robot.line & 0x04) >> 2) << " C = " << ((robot.line & 0x02) >> 1) << " R = " << (robot.line & 0x01) << "    J = " << ((robot.line & 0x08) >> 3);
        
        switch((robot.line & 0x07)) {
            
            /* 0 0 0 - Where's the line? */
            case 0:
                cout << "No line detected" << endl;
                break;
            
            /* 0 0 1 - Moving Serverly Left */
            case 1:
                left_motor((base_power_creep + (2*delta_p_creep)), FORWARD);
                right_motor((base_power_creep - (2*delta_p_creep)), FORWARD);
                cout << "Serverely Left" << endl;
                break;
        
            /* 0 1 0 - Following the Line */
            case 2:
                left_motor(base_power_creep, FORWARD);
                right_motor(base_power_creep, FORWARD);
                cout << "All Good" << endl; 
                break;
                
            /* 0 1 1 - Moving Slightly Left */
            case 3:
                left_motor((base_power_creep + delta_p_creep), FORWARD);
                right_motor((base_power_creep - delta_p_creep), FORWARD);
                cout << "Slightly Left" << endl;  
                break;
                
            /* 1 0 0 - Moving Serverly Right */
            case 4:
                left_motor((base_power_creep - (2*delta_p_creep)), FORWARD);
                right_motor((base_power_creep + (2*delta_p_creep)), FORWARD); 
                cout << "Serverely Right" << endl; 
                break;

            /* 1 1 0 - Moving Slightly Right */
            case 6:
                left_motor((base_power_creep - delta_p_creep), FORWARD);
                right_motor((base_power_creep + delta_p_creep), FORWARD); 
                cout << "Slightly Right" << endl; 
                break;
            /* 1 1 1 - Continue */
            case 7:
                cout << "Continuing" << endl;
                left_motor(base_power_creep, FORWARD);
                right_motor(base_power_creep, FORWARD);                 
                break;         
	    }      
    
        read_line_sensors();
	}
	
	stop();
}


void turn_90_anti_clockwise(void) {

    cout << "Begin Turn Anticlockwise" << endl;

    /* Start Stopwatch */
    turning_watch.start();

    /* Turn until Junction Sensor goes White w/ Grace Period */
    while((turning_watch.read() < turning_grace)||((robot.line & 0x01) == 0)) {
        read_line_sensors();
        left_motor(turn_power_l, REVERSE);
        right_motor(turn_power_r, FORWARD);
    }
    
    stop();
    turning_watch.stop();
}


void turn_90_clockwise(void) {

    cout << "Begin Turn Clockwise" << endl;

    /* Start Stopwatch */
    turning_watch.start();

    /* Turn until Junction Sensor goes White w/ Grace Period */
    while((turning_watch.read() < turning_grace)||((robot.line & 0x02) == 0)) {
        read_line_sensors();
        left_motor(turn_power_l, FORWARD);
        right_motor(turn_power_r, REVERSE);
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
            turn_90_clockwise();
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
            turn_90_clockwise();
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
            turn_90_clockwise();
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
            turn_90_clockwise();
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


void traverse_ramp(int direction) {
        
    ramp_watch.start();    
     
    /* Going up the slide */
    if(direction == UPWARDS) {
    
        cout << "Ramp Approach" << endl;
        
        /* Handle Trust Issues */            
        while(ramp_watch.read() < up_t1) {
        
            /* Read Line Sensors */
            read_line_sensors();
                  
            switch((robot.line & 0x07)) {
            
                /* 0 0 0 - Where's the line? */
                case 0:
                    cout << "No line detected" << endl;
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

                /* 1 1 0 - Moving Slightly Right */
                case 6:
                    left_motor((base_power - delta_p), FORWARD);
                    right_motor((base_power + delta_p), FORWARD); 
                    cout << "Slightly Right" << endl; 
                    break;
                    
                /* 1 1 1 - Junction Detected */
                case 7:
                  cout << "Junction Detected?" << endl; 
                  break;         
	        }
        
        }
        
        cout << "Ramp Engaged" << endl;
        
        while(ramp_watch.read() < up_t2) {
        
            /* Move forward blind at 80 */
             left_motor(ramp_power_l, FORWARD);
             right_motor(ramp_power_r, FORWARD);
        
        }
        
        cout << "Ramp Comfortable" << endl;
        
        while(ramp_watch.read() < up_t3) {
        
            /* Read Line Sensors */
            read_line_sensors();
        
            /* Follow line at power 80 */             
            switch((robot.line & 0x07)) {
                
                /* 0 0 0 - Where's the line? */
                case 0:
                    cout << "No line detected" << endl;
                    break;
                
                /* 0 0 1 - Moving Serverly Left */
                case 1:
                    left_motor((ramp_base_power + (2*ramp_delta_p)), FORWARD);
                    right_motor((ramp_base_power - (2*ramp_delta_p)), FORWARD);
                    cout << "Serverely Left" << endl;
                    break;
            
                /* 0 1 0 - Following the Line */
                case 2:
                    left_motor(ramp_base_power, FORWARD);
                    right_motor(ramp_base_power, FORWARD);
                    cout << "All Good" << endl; 
                    break;
                    
                /* 0 1 1 - Moving Slightly Left */
                case 3:
                    left_motor((ramp_base_power + ramp_delta_p), FORWARD);
                    right_motor((ramp_base_power - ramp_delta_p), FORWARD);
                    cout << "Slightly Left" << endl;  
                    break;
                    
                /* 1 0 0 - Moving Serverly Right */
                case 4:
                    left_motor((ramp_base_power - (2*ramp_delta_p)), FORWARD);
                    right_motor((ramp_base_power + (2*ramp_delta_p)), FORWARD); 
                    cout << "Serverely Right" << endl; 
                    break;
       
                /* 1 0 1 - What the actual */
                case 5:
                    cout << "I am a confused robot" << endl;
                    stop(); 
                    break;

                /* 1 1 0 - Moving Slightly Right */
                case 6:
                    left_motor((ramp_base_power - ramp_delta_p), FORWARD);
                    right_motor((ramp_base_power + ramp_delta_p), FORWARD); 
                    cout << "Slightly Right" << endl; 
                    break;
                    
                /* 1 1 1 - Junction Detected */
                case 7:
                  stop();
                  cout << "Junction Detected?" << endl;
                  break;         
            }
        
        }
        
        cout << "Re-entry" << endl;
        
        while(ramp_watch.read() < up_t4) {
        
            /* Move forward blind at 80 */
            left_motor(ramp_power_l, FORWARD);
            right_motor(ramp_power_r, FORWARD); 
        
        }
        
        /* Read Line Sensors */
        read_line_sensors();
        
        /* DEBUG */
        cout << "Creeping to Junction" << endl;

	    /* Position axel over Junction Centre */
        while((robot.line & 0x08) == 0) {
            
            /* Follow Line */
            switch((robot.line & 0x07)) {
                
                /* 0 0 0 - Where's the line? */
                case 0:
                    cout << "No line detected" << endl; 
                    break;
                
                /* 0 0 1 - Moving Serverly Left */
                case 1:
                    left_motor((base_power_creep + (2*delta_p_creep)), FORWARD);
                    right_motor((base_power_creep - (2*delta_p_creep)), FORWARD);
                    cout << "Serverely Left" << endl;
                    break;
            
                /* 0 1 0 - Following the Line */
                case 2:
                    left_motor(base_power_creep, FORWARD);
                    right_motor(base_power_creep, FORWARD);
                    cout << "All Good" << endl; 
                    break;
                    
                /* 0 1 1 - Moving Slightly Left */
                case 3:
                    left_motor((base_power_creep + delta_p_creep), FORWARD);
                    right_motor((base_power_creep - delta_p_creep), FORWARD);
                    cout << "Slightly Left" << endl;  
                    break;
                    
                /* 1 0 0 - Moving Serverly Right */
                case 4:
                    left_motor((base_power_creep - (2*delta_p_creep)), FORWARD);
                    right_motor((base_power_creep + (2*delta_p_creep)), FORWARD); 
                    cout << "Serverely Right" << endl; 
                    break;
       
                /* 1 0 1 - What the actual */
                case 5:
                    cout << "I am a confused robot" << endl;
                    stop(); 
                    break;

                /* 1 1 0 - Moving Slightly Right */
                case 6:
                    left_motor((base_power_creep - delta_p_creep), FORWARD);
                    right_motor((base_power_creep + delta_p_creep), FORWARD); 
                    cout << "Slightly Right" << endl; 
                    break;        
	        }      
        
            read_line_sensors();
	    }
	    
	    stop();
	    ramp_watch.stop();             
    }
}

