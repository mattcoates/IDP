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

/* Prototypes */
void turn_90_anti_clockwise(void);

/* Output Powers */
const int base_power = 60;
const int delta_p = 7;

const int base_power_creep = 25;
const int delta_p_creep = 4;

const int turn_power_l = 36;
const int turn_power_r = 40;

const int ramp_power_l = 22;
const int ramp_power_r = 25;

/* Ramp Timings */
const int up_t1 = 1000;
const int up_t2 = 2000;
const int up_t3 = 8000;
const int up_t4 = 9500;

const int down_t1 = 2000;
const int down_t2 = 3000;
const int down_t3 = 9000;
const int down_t4 = 10500;

/* Turning Timings */
const int turning_grace = 1000;


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
    cout << "Creeping to Junction" << endl;

	/* Position axel over Junction Centre */
    while((robot.line & 0x08) == 0) {
        
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
    
    /* Reverse at following line until Near Side of Junction Detected */
    while(!(near_junction_detected)) {
    
        /* Read Line Sensors */
        read_line_sensors();
        
        /* Reverse */
        switch((robot.line & 0x07)) {
            
            /* 0 0 0 - Where's the line? */
            case 0:
                cout << "No line detected" << endl;
                stop(); 
                break;
            
            /* 0 0 1 - Reversing Serverly Right */
            case 1:
                left_motor((base_power_creep - (2*delta_p_creep)), REVERSE);
                right_motor((base_power_creep + (2*delta_p_creep)), REVERSE);
                cout << "Serverely Right" << endl;
                break;
        
            /* 0 1 0 - Following the Line */
            case 2:
                left_motor(base_power_creep, REVERSE);
                right_motor(base_power_creep, REVERSE);
                cout << "All Good" << endl; 
                break;
                
            /* 0 1 1 - Reversing Slightly Right */
            case 3:
                left_motor((base_power_creep - delta_p_creep), REVERSE);
                right_motor((base_power_creep + delta_p_creep), REVERSE);
                cout << "Slightly Right" << endl;  
                break;
                
            /* 1 0 0 - Reversing Serverly Left */
            case 4:
                left_motor((base_power_creep + (2*delta_p_creep)), REVERSE);
                right_motor((base_power_creep - (2*delta_p_creep)), REVERSE); 
                cout << "Serverely Left" << endl; 
                break;
   
            /* 1 0 1 - What the actual */
            case 5:
                cout << "I am a confused robot" << endl;
                stop(); 
                break;

            /* 1 1 0 - Reversing Slightly Left */
            case 6:
                left_motor((base_power_creep + delta_p_creep), REVERSE);
                right_motor((base_power_creep - delta_p_creep), REVERSE); 
                cout << "Slightly Left" << endl; 
                break;        
	    }      
        
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
        switch((robot.line & 0x07)) {
            
            /* 0 0 0 - Where's the line? */
            case 0:
                cout << "No line detected" << endl;
                stop(); 
                break;
            
            /* 0 0 1 - Reversing Serverly Right */
            case 1:
                left_motor((base_power_creep - (2*delta_p_creep)), REVERSE);
                right_motor((base_power_creep + (2*delta_p_creep)), REVERSE);
                cout << "Serverely Right" << endl;
                break;
        
            /* 0 1 0 - Following the Line */
            case 2:
                left_motor(base_power_creep, REVERSE);
                right_motor(base_power_creep, REVERSE);
                cout << "All Good" << endl; 
                break;
                
            /* 0 1 1 - Reversing Slightly Right */
            case 3:
                left_motor((base_power_creep - delta_p_creep), REVERSE);
                right_motor((base_power_creep + delta_p_creep), REVERSE);
                cout << "Slightly Right" << endl;  
                break;
                
            /* 1 0 0 - Reversing Serverly Left */
            case 4:
                left_motor((base_power_creep + (2*delta_p_creep)), REVERSE);
                right_motor((base_power_creep - (2*delta_p_creep)), REVERSE); 
                cout << "Serverely Left" << endl; 
                break;
   
            /* 1 0 1 - What the actual */
            case 5:
                cout << "I am a confused robot" << endl;
                stop(); 
                break;

            /* 1 1 0 - Reversing Slightly Left */
            case 6:
                left_motor((base_power_creep + delta_p_creep), REVERSE);
                right_motor((base_power_creep - delta_p_creep), REVERSE); 
                cout << "Slightly Left" << endl; 
                break;        
	    }
        
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

    /* Turn until Junction Sensor goes White w/ 1 Second Grace Period */
    while((turning_watch.read() < turning_grace)||((robot.line & 0x08) == 0)) {
        read_line_sensors();
        left_motor(turn_power_l, REVERSE);
        right_motor(turn_power_r, FORWARD);
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


void traverse_ramp(int direction) {
        
    ramp_watch.start();    
    
    /* RAMP ISSUES:
     * Because the line sensors lift off the ground on exit and entry to
     * the ramp we are unable to trust them from t1 to t2 and from t3 to t4. 
     * In this period we will have to output an equal power to both wheels 
     * and correct when the sensors can be trusted again.
     */    
        
    if(direction == UPWARDS) {
    
        /* Junction Detection Flag */
	    bool junction_detected = false;

	    /* Loop until Junction Detected */
	    while(!(junction_detected)) { 
          
            /* Handle Sensor Trust Issues */
            if(((ramp_watch.read() > up_t1) && (ramp_watch.read() < up_t2)) ||  ((ramp_watch.read() > up_t3) && (ramp_watch.read() < up_t4))) {
            
                /* Dont Trust Sensors */
                left_motor(ramp_power_l, FORWARD);
                right_motor(ramp_power_r, FORWARD);
            
            
            } else {
            
                /* Read Line Sensors */
                read_line_sensors();
            
                /* Follow Line */
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
                    stop(); 
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
    }

    
    if(direction == DOWNWARDS) {
    
        /* Junction Detection Flag */
	    bool junction_detected = false;

	    /* Loop until Junction Detected */
	    while(!(junction_detected)) { 
          
            /* Handle Sensor Trust Issues */
            if(((ramp_watch.read() > down_t1) && (ramp_watch.read() < down_t2)) ||  ((ramp_watch.read() > down_t3) && (ramp_watch.read() < down_t4))) {
            
                /* Dont Trust Sensors */
                left_motor(ramp_power_l, FORWARD);
                right_motor(ramp_power_r, FORWARD);
            
            
            } else {
            
                /* Read Line Sensors */
                read_line_sensors();
            
                /* Follow Line */
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
                    stop(); 
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
    }
     
     ramp_watch.stop();
}

