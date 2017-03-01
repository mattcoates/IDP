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

const int base_power = 25;
const int delta_p = 4;

void reverse_test(void) {


    while(true) {
        
        /* Read Line Sensors */
        read_line_sensors();
        
        /* DEBUG */
        cout << "b0 = " << ((robot.line & 0x01)) << " b1 = " << ((robot.line & 0x02) >> 1) << " b2 = " << ((robot.line & 0x04) >> 2) << "    J = " << ((robot.line & 0x08) >> 3);
        
        /* Reverse */
        switch((robot.line & 0x07)) {
            
            /* 0 0 0 - Where's the line? */
            case 0:
                cout << "No line detected" << endl;
                break;
            
            /* 0 0 1 - Reversing Serverly */
            case 1:
                
                /* Junction Test */
                if(((robot.line) & 0x08) == 0x08) {               
                    left_motor((base_power + (2*delta_p)), REVERSE);
                    right_motor((base_power - (2*delta_p)), REVERSE);
                    cout << "Serverely Left" << endl;
                } else {
                    left_motor((base_power - (2*delta_p)), REVERSE);
                    right_motor((base_power + (2*delta_p)), REVERSE);
                    cout << "Serverely Right" << endl;
                }
                
            break;
        
            /* 0 1 0 - Following the Line */
            case 2:
                left_motor(base_power, REVERSE);
                right_motor(base_power, REVERSE);
                cout << "All Good" << endl; 
                break;
                
            /* 0 1 1 - Reversing Slightly */
            case 3:
                
                /* Junction Test */
                if(((robot.line) & 0x08) == 0x08) {               
                    left_motor((base_power + delta_p), REVERSE);
                    right_motor((base_power - delta_p), REVERSE);
                    cout << "Slightly Left" << endl;
                } else {
                    left_motor((base_power - delta_p), REVERSE);
                    right_motor((base_power + delta_p), REVERSE);
                    cout << "Slightly Right" << endl;
                }
                
                break;
                
            /* 1 0 0 - Reversing Serverly */
            case 4:
                
                /* Junction Test */
                if(((robot.line) & 0x08) == 0x08) {               
                    left_motor((base_power + (2*delta_p)), REVERSE);
                    right_motor((base_power - (2*delta_p)), REVERSE);
                    cout << "Serverely Right" << endl;
                } else {
                    left_motor((base_power - (2*delta_p)), REVERSE);
                    right_motor((base_power + (2*delta_p)), REVERSE);
                    cout << "Serverely Left" << endl;
                }
                
                break;
   
            /* 1 0 1 - What the actual */
            case 5:
                cout << "I am a confused robot" << endl;
                stop(); 
                break;

            /* 1 1 0 - Reversing Slightly */
            case 6:
               
               /* Junction Test */
                if(((robot.line) & 0x08) == 0x08) {               
                    left_motor((base_power + delta_p), REVERSE);
                    right_motor((base_power - delta_p), REVERSE);
                    cout << "Serverely Right" << endl;
                } else {
                    left_motor((base_power - delta_p), REVERSE);
                    right_motor((base_power + delta_p), REVERSE);
                    cout << "Serverely Left" << endl;
                }
               
                break;
            
            /* 1 1 1 - Continue */
            case 7:
                cout << "Continuing" << endl;
                stop();              
                break;        
	    }      
        
    }
}
