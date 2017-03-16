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
#include "fork.h"

using namespace std;

void travel(int current_location, int destination, int desired_heading) {

    switch(current_location) {
    
        /* Starting from P1 */
        case P1:
        
            if(destination == C1) {
            
                cout << "Travelling from P1 to C1" << endl;
                
                /* Move to C1 */
                back_up_from_limit(robot.location);
                change_heading(robot.heading, EAST);
                next_limit();
                
                /* Update Location */
                robot.location = C1;             
            }   
        
            break;
        
        /* Starting from P2 */   
        case P2:
        
            if(destination == START_BOX) {
            
                cout << "Travelling from P2 to Start Box" << endl;
                
                /* Pull Away from Truck */
                back_up_from_limit(robot.location);
                
                /* Pull upto Ramp */
                change_heading(robot.heading, WEST);
                next_junction();
                next_junction();
                change_heading(robot.heading, NORTH);
                
                /* Update Location */
                robot.location = START_BOX;             
            }   
        
            break;
            
    
        /* Starting from C1 */
        case C1:
        
            if(destination == P1) {
            
                cout << "Travelling from C1 to P1" << endl;
                
                /* Move to P1 */
                back_up_from_limit(robot.location);
                change_heading(robot.heading, SOUTH);
                next_limit();
                
                /* Update Location */
                robot.location = P1;             
            }
            
            if(destination == P2) {
            
                cout << "Travelling from C1 to P2" << endl;
                
                /* Move to P2 */
                back_up_from_limit(robot.location);
                turn_90_anti_clockwise();
                turn_90_anti_clockwise();
                robot.heading = WEST;
                next_junction();
                change_heading(robot.heading, SOUTH);
                next_limit();
                
                /* Update Location */
                robot.location = P2;             
            } 
        
            break; 
        
        /* Starting from A */    
        case A:
        
            if(destination == D1) {
            
                cout << "Travelling from A to D1" << endl;
            
                /* Move to D1 */
                change_heading(robot.heading, EAST);
                next_junction();
                next_junction();
                change_heading(robot.heading, NORTH);
                next_limit();                
                
                /* Update Location */
                robot.location = D1;             
            } 
            
            if(destination == D2) {
            
                cout << "Travelling from A to D2" << endl;

                /* Move to D2 */
                change_heading(robot.heading, EAST);
                next_junction();
                next_junction();
                change_heading(robot.heading, SOUTH);
                next_limit();
                
                /* Update Location */
                robot.location = D2;             
            } 
            
            if(destination == D3) {
            
                cout << "Travelling from A to D3" << endl;
            
                /* Move to D3 */
                change_heading(robot.heading, SOUTH);
                next_junction();
                next_limit();
                
                /* Update Location */
                robot.location = D3;             
            } 
            
            if(destination == C2) {
            
                cout << "Travelling from A to C2" << endl;
            
                /* Move to C2 */
                change_heading(robot.heading, EAST);
                next_junction();
                next_junction();
                next_limit();
                
                /* Update Location */
                robot.location = C2;
            
            }
            
            if(destination == FINISH_BOX) {
            
            }
              
            break;
        
        /* Starting from C2 */  
        case C2:
        
            if(destination == D1) {
            
                cout << "Travelling from C2 to D1" << endl;
            
                /* Move to D1 */
                back_up_from_limit(robot.location);
                change_heading(robot.heading, NORTH);
                next_limit();           
                
                /* Update Location */
                robot.location = D1;             
            } 
            
            if(destination == D2) {
            
                cout << "Travelling from C2 to D2" << endl;
            
                /* Move to D2 */
                back_up_from_limit(robot.location);
                change_heading(robot.heading, SOUTH);
                next_limit();                
                
                /* Update Location */
                robot.location = D2;             
            } 
            
            if(destination == D3) {
            
                cout << "Travelling from C2 to D3" << endl;
            
                /* Move to D3 */
                back_up_from_limit(robot.location);
                turn_90_anti_clockwise();
                turn_90_anti_clockwise();
                robot.heading = WEST;
                next_junction();
                next_junction();
                change_heading(robot.heading, SOUTH);
                next_junction();
                next_limit();
                
                /* Update Location */
                robot.location = D3;             
            } 
        
            break; 
               
               
        /* Starting from D1 */  
        case D1:   
     
            if(destination == C2) {
            
                cout << "Travelling from D1 to C2" << endl;
                
                /* Move to C2 */
                back_up_from_limit(robot.location);
                change_heading(robot.heading, EAST);
                move_fork(robot.forklift_position, JUST_BELOW);
                next_limit();
                
                /* Update Location */
                robot.location = C2;
            }
            
            if(destination == FINISH_BOX) {
            
                cout << "Travelling from D1 to Finish" << endl;
                
                /* Move to FINISH */
                back_up_from_limit(robot.location);
                change_heading(robot.heading, WEST);
                next_junction();
                next_junction();
                
                /* Update Location */
                robot.location = FINISH_BOX;
            }
        
            break; 
              
              
        /* Starting from D2 */   
        case D2:
        
            if(destination == C2) {
            
                cout << "Travelling from D2 to C2" << endl;
                
                /* Move to C2 */
                back_up_from_limit(robot.location);
                change_heading(robot.heading, EAST);
                move_fork(robot.forklift_position, JUST_BELOW);
                next_limit();
                
                /* Update Location */
                robot.location = C2;
            }
            
            if(destination == FINISH_BOX) {
            
            cout << "Travelling from D2 to Finish" << endl;
                
                /* Move to FINISH */
                back_up_from_limit(robot.location);
                change_heading(robot.heading, WEST);
                next_junction();
                next_junction();
                
                /* Update Location */
                robot.location = FINISH_BOX;
            }
        
            break;  
             
             
        /* Starting from D3 */  
        case D3:
        
            if(destination == A) {
            
                cout << "Travelling from D3 to A" << endl;
            
                /* Move to A */
                back_up_from_limit(robot.location);
                back_up_from_limit(robot.location);
                change_heading(robot.heading, desired_heading);                
                
                /* Update Location */
                robot.location = A;             
            }
            
            else if(destination == FINISH_BOX) {
            
                cout << "Travelling from D3 to Finish" << endl;
            
                /* Move to A */
                back_up_from_limit(robot.location);
                back_up_from_limit(robot.location);
                change_heading(robot.heading, desired_heading);                
                
                /* Update Location */
                robot.location = FINISH_BOX;             
            }
            
            if(destination == C2) {
            
               cout << "Travelling from D3 to C2" << endl;
                
                /* Move to C2 */
                back_up_from_limit(robot.location);
                back_up_from_limit(robot.location);
                change_heading(robot.heading, EAST);
                next_junction();
                next_junction();
                next_limit();
                
                /* Update Location */
                robot.location = C2;
            }
        
            break;
            
            
        /* Starting from START */    
        case START_BOX:
        
            if(destination == P1) {
            
               cout << "Travelling from Start to P1" << endl;
                
                /* Orient for Movement */
                change_heading(robot.heading, EAST);
                
                /* Move to Desired Location */
                next_junction();
                next_junction();
                next_junction();                
                change_heading(robot.heading, SOUTH);
                next_limit();
                
                /* Update Location */
                robot.location = P1;             
            }      
            
            if(destination == A){
            
               cout << "Travelling from Start to A" << endl;
                
                /* Approach Ramp */
                change_heading(robot.heading, NORTH);
                next_junction();
                move_fork(robot.forklift_position, VERY_BOTTOM);
                
                /* Go Up Ramp */
                traverse_ramp(UPWARDS);
                move_fork(robot.forklift_position, JUST_BELOW);
                
                /* Pull upto A */
                change_heading(robot.heading, EAST);
                next_junction();
                next_junction();
                
                /* Change to Desired Heading */
                change_heading(robot.heading, desired_heading); 
                
                /* Update Location */
                robot.location = A;
            }
            
            break;    
    }

}

