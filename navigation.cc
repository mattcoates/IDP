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

   /* 
    * TODO: Go Places, Do Things ect... 
    *  - next_junction(void);
    *  - next_limit(void);
    *  - back_up_from_limit(void);
    *  - change_heading(int old_heading, int new_heading); 
    */


    switch(current_location) {
    
        /* Starting from P1 */
        case P1:
        
            if(destination == C1) {
                
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
        
            if(destination == A) {
                
                /* Pull Away from Truck */
                back_up_from_limit(robot.location);
                
                /* Pull upto Ramp */
                change_heading(robot.heading, WEST);
                next_junction();
                next_junction();
                change_heading(robot.heading, NORTH);
                next_junction();
                move_fork(robot.forklift_position, VERY_BOTTOM);
                
                /* Go Up Ramp */
                traverse_ramp(UPWARDS);
                move_fork(robot.forklift_position, JUST_ABOVE);
                
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
            
    
        /* Starting from C1 */
        case C1:
        
            if(destination == P1) {
                
                /* Move to P1 */
                back_up_from_limit(robot.location);
                change_heading(robot.heading, SOUTH);
                next_limit();
                
                /* Update Location */
                robot.location = P1;             
            }
            
            if(destination == P2) {
                
                /* Move to P2 */
                back_up_from_limit(robot.location);
                change_heading(robot.heading, WEST);
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
            
                /* Move to D3 */
                change_heading(robot.heading, SOUTH);
                next_junction();
                next_limit();
                
                /* Update Location */
                robot.location = D3;             
            } 
              
            break;
        
        /* Starting from C2 */  
        case C2:
        
            break; 
               
               
        /* Starting from D1 */  
        case D1:   
     
            if(destination == C2) {
                
                /* Move to C2 */
                back_up_from_limit(robot.location);
                change_heading(robot.heading, EAST);
                next_limit();
                
                /* Update Location */
                robot.location = C2;
            }
        
            break; 
              
              
        /* Starting from D2 */   
        case D2:
        
            if(destination == C2) {
                
                /* Move to C2 */
                back_up_from_limit(robot.location);
                change_heading(robot.heading, EAST);
                next_limit();
                
                /* Update Location */
                robot.location = C2;
            }
        
            break;  
             
             
        /* Starting from D3 */  
        case D3:
        
            if(destination == A) {
            
                /* Move to A */
                back_up_from_limit(robot.location);
                back_up_from_limit(robot.location);
                change_heading(robot.heading, desired_heading);                
                
                /* Update Location */
                robot.location = A;             
            }
            
            if(destination == C2) {
                
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
             
            break;    
    }


}

