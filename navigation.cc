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

void travel(int current_location, int current_heading, int destination, int desired_heading) {

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
        
            if(destination == P2) {
                
                back_up_from_limit();
                robot.location = 1;
                robot.heading = SOUTH;
                
                change_heading(SOUTH, WEST);
                
                next_junction();
                robot.location = 3;
                
                change_heading(WEST, SOUTH);
                
                next_limit();
                robot.location = P2;
                
            }
            
            if(destination == C1) {
    
            }
            
            if(destination == C2) {
            
            }
            
            if(destination == D1) {
            
            }
            
            if(destination == D2) {
            
            }
            
            if(destination == D3) {
            
            }
            
            if(destination == FINISH_BOX) {
            
            }
        
            break;
            
        
        /* Starting from P2 */   
        case P2:
        
            break;
            
    
        /* Starting from C1 */
        case C1:
        
            break; 
              
        
        /* Starting from C2 */  
        case C2:
        
            break; 
               
               
        /* Starting from D1 */  
        case D1:
        
            break; 
              
              
        /* Starting from D2 */   
        case D2:
        
            break;  
             
             
        /* Starting from D3 */  
        case D3:
        
            break;
            
            
        /* Starting from START */    
        case START_BOX:
        
            break;
    
    }


}

