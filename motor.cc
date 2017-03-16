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


void motor_init(void){
    
    rlink.command(RAMP_TIME, RAMP_RATE);
}


void right_motor(int speed, int direction){ 

    int output = 0;
    
    if(direction) {
        output = (0x7F & speed);
    } else {
        output = (0x7F & speed) + 128;    
    }
    
    rlink.command(MOTOR_2_GO, output);
    
    /* DEBUG */
    //cout << "  R =  " << speed << " (" << output << ")" << endl;
}


void left_motor(int speed, int direction){

    int output = 0;
    
    if(direction) {
        output = (0x7F & speed) + 128;
    } else {
        output = (0x7F & speed);    
    }
    
    rlink.command(MOTOR_1_GO, output); 
    
    /* DEBUG */
    //cout << "  L =  " << speed << " (" << output << ")" ;
}


void stop(void){
    rlink.command(BOTH_MOTORS_GO_SAME, 0);
    rlink.command(MOTOR_3_GO, 0);
}

