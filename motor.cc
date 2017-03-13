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

stopwatch chain_watch;

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
    cout << "  R =  " << speed << " (" << output << ")" << endl;
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
    cout << "  L =  " << speed << " (" << output << ")" ;
}


void lift(int direction) {

    int output = 0;
    
    if(direction) {
        
        /* Lift Until Tape */
        output = LIFT_SPEED + 128;
        
        read_line_sensors();
        chain_watch.start();
        
        while(((robot.line & 0x80) == 0x00) || (chain_watch.read() < 1000)) {
        
            rlink.command(MOTOR_3_GO, output);
            read_line_sensors();        
        }
        
        chain_watch.stop();
        stop();
        
    } else {
        
        /* Lift Until Tape */
        output = LIFT_SPEED;
        
        read_line_sensors();
        chain_watch.start();
        
        while(((robot.line & 0x80) == 0x00) || (chain_watch.read() < 1000)) {
        
            rlink.command(MOTOR_3_GO, output);
            read_line_sensors();        
        }
        
        chain_watch.stop();
        stop();    
    }
}


void stop(void){
    rlink.command(BOTH_MOTORS_GO_SAME, 0);
    rlink.command(MOTOR_3_GO, 0);
}

