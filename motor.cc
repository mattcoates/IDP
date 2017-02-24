#include <iostream>
#include <robot_instr.h>
#include <robot_link.h>
#include <stopwatch.h>
#include <robot_delay.h>
#include "robot.h"
#include "line.h"
#include "motor.h"
#include "sensors.h"
using namespace std;

void motor_init(void){
    
    rlink.command(RAMP_TIME, RAMP_RATE);
}

void right_motor(int speed, int direction){ 

    int output = (~(direction << 7) | (0x7F & speed));
    rlink.command(MOTOR_2_GO, output);
}

void left_motor(int speed, int direction){

    int output = ((direction << 7) | (0x7F & speed));
    rlink.command(MOTOR_1_GO, output);
}

void stop(void){
    rlink.command(BOTH_MOTORS_GO_SAME, 0);
}

