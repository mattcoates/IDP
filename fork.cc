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

stopwatch chain_watch;

/* Chain Timings */
const int chain_grace = 800;


/* Pallet Deetection */
int pallet_detect(void) {




}

/* Forklift Movement */
void lift(int direction) {

    int output = 0;
    
    if(direction) {
        
        /* Lift Up Until Tape */
        output = LIFT_SPEED_UP + 128;
        
        read_line_sensors();
        chain_watch.start();
        
        while(((robot.line & 0x80) == 0x00) || (chain_watch.read() < chain_grace)) {
        
            rlink.command(MOTOR_3_GO, output);
            read_line_sensors();        
        }
        
        chain_watch.stop();
        stop();
        
    } else {
        
        /* Lift Down Until Tape */
        output = LIFT_SPEED_DOWN;
        
        read_line_sensors();
        chain_watch.start();
        
        while(((robot.line & 0x80) == 0x00) || (chain_watch.read() < chain_grace)) {
        
            rlink.command(MOTOR_3_GO, output);
            read_line_sensors();        
        }
        
        chain_watch.stop();
        stop();    
    }
}

