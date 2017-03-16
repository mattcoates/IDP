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


/* 
 * Line Sensor Address:
 * A0 = 1, A1 = 1, A2 = 0 -> PORT 0x03
 *
 * Line Sensor Inputs:
 * b0 = Front Right, b1 = Front Centre
 * b2 = Front Left, b3 = Junction Detect
 * b7 = Chain Detect
 *
 * Limit Switches/LEDs Address:
 * A0 = 0, A1 = 1, A2 = 0 -> PORT 0x02
 *
 * Limit Switch/LEDs I/Os:
 * b0 to b4 = LEDs
 * b6 = Front Right, b5 = Front Left
 */
 

/* Signal Pallet Type */ 
void signal_pallet_type(int colour) {

    if(colour == RED) {
        rlink.command(WRITE_PORT_2, 0xF7);
    }
    else if(colour == WHITE) {
        rlink.command(WRITE_PORT_2, 0xFB);
    }
    else if(colour == GREEN) {
        rlink.command(WRITE_PORT_2, 0xFD);
    }
    else if(colour == BLACK) {
        rlink.command(WRITE_PORT_2, 0xFE);
    }
    else {
        rlink.command(WRITE_PORT_2, 0xF0);
    }
}

void clear_leds(void) {

    rlink.command(WRITE_PORT_2, 0xFF);

}


/* Signal New Load */ 
void signal_new_load(void) {

    rlink.command(WRITE_PORT_2, 0xEF);
    delay(4000);
    rlink.command(WRITE_PORT_2, 0xFF);
    delay(6000);

}


/* Read Inputs */ 
void read_line_sensors(void) {

    robot.line = rlink.request (READ_PORT_3);
}

void read_limit_switches(void) {

    robot.limit = rlink.request (READ_PORT_2);
}
