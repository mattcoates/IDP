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


/* 
 * Line Sensor Address:
 * A0 = 1, A1 = 1, A2 = 0 -> PORT 0x03
 *
 * Line Sensor Inputs:
 * b0 = Front Right, b1 = Front Centre
 * b2 = Front Left, b3 = Junction Detect
 *
 * Limit Switches Address:
 * A0 = 0, A1 = 1, A2 = 0 -> PORT 0x02
 *
 * Limit Switch Inputs:
 * b4 = Front Right, b5 = Front Left
 */
 

void read_line_sensors(void) {

    robot.line = rlink.request (READ_PORT_3);
}

void read_limit_switches(void) {

    robot.limit = rlink.request (READ_PORT_2);
}
