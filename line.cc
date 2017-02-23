#include <iostream>
#include <robot_instr.h>
#include <robot_link.h>
#include <stopwatch.h>
#include <robot_delay.h>
#include "robot.h"
#include "line.h"
#include "motor.h"
using namespace std;


void next_junction(void) {

    right_motor(127, FORWARD);
    left_motor(127, FORWARD);
}
