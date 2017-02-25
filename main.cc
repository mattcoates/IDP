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

/* Robot Classes */
robot_link rlink;
robot_data robot;

int main() {

    /* Commence Mission */
    mission_begin();
    
}
