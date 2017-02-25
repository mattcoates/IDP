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

bool robot_init(void);

int main() {

    /* Init Robot */
    if (robot_init()) {
        cout << "Robot Initialised" << endl;
    }
    
    /* Commence Mission */
    mission_begin();
    
}



bool robot_init(void) {

    /* Link Innit */                      
    if (!rlink.initialise(ROBOT_NUM)) {
        cout << "Cannot initialise link" << endl;
        rlink.print_errs("    ");
        return false;
    } 
    
    /* Motor Init */
    motor_init();

    /* Navigation Init */
    robot.location = START_BOX;
    robot.heading = EAST;
    
    /* TODO: Pallet Init */
    robot.carrying_pallet = false;
    
    return true;
}
