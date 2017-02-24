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

/* Robot Classes */
robot_link rlink;
robot_data robot;


int main() {

    /* Setup Link */                      
    if (!rlink.initialise(ROBOT_NUM)) {
        cout << "Cannot initialise link" << endl;
        rlink.print_errs("    ");
        return -1;
    }    
    
    right_motor(127, REVERSE);
    left_motor(127, REVERSE);
    delay(2000);
}
