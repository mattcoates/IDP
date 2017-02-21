#include <iostream>
#include <robot_instr.h>
#include <robot_link.h>
#include <stopwatch.h>
#include <robot_delay.h>
using namespace std;

#define ROBOT_NUM  15
robot_link rlink;                           
stopwatch watch;

/* Entry Point */
int main () {

    /* Setup Link */                        
    if (!rlink.initialise (ROBOT_NUM)) {
        cout << "Cannot initialise link" << endl;
        rlink.print_errs("    ");
        return -1;
    }
    
    rlink.command (RAMP_TIME, 128);

    rlink.command(MOTOR_1_GO, 100);
    rlink.command(MOTOR_2_GO, 100);
    rlink.command(MOTOR_3_GO, 100);
    rlink.command(MOTOR_4_GO, 100);

    delay(8000);
    
    return 0;
}
