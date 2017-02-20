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
    
    /* Command I2C */
    int out = 0xAA; 
    while(true) {
        rlink.command(WRITE_PORT_3, out);
        delay (200);
        if(out == 0xAA){
            out = 0x55;
        } else {
            out = 0xAA;
        }
    }
}
