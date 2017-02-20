#include <iostream>
#include <robot_instr.h>
#include <robot_link.h>
#include <stopwatch.h>
using namespace std;

#define ROBOT_NUM  15
robot_link rlink;                           
stopwatch watch;

/* Entry Point */
int main () {

    /* Setup Link */
    int val;                          
    if (!rlink.initialise (ROBOT_NUM)) {
    cout << "Cannot initialise link" << endl;
    rlink.print_errs("    ");
    return -1;
    }
    
    /* Test response time */
    watch.start();
    for(int i=0; i<100; i++){
        val = rlink.request (TEST_INSTRUCTION);   // send test instruction
        if (val == REQUEST_ERROR) {
            cout << "Fatal Error: " << endl;
            rlink.print_errs();
            return -1;
       }
    }
    cout << "Time for 1 transmission: " << (watch.stop()/100) << "ms" << endl;
    
    return 0;
}
