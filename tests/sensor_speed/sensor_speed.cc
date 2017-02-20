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

    cout << "Sensor Testing" << endl;
    
    /* Setup Link */
    if (!rlink.initialise (ROBOT_NUM)) {
        cout << "Cannot initialise link" << endl;
        rlink.print_errs("    ");
        return -1;
    }   
    
    /* Test Ping Time */
    int val = 0;
    watch.start();
    for(int i=0; i<1000; i++) {
        val = rlink.request (TEST_INSTRUCTION);
        if (val == REQUEST_ERROR) {
            cout << "Fatal Error: " << endl;
            rlink.print_errs();
            return -1;
       }
    }
    double ping_time = (watch.stop()/1000);
    
    /* Write Byte Time */
    watch.start();
    for(int j=0; j<1000; j++) {
       rlink.command(WRITE_PORT_3, 0xAA);     
    }
    double write_time = (watch.stop()/1000);
    
    /* Read Byte Time */
    int v = 0;
    watch.start();
    for(int k=0; k<1000; k++) {
       v = rlink.request (READ_PORT_3);    
    }
    double read_time = (watch.stop()/1000);
    
    /* Read ADC Time */
    int a = 0;
    watch.start();
    for(int l=0; l<1000; l++) {
       a = rlink.request (ADC3);   
    }
    double read_adc_time = (watch.stop()/1000);
    
    
    /* Output Results */   
    cout << "Ping Time: " << ping_time << "ms" << endl;
    cout << "Write Time: " << write_time << "ms" << endl;
    cout << "Read Time: " << read_time << "ms" << endl;
    cout << "ADC Time: " << read_adc_time << "ms" << endl;
}
