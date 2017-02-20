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
     rlink.command(WRITE_PORT_5, 64+16+4+1);
}
