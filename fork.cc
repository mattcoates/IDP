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
#include "fork.h"

using namespace std;

stopwatch chain_watch;

/* Chain Timings */
const int chain_grace = 800;


/* Pallet Deetection */
int pallet_detect(void) {

    /* Housekeeping */
    bool detected = false;
    int attempts = 0;
    
    int pallet_colour = 0;

    while(!(detected) || (attempts < 4)) {

        /* Increment Attempts */
        attempts += 1;

        /* Let LDR Stabalise */
        delay(1000);
        
        /* Read and Decide - Attempt 1 */
        int reading1 = rlink.request(ADC4);    
        
        
        if((robot.pallet_colour > RED_LOW) && (robot.pallet_colour < RED_HIGH)) {
        
            reading1 = RED;
        
        } 
        
        else if((robot.pallet_colour > GREEN_LOW) && (robot.pallet_colour < GREEN_HIGH)) {
        
            reading1 = GREEN;
        
        }
        
        else if((robot.pallet_colour > BLACK_LOW) && (robot.pallet_colour < BLACK_HIGH)) {
        
            reading1 = BLACK;
        
        }
        
        else if((robot.pallet_colour > WHITE_LOW) && (robot.pallet_colour < WHITE_HIGH)) {
        
            reading1 = WHITE;
        
        }
        
        else {
        
            reading1 = 0;
        
        }
        
        /* Let LDR Stabalise */
        delay(1000);
        
        /* Read and Decide - Attempt 2 */
        int reading2 = rlink.request(ADC4);    
        
        
        if((robot.pallet_colour > RED_LOW) && (robot.pallet_colour < RED_HIGH)) {
        
            reading2 = RED;
        
        } 
        
        else if((robot.pallet_colour > GREEN_LOW) && (robot.pallet_colour < GREEN_HIGH)) {
        
            reading2 = GREEN;
        
        }
        
        else if((robot.pallet_colour > BLACK_LOW) && (robot.pallet_colour < BLACK_HIGH)) {
        
            reading2 = BLACK;
        
        }
        
        else if((robot.pallet_colour > WHITE_LOW) && (robot.pallet_colour < WHITE_HIGH)) {
        
            reading2 = WHITE;
        
        }
        
        else {
        
            reading2 = 0;
        
        }
        
        
        if (reading1 == reading2) {
        
             detected = true;
             pallet_colour = reading1;          
            
        }
    }
    
    return pallet_colour;
}

/* Forklift Movement */
void lift(int direction) {

    int output = 0;
    
    if(direction) {
        
        /* Lift Up Until Tape */
        output = LIFT_SPEED_UP + 128;
        
        read_line_sensors();
        chain_watch.start();
        
        while(((robot.line & 0x80) == 0x00) || (chain_watch.read() < chain_grace)) {
        
            rlink.command(MOTOR_3_GO, output);
            read_line_sensors();        
        }
        
        chain_watch.stop();
        stop();
        
    } else {
        
        /* Lift Down Until Tape */
        output = LIFT_SPEED_DOWN;
        
        read_line_sensors();
        chain_watch.start();
        
        while(((robot.line & 0x80) == 0x00) || (chain_watch.read() < chain_grace)) {
        
            rlink.command(MOTOR_3_GO, output);
            read_line_sensors();        
        }
        
        chain_watch.stop();
        stop();    
    }
}

