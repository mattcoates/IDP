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
const int chain_dog = 3500;


/* Pallet Deetection */
int pallet_detect(void) {

    /* Housekeeping */
    bool detected = false;
    int attempts = 0;
    
    int pallet_colour = 0;

    while(!(detected) && (attempts < 6)) {

        /* Increment Attempts */
        attempts += 1;

        /* Let LDR Stabalise */
        delay(1000);
        
        /* Read and Decide - Attempt 1 */
        int reading1 = rlink.request(ADC4);
        
        cout << "Reading 1 = " << reading1 << endl; 
        
        
        if((reading1 > RED_LOW) && (reading1 < RED_HIGH)) {
        
            reading1 = RED;
        
        } 
        
        else if((reading1 > GREEN_LOW) && (reading1 < GREEN_HIGH)) {
        
            reading1 = GREEN;
        
        }
        
        else if((reading1 > BLACK_LOW) && (reading1 < BLACK_HIGH)) {
        
            reading1 = BLACK;
        
        }
        
        else if((reading1 > WHITE_LOW) && (reading1 < WHITE_HIGH)) {
        
            reading1 = WHITE;
        
        }
        
        else {
        
            reading1 = 0;
        
        }
        
        /* Let LDR Stabalise */
        delay(1000);
        
        /* Read and Decide - Attempt 2 */
        int reading2 = rlink.request(ADC4);    
        
        cout << "Reading 2 = " << reading2 << endl;
        
        
        if((reading2 > RED_LOW) && (reading2 < RED_HIGH)) {
        
            reading2 = RED;
        
        } 
        
        else if((reading2 > GREEN_LOW) && (reading2 < GREEN_HIGH)) {
        
            reading2 = GREEN;
        
        }
        
        else if((reading2 > BLACK_LOW) && (reading2 < BLACK_HIGH)) {
        
            reading2 = BLACK;
        
        }
        
        else if((reading2 > WHITE_LOW) && (reading2 < WHITE_HIGH)) {
        
            reading2 = WHITE;
        
        }
        
        else {
        
            reading2 = 5;
        
        }
        
        
        if (reading1 == reading2) {
        
             detected = true;
             pallet_colour = reading1;          
            
        }
    }
    robot.pallet_colour = pallet_colour;
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
        
        while((((robot.line & 0x80) == 0x00) || (chain_watch.read() < chain_grace)) && (chain_watch.read() < chain_dog)) {
        
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
        
        while((((robot.line & 0x80) == 0x00) || (chain_watch.read() < chain_grace)) && (chain_watch.read() < chain_dog)) {
        
            rlink.command(MOTOR_3_GO, output);
            read_line_sensors();        
        }
        
        chain_watch.stop();
        stop();    
    }
}


void move_fork(int current_pos, int des_pos) {

    robot.forklift_position = des_pos;

    switch (current_pos) {
        
        case VERY_BOTTOM:
        
            if(des_pos == JUST_BELOW) {
                lift(UP);        
            } 
            else if(des_pos == JUST_ABOVE) {
                lift(UP);
                lift(UP);
            }
            else if(des_pos == STACKING){
                lift(UP);
                lift(UP);
                lift(UP);
            }
            
        break;
        
        case JUST_BELOW:
        
            if(des_pos == VERY_BOTTOM) {
                lift(DOWN);        
            } 
            else if(des_pos == JUST_ABOVE) {
                lift(UP);
            }
            else if(des_pos == STACKING){
                lift(UP);
                lift(UP);
            }
        
        break;

        case JUST_ABOVE:
        
            if(des_pos == VERY_BOTTOM) {
                lift(DOWN);
                lift(DOWN);     
            } 
            else if(des_pos == JUST_BELOW) {
                lift(DOWN);
            }
            else if(des_pos == STACKING){
                lift(UP);
            }
        
        break;
    
        case STACKING:
        
            if(des_pos == VERY_BOTTOM) {
                lift(DOWN);
                lift(DOWN); 
                lift(DOWN);    
            } 
            else if(des_pos == JUST_BELOW) {
                lift(DOWN);
                lift(DOWN);
            }
            else if(des_pos == JUST_ABOVE){
                lift(DOWN);
            }
        
        break;    
    
    }
}

