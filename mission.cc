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

typedef enum {
    STATE_INIT = 0, STATE_TEST, STATE_END, NUM_STATES
} state_t;

typedef state_t state_func_t(void);

/* Function Prototypes */
state_t run_state(state_t cur_state);
static state_t do_state_init(void);
static state_t do_state_test(void);
static state_t do_state_end(void);

/* State Function Look Up Table */
state_func_t* const state_table[NUM_STATES] = {
    do_state_init, do_state_test, do_state_end
};

/* Run State Function */
state_t run_state(state_t cur_state) {
    return state_table[cur_state]();
};



/** STATE FUNCTIONS **/

/* 0. State Init */
static state_t do_state_init() {

    /* Link Init */                      
    if (!rlink.initialise(ROBOT_NUM)) {
        cout << "Cannot initialise link" << endl;
        rlink.print_errs("    ");
        /* TODO: Flush & Retry */
    } 
    
    /* Motor Init */
    motor_init();

    /* Navigation Init */
    robot.location = START_BOX;
    robot.heading = EAST;
    
    /* Pallet Init */
    robot.carrying_pallet = false;

    return STATE_TEST;
}



/* 1. State Two */
static state_t do_state_test() {
    
     cout << "Team Challanger - Testing Console v1.0" << endl;
     cout << "[Press h for help]" << endl;
    
    while(true) {
    
        cout << endl << "Enter Command: " << endl;
        
        char a;        
        cin >> a;
        
        switch(a) {
        
            case 'n':
                next_junction();
                break;
            
            case 't':
                turn_90_clockwise();
                break;
            
            case 's':
                turn_90_anti_clockwise();
                break;
                
            case 'l':
                next_limit();
                break;
                
            case 'y':
                back_up_from_limit(D1);
                break;
                
            case 'u':
                back_up_from_limit(D3);
                break;
                
            case 'r':
                traverse_ramp(UPWARDS);
                break;
                
            case 'f':
                lift(UP);
                break;
                
            case 'g':
                lift(DOWN);
                break;
                
            case 'd':
                cout << pallet_detect() << endl;
                break;
            
            case 'a':
                next_junction();
                next_junction();
                turn_90_anti_clockwise();
                next_junction();           
                break;
                
            case 'b':
                next_junction();
                next_junction();
                next_junction();     
                turn_90_clockwise();
                next_limit();                      
                break;
                
            case 'c':
                next_junction();
                traverse_ramp(UPWARDS);
                turn_90_clockwise();
                next_junction();
                next_junction();
                next_junction();
                next_junction();
                next_limit();                   
                break;            
            
            case 'h':
                cout << "a = Test 1 - Follow, Turn, Follow" << endl;
                cout << "b = Test 2 - Follow, Align, Stop" << endl;
                cout << "c = Test 3 - Follow, Ramp, Follow, Align, Stop" << endl << endl;
                cout << "n = Next Junction" << endl;
                cout << "t = Turn Clockwise" << endl;
                cout << "s = Turn Anticlockwise" << endl;
                cout << "r = Traverse Ramp" << endl;
                cout << "l = Next Limit" << endl;
                cout << "y = Backup from Limit at D1 or D2" << endl;
                cout << "u = Backup from Limit anywhere else" << endl;
                cout << "f = Forklift Up" << endl;
                cout << "g = Forklift Down" << endl;
                cout << "d = Detect Pallet Type [ 1-R 2-G 3-B 4-W ]" << endl;
                break;
        }
    }
    
    return STATE_END;
}



/* 2. State Three */
static state_t do_state_end() {
    stop();
    return STATE_END;
}

/** END STATE FUNCTIONS **/



/* State Machine Controller */
void mission_begin(void) {
    
    state_t cur_state = STATE_INIT;
    state_t new_state;

    while(true) {

        /* Run state machine current state function */
        new_state = run_state(cur_state);

        if(new_state != cur_state) {
      
            /* Swap to the new state */
            cur_state = new_state;
            cout << "Entering State " << cur_state << endl;
        }

        /* Tick the state machine */
        delay(2);
    }
}
