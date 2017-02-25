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

typedef enum {
    STATE_INIT = 0, STATE_TWO, STATE_THREE, NUM_STATES
} state_t;

typedef state_t state_func_t(void);

/* Function Prototypes */
state_t run_state(state_t cur_state);
static state_t do_state_init(void);
static state_t do_state_two(void);
static state_t do_state_three(void);

/* State Function Look Up Table */
state_func_t* const state_table[NUM_STATES] = {
    do_state_init, do_state_two, do_state_three
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

    return STATE_TWO;
}



/* 1. State Two */
static state_t do_state_two() {
    
    return STATE_THREE;
}



/* 2. State Three */
static state_t do_state_three() {

    return STATE_THREE;
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
