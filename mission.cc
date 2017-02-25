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
    STATE_ONE = 0, STATE_TWO, STATE_THREE, NUM_STATES
} state_t;

typedef state_t state_func_t(void);

/* Function Prototypes */
state_t run_state(state_t cur_state);
static state_t do_state_one(void);
static state_t do_state_two(void);
static state_t do_state_three(void);

/* State Function Look Up Table */
state_func_t* const state_table[NUM_STATES] = {
    do_state_one, do_state_two, do_state_three
};

/* Run State Function */
state_t run_state(state_t cur_state) {
    return state_table[cur_state]();
};


/* State One */
static state_t do_state_one() {
    return STATE_ONE;
}


/* State Two */
static state_t do_state_two() {
    return STATE_ONE;
}


/* State Three */
static state_t do_state_three() {
    return STATE_ONE;
}


/* State Machine Handler */
void mission_begin(void) {
    
    state_t cur_state = STATE_ONE;
    state_t new_state;

    while(true) {

        /* Run state machine current state function */
        new_state = run_state(cur_state);

        if(new_state != cur_state) {

            /* Swap to the new state */
            cur_state = new_state;
        }

        /* Tick the state machine */
        delay(2);
    }
}
