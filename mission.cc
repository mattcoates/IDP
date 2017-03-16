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
    STATE_INIT = 0, STATE_TEST, STATE_END, STATE_MISSION_LOAD, STATE_BEGIN_DELIVER, STATE_UNLOAD, STATE_HOME, STATE_RAMP, NUM_STATES
} state_t;

typedef state_t state_func_t(void);

/* Function Prototypes */
state_t run_state(state_t cur_state);
static state_t do_state_init(void);
static state_t do_state_test(void);
static state_t do_state_end(void);
static state_t do_state_mission_load(void);
static state_t do_state_begin_deliver(void);
static state_t do_state_unload(void);
static state_t do_state_home(void);
static state_t do_state_ramp(void);

/* State Function Look Up Table */
state_func_t* const state_table[NUM_STATES] = {
    do_state_init, do_state_test, do_state_end, do_state_mission_load, do_state_begin_deliver, do_state_unload, do_state_home, do_state_ramp
};

/* Run State Function */
state_t run_state(state_t cur_state) {
    return state_table[cur_state]();
};



/** STATE FUNCTIONS **/

/* 0. State Init - Setup Robot */
static state_t do_state_init() {

    /* Link Init */   
    #ifdef __arm__
   if (!rlink.initialise ("127.0.0.1")) {
    #else
   if (!rlink.initialise (ROBOT_NUM)) {
    #endif                   
        cout << "Cannot initialise link" << endl;
        rlink.print_errs("    ");
    } 
    
    /* Motor Init */
    motor_init();

    /* Navigation Init */
    robot.location = START_BOX;
    robot.heading = EAST;
    
    /* Pallet Init */
    robot.pallet_colour = 0;
    robot.forklift_position = VERY_BOTTOM;
    robot.delivered_d2 = 0;
	robot.delivered_d1 = 0;
	robot.delivered = 0;
    clear_leds();
    robot.reset = false;

    /* Select Mode */
    cout << "Atlantis AGV Online" << endl << endl;
    cout << "Select Mode:" << endl << "m = Commence Mission" << endl << "t = Enter Test Mode" << endl;
        
    char a;    
    cin >> a;

    if(a == 'm') {
        
        cout << "Select Checkpoint:" << endl;
        cout << "a - [Start] - Startbox facing East with forks at Bottom" << endl;
        cout << "b - [Pre-Ramp] - Startbox facing East with forks at Bottom" << endl;
        
        char b;    
        cin >> b;
        
        if(b == 'a') {
            return STATE_MISSION_LOAD;    
        } else if(b == 'b') {
            robot.reset = true;
            robot.forklift_position = VERY_BOTTOM;
            return STATE_RAMP;
        }
       else {    
            cout << "Invalid Selection!" << endl;
            return STATE_END;
        }
    }
    else if (a == 't') {
        return STATE_TEST;
    }
    else {    
        cout << "Invalid Selection!" << endl;
        return STATE_END;       
    }
}



/* 1. State Two - Test Console */
static state_t do_state_test() {
    
     cout << endl << "Atlantis - Testing Console v2.0" << endl;
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
                signal_pallet_type(pallet_detect()); 
                break; 
                
            case 'q':
                return STATE_END;
                break;         
            
            case 'h':
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
                cout << "q = Quit Testing" << endl;
                break;
        }
    }
}



/* 2. State Three - End */
static state_t do_state_end() {
    stop();
    cout << "Task Complete!" << endl;
    return STATE_END;
}

/* 3. State Four - Begin Mission */
static state_t do_state_mission_load() {

    /* Init Forklift */
    move_fork(robot.forklift_position, JUST_BELOW);
    /* Move to P1 */
    travel(robot.location, P1, SOUTH);
    
    /** PICK UP PALLET ONE **/
    
    /* Raise Forklift */
    move_fork(robot.forklift_position, JUST_ABOVE);
    /* Detect Pallet */
    signal_pallet_type(pallet_detect());    
    /* Move to C1 */
    travel(robot.location, C1, EAST);
    /* Lower Forklift */
    move_fork(robot.forklift_position, JUST_BELOW);
    
    /** PICK UP PALLET TWO **/
    
    /* Signal New Load */
    signal_new_load();  
    /* Move to P1 */
    travel(robot.location, P1, SOUTH);  
    /* Raise Forklift */
    move_fork(robot.forklift_position, JUST_ABOVE);
    /* Detect Pallet */
    signal_pallet_type(pallet_detect()); 
    /* Move to C1 */
    travel(robot.location, C1, EAST);
    /* Lower Forklift */
    move_fork(robot.forklift_position, JUST_BELOW);
    
    /** PICK UP PALLET THREE **/
    
    /* Signal New Load */
    signal_new_load();  
    /* Move to P1 */
    travel(robot.location, P1, SOUTH);  
    /* Raise Forklift */
    move_fork(robot.forklift_position, JUST_ABOVE);
    /* Detect Pallet */
    signal_pallet_type(pallet_detect()); 
    /* Move to C1 */
    travel(robot.location, C1, EAST);
    /* Lower Forklift */
    move_fork(robot.forklift_position, JUST_BELOW);
    
    /** PICK UP PALLET FOUR **/
    
    /* Move to P2 */
    travel(C1, P2, SOUTH);
    /* Raise Forklift */
    move_fork(robot.forklift_position, JUST_ABOVE);
    /* Detect Pallet */
    signal_pallet_type(pallet_detect());
    
    /* Move to START BOX from P2 */
    travel(robot.location, START_BOX, NORTH);
    
    return STATE_RAMP;

}

/* 7. State Eight - Ramp Mode! */
static state_t do_state_ramp(void) {
    
    /* Go Up Ramp */
    travel(robot.location, A, EAST);
    
    return STATE_BEGIN_DELIVER;
    
}

/* 4. State Five - Begin Delivery */
static state_t do_state_begin_deliver(void) {

    if(robot.reset == false) {
    
        move_fork(robot.forklift_position, JUST_ABOVE);
        
        if(robot.pallet_colour == RED) {
            
            /* Deliver to D1 */
            travel(robot.location, D1, NORTH);
            move_fork(robot.forklift_position, JUST_BELOW);
            robot.delivered_d1 = 1;
        
        }
        
        else if((robot.pallet_colour == GREEN) || (robot.pallet_colour == WHITE)) {
        
            /* Deliver to D2 */
            travel(robot.location, D2, SOUTH);
            move_fork(robot.forklift_position, JUST_BELOW);
            robot.delivered_d2 = 1;
        
        }
        
        else {
        
            /* Deliver to D3 */
            travel(robot.location, D3, SOUTH);
            move_fork(robot.forklift_position, VERY_BOTTOM);
            travel(robot.location, A, EAST);
            move_fork(robot.forklift_position, JUST_BELOW);
        
        }
    }
    
    /* Update Delivery Record */
    robot.delivered = 1;
    
    move_fork(robot.forklift_position, JUST_BELOW);
    
    /* Move to C2 */
    travel(A, C2, EAST);
    
    return STATE_UNLOAD;
    
}

/* 5. State Six - Unload Conveyor */
static state_t do_state_unload(void) {

    while(robot.delivered < 4) {
    
        /* Lift Forklift */
        move_fork(robot.forklift_position, STACKING);
        
        /* Detect Pallet */
        signal_pallet_type(pallet_detect());
        
        /* Deliver to Correct Location */
        if(robot.pallet_colour == RED) {
        
            /* Deliver D1 */
            travel(robot.location, D1, NORTH);
            
            /* Lower Fork */
            if(robot.delivered_d1 == 0) {
                move_fork(robot.forklift_position, JUST_BELOW);
                robot.delivered_d1 += 1;
            } else if(robot.delivered_d1 == 1) {
                move_fork(robot.forklift_position, JUST_ABOVE);
            }
        
        } 
        else if((robot.pallet_colour == GREEN) || (robot.pallet_colour == WHITE)) {
        
            /* Deliver D2 */
            travel(robot.location, D2, SOUTH);
            
            /* Lower Fork */
            if(robot.delivered_d2 == 0) {
                move_fork(robot.forklift_position, JUST_BELOW);
                robot.delivered_d2 += 1;
            } else if(robot.delivered_d2 == 1) {
                move_fork(robot.forklift_position, JUST_ABOVE);
            }
        
        }
        else {
        
            /* Deliver D3 */
            travel(robot.location, D3, SOUTH);
            move_fork(robot.forklift_position, VERY_BOTTOM);
            travel(robot.location, A, EAST);
            move_fork(robot.forklift_position, JUST_BELOW);   
        }
        
        
        /* Update Delivery Record */
        robot.delivered += 1;
        
        if(robot.delivered != 4) {
        
            /* Move to C2 */
            travel(robot.location, C2, EAST); 
        }
    }
    
    return STATE_HOME;
    
}


/* 6. State Seven - Go Home */
static state_t do_state_home(void) {

    /* Head to Finish Box */
    travel(robot.location, FINISH_BOX, WEST); 
    
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
