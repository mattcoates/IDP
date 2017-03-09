const int base_power = 50;
const int delta_p = 10;

const int ramp_base_power = 80;
const int ramp_delta_p = 15;

const int base_power_creep = 25;
const int delta_p_creep = 9;

const int ramp_power_l = 80;
const int ramp_power_r = 85;

void traverse_ramp(int direction) {
        
    ramp_watch.start();    
     
    /* Going up the slide */
    if(direction == UPWARDS) {
    
        cout << "Ramp Approach" << endl;
        
        /* Handle Trust Issues */            
        while(ramp_watch.read() < up_t1) {
                  
            switch((robot.line & 0x07)) {
            
                /* 0 0 0 - Where's the line? */
                case 0:
                    cout << "No line detected" << endl;
                    break;
                
                /* 0 0 1 - Moving Serverly Left */
                case 1:
                    left_motor((base_power + (2*delta_p)), FORWARD);
                    right_motor((base_power - (2*delta_p)), FORWARD);
                    cout << "Serverely Left" << endl;
                    break;
            
                /* 0 1 0 - Following the Line */
                case 2:
                    left_motor(base_power, FORWARD);
                    right_motor(base_power, FORWARD);
                    cout << "All Good" << endl; 
                    break;
                    
                /* 0 1 1 - Moving Slightly Left */
                case 3:
                    left_motor((base_power + delta_p), FORWARD);
                    right_motor((base_power - delta_p), FORWARD);
                    cout << "Slightly Left" << endl;  
                    break;
                    
                /* 1 0 0 - Moving Serverly Right */
                case 4:
                    left_motor((base_power - (2*delta_p)), FORWARD);
                    right_motor((base_power + (2*delta_p)), FORWARD); 
                    cout << "Serverely Right" << endl; 
                    break;

                /* 1 1 0 - Moving Slightly Right */
                case 6:
                    left_motor((base_power - delta_p), FORWARD);
                    right_motor((base_power + delta_p), FORWARD); 
                    cout << "Slightly Right" << endl; 
                    break;
                    
                /* 1 1 1 - Junction Detected */
                case 7:
                  stop();
                  break;         
	        }
        
        }
        
        cout << "Ramp Engaged" << endl;
        
        while(ramp_watch.read() < up_t2) {
        
            /* Move forward blind at 80 */
             left_motor(ramp_power_l, FORWARD);
             right_motor(ramp_power_r, FORWARD);
        
        }
        
        cout << "Ramp Comfortable" << endl;
        
        while(ramp_watch.read() < up_t3) {
        
            /* Follow line at power 80 */             
            switch((robot.line & 0x07)) {
                
                /* 0 0 0 - Where's the line? */
                case 0:
                    cout << "No line detected" << endl;
                    stop(); 
                    break;
                
                /* 0 0 1 - Moving Serverly Left */
                case 1:
                    left_motor((ramp_base_power + (2*ramp_delta_p)), FORWARD);
                    right_motor((ramp_base_power - (2*ramp_delta_p)), FORWARD);
                    cout << "Serverely Left" << endl;
                    break;
            
                /* 0 1 0 - Following the Line */
                case 2:
                    left_motor(ramp_base_power, FORWARD);
                    right_motor(ramp_base_power, FORWARD);
                    cout << "All Good" << endl; 
                    break;
                    
                /* 0 1 1 - Moving Slightly Left */
                case 3:
                    left_motor((ramp_base_power + ramp_delta_p), FORWARD);
                    right_motor((ramp_base_power - ramp_delta_p), FORWARD);
                    cout << "Slightly Left" << endl;  
                    break;
                    
                /* 1 0 0 - Moving Serverly Right */
                case 4:
                    left_motor((ramp_base_power - (2*ramp_delta_p)), FORWARD);
                    right_motor((ramp_base_power + (2*ramp_delta_p)), FORWARD); 
                    cout << "Serverely Right" << endl; 
                    break;
       
                /* 1 0 1 - What the actual */
                case 5:
                    cout << "I am a confused robot" << endl;
                    stop(); 
                    break;

                /* 1 1 0 - Moving Slightly Right */
                case 6:
                    left_motor((ramp_base_power - ramp_delta_p), FORWARD);
                    right_motor((ramp_base_power + ramp_delta_p), FORWARD); 
                    cout << "Slightly Right" << endl; 
                    break;
                    
                /* 1 1 1 - Junction Detected */
                case 7:
                  stop();
                  cout << "Junction Detected?" << endl;
                  break;         
            }
        
        }
        
        cout << "Re-entry" << endl;
        
        while(ramp_watch.read() < up_t4) {
        
            /* Move forward blind at 80 */
            left_motor(ramp_power_l, FORWARD);
            right_motor(ramp_power_r, FORWARD); 
        
        }
        
        /* Read Line Sensors */
        read_line_sensors();
        
        /* DEBUG */
        cout << "Creeping to Junction" << endl;

	    /* Position axel over Junction Centre */
        while((robot.line & 0x08) == 0) {
            
            /* Follow Line */
            switch((robot.line & 0x07)) {
                
                /* 0 0 0 - Where's the line? */
                case 0:
                    cout << "No line detected" << endl;
                    stop(); 
                    break;
                
                /* 0 0 1 - Moving Serverly Left */
                case 1:
                    left_motor((base_power_creep + (2*delta_p_creep)), FORWARD);
                    right_motor((base_power_creep - (2*delta_p_creep)), FORWARD);
                    cout << "Serverely Left" << endl;
                    break;
            
                /* 0 1 0 - Following the Line */
                case 2:
                    left_motor(base_power_creep, FORWARD);
                    right_motor(base_power_creep, FORWARD);
                    cout << "All Good" << endl; 
                    break;
                    
                /* 0 1 1 - Moving Slightly Left */
                case 3:
                    left_motor((base_power_creep + delta_p_creep), FORWARD);
                    right_motor((base_power_creep - delta_p_creep), FORWARD);
                    cout << "Slightly Left" << endl;  
                    break;
                    
                /* 1 0 0 - Moving Serverly Right */
                case 4:
                    left_motor((base_power_creep - (2*delta_p_creep)), FORWARD);
                    right_motor((base_power_creep + (2*delta_p_creep)), FORWARD); 
                    cout << "Serverely Right" << endl; 
                    break;
       
                /* 1 0 1 - What the actual */
                case 5:
                    cout << "I am a confused robot" << endl;
                    stop(); 
                    break;

                /* 1 1 0 - Moving Slightly Right */
                case 6:
                    left_motor((base_power_creep - delta_p_creep), FORWARD);
                    right_motor((base_power_creep + delta_p_creep), FORWARD); 
                    cout << "Slightly Right" << endl; 
                    break;        
	        }      
        
            read_line_sensors();
	    }
	    
	    stop();
	    ramp_watch.stop();             
    }
}

