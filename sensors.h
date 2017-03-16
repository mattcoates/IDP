#ifndef SENSORS_H
#define SENSORS_H

/* Function Prototypes */
void read_line_sensors(void);
void read_limit_switches(void);
void signal_pallet_type(int colour);
void signal_new_load(void);
void clear_leds(void);

#endif
