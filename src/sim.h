#ifndef SIM_H
#define SIM_H

#include <stdint.h>

struct SimState
{
    unsigned long start_ms;
    float distance_km;            // accumulated travel distance since connect
    float fuel_L;                 // current fuel level, decreases with distance
    unsigned long last_update_ms; // timestamp for dt-based odometer/fuel integration
};
extern SimState sim_state;

// Clock the simulation runs on: millis() in the firmware (main.cpp), a fake
// clock in the tests.
uint32_t sim_millis();

float get_simulated_speed_kmh();
uint8_t get_current_gear(float speed_kmh);
int8_t get_simulated_coolant_temp();
int8_t get_simulated_oil_temp();
uint8_t get_simulated_oil_level();
uint16_t get_simulated_rpm();
uint8_t get_simulated_engine_load();
uint8_t get_simulated_exhaust_temp_b();
uint8_t get_simulated_o2_voltage_b();

#endif
