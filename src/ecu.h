#ifndef ECU_H
#define ECU_H

#include <avr/pgmspace.h>
#include <stdint.h>

/// ECU DEFINITIONS (PROGMEM)
struct ECUDef
{
    uint8_t address;
    uint16_t baudrate;
    char part_number[13];
    char component[19];
    char coding_wsc[17];
    uint8_t num_groups;
    uint8_t num_faults;
    uint8_t faults[8][3];
    uint8_t groups[23][4][3]; // groups[group_idx][field_idx][type/a/b]
};

// cppcheck-suppress unknownMacro
extern const ECUDef ECU_TABLE[] PROGMEM;
extern const uint8_t ECU_COUNT; // number of entries in ECU_TABLE

void load_ecu_def(const ECUDef* ecu_progmem, ECUDef& ecu_ram);

extern ECUDef current_ecu;   // loaded from PROGMEM on connect
extern uint8_t current_addr; // decoded from 5-baud init

#endif
