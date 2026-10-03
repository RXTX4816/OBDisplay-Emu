#ifndef GROUPS_H
#define GROUPS_H

#include <stdint.h>

// Reply to a group reading request for `group` from the current ECU
// (current_addr / current_ecu), with block counter `counter`. Writes the block
// to `out` (room for 16 bytes) and returns its length: 4 for the KWP_REFUSE
// sent for group 0, otherwise 16.
uint8_t build_group_reading(uint8_t group, uint8_t counter, uint8_t* out);

#endif
