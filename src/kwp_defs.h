#ifndef KWP_DEFS_H
#define KWP_DEFS_H

#include <stdint.h>

/// VARIABLES/TYPES
static const uint8_t KWP_ACKNOWLEDGE = 0x09; // the module has no more data to send
static const uint8_t KWP_REFUSE = 0x0A;      // the module can not fulfill a request
static const uint8_t KWP_DISCONNECT = 0x06;  // the tester wants to disconnect from the module

static const uint8_t KWP_REQUEST_EXTRA_ID =
    0x00; // response: KWP_RECEIVE_ID_DATA (data available) / KWP_ACKNOWLEDGE (no data available)
static const uint8_t KWP_REQUEST_LOGIN =
    0x2B; // response: KWP_ACKNOWLEDGE (login successful) / KWP_REFUSE (login not successful)
static const uint8_t KWP_REQUEST_RECODE = 0x10; // response: KWP_REQUEST_EXTRA_ID...
static const uint8_t KWP_REQUEST_FAULT_CODES =
    0x07; // response: KWP_RECEIVE_FAULT_CODES (ok) / KWP_REFUSE (fault codes not supported)
static const uint8_t KWP_REQUEST_CLEAR_FAULTS =
    0x05; // response: KWP_ACKNOWLEDGE (ok) / KWP_REFUSE (clearing fault codes not supported)
static const uint8_t KWP_REQUEST_ADAPTATION =
    0x21; // response: KWP_RECEIVE_ADAPTATION (ok) / KWP_REFUSE (invalid channel)
static const uint8_t KWP_REQUEST_ADAPTATION_TEST =
    0x22; // response: KWP_RECEIVE_ADAPTATION (ok) / KWP_REFUSE (invalid channel)
static const uint8_t KWP_REQUEST_ADAPTATION_SAVE =
    0x2A; // response: KWP_RECEIVE_ADAPTATION (ok) / KWP_REFUSE (invalid channel or value)
static const uint8_t KWP_REQUEST_GROUP_READING =
    0x29; // response: KWP_RECEIVE_GROUP_READING (ok) / KWP_ACKNOWLEDGE (empty group) / KWP_REFUSE
          // (invalid group)
static const uint8_t KWP_REQUEST_GROUP_READING_0 =
    0x12; // response: KWP_RECEIVE_GROUP_READING (ok) / KWP_ACKNOWLEDGE (empty group) / KWP_REFUSE
          // (invalid group)
static const uint8_t KWP_REQUEST_READ_ROM =
    0x03; // response: KWP_RECEIVE_ROM (ok) / KWP_REFUSE (reading ROM not supported or invalid
          // parameters)
static const uint8_t KWP_REQUEST_OUTPUT_TEST =
    0x04; // response: KWP_RECEIVE_OUTPUT_TEST (ok) / KWP_REFUSE (output tests not supported)
static const uint8_t KWP_REQUEST_BASIC_SETTING =
    0x28; // response: KWP_RECEIVE_BASIC_SETTING (ok) / KWP_ACKNOWLEDGE (empty group) / KWP_REFUSE
          // (invalid channel or not supported)
static const uint8_t KWP_REQUEST_BASIC_SETTING_0 =
    0x11; // response: KWP_RECEIVE_BASIC_SETTING (ok) / KWP_ACKNOWLEDGE (empty group) / KWP_REFUSE
          // (invalid channel or not supported)

static const uint8_t KWP_RECEIVE_ID_DATA =
    0xF6; // request: connect/KWP_REQUEST_EXTRA_ID/KWP_REQUEST_RECODE
static const uint8_t KWP_RECEIVE_FAULT_CODES = 0xFC; // request: KWP_REQUEST_FAULT_CODES
static const uint8_t KWP_RECEIVE_ADAPTATION =
    0xE6; // request: KWP_REQUEST_ADAPTATION/KWP_REQUEST_ADAPTATION_TEST/KWP_REQUEST_ADAPTATION_SAVE
static const uint8_t KWP_RECEIVE_GROUP_READING = 0xE7; // request: KWP_REQUEST_GROUP_READING
static const uint8_t KWP_RECEIVE_ROM = 0xFD;           // request: KWP_REQUEST_READ_ROM
static const uint8_t KWP_RECEIVE_OUTPUT_TEST = 0xF5;   // request: KWP_REQUEST_OUTPUT_TEST
static const uint8_t KWP_RECEIVE_BASIC_SETTING = 0xF4; // request: KWP_REQUEST_BASIC_SETTING

#endif
