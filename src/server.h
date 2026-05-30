#include "display.h"
#include <Arduino.h>

#define PIN_TX 18 // Serial1
#define PIN_RX 19

#define ADDR 0x17
#define BAUDRATE 10400

#define TIMEOUT 1250

// Some diagnostic clients expect different physical-layer behavior.
// - On real K-line, the *tester* often sees an electrical echo of its own TX.
//   When using separate RX/TX wires, that echo is usually not present.
//   Keep this configurable.
#ifndef KWP_EMU_ECHO_RX_BYTES
#define KWP_EMU_ECHO_RX_BYTES 0
#endif

#ifndef KWP_EMU_INTERBYTE_DELAY_MS
#define KWP_EMU_INTERBYTE_DELAY_MS 5
#endif

// Some clients send a complement after the last byte too; others don't.
// If enabled, we will consume it only if it matches the expected complement.
#ifndef KWP_EMU_CONSUME_OPTIONAL_LAST_COMPLEMENT
#define KWP_EMU_CONSUME_OPTIONAL_LAST_COMPLEMENT 1
#endif

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

// Forward declarations
bool KWP_send_fault_codes_empty();

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
    uint8_t faults[4][3];
    uint8_t groups[23][4][3]; // groups[group_idx][field_idx][type/a/b]
};

// Helper: copy ECUDef from PROGMEM to RAM
void load_ecu_def(const ECUDef* ecu_progmem, ECUDef& ecu_ram)
{
    ecu_ram.address = pgm_read_byte(&ecu_progmem->address);
    ecu_ram.baudrate = pgm_read_word(&ecu_progmem->baudrate);
    memcpy_P(ecu_ram.part_number, &ecu_progmem->part_number, 13);
    memcpy_P(ecu_ram.component, &ecu_progmem->component, 19);
    memcpy_P(ecu_ram.coding_wsc, &ecu_progmem->coding_wsc, 17);
    ecu_ram.num_groups = pgm_read_byte(&ecu_progmem->num_groups);
    ecu_ram.num_faults = pgm_read_byte(&ecu_progmem->num_faults);
    for (uint8_t i = 0; i < 4; i++)
        for (uint8_t j = 0; j < 3; j++)
            ecu_ram.faults[i][j] = pgm_read_byte(&ecu_progmem->faults[i][j]);
    for (uint8_t i = 0; i < 23; i++)
        for (uint8_t j = 0; j < 4; j++)
            for (uint8_t k = 0; k < 3; k++)
                ecu_ram.groups[i][j][k] = pgm_read_byte(&ecu_progmem->groups[i][j][k]);
}

// Measurement value encoding formulas (VCDS/KWP1281 standard):
// 0x01: rpm   = A * B * 0.2          (A=40, B=rpm/8)
// 0x02: %     = A * B * 0.002        (placeholder for fuel trim; negative not representable)
// 0x04: -     = A * B * 0.001        (dimensionless with decimal)
// 0x05: °C    = A * B * 0.1 - 40     (A=10, B=°C+40)
// 0x06: V     = A * B * 0.001        (A=100, B=V*10)
// 0x07: km/h  = A * B * 0.01         (A=100, B=km/h)
// 0x0C: bar   = A * B * 0.001        (A=42, B=10 → 0.42 bar)
// 0x0E: -     = raw label lookup; B is the raw index/value VCDS maps via label file
// 0x0F: bits  = B displayed as 8-bit binary string
// 0x14: Ohm   = A * B * 0.1          (A=10, B=Ohm)
// 0x17: mbar  = A * B * 0.04         (A=100, B=mbar/4)
// 0x1A: °     = A * B * 0.1 - 127   (A=10, B=°+127; 0°→B=127)
// 0x21: -     = A * B * 0.1          (A=10, B=val; for servo positions 0-255)

// cppcheck-suppress unknownMacro
static const ECUDef ECU_TABLE[] PROGMEM = {
    // 0x01 Engine (Marelli 4LV) — 036 906 034AM
    // Dynamic groups 1 and 3 are overridden in KWP_send_group_reading.
    // Type encoding: all-zero group → KWP_ACKNOWLEDGE; group > num_groups → KWP_REFUSE.
    {0x01,
     9600,
     "036 906 034AM",
     "MARELLI 4LV 3290  ",
     "00031  WSC 01317  ",
     23,
     0,
     {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
     {
         // Grp1 (dynamic): RPM, Temp, Lambda%, Readiness bits — overridden in code
         {{0x01, 40, 0}, {0x05, 10, 57}, {0x02, 10, 0}, {0x0F, 0, 0xB2}},
         // Grp2: RPM=0, Load=0.0%, TimeCorr=0.0ms, AbsPres=1012.0mbar
         {{0x01, 40, 0}, {0x21, 10, 0}, {0x0D, 10, 0}, {0x17, 100, 253}},
         // Grp3 (dynamic): RPM, AbsPres, TBAngle, SteerAngle — overridden in code
         {{0x01, 40, 0}, {0x17, 100, 254}, {0x21, 1, 55}, {0x1A, 10, 127}},
         // Grp4: RPM=0, 11.70V, 17.0°C, 14.0°C
         {{0x01, 40, 0}, {0x06, 100, 117}, {0x05, 10, 57}, {0x05, 10, 54}},
         // Grp5: RPM=0, Load=0.0%, Speed=0.0km/h, PartThrottle (label idx 0)
         {{0x01, 40, 0}, {0x21, 10, 0}, {0x07, 100, 0}, {0x0E, 0, 0}},
         // Grp6: RPM=0, Load=0.0%, 14.0°C, Lambda=-1.0% (placeholder 0)
         {{0x01, 40, 0}, {0x21, 10, 0}, {0x05, 10, 54}, {0x02, 10, 0}},
         // Grp7-9: empty → group reading with zero fields
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         // Grp10: RPM=0, Load=0.0%, Load=6.0%, SteerAngle=0.0°
         {{0x01, 40, 0}, {0x21, 10, 0}, {0x21, 10, 6}, {0x1A, 10, 127}},
         // Grp11: RPM=0, 17.0°C, 14.0°C, SteerAngle=0.0°
         {{0x01, 40, 0}, {0x05, 10, 57}, {0x05, 10, 54}, {0x1A, 10, 127}},
         // Grp12-13: empty → ACK
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         // Grp14: RPM=0, Load=0.0%, 0.0(no unit), Enabled (label idx 1)
         {{0x01, 40, 0}, {0x21, 10, 0}, {0x0E, 0, 0}, {0x0E, 0, 1}},
         // Grp15: 0.0, 0.0, 0.0, Enabled
         {{0x0E, 0, 0}, {0x0E, 0, 0}, {0x0E, 0, 0}, {0x0E, 0, 1}},
         // Grp16: 0.0, empty, empty, Enabled
         {{0x0E, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x0E, 0, 1}},
         // Grp17: empty → ACK
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         // Grp18: RPM=0, RPM=0, Lambda=0.0%, Lambda=0.0%
         {{0x01, 40, 0}, {0x01, 40, 0}, {0x02, 10, 0}, {0x02, 10, 0}},
         // Grp19: empty → ACK
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         // Grp20: SteerAngle=0.0° × 4
         {{0x1A, 10, 127}, {0x1A, 10, 127}, {0x1A, 10, 127}, {0x1A, 10, 127}},
         // Grp21: empty → ACK
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         // Grp22: RPM=0, Load=0.0%, SteerAngle=0.0°, SteerAngle=0.0°
         {{0x01, 40, 0}, {0x21, 10, 0}, {0x1A, 10, 127}, {0x1A, 10, 127}},
         // Grp23: RPM=0, Load=0.0%, SteerAngle=0.0°, SteerAngle=0.0°
         {{0x01, 40, 0}, {0x21, 10, 0}, {0x1A, 10, 127}, {0x1A, 10, 127}},
     }},
    // 0x03 ABS/ESP — 1C0 907 379
    {0x03,
     9600,
     "1C0 907 379   ",
     "ESP 20 CAN V005   ",
     "10241  WSC 01317  ",
     5,
     0,
     {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
     {
         // Grp1: wheel speeds 0.0 km/h × 4
         {{0x07, 100, 0}, {0x07, 100, 0}, {0x07, 100, 0}, {0x07, 100, 0}},
         // Grp2: wheel speeds 255.0 km/h × 4 (sensor max/unplugged)
         {{0x07, 100, 255}, {0x07, 100, 255}, {0x07, 100, 255}, {0x07, 100, 255}},
         // Grp3: Not Oper., Not Oper., N/A, N/A (label idx 0)
         {{0x0E, 0, 0}, {0x0E, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         // Grp4: 0.00° SteerAngle, 0.31 m/s² LatAccel (A*B*0.001: 31*10=0.31), TurnRate
         // placeholder, N/A
         {{0x1A, 10, 127}, {0x04, 31, 10}, {0x0E, 0, 0}, {0x00, 0, 0}},
         // Grp5: -1.27 bar (placeholder 0), 0.42 bar (A=42, B=10: 42*10*0.001=0.42), N/A, N/A
         {{0x0C, 0, 0}, {0x0C, 42, 10}, {0x00, 0, 0}, {0x00, 0, 0}},
         // Grp6-23: unused padding
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
     }},
    // 0x08 HVAC — 3B1 907 044 C (Climatronic)
    {0x08,
     9600,
     "3B1 907 044 C ",
     "CLIMATRONIC C 0.7.0",
     "01000  WSC 01317  ",
     8,
     1,
     {{0x01, 0x27, 0x4E}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
     {
         // Grp1: A/C sw-off cond=9 (label), EngSpeedRecog=0, Speed=0.0km/h, StandingTime=121.0min
         // StandingTime: 0x21 A=10 B=121 → 10*121*0.1=121.0
         {{0x0E, 0, 9}, {0x0E, 0, 0}, {0x07, 100, 0}, {0x21, 10, 121}},
         // Grp2: Measured=42.0, Specified=42.0, Pos:air-supply-cooled=219.0,
         // Pos:air-supply-heated=42.0
         {{0x21, 10, 42}, {0x21, 10, 42}, {0x21, 10, 219}, {0x21, 10, 42}},
         // Grp3: Measured=221.0, Specified=221.0, Pos:panel=221.0, Pos:footwell=40.0
         {{0x21, 10, 221}, {0x21, 10, 221}, {0x21, 10, 221}, {0x21, 10, 40}},
         // Grp4: Measured=223.0, Specified=223.0, Pos:footwell=223.0, Pos:defroster=39.0
         {{0x21, 10, 223}, {0x21, 10, 223}, {0x21, 10, 223}, {0x21, 10, 39}},
         // Grp5: Measured=237.0, Specified=234.0, Pos:fresh-air=234.0, Pos:recirc=30.0
         {{0x21, 10, 237}, {0x21, 10, 234}, {0x21, 10, 234}, {0x21, 10, 30}},
         // Grp6: Temp-display=0.0°C, Air-intake=7.0°C, Outside=0.0°C, Sun-sensor=0.0%
         {{0x05, 10, 40}, {0x05, 10, 47}, {0x05, 10, 40}, {0x21, 10, 0}},
         // Grp7: OutletPanel=0.0(raw), FloorOutlet=5.0°C, PanelNearLCD=3.0°C, N/A
         {{0x0E, 0, 0}, {0x05, 10, 45}, {0x05, 10, 43}, {0x00, 0, 0}},
         // Grp8: Spec.V-blower=0.00V, Meas.V-blower=0.28V (A=28,B=10: 0.28V), Meas.V-A/C=12.18V,
         // empty
         {{0x06, 100, 0}, {0x06, 28, 10}, {0x06, 100, 122}, {0x00, 0, 0}},
         // Grp9-23: unused padding
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
     }},
    // 0x15 Airbags — no real data provided, kept as placeholder
    {0x15,
     9600,
     "6Q0 909 605 A ",
     "02 AIRBAG VW5 0004 ",
     "12338  WSC 01317  ",
     1,
     0,
     {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
     {
         {{0x06, 100, 0}, {0x06, 100, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
     }},
    // 0x17 Instruments cluster — 1J0-920-XX0.LBL
    // All 3 groups are overridden dynamically in KWP_send_group_reading.
    // Static entries here are fallback / reference only.
    {0x17,
     10400,
     "1J0 920 822 A ",
     "KOMBI+WEGFAHRS. BOO",
     "05143  WSC 01266  ",
     3,
     0,
     {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
     {
         // Grp1: Speed(dynamic), RPM(dynamic), OilPressure(label idx 2), Time(A=21 B=50
         // placeholder)
         {{0x07, 100, 0}, {0x01, 40, 0}, {0x0E, 0, 2}, {0x0E, 21, 50}},
         // Grp2: Odometer(dynamic), FuelLevel=23.0L, FuelSenderRes=93Ohm, AmbientTemp=0.0°C
         // FuelLevel: 0x04 A*B*0.001=23.0 → A=100 B=230
         // FuelSender: 0x14 A*B*0.1=93Ohm → A=10 B=93
         {{0x24, 0, 0}, {0x04, 100, 230}, {0x14, 10, 93}, {0x05, 10, 40}},
         // Grp3: CoolantTemp(dynamic), OilLevel=OK(label 0), OilTemp=11.0°C, N/A
         {{0x05, 10, 52}, {0x0E, 0, 0}, {0x05, 10, 51}, {0x00, 0, 0}},
         // Grp4-23: unused
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
     }},
    // 0x19 CAN Gateway — 6N0 909 901 (no groups available in VCDS)
    {0x19,
     9600,
     "6N0 909 901   ",
     "Gateway K<->CAN 0001",
     "00006  WSC 01317  ",
     0,
     0,
     {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
     {
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
     }},
    // 0x46 Central Convenience — 1J0 959 799 AH
    {0x46,
     9600,
     "1J0 959 799 AH",
     "2K Zentral-SG Komf. ",
     "04097  WSC 01317  ",
     16,
     2,
     {{0x00, 0x94, 0x3F}, {0x00, 0x94, 0x40}, {0x00, 0, 0}, {0x00, 0, 0}},
     {
         // Grp1: RearWinLock=OFF(0), DDLockSw=NotOper(0), DDWindowMotor=Still(0), N/A
         {{0x0E, 0, 0}, {0x0E, 0, 0}, {0x0E, 0, 0}, {0x00, 0, 0}},
         // Grp2: window switches — all Not Oper (0)
         {{0x0E, 0, 0}, {0x0E, 0, 0}, {0x0E, 0, 0}, {0x0E, 0, 0}},
         // Grp3 (drivers door): DDKeySw=NotOper, LatchProtect=binary 0b01,
         // LatchFeedback=Unlocked(0), CLFeedback=NotSafe(0)
         {{0x0E, 0, 0}, {0x0F, 0, 0x01}, {0x0E, 0, 0}, {0x0E, 0, 0}},
         // Grp4 (mirrors): DDMirrorUD=NotOper, DDMirrorLR=NotOper, DDFolding=NotInstalled, N/A
         {{0x0E, 0, 0}, {0x0E, 0, 0}, {0x0E, 0, 0}, {0x00, 0, 0}},
         // Grp5 (pass door): PassWindowSw=NotOper, PassLockSw=NotOper, PassFolding=NotInstalled,
         // N/A
         {{0x0E, 0, 0}, {0x0E, 0, 0}, {0x0E, 0, 0}, {0x00, 0, 0}},
         // Grp6 (pass door): PassKeySw=NotOper, LatchProtect=binary 0b01, LatchFeedback=Unlocked,
         // CLFeedback=NotSafe
         {{0x0E, 0, 0}, {0x0F, 0, 0x01}, {0x0E, 0, 0}, {0x0E, 0, 0}},
         // Grp7 (RR door): RRWindowSw=NotOper, LatchProtect=binary 0b01, LatchFeedback=Unlocked,
         // CLFeedback=NotSafe
         {{0x0E, 0, 0}, {0x0F, 0, 0x01}, {0x0E, 0, 0}, {0x0E, 0, 0}},
         // Grp8 (LR door): LRWindowSw=NotOper, LatchProtect=binary 0b01, LatchFeedback=Unlocked,
         // CLFeedback=NotSafe
         {{0x0E, 0, 0}, {0x0F, 0, 0x01}, {0x0E, 0, 0}, {0x0E, 0, 0}},
         // Grp9 (signals): InstLightSig=0.0%, CarSpeed=0.0km/h, KeyRemoteSig=0b00000000,
         // InteriorMon=NotInstalled(0)
         {{0x21, 10, 0}, {0x07, 100, 0}, {0x0F, 0, 0x00}, {0x0E, 0, 0}},
         // Grp10 (signals): SContact=Activated(1), MirrorHeat=OFF(0), TrunkLock=NotOper(0),
         // Term15=ON(1)
         {{0x0E, 0, 1}, {0x0E, 0, 0}, {0x0E, 0, 0}, {0x0E, 0, 1}},
         // Grp11 (signals): HoodRadioSw=Closed(1), TrunkLatchSw=Closed(1), SunroofSync=Yes(1),
         // CLTempSw=N/A(0)
         {{0x0E, 0, 1}, {0x0E, 0, 1}, {0x0E, 0, 1}, {0x0E, 0, 0}},
         // Grp12 (CAN): BusOK(1), FrOptEquip=binary, RrOptEquip=binary, EmptyOptEquip(0)
         {{0x0E, 0, 1}, {0x0F, 0, 0xFF}, {0x0F, 0, 0xFF}, {0x0E, 0, 0}},
         // Grp13 (remotes): no value × 3, KeyNumber=0
         {{0x0E, 0, 0}, {0x0E, 0, 0}, {0x0E, 0, 0}, {0x0E, 0, 0}},
         // Grp14 (CCM): Term30=12.32V, RearUnlatch=NotOper(0), InteriorMon=NotInstalled(0),
         // ThermoProtect=binary 0b00011111
         {{0x06, 100, 123}, {0x0E, 0, 0}, {0x0E, 0, 0}, {0x0F, 0, 0x1F}},
         // Grp15 (alarm): Last=16, 2nd=4, 3rd=128, 4th=128
         {{0x0E, 0, 16}, {0x0E, 0, 4}, {0x0E, 0, 128}, {0x0E, 0, 128}},
         // Grp16 (auto locks): ImobKeyRecogn=NotInstalled(0), AutoIntLock=NotOper(0),
         // RearLatchDetent=Closed(1), N/A
         {{0x0E, 0, 0}, {0x0E, 0, 0}, {0x0E, 0, 1}, {0x00, 0, 0}},
         // Grp17-23: unused padding
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
     }}};

/// GLOBAL STATE
ECUDef current_ecu;          // loaded from PROGMEM on connect
uint8_t current_addr = 0x00; // decoded from 5-baud init

struct SimState
{
    unsigned long start_ms;
    uint16_t odometer_km;
    uint16_t fuel_tenth_L; // 550 = 55L full
} sim_state;

bool awake = false;
bool initial_condition = HIGH;

bool connected = false;

uint8_t block_counter = 0;
/**
 * @brief Read OBD input from ECU
 *
 * @return uint8_t The incoming byte or -1 if timeout
 */
int16_t OBD_read()
{
    unsigned long timeout = millis() + TIMEOUT;
    while (!Serial1.available())
    {
        if (millis() >= timeout)
        {
            Serial.println("ERROR: OBD_read() timeout");
            return -1;
        }
    }
    int16_t data = Serial1.read();

#if KWP_EMU_ECHO_RX_BYTES
    // Optional: mimic "echo" some K-line setups exhibit.
    Serial1.write((uint8_t)data);
#endif

    return data;
}

/**
 * @brief Write data to the ECU, wait 5ms before each write to ensure connectivity.
 *
 * @param data The data to send.
 */
void OBD_write(uint8_t data)
{
    delay(KWP_EMU_INTERBYTE_DELAY_MS);
    Serial1.write(data);
}

static inline void OBD_consume_if_next_byte_is(uint8_t expected)
{
#if KWP_EMU_CONSUME_OPTIONAL_LAST_COMPLEMENT
    if (Serial1.available() && Serial1.peek() == expected)
    {
        (void)Serial1.read();
    }
#else
    (void)expected;
#endif
}

/**
 * @brief Send a request to the ECU
 *
 * @param s Array where the data is stored
 * @param size The size of the request
 * @return true If no errors occured, will resume
 * @return false If errors occured, will disconnect
 */
bool KWP_send_block(uint8_t* s, int size)
{
    Serial.print("TX [BC=");
    Serial.print(block_counter);
    Serial.print("]: ");
    for (uint8_t i = 0; i < size; i++)
    {
        if (s[i] < 0x10)
            Serial.print("0");
        Serial.print(s[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    for (uint8_t i = 0; i < size; i++)
    {
        uint8_t data = s[i];
        OBD_write(data);

        if (i < size - 1)
        {
            int16_t complement = OBD_read();
            if (complement != (data ^ 0xFF))
            {
                Serial.print("  complement error byte[");
                Serial.print(i);
                Serial.print("]=");
                Serial.print(data, HEX);
                Serial.print(" got=");
                Serial.print(complement, HEX);
                Serial.print(" want=");
                Serial.println(data ^ 0xFF, HEX);
                return false;
            }
        }
        else
        {
            // Some clients also send a complement for the final byte.
            OBD_consume_if_next_byte_is(data ^ 0xFF);
        }
    }
    block_counter++;
    return true;
}

bool KWP_send_syncbytes()
{
    uint8_t s[32] = {0x55, 0x01, 0x8A};
    uint8_t size = 3;
    Serial.print("Sending ");
    for (uint8_t i = 0; i < size; i++)
    {
        Serial.print(s[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    for (uint8_t i = 0; i < size; i++)
    {
        uint8_t data = s[i];
        OBD_write(data);

        if (i == 2)
        {
            int16_t complement = OBD_read();
            if (complement != (data ^ 0xFF))
            {
                Serial.print("Received: ");
                Serial.print(complement, HEX);
                Serial.print(" Expected: ");
                Serial.println(data ^ 0xFF, HEX);
                return false;
            }
        }
    }
    block_counter++;
    return true;
}

// Time-based simulation helpers
float get_simulated_speed_kmh()
{
    unsigned long elapsed_ms = millis() - sim_state.start_ms;

    // Short idle: first 2s at 0 km/h
    if (elapsed_ms < 2000UL)
        return 0.0f;

    // Sine-wave 0-120 km/h, 30s period (ms resolution so polls see changes)
    float phase = (float)(elapsed_ms % 30000UL) / 30000.0f; // 0..1
    float angle = phase * 6.283185f;                        // 2π
    float speed = (sinf(angle) + 1.0f) * 60.0f;             // 0..120
    return speed;
}

int8_t get_simulated_coolant_temp()
{
    unsigned long elapsed_s = (millis() - sim_state.start_ms) / 1000;
    if (elapsed_s < 30)
        return (int8_t)(20 + elapsed_s * 2.33f); // 20..90 over 30s
    return 90;
}

int8_t get_simulated_oil_temp()
{
    int8_t coolant = get_simulated_coolant_temp();
    return (int8_t)(coolant - 15); // lags coolant by ~15°C
}

uint16_t get_simulated_rpm()
{
    float speed = get_simulated_speed_kmh();
    if (speed < 5.0f)
        return 800; // idle
    // RPM ~= (speed/30 + 0.8) * 1000, peaks ~4800 at 120 km/h
    uint16_t rpm = (uint16_t)((speed / 30.0f + 0.8f) * 1000.0f);
    return rpm;
}

bool KWP_send_group_reading(uint8_t group)
{
    Serial.print("group reading group=");
    Serial.println(group);

    // Validate group number
    if (group == 0)
    {
        // Invalid group: send REFUSE
        uint8_t refuse_buf[4] = {0x03, block_counter, KWP_REFUSE, 0x03};
        return KWP_send_block(refuse_buf, 4);
    }

    // Send group data from ECU definition
    uint8_t buf[16] = {0x0F, block_counter, KWP_RECEIVE_GROUP_READING,
                       0x00, 0x00,          0x00,
                       0x00, 0x00,          0x00,
                       0x00, 0x00,          0x00,
                       0x00, 0x00,          0x00,
                       0x03};

    // Dynamic overrides — populated before falling through to static table
    if (current_addr == 0x17 && group == 1)
    {
        // Grp1: Speed(km/h), RPM, OilPressureIndicator(static label 2), Time(placeholder A=21 B=50)
        float speed = get_simulated_speed_kmh();
        uint16_t rpm = get_simulated_rpm();

        buf[3] = 0x07;
        buf[4] = 100;
        buf[5] = (uint8_t)speed; // km/h
        buf[6] = 0x01;
        buf[7] = 40;
        buf[8] = (uint8_t)(rpm / 8); // RPM
        buf[9] = 0x0E;
        buf[10] = 0;
        buf[11] = 2; // oil pressure (label idx 2)
        buf[12] = 0x0E;
        buf[13] = 21;
        buf[14] = 50; // time 21:50 (placeholder; verify encoding with real car)
    }
    else if (current_addr == 0x17 && group == 2)
    {
        // Grp2: Odometer(km), FuelLevel=23.0L, FuelSenderRes=93Ohm, AmbientTemp=0.0°C
        unsigned long elapsed_s = (millis() - sim_state.start_ms) / 1000;
        float speed = get_simulated_speed_kmh();
        uint16_t km_delta = (uint16_t)(speed * elapsed_s / 3600.0f);
        uint16_t total_km = sim_state.odometer_km + km_delta;

        buf[3] = 0x24;
        buf[4] = (uint8_t)(total_km >> 8);
        buf[5] = (uint8_t)(total_km & 0xFF); // odometer
        buf[6] = 0x04;
        buf[7] = 100;
        buf[8] = 230; // 23.0L fuel (100*230*0.001)
        buf[9] = 0x14;
        buf[10] = 10;
        buf[11] = 93; // 93Ohm sender (10*93*0.1)
        buf[12] = 0x05;
        buf[13] = 10;
        buf[14] = 40; // 0.0°C ambient (10*40*0.1-40)
    }
    else if (current_addr == 0x17 && group == 3)
    {
        // Grp3: CoolantTemp, OilLevel=OK(label 0), OilTemp=11.0°C, N/A
        int8_t coolant = get_simulated_coolant_temp();

        buf[3] = 0x05;
        buf[4] = 10;
        buf[5] = (uint8_t)(coolant + 40); // coolant °C
        buf[6] = 0x0E;
        buf[7] = 0;
        buf[8] = 0; // oil level OK (label idx 0)
        buf[9] = 0x05;
        buf[10] = 10;
        buf[11] = 51; // 11.0°C oil temp
        buf[12] = 0x00;
        buf[13] = 0;
        buf[14] = 0; // N/A
    }
    else if (current_addr == 0x01 && group == 1)
    {
        // Grp1: RPM, AirTemp=17°C(static), Lambda=0.0%(static), ReadinessBits(static)
        uint16_t rpm = get_simulated_rpm();

        buf[3] = 0x01;
        buf[4] = 40;
        buf[5] = (uint8_t)(rpm / 8); // RPM
        buf[6] = 0x05;
        buf[7] = 10;
        buf[8] = 57; // 17.0°C air temp
        buf[9] = 0x02;
        buf[10] = 10;
        buf[11] = 0; // 0.0% lambda
        buf[12] = 0x0F;
        buf[13] = 0;
        buf[14] = 0xB2; // readiness bits 10110010
    }
    else if (current_addr == 0x01 && group == 3)
    {
        // Grp3: RPM, AbsPres=1016mbar(static), TBAngle=5.5°(static), SteerAngle=0.0°(static)
        uint16_t rpm = get_simulated_rpm();

        buf[3] = 0x01;
        buf[4] = 40;
        buf[5] = (uint8_t)(rpm / 8); // RPM
        buf[6] = 0x17;
        buf[7] = 100;
        buf[8] = 254; // 1016 mbar (100*254*0.04)
        buf[9] = 0x21;
        buf[10] = 1;
        buf[11] = 55; // 5.5° TB angle (1*55*0.1)
        buf[12] = 0x1A;
        buf[13] = 10;
        buf[14] = 127; // 0.0° steering (10*127*0.1-127)
    }
    else if (current_addr == 0x03 && group == 1)
    {
        // ABS Grp1: four wheel speeds — all follow simulated vehicle speed
        uint8_t spd = (uint8_t)get_simulated_speed_kmh();
        buf[3] = 0x07;
        buf[4] = 100;
        buf[5] = spd; // FL
        buf[6] = 0x07;
        buf[7] = 100;
        buf[8] = spd; // FR
        buf[9] = 0x07;
        buf[10] = 100;
        buf[11] = spd; // RL
        buf[12] = 0x07;
        buf[13] = 100;
        buf[14] = spd; // RR
    }
    else if (current_addr == 0x08 && group == 1)
    {
        // HVAC Grp1: A/C sw-off cond (static), EngSpeedRecog, Speed, StandingTime
        float speed = get_simulated_speed_kmh();
        buf[3] = 0x0E;
        buf[4] = 0;
        buf[5] = 9; // A/C cond label 9
        buf[6] = 0x0E;
        buf[7] = 0;
        buf[8] = (speed > 5.0f) ? 1 : 0; // engine running
        buf[9] = 0x07;
        buf[10] = 100;
        buf[11] = (uint8_t)speed; // km/h
        buf[12] = 0x21;
        buf[13] = 10;
        buf[14] = 121; // standing time static
    }
    else if (current_addr == 0x46 && group == 9)
    {
        // CCM Grp9: InstLightSig, CarSpeed, KeyRemoteSig, InteriorMon
        float speed = get_simulated_speed_kmh();
        buf[3] = 0x21;
        buf[4] = 10;
        buf[5] = 0; // InstLightSig 0.0%
        buf[6] = 0x07;
        buf[7] = 100;
        buf[8] = (uint8_t)speed; // CarSpeed
        buf[9] = 0x0F;
        buf[10] = 0;
        buf[11] = 0x00; // KeyRemoteSig bits
        buf[12] = 0x0E;
        buf[13] = 0;
        buf[14] = 0; // InteriorMon: not installed
    }
    else if (group <= current_ecu.num_groups)
    {
        // Static group from definition (may be all-zero = empty, still sends 0xE7)
        for (uint8_t i = 0; i < 4; i++)
        {
            buf[3 + i * 3] = current_ecu.groups[group - 1][i][0]; // type
            buf[4 + i * 3] = current_ecu.groups[group - 1][i][1]; // a
            buf[5 + i * 3] = current_ecu.groups[group - 1][i][2]; // b
        }
    }
    // else: group > num_groups — buf already zeroed, sends 0xE7 with empty fields

    return KWP_send_block(buf, 16);
}

bool KWP_send_fault_codes()
{
    if (current_ecu.num_faults == 0)
        return KWP_send_fault_codes_empty();

    uint8_t buf[16] = {0x0F, block_counter, KWP_RECEIVE_FAULT_CODES,
                       0x00, 0x00,          0x00,
                       0x00, 0x00,          0x00,
                       0x00, 0x00,          0x00,
                       0x00, 0x00,          0x00,
                       0x03};

    // Copy up to 4 fault codes from ECU definition
    for (uint8_t i = 0; i < current_ecu.num_faults && i < 4; i++)
    {
        buf[3 + i * 3] = current_ecu.faults[i][0]; // DTC high
        buf[4 + i * 3] = current_ecu.faults[i][1]; // DTC low
        buf[5 + i * 3] = current_ecu.faults[i][2]; // status
    }

    return (KWP_send_block(buf, 16));
}

bool KWP_send_fault_codes_empty()
{
    uint8_t buf[7] = {0x06, block_counter, KWP_RECEIVE_FAULT_CODES, 0xFF, 0xFF, 0x88, 0x03};
    return (KWP_send_block(buf, 7));
}

/**
 * @brief The default way to keep the ECU awake is to send an Acknowledge Block.
 * Alternatives include Group Readings..
 *
 * @return true No errors
 * @return false Errors, disconnect
 */
bool KWP_send_ack()
{
    uint8_t buf[4] = {0x03, block_counter, 0x09, 0x03};
    return (KWP_send_block(buf, 4));
}

bool KWP_send_devicedata(const char* id_string)
{
    uint8_t s[32];
    s[0] = 0x0F;
    s[1] = block_counter;
    s[2] = 0xF6;
    s[15] = 0x03;

    // Copy ID string (12 bytes, padded with spaces if shorter)
    for (uint8_t i = 0; i < 12; i++)
    {
        if (id_string && id_string[i] != '\0')
            s[3 + i] = (uint8_t)id_string[i];
        else
            s[3 + i] = 0x20; // space
    }

    uint8_t size = 0x0F + 1;
    return KWP_send_block(s, size);
}

bool KWP_receive_ack()
{
    unsigned long timeout = millis() + TIMEOUT;
    uint8_t s[32];
    int recvcount = 0;

    while (recvcount < 4)
    {
        while (Serial1.available())
        {
            int16_t data = OBD_read();
            if (data == -1)
            {
                Serial.println("receive ack error AVA=0 or empty buffer");
                return false;
            }
            s[recvcount] = data;
            recvcount++;

            if (recvcount >= 4)
            {
                timeout = millis() + TIMEOUT;
                break;
            }

            if (recvcount == 2 && data != block_counter)
            {
                Serial.print("ACK block counter mismatch: got=0x");
                Serial.print(data, HEX);
                Serial.print(" expected=0x");
                Serial.println(block_counter, HEX);
                return false;
            }

            delay(5);
            OBD_write(data ^ 0xFF); // send complement ack

            timeout = millis() + TIMEOUT;

            // debugstrnum(F(" - KWP_receive_block: Added timeout. ReceiveCount: "),
            // (uint8_t)recvcount); debug(F(". Processed data: ")); debughex(data);
            // debugstrnumln(F(". ACK compl: "), ((!ackeachbyte) && (recvcount == size)) ||
            // ((ackeachbyte) && (recvcount < size)));
        }

        if (millis() >= timeout)
        {
            Serial.print("Timeout - recvcount = ");
            Serial.println(recvcount);
            return false;
        }
    }

    if (s[0] != 0x03 || s[1] != block_counter || s[2] != 0x09 || s[3] != 0x03)
    {
        Serial.print("ACK parse error [BC=");
        Serial.print(block_counter);
        Serial.print("]: got ");
        for (uint8_t i = 0; i < 4; i++)
        {
            if (s[i] < 0x10)
                Serial.print("0");
            Serial.print(s[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        return false;
    }

    Serial.print("ACK ok [BC=");
    Serial.print(block_counter);
    Serial.println("]");
    block_counter++;

    return true;
}

uint8_t wait_5baud()
{
    pinMode(PIN_RX, INPUT_PULLUP);
    initial_condition = digitalRead(PIN_RX);
    Serial.print("initial_condition ");
    Serial.println(initial_condition);
    if (initial_condition == LOW)
    {
        initial_condition = HIGH;
        return 0x00;
    }
    Serial.println("waiting 5baud address on PIN_RX_19 ...");
    g.print("Waiting 5baud..", LEFT, rows[4]);
    bool ready = false;
    unsigned long fall_time = 0;
    while (!ready)
    {
        if (digitalRead(PIN_RX) == LOW)
        {
            fall_time = millis();
            bool glitch = false;
            while (millis() - fall_time < 80)
            {
                if (digitalRead(PIN_RX) == HIGH)
                {
                    glitch = true;
                    break;
                }
            }
            if (!glitch)
                ready = true;
        }
    }
    Serial.println("initial_condition changed");
    g.setColor(TFT_GREEN);
    g.print("---", RIGHT, rows[4]);
    g.setColor(font_color);
    bool bits[10];
    bits[0] = LOW;
    Serial.print(bits[0]);
    unsigned long elapsed = millis() - fall_time;
    if (elapsed < 300)
        delay(300 - elapsed);
    for (uint8_t i = 1; i < sizeof(bits); i++)
    {
        bits[i] = digitalRead(PIN_RX);
        delay(200);
    }
    Serial.print(" ");
    for (uint8_t i = 0; i < sizeof(bits); i++)
    {
        Serial.print(bits[i]);
        Serial.print(" ");
        g.printNumI(bits[i], cols[2 + i * 2], rows[5], 1);
    }
    Serial.println();

    // Debug: print exact bit indices and values
    Serial.println("Debug - individual bits:");
    for (uint8_t i = 0; i < 10; i++)
    {
        Serial.print("bits[");
        Serial.print(i);
        Serial.print("]=");
        Serial.print(bits[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("data[1..7] parity[8] stop[9]: ");
    for (uint8_t i = 1; i <= 9; i++)
    {
        Serial.print(bits[i]);
        Serial.print(" ");
    }
    Serial.println();

    // First validate start (bits[0]=LOW) and stop (bits[9]=HIGH)
    if (bits[0] != LOW || bits[9] != HIGH)
    {
        Serial.println("5baud validation failed (bad start/stop bits)");
        initial_condition = HIGH;
        g.print("   ", RIGHT, rows[4]);
        return 0x00;
    }

    // Decode 7 data bits (bits[1..7]) LSB-first — KWP1281 5-baud uses 7O1 framing
    uint8_t addr = 0;
    for (uint8_t i = 1; i <= 7; i++)
    {
        if (bits[i] == HIGH)
            addr |= (1 << (i - 1));
    }

    // Verify odd parity: bits[8] must make the total number of 1-bits odd
    uint8_t ones = 0;
    for (uint8_t i = 1; i <= 7; i++)
        if (bits[i] == HIGH)
            ones++;
    bool parity_ok = (ones % 2 == 0) ? (bits[8] == HIGH) : (bits[8] == LOW);
    if (!parity_ok)
    {
        Serial.println("5baud parity check failed");
        initial_condition = HIGH;
        g.print("   ", RIGHT, rows[4]);
        return 0x00;
    }

    // 7-bit address range: 0x00 and anything >= 0x80 are impossible
    if (addr == 0x00 || addr >= 0x80)
    {
        Serial.print("decoded address 0x");
        Serial.print(addr, HEX);
        Serial.println(" - rejected (invalid)");
        initial_condition = HIGH;
        g.print("   ", RIGHT, rows[4]);
        return 0x00;
    }

    // Verify address exists in ECU_TABLE
    bool found_in_table = false;
    for (uint8_t i = 0; i < sizeof(ECU_TABLE) / sizeof(ECUDef); i++)
    {
        if (pgm_read_byte(&ECU_TABLE[i].address) == addr)
        {
            found_in_table = true;
            break;
        }
    }

    if (!found_in_table)
    {
        Serial.print("decoded address 0x");
        Serial.print(addr, HEX);
        Serial.println(" - not in ECU table");
        initial_condition = HIGH;
        g.print("   ", RIGHT, rows[4]);
        return 0x00;
    }

    Serial.print("decoded address: 0x");
    Serial.println(addr, HEX);
    g.setColor(TFT_GREEN);
    g.print("---", RIGHT, rows[5]);
    return addr;
}

bool KWP_receive_block(uint8_t buff[], uint8_t& received_count, uint8_t& message_type)
{
    uint8_t recvcount = 0;
    uint8_t expected_total = 0; // total bytes including length byte
    unsigned long timeout = millis() + TIMEOUT;

    while (true)
    {
        while (Serial1.available())
        {
            int16_t raw = OBD_read();
            if (raw < 0)
            {
                Serial.println("data = -1");
                return false;
            }
            uint8_t data = (uint8_t)raw;

            if (recvcount >= 32)
            {
                Serial.println("receive block error: buffer overflow");
                return false;
            }
            buff[recvcount] = data;
            recvcount++;

            if (recvcount == 1)
            {
                expected_total = (uint8_t)(data + 1);
                if (expected_total < 4 || expected_total > 32)
                {
                    Serial.println("receive block error: invalid length");
                    return false;
                }
            }
            else if (recvcount == 2)
            {
                if (data != block_counter)
                {
                    Serial.println("WARNING: block counter does not match");
                }
            }
            else if (recvcount == 3)
            {
                message_type = data;
            }

            timeout = millis() + TIMEOUT;

            if (expected_total != 0 && recvcount >= expected_total)
            {
                received_count = recvcount;
                block_counter++;
                Serial.print("RX [BC=");
                Serial.print(block_counter - 1);
                Serial.print("] type=0x");
                Serial.print(message_type, HEX);
                Serial.print(" len=");
                Serial.print(recvcount);
                Serial.print(" bytes: ");
                for (uint8_t i = 0; i < recvcount; i++)
                {
                    if (buff[i] < 0x10)
                        Serial.print("0");
                    Serial.print(buff[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                return true;
            }

            OBD_write(data ^ 0xFF); // complement ack for all non-final bytes
        }

        if (millis() >= timeout)
        {
            Serial.print("Timeout - recvcount = ");
            Serial.println(recvcount);
            error_timeout(TIMEOUT);
            return false;
        }
    }
}

void reset()
{
    connected = false;
    awake = false;
    block_counter = 0;
    initial_condition = HIGH;

    Serial.println("Waiting 3 sec");
    delay(3000);
    for (uint8_t i = 4; i < 20; i++)
    {
        clearRow(i);
    }
    Serial1.end();
}

bool wakeup()
{
    current_addr = wait_5baud();
    if (current_addr == 0x00)
    {
        Serial.println("5baud error");
        g.setColor(TFT_RED);
        g.print("xxx", RIGHT, rows[5]);
        g.setColor(font_color);
        reset();
        return false;
    }
    awake = true;
    initial_condition = HIGH;
    Serial.println("5baud success");
    return true;
}

bool connect()
{
    // Look up ECU definition by address
    bool found = false;
    for (uint8_t i = 0; i < sizeof(ECU_TABLE) / sizeof(ECUDef); i++)
    {
        if (pgm_read_byte(&ECU_TABLE[i].address) == current_addr)
        {
            load_ecu_def(&ECU_TABLE[i], current_ecu);
            found = true;
            break;
        }
    }
    if (!found)
    {
        Serial.print("unknown address 0x");
        Serial.println(current_addr, HEX);
        reset();
        return false;
    }

    // Determine baud rate from address (safer than reading from PROGMEM struct)
    uint16_t baud = 10400; // default
    if (current_addr == 0x01 || current_addr == 0x03 || current_addr == 0x08 ||
        current_addr == 0x15 || current_addr == 0x19 || current_addr == 0x46)
    {
        baud = 9600;
    }

    Serial.print("ECU addr=0x");
    Serial.print(current_ecu.address, HEX);
    Serial.print(" baud=");
    Serial.println(baud);

    Serial1.begin(baud);
    delay(100); // Give Serial1 time to stabilize after baud change

    if (!KWP_send_syncbytes())
    {
        Serial.println("syncbytes error");
        reset();
        return false;
    }
    g.print("Sending syncbytes..", LEFT, rows[6]);
    Serial.println("syncbytes success");

    // Send 3 identity strings
    for (uint8_t i = 0; i < 3; i++)
    {
        Serial.print("-> sending device data ");
        Serial.print(i + 1);
        Serial.println(" / 3");

        const char* id_str = nullptr;
        if (i == 0)
            id_str = current_ecu.part_number;
        else if (i == 1)
            id_str = current_ecu.component;
        else
            id_str = current_ecu.coding_wsc;

        if (!KWP_send_devicedata(id_str))
        {
            Serial.println("send device data error");
            g.setColor(TFT_RED);
            g.print("xxx", RIGHT, rows[6]);
            g.setColor(font_color);
            reset();
            return false;
        }
        Serial.print("---> receive ack block ");
        Serial.print(i + 1);
        Serial.println(" / 3");
        if (!KWP_receive_ack())
        {
            Serial.println("receive ack block error");
            g.setColor(TFT_RED);
            g.print("xxx", RIGHT, rows[6]);
            g.setColor(font_color);
            reset();
            return false;
        }
    }
    Serial.println("device data success");
    g.setColor(TFT_GREEN);
    g.print("---", RIGHT, rows[6]);

    g.print("Sending ack..", LEFT, rows[7]);
    Serial.println("sending ack block");
    if (!KWP_send_ack())
    {
        Serial.println("send ack error");
        g.setColor(TFT_RED);
        g.print("failed", RIGHT, rows[7]);
        g.setColor(font_color);
        reset();
        return false;
    }
    connected = true;

    // Initialize simulation state
    sim_state.start_ms = millis();
    sim_state.odometer_km = 0;
    sim_state.fuel_tenth_L = 550; // 55 liters full

    g.setColor(TFT_YELLOW);
    g.print("connected", RIGHT, rows[7]);
    g.setColor(font_color);
    draw_line_on_row(8);
    draw_line_on_row(12);
    g.setColor(TFT_GREEN);
    Serial.println("------------------------------------------");
    Serial.println("|               connected                |");
    Serial.println("|----------------------------------------|");
    Serial.print("|keep alive by sending ack within ");
    Serial.print(TIMEOUT);
    Serial.println(" ms |");
    Serial.println("|----------------------------------------|");
    return true;
}
