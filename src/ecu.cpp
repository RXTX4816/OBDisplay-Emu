#include "ecu.h"

#include <string.h>

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
    for (uint8_t i = 0; i < 8; i++)
        for (uint8_t j = 0; j < 3; j++)
            ecu_ram.faults[i][j] = pgm_read_byte(&ecu_progmem->faults[i][j]);
    for (uint8_t i = 0; i < 23; i++)
        for (uint8_t j = 0; j < 4; j++)
            for (uint8_t k = 0; k < 3; k++)
                ecu_ram.groups[i][j][k] = pgm_read_byte(&ecu_progmem->groups[i][j][k]);
}

// Measurement value encoding — blafusel.de KWP1281 type table (hex = decimal):
// 0x01=1:  rpm        = 0.2 * A * B              (A=160, B=rpm/32; max 8160 RPM)
// 0x02=2:  %          = A * 0.002 * B            (throttle/lambda; A=10, B=val)
// 0x04=4:  fuel/ATDC  = abs(B-127) * 0.01 * A   (K4; A=100 → |B-127| liters or degrees)
// 0x05=5:  °C         = A * (B-100) * 0.1        (A=10, B=°C+100)
// 0x06=6:  V          = 0.001 * A * B            (A=100, B=V*10)
// 0x07=7:  km/h       = 0.01 * A * B             (A=100, B=km/h)
// 0x08=8:  unitless   = 0.1 * A * B              (counter/position; A=10)
// 0x09=9:  °(signed)  = (B-127) * 0.02 * A       (steering angle; A=50, B=127→0°)
// 0x0A=10: COLD/WARM  = B==0 → "COLD" else "WARM"
// 0x0C=12: Ohm        = 0.001 * A * B            (A=100, B=val)
// 0x10=16: bits       = B as 8-bit binary string
// 0x12=18: mbar       = 0.04 * A * B             (A=100, B=mbar/4)
// 0x16=22: ms         = 0.001 * A * B            (A=10, B=ms)
// 0x1B=27: °(ign)     = abs(B-128) * 0.01 * A   (ignition timing; A=100, B=128→0°)
// 0x21=33: %          = 100 * B / A              (A=100, B=val 0-100 → displays B %)
// 0x24=36: km         = A * 2560 + B * 10        (odometer)
// 0x2C=44: h:m        = A : B                    (time display)
// 0x34=52: Nm         = B * 0.02 * A - A         (torque; A=50, B=50→0 Nm)
// 0x37=55: s          = A * B / 200              (seconds; A=200, B=val)

// cppcheck-suppress unknownMacro
const ECUDef ECU_TABLE[] PROGMEM = {
    // 0x01 Engine (Marelli 4LV) — 036 906 034AM
    // Dynamic groups 1 and 3 are overridden in KWP_send_group_reading.
    // Type encoding: all-zero group → KWP_ACKNOWLEDGE; group > num_groups → KWP_REFUSE.
    {0x01,
     9600,
     "036 906 034AM",
     "MARELLI 4LV 3290  ",
     "00031  WSC 01317  ",
     23,
     6,
     {{0x02, 0x01, 0x2F}, // 00513 Crankshaft Position Sensor (sporadic)
      {0x02, 0x19, 0x3F}, // 00537 Throttle Position Sensor 1 (static)
      {0x02, 0x31, 0x2F}, // 00561 Fuel Trim Additive Bank 1 (sporadic)
      {0x02, 0x41, 0x2F}, // 00577 Mass Air Flow Sensor (sporadic)
      {0x02, 0x9C, 0x2F}, // 00668 Supply Voltage (sporadic)
      {0x01, 0x2C, 0x3F}, // 00300 Random Misfire (static)
      {0, 0, 0},
      {0, 0, 0}},
     {
         // Grp1 (dynamic): RPM, Temp, Lambda%, Readiness bits — overridden in code
         {{0x01, 40, 0}, {0x05, 10, 117}, {0x02, 10, 0}, {0x10, 0, 0xB2}},
         // Grp2: RPM(dyn), Load(dyn), InjTiming(dyn), AbsPres(dyn) — all patched at runtime
         {{0x01, 160, 0}, {0x21, 10, 0}, {0x16, 10, 0}, {0x12, 100, 253}},
         // Grp3 (dynamic): RPM, AbsPres, TBAngle, SteerAngle — overridden in code
         {{0x01, 40, 0}, {0x12, 100, 254}, {0x21, 1, 55}, {0x09, 50, 127}},
         // Grp4: RPM=0, 11.70V, 17.0°C, 14.0°C (K5: b=T+100)
         {{0x01, 160, 0}, {0x06, 100, 117}, {0x05, 10, 117}, {0x05, 10, 114}},
         // Grp5: RPM=0, Load=0.0%, Speed=0.0km/h, PartThrottle
         {{0x01, 160, 0}, {0x21, 10, 0}, {0x07, 100, 0}, {0x0A, 0, 1}},
         // Grp6: RPM=0, Load=0.0%, 14.0°C, Lambda=-1.0% (placeholder 0) (K5: b=T+100)
         {{0x01, 160, 0}, {0x21, 10, 0}, {0x05, 10, 114}, {0x02, 10, 0}},
         // Grp7-9: empty → group reading with zero fields
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         // Grp10: RPM=0, Load=0.0%, TDAS1-EPC=6.0%, SteerAngle=0.0°
         {{0x01, 160, 0}, {0x21, 10, 0}, {0x21, 100, 6}, {0x09, 50, 127}},
         // Grp11: RPM=0, 17.0°C, 14.0°C, SteerAngle=0.0° (K5: b=T+100)
         {{0x01, 160, 0}, {0x05, 10, 117}, {0x05, 10, 114}, {0x09, 50, 127}},
         // Grp12-13: empty → ACK
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         // Grp14: RPM=0, Load=0.0%, misfire counter=0, recognition=active
         {{0x01, 160, 0}, {0x21, 10, 0}, {0x08, 10, 0}, {0x0A, 0, 1}},
         // Grp15: misfire cyl1=0, cyl2=0, cyl3=0, recognition=active
         {{0x08, 10, 0}, {0x08, 10, 0}, {0x08, 10, 0}, {0x0A, 0, 1}},
         // Grp16: misfire cyl4=0, empty, empty, recognition=active
         {{0x08, 10, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x0A, 0, 1}},
         // Grp17: empty → ACK
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         // Grp18: RPM=0, RPM=0, Lambda=0.0%, Lambda=0.0%
         {{0x01, 160, 0}, {0x01, 160, 0}, {0x02, 10, 0}, {0x02, 10, 0}},
         // Grp19: empty → ACK
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         // Grp20: SteerAngle=0.0° × 4 (K9: (B-127)*0.02*A, A=50 → 0°)
         {{0x09, 50, 127}, {0x09, 50, 127}, {0x09, 50, 127}, {0x09, 50, 127}},
         // Grp21: empty → ACK
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         // Grp22: RPM=0, Load=0.0%, cyl1 ign.delay=0.0°, cyl2 ign.delay=0.0°
         // K27: abs(B-128)*0.01*A °; A=100, B=128 → 0°
         {{0x01, 160, 0}, {0x21, 10, 0}, {0x1B, 100, 128}, {0x1B, 100, 128}},
         // Grp23: RPM=0, Load=0.0%, cyl3 ign.delay=0.0°, cyl4 ign.delay=0.0°
         {{0x01, 160, 0}, {0x21, 10, 0}, {0x1B, 100, 128}, {0x1B, 100, 128}},
     }},
    // 0x03 ABS/ESP — 1C0 907 379
    {0x03,
     9600,
     "1C0 907 379   ",
     "ESP 20 CAN V005   ",
     "10241  WSC 01317  ",
     5,
     0,
     {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
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
     {{0x01, 0x27, 0x4E},
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0}},
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
         // Grp6: Temp-display=0.0°C, Air-intake=7.0°C, Outside=0.0°C, Sun-sensor=0.0% (K5: b=T+100)
         {{0x05, 10, 100}, {0x05, 10, 107}, {0x05, 10, 100}, {0x21, 10, 0}},
         // Grp7: OutletPanel=0.0(raw), FloorOutlet=5.0°C, PanelNearLCD=3.0°C, N/A (K5: b=T+100)
         {{0x0E, 0, 0}, {0x05, 10, 105}, {0x05, 10, 103}, {0x00, 0, 0}},
         // Grp8: Spec.V-blower=0.00V, Meas.V-blower=0.28V (A=28,B=10: 0.28V), Meas.V-A/C=12.2V
         // (closest to captured 12.18V with uint8_t B), empty
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
     {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
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
     6,
     {{0x02, 0x4C, 0x2F}, // 00588 Vehicle Speed Sensor (sporadic)
      {0x03, 0x08, 0x2F}, // 00776 Fuel Level Sensor (sporadic)
      {0x05, 0x1B, 0x3F}, // 01307 Speedometer (static)
      {0x04, 0x98, 0x2F}, // 01176 Oil Level Sensor (sporadic)
      {0x03, 0x0B, 0x2F}, // 00779 Coolant Temp Sensor (sporadic)
      {0x01, 0xBE, 0x3F}, // 00446 Oil Pressure Switch (static)
      {0, 0, 0},
      {0, 0, 0}},
     {
         // Grp1: Speed(dynamic), RPM(dynamic), OilPressureMin(k=0x25 F_B b=31=ok), Time(A=21 B=50
         // placeholder)
         {{0x07, 100, 0}, {0x01, 40, 0}, {0x25, 0, 31}, {0x0E, 21, 50}},
         // Grp2: Odometer(dynamic), FuelLevel(dynamic), FuelSenderRes=93Ohm, AmbientTemp=20°C
         // FuelLevel: K4 abs(b-127)*0.01*100=55L → B=182; AmbientTemp: K5 10*(b-100)*0.1=0°C →
         // B=100
         {{0x24, 0, 0}, {0x04, 100, 182}, {0x14, 10, 93}, {0x05, 10, 120}},
         // Grp3: CoolantTemp(dynamic), OilLevel(k=0x25 MW_B 0-255, dynamic), OilTemp(dynamic), N/A
         // K5: 10*(b-100)*0.1=T°C; 12°C → B=112, 11°C → B=111
         {{0x05, 10, 112}, {0x25, 0, 127}, {0x05, 10, 111}, {0x00, 0, 0}},
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
     {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
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
     {{0x00, 0x94, 0x3F},
      {0x00, 0x94, 0x40},
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0}},
     {
         // Grp1: ChildSafety=OFF(no=1), DDLockSw=NotOper(2), WindowMotor=Still(stop=1), N/A
         {{0x0E, 0, 1}, {0x0E, 0, 2}, {0x0E, 0, 1}, {0x00, 0, 0}},
         // Grp2: window switches — all Not Oper
         // E40 driver: auto.open/auto.close/man.open/man.close/close not operated/implausible → idx
         // 4
         // E81/E53/E55: autom.open/autom.close/man.open/man.close/not operated/implausible → idx 4
         {{0x0E, 0, 4}, {0x0E, 0, 4}, {0x0E, 0, 4}, {0x0E, 0, 4}},
         // Grp3 (driver door): DDKeySw=NotOper(2), LatchProtect=binary 0b01,
         // CLFeedback=Unlocked(1), SafeFeedback=NotSafe(1)
         {{0x0E, 0, 2}, {0x10, 0, 0x01}, {0x0E, 0, 1}, {0x0E, 0, 1}},
         // Grp4 (mirrors): MirrorUD=NotOper(4), MirrorLR=NotOper(3), Folding=NotInstalled(2), N/A
         {{0x0E, 0, 4}, {0x0E, 0, 3}, {0x0E, 0, 2}, {0x00, 0, 0}},
         // Grp5 (pass door): PassWindowSw=NotOper(4), PassLockSw=NotOper(2),
         // PassFolding=NotInstalled(2), N/A
         {{0x0E, 0, 4}, {0x0E, 0, 2}, {0x0E, 0, 2}, {0x00, 0, 0}},
         // Grp6 (pass door): PassKeySw=NotOper(2), LatchProtect=binary 0b01,
         // CLFeedback=Unlocked(1), SafeFeedback=NotSafe(1)
         {{0x0E, 0, 2}, {0x10, 0, 0x01}, {0x0E, 0, 1}, {0x0E, 0, 1}},
         // Grp7 (RR door): RRWindowSw=NotOper(4), LatchProtect=binary 0b01,
         // CLFeedback=Unlocked(1), SafeFeedback=NotSafe(1)
         {{0x0E, 0, 4}, {0x10, 0, 0x01}, {0x0E, 0, 1}, {0x0E, 0, 1}},
         // Grp8 (RL door): RLWindowSw=NotOper(4), LatchProtect=binary 0b01,
         // CLFeedback=Unlocked(1), SafeFeedback=NotSafe(1)
         {{0x0E, 0, 4}, {0x10, 0, 0x01}, {0x0E, 0, 1}, {0x0E, 0, 1}},
         // Grp9 (signals): InstLightSig=0.0%, CarSpeed=0.0km/h, KeyRemoteSig=0b00000000,
         // InteriorMon=NotInstalled(2 in yes/no/not installed)
         {{0x21, 10, 0}, {0x07, 100, 0}, {0x10, 0, 0x00}, {0x0E, 0, 2}},
         // Grp10 (signals): SContact=operated(0), MirrorHeat=off(1),
         // TrunkKeySw=not operated(2), Term15=on(0)
         {{0x0E, 0, 0}, {0x0E, 0, 1}, {0x0E, 0, 2}, {0x0E, 0, 0}},
         // Grp11 (signals): HoodSw=not operated(1)=closed, TrunkContact=closed(1),
         // SunroofReleased=yes(0), N/A
         {{0x0E, 0, 1}, {0x0E, 0, 1}, {0x0E, 0, 0}, {0x0E, 0, 0}},
         // Grp12 (CAN): BusOK(1), FrDoorModules=binary, RrDoorModules=binary, AddEquip=Memory(0)
         {{0x0E, 0, 1}, {0x10, 0, 0xFF}, {0x10, 0, 0xFF}, {0x0E, 0, 0}},
         // Grp13 (remotes): OK(0) × 3, KeyNumber=0
         {{0x0E, 0, 0}, {0x0E, 0, 0}, {0x0E, 0, 0}, {0x0E, 0, 0}},
         // Grp14 (CCM): Term30=12.3V, RearUnlatch=not operated(0),
         // InteriorMonSw=not installed(2), ThermoProtect=binary 0b00011111
         {{0x06, 100, 123}, {0x0E, 0, 0}, {0x0E, 0, 2}, {0x10, 0, 0x1F}},
         // Grp15 (alarm): Last=16, 2nd=4, 3rd=128, 4th=128
         {{0x0E, 0, 16}, {0x0E, 0, 4}, {0x0E, 0, 128}, {0x0E, 0, 128}},
         // Grp16 (auto locks): ImobKeyRecogn=not installed(2), AutoLockSw=not oper.(1),
         // RearDetent=closed(1), N/A
         {{0x0E, 0, 2}, {0x0E, 0, 1}, {0x0E, 0, 1}, {0x00, 0, 0}},
         // Grp17-23: unused padding
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
         {{0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}, {0x00, 0, 0}},
     }}};

const uint8_t ECU_COUNT = sizeof(ECU_TABLE) / sizeof(ECUDef);

/// GLOBAL STATE
ECUDef current_ecu;          // loaded from PROGMEM on connect
uint8_t current_addr = 0x00; // decoded from 5-baud init
