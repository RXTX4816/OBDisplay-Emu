# List of all groups of ECUs

## 0x01 Engine

Group 0: 3 digit numbers for 1-10. Real values: 1=148, 2=203, 3=000, 4=126, 5=040, 6=127, 7=143, 8=128 i have no clue what they mean, but i guess they are values or something, not sure.. maybe measurement blocks - since in vcds switch to meas. blocks it only shows group 0 as available

Grp1: 0 /min RPM, 17.0°C Temperature oil, 0.0% Lambda, 10110010 Binary Bits (idk, when clicking on readiness in vcds it shows 10100101 readiness status: exhaust gas recirculation failed or incomplete, secondary air injection passed, oxygen sensor heating passed, evaporative emissions failed or incomplete, oxygen sensors failed or incomplete, catalyst heating passed, air conditioning passed, catalytic converter(s) failed or incomplete make sure to implement this all right wherever it is)

Grp2: 0/min RPM, 0.0% Load, 0.0 ms Time Corr., 1012.0 mbar Absolute Pres.

Grp3: 0/min RPM, 1016.0 mbar Absolute Pres., 5.5° T.B. Angle, 0.0° Steering Angle

Grp4: 0/min RPM, 11.70V Voltage, 17.0°C Temperature, 14.0°C Tempreature

Grp5: 0/min RPM, 0.0% Load, 0.0km/h Speed, Part Throttle (idk what this is)

Grp6: 0/min RPM, 0.0% Load, 14.0°C Temperature, -1.0% Lambda

Grp 7 to Grp 9 no values shown in vcds

Grp 10: 0/min RPM, 0.0% Load, 6.0% Load, 0.0° Steering angle

Grp 11: 0/min RPM, 17.0°C Temp, 14.0°C Temp, 0.0° Steering angle

Grp 12-13 empty

Grp 14: 0/min RPM, 0.0% load, 0.0 (no units), Enabled (idk what this means)

Grp 15: 0.0 (no units), 0.0 (no units),0.0 (no units), Enabled (idk)

Grp 16: 0.0 (no units), EMPTY, EMPTY, Enabled (idk)

Grp 17: Empty

Grp 18: 0/min RPM, 0/min RPM, 0.0% Lambda, 0.0% Lambda

Grp 19: Empty

Grp 20: 0.0° Steering angle, 0.0° Steering angle, 0.0° Steering angle, 0.0° Steering angle

Grp 21: Empty

Grp 22: 0/min rpm, 0.0% Load, 0.0° Steering angle, 0.0° Steering angle

Grp 23: 0/min rpm, 0.0% Load, 0.0° Steering angle, 0.0° Steering angle

## 0x03 ABS

Grp0: not available

Grp1: 0.0kmh speed, 0.0kmh speed, 0.0kmh speed, 0.0kmh speed, 

Grp2: 255.0 kmh speed, 255.0 kmh speed, 255.0 kmh speed, 255.0 kmh speed, 

Grp3: Not Oper., Not Oper, N/A, N/A (Stands like that there)

Grp 4: 0.00° Steering Angle, 0.31 m/s² Lateral Accel., -0.18°/s Turn rate, N/A

Grp 5: -1.27 bar Pressure, 0.42 bar Pressure, N/A, N/A

Every group above it: Not available

## 0x08 HVAC 3B1-907-044.LBL

Grp0: not available

Grp1: 9.0 A/C clutch sw-off cond. 1-12, 0.0 Eng. speed recog (0=no/1=yes), 0.0kmh road speed, 121.0 standing time(0-240min.)

Grp2: 42.0 Mesaured value, 42.0 Specified value, 219.0 Position:air supply cooled, 42.0 Position: air supply heated

Grp3: 221.0 Mesaured value, 221.0 specified value, 221.0 position: air flow to panel, 40.0 position: air flow to footwell

Grp4: 223.0 Mesaured value, 223.0 specified value, 223.0 position: air flow to footwell, 39.0 position: air flow to defroster

Grp5: 237.0 Mesaured value, 234.0 specified value, 234.0 position: fresh air, 30.0 position: recirculating

Grp6: 0.0°C Temp: displayed on panel, 7.0°C Temp. air intake duct, 0.0°C Outside air temp, 0.0% Sun photo sensor (0-120%)

Grp7: 0.0 Outlet temp. panel (G191), 5.0°C Outlet temp.floor(G192), 3.0°C Panel temp. near LCD (G56), 0.0 n/a

Grp8: 0.00V Specif. Voltage air blower, 0.28V mesasured voltage air blower, 12.18V Mesaured Voltage A/C clutch, 0.0 (no units)

Groups above: not available

## 0x17 Instruments 1J0-920-XX0.LBL

Grp0: not available,

Grp1: 0.0kmh speed, 0/min rpm, Oil Pr. 2<min Oil pressure 0.9 bar (is probably just some number value like 0 or 1 or even 2 but not sure), 21:50 time

Grp2: 272727 km Odometer, 23.0L fuel level(l), 93 ohms Fuel sender resistance, 0.0°C Ambient temperature

Grp3: 12.0°C Coolant Temp, OK oil Level (Ok/n.Ok) (probably is just a number like 0 or 1), 11.0°C oil temp, N/A (N/A)

Groups above it not available

## 0x19 CAN Gateway 6N0-909-901.LBL

 Grp0: not available
 No groups available in VCDS, so probably unused, but we can have it just dont send any values

 ## 0x46 Central Conv. 1J0-959-799.LBL

 Grp0: not available

 Grp1: OFF RearWinLock, Not Oper. DD lock switch, Still DD Window Motor, N/A

 Grp2: Not Oper. DD WindowSw, Not Oper. Pass Window Switch, Not Oper. LR Window Switch, Not Oper. RR Window Switch

 Grp3(drivers door switches): Not Oper. DD key switch, 0 1 DD Latch/Protection, Unlocked DD latch feedback, Not safe DD C/L feedback

 Grp4(mirrors): Not Oper. DD Mirror Sw U/D, Not Oper. DD mirror Sw L/R, Not installed DD folding mirror, N/A Empty

 Grp5(Passenger door switches): Not Oper. Pass window switch, Not Oper.Pass Lock switch, Not installed Pass folding mirror, N/A

 Grp6(passneger door): Not Oper. Pass key switch, 0 1 Pass latch/protect, Unlocked Pass latch fdback, Not safe Pass C/L feedback

 Grp7(RR door): Not Oper. RR Window sw, 0 1 RR latch/protect, Unlocked RR latch feedback, Not safe RR C/L feedback

 Grp8(LR door): Not Oper. LR Window sw, 0 1 LR latch/protect, Unlocked LR latch feedback, Not safe LR C/L feedback

 Grp9(signals): 0.0% inst light sig, 0.0kmh car speed(G22), 0 0 0 0 key remote sig, Not installed Interior mon

 Grp10(signals): Activated S contact, OFF mirror heat, Not oper. trunk lock, Term 15 on T15 on/off

 Grp11(signals): Closed hoodRadio sw, Closed trunk latch sw, Yes sunroof sync, N/A C/L temp sw

 Grp12: Bus OK CAN Bus, Driver Pass. opt fr equip, R/Left + Rear/Right opt rr equip, EMPTYVALUE opt equip

 Grp13(remotes): no Value permament code, no value code in range, no value algorithm, 0 key number(pressremote)

 Grp14(Central convenience module): 12.32V Terminal 30, Not Oper. Rear unlatch sig, Not installed interior mon, 11 11 1 Thermo protect DS/PS/RL/RR

 Grp15(alarm sources): 16 last (see label file), 4 2nd (see label file), 128 3rd (see label file), 128 4th (see label file)

 Grp16(auto locks): Not installed Imob key recogn, Not oper. auto int lock sw, Closed rear latch detent, N/A

 Groups above it: Not available





