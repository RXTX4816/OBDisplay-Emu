#include "groups.h"

#include "ecu.h"
#include "kwp_defs.h"
#include "sim.h"

#include <math.h>
#include <string.h>

uint8_t build_group_reading(uint8_t group, uint8_t counter, uint8_t* out)
{
    // Validate group number
    if (group == 0)
    {
        // Invalid group: send REFUSE
        uint8_t refuse_buf[4] = {0x03, counter, KWP_REFUSE, 0x03};
        memcpy(out, refuse_buf, 4);
        return 4;
    }

    // Send group data from ECU definition
    uint8_t buf[16] = {0x0F, counter, KWP_RECEIVE_GROUP_READING,
                       0x00, 0x00,    0x00,
                       0x00, 0x00,    0x00,
                       0x00, 0x00,    0x00,
                       0x00, 0x00,    0x00,
                       0x03};

    // Dynamic overrides — populated before falling through to static table
    if (current_addr == 0x17 && group == 1)
    {
        // Grp1: Speed(km/h), RPM, OilPressureMin, Time(placeholder A=21 B=50)
        float speed = get_simulated_speed_kmh();
        uint16_t rpm = get_simulated_rpm();

        buf[3] = 0x07;
        buf[4] = 100;
        buf[5] = (uint8_t)speed; // km/h
        buf[6] = 0x01;
        buf[7] = 160;
        buf[8] = (uint8_t)(rpm / 32); // RPM
        // k=0x25 (37): F_B formula — stored value = b directly.
        // Normal: b=31. Fault sim: b=222 for 3 s every 15 s, starting after the first 15 s.
        buf[9] = 0x25;
        buf[10] = 0;
        {
            unsigned long elapsed_ms = sim_millis() - sim_state.start_ms;
            bool fault = elapsed_ms >= 15000UL && ((elapsed_ms - 15000UL) % 15000UL) < 3000UL;
            buf[11] = fault ? 222 : 31;
        }
        buf[12] = 0x0E;
        buf[13] = 21;
        buf[14] = 50; // time 21:50 (placeholder; verify encoding with real car)
    }
    else if (current_addr == 0x17 && group == 2)
    {
        unsigned long now_ms = sim_millis();
        float dt_sec = (float)(now_ms - sim_state.last_update_ms) * 0.001f;
        if (dt_sec > 10.0f)
            dt_sec = 0.0f; // guard: first call or stale state after reset
        sim_state.last_update_ms = now_ms;

        float speed = get_simulated_speed_kmh();

        // Odometer: integrate speed over time
        if (sim_state.distance_km < 444444.0f)
        {
            sim_state.distance_km += speed * dt_sec / 3600.0f;
            if (sim_state.distance_km > 444444.0f)
                sim_state.distance_km = 444444.0f;
        }

        // Fuel: physics-based consumption
        if (sim_state.fuel_L > 0.0f)
        {
            float fuel_rate;
            if (speed < 2.0f)
            {
                fuel_rate = 0.6f / 3600.0f; // idle ~0.6 L/h
            }
            else
            {
                uint16_t rpm = get_simulated_rpm();
                // Base 7.0 L/100km + RPM load penalty + aerodynamic penalty above 100 km/h
                float l_per_100km = 7.0f + (float)(rpm - 800) * (1.5f / 5700.0f) +
                                    (speed > 100.0f ? (speed - 100.0f) * 0.015f : 0.0f);
                fuel_rate = (speed / 100.0f) * l_per_100km / 3600.0f;
            }
            sim_state.fuel_L -= fuel_rate * dt_sec;
            if (sim_state.fuel_L < 0.0f)
                sim_state.fuel_L = 0.0f;
        }

        // Odometer: type 0x24, formula: km = A*2560 + B*10. Max ~653350 km.
        uint32_t raw_km = 50000UL + (uint32_t)sim_state.distance_km;
        uint8_t odo_a = (uint8_t)(raw_km / 2560);
        uint8_t odo_b = (uint8_t)((raw_km % 2560) / 10);

        // K4 fuel: value = A * |B-127| * 0.01; with A=100: fuel_L = |B-127|; B = 127+fuel_L
        uint8_t fuel_b = (uint8_t)(127.0f + sim_state.fuel_L);

        buf[3] = 0x24;
        buf[4] = odo_a;
        buf[5] = odo_b;
        buf[6] = 0x04;
        buf[7] = 100;
        buf[8] = fuel_b; // K4: abs(b-127)*0.01*100 = liters
        buf[9] = 0x14;
        buf[10] = 10;
        buf[11] = 93; // 93 Ohm sender resistance
        buf[12] = 0x05;
        buf[13] = 10;
        buf[14] = 120; // 20°C ambient (K5: 10*(120-100)*0.1 = 20°C)
    }
    else if (current_addr == 0x17 && group == 3)
    {
        // Grp3: CoolantTemp, OilLevel(k=0x25 MW_B 0-255), OilTemp, N/A
        // K5 formula: a*(b-100)*0.1 → T°C, so b = T + 100 (with a=10)
        int8_t coolant = get_simulated_coolant_temp();
        int8_t oil_temp = get_simulated_oil_temp();
        uint8_t oil_level = get_simulated_oil_level();

        buf[3] = 0x05;
        buf[4] = 10;
        buf[5] = (uint8_t)(coolant + 100); // coolant °C (K5: 10*(b-100)*0.1 = coolant)
        buf[6] = 0x25;
        buf[7] = 0;
        buf[8] = oil_level; // oil level 0-255 (k=0x25 MW_B: just B)
        buf[9] = 0x05;
        buf[10] = 10;
        buf[11] = (uint8_t)(oil_temp + 100); // oil temp °C (K5: 10*(b-100)*0.1 = oil_temp)
        buf[12] = 0x00;
        buf[13] = 0;
        buf[14] = 0; // N/A
    }
    else if (current_addr == 0x01 && group == 1)
    {
        // Grp1: RPM, AirTemp=17°C(static), Lambda=0.0%(static), ReadinessBits(static)
        uint16_t rpm = get_simulated_rpm();

        buf[3] = 0x01;
        buf[4] = 160;
        buf[5] = (uint8_t)(rpm / 32); // RPM
        buf[6] = 0x05;
        buf[7] = 10;
        buf[8] = 117; // 17.0°C air temp (K5: 10*(117-100)*0.1 = 17°C)
        buf[9] = 0x02;
        buf[10] = 10;
        buf[11] = 0; // 0.0% lambda
        buf[12] = 0x10;
        buf[13] = 0;
        buf[14] = 0xB2; // readiness bits 10110010
    }
    else if (current_addr == 0x01 && group == 3)
    {
        // Grp3: RPM, AbsPres(dynamic), TBAngle(dynamic), SteerAngle=0.0°
        uint16_t rpm = get_simulated_rpm();
        uint8_t load = get_simulated_engine_load();

        // MAP: 300 mbar at idle, rises to ~950 mbar at full load
        uint16_t map_mbar = 300 + (uint16_t)load * 65 / 10; // 300..950
        // TB angle: 5.5° at idle, opens with load. K9 A=55: (B-127)*1.1 deg
        float tb_deg = 5.5f + load * 0.55f; // 5.5° idle, ~60° full load
        uint8_t tb_b = (uint8_t)(127.0f + tb_deg / 1.1f);

        buf[3] = 0x01;
        buf[4] = 160;
        buf[5] = (uint8_t)(rpm / 32); // RPM
        buf[6] = 0x12;
        buf[7] = 100;
        buf[8] = (uint8_t)(map_mbar / 4); // MAP mbar (K18: 0.04*100*B)
        buf[9] = 0x09;
        buf[10] = 55;
        buf[11] = tb_b; // TB angle (K9: (B-127)*0.02*55 deg)
        buf[12] = 0x09;
        buf[13] = 50;
        buf[14] = 127; // 0.0° steering (K9: (127-127)*0.02*50)
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
        buf[9] = 0x10;
        buf[10] = 0;
        buf[11] = 0x00; // KeyRemoteSig bits
        buf[12] = 0x0E;
        buf[13] = 0;
        buf[14] = 2; // InteriorMon: not installed (idx 2 of yes/no/not installed)
    }
    else if (current_addr == 0x01 && group > 23)
    {
        // Engine ECU groups 30–125 (all [verify] from label file 036-906-034-APE)
        uint16_t rpm = get_simulated_rpm();
        uint8_t rpm_b = (uint8_t)(rpm / 32);
        int8_t coolant = get_simulated_coolant_temp();

        switch (group)
        {
            case 28: // Knock sensor test (RPM, Load, CoolantTemp, test result)
                buf[3] = 0x01;
                buf[4] = 160;
                buf[5] = rpm_b;
                buf[6] = 0x21;
                buf[7] = 100;
                buf[8] = (uint8_t)((float)(rpm - 800) / (6500.0f - 800.0f) * 100.0f); // load %
                buf[9] = 0x05;
                buf[10] = 10;
                buf[11] = (uint8_t)(coolant + 100); // coolant temp
                buf[12] = 0x0A;
                buf[13] = 0;
                buf[14] = 0; // Test OFF (COLD)
                break;
            case 30: // O2 sensor status bits: 1xx=heater,x1x=ready,xx1=lambda
                buf[3] = 0x10;
                buf[4] = 0;
                buf[5] = 0x07; // B1-S1 spec: 111
                buf[6] = 0x10;
                buf[7] = 0;
                buf[8] = 0x06; // B1-S2 spec: 110
                break;
            case 32: // Lambda self-adaptation (negative not representable → 0.0%)
                buf[3] = 0x02;
                buf[4] = 10;
                buf[5] = 0;
                buf[6] = 0x02;
                buf[7] = 10;
                buf[8] = 0;
                break;
            case 33: // Lambda control + O2 sensor voltage
                buf[3] = 0x02;
                buf[4] = 10;
                buf[5] = 0; // lambda control 0.0%
                buf[6] = 0x06;
                buf[7] = 50;
                buf[8] = get_simulated_o2_voltage_b(); // B1-S1 voltage (oscillating)
                break;
            case 34: // O2 sensor aging test (B1-S1)
                buf[3] = 0x01;
                buf[4] = 160;
                buf[5] = rpm_b;
                buf[6] = 0x05;
                buf[7] = 40;
                buf[8] = get_simulated_exhaust_temp_b(); // exhaust temp (K5 A=40)
                buf[9] = 0x37;
                buf[10] = 200;
                buf[11] = 0; // dynamic factor 0.0 s
                buf[12] = 0x0A;
                buf[13] = 0;
                buf[14] = 0; // Test OFF (COLD)
                break;
            case 36: // B1-S2 sensor readiness
                buf[3] = 0x06;
                buf[4] = 50;
                buf[5] = get_simulated_o2_voltage_b(); // B1-S2 voltage (oscillating)
                buf[6] = 0x0A;
                buf[7] = 0;
                buf[8] = 0; // Test OFF (COLD)
                break;
            case 37: // B1-S2 diagnostic
                buf[3] = 0x21;
                buf[4] = 100;
                buf[5] = (uint8_t)((float)(rpm - 800) / (6500.0f - 800.0f) * 100.0f); // load %
                buf[6] = 0x06;
                buf[7] = 50;
                buf[8] = get_simulated_o2_voltage_b(); // B1-S2 voltage (oscillating)
                // pos 3 empty
                buf[12] = 0x0A;
                buf[13] = 0;
                buf[14] = 0; // Test OFF (COLD)
                break;
            case 41: // O2 heater resistance
                buf[3] = 0x0C;
                buf[4] = 100;
                buf[5] = 50; // B1-S1: 5.0 Ohm (K12: 0.001*100*50)
                buf[6] = 0x0A;
                buf[7] = 0;
                buf[8] = 1; // heater OK (WARM)
                buf[9] = 0x0C;
                buf[10] = 100;
                buf[11] = 50; // B1-S2: 5.0 Ohm
                buf[12] = 0x0A;
                buf[13] = 0;
                buf[14] = 1; // heater OK (WARM)
                break;
            case 46: // Catalytic converter efficiency test
            {
                // Cat temp lags exhaust: ~80°C cold, ~430°C warm (coolant*5 offset)
                int8_t cat_coolant = get_simulated_coolant_temp();
                int cat_c = 80 + (cat_coolant - 20) * 5;
                uint8_t cat_b = (uint8_t)(cat_c / 4 + 100); // K5 A=40 encoding
                buf[3] = 0x01;
                buf[4] = 160;
                buf[5] = rpm_b;
                buf[6] = 0x05;
                buf[7] = 40;
                buf[8] = cat_b; // cat temp (K5 A=40)
                buf[9] = 0x21;
                buf[10] = 100;
                buf[11] = 0; // amplitude 0%
                buf[12] = 0x0A;
                buf[13] = 0;
                buf[14] = 0; // Test OFF (COLD)
                break;
            }
            case 50: // Speed regulation
                buf[3] = 0x01;
                buf[4] = 160;
                buf[5] = rpm_b;
                buf[6] = 0x01;
                buf[7] = 160;
                buf[8] = 25; // target 800 RPM (160*25*0.2)
                buf[9] = 0x0A;
                buf[10] = 0;
                buf[11] = 0; // A/C-Low (COLD)
                buf[12] = 0x0A;
                buf[13] = 0;
                buf[14] = 0; // Compr.OFF (COLD)
                break;
            case 54: // Throttle and pedal sensors
                buf[3] = 0x01;
                buf[4] = 160;
                buf[5] = rpm_b;
                buf[6] = 0x0A;
                buf[7] = 0;
                buf[8] = 1; // Part Throttle (WARM)
                buf[9] = 0x21;
                buf[10] = 100;
                buf[11] = 0; // acc pedal pos 0%
                buf[12] = 0x21;
                buf[13] = 100;
                buf[14] = 6; // TDAS1 (G187) 6%
                break;
            case 55: // Idle regulator
                buf[3] = 0x01;
                buf[4] = 160;
                buf[5] = rpm_b;
                buf[6] = 0x02;
                buf[7] = 10;
                buf[8] = 0; // idle regulator 0.0%
                buf[9] = 0x02;
                buf[10] = 10;
                buf[11] = 0; // self-adaptation 0.0%
                buf[12] = 0x10;
                buf[13] = 0;
                buf[14] = 0x00; // load status bits
                break;
            case 56: // Idle torque regulation
                buf[3] = 0x01;
                buf[4] = 160;
                buf[5] = rpm_b;
                buf[6] = 0x01;
                buf[7] = 160;
                buf[8] = 25; // target 800 RPM (160*25*0.2)
                buf[9] = 0x34;
                buf[10] = 50;
                buf[11] = 50; // idle regulator 0.0 Nm (K52: 50*0.02*50-50=0)
                buf[12] = 0x10;
                buf[13] = 0;
                buf[14] = 0x00; // load status bits
                break;
            case 60: // EPC throttle adaptation
                buf[3] = 0x21;
                buf[4] = 100;
                buf[5] = 10; // TDAS1 10% (K33: 100*10/100)
                buf[6] = 0x21;
                buf[7] = 100;
                buf[8] = 85; // TDAS2 85% (inverse)
                buf[9] = 0x08;
                buf[10] = 10;
                buf[11] = 12; // steps counter 12 (K8: 0.1*10*12=12)
                buf[12] = 0x0A;
                buf[13] = 0;
                buf[14] = 1; // ADP OK (WARM)
                break;
            case 61: // EPC system status
                buf[3] = 0x01;
                buf[4] = 160;
                buf[5] = rpm_b;
                buf[6] = 0x06;
                buf[7] = 100;
                buf[8] = (rpm > 800) ? 142 : 120; // battery: 14.2V running, 12.0V key-on
                buf[9] = 0x21;
                buf[10] = 100;
                buf[11] = 6; // TDAS1 6% (idle position)
                buf[12] = 0x10;
                buf[13] = 0;
                buf[14] = 0x00; // load status bits
                break;
            case 62: // All throttle/pedal sensors
                buf[3] = 0x21;
                buf[4] = 100;
                buf[5] = 6; // TDAS1 (G187) 6%
                buf[6] = 0x21;
                buf[7] = 100;
                buf[8] = 94; // TDAS2 (G188) 94% (inverse)
                buf[9] = 0x21;
                buf[10] = 100;
                buf[11] = 6; // throttle pos (G79) 6%
                buf[12] = 0x21;
                buf[13] = 100;
                buf[14] = 0; // acc pedal sensor 2 (G185) 0%
                break;
            case 70: // Evaporative emissions (tank ventilation)
                buf[3] = 0x21;
                buf[4] = 100;
                buf[5] = 0; // TVV opening 0%
                buf[6] = 0x02;
                buf[7] = 10;
                buf[8] = 0; // lambda diag 0.0%
                buf[9] = 0x12;
                buf[10] = 100;
                buf[11] = 253; // intake pressure 1012 mbar (K18: 0.04*100*253)
                buf[12] = 0x0A;
                buf[13] = 0;
                buf[14] = 0; // Test OFF (COLD)
                break;
            case 74: // EGR valve adaptation
                buf[3] = 0x06;
                buf[4] = 100;
                buf[5] = 3; // min pos 0.3V (spec min)
                buf[6] = 0x06;
                buf[7] = 100;
                buf[8] = 29; // max pos 2.9V (spec min)
                buf[9] = 0x06;
                buf[10] = 100;
                buf[11] = 3; // actual 0.3V
                buf[12] = 0x0A;
                buf[13] = 0;
                buf[14] = 1; // ADP OK (WARM)
                break;
            case 75: // EGR test
                buf[3] = 0x01;
                buf[4] = 160;
                buf[5] = rpm_b;
                buf[6] = 0x12;
                buf[7] = 100;
                buf[8] = 253; // intake pressure 1012 mbar (K18: 0.04*100*253)
                buf[9] = 0x12;
                buf[10] = 100;
                buf[11] = 0; // pressure diff 0.0 mbar
                buf[12] = 0x0A;
                buf[13] = 0;
                buf[14] = 0; // Test OFF (COLD)
                break;
            case 99: // OBD compatibility
                buf[3] = 0x01;
                buf[4] = 160;
                buf[5] = rpm_b;
                buf[6] = 0x05;
                buf[7] = 10;
                buf[8] = (uint8_t)(coolant + 100);
                buf[9] = 0x02;
                buf[10] = 10;
                buf[11] = 0; // O2 control 0.0%
                buf[12] = 0x0A;
                buf[13] = 0;
                buf[14] = 1; // O2 control ON (WARM)
                break;
            case 100: // OBD readiness (VCDS readiness screen reads this group)
            {
                unsigned long elapsed_s = (sim_millis() - sim_state.start_ms) / 1000;
                uint8_t h = (uint8_t)(elapsed_s / 3600 > 255 ? 255 : elapsed_s / 3600);
                uint8_t m = (uint8_t)((elapsed_s % 3600) / 60);
                buf[3] = 0x10;
                buf[4] = 0;
                buf[5] = 0xA5; // readiness bits 10100101
                buf[6] = 0x05;
                buf[7] = 10;
                buf[8] = (uint8_t)(coolant + 100); // coolant temp
                buf[9] = 0x2C;
                buf[10] = h;
                buf[11] = m; // time since start h:m (K44: A:B)
                buf[12] = 0x10;
                buf[13] = 0;
                buf[14] = 0x00; // OBD status flags
                break;
            }
            case 120: // Traction control (ASR/TCS)
                buf[3] = 0x01;
                buf[4] = 160;
                buf[5] = rpm_b;
                buf[6] = 0x01;
                buf[7] = 160;
                buf[8] = 25; // target 800 RPM (160*25*0.2)
                buf[9] = 0x01;
                buf[10] = 160;
                buf[11] = rpm_b; // actual RPM
                buf[12] = 0x0A;
                buf[13] = 0;
                buf[14] = 0; // ASR not active (COLD)
                break;
            case 122: // Transmission torque reduction
                buf[3] = 0x01;
                buf[4] = 160;
                buf[5] = rpm_b;
                buf[6] = 0x01;
                buf[7] = 160;
                buf[8] = 25; // target 800 RPM (160*25*0.2)
                buf[9] = 0x01;
                buf[10] = 160;
                buf[11] = rpm_b; // actual RPM
                buf[12] = 0x0A;
                buf[13] = 0;
                buf[14] = 0; // No torque red. (COLD)
                break;
            case 125: // CAN powertrain bus status
                buf[3] = 0x21;
                buf[4] = 10;
                buf[5] = 10; // brake electronics 1.0 (OK)
                buf[6] = 0x21;
                buf[7] = 10;
                buf[8] = 10; // transmission 1.0 (OK)
                buf[9] = 0x21;
                buf[10] = 10;
                buf[11] = 10; // instrument cluster 1.0 (OK)
                buf[12] = 0x21;
                buf[13] = 10;
                buf[14] = 10; // airbag 1.0 (OK)
                break;
            default:
                // unimplemented group: leave buf zeroed → sends empty 0xE7
                break;
        }
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

    // Post-process all 0x01 groups: make RPM, Speed, and engine Load fields dynamic.
    // Groups 1 & 3 already have correct values set above — this loop is idempotent for them.
    // Guard A==10 on K=0x21 avoids overwriting Grp3 throttle-body angle {0x21, 1, 55}.
    // Only patch groups 1-23; group >23 cases are already handled correctly above.
    if (current_addr == 0x01 && group <= 23)
    {
        uint16_t rpm = get_simulated_rpm();
        float spd = get_simulated_speed_kmh();
        uint8_t load = get_simulated_engine_load();

        for (uint8_t s = 0; s < 4; s++)
        {
            uint8_t k = 3 + s * 3;
            switch (buf[k])
            {
                case 0x01: // RPM
                    buf[k + 1] = 160;
                    buf[k + 2] = (uint8_t)(rpm / 32);
                    break;
                case 0x07: // Speed km/h
                    buf[k + 1] = 100;
                    buf[k + 2] = (uint8_t)spd;
                    break;
                case 0x21: // Load % (K33: 100*B/A). Guard A==10 skips fixed values
                           // (e.g. Grp3 TB angle A=1, Grp10 TDAS1 A=100).
                    if (buf[k + 1] == 10)
                    {
                        buf[k + 1] = 100; // K33 needs A=100 for correct display
                        buf[k + 2] = load;
                    }
                    break;
                default:
                    break;
            }
        }

        // Grp2: MAP and injection timing are dynamic.
        // Field 3 = injection timing (K22 A=20: 0.001*20*B ms, so B=ms*50).
        // Field 4 = MAP (K18 A=100: 0.04*100*B mbar, so B=mbar/4).
        if (group == 2)
        {
            uint16_t map_mbar = 300 + (uint16_t)load * 65 / 10;
            buf[14] = (uint8_t)(map_mbar / 4);
            float timing_ms = (rpm > 800) ? (2.5f + load * 0.015f) : 0.0f;
            buf[10] = 20; // A=20 for K22 to cover up to ~5ms
            buf[11] = (uint8_t)(timing_ms * 50.0f);
        }

        // Grp4 field 2 = Battery (K6 A=100: 0.001*100*B V → B=V*10).
        // Field 3 = Coolant Temp (K5, spec 80-110 deg C warm).
        if (group == 4)
        {
            buf[8] = (rpm > 800) ? 142 : 120; // 14.2V running, 12.0V key-on
            int8_t coolant = get_simulated_coolant_temp();
            buf[10] = (uint8_t)(coolant + 100);
        }

        // Grp5 field 4 = load status: COLD=idle, WARM=part throttle when moving.
        if (group == 5)
            buf[14] = (spd > 2.0f) ? 1 : 0;
    }

    memcpy(out, buf, 16);
    return 16;
}
