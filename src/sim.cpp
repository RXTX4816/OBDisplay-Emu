#include "sim.h"

#include <math.h>

// VW 02J gearbox — Golf 4 1.6 16V, 195/65 R15 (circumference = 1.992 m)
// km/h per RPM = (0.001992 * 60) / (gear_ratio * 4.238)
static const float KMH_PER_RPM[6] = {
    0.0f,      // index 0 unused
    0.007468f, // gear 1  (ratio 3.778)
    0.013314f, // gear 2  (ratio 2.118)
    0.020737f, // gear 3  (ratio 1.360)
    0.029045f, // gear 4  (ratio 0.971)
    0.037300f, // gear 5  (ratio 0.756)
};
// Upshift speed thresholds (km/h): index i = shift from gear i+1 → i+2
static const float UPSHIFT_KMH[4] = {26.0f, 48.0f, 70.0f, 90.0f};

// Drive cycle: paired (time_s, speed_kmh) waypoints, linearly interpolated.
// Narrative: idle → accel through gears → cruise at 120 → step-down decel → idle → repeat.
static const uint8_t DRIVE_CYCLE_T[] = {0,  3,  8,  12, 18, 24, 28,  34,  40,  46,  52,  56,  62,
                                        68, 72, 78, 84, 90, 96, 100, 106, 112, 116, 120, 125, 130};
static const uint8_t DRIVE_CYCLE_V[] = {0,  0,   30,  30,  50,  50, 35, 35, 60, 60, 90, 90, 70,
                                        70, 100, 100, 120, 120, 90, 90, 60, 60, 30, 30, 0,  0};
#define DRIVE_CYCLE_POINTS 26
static const uint32_t DRIVE_CYCLE_PERIOD_MS = 130000UL;

SimState sim_state;

// Time-based simulation helpers
float get_simulated_speed_kmh()
{
    unsigned long elapsed_ms = sim_millis() - sim_state.start_ms;
    uint32_t t_ms = (uint32_t)(elapsed_ms % DRIVE_CYCLE_PERIOD_MS);
    uint16_t t_s_whole = (uint16_t)(t_ms / 1000UL);
    float frac = (float)(t_ms % 1000UL) * 0.001f;

    uint8_t i = 0;
    for (uint8_t k = 0; k < DRIVE_CYCLE_POINTS - 1; k++)
    {
        if (DRIVE_CYCLE_T[k + 1] <= t_s_whole)
            i = k + 1;
        else
            break;
    }
    uint8_t next = (i + 1 < DRIVE_CYCLE_POINTS) ? i + 1 : i;
    if (next == i)
        return (float)DRIVE_CYCLE_V[i];

    float dt_seg = (float)(DRIVE_CYCLE_T[next] - DRIVE_CYCLE_T[i]);
    float t_in = (float)(t_s_whole - DRIVE_CYCLE_T[i]) + frac;
    float alpha = t_in / dt_seg;
    return (float)DRIVE_CYCLE_V[i] +
           alpha * (float)((int)DRIVE_CYCLE_V[next] - (int)DRIVE_CYCLE_V[i]);
}

uint8_t get_current_gear(float speed_kmh)
{
    uint8_t gear = 1;
    for (uint8_t gi = 0; gi < 4; gi++)
    {
        if (speed_kmh >= UPSHIFT_KMH[gi])
            gear = gi + 2;
        else
            break;
    }
    return gear;
}

int8_t get_simulated_coolant_temp()
{
    unsigned long elapsed_s = (sim_millis() - sim_state.start_ms) / 1000;
    if (elapsed_s < 30)
        return (int8_t)(20 + elapsed_s * 2.33f); // 20..90 over 30s
    return 90;
}

int8_t get_simulated_oil_temp()
{
    int8_t coolant = get_simulated_coolant_temp();
    return (int8_t)(coolant - 15); // lags coolant by ~15°C
}

uint8_t get_simulated_oil_level()
{
    // Oscillates 0–255 with a ~10 s period; 127 = mid-level
    float phase =
        (float)(sim_millis() - sim_state.start_ms) / 5000.0f; // π rad/5 s → 10 s full cycle
    return (uint8_t)(127.5f + 100.0f * sinf(phase));
}

uint16_t get_simulated_rpm()
{
    float speed = get_simulated_speed_kmh();
    if (speed < 2.0f)
        return 800;
    uint8_t gear = get_current_gear(speed);
    float rpm_f = speed / KMH_PER_RPM[gear];
    if (rpm_f < 800.0f)
        rpm_f = 800.0f;
    if (rpm_f > 6500.0f)
        rpm_f = 6500.0f;
    return (uint16_t)rpm_f;
}

uint8_t get_simulated_engine_load()
{
    uint16_t rpm = get_simulated_rpm();
    if (rpm <= 800)
        return 0;
    return (uint8_t)((float)(rpm - 800) / (6500.0f - 800.0f) * 100.0f);
}

// Exhaust gas temp in °C: ~100°C cold, ~520°C at full warm (tracks coolant *6).
// Encoded with K5 A=40: B = temp/4 + 100.  Max: 40*(255-100)*0.1 = 620°C.
uint8_t get_simulated_exhaust_temp_b()
{
    int8_t coolant = get_simulated_coolant_temp();
    int exhaust_c = 100 + (coolant - 20) * 6; // 100°C cold, 520°C warm
    return (uint8_t)(exhaust_c / 4 + 100);
}

// O2 sensor voltage B value for K6 A=50 (0.001*50*B V).
// Cold: stable 0.45V; warm: oscillates 0.1-0.9V at ~2 Hz.
uint8_t get_simulated_o2_voltage_b()
{
    int8_t coolant = get_simulated_coolant_temp();
    float v;
    if (coolant < 50)
        v = 0.45f;
    else
        v = 0.5f + 0.4f * sinf((float)sim_millis() / 250.0f);
    return (uint8_t)(v * 20.0f); // B = V / (0.001*50) = V*20
}
