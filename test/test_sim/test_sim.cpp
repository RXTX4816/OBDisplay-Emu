// Simulation tests: drive cycle, gearbox, engine and temperature models.
// Run on the host and on the simulated ATmega2560 (avr-libc float math).

#include "../test_clock.h"
#include "../unity_runner.h"

void setUp()
{
    startSim(1000);
}

void tearDown() {}

static float speedAt(uint32_t ms)
{
    at(ms);
    return get_simulated_speed_kmh();
}

void test_speed_hits_drive_cycle_waypoints()
{
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, speedAt(0));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, speedAt(12000));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, speedAt(20000));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 120.0f, speedAt(84000));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, speedAt(127000));
}

void test_speed_interpolates_between_waypoints()
{
    // 3 s: 0 km/h, 8 s: 30 km/h
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 15.0f, speedAt(5500));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 6.0f, speedAt(4000));
}

void test_drive_cycle_repeats_every_130_s()
{
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, speedAt(130000));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, speedAt(130000 + 12000));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 120.0f, speedAt(10UL * 130000 + 84000));
}

void test_speed_stays_within_cycle_range()
{
    for (uint32_t t = 0; t < 130000; t += 50)
    {
        float v = speedAt(t);
        TEST_ASSERT_TRUE(v >= 0.0f && v <= 120.0f);
    }
}

void test_gear_upshift_points()
{
    TEST_ASSERT_EQUAL_UINT8(1, get_current_gear(0.0f));
    TEST_ASSERT_EQUAL_UINT8(1, get_current_gear(25.9f));
    TEST_ASSERT_EQUAL_UINT8(2, get_current_gear(26.0f));
    TEST_ASSERT_EQUAL_UINT8(3, get_current_gear(48.0f));
    TEST_ASSERT_EQUAL_UINT8(4, get_current_gear(70.0f));
    TEST_ASSERT_EQUAL_UINT8(5, get_current_gear(90.0f));
    TEST_ASSERT_EQUAL_UINT8(5, get_current_gear(200.0f));
}

void test_rpm_idles_when_standing()
{
    at(0);
    TEST_ASSERT_EQUAL_UINT16(800, get_simulated_rpm());
    at(127000);
    TEST_ASSERT_EQUAL_UINT16(800, get_simulated_rpm());
}

void test_rpm_at_120_kmh_in_fifth_gear()
{
    at(84000);
    // 120 km/h / 0.0373 km/h per rpm = 3217 rpm
    TEST_ASSERT_UINT16_WITHIN(1, 3217, get_simulated_rpm());
}

void test_rpm_and_load_stay_in_range_over_a_cycle()
{
    for (uint32_t t = 0; t < 130000; t += 50)
    {
        at(t);
        uint16_t rpm = get_simulated_rpm();
        TEST_ASSERT_TRUE(rpm >= 800 && rpm <= 6500);
        TEST_ASSERT_TRUE(get_simulated_engine_load() <= 100);
    }
}

void test_load_is_zero_at_idle_and_follows_rpm()
{
    at(0);
    TEST_ASSERT_EQUAL_UINT8(0, get_simulated_engine_load());
    at(84000);
    // (3217 - 800) / 5700 * 100 = 42 %
    TEST_ASSERT_UINT8_WITHIN(1, 42, get_simulated_engine_load());
}

void test_coolant_warms_up_in_30_s_and_holds_90()
{
    at(0);
    TEST_ASSERT_EQUAL_INT8(20, get_simulated_coolant_temp());
    at(10000);
    TEST_ASSERT_EQUAL_INT8(43, get_simulated_coolant_temp());
    at(30000);
    TEST_ASSERT_EQUAL_INT8(90, get_simulated_coolant_temp());
    at(3600000UL);
    TEST_ASSERT_EQUAL_INT8(90, get_simulated_coolant_temp());

    int8_t prev = 0;
    for (uint32_t t = 0; t < 40000; t += 100)
    {
        at(t);
        int8_t c = get_simulated_coolant_temp();
        TEST_ASSERT_TRUE(c >= prev);
        prev = c;
    }
}

void test_oil_temp_lags_coolant_by_15()
{
    for (uint32_t t = 0; t < 40000; t += 500)
    {
        at(t);
        TEST_ASSERT_EQUAL_INT8(get_simulated_coolant_temp() - 15, get_simulated_oil_temp());
    }
}

void test_oil_level_oscillates_around_mid_scale()
{
    at(0);
    TEST_ASSERT_EQUAL_UINT8(127, get_simulated_oil_level());
    uint8_t lo = 255, hi = 0;
    for (uint32_t t = 0; t < 40000; t += 50)
    {
        at(t);
        uint8_t v = get_simulated_oil_level();
        if (v < lo)
            lo = v;
        if (v > hi)
            hi = v;
    }
    TEST_ASSERT_UINT8_WITHIN(1, 27, lo);
    TEST_ASSERT_UINT8_WITHIN(1, 227, hi);
}

void test_exhaust_temp_tracks_coolant()
{
    at(0); // coolant 20 °C → exhaust 100 °C → B = 100/4 + 100
    TEST_ASSERT_EQUAL_UINT8(125, get_simulated_exhaust_temp_b());
    at(60000); // coolant 90 °C → exhaust 520 °C
    TEST_ASSERT_EQUAL_UINT8(230, get_simulated_exhaust_temp_b());
}

void test_o2_voltage_steady_cold_and_swinging_warm()
{
    at(0); // cold: 0.45 V → B = V * 20
    TEST_ASSERT_EQUAL_UINT8(9, get_simulated_o2_voltage_b());

    uint8_t lo = 255, hi = 0;
    for (uint32_t t = 60000; t < 64000; t += 10)
    {
        at(t);
        uint8_t b = get_simulated_o2_voltage_b();
        if (b < lo)
            lo = b;
        if (b > hi)
            hi = b;
    }
    // 0.1 .. 0.9 V
    TEST_ASSERT_UINT8_WITHIN(1, 2, lo);
    TEST_ASSERT_UINT8_WITHIN(1, 18, hi);
}

void runTests()
{
    RUN_TEST(test_speed_hits_drive_cycle_waypoints);
    RUN_TEST(test_speed_interpolates_between_waypoints);
    RUN_TEST(test_drive_cycle_repeats_every_130_s);
    RUN_TEST(test_speed_stays_within_cycle_range);
    RUN_TEST(test_gear_upshift_points);
    RUN_TEST(test_rpm_idles_when_standing);
    RUN_TEST(test_rpm_at_120_kmh_in_fifth_gear);
    RUN_TEST(test_rpm_and_load_stay_in_range_over_a_cycle);
    RUN_TEST(test_load_is_zero_at_idle_and_follows_rpm);
    RUN_TEST(test_coolant_warms_up_in_30_s_and_holds_90);
    RUN_TEST(test_oil_temp_lags_coolant_by_15);
    RUN_TEST(test_oil_level_oscillates_around_mid_scale);
    RUN_TEST(test_exhaust_temp_tracks_coolant);
    RUN_TEST(test_o2_voltage_steady_cold_and_swinging_warm);
}

UNITY_SUITE_MAIN()
