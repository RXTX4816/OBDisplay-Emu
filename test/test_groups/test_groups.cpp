// Group reading tests: block framing for every ECU and group, the static table
// groups, and the live values the dynamic groups encode.
// Run on the host and on the simulated ATmega2560.

#include "../test_clock.h"
#include "../unity_runner.h"

#include "ecu.h"
#include "groups.h"
#include "kwp_defs.h"

static const uint8_t kCounter = 7;
static uint8_t blk[16];
static uint8_t blkLen;

void setUp()
{
    startSim(1000);
}

void tearDown() {}

static void useEcu(uint8_t addr)
{
    for (uint8_t i = 0; i < ECU_COUNT; ++i)
    {
        if (pgm_read_byte(&ECU_TABLE[i].address) == addr)
        {
            current_addr = addr;
            load_ecu_def(&ECU_TABLE[i], current_ecu);
            return;
        }
    }
    TEST_FAIL_MESSAGE("address not in ECU_TABLE");
}

static void readGroup(uint8_t group)
{
    blkLen = build_group_reading(group, kCounter, blk);
}

// Field n (0..3) of the last group reading: type, A, B.
static uint8_t fk(uint8_t n)
{
    return blk[3 + n * 3];
}
static uint8_t fa(uint8_t n)
{
    return blk[4 + n * 3];
}
static uint8_t fb(uint8_t n)
{
    return blk[5 + n * 3];
}

// Decoders from the blafusel.de type table (see ecu.cpp).
static uint16_t rpmOf(uint8_t n) // K1: 0.2 * A * B
{
    return (uint16_t)((uint32_t)fa(n) * fb(n) / 5);
}
static uint16_t kmhOf(uint8_t n) // K7: 0.01 * A * B
{
    return (uint16_t)((uint16_t)fa(n) * fb(n) / 100);
}
static int16_t celsiusOf(uint8_t n) // K5: A * (B - 100) * 0.1
{
    return (int16_t)((int16_t)fa(n) * ((int16_t)fb(n) - 100) / 10);
}

void test_group_zero_is_refused()
{
    useEcu(0x17);
    readGroup(0);
    TEST_ASSERT_EQUAL_UINT8(4, blkLen);
    const uint8_t expected[4] = {0x03, kCounter, KWP_REFUSE, 0x03};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, blk, 4);
}

void test_every_group_of_every_ecu_is_a_well_formed_block()
{
    static const uint32_t times[] = {0, 16000, 84000};
    for (uint8_t e = 0; e < ECU_COUNT; ++e)
    {
        useEcu(pgm_read_byte(&ECU_TABLE[e].address));
        for (uint8_t ti = 0; ti < 3; ++ti)
        {
            for (uint16_t g = 1; g <= 255; ++g)
            {
                at(times[ti]);
                readGroup((uint8_t)g);
                TEST_ASSERT_EQUAL_UINT8(16, blkLen);
                TEST_ASSERT_EQUAL_HEX8(0x0F, blk[0]);
                TEST_ASSERT_EQUAL_UINT8(kCounter, blk[1]);
                TEST_ASSERT_EQUAL_HEX8(KWP_RECEIVE_GROUP_READING, blk[2]);
                TEST_ASSERT_EQUAL_HEX8(0x03, blk[15]);
            }
        }
    }
}

// Groups with live values: everything else comes straight from the table.
static bool isDynamic(uint8_t addr, uint8_t group)
{
    return (addr == 0x03 && group == 1) || (addr == 0x08 && group == 1) ||
           (addr == 0x46 && group == 9);
}

void test_static_groups_come_from_the_table()
{
    for (uint8_t e = 0; e < ECU_COUNT; ++e)
    {
        uint8_t addr = pgm_read_byte(&ECU_TABLE[e].address);
        if (addr == 0x01 || addr == 0x17) // live fields patched into most groups
            continue;
        useEcu(addr);
        for (uint8_t g = 1; g <= current_ecu.num_groups; ++g)
        {
            if (isDynamic(addr, g))
                continue;
            readGroup(g);
            for (uint8_t f = 0; f < 4; ++f)
            {
                TEST_ASSERT_EQUAL_HEX8(current_ecu.groups[g - 1][f][0], fk(f));
                TEST_ASSERT_EQUAL_HEX8(current_ecu.groups[g - 1][f][1], fa(f));
                TEST_ASSERT_EQUAL_HEX8(current_ecu.groups[g - 1][f][2], fb(f));
            }
        }
    }
}

void test_groups_past_the_table_are_empty()
{
    static const uint8_t zero[12] = {0};
    useEcu(0x46);
    readGroup(20);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(zero, blk + 3, 12);
    useEcu(0x19); // defines no groups
    readGroup(1);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(zero, blk + 3, 12);
}

void test_cluster_group1_speed_and_rpm()
{
    useEcu(0x17);
    at(84000);
    readGroup(1);
    TEST_ASSERT_EQUAL_HEX8(0x07, fk(0));
    TEST_ASSERT_EQUAL_UINT16(120, kmhOf(0));
    TEST_ASSERT_EQUAL_HEX8(0x01, fk(1));
    TEST_ASSERT_UINT16_WITHIN(32, get_simulated_rpm(), rpmOf(1));
}

void test_cluster_oil_pressure_fault_window()
{
    useEcu(0x17);
    // Normal 31; 222 for 3 s every 15 s, starting after the first 15 s.
    static const uint32_t t[] = {10000, 14999, 15000, 17999, 18000, 29999, 30000, 32999, 33000};
    static const uint8_t b[] = {31, 31, 222, 222, 31, 31, 222, 222, 31};
    for (uint8_t i = 0; i < 9; ++i)
    {
        at(t[i]);
        readGroup(1);
        TEST_ASSERT_EQUAL_HEX8(0x25, fk(2));
        TEST_ASSERT_EQUAL_UINT8(b[i], fb(2));
    }
}

static uint32_t odometerKm() // K36: A * 2560 + B * 10
{
    return (uint32_t)fa(0) * 2560UL + (uint32_t)fb(0) * 10UL;
}

void test_cluster_group2_starts_at_50000_km_and_full_tank()
{
    useEcu(0x17);
    at(0);
    readGroup(2);
    TEST_ASSERT_EQUAL_HEX8(0x24, fk(0));
    TEST_ASSERT_EQUAL_UINT32(50000UL, odometerKm());
    TEST_ASSERT_EQUAL_HEX8(0x04, fk(1));
    TEST_ASSERT_EQUAL_UINT8(127 + 55, fb(1)); // K4 with A=100: |B - 127| litres
    TEST_ASSERT_EQUAL_INT16(20, celsiusOf(3)); // ambient
}

void test_cluster_group2_integrates_distance_and_fuel()
{
    useEcu(0x17);
    for (uint32_t t = 0; t <= 130000; t += 1000)
    {
        at(t);
        readGroup(2);
    }
    // One drive cycle averages ~55 km/h over 130 s: about 2 km, a fraction of a litre.
    TEST_ASSERT_TRUE(sim_state.distance_km > 1.5f && sim_state.distance_km < 2.5f);
    TEST_ASSERT_TRUE(sim_state.fuel_L < 55.0f && sim_state.fuel_L > 54.5f);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)(127.0f + sim_state.fuel_L), fb(1));
}

void test_cluster_group2_skips_gaps_over_10_s()
{
    useEcu(0x17);
    at(80000);
    readGroup(2); // > 10 s since connect: treated as a fresh start
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sim_state.distance_km);
    at(81000);
    readGroup(2);
    float d = sim_state.distance_km;
    TEST_ASSERT_TRUE(d > 0.0f);
    at(95000); // 14 s gap
    readGroup(2);
    TEST_ASSERT_EQUAL_FLOAT(d, sim_state.distance_km);
}

void test_cluster_group3_temperatures()
{
    useEcu(0x17);
    at(0);
    readGroup(3);
    TEST_ASSERT_EQUAL_INT16(20, celsiusOf(0)); // coolant
    TEST_ASSERT_EQUAL_INT16(5, celsiusOf(2));  // oil
    at(60000);
    readGroup(3);
    TEST_ASSERT_EQUAL_INT16(90, celsiusOf(0));
    TEST_ASSERT_EQUAL_INT16(75, celsiusOf(2));
    TEST_ASSERT_EQUAL_UINT8(get_simulated_oil_level(), fb(1));
}

void test_engine_group1_idle_rpm()
{
    useEcu(0x01);
    at(0);
    readGroup(1);
    TEST_ASSERT_EQUAL_HEX8(0x01, fk(0));
    TEST_ASSERT_EQUAL_UINT16(800, rpmOf(0));
    TEST_ASSERT_EQUAL_INT16(17, celsiusOf(1)); // intake air
}

void test_engine_group4_battery_and_coolant()
{
    useEcu(0x01);
    at(0); // standing: key-on voltage
    readGroup(4);
    TEST_ASSERT_EQUAL_UINT8(120, blk[8]); // 12.0 V
    TEST_ASSERT_EQUAL_INT16(20, celsiusOf(2));
    at(84000); // driving: charging voltage, warm engine
    readGroup(4);
    TEST_ASSERT_EQUAL_UINT8(142, blk[8]); // 14.2 V
    TEST_ASSERT_EQUAL_INT16(90, celsiusOf(2));
}

void test_engine_groups_carry_live_rpm_and_speed()
{
    useEcu(0x01);
    at(84000);
    uint16_t rpm = get_simulated_rpm();
    for (uint8_t g = 1; g <= 23; ++g)
    {
        readGroup(g);
        for (uint8_t f = 0; f < 4; ++f)
        {
            if (fk(f) == 0x01)
            {
                TEST_ASSERT_EQUAL_UINT8(160, fa(f));
                TEST_ASSERT_EQUAL_UINT8((uint8_t)(rpm / 32), fb(f));
            }
            else if (fk(f) == 0x07)
            {
                TEST_ASSERT_EQUAL_UINT16(120, kmhOf(f));
            }
        }
    }
}

void test_abs_wheel_speeds_follow_the_car()
{
    useEcu(0x03);
    at(84000);
    readGroup(1);
    for (uint8_t f = 0; f < 4; ++f)
    {
        TEST_ASSERT_EQUAL_HEX8(0x07, fk(f));
        TEST_ASSERT_EQUAL_UINT16(120, kmhOf(f));
    }
}

void runTests()
{
    RUN_TEST(test_group_zero_is_refused);
    RUN_TEST(test_every_group_of_every_ecu_is_a_well_formed_block);
    RUN_TEST(test_static_groups_come_from_the_table);
    RUN_TEST(test_groups_past_the_table_are_empty);
    RUN_TEST(test_cluster_group1_speed_and_rpm);
    RUN_TEST(test_cluster_oil_pressure_fault_window);
    RUN_TEST(test_cluster_group2_starts_at_50000_km_and_full_tank);
    RUN_TEST(test_cluster_group2_integrates_distance_and_fuel);
    RUN_TEST(test_cluster_group2_skips_gaps_over_10_s);
    RUN_TEST(test_cluster_group3_temperatures);
    RUN_TEST(test_engine_group1_idle_rpm);
    RUN_TEST(test_engine_group4_battery_and_coolant);
    RUN_TEST(test_engine_groups_carry_live_rpm_and_speed);
    RUN_TEST(test_abs_wheel_speeds_follow_the_car);
}

UNITY_SUITE_MAIN()
