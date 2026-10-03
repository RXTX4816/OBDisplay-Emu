// ECU table tests: the emulated modules, their limits and what reaches the wire.
// Run on the host and on the simulated ATmega2560 (real PROGMEM reads).

#include "../test_clock.h"
#include "../unity_runner.h"

#include "ecu.h"

#include <string.h>

void setUp() {}
void tearDown() {}

static void loadEcu(uint8_t i, ECUDef& e)
{
    memset(&e, 0, sizeof(e));
    load_ecu_def(&ECU_TABLE[i], e);
}

void test_emulates_the_expected_modules()
{
    static const uint8_t expected[] = {0x01, 0x03, 0x08, 0x15, 0x17, 0x19, 0x46};
    TEST_ASSERT_EQUAL_UINT8(sizeof(expected), ECU_COUNT);
    for (uint8_t i = 0; i < ECU_COUNT; ++i)
        TEST_ASSERT_EQUAL_HEX8(expected[i], pgm_read_byte(&ECU_TABLE[i].address));
}

void test_baud_rates()
{
    ECUDef e;
    for (uint8_t i = 0; i < ECU_COUNT; ++i)
    {
        loadEcu(i, e);
        TEST_ASSERT_EQUAL_UINT16(e.address == 0x17 ? 10400 : 9600, e.baudrate);
    }
}

void test_group_and_fault_counts_fit_their_arrays()
{
    ECUDef e;
    for (uint8_t i = 0; i < ECU_COUNT; ++i)
    {
        loadEcu(i, e);
        TEST_ASSERT_TRUE(e.num_groups <= 23);
        TEST_ASSERT_TRUE(e.num_faults <= 8);
    }
}

void test_load_copies_the_whole_definition()
{
    ECUDef loaded, raw;
    for (uint8_t i = 0; i < ECU_COUNT; ++i)
    {
        loadEcu(i, loaded);
        memset(&raw, 0, sizeof(raw));
        memcpy_P(&raw, &ECU_TABLE[i], sizeof(raw));
        TEST_ASSERT_EQUAL_MEMORY(&raw, &loaded, sizeof(raw));
    }
}

// KWP_send_devicedata sends 12 characters of each ID string, padding with
// spaces after a terminator; those 12 must be printable.
static void assertSentPartIsPrintable(const char* s)
{
    for (uint8_t i = 0; i < 12 && s[i] != '\0'; ++i)
        TEST_ASSERT_TRUE(s[i] >= 0x20 && s[i] <= 0x7E);
}

void test_id_strings_sent_are_printable()
{
    ECUDef e;
    for (uint8_t i = 0; i < ECU_COUNT; ++i)
    {
        loadEcu(i, e);
        assertSentPartIsPrintable(e.part_number);
        assertSentPartIsPrintable(e.component);
        assertSentPartIsPrintable(e.coding_wsc);
    }
}

void runTests()
{
    RUN_TEST(test_emulates_the_expected_modules);
    RUN_TEST(test_baud_rates);
    RUN_TEST(test_group_and_fault_counts_fit_their_arrays);
    RUN_TEST(test_load_copies_the_whole_definition);
    RUN_TEST(test_id_strings_sent_are_printable);
}

UNITY_SUITE_MAIN()
