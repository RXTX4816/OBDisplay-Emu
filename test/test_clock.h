// Fake clock for the simulation (sim_millis() is millis() in the firmware).
// Include in exactly one file per test suite.
#pragma once

#include "sim.h"

static uint32_t testNowMs = 0;

uint32_t sim_millis()
{
    return testNowMs;
}

// Start the simulation the way connect() does, at `startMs`.
static void startSim(uint32_t startMs)
{
    testNowMs = startMs;
    sim_state.start_ms = startMs;
    sim_state.distance_km = 0.0f;
    sim_state.fuel_L = 55.0f; // full tank
    sim_state.last_update_ms = startMs;
}

// Put the clock `ms` after the simulation start.
static void at(uint32_t ms)
{
    testNowMs = (uint32_t)sim_state.start_ms + ms;
}
