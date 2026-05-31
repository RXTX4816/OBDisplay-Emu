#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <Arduino.h>

struct SchedulerState
{
    unsigned long display_elapsed_us; // Display work done since last KWP_receive_block call
    uint8_t render_line_idx;          // Next log line to check during render pass
    bool rendering_in_progress;       // A render pass is active
};

SchedulerState scheduler = {0};

inline void scheduler_init()
{
    scheduler = {0};
}

// Called by push_status — starts a render pass from the top
inline void scheduler_mark_display_dirty()
{
    scheduler.render_line_idx = 0;
    scheduler.rendering_in_progress = true;
}

// True when a KWP byte has arrived — display work must yield immediately
inline bool scheduler_kwp_pending()
{
    return Serial1.available() > 0;
}

#endif
