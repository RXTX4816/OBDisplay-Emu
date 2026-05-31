// ARDUINO MEGA 2560 + TFT LCD SHIELD + Serial1
#include "scheduler.h"
#include "server.h"

void push_status_msg_type(uint8_t msg_type)
{
    switch (msg_type)
    {
        case KWP_ACKNOWLEDGE:
        case KWP_REQUEST_GROUP_READING:
        case KWP_REQUEST_GROUP_READING_0:
            return; // routine keep-alive, don't log
        case KWP_DISCONNECT:
            push_status(">> DISCONNECT");
            return;
        case KWP_REQUEST_FAULT_CODES:
            push_status(">> DTC READ");
            return;
        case KWP_REQUEST_CLEAR_FAULTS:
            push_status(">> CLR FAULTS");
            return;
        case KWP_REQUEST_LOGIN:
            push_status(">> LOGIN");
            return;
        case KWP_REQUEST_RECODE:
            push_status(">> RECODE");
            return;
        case KWP_REQUEST_ADAPTATION:
            push_status(">> ADAPT READ");
            return;
        case KWP_REQUEST_ADAPTATION_TEST:
            push_status(">> ADAPT TEST");
            return;
        case KWP_REQUEST_ADAPTATION_SAVE:
            push_status(">> ADAPT SAVE");
            return;
        case KWP_REQUEST_READ_ROM:
            push_status(">> ROM READ");
            return;
        case KWP_REQUEST_OUTPUT_TEST:
            push_status(">> ACT TEST");
            return;
        case KWP_REQUEST_BASIC_SETTING:
            push_status(">> BASIC SET");
            return;
        case KWP_REQUEST_BASIC_SETTING_0:
            push_status(">> BASIC SET 0");
            return;
        default:
        {
            char buf[STATUS_LINE_LEN + 1];
            snprintf(buf, sizeof(buf), "?? UNKN: 0x%02X", msg_type);
            push_status(buf);
        }
    }
}

void setup()
{
    Serial.begin(115200); // USB Debug if you dont want to use a TFT display shield
    init_display();
    return;
}

void loop()
{
    unsigned long display_work_start = micros();

    display_status_bar(block_counter, awake, connected);

    // Render one changed log line per loop — yields immediately if KWP byte arrived
    if (!scheduler_kwp_pending() && scheduler.rendering_in_progress)
        scheduler_render_next_word();

    scheduler.display_elapsed_us += micros() - display_work_start;

    // 5 baud
    if (!awake)
        if (!wakeup())
        {
            return;
        }

    // device data
    if (!connected)
        if (!connect())
            return;

    // keep alive / group reading / etc
    uint8_t received_count = 0;
    uint8_t message_type = 0;
    uint8_t buff[32];
    if (!KWP_receive_block(buff, received_count, message_type))
    {
        Serial.println("receive block error");
        Serial.print("received: ");
        Serial.print(received_count);
        Serial.print(" message type: ");
        Serial.println(message_type, HEX);
        reset();
        return;
    }
    // Print message type in hex scroll area, log notable events
    print_message_type(message_type);
    push_status_msg_type(message_type);
    switch (message_type)
    {
        case KWP_DISCONNECT:
            // Serial.println("-> DISCONNECT");
            reset();
            return;
            break;
        case KWP_ACKNOWLEDGE:
            // Serial.println("-> ACKNOWLEDGE");
            if (fault_send_offset > 0 && fault_send_offset < current_ecu.num_faults)
            {
                if (!KWP_send_fault_codes_from(fault_send_offset))
                {
                    Serial.println("send fault codes (cont) error");
                    reset();
                    return;
                }
            }
            else
            {
                fault_send_offset = 0;
                if (!KWP_send_ack())
                {
                    Serial.println("send ack block error");
                    reset();
                    return;
                }
            }
            break;
        case KWP_REQUEST_GROUP_READING:
        {
            uint8_t group_selected = buff[3];
            // Serial.print("-> GROUP_READING Group: ");
            // Serial.println(group_selected);
            if (!KWP_send_group_reading(group_selected))
            {
                Serial.println("send group reading error");
                reset();
                return;
            }
        }
        break;
        case KWP_REQUEST_FAULT_CODES:
            Serial.println("-> FAULT_CODES");
            if (!KWP_send_fault_codes())
            {
                Serial.println("send fault codes error");
                reset();
                return;
            }
            break;
        case KWP_REQUEST_CLEAR_FAULTS:
            Serial.println("-> CLEAR_FAULTS");
            fault_send_offset = 0;
            if (!KWP_send_ack())
            {
                Serial.println("send ack block error");
                reset();
                return;
            }
            break;
        default:
            Serial.print("Message type: ");
            Serial.print(message_type, HEX);
            Serial.println(" unsupported");
            break;
    }

    return;
}
