// ARDUINO MEGA 2560 + TFT LCD SHIELD + Serial1
#include "server.h"

void setup() {
    Serial.begin(115200); // USB Debug if you dont want to use a TFT display shield
    init_display();
    return;
}

void loop() {
    display_status_bar(block_counter, awake, connected);

    // 5 baud
    if (!awake)
        if (!wakeup()) {
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
    if (!KWP_receive_block(buff, received_count, message_type)) {
        Serial.println("receive block error");
        Serial.print("received: ");
        Serial.print(received_count);
        Serial.print(" message type: ");
        Serial.println(message_type, HEX);
        reset();
        return;
    }
    // Print message type in list
    print_message_type(message_type);
    switch (message_type) {
    case KWP_DISCONNECT:
        // Serial.println("-> DISCONNECT");
        reset();
        return;
        break;
    case KWP_ACKNOWLEDGE:
        // Serial.println("-> ACKNOWLEDGE");
        if (!KWP_send_ack()) {
            Serial.println("send ack block error");
            reset();
            return;
        }
        break;
    case KWP_REQUEST_GROUP_READING: {
        uint8_t group_selected = buff[3];
        // Serial.print("-> GROUP_READING Group: ");
        // Serial.println(group_selected);
        if (!KWP_send_group_reading(group_selected)) {
            Serial.println("send group reading error");
            reset();
            return;
        }
    } break;
    case KWP_REQUEST_FAULT_CODES:
        Serial.println("-> FAULT_CODES");
        if (!KWP_send_fault_codes()) { // or KWP_send_fault_codes_empty
            Serial.println("send fault codes error");
            reset();
            return;
        }
        break;
    case KWP_REQUEST_CLEAR_FAULTS:
        Serial.println("-> CLEAR_FAULTS");
        ;
        if (!KWP_send_ack()) {
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
