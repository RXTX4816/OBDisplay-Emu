#include "display.h"
#include <Arduino.h>

#include "ecu.h"
#include "groups.h"
#include "kwp_defs.h"
#include "sim.h"

#ifndef APP_VERSION
#define APP_VERSION "dev"
#endif

#define PIN_TX 18 // Serial1
#define PIN_RX 19

#define ADDR 0x17
#define BAUDRATE 10400

#define TIMEOUT 1250

// Some diagnostic clients expect different physical-layer behavior.
// - On real K-line, the *tester* often sees an electrical echo of its own TX.
//   When using separate RX/TX wires, that echo is usually not present.
//   Keep this configurable.
#ifndef KWP_EMU_ECHO_RX_BYTES
#define KWP_EMU_ECHO_RX_BYTES 0
#endif

#ifndef KWP_EMU_INTERBYTE_DELAY_MS
#define KWP_EMU_INTERBYTE_DELAY_MS 5
#endif

// Some clients send a complement after the last byte too; others don't.
// If enabled, we will consume it only if it matches the expected complement.
#ifndef KWP_EMU_CONSUME_OPTIONAL_LAST_COMPLEMENT
#define KWP_EMU_CONSUME_OPTIONAL_LAST_COMPLEMENT 1
#endif

// Forward declarations
bool KWP_send_ack();
bool KWP_send_fault_codes_empty();
bool KWP_send_fault_codes_from(uint8_t offset);

bool awake = false;
bool initial_condition = HIGH;

bool connected = false;

uint8_t block_counter = 0;
uint8_t fault_send_offset = 0;
/**
 * @brief Read OBD input from ECU
 *
 * @return uint8_t The incoming byte or -1 if timeout
 */
int16_t OBD_read()
{
    unsigned long timeout = millis() + TIMEOUT;
    while (!Serial1.available())
    {
        if (millis() >= timeout)
        {
            Serial.println("ERROR: OBD_read() timeout");
            return -1;
        }
    }
    int16_t data = Serial1.read();

#if KWP_EMU_ECHO_RX_BYTES
    // Optional: mimic "echo" some K-line setups exhibit.
    Serial1.write((uint8_t)data);
#endif

    return data;
}

/**
 * @brief Write data to the ECU, wait 5ms before each write to ensure connectivity.
 *
 * @param data The data to send.
 */
void OBD_write(uint8_t data)
{
    delay(KWP_EMU_INTERBYTE_DELAY_MS);
    Serial1.write(data);
}

static inline void OBD_consume_if_next_byte_is(uint8_t expected)
{
#if KWP_EMU_CONSUME_OPTIONAL_LAST_COMPLEMENT
    if (Serial1.available() && Serial1.peek() == expected)
    {
        (void)Serial1.read();
    }
#else
    (void)expected;
#endif
}

/**
 * @brief Send a request to the ECU
 *
 * @param s Array where the data is stored
 * @param size The size of the request
 * @return true If no errors occured, will resume
 * @return false If errors occured, will disconnect
 */
bool KWP_send_block(uint8_t* s, int size)
{
    Serial.print("TX [BC=");
    Serial.print(block_counter);
    Serial.print("]: ");
    for (uint8_t i = 0; i < size; i++)
    {
        if (s[i] < 0x10)
            Serial.print("0");
        Serial.print(s[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    for (uint8_t i = 0; i < size; i++)
    {
        uint8_t data = s[i];
        OBD_write(data);

        if (i < size - 1)
        {
            int16_t complement = OBD_read();
            if (complement != (data ^ 0xFF))
            {
                Serial.print("  complement error byte[");
                Serial.print(i);
                Serial.print("]=");
                Serial.print(data, HEX);
                Serial.print(" got=");
                Serial.print(complement, HEX);
                Serial.print(" want=");
                Serial.println(data ^ 0xFF, HEX);
                return false;
            }
        }
        else
        {
            // Some clients also send a complement for the final byte.
            OBD_consume_if_next_byte_is(data ^ 0xFF);
        }
    }
    block_counter++;
    return true;
}

bool KWP_send_syncbytes()
{
    uint8_t s[32] = {0x55, 0x01, 0x8A};
    uint8_t size = 3;
    Serial.print("Sending ");
    for (uint8_t i = 0; i < size; i++)
    {
        Serial.print(s[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    for (uint8_t i = 0; i < size; i++)
    {
        uint8_t data = s[i];
        OBD_write(data);

        if (i == 2)
        {
            int16_t complement = OBD_read();
            if (complement != (data ^ 0xFF))
            {
                Serial.print("Received: ");
                Serial.print(complement, HEX);
                Serial.print(" Expected: ");
                Serial.println(data ^ 0xFF, HEX);
                return false;
            }
        }
    }
    block_counter++;
    return true;
}

bool KWP_send_group_reading(uint8_t group)
{
    Serial.print("group reading group=");
    Serial.println(group);

    uint8_t buf[16];
    uint8_t size = build_group_reading(group, block_counter, buf);
    return KWP_send_block(buf, size);
}

bool KWP_send_fault_codes_from(uint8_t offset)
{
    uint8_t remaining = current_ecu.num_faults - offset;
    if (remaining == 0)
    {
        fault_send_offset = 0;
        return KWP_send_ack();
    }

    uint8_t count = (remaining > 4) ? 4 : remaining;
    uint8_t frame_size = count * 3 + 4; // header(3) + data + terminator(1)
    uint8_t buf[16] = {0};
    buf[0] = frame_size - 1;
    buf[1] = block_counter;
    buf[2] = KWP_RECEIVE_FAULT_CODES;
    for (uint8_t i = 0; i < count; i++)
    {
        buf[3 + i * 3] = current_ecu.faults[offset + i][0]; // DTC high
        buf[4 + i * 3] = current_ecu.faults[offset + i][1]; // DTC low
        buf[5 + i * 3] = current_ecu.faults[offset + i][2]; // status
    }
    buf[3 + count * 3] = 0x03;

    fault_send_offset = offset + count;
    return KWP_send_block(buf, frame_size);
}

bool KWP_send_fault_codes()
{
    if (current_ecu.num_faults == 0)
        return KWP_send_fault_codes_empty();
    return KWP_send_fault_codes_from(0);
}

bool KWP_send_fault_codes_empty()
{
    uint8_t buf[7] = {0x06, block_counter, KWP_RECEIVE_FAULT_CODES, 0xFF, 0xFF, 0x88, 0x03};
    return (KWP_send_block(buf, 7));
}

/**
 * @brief The default way to keep the ECU awake is to send an Acknowledge Block.
 * Alternatives include Group Readings..
 *
 * @return true No errors
 * @return false Errors, disconnect
 */
bool KWP_send_ack()
{
    uint8_t buf[4] = {0x03, block_counter, 0x09, 0x03};
    return (KWP_send_block(buf, 4));
}

bool KWP_send_devicedata(const char* id_string)
{
    uint8_t s[32];
    s[0] = 0x0F;
    s[1] = block_counter;
    s[2] = 0xF6;
    s[15] = 0x03;

    // Copy ID string (12 bytes, padded with spaces if shorter)
    for (uint8_t i = 0; i < 12; i++)
    {
        if (id_string && id_string[i] != '\0')
            s[3 + i] = (uint8_t)id_string[i];
        else
            s[3 + i] = 0x20; // space
    }

    uint8_t size = 0x0F + 1;
    return KWP_send_block(s, size);
}

bool KWP_receive_ack()
{
    unsigned long timeout = millis() + TIMEOUT;
    uint8_t s[32];
    int recvcount = 0;

    while (recvcount < 4)
    {
        while (Serial1.available())
        {
            int16_t data = OBD_read();
            if (data == -1)
            {
                Serial.println("receive ack error AVA=0 or empty buffer");
                return false;
            }
            s[recvcount] = data;
            recvcount++;

            if (recvcount >= 4)
            {
                timeout = millis() + TIMEOUT;
                break;
            }

            if (recvcount == 2 && data != block_counter)
            {
                Serial.print("ACK block counter mismatch: got=0x");
                Serial.print(data, HEX);
                Serial.print(" expected=0x");
                Serial.println(block_counter, HEX);
                return false;
            }

            delay(5);
            OBD_write(data ^ 0xFF); // send complement ack

            timeout = millis() + TIMEOUT;

            // debugstrnum(F(" - KWP_receive_block: Added timeout. ReceiveCount: "),
            // (uint8_t)recvcount); debug(F(". Processed data: ")); debughex(data);
            // debugstrnumln(F(". ACK compl: "), ((!ackeachbyte) && (recvcount == size)) ||
            // ((ackeachbyte) && (recvcount < size)));
        }

        if (millis() >= timeout)
        {
            Serial.print("Timeout - recvcount = ");
            Serial.println(recvcount);
            return false;
        }
    }

    if (s[0] != 0x03 || s[1] != block_counter || s[2] != 0x09 || s[3] != 0x03)
    {
        Serial.print("ACK parse error [BC=");
        Serial.print(block_counter);
        Serial.print("]: got ");
        for (uint8_t i = 0; i < 4; i++)
        {
            if (s[i] < 0x10)
                Serial.print("0");
            Serial.print(s[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        return false;
    }

    Serial.print("ACK ok [BC=");
    Serial.print(block_counter);
    Serial.println("]");
    block_counter++;

    return true;
}

uint8_t wait_5baud()
{
    pinMode(PIN_RX, INPUT_PULLUP);
    initial_condition = digitalRead(PIN_RX);
    Serial.print("initial_condition ");
    Serial.println(initial_condition);
    if (initial_condition == LOW)
    {
        initial_condition = HIGH;
        return 0x00;
    }
    Serial.println("waiting 5baud address on PIN_RX_19 ...");
    g.print("Waiting 5baud..", LEFT, rows[4]);
    bool ready = false;
    unsigned long fall_time = 0;
    while (!ready)
    {
        if (digitalRead(PIN_RX) == LOW)
        {
            fall_time = millis();
            bool glitch = false;
            while (millis() - fall_time < 80)
            {
                if (digitalRead(PIN_RX) == HIGH)
                {
                    glitch = true;
                    break;
                }
            }
            if (!glitch)
                ready = true;
        }
    }
    Serial.println("initial_condition changed");
    g.setColor(TFT_GREEN);
    g.print("---", RIGHT, rows[4]);
    g.setColor(font_color);
    bool bits[10];
    bits[0] = LOW;
    Serial.print(bits[0]);
    unsigned long elapsed = millis() - fall_time;
    if (elapsed < 300)
        delay(300 - elapsed);
    for (uint8_t i = 1; i < sizeof(bits); i++)
    {
        bits[i] = digitalRead(PIN_RX);
        delay(200);
    }
    Serial.print(" ");
    for (uint8_t i = 0; i < sizeof(bits); i++)
    {
        Serial.print(bits[i]);
        Serial.print(" ");
        g.printNumI(bits[i], cols[2 + i * 2], rows[5], 1);
    }
    Serial.println();

    // Debug: print exact bit indices and values
    Serial.println("Debug - individual bits:");
    for (uint8_t i = 0; i < 10; i++)
    {
        Serial.print("bits[");
        Serial.print(i);
        Serial.print("]=");
        Serial.print(bits[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.print("data[1..7] parity[8] stop[9]: ");
    for (uint8_t i = 1; i <= 9; i++)
    {
        Serial.print(bits[i]);
        Serial.print(" ");
    }
    Serial.println();

    // First validate start (bits[0]=LOW) and stop (bits[9]=HIGH)
    if (bits[0] != LOW || bits[9] != HIGH)
    {
        Serial.println("5baud validation failed (bad start/stop bits)");
        initial_condition = HIGH;
        g.print("   ", RIGHT, rows[4]);
        return 0x00;
    }

    // Decode 7 data bits (bits[1..7]) LSB-first — KWP1281 5-baud uses 7O1 framing
    uint8_t addr = 0;
    for (uint8_t i = 1; i <= 7; i++)
    {
        if (bits[i] == HIGH)
            addr |= (1 << (i - 1));
    }

    // Verify odd parity: bits[8] must make the total number of 1-bits odd
    uint8_t ones = 0;
    for (uint8_t i = 1; i <= 7; i++)
        if (bits[i] == HIGH)
            ones++;
    bool parity_ok = (ones % 2 == 0) ? (bits[8] == HIGH) : (bits[8] == LOW);
    if (!parity_ok)
    {
        Serial.println("5baud parity check failed");
        initial_condition = HIGH;
        g.print("   ", RIGHT, rows[4]);
        return 0x00;
    }

    // 7-bit address range: 0x00 and anything >= 0x80 are impossible
    if (addr == 0x00 || addr >= 0x80)
    {
        Serial.print("decoded address 0x");
        Serial.print(addr, HEX);
        Serial.println(" - rejected (invalid)");
        initial_condition = HIGH;
        g.print("   ", RIGHT, rows[4]);
        return 0x00;
    }

    // Verify address exists in ECU_TABLE
    bool found_in_table = false;
    for (uint8_t i = 0; i < ECU_COUNT; i++)
    {
        if (pgm_read_byte(&ECU_TABLE[i].address) == addr)
        {
            found_in_table = true;
            break;
        }
    }

    if (!found_in_table)
    {
        Serial.print("decoded address 0x");
        Serial.print(addr, HEX);
        Serial.println(" - not in ECU table");
        initial_condition = HIGH;
        g.print("   ", RIGHT, rows[4]);
        return 0x00;
    }

    Serial.print("decoded address: 0x");
    Serial.println(addr, HEX);
    g.setColor(TFT_GREEN);
    g.print("---", RIGHT, rows[5]);
    return addr;
}

bool KWP_receive_block(uint8_t buff[], uint8_t& received_count, uint8_t& message_type)
{
    uint8_t recvcount = 0;
    uint8_t expected_total = 0; // total bytes including length byte

    // Subtract display overhead from initial timeout so the ECU appears as fast as real hardware
    unsigned long display_overhead_ms = scheduler.display_elapsed_us / 1000UL;
    unsigned long timeout =
        millis() + (display_overhead_ms < TIMEOUT ? TIMEOUT - display_overhead_ms : 0);
    scheduler.display_elapsed_us = 0;

    while (true)
    {
        while (Serial1.available())
        {
            int16_t raw = OBD_read();
            if (raw < 0)
            {
                Serial.println("data = -1");
                return false;
            }
            uint8_t data = (uint8_t)raw;

            if (recvcount >= 32)
            {
                Serial.println("receive block error: buffer overflow");
                return false;
            }
            buff[recvcount] = data;
            recvcount++;

            if (recvcount == 1)
            {
                expected_total = (uint8_t)(data + 1);
                if (expected_total < 4 || expected_total > 32)
                {
                    Serial.println("receive block error: invalid length");
                    return false;
                }
            }
            else if (recvcount == 2)
            {
                if (data != block_counter)
                {
                    Serial.println("WARNING: block counter does not match");
                }
            }
            else if (recvcount == 3)
            {
                message_type = data;
            }

            timeout = millis() + TIMEOUT;

            if (expected_total != 0 && recvcount >= expected_total)
            {
                received_count = recvcount;
                block_counter++;
                Serial.print("RX [BC=");
                Serial.print(block_counter - 1);
                Serial.print("] type=0x");
                Serial.print(message_type, HEX);
                Serial.print(" len=");
                Serial.print(recvcount);
                Serial.print(" bytes: ");
                for (uint8_t i = 0; i < recvcount; i++)
                {
                    if (buff[i] < 0x10)
                        Serial.print("0");
                    Serial.print(buff[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                return true;
            }

            OBD_write(data ^ 0xFF); // complement ack for all non-final bytes
        }

        if (millis() >= timeout)
        {
            Serial.print("Timeout - recvcount = ");
            Serial.println(recvcount);
            error_timeout(TIMEOUT);
            return false;
        }
    }
}

void reset()
{
    connected = false;
    awake = false;
    block_counter = 0;
    fault_send_offset = 0;
    initial_condition = HIGH;

    Serial.println("Waiting 3 sec");
    delay(3000);
    // Stop renderer first so it can't repaint stale buffers onto the cleared screen
    scheduler_init();
    for (uint8_t i = 4; i < 20; i++)
        clearRow(i);
    for (uint8_t i = 0; i < STATUS_LOG_SIZE; i++)
    {
        _status_log[i][0] = '\0';
        _status_log_prev[i][0] = '\0';
    }
    _status_count = 0;
    _status_log_initialized = false;
    display_ecu_info(0, 0);
    Serial1.end();
}

bool wakeup()
{
    current_addr = wait_5baud();
    if (current_addr == 0x00)
    {
        Serial.println("5baud error");
        g.setColor(TFT_RED);
        g.print("xxx", RIGHT, rows[5]);
        g.setColor(font_color);
        reset();
        return false;
    }
    awake = true;
    initial_condition = HIGH;
    Serial.println("5baud success");
    return true;
}

bool connect()
{
    // Look up ECU definition by address
    bool found = false;
    for (uint8_t i = 0; i < ECU_COUNT; i++)
    {
        if (pgm_read_byte(&ECU_TABLE[i].address) == current_addr)
        {
            load_ecu_def(&ECU_TABLE[i], current_ecu);
            found = true;
            break;
        }
    }
    if (!found)
    {
        Serial.print("unknown address 0x");
        Serial.println(current_addr, HEX);
        reset();
        return false;
    }

    // Determine baud rate from address (safer than reading from PROGMEM struct)
    uint16_t baud = 10400; // default
    if (current_addr == 0x01 || current_addr == 0x03 || current_addr == 0x08 ||
        current_addr == 0x15 || current_addr == 0x19 || current_addr == 0x46)
    {
        baud = 9600;
    }

    Serial.print("ECU addr=0x");
    Serial.print(current_ecu.address, HEX);
    Serial.print(" baud=");
    Serial.println(baud);

    Serial1.begin(baud);
    delay(100); // Give Serial1 time to stabilize after baud change

    if (!KWP_send_syncbytes())
    {
        Serial.println("syncbytes error");
        reset();
        return false;
    }
    g.print("Sending syncbytes..", LEFT, rows[6]);
    Serial.println("syncbytes success");

    // Send 3 identity strings
    for (uint8_t i = 0; i < 3; i++)
    {
        Serial.print("-> sending device data ");
        Serial.print(i + 1);
        Serial.println(" / 3");

        const char* id_str = nullptr;
        if (i == 0)
            id_str = current_ecu.part_number;
        else if (i == 1)
            id_str = current_ecu.component;
        else
            id_str = current_ecu.coding_wsc;

        if (!KWP_send_devicedata(id_str))
        {
            Serial.println("send device data error");
            g.setColor(TFT_RED);
            g.print("xxx", RIGHT, rows[6]);
            g.setColor(font_color);
            reset();
            return false;
        }
        Serial.print("---> receive ack block ");
        Serial.print(i + 1);
        Serial.println(" / 3");
        if (!KWP_receive_ack())
        {
            Serial.println("receive ack block error");
            g.setColor(TFT_RED);
            g.print("xxx", RIGHT, rows[6]);
            g.setColor(font_color);
            reset();
            return false;
        }
    }
    Serial.println("device data success");
    g.setColor(TFT_GREEN);
    g.print("---", RIGHT, rows[6]);

    g.print("Sending ack..", LEFT, rows[7]);
    Serial.println("sending ack block");
    if (!KWP_send_ack())
    {
        Serial.println("send ack error");
        g.setColor(TFT_RED);
        g.print("failed", RIGHT, rows[7]);
        g.setColor(font_color);
        reset();
        return false;
    }
    connected = true;

    // Initialize simulation state
    unsigned long now = millis();
    sim_state.start_ms = now;
    sim_state.distance_km = 0.0f;
    sim_state.fuel_L = 55.0f; // full tank
    sim_state.last_update_ms = now;

    g.setColor(TFT_YELLOW);
    g.print("connected", RIGHT, rows[7]);
    g.setColor(font_color);
    draw_line_on_row(8);
    draw_line_on_row(12);
    display_ecu_info(current_addr, baud);
    init_status_log();
    scheduler_init();
    char conn_msg[STATUS_LINE_LEN + 1];
    snprintf(conn_msg, sizeof(conn_msg), "CONNECTED 0x%02X", current_addr);
    push_status(conn_msg);
    g.setColor(TFT_GREEN);
    Serial.println("------------------------------------------");
    Serial.println("|               connected                |");
    Serial.println("|----------------------------------------|");
    Serial.print("|keep alive by sending ack within ");
    Serial.print(TIMEOUT);
    Serial.println(" ms |");
    Serial.println("|----------------------------------------|");
    return true;
}
