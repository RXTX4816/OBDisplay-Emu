#include "UTFT.h"

/* Display:
Top status bar: OBD Connected, Backend connected, OBD available, block counter, com_error status, engine cold (Blue rectangle)
*/
#define WIDTH 480
#define HEIGHT 320
const int rows[20] = {1, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 256, 272, 288, 304};
const int cols[30] = {1, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 256, 272, 288, 304, 320, 336, 352, 368, 384, 400, 416, 432, 448, 464};
const byte max_chars_per_row_smallfont = 60;
const byte max_chars_per_row_bigfont = 30; // = maximum 1560 characters with small font
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[]; // Declare which fonts we will be using
UTFT g(CTE40, 38, 39, 40, 41);    // Graphics g
word back_color = TFT_BLACK;
word font_color = TFT_CYAN;

// Per row 15 = 12 length two HEX pairs and 6=3*2 spaces
#define _ROW_PRINT_RECEIVED_DEFAULT 9
#define _COL_PRINT_RECEIVED_DEFAULT 0
uint8_t _row_print_received = 9; // 9 and 10
uint8_t _col_print_received = 0; // 0 - 30
word color_hide = TFT_YELLOW;
word color_focus = TFT_CYAN;
uint8_t _row_print_received_last = _row_print_received;
uint8_t _col_print_received_last = _col_print_received;
String _current_message_type = "00";
String _last_message_type = "00";
void increment_row_print() {
    _row_print_received_last = _row_print_received;
    if (_row_print_received >= 10) {
        _row_print_received = 9;
    } else {
        _row_print_received++;
    }
}
void increment_col_print() {
    _col_print_received_last = _col_print_received;
    if (_col_print_received >= 27) {
        _col_print_received = 0;
        increment_row_print();
    } else {
        // Value
        _col_print_received += 3;
    }
}

void print_message_type(uint8_t message_type) {
    char received_message_type[3];
    snprintf(received_message_type, sizeof(received_message_type), "%02X", message_type);
    _current_message_type = received_message_type;
    // Recolor old value
    if (!_last_message_type.equals("00")) {
        g.setColor(color_hide);
        if (_col_print_received > 0)
            g.print(_last_message_type, cols[_col_print_received_last], rows[_row_print_received]);
        else
            g.print(_last_message_type, cols[_col_print_received_last], rows[_row_print_received_last]);
    }

    g.setColor(color_focus);
    // Print current value
    g.print(_current_message_type, cols[_col_print_received], rows[_row_print_received]);

    _last_message_type = _current_message_type;
    g.setColor(font_color);
    increment_col_print();
}

void clearRow(byte row) {
    g.setColor(back_color);
    g.print(String("                              "), LEFT, rows[row]);
    g.setColor(font_color);
}

void draw_line_on_row(byte row) {
    g.drawLine(0, rows[row], WIDTH, rows[row]);
}

void init_status_bar() {
    g.print("0x17 10400 EMU", CENTER, rows[0]);
    g.print("BLOCK:     | AWAKE:  ", LEFT, rows[1]);
    g.print("CON:   | AVA:     ", LEFT, rows[2]);
    draw_line_on_row(3);
    g.print("0", cols[7], rows[1]);
    g.print("0", cols[19], rows[1]);
    g.print("0", cols[4], rows[2]);
    g.print("0", cols[14], rows[2]);
}

uint8_t display_block_counter = 0;
bool display_awake = false;
bool display_connected = false;
bool display_available = false;
void display_status_bar(uint8_t block_counter, bool awake, bool connected) {
    if (display_block_counter != block_counter) {
        display_block_counter = block_counter;
        g.printNumI(block_counter, cols[7], rows[1], 3, '0');
    }
    if (display_awake != awake) {
        display_awake = awake;
        g.printNumI(awake, cols[19], rows[1], 1, '0');
    }
    if (display_connected != connected) {
        display_connected = connected;
        g.printNumI(connected, cols[4], rows[2], 1, '0');
    }
    uint8_t temp_available = Serial1.available();
    if (display_available != temp_available) {
        display_available = temp_available;
        g.printNumI(temp_available, cols[14], rows[2], 1, '0');
    }
}

void startup_animation() {
    g.fillScr(back_color);
    // for(int i = 0; i<20;i++) {
    //     g.print("OBDisplay", CENTER, rows[i]);
    //     if (i > 0) {
    //         clearRow(i-1);
    //     }
    //     delay(333);
    // }
    // clearRow(20);
    g.setColor(font_color);
    // int x = 0;
    // for(int i =0; i<20; i++) {
    //     g.print("O B D i s p l a y", x, rows[i]);
    //     if (i > 0) {
    //         clearRow(i-1);
    //     }
    //     x+=10;
    //     delay(10);
    // }
    g.print("Welcome to", CENTER, rows[3]);
    g.print("OBDServer", CENTER, rows[5]);
    g.setFont(SmallFont);
    g.print("Version Alpha", CENTER, rows[6]);
    g.setFont(BigFont);
    g.drawRect(4 + 2, rows[17], 474, rows[17] + 12);
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 59; j++) {
            if ((i == 0 && j < 2) || (i == 7 && j >= 57))
                continue;
            g.drawLine(4 + i * 59 + j, rows[17], 4 + i * 59 + j + 1, rows[17] + 12);
            delay(1);
        }
        // g.fillRect(4+i*59+j, rows[17], 4+i*59+59, rows[17]+12);
    }
    clearRow(17);

    delay(222);

    // g.fillScr(back_color);
    g.setColor(back_color);
    g.clrScr();
    clearRow(3);
    clearRow(5);
    clearRow(6);
    g.setColor(font_color);
}

void init_display() {
    g.InitLCD(LANDSCAPE);
    g.clrScr();
    g.fillScr(back_color);
    g.setBackColor(back_color);
    g.setColor(font_color);
    g.setFont(BigFont);
    startup_animation();
    init_status_bar();
}

void reset_display() {
    _col_print_received = _COL_PRINT_RECEIVED_DEFAULT;
    _row_print_received = _ROW_PRINT_RECEIVED_DEFAULT;
    _col_print_received_last = _col_print_received;
    _row_print_received_last = _row_print_received;
}

void error_timeout() {
    g.setColor(TFT_RED);
    g.print("!!! Timeout 500 ms !!!", CENTER, rows[13]);
    delay(333);
    g.setColor(font_color);
}