/*
 * Authors:     Rutwij Makwana
 * Project:     Dalhousie CubeSat
 * SubSystem:   ADCS
 * Date:        2021-05-12
 * References:

 * Description: This file contains common definitions for main.cpp.
 * 
 *
 */

#ifndef MAIN_H
#define MAIN_H

#include <mutex>

// Global configurations
// #define ENABLE_UART         1
// #define ENABLE_MAIN         1
// #define ENABLE_LOGGING      1
// #define ENABLE_WATCHDOG     1

#define ENABLE_THREADING    1   // enable/disable threads
#define ENABLE_UART_OPEN    0   // enable/disable opening UART
#define TEST_UART           1   // enable/disable UART test 

// Global constants
uint SUBSYTEM_TICK = 1000;      // subsytem tick interval in milliseconds

// Subsytem states
enum state {
    STATE_OK,                   // system is running as intended
    STATE_INITIALIZING,         // subsystem initializing
    STATE_ERR_INIT_FAILED,      // subsystem initialization failed
    STATE_ERR_UART_RECEIVE,     // OBC is not receiving data from MCU
};

std::mutex state_mutex;         // guards access to current_state
state current_state;            // holds currently active state
void set_state(state s);        // set a new state
state get_state();              // get current state

// Watchdog configuration
const int WATCHDOG_RESET_THRESHOLD = 3; // watchdog reset threshold in multiple of SUBSYTEM_TICK
const int WATCHDOG_TICK = 100000;       // watchdog tick interval in microseconds

std::mutex watchdog_mutex;      // guards watchdog_state

std::chrono::time_point<std::chrono::high_resolution_clock> system_start_time;  // holds the time subsytem started
std::chrono::time_point<std::chrono::high_resolution_clock> last_tick_time;     // holds last time WD thread ticked
std::chrono::time_point<std::chrono::high_resolution_clock> last_reset_time;    // holds the time WD was last reset
bool watchdog_state;            // holds current watchdog state
void reset_watchdog();          // resets watchdog
bool get_watchdog_state();      // returns watchdog state - true = ok, false = triggered

// UART configuration
const int UART_BAUDRATE = 57600;                    // UART baudrate
const char *UART_DEVICE = "/dev/ttyS1";             // production UART device
const char *UART_TEST_DEVICE = "/home/oem/com1";    // test UART device

// test string for JSON
std::string json_test = "{\"name\" : \"Jack\", \"age\" : 27, \"age\" : 27, \"age\" : 27, \"age\" : 27, \"age\" : 27}";
std::string json_test2 = "{\"menu\": { \"id\": \"file\", \"value\": \"File\", \"popup\": {\"menuitem\": [{\"value\": \"New\", \"onclick\": \"CreateNewDoc()\"}, {\"value2\": \"Open\", \"onclick\": \"OpenDoc()\"}, {\"value3\": \"Close\", \"onclick\": \"CloseDoc()\"}]}}}";
std::string json_test3 = "{\"menu\": {\"id\": -123.2345, \"value\": false, \"popup\": {\"menuitem\": {\"key1\": \"value1\", \"key2\": {\"key3\": \"value3\"}}}}}";

#endif      // MAIN_H