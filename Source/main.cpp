//this is the main function used to run our system
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>

#include "CubeSat_ADCSConfig.h"
#include "main.h"
#include "JSON.h"
#include "UART.h"

using namespace std;

void handle_uart();
void handle_main();
void handle_watchdog();
void handle_logging();

int main() {
    
#if ENABLE_THREADING
    // create threads
    std::thread uart_thread(handle_uart);           // handles reading from UART device
    std::thread main_thread(handle_main);           // handles ADCS calculations
    std::thread watchdog_thread(handle_watchdog);   // watchdog thread
    std::thread logging_thread(handle_logging);     // handles logging
#endif
    
    system_start_time = chrono::high_resolution_clock::now();

    while(true) {
        sleep(1);
    }

#if ENABLE_THREADING
    uart_thread.join();
    main_thread.join();
    watchdog_thread.join();
    logging_thread.join();
#endif

    return SUCCESS;
}

void set_state(state s) {
    std::lock_guard<std::mutex> guard(state_mutex);
    current_state = s;
}

state get_state(state s) {
    std::lock_guard<std::mutex> guard(state_mutex);
    return current_state;
}

void handle_watchdog() {
    last_tick_time = chrono::high_resolution_clock::now();
    while(1) {
        last_tick_time += chrono::microseconds(WATCHDOG_TICK);
        this_thread::sleep_until(last_tick_time);

        chrono::high_resolution_clock::time_point current_time = chrono::high_resolution_clock::now();
        chrono::duration<double, std::ratio<1, 1000>> time_span = chrono::duration_cast<chrono::duration<double, std::ratio<1, 1000>>>(current_time - last_reset_time);
        cout << time_span.count() << endl;
        
        if(time_span.count() > SUBSYTEM_TICK * WATCHDOG_RESET_THRESHOLD) {
            set_state(STATE_ERR_UART_RECEIVE);              // todo: set state according to error
            cout << "Watchdog triggered" << endl;
        }
        else {
            set_state(STATE_OK);
        }
    }
}

void handle_main() {
    
}

void handle_logging() {
    
}

void handle_uart() {

#if ENABLE_UART_OPEN && !TEST_UART
    UART ser = UART(UART_DEVICE);
    ret_val r = ser.begin(UART_BAUDRATE);
#endif

#if ENABLE_UART_OPEN && TEST_UART
    UART ser = UART(UART_DEVICE);
    ret_val r = ser.begin(UART_BAUDRATE);
#endif

#if !ENABLE_UART_OPEN
    ret_val r = SUCCESS;
#endif

    int i = 5;

    if(r == SUCCESS) {
        while(1) {
            if(i > 0) {
                reset_watchdog();
                cout << "Watchdog reset" << endl;
            }
            i--;
            sleep(1);
        }
    }
}

void reset_watchdog() {
    std::lock_guard<std::mutex> guard(watchdog_mutex);
    last_reset_time = chrono::high_resolution_clock::now();
}

bool get_watchdog_state() {
    std::lock_guard<std::mutex> guard(watchdog_mutex);
    return watchdog_state;
}