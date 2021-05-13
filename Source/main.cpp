/*
 * Authors:     Rutwij Makwana and Mark MacGillivray
 * Project:     Dalhousie CubeSat
 * SubSystem:   ADCS
 * Date:        2021-05-12
 * References:

 * Description: main file containing core logic for ADCS.
 * 
 * Todo:
 * 1. Implement send queue
 * 2. Check for thread safety
 * 3. Implement logging thread
 * 4. Add data processing to main_thread
 * 5. Add error handling to watchdog_thread
 *
 */

#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <queue>
#include <condition_variable>

#include "CubeSat_ADCSConfig.h"
#include "main.h"
#include "JSON.h"
#include "UART.h"

using namespace std;

// struct to hold incoming message from UART
struct r_message {
    JSON *json;
    std::chrono::time_point<std::chrono::high_resolution_clock> receive_time;
};

std::queue<r_message> receive_queue;        // message receive queue uart_thread ---> receive_queue ----> main_thread
std::mutex receive_queue_mutex;             // mutex to guard access to queue
std::condition_variable receive_queue_cv;   // condition variable to inform that queue has a new message

std::queue<r_message> send_queue;           // message send queue main_thread ---> send_queue ----> uart_thread
std::mutex send_queue_mutex;                // mutex to guard access to queue
std::condition_variable send_queue_cv;      // condition variable to inform that queue has a new message

void handle_uart();
void handle_main();
void handle_watchdog();
void handle_logging();

int get_time(std::chrono::time_point<std::chrono::high_resolution_clock> t);

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

// watchdog thread wakes up every WATCHDO_TICK and calculates the difference between current time and the time it was last reset
void handle_watchdog() {
    last_tick_time = chrono::high_resolution_clock::now();
    while(1) {
        last_tick_time += chrono::microseconds(WATCHDOG_TICK);
        this_thread::sleep_until(last_tick_time);

        chrono::high_resolution_clock::time_point current_time = chrono::high_resolution_clock::now();
        chrono::duration<double, std::ratio<1, 1000>> time_since_reset = chrono::duration_cast<chrono::duration<double, std::ratio<1, 1000>>>(current_time - last_reset_time);
        // cout << time_since_reset.count() << endl;
        
        if(time_since_reset.count() > SUBSYTEM_TICK * WATCHDOG_RESET_THRESHOLD) {
            set_state(STATE_ERR_UART_RECEIVE);              // todo: set state according to error
            cout << "Watchdog triggered" << endl;
            watchdog_state = false;
        }
        else {
            set_state(STATE_OK);
            watchdog_state = true;
        }
    }
}

void handle_main() {
    while(1) {
        std::unique_lock<std::mutex> lk(receive_queue_mutex);
        receive_queue_cv.wait(lk);
        if(receive_queue.empty() == false) {
            r_message &m = reinterpret_cast<r_message &>(receive_queue.front());
            cout << "Popped a message" << endl;
            receive_queue.pop();
            cout << "Receive time = " << get_time(m.receive_time) << endl;
        }
        lk.unlock();
    }
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
#if ENABLE_UART_OPEN
            std::str rcv;
            std::str &r_rcv;
            ret_val ret = ser.readString(r_rcv);
            r_message m;
            m->json = JSON(rcv)
            m.receive_time = chrono::high_resolution_clock::now();
            std::unique_lock<std::mutex> lk(receive_queue_mutex);
            receive_queue.push(m);
            lk.unlock();
            receive_queue_cv.notify_one();
            reset_watchdog();
            cout << "Pushed a message" << endl;
#else
            r_message m;
            m.json = new JSON(json_test2);
            m.receive_time = chrono::high_resolution_clock::now();
            std::unique_lock<std::mutex> lk(receive_queue_mutex);
            receive_queue.push(m);
            lk.unlock();
            cout << "Pushed a message" << endl;
            receive_queue_cv.notify_one();
            reset_watchdog();
            sleep(1);
#endif  // ENABLE_UART_OPEN
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

int get_time(std::chrono::time_point<std::chrono::high_resolution_clock> t) {
    chrono::duration<double, std::ratio<1, 1000>> time_span = chrono::duration_cast<chrono::duration<double, std::ratio<1, 1000>>>(t - system_start_time);
    return time_span.count();
}