#ifndef TIMER_H
#define TIMER_H

#include <iostream>
#include <chrono>
#include "./cppTimer/CppTimerCallback.h"
#include "Errors.h"

int seconds(int secs);

void print_time();

class Timer_callback : public CppTimerCallback::Runnable{
    public:
        void run() {
            if(debug) {
                print_time();
            }
            cb();
            loop_count++;
        }
        void set_callback(void (*callback_function)(void), bool debug_print = false) {
            cb = callback_function;
            debug = debug_print;
        }
        uint32_t get_count() {
            return loop_count;
        }
    private:
        void (*cb)(void);
        bool debug;
        uint32_t loop_count = 0;
};

class Timer : public CppTimerCallback{
    public:
        /*
        Timer constructor
        Input:
            callback_function: pointer to the callback function
            interval: timer interval in seconds (default = 1s)
        */
        Timer(void (*callback_function)(void), uint32_t interval = 1, bool debug_print = false) {
            loop_time = interval;
            callback = callback_function;
            tcb.set_callback(callback_function, debug_print);
        }
        ret_val start_timer();
        ret_val stop_timer();
        uint32_t get_interval();
        ret_val set_interval(uint32_t interval);
        uint32_t get_loop_count();
    private:
        void (*callback)(void);
        uint32_t loop_time;
        CppTimerCallback cpptimer;
        Timer_callback tcb;

};





#endif  // TIMER_H