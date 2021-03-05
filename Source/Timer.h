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
            print_time();
            cb();
        }
        void set_callback(void (*callback_function)(void)) {
            cb = callback_function;
        }
    private:
        void (*cb)(void);
};

class Timer : public CppTimerCallback{
    public:
        /*
        Timer constructor
        Input:
            callback_function: pointer to the callback function
            interval: timer interval in seconds (default = 1s)
        */
        Timer(void (*callback_function)(void), uint32_t interval = 1) {
            loop_time = interval;
            callback = callback_function;
            tcb.set_callback(callback_function);
        }
        ret_val start_timer();
        ret_val stop_timer();
        uint32_t get_interval();
        ret_val set_interval(uint32_t interval);
    private:
        void (*callback)(void);
        uint32_t loop_time;
        CppTimerCallback cpptimer;
        Timer_callback tcb;
};





#endif  // TIMER_H