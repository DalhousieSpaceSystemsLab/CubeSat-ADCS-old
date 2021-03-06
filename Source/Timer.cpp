#include "Timer.h"

float average_time = 0;
std::chrono::system_clock::time_point then = std::chrono::high_resolution_clock::now();

int seconds(int secs) {
    return secs * 1000000000;
}

void print_time() {
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = now-then;
    then = now;
    average_time += elapsed.count();
    std::cout << std::endl << "Elapsed ms: " << elapsed.count() << std::endl;
}

ret_val Timer::set_interval(uint32_t interval) {
    this->loop_time = interval;
    return SUCCESS;
}

uint32_t Timer::get_interval() {
    return this->loop_time;
}

uint32_t Timer::get_loop_count() {
    return this->tcb.get_count();
}

ret_val Timer::start_timer() {
    this->registerEventRunnable(tcb);
    this->start(seconds(loop_time));
    return SUCCESS;
}

ret_val Timer::stop_timer() {
    this->unregisterEventRunnable();
    this->stop();
    return SUCCESS;
}

