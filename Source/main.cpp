//this is the main function used to run our system
#include <iostream>
#include <thread>
#include "CubeSat_ADCSConfig.h"
#include "Timer.h"

void test() {
    std::cout << "Hello\n";
}

int main() {
    // Initialize periodic timer with callback function as first arg and period in seconds as second arg
    Timer t(test, 1);
    t.start_timer();
    // Sleep thread for desired time, callback will be called at a regular interval
    std::this_thread::sleep_for(std::chrono::seconds(10));
    t.stop_timer();

    return 0;
}