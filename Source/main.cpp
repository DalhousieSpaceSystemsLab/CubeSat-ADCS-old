//this is the main function used to run our system
#include <iostream>
#include <thread>

#include "CubeSat_ADCSConfig.h"
#include "main.h"
#include "JSON.h"
#include "UART.h"

using namespace std;

void handle_uart();

int main() {
    
    // create uart thread and main processing thread
    std::thread uart_thread(handle_uart);
    std::thread main_thread(handle_main);
    std::thread watchdog_thread(handle_watchdog);
    std::thread logging_thread(handle_logging);
    
    while(true) {
        
    }
    
    uart_thread.join();
    main_thread.join();
    watchdog_thread.join();
    logging_thread.join();

    return SUCCESS;
}

void handle_uart() {
    UART ser = UART(UART_DEVICE);
    cout << ser.getDeviceName() << endl;
    ret_val r = ser.begin(UART_BAUDRATE);
    cout << r << endl;

    if(r == SUCCESS) {
        while(1) {
            ret_val ret = ser.write(std::string("Hi there\n"));
            cout << "Write returns " << ret << endl;
            std::string s;
            std::string &sr = s;
            ret = ser.readString(s);
            cout << "Read returns " << ret << " Value = " << s << endl;
            sleep(1);
        }
    }
}