//this is the main function used to run our system
#include <iostream>
#include <thread>

#include "CubeSat_ADCSConfig.h"
#include "JSON.h"
#include "UART.h"

using namespace std;

std::string json_test = "{\"name\" : \"Jack\", \"age\" : 27, \"age\" : 27, \"age\" : 27, \"age\" : 27, \"age\" : 27}";

std::string json_test2 = "{\"menu\": { \"id\": \"file\", \"value\": \"File\", \"popup\": {\"menuitem\": [{\"value\": \"New\", \"onclick\": \"CreateNewDoc()\"}, {\"value2\": \"Open\", \"onclick\": \"OpenDoc()\"}, {\"value3\": \"Close\", \"onclick\": \"CloseDoc()\"}]}}}";

std::string json_test3 = "{\"menu\": {\"id\": -123.2345, \"value\": false, \"popup\": {\"menuitem\": {\"key1\": \"value1\", \"key2\": {\"key3\": \"value3\"}}}}}";

void handle_uart();

int main() {
    
    std::thread uart_thread(handle_uart);
    
    while(true) {
        cout << "Hello from main" << endl;
        sleep(3.14159);
    }
    
    uart_thread.join();
    return 0;
}

void handle_uart() {
    UART ser = UART("/home/oem/com1");
    cout << ser.getDeviceName() << endl;
    ret_val r = ser.begin(57600);
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