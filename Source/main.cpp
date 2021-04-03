//this is the main function used to run our system
#include <iostream>

#include "CubeSat_ADCSConfig.h"
#include "JSON.h"
#include "UART.h"

using namespace std;

std::string json_test = "{\"name\" : \"Jack\", \"age\" : 27, \"age\" : 27, \"age\" : 27, \"age\" : 27, \"age\" : 27}";

std::string json_test2 = "{\"menu\": { \"id\": \"file\", \"value\": \"File\", \"popup\": {\"menuitem\": [{\"value\": \"New\", \"onclick\": \"CreateNewDoc()\"}, {\"value2\": \"Open\", \"onclick\": \"OpenDoc()\"}, {\"value3\": \"Close\", \"onclick\": \"CloseDoc()\"}]}}}";

std::string json_test3 = "{\"menu\": {\"id\": -123.2345, \"value\": false, \"popup\": {\"menuitem\": {\"key1\": \"value1\", \"key2\": {\"key3\": \"value3\"}}}}}";

int main() {
    
    UART ser = UART("/dev/ttyUSB0");
    cout << ser.getDeviceName() << endl;
    ret_val r = ser.begin(57600);
    cout << r << endl;
    // if(r == SUCCESS) {
    //     char c;
    //     char &cr = c;
    //     for(;;) {
    //         // sleep(1);
    //         ret_val s = ser.readChar(cr);
    //         if(s == SUCCESS) {
    //             cout << c << endl;
    //         }
    //     }
    // }
    if(r == SUCCESS) {
        while(1) {
            std::string s;
            std::string &sr = s;
            ret_val ret = ser.readString(s);
            cout << ret << " " << s << endl;

            ret = ser.write(std::string("Hi there"));
            cout << ret << endl;
        }
    }
    return 0;
}