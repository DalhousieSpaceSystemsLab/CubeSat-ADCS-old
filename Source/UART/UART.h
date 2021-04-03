#ifndef UART_H
#define UART_H

#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "../Errors.h"

const uint MAX_BUFFER = 1000;       // Maximum receive buffer size for UART

enum UART_STATE {
    UART_OPEN,
    UART_CLOSED
};

class UART {
    public:
    UART(const char*);
    ~UART() {
        close(fd);
        fclose(f);
    }

    ret_val begin(uint speed);                                          // initializes UART with baudrate
    // ret_val available();
    ret_val flush();                                                    // waits till all the output is written
    // ret_val setTimeout(uint timeout);                                   // sets the timeout

    ret_val readChar(char &ch);                                         // read one character
    ret_val readString(std::string &str);                               // read string ending with a newline
    ret_val readBytesUntil(char end_char, char *buffer, uint length);   // read max length chars or until end_char
                                                                        // is found and save it in the buffer

    ret_val write(char ch);                                             // writes 1 character
    ret_val write(const char *buffer, uint length);                     // writes "length" chars from the buffer
    ret_val write(std::string);                                         // writes a string

    const char* getDeviceName();                                        // returns the device file name
    uint getSpeed();                                                    // returns current baudrate

    private:
    const char *dev_name = NULL;                                        // holds the device filename
    char rbuffer[MAX_BUFFER];                                           // receive buffer
    uint baudrate = 0;                                                  // current baudrate
    UART_STATE state = UART_CLOSED;                                     // current UART state

    struct termios tty;
    int fd;
    int flags = O_RDWR | O_NOCTTY | O_NONBLOCK;
    FILE *f;
};



#endif  // UART_H