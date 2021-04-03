#include "UART.h"

UART::UART(const char* device_name) {
    dev_name = device_name;
}

ret_val UART::begin(uint speed) {
    baudrate = speed;
    switch (speed)
    {
    case 9600:
        speed = B9600;
        break;
    case 19200:
        speed = B19200;
        break;
    case 38400:
        speed = B38400;
        break;
    case 57600:
        speed = B57600;
        break;
    case 115200:
        speed = B115200;
        break;
    case 230400:
        speed = B230400;
        break;
    case 500000:
        speed = B500000;
        break;
    case 1000000:
        speed = B1000000;
        break;
    case 2000000:
        speed = B2000000;
        break;
    case 4000000:
        speed = B4000000;
        break;
    default:
        baudrate = 0;
        return ERR_INVALID_ARG;
    }
    
    // std::cout << dev_name << std::endl;
    fd = open(dev_name, flags);
    // std::cout << fd << std::endl;
    f = fdopen(fd, "r+");

	tcgetattr(fd, &tty);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
	tty.c_cflag |= baudrate;  

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
	    std::cout << "UART: error " << errno << " from tcsetattr" << std::endl;
        close(fd);
        fclose(f);
	    return FAIL;
	}

    state = UART_OPEN;
    return SUCCESS;
}

ret_val UART::readChar(char &ch) {
    if(state == UART_CLOSED) {
        return FAIL;
    } 
    char buff;
	int readlen = 0;
	readlen = fread(&buff, sizeof(char), 1, f);
	if(readlen != 0) {
		ch = buff;
        return SUCCESS;
	}
	return FAIL;
}

ret_val UART::readBytesUntil(char end_char, char *buffer, uint length) {
    if(state == UART_CLOSED) {
        return FAIL;
    } 
    uint i = 0;
    char c;
    char &cr = c;
    while(i < length) {
        ret_val ret = readChar(cr);
        if(ret == SUCCESS) {
            *buffer = c;
            if(c == end_char) {
                *buffer++ = '\0';
                return SUCCESS;
            }
            buffer++;
            i++;
        }
    }
    return SUCCESS;
}

ret_val UART::readString(std::string &str) {
    if(state == UART_CLOSED) {
        return FAIL;
    } 
    ret_val ret = readBytesUntil('\n', rbuffer, MAX_BUFFER);
    str = std::string(rbuffer);
    return ret;
}

ret_val UART::write(const char *buffer, uint length) {
    if(state == UART_CLOSED) {
        return FAIL;
    } 
    size_t ret = fwrite(buffer, sizeof(char), length, f);
    if(ret != length) {
        return FAIL;
    }
    return SUCCESS;
}

ret_val UART::write(char ch) {
    return write(&ch, 1);
}

ret_val UART::write(std::string str) {
    return write(str.c_str(), str.length());
}

const char* UART::getDeviceName() {
    return dev_name;
}

uint UART::getSpeed() {
    return baudrate;
}