#include <termios.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>

struct termios tty;
int fd;
int flags = O_RDWR | O_NOCTTY | O_NONBLOCK;


// int read_test() {
// 	fd = open("/dev/ttyUSB0", flags);

// 	tcgetattr(fd, &tty);

// 	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
// 	tty.c_cflag |= B57600;  

// 	if (tcsetattr (fd, TCSANOW, &tty) != 0)
// 	{
// 	    fprintf (stderr, "error %d from tcsetattr", errno);
// 	    return -1;
// 	}
// 	void *buff[100];
// 	int readlen = 0;
//     readlen = read(fd, buff, 1);
//     if(readlen != 0)
//         printf("%c", *(char *)buff[0]);
// 	return 0;   
// }