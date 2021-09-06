#include "serial_port.h"
#include <fcntl.h>

int SerialPort::openPort() {
    struct termios termios_opt;
    const char* addr = path_.c_str();
    fd_ = open(addr, O_RDWR | O_NOCTTY| O_NDELAY);

    if(fd_ == -1)
        return -1;

    if((fcntl(fd_, F_SETFL, 0)) < 0) {
        return -1;
    }
    if(tcgetattr(fd_, &termios_opt) != 0) {
        return -1;
    }

    cfmakeraw(&termios_opt);
    //set speed
    cfsetispeed(&termios_opt, baudrate_);
    cfsetospeed(&termios_opt, baudrate_);

    //set databits
    termios_opt.c_cflag |= (CLOCAL|CREAD);
    termios_opt.c_cflag &= ~CSIZE;
    termios_opt.c_cflag |= CS8;

    //set parity
    termios_opt.c_cflag &= ~PARENB;
    termios_opt.c_iflag &= ~INPCK;

    //set stopbits
    termios_opt.c_cflag &= ~CSTOPB;
    termios_opt.c_cc[VTIME] = 0;
    termios_opt.c_cc[VMIN] = 1;
    tcflush(fd_,TCIFLUSH);

    if(tcsetattr(fd_, TCSANOW, &termios_opt) != 0) {
        return -1;
    }

    return 0;
}

int SerialPort::closePort() {
    if(close(fd_) < 0) {
        return -1;
    }
    else {
        return 0;
    }
}
