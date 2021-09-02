#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>

class SerialPort {
public:
    SerialPort(std::string pathname, uint32_t rate = B38400): path_(pathname), baudrate_(rate) {}

    ~SerialPort() {
        if(isOpen() != -1) {
            closePort();
        }
    }

    int openPort();
    int closePort();

    std::string getDevPath() { return path_; }
    int isOpen() { return fd_ >= 0 ? 0 : -1; }

    inline int readByte(uint8_t *data) { return read(fd_, data, 1); }
    inline int writeData(const uint8_t *buf, uint16_t length) { return write(fd_, buf, length); }

private:
    std::string path_;
    uint32_t baudrate_;
    int fd_;
};

#endif // SERIAL_PORT_H
