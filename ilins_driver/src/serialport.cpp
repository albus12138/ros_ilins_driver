/*
***************************************************************************
*
* Author: oyoung
*
* Copyright (C) 2018 oyoung
*
* Email: 
*
***************************************************************************
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
***************************************************************************
*
* Last revision: May, 2018
*
* For more info and how to use this library, visit: https://blog.csdn.net/oyoung_2012/article/details/80403258
*
***************************************************************************
*/

#include "serialport.h"

const SerialPort::OpenOptions SerialPort::defaultOptions = {
    true, //        bool autoOpen;
    SerialPort::BR9600, //    BaudRate baudRate;
    SerialPort::DataBits8, //    DataBits dataBits;
    SerialPort::StopBits1, //    StopBits stopBits;
    SerialPort::ParityNone,//    Parity parity;
    false,                  // input xon
    false,                  // input xoff
    false,                  // input xany
    0,                      // c_cc vmin
    50,                     // c_cc vtime
};

SerialPort::SerialPort(const std::string &path, const OpenOptions options)
    : _path(path), _open_options(options) {
    if(options.autoOpen) {
        _is_open = open(_path, _open_options);
    }
}


bool SerialPort::open() {
    return _is_open = open(_path, _open_options), _is_open;
}

bool SerialPort::open(const std::string &path, const OpenOptions &options) {

    if(_path != path) _path = path;
    if(_open_options != options) _open_options = options;

    _tty_fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(_tty_fd < 0) {
        return false;
    }

    struct termios  tios;
    termiosOptions(tios, options);
    tcsetattr(_tty_fd, TCSANOW, &tios);
    tcflush(_tty_fd, TCIOFLUSH);
    return true;
}

void SerialPort::termiosOptions(termios &tios, const OpenOptions &options) {


    tcgetattr(_tty_fd, &tios);

    tios.c_oflag = 0;
    tios.c_iflag = 0;
    tios.c_lflag = 0;

    cfsetispeed(&tios, options.baudRate);
    cfsetospeed(&tios, options.baudRate);

    tios.c_iflag |= (options.xon ? IXON : 0)
            | (options.xoff ? IXOFF: 0)
            | (options.xany ? IXANY : 0);

    // data bits

    int databits[] =  {CS5, CS6, CS7, CS8};
    tios.c_cflag &= ~0x30;
    tios.c_cflag |= databits[options.dataBits];

    // stop bits
    if(options.stopBits == StopBits2) {
        tios.c_cflag |= CSTOPB;
    } else {
        tios.c_cflag &= ~CSTOPB;
    }

    // parity
    if(options.parity == ParityNone) {
        tios.c_cflag &= ~PARENB;
    } else {
        tios.c_cflag |= PARENB;

        if(options.parity == PariteMark) {
            tios.c_cflag |= PARMRK;
        } else {
            tios.c_cflag &= ~PARMRK;
        }

        if(options.parity == ParityOdd) {
            tios.c_cflag |= PARODD;
        } else {
            tios.c_cflag &= ~PARODD;
        }
    }

    tios.c_cc[VMIN] = options.vmin;
    tios.c_cc[VTIME] = options.vtime;
}

bool SerialPort::isOpen() const {
    return _is_open;
}

int SerialPort::write(const void *data, int length) {
    return ::write(_tty_fd, data, length);
}

int SerialPort::read(void *data, int length) {
    return ::read(_tty_fd, data, length);
}

void SerialPort::close() {
    ::close(_tty_fd);
    _is_open = false;
}

SerialPort::BaudRate SerialPort::BaudRateMake(unsigned long baudrate) {
    switch (baudrate) {
    case 50:
        return BR50;
    case 75:
        return BR75;
    case 134:
        return BR134;
    case 150:
        return BR150;
    case 200:
        return BR200;
    case 300:
        return BR300;
    case 600:
        return BR600;
    case 1200:
        return BR1200;
    case 1800:
        return BR1800;
    case 2400:
        return BR2400;
    case 4800:
        return BR4800;
    case 9600:
        return BR9600;
    case 19200:
        return BR19200;
    case 38400:
        return BR38400;
    case 57600:
        return BR57600;
    case 115200:
        return BR115200;
    case 230400:
        return BR230400;
    case 460800:
        return BR460800;
    case 500000:
        return BR500000;
    case 576000:
        return BR576000;
    case 921600:
        return BR921600;
    case 1000000:
        return BR1000000;
    case 1152000:
        return BR1152000;
    case 1500000:
        return BR1500000;
    case 2000000:
        return BR2000000;
    case 2500000:
        return BR2500000;
    case 3000000:
        return BR3000000;
    case 3500000:
        return BR3500000;
    case 4000000:
        return BR4000000;
    default:
        break;
    }
    return BR0;
}


std::vector<std::string> SerialPort::list() {
    DIR *dir;
    struct dirent *ent;
    dir = opendir("/dev");
    std::vector<std::string> ttyList;

    while(ent = readdir(dir), ent != nullptr) {
        if("tty" == std::string(ent->d_name).substr(0, 3)) {
            ttyList.emplace_back(ent->d_name);
        }
    }

    return ttyList;
}
bool operator==(const SerialPort::OpenOptions& lhs, const SerialPort::OpenOptions& rhs)
{
    return lhs.autoOpen == rhs.autoOpen
            && lhs.baudRate == rhs.baudRate
            && lhs.dataBits == rhs.dataBits
            && lhs.parity == rhs.parity
            && lhs.stopBits == rhs.stopBits
            && lhs.vmin == rhs.vmin
            && lhs.vtime == rhs.vtime
            && lhs.xon == rhs.xon
            && lhs.xoff == rhs.xoff
            && lhs.xany == rhs.xany;
}

bool operator!=(const SerialPort::OpenOptions& lhs, const SerialPort::OpenOptions& rhs){
    return !(lhs == rhs);
}