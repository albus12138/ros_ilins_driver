#include "serialport.h"

int main(){
    SerialPort::OpenOptions options = {
        true, 
        SerialPort::BR1152000, 
        SerialPort::DataBits8, 
        SerialPort::StopBits1,
        SerialPort::ParityNone,
        false,
        false,
        false,
        0,
        50,
    };
    SerialPort com("/dev/pts/19", options);
    const char str[13] = "Hello World!";
    com.write(str, 13);
    com.close();
}