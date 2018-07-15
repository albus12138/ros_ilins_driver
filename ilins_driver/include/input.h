#ifndef __ILINS_INPUT_H
#define __ILINS_INPUT_H

#include <string.h>
#include <signal.h>
#include <time.h>
#include <ros/ros.h>
#include <ilins_msgs/ilinsNMEA.h>
#include "serialport.h"
using namespace std;

namespace il_driver {
    static string DATA_SERIAL_PORT = "/dev/pts/19";

    class Input {
    public:
        Input(ros::NodeHandle);

        virtual ~Input() {}

        virtual int getPackage(ilins_msgs::ilinsNMEA *pkt) = 0;

    protected:
        ros::NodeHandle private_nh_;
        string serial_port_;
        SerialPort::OpenOptions options_ = SerialPort::defaultOptions;
    };

    class InputSocket : public Input {
    public:
        InputSocket(ros::NodeHandle private_nh);

        ~InputSocket(void);

        int getPackage(ilins_msgs::ilinsNMEA *pkt);
    
        int checksum(const char* s);

    private:
        SerialPort com_;
    };
}

#endif