#include "input.h"

extern sig_atomic_t flag;

namespace il_driver {
    
    Input::Input(ros::NodeHandle private_nh) :
            private_nh_(private_nh) {
        private_nh.param("NMEA_serial_port", serial_port_, string(""));
        if (!serial_port_.empty()) {
            ROS_INFO_STREAM("Accepting packets from serial port: " << serial_port_);
        }

        int br;
        private_nh.param("NMEA_BaudRate", br, int(115200));
        ROS_INFO_STREAM("BaudRate: " << br);
        options_.baudRate = SerialPort::BaudRateMake(br);
    }

    InputSocket::InputSocket(ros::NodeHandle private_nh) :
            Input(private_nh) {
        if (!serial_port_.empty()) {
            com_ = SerialPort(serial_port_, options_);
        }
        if (com_.isOpen()) {
            ROS_INFO_STREAM("Connected to serial port \"" << serial_port_ << "\"");
        } else {
            ROS_ERROR_STREAM("Open serial port failed.");
        }
    }

    InputSocket::~InputSocket(void) {
        com_.close();
    }

    int InputSocket::getPackage(ilins_msgs::ilinsNMEA *pkt) {
        int nbytes, checksum;
        bool status = true;
        char *pch;
        char raw_data[93];

        while (flag == 1) {
            do {
                nbytes = com_.read(raw_data, 93);
            } while (nbytes <= 0);

            if ((size_t) nbytes == 93) {
                if (raw_data[0] != '$') return -1;
                pch = strrchr(raw_data, '*');
                *pch = '\0';
                pch++;
                pkt->checksum = strtol(pch, &pch, 16);
                if (pkt->checksum != InputSocket::checksum(raw_data)) return -1;

                pch = strtok(raw_data, ",");
                if (pch != NULL && !strcmp(pch, "$PAPR")) {
                    pkt->message_id = pch;
                    pch = strtok(NULL, ",");
                    pkt->latitude = atof(pch);
                    pch = strtok(NULL, ",");
                    pkt->latitude_dir = *pch;
                    pch = strtok(NULL, ",");
                    pkt->longitude = atof(pch);
                    pch = strtok(NULL, ",");
                    pkt->longitude_dir = *pch;
                    pch = strtok(NULL, ",");
                    pkt->altitude = atof(pch);
                    pch = strtok(NULL, ",");
                    pkt->altitude_type = *pch;
                    pch = strtok(NULL, ",");
                    pkt->roll = atof(pch);
                    pch = strtok(NULL, ",");
                    pkt->pitch = atof(pch);
                    pch = strtok(NULL, ",");
                    pkt->heading = atof(pch);
                    pch = strtok(NULL, ",");
                    pkt->timestamp = atol(pch);
                    pch = strtok(NULL, ",");
                    pkt->temperature = atof(pch);
                    pch = strtok(NULL, ",");
                    pkt->voltage = atof(pch);
                    pch = strtok(NULL, ",");
                    pkt->status = string(pch);
                }

                break;
            }
        }

        if (flag == 0) {
            abort();
        }

        return 0;
    }

    int InputSocket::checksum(const char* s)
    {
        int c = 0;
        while (*s)
            c ^= *s++;
        return c;
    }
}