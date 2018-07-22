/*
***************************************************************************
*
* Author: r4phael
*
* Copyright (C) 2018 r4phael
*
* Email: albus.zly@gmail.com
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
* Last revision: July 23, 2018
*
* For more info and how to use this library, visit: https://github.com/albus12138/ros_ilins_driver
*
***************************************************************************
*/

#include "input.h"

extern sig_atomic_t flag;

namespace il_driver {
    static int packet_size_nmea = 93;
    static int packet_size_opvt2a = 101;
    
    Input::Input(ros::NodeHandle private_nh) :
            private_nh_(private_nh) {
        string protocol_type;
        int br;
        private_nh.param("protocol", protocol_type, string("NMEA"));
        ROS_INFO_STREAM("Protocol: " << protocol_type);

        if (!protocol_type.compare("NMEA")) {
            private_nh.param("NMEA_serial_port", serial_port_, string(""));
            private_nh.param("NMEA_BaudRate", br, int(230400));
            ROS_INFO_STREAM("NMEA");
        } else if (!protocol_type.compare("OPVT2A")) {
            private_nh.param("OPVT2A_serial_port", serial_port_, string(""));
            private_nh.param("OPVT2A_BaudRate", br, int(230400));
            ROS_INFO_STREAM("OPVT2A");
        }
        
        options_.baudRate = SerialPort::BaudRateMake(br);
        if (!serial_port_.empty()) {
            ROS_INFO_STREAM("Accepting packets from serial port: " << serial_port_);
        }
        ROS_INFO_STREAM("BaudRate: " << br);
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
        char *pch;
        char raw_data[packet_size_nmea];

        while (flag == 1) {
            do {
                nbytes = com_.read(raw_data, packet_size_nmea);
            } while (nbytes <= 0);

            if ((size_t) nbytes > 0) {
                if (raw_data[0] != '$') return -1;
                pch = strrchr(raw_data, '*');
                *pch = '\0';
                pch++;
                pkt->checksum = strtol(pch, &pch, 16);
                if (pkt->checksum != InputSocket::checksum(raw_data+1)) return -1;                

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

    int InputSocket::getPackage(ilins_msgs::ilinsOPVT2A *pkt) {
        int nbytes, checksum;
        bool status = true;
        char *pch;
        unsigned char raw_data[packet_size_opvt2a];

        while (flag == 1) {
            do {
                nbytes = com_.read(raw_data, packet_size_opvt2a);
            } while (nbytes <= 0);

            if ((size_t) nbytes == packet_size_opvt2a) {
                ros::serialization::IStream in_stream(raw_data, packet_size_opvt2a);
                ros::serialization::Serializer< ilins_msgs::ilinsOPVT2A >::read(in_stream, *pkt);
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

    InputFile::InputFile(ros::NodeHandle private_nh, string dump_file) : 
            Input(private_nh) {
        string protocol_type;
        private_nh.param("protocol", protocol_type, string("NMEA"));
        private_nh.param("mode", mode, string("once"));

        if (!protocol_type.compare("NMEA")) {
            in.open(dump_file.c_str(), ios::in);
        } else if (!protocol_type.compare("OPVT2A")) {
            in.open(dump_file.c_str(), ios::in|ios::binary);
        }

        if (!protocol_type.compare("once")) {
            ROS_INFO_STREAM("Worked in read_once mode.");
        } else {
            ROS_INFO_STREAM("Worked in loop mode.");
        }

        if (!in.is_open()) {
            ROS_ERROR_STREAM("Cannot open dump file " << dump_file);
        } else {
            ROS_INFO_STREAM("Restore data from file " << dump_file);
        }
    }

    InputFile::~InputFile(void) {
        in.close();
    }

    int InputFile::getPackage(ilins_msgs::ilinsNMEA *pkt) {
        int nbytes, checksum;
        char *pch;
        char raw_data[packet_size_nmea];

        while (flag == 1) {
            if (in.eof()) {
                if (!mode.compare("loop")) {
                    in.clear();
                    in.seekg(0, ios::beg);
                } else {
                    return -1;
                }
            }

            in.getline(raw_data, sizeof(raw_data));

            if (strlen(raw_data) != 0) {
                if (raw_data[0] != '$') return -1;
                pch = strrchr(raw_data, '*');
                *pch = '\0';
                pch++;
                pkt->checksum = strtol(pch, &pch, 16);
                if (pkt->checksum != InputFile::checksum(raw_data+1)) return -1;                

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

    int InputFile::getPackage(ilins_msgs::ilinsOPVT2A *pkt) {
        int nbytes, checksum;
        bool status = true;
        char *pch;
        char raw_data[packet_size_opvt2a];

        while (flag == 1) {
            if (in.eof()) {
                ROS_INFO_STREAM("1");
                if (!mode.compare("loop")) {
                    ROS_INFO_STREAM("2");
                    in.clear();
                    in.seekg(0, ios::beg);
                } else {
                    return -1;
                }
            }

            in.read(raw_data, sizeof(raw_data));

            ros::serialization::IStream in_stream((unsigned char *)raw_data, packet_size_opvt2a);
            ROS_INFO_STREAM(in_stream.getLength());
            ros::serialization::Serializer< ilins_msgs::ilinsOPVT2A >::read(in_stream, *pkt);
            break;
        }

        if (flag == 0) {
            abort();
        }

        return 0;
    }

    int InputFile::checksum(const char* s)
    {
        int c = 0;
        while (*s)
            c ^= *s++;
        return c;
    }
}