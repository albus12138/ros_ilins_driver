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

#ifndef __ILINS_INPUT_H
#define __ILINS_INPUT_H

#include <string.h>
#include <signal.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <ilins_msgs/ilinsNMEA.h>
#include <ilins_msgs/ilinsOPVT2A.h>
#include "serialport.h"
using namespace std;

namespace il_driver {

    class Input {
    public:
        Input(ros::NodeHandle);

        virtual ~Input() {}

        virtual int getPackage(ilins_msgs::ilinsNMEA *pkt) = 0;

        virtual int getPackage(ilins_msgs::ilinsOPVT2A *pkt) = 0;

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

        int getPackage(ilins_msgs::ilinsOPVT2A *pkt);
    
        int checksum(const char* s);

    private:
        SerialPort com_;
        
    };

    class InputFile : public Input {
    public:
        InputFile(ros::NodeHandle private_nh, string dump_file);

        ~InputFile(void);

        int getPackage(ilins_msgs::ilinsNMEA *pkt);

        int getPackage(ilins_msgs::ilinsOPVT2A *pkt);
    
        int checksum(const char* s);

    private:
        ifstream in;
        string mode;
    };
}

#endif

