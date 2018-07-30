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
* Last revision: July 30, 2018
*
* For more info and how to use this library, visit: https://github.com/albus12138/ros_ilins_driver
*
***************************************************************************
*/

#include "ildriver.h"

namespace il_driver {
    
    ilinsDriver::ilinsDriver(ros::NodeHandle node, ros::NodeHandle private_nh) {
        string dump_file;
        private_nh.param("protocol", protocol_type, string("NMEA"));
        private_nh.param("deviceName", deviceName, string(""));
        private_nh.param("baudrate", config_.baudrate, int(230400));
        private_nh.param("replay_file", dump_file, string(""));
        ROS_INFO_STREAM(dump_file);
        
        switch (config_.baudrate) {
        case 9600:
            config_.max_frequency = 9;
        case 19200:
            config_.max_frequency = 10;
        case 38400:
            config_.max_frequency = 30;
        case 115200:
            config_.max_frequency = 100;
        case 230400:
            config_.max_frequency = 200;
        case 460800:
            config_.max_frequency = 200;
        }

        diagnostics_.setHardwareID(deviceName);
        const double diag_freq = config_.max_frequency;
        diag_max_freq_ = diag_freq;
        diag_min_freq_ = diag_freq;
        ROS_INFO_STREAM("Expected maximum output frequency: " << diag_freq << " Hz");

        using namespace diagnostic_updater;
        diag_topic_.reset(new TopicDiagnostic("ilins_packets", diagnostics_,
                FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10), 
                TimeStampStatusParam()));

        if (!dump_file.empty()) {
            input_.reset(new il_driver::InputFile(private_nh, dump_file));
        } else {
            input_.reset(new il_driver::InputSocket(private_nh));
        }

        if (!protocol_type.compare("NMEA")) {
            output_ = node.advertise<ilins_msgs::ilinsNMEA>("ilins_packets", 10);
        } else if (!protocol_type.compare("OPVT2A")) {
            output_ = node.advertise<ilins_msgs::ilinsOPVT2A>("ilins_packets", 10);
        }
    }

    bool ilinsDriver::poll(void) {
        if (!protocol_type.compare("NMEA")) {
            ilins_msgs::ilinsNMEAPtr pkt(new ilins_msgs::ilinsNMEA);

            while (true) {
                int rc = input_->getPackage(&(*pkt));
                if (rc == 0) break;
            }

            ROS_INFO_STREAM("Publish a packet. " << pkt->timestamp);
            output_.publish(pkt);
            
            ros::Time timestamp(pkt->timestamp / 1000, (pkt->timestamp % 1000) * 1000000);
            diag_topic_->tick(timestamp);
            diagnostics_.update();
        } else if (!protocol_type.compare("OPVT2A")) {
            ilins_msgs::ilinsOPVT2APtr pkt(new ilins_msgs::ilinsOPVT2A);

            while (true) {
                int rc = input_->getPackage(&(*pkt));
                if (rc == 0) break;
            }

            ROS_INFO_STREAM("Publish a packet. " << pkt->timestamp << " " << pkt->voltage);
            output_.publish(pkt);

            ros::Time timestamp(pkt->timestamp / 1000, (pkt->timestamp % 1000) * 1000000);
            diag_topic_->tick(timestamp);
            diagnostics_.update();
        } else {
            ROS_ERROR_STREAM("Unknown protocol. Please check your lauchfile.");
        }

        return true;
    }
}