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
* Last revision: July 22, 2018
*
* For more info and how to use this library, visit: https://github.com/albus12138/ros_ilins_driver
*
***************************************************************************
*/

#ifndef _ILDRIVER_H_
#define _ILDRIVER_H_

#include <ros/ros.h>
#include <string.h>
#include <ilins_msgs/ilinsNMEA.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include "input.h"
using namespace std;

namespace il_driver {

    class ilinsDriver {
    public:
        ilinsDriver(ros::NodeHandle, ros::NodeHandle);

        ~ilinsDriver() {}

        bool poll(void);

    private:
        void callback();

        struct {
            int baudrate;
            int max_frequency;
        } config_;

        string deviceName;
        string protocol_type;
        diagnostic_updater::Updater diagnostics_;
        boost::shared_ptr<Input> input_;
        ros::Publisher output_;
        boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
        double diag_min_freq_;
        double diag_max_freq_;
    };
}

#endif