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

#include "ilins_subscriber.h"

void NMEACallback(const ilins_msgs::ilinsNMEAPtr &msg) {
    ROS_INFO_STREAM("Get a packet at " << msg->timestamp);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ilins_subscriber_demo");
    ros::NodeHandle node;
    ros::Subscriber sub;
    if (1) {
        sub = node.subscribe("ilins_packets", 1000, NMEACallback);
    }
    ros::spin();
    return 0;
}