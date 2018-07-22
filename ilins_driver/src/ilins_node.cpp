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
* Last revision: July 18, 2018
*
* For more info and how to use this library, visit: https://github.com/albus12138/ros_ilins_driver
*
***************************************************************************
*/

#include <ros/ros.h>
#include <signal.h>
#include "ildriver.h"

volatile sig_atomic_t flag = 1;
static void my_handler(int sig){
    flag = 0;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ins-driver");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    signal(SIGINT, my_handler);

    il_driver::ilinsDriver dvr(node, private_nh);
    while (ros::ok() && dvr.poll()){
        ros::spinOnce();
    }
    return 0;
}