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