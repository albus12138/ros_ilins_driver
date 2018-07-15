#include "ildriver.h"

namespace il_driver {
    
    ilinsDriver::ilinsDriver(ros::NodeHandle node, ros::NodeHandle private_nh) {
        private_nh.param("NMEA_baudrate", config_.baudrate, int(115200));
        
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

        string dump_file;
        private_nh.param("deviceName", deviceName, string(""));
        private_nh.param("dump_file", dump_file, string(""));
        diagnostics_.setHardwareID(deviceName);
        const double diag_freq = config_.max_frequency;
        diag_max_freq_ = diag_freq;
        diag_min_freq_ = diag_freq;
        ROS_INFO_STREAM("Expected output frequency: " << diag_freq << " Hz");

        using namespace diagnostic_updater;
        diag_topic_.reset(new TopicDiagnostic("ilins_packets", diagnostics_,
                FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10), 
                TimeStampStatusParam()));

        if (dump_file != "") {
            ROS_INFO_STREAM("TODO");
        } else {
            input_.reset(new il_driver::InputSocket(private_nh));
        }

        output_ = node.advertise<ilins_msgs::ilinsNMEA>("ilins_packets", 10);
    }

    bool ilinsDriver::poll(void) {
        ilins_msgs::ilinsNMEAPtr nmea(new ilins_msgs::ilinsNMEA);

        while (true) {
            int rc = input_->getPackage(&(*nmea));
            if (rc == 0) break;
        }

        ROS_INFO_STREAM("Publish a packet.");
        output_.publish(nmea);

        //diag_topic_->tick(nmea->timestamp);
        //diagnostics_.update();

        return true;
    }
}