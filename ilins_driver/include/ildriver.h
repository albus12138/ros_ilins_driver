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
        diagnostic_updater::Updater diagnostics_;
        boost::shared_ptr<Input> input_;
        ros::Publisher output_;
        boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
        double diag_min_freq_;
        double diag_max_freq_;
    };
}

#endif