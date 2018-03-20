#ifndef _AUTOLIV_NODE_H_
#define _AUTOLIV_NODE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "dispatch.h"
#include <dataspeed_can_msgs/CanMessage.h>
#include <dataspeed_can_msgs/CanMessageStamped.h>
#include <autoliv/Targets.h>
#include <autoliv/Versions.h>
#include <autoliv/Diagnostics.h>

namespace octopus{

class AutolivNode{

public:

    AutolivNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
    ~AutolivNode();

private:
    
    void reset();
    void communication_cycle(const ros::TimerEvent& e);
    void send_sync(int mode);
    void send_command(int sensor_nr);

    void receive_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void target_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void diagnostic_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void versions_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    uint16_t detections_count_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);

    ros::Subscriber sub_can_;
    ros::Publisher pub_targets_;
    ros::Publisher pub_versions_;
    ros::Publisher pub_diagnostics_;
    ros::Publisher pub_can_;
    ros::Timer msg_timer;
    int msg_counter_cnt = 0;
    int valid_detections_cnt = 0;
    int crc = 0;
};

}

#endif // _AUTOLIV_NODE_H_


