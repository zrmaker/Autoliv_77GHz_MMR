#ifndef _AUTOLIV_NODE_H_
#define _AUTOLIV_NODE_H_

#include <string>
#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
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
    void send_sync(uint8_t mode);
    void send_command(uint8_t sensor_nr);
    void receive_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void target_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void diagnostic_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void versions_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    uint16_t detections_count_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void visualize(const autoliv::Targets &msg);

    ros::Subscriber sub_can_;
    ros::Publisher pub_targets_;
    ros::Publisher pub_versions_;
    ros::Publisher pub_diagnostics_;
    ros::Publisher pub_rviz_autoliv_;
    ros::Publisher pub_can_;
    ros::Timer msg_timer;
    const float CYCLE_TIME = .04;
    uint8_t msg_counter_cnt = 0;
    uint8_t valid_detections_cnt = 0;
    uint8_t crc = 0;
};

}

#endif // _AUTOLIV_NODE_H_


