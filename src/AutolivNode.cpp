// Author: Renyuan Zhang & Fuheng Deng
// Rewrite the driver according to new documentation of 77GHz

#include "AutolivNode.h"

namespace octopus
{

uint8_t inline crc8(MsgSync *ptr){
    MsgSync tmp = *ptr;
    MsgSync *tmp_ptr = &tmp;
    uint64_t *msg = (uint64_t*)tmp_ptr;
    for(unsigned short i = 8; i > 0; --i)
        *msg = *msg & 0x80?((*msg << 1) ^ 0x31):(*msg << 1);
    uint8_t crc = (uint8_t)*msg;
    return crc;
}

AutolivNode::AutolivNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh){
    sub_can_ = node.subscribe("/can_rx", 10, &AutolivNode::receive_parser, this);
    pub_targets_ = node.advertise<autoliv::Targets>("/autoliv/targets", 10);
    pub_versions_ = node.advertise<autoliv::Versions>("/autoliv/versions", 10);
    pub_diagnostics_ = node.advertise<autoliv::Diagnostics>("/autoliv/diagnostics", 10);
    pub_rviz_autoliv_ = node.advertise<visualization_msgs::Marker>("/autoliv/markers", 1);
    pub_can_ = node.advertise<dataspeed_can_msgs::CanMessage>("/can_tx", 10);
    reset();
    msg_timer = node.createTimer(ros::Duration(CYCLE_TIME), &AutolivNode::communication_cycle, this);
}

AutolivNode::~AutolivNode(){}

void AutolivNode::reset(){
    int reset_cnt = 3; 
    while(ros::ok() && reset_cnt--){
        send_sync(MODE_RESET);
        msg_counter_cnt++;
    }
}
 
void AutolivNode::communication_cycle(const ros::TimerEvent& e){
    send_sync(MODE_LONG);
    for(int i = 1; i <= 4; ++i){
        ros::Duration(.00024).sleep();
        send_command(i);
        if(msg_counter_cnt++ > 15) msg_counter_cnt = 0;
    }
}

void AutolivNode::send_sync(uint8_t mode){
    dataspeed_can_msgs::CanMessage out;
    out.id = CAN_ID_SYNC;
    out.extended = false;
    out.dlc = 8;
    MsgSync *ptr = (MsgSync*)out.data.elems;
    memset(ptr, 0x00, sizeof(*ptr));
    ptr->sensor_1_mode = mode;
    ptr->sensor_2_mode = mode;
    ptr->sensor_3_mode = mode;
    ptr->sensor_4_mode = mode;
    ptr->msg_counter = msg_counter_cnt;
    ptr->data_channel_msb = 0x00;
    ptr->data_channel_lsb = 0x01;
    ptr->host_yaw_msb = 0x00;
    ptr->host_yaw_lsb = 0x0;
    ptr->host_velocity_msb = 0x0;
    ptr->host_velocity_lsb = 0x00;
    pub_can_.publish(out);
    crc = crc8(ptr);
}

void AutolivNode::send_command(uint8_t sensor_nr){
    dataspeed_can_msgs::CanMessage out;
    out.id = CAN_ID_COMMAND_SENSOR_N + sensor_nr;
    out.extended = false;
    out.dlc = 7;
    MsgCommand *ptr = (MsgCommand*)out.data.elems;
    memset(ptr, 0x00, sizeof(*ptr));
    ptr->msg_counter = msg_counter_cnt;
    ptr->meas_page_select = 2;
    ptr->data_channel_1_msb = 0;
    ptr->data_channel_1_lsb = 0;
    ptr->data_channel_2_msb = 0;
    ptr->data_channel_2_lsb = 0;
    ptr->sync_msg_content = crc;
    pub_can_.publish(out);
}

void AutolivNode::receive_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    if (msg->msg.id >= CAN_ID_TARGET_STARTER && msg->msg.id < CAN_ID_TARGET_ENDER && msg->msg.data.elems[0] != 0 && msg->msg.data.elems[1] != 0)    target_parser(msg);
    switch (msg->msg.id) {
        // case CAN_ID_DIAGNOSTIC_1:
        // case CAN_ID_DIAGNOSTIC_2:
        case CAN_ID_DIAGNOSTIC_3:   diagnostic_parser(msg); break;
        case CAN_ID_VERSIONS:       versions_parser(msg);   break;
        case CAN_ID_VALID_DETECTIONS:
            valid_detections_cnt = detections_count_parser(msg);
            // ROS_ERROR("valid detections: %d",valid_detections_cnt);
    }
}

void AutolivNode::target_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgTargets *ptr = (const MsgTargets*)msg->msg.data.elems;
    double range = ((double)(ptr->range_msb << 8) + (double)ptr->range_lsb) * .01;
    double velocity = ((double)(ptr->velocity_msb << 8) + (double)ptr->velocity_lsb) * .01 - 64;
    double bearing = ((double)(ptr->bearing_sign)*(-32768) + (double)(ptr->bearing_msb << 8) + (double)ptr->bearing_lsb) * 180 / 32768;
    double snr = 10 * std::log10((1 + ((double)(ptr->snr_sign)*(-128) + (double)ptr->snr)/127) / (1 - ((double)(ptr->snr_sign)*(-128) + (double)ptr->snr)/127)); // not sure

    double x = range * std::sin(bearing * M_PI / 180);
    double y = range * std::cos(bearing * M_PI / 180);
    double vx = velocity * std::sin(bearing * M_PI / 180);
    double vy = velocity * std::cos(bearing * M_PI / 180);

    autoliv::Targets pub_;
    pub_.header.frame_id = "autoliv";
    pub_.header.stamp = ros::Time::now();
    pub_.target_id = msg->msg.id - CAN_ID_TARGET_STARTER;
    pub_.x = x;
    pub_.y = y;
    pub_.vx = vx;
    pub_.vy = vy;
    pub_.range = range;  
    pub_.velocity = velocity;
    pub_.bearing = bearing;
    pub_.snr = snr;
    pub_.flags = ptr->flags;
    pub_targets_.publish(pub_);
    visualize(pub_);
}

void AutolivNode::diagnostic_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgDiagnostics *ptr = (const MsgDiagnostics*)msg->msg.data.elems;
    // ROS_ERROR("%X: [%X %X %X %X %X %X %X %X]",msg->msg.id,msg->msg.data.elems[0],msg->msg.data.elems[1],msg->msg.data.elems[2],msg->msg.data.elems[3],msg->msg.data.elems[4],msg->msg.data.elems[5],msg->msg.data.elems[6],msg->msg.data.elems[7]);
    // if (msg->msg.id == CAN_ID_DIAGNOSTIC_3) uint16_t diag_code = ((uint16_t)ptr->diag_code_msb << 8) | ptr->diag_code_lsb;
    uint16_t diag_code = ((uint16_t)ptr->diag_code_msb << 8) | ptr->diag_code_lsb;

    autoliv::Diagnostics pub_;
    pub_.header.frame_id = "autoliv";
    pub_.header.stamp = ros::Time::now();
    pub_.fault = diag_code < 1000 ? 1 : 0;
    pub_diagnostics_.publish(pub_);
}

void AutolivNode::versions_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgVersions *ptr = (const MsgVersions*)msg->msg.data.elems;
    uint32_t sw_version = ((uint32_t)ptr->sw_1 << 24) | ((uint32_t)ptr->sw_2 << 16) | ((uint32_t)ptr->sw_3 << 8) | (uint32_t)ptr->sw_4;
    uint32_t hw_version = ((uint32_t)ptr->hw_1 << 16) | ((uint32_t)ptr->hw_2 << 8) | (uint32_t)ptr->hw_3;

    autoliv::Versions pub_;
    pub_.header.frame_id = "autoliv";
    pub_.header.stamp = ros::Time::now();
    pub_.software_version = sw_version;
    pub_.software_subversion = ptr->sub;
    pub_.hardware_version = hw_version;
    pub_versions_.publish(pub_);
}

uint16_t AutolivNode::detections_count_parser(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgDetectionCount *ptr = (const MsgDetectionCount*)msg->msg.data.elems;
    return ((uint16_t)ptr->valid_detections_msb << 8) | ptr->valid_detections_lsb;
}

void AutolivNode::visualize(const autoliv::Targets &msg){
    visualization_msgs::Marker marker;

    marker.header.frame_id = "autoliv";
    marker.header.stamp = ros::Time::now();
    marker.id = msg.target_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration(CYCLE_TIME);
    marker.action = marker.ADD;

    marker.pose.position.x = msg.x;
    marker.pose.position.y = msg.y;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 0;

    marker.scale.x = 2;
    marker.scale.y = 2;
    marker.scale.z = 2;
    
    marker.color.a = 1;
    if (msg.flags == 1) {
        marker.color.r = 255;
        marker.color.g = 255;
        marker.color.b = 1;
    } else {
        marker.color.r = 1;
        marker.color.g = 255;
        marker.color.b = 1;
    }    
    pub_rviz_autoliv_.publish(marker);
}
}
