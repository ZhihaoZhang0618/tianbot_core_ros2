#include "rover.h"
#include "protocol.h"

void TianbotRover::roverCallback(const std_msgs::msg::String::ConstPtr &msg)
{
    vector<uint8_t> buf;
    struct motion_mode motion_mode;
    uint8_t *out = (uint8_t *)&motion_mode;

    if (msg->data == "ackermann")
    {
        motion_mode.mode = MOVE_TYPE_ACKERMAN;
        //ROS_INFO("rover motion mode set to ackermann");
        RCLCPP_INFO(get_logger(), "rover motion mode set to ackermann");
    }
    else if (msg->data == "rotate")
    {
        motion_mode.mode = MOVE_TYPE_ROTATE;
        //ROS_INFO("rover motion mode set to rotate");
        RCLCPP_INFO(get_logger(), "rover motion mode set to rotate");
    }
    else if (msg->data == "omni")
    {
        motion_mode.mode = MOVE_TYPE_OMNI;
        //ROS_INFO("rover motion mode set to omni");
        RCLCPP_INFO(get_logger(), "rover motion mode set to omni");
    }
    else
    {
        //ROS_WARN("rover motion mode set failed, only ackermann / rotate / omni supported!");
        RCLCPP_WARN(get_logger(), "rover motion mode set failed, only ackermann / rotate / omni supported!");
    }

    buildCmd(buf, PACK_TYPE_SET_ROVER_MOTION_MODE, (uint8_t *)&motion_mode, sizeof(motion_mode));
    serial_.send(&buf[0], buf.size());
    heartbeat_timer_->cancel();
    heartbeat_timer_->reset();
}


TianbotRover::TianbotRover(const std::shared_ptr<rclcpp::Node> &node) : TianbotAckermann(node)
{
    rover_sub_ = node->create_subscription<std_msgs::msg::String>(
        "rover_motion_mode", 1, std::bind(&TianbotRover::roverCallback, this, std::placeholders::_1));

    initDone_ = true;
}
