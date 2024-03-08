#include "differential.h"
#include "protocol.h"

void TianbotDifferential::velocityCallback(const geometry_msgs::msg::Twist::ConstPtr &msg)
{
    uint16_t len;
    vector<uint8_t> buf;

    struct twist twist;
    uint8_t *out = (uint8_t *)&twist;
    twist.linear.x = msg->linear.x;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = msg->angular.z;

    buildCmd(buf, PACK_TYPE_CMD_VEL, (uint8_t *)&twist, sizeof(twist));
    serial_.send(&buf[0], buf.size());

    heartbeat_timer_->cancel();
    heartbeat_timer_->reset();
}


TianbotDifferential::TianbotDifferential(const std::shared_ptr<rclcpp::Node> &node) : TianbotChasis(node)
{
    cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&TianbotDifferential::velocityCallback, this, std::placeholders::_1));
    initDone_ = true;
}
