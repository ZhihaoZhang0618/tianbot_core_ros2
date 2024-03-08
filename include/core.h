#ifndef __CORE_H__
#define __CORE_H__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tianbot_core/srv/debug_cmd.hpp"
#include <string>
#include "serial.h"
#define DEFAULT_SERIAL_DEVICE "/dev/ttyUSB0"
#define DEFAULT_SERIAL_BAUDRATE 460800
#define DEFAULT_TYPE "omni"
#define DEFAULT_TYPE_VERIFY true

using namespace std;

class TianbotCore : public rclcpp::Node
{
public:
    TianbotCore(const std::shared_ptr<rclcpp::Node> & node);
    Serial serial_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_result_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_cmd_sub_;
    rclcpp::Service<tianbot_core::srv::DebugCmd>::SharedPtr debug_cmd_srv_;

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    rclcpp::Service<tianbot_core::srv::DebugCmd>::SharedPtr param_set_;

    bool debugResultFlag_;
    string debugResultStr_;
    bool initDone_;

    void checkDevType();
    virtual void tianbotDataProc(unsigned char *buf, int len) = 0;


    void debugCmdCallback(const std_msgs::msg::String::ConstPtr &msg);

    bool debugCmdSrv(const std::shared_ptr<tianbot_core::srv::DebugCmd::Request> req, 
                            std::shared_ptr<tianbot_core::srv::DebugCmd::Response> res);

    rclcpp::TimerBase::SharedPtr communication_timer_;

    void serialDataProc(uint8_t *data, unsigned int data_len);
    void heartCallback();
    void communicationErrorCallback();

private:


    

};

#endif
