#include "chassis.h"
#include "protocol.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>


void TianbotChasis::tianbotDataProc(unsigned char *buf, int len)
{
    if (!publisher_init_done_)
    {
        return;
    }
    struct protocol_pack *p = (struct protocol_pack *)buf;
    switch (p->pack_type)
    {
    case PACK_TYPE_ODOM_RESPONSE:
        if (sizeof(struct odom) == p->len - 2)
        {
            // nav_msgs::Odometry odom_msg;
            nav_msgs::msg::Odometry odom_msg;
            struct odom *pOdom = (struct odom *)(p->data);
            // ros::Time current_time = ros::Time::now();
            //rclcpp::Time current_time = rclcpp::Time(current_time);
            odom_msg.header.stamp = this->now();
            // odom_msg.header.frame_id = (nh_.getNamespace() + "/" + odom_frame_).erase(0,1);
            odom_msg.header.frame_id = odom_frame_;

            odom_msg.pose.pose.position.x = pOdom->pose.point.x;
            odom_msg.pose.pose.position.y = pOdom->pose.point.y;
            odom_msg.pose.pose.position.z = pOdom->pose.point.z;
            double yaw = pOdom->pose.yaw;
            geometry_msgs::msg::Quaternion q;
            q.x = 0.0;
            q.y = 0.0;
            q.z = sin(yaw / 2.0);
            q.w = cos(yaw / 2.0);            
            odom_msg.pose.pose.orientation = q;
            // set the velocity
            odom_msg.child_frame_id = base_frame_;
            odom_msg.twist.twist.linear.x = pOdom->twist.linear.x;
            odom_msg.twist.twist.linear.y = pOdom->twist.linear.y;
            odom_msg.twist.twist.linear.z = pOdom->twist.linear.z;
            odom_msg.twist.twist.angular.x = pOdom->twist.angular.x;
            odom_msg.twist.twist.angular.y = pOdom->twist.angular.y;
            odom_msg.twist.twist.angular.z = pOdom->twist.angular.z;
            // publish the message
            odom_pub_->publish(odom_msg);
            if (publish_tf_)
            {
                geometry_msgs::msg::TransformStamped odom_tf;
                odom_tf.header.stamp = this->now();
                odom_tf.header.frame_id = odom_frame_;
                odom_tf.child_frame_id = base_frame_;
                odom_tf.transform.translation.x = pOdom->pose.point.x;
                odom_tf.transform.translation.y = pOdom->pose.point.y;
                odom_tf.transform.translation.z = pOdom->pose.point.z;

               // odom_tf.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, pOdom->pose.yaw));
                // tf2::Quaternion q_tf;
                // q_tf.setRPY(0, 0, pOdom->pose.yaw);
                // odom_tf.transform.rotation.x = q_tf.x();
                // odom_tf.transform.rotation.y = q_tf.y();
                // odom_tf.transform.rotation.z = q_tf.z();
                // odom_tf.transform.rotation.w = q_tf.w();

                odom_tf_.transform.rotation = odom_msg.pose.pose.orientation;

                //odom_tf.transform.rotation.w = 1;

                tf_broadcaster_->sendTransform(odom_tf);
            }
        }
        break;  

    case PACK_TYPE_UWB_RESPONSE:
                break;
    
        // if (sizeof(struct uwb) == p->len - 2)
        // {

        //     auto pUwb = reinterpret_cast<struct uwb *>(p->data);

        //     geometry_msgs::msg::Pose2D pose2d_msg;
        //     pose2d_msg.x = pUwb->x_m;
        //     pose2d_msg.y = pUwb->y_m;
        //     pose2d_msg.theta = pUwb->yaw;
        //     uwb_pub_->publish(pose2d_msg);
        // }

    case PACK_TYPE_HEART_BEAT_RESPONSE:
        break;

    

    case PACK_TYPE_IMU_REPONSE:
        if (sizeof(struct imu_feedback) == p->len - 2)
        {

            auto pImu = reinterpret_cast<struct imu_feedback *>(p->data);
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = this->now();
            // imu_msg.header.frame_id = (nh_.getNamespace() + "/" + imu_frame_).erase(0,1);
            imu_msg.header.frame_id = imu_frame_;
            imu_msg.orientation.x = pImu->quat.x;
            imu_msg.orientation.y = pImu->quat.y;
            imu_msg.orientation.z = pImu->quat.z;
            imu_msg.orientation.w = pImu->quat.w;
            imu_msg.angular_velocity.x = pImu->angular_vel.x;
            imu_msg.angular_velocity.y = pImu->angular_vel.y;
            imu_msg.angular_velocity.z = pImu->angular_vel.z;
            imu_msg.linear_acceleration.x = pImu->linear_acc.x;
            imu_msg.linear_acceleration.y = pImu->linear_acc.y;
            imu_msg.linear_acceleration.z = pImu->linear_acc.z;
            imu_pub_->publish(imu_msg);
        }
        break;

    case PACK_TYPE_DEBUG_RESPONSE:
    {
        std_msgs::msg::String debug_msg;
        p->data[p->len - 2] = '\0';
        debug_msg.data = (char *)(p->data);
        debugResultStr_ = (char *)(p->data);
        debugResultFlag_ = true;
        debug_result_pub_->publish(debug_msg);
    }
    break;

    default:
        break;
    }
}

// TianbotChasis::TianbotChasis(ros::NodeHandle *nh) : TianbotCore(nh), publisher_init_done(false)
// {
//     nh_.param<std::string>("base_frame", base_frame_, DEFAULT_BASE_FRAME);
//     nh_.param<std::string>("odom_frame", odom_frame_, DEFAULT_ODOM_FRAME);
//     nh_.param<std::string>("imu_frame", imu_frame_, DEFAULT_IMU_FRAME);
//     nh_.param<bool>("publish_tf", publish_tf_, DEFAULT_PUBLISH_TF);

//     odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
//     imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1);
//     uwb_pub_ = nh_.advertise<geometry_msgs::Pose2D>("uwb", 1);

//     publisher_init_done = true;

//     odom_tf_.header.frame_id = odom_frame_;
//     odom_tf_.child_frame_id = base_frame_;
// }




TianbotChasis::TianbotChasis(const std::shared_ptr<rclcpp::Node> & node) : TianbotCore(node), publisher_init_done_(false)
{

    
    // 获取参数
    // node->get_parameter_or("base_frame", base_frame_, DEFAULT_BASE_FRAME);
    // node->get_parameter_or("odom_frame", odom_frame_, DEFAULT_ODOM_FRAME);
    // node->get_parameter_or("imu_frame", imu_frame_, DEFAULT_IMU_FRAME);
    // node->get_parameter_or("publish_tf", publish_tf_, DEFAULT_PUBLISH_TF);
   
    node->declare_parameter("base_frame", base_frame_);
    node->declare_parameter("odom_frame", odom_frame_);
    node->declare_parameter("imu_frame", imu_frame_);
    node->declare_parameter("publish_tf", publish_tf_);

    if (!node->get_parameter("base_frame", base_frame_)) {
        base_frame_ = DEFAULT_BASE_FRAME;
    }
    if (!node->get_parameter("odom_frame", odom_frame_)) {
        odom_frame_ = DEFAULT_ODOM_FRAME;
    }
    if (!node->get_parameter("imu_frame", imu_frame_)) {
        imu_frame_ = DEFAULT_IMU_FRAME;
    }
    if (!node->get_parameter("publish_tf", publish_tf_)) {
        publish_tf_ = DEFAULT_PUBLISH_TF;
    }   


    // 初始化发布者
    // rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("tianbot_core");
    odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    // uwb_pub_ = node->create_publisher<geometry_msgs::msg::Pose2D>("uwb", 1);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node);

    publisher_init_done_ = true;

    // 初始化TF
    odom_tf_.header.frame_id = odom_frame_;
    odom_tf_.child_frame_id = base_frame_;
}