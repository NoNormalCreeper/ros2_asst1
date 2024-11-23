#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "turtlesim/msg/pose.hpp"
#include <tf2/LinearMath/Quaternion.h>

class TurtleBroadcaster : public rclcpp::Node
{
public:
    TurtleBroadcaster(std::string turtle_name) : Node("turtle_broadcaster"), turtle_name_(turtle_name)
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(turtle_name_ + "/velocity", 10);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            turtle_name_ + "/pose", 10,
            std::bind(&TurtleBroadcaster::poseCallback, this, std::placeholders::_1));
    }

private:
    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        // Broadcast the transform
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = turtle_name_;
        transformStamped.transform.translation.x = msg->x;
        transformStamped.transform.translation.y = msg->y;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(transformStamped);

        // Publish the velocity
        geometry_msgs::msg::TwistStamped twistStamped;
        twistStamped.header.stamp = this->get_clock()->now();
        twistStamped.header.frame_id = turtle_name_;
        twistStamped.twist.linear.x = msg->linear_velocity;
        twistStamped.twist.linear.y = 0.0;
        twistStamped.twist.linear.z = 0.0;
        twistStamped.twist.angular.x = 0.0;
        twistStamped.twist.angular.y = 0.0;
        twistStamped.twist.angular.z = msg->angular_velocity;
        velocity_publisher_->publish(twistStamped);
    }

    std::string turtle_name_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    if (argc != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Need turtle name as argument");
        return 1;
    }
    std::string turtle_name = argv[1];
    rclcpp::spin(std::make_shared<TurtleBroadcaster>(turtle_name));
    rclcpp::shutdown();
    return 0;
}