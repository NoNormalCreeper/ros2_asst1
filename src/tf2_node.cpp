#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose.hpp"
#include "turtlesim/msg/pose.hpp"
#include <algorithm>
#include <vector>
#include <string>

class node: public rclcpp::Node
{
private:
    tf2_ros::TransformBroadcaster tfb;
    std::vector<std::string> turtleNames;
    std::vector<rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr> subscriptions_;

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg, const std::string &turtleName)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = turtleName;
        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tfb.sendTransform(t);
    }

public:
    node(const std::vector<std::string> &turtles): Node("multi_turtle_broadcaster"), tfb(this), turtleNames(turtles)
    {
        for (const auto &turtleName : turtleNames)
        {
            auto subscription = this->create_subscription<turtlesim::msg::Pose>(
                turtleName + "/pose", 10, 
                [this, turtleName](const turtlesim::msg::Pose::SharedPtr msg) 
                {
                    this->pose_callback(msg, turtleName);
                });
            subscriptions_.push_back(subscription);
        }
        RCLCPP_INFO(this->get_logger(), "Multi-turtle broadcaster node started");
    }

    ~node()
    {
        RCLCPP_INFO(this->get_logger(), "Multi-turtle broadcaster node shutting down");
    }
};

class ROS_EVENT_LOOP
{
public:
    ROS_EVENT_LOOP(int argc, char *argv[], const std::vector<std::string> &turtleNames)
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<node>(turtleNames));
    }
    ~ROS_EVENT_LOOP()
    {
        rclcpp::shutdown();
    }
};

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: tf turtle_name1 [turtle_name2...]");
        return 1;
    }

    std::vector<std::string> turtleNames;
    for (int i = 1; i < argc; ++i)
    {
        turtleNames.push_back(argv[i]);
    }

    ROS_EVENT_LOOP(argc, argv, turtleNames);
    return 0;
}