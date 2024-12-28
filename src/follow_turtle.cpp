#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include <cmath>


// https://figoowen2003.github.io/2021/08/18/%E7%94%A8ROS2%E8%AE%BE%E8%AE%A1%E4%B8%80%E4%B8%AA%E8%B7%9F%E9%9A%8F%E7%9A%84%E5%B0%8F%E4%B9%8C%E9%BE%9F/
// PID控制参数
double Kp_linear = 0.5, Ki_linear = 0.0, Kd_linear = 0.05;
double Kp_angular = 4.0, Ki_angular = 0.0, Kd_angular = 0.2;

// 误差变量
double prev_error_linear = 0.0, integral_linear = 0.0;
double prev_error_angular = 0.0, integral_angular = 0.0;

class node: public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_, publisher2_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target, catcher1, catcher2;
    bool reach_flag1 = false, reach_flag2 = false;

    // Well, these two variables are not used in the code, but they may help in the task
    bool inital_flag;
    geometry_msgs::msg::TransformStamped tf_inital;

    double prev_error_linear1 = 0.0, integral_linear1 = 0.0;
    double prev_error_linear2 = 0.0, integral_linear2 = 0.0;
    double prev_error_angular1 = 0.0, integral_angular1 = 0.0;
    double prev_error_angular2 = 0.0, integral_angular2 = 0.0;

    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped tf1, tf2;
        try
        {
            tf1 = tf_buffer_->lookupTransform(catcher1, target, tf2::TimePointZero);
            tf2 = tf_buffer_->lookupTransform(catcher2, target, tf2::TimePointZero);
        } catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        // now the code was implemented to make the catcher follow the target
        // you should modify it to keep the relative position and orientation between the target and the catcher
        // you can first try to make the catcher keep a fixed distance from the target
        // then you can try to make the catcher keep a fixed angle from the target
        // these two tasks are not easy, but you can do it!

        auto t1 = tf1.transform;
        auto t2 = tf2.transform;
        auto message1 = geometry_msgs::msg::Twist();
        auto message2 = geometry_msgs::msg::Twist();
        auto fixed_dist = 0.5;
        auto fixed_angle = 10.0;

        // 计算距离误差
        double distance_error1 = hypot(t1.translation.x, t1.translation.y) - fixed_dist;
        double distance_error2 = hypot(t2.translation.x, t2.translation.y) - fixed_dist;

        // 计算角度误差
        double angle_error1 = atan2(t1.translation.y, t1.translation.x) - fixed_angle;
        double angle_error2 = atan2(t2.translation.y, t2.translation.x) - fixed_angle;

        // 线性速度PID控制
        double error_diff_linear1 = distance_error1 - prev_error_linear1;
        integral_linear1 += distance_error1;
        double linear_speed1 = Kp_linear * distance_error1 + Ki_linear * integral_linear1 + Kd_linear * error_diff_linear1;
        prev_error_linear1 = distance_error1;

        double error_diff_linear2 = distance_error2 - prev_error_linear2;
        integral_linear2 += distance_error2;
        double linear_speed2 = Kp_linear * distance_error2 + Ki_linear * integral_linear2 + Kd_linear * error_diff_linear2;
        prev_error_linear2 = distance_error2;

        // 角速度PID控制
        double error_diff_angular1 = angle_error1 - prev_error_angular1;
        integral_angular1 += angle_error1;
        double angular_speed1 = Kp_angular * angle_error1 + Ki_angular * integral_angular1 + Kd_angular * error_diff_angular1;
        prev_error_angular1 = angle_error1;

        double error_diff_angular2 = angle_error2 - prev_error_angular2;
        integral_angular2 += angle_error2;
        double angular_speed2 = Kp_angular * angle_error2 + Ki_angular * integral_angular2 + Kd_angular * error_diff_angular2;
        prev_error_angular2 = angle_error2;

        if (hypot(t1.translation.x, t1.translation.y) < fixed_dist)
        {
            message1.linear.x = 0.0;
            message1.angular.z = 0.0;
            publisher1_->publish(message1);
            if (!reach_flag1) RCLCPP_INFO(this->get_logger(), "Catcher1 reached target");
            reach_flag1 = true;
        }
        else
        {
            reach_flag1 = false;
            message1.linear.x = linear_speed1;
            message1.angular.z = angular_speed1;
            publisher1_->publish(message1);
        }

        if (hypot(t2.translation.x, t2.translation.y) < fixed_dist)
        {
            message2.linear.x = 0.0;
            message2.angular.z = 0.0;
            publisher2_->publish(message2);
            if (!reach_flag2) RCLCPP_INFO(this->get_logger(), "Catcher2 reached target");
            reach_flag2 = true;
        }
        else
        {
            reach_flag2 = false;
            message2.linear.x = linear_speed2;
            message2.angular.z = angular_speed2;
            publisher2_->publish(message2);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing: Catcher1 linear.x: '%f', angular.z: '%f'", message1.linear.x, message1.angular.z);
        RCLCPP_INFO(this->get_logger(), "Publishing: Catcher2 linear.x: '%f', angular.z: '%f'", message2.linear.x, message2.angular.z);
    }

public:
    node(std::string target, std::string catcher1, std::string catcher2): Node("follower"), target(target), catcher1(catcher1), catcher2(catcher2)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&node::timer_callback, this));
        publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>(catcher1 + "/cmd_vel", 10);
        publisher2_ = this->create_publisher<geometry_msgs::msg::Twist>(catcher2 + "/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Hello, world");
    }
    ~node()
    {
        RCLCPP_INFO(this->get_logger(), "Goodbye, world");
    }
};

class ROS_EVENT_LOOP
{
public:
    ROS_EVENT_LOOP(int argc, char *argv[], std::string target, std::string catcher1, std::string catcher2)
    {
        rclcpp::init(argc, argv);
        auto node1 = std::make_shared<node>(target, catcher1, catcher2);
        auto node2 = std::make_shared<node>(target, catcher1, catcher2);
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node1);
        executor.add_node(node2);
        executor.spin();
    }
    ~ROS_EVENT_LOOP()
    {
        rclcpp::shutdown();
    }
};

int main(int argc, char *argv[])
{
    if (argc != 4)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: follow target catcher1 catcher2");
        return 1;
    }
    ROS_EVENT_LOOP(argc, argv, argv[1], argv[2], argv[3]);
    return 0;
}