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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target,catcher;
    bool reach_flag;

    // Well, these two variables are not used in the code, but they may help in the task
    bool inital_flag;
    geometry_msgs::msg::TransformStamped tf_inital;

    void timer_callback()
    {

        geometry_msgs::msg::TransformStamped tf;     
        try
        {
            tf = tf_buffer_->lookupTransform(catcher, target, tf2::TimePointZero);

        } catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", catcher.c_str(), target.c_str(), ex.what());
            return;
        }

        // now the code was implemented to make the catcher follow the target
        // you should modify it to keep the relative position and orientation between the target and the catcher
        // you can first try to make the catcher keep a fixed distance from the target
        // then you can try to make the catcher keep a fixed angle from the target
        // these two tasks are not easy, but you can do it!

        auto t = tf.transform;
        auto message = geometry_msgs::msg::Twist();
        auto fixed_dist = 0.5;


        // 计算距离误差
        double distance_error = hypot(t.translation.x, t.translation.y);
        
        // 计算角度误差
        double angle_error = atan2(t.translation.y, t.translation.x);

        // 线性速度PID控制
        double error_diff_linear = distance_error - prev_error_linear;
        integral_linear += distance_error;
        double linear_speed = Kp_linear * distance_error + Ki_linear * integral_linear + Kd_linear * error_diff_linear;
        prev_error_linear = distance_error;

        // 角速度PID控制
        double error_diff_angular = angle_error - prev_error_angular;
        integral_angular += angle_error;
        double angular_speed = Kp_angular * angle_error + Ki_angular * integral_angular + Kd_angular * error_diff_angular;
        prev_error_angular = angle_error;

        if (hypot(t.translation.x, t.translation.y) < fixed_dist)
        {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            publisher_->publish(message);
            if (!reach_flag) RCLCPP_INFO(this->get_logger(), "Reached target");
            reach_flag = true;
            return;
        }
        else
        {
            reach_flag = false;
        }

        // message.linear.x = 0.5 * hypot(t.translation.x, t.translation.y);
        // message.angular.z = 1.0 * atan2(t.translation.y, t.translation.x);
        message.linear.x = linear_speed;
        message.angular.z = angular_speed;

        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x: '%f', angular.z: '%f'", message.linear.x, message.angular.z);
        publisher_->publish(message);

    }
public: 
    node(std::string target, std::string catcher): Node("follower"), target(target), catcher(catcher)
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&node::timer_callback, this));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(catcher+"/cmd_vel", 10);
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
    ROS_EVENT_LOOP(int argc, char *argv[], std::string target,std::string catcher)
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<node>(target,catcher));
    }
    ~ROS_EVENT_LOOP()
    {
        rclcpp::shutdown();
    }
};

int main(int argc, char *argv[])
{
    if (argc != 3 )
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: follow target catcher");
        return 1;
    }
    ROS_EVENT_LOOP(argc, argv, argv[1], argv[2]);
    return 0;
}