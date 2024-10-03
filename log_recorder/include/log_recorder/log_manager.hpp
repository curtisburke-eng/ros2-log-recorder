#ifndef LOG_MANAGER_HPP
#define LOG_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <sstream>
#include <iomanip>
#include <filesystem>

class LogManager : public rclcpp::Node {
public:
    LogManager(const std::string &nodeName);
    void init(); // New init function
    
private:
    // Declare members variables
    bool isRecording_ = false;
    std::unique_ptr<rosbag2_cpp::Writer> recorder_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startService_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopService_;
    // Declare Subscriptions variables
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfSub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfStaticSub_;
    // Declare Function prototypes
    void startRecording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void stopRecording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    std::string generateUniqueFilename();
    void tfSubCallback(std::shared_ptr<rclcpp::SerializedMessage> msg);
    void tfStaticSubCallback(std::shared_ptr<rclcpp::SerializedMessage> msg);
};

#endif // LOG_MANAGER_HPP
