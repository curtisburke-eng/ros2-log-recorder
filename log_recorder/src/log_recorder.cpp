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
    LogManager()
        : Node("log_manager"), isRecording(false) {
        
        // Service for starting recording
        startService = this->create_service<std_srvs::srv::Trigger>(
            "start_log_recording",
            std::bind(&LogManager::startRecording, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Service for stopping recording
        stopService = this->create_service<std_srvs::srv::Trigger>(
            "stop_log_recording",
            std::bind(&LogManager::stopRecording, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Subscribe to multiple topics
        tfSub = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "tf", 10,std::bind(&LogManager::tfSubCallback, this, std::placeholders::_1));

        tfStaticSub = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "tf_static", 10,std::bind(&LogManager::tfStaticSubCallback, this, std::placeholders::_1));


        RCLCPP_INFO(this->get_logger(), "Log Manager Node Initialized");
    }

private:
    // Service Trigger Functions
    void startRecording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (isRecording) {
            response->success = false;
            response->message = "Recording is already in progress.";
            return;
        }

        // Generate unique filename
        std::string filename = generateUniqueFilename();
        // Start recording to filename
        recorder->open(filename);

        isRecording = true;
        response->success = true;
        response->message = "Started recording to " + filename;
        RCLCPP_INFO(this->get_logger(), "Started recording to %s", filename.c_str());
    }

    void stopRecording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!isRecording) {
            response->success = false;
            response->message = "No recording is in progress.";
            return;
        }
        // Stop recording
        recorder->close();
        isRecording = false;

        response->success = true;
        response->message = "Stopped recording.";
        RCLCPP_INFO(this->get_logger(), "Stopped recording.");
    }

    std::string generateUniqueFilename() {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);    
        std::stringstream ss;

        ss << "log_recording_" << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S") << ".db3";
        return ss.str();
    }

    // Subscription Callback functions
    void tfSubCallback(std::shared_ptr<rclcpp::SerializedMessage> msg) const {
        rclcpp::Time timestamp = this->now();

        recorder->write(msg, "tf", "tf2_msgs/msg/TFMessage", timestamp);
    }
    void tfStaticSubCallback(std::shared_ptr<rclcpp::SerializedMessage> msg) const {
        rclcpp::Time timestamp = this->now();

        recorder->write(msg, "tf_static", "tf2_msgs/msg/TFMessage", timestamp);
    }

    bool isRecording;
    std::shared_ptr<rosbag2_cpp::Writer> recorder;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startService;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopService;
    // Declare Subscriptions
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfSub;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfStaticSub;
    
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LogManager>());
    rclcpp::shutdown();
    return 0;
}
