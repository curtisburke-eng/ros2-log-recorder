#include "log_recorder/log_recorder.hpp"

LogRecorder::LogRecorder(const std::string &nodeName) : Node(nodeName) {
    RCLCPP_INFO(this->get_logger(), "Log Recorder Node Created");
}

void LogRecorder::init() {
    isRecording_ = false;
    // Service for starting recording
    startService_ = this->create_service<std_srvs::srv::Trigger>(
        "start_recording",
        std::bind(&LogRecorder::startRecording, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Service for stopping recording
    stopService_ = this->create_service<std_srvs::srv::Trigger>(
        "stop_recording",
        std::bind(&LogRecorder::stopRecording, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Subscribe to multiple topics
    tfSub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "tf", 10,std::bind(&LogRecorder::tfSubCallback, this, std::placeholders::_1));

    tfStaticSub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "tf_static", 10,std::bind(&LogRecorder::tfStaticSubCallback, this, std::placeholders::_1));


    RCLCPP_INFO(this->get_logger(), "Log Recorder Node Initialized");
}

// Service Trigger Functions
void LogRecorder::startRecording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (isRecording_) {
        response->success = false;
        response->message = "Recording is already in progress.";
        return;
    }
    // Initialize the recorder
    recorder_ = std::make_unique<rosbag2_cpp::Writer>();

    // Generate unique filename
    std::string filename = generateUniqueFilename();

    // Start recording to filename
    recorder_->open(filename);
    
    // Set recording flag
    isRecording_ = true;

    // Send debug messages
    response->success = true;
    response->message = "Started recording to " + filename;
    RCLCPP_INFO(this->get_logger(), "Started recording to: %s", filename.c_str());
}

void LogRecorder::stopRecording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!isRecording_) {
        response->success = false;
        response->message = "No recording is in progress.";
        return;
    }
    // Stop recording
    recorder_->close();
    // Set recording flag
    isRecording_ = false;

    // Send debug messages
    response->success = true;
    response->message = "Stopped recording.";
    RCLCPP_INFO(this->get_logger(), "Stopped recording.");
}

// Helper Functions
std::string LogRecorder::generateUniqueFilename() {
    // Declare local vars
    std::stringstream ss;

    // Set up the clock
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    // Create a stringstream with the current timestamp
    ss << "log_recording_" << std::put_time(std::localtime(&in_time_t), "%Y%m%d-%H%M%S");// << ".db3";

    // Console Log the stringstream converted to a string
    RCLCPP_INFO(this->get_logger(), "Generated filename: %s", ss.str().c_str());

    return ss.str();
}

// Subscription Callback functions
void LogRecorder::tfSubCallback(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    rclcpp::Time timestamp = this->now();
    if(isRecording_){
        recorder_->write(msg, "tf", "tf2_msgs/msg/TFMessage", timestamp);
    }
}

void LogRecorder::tfStaticSubCallback(std::shared_ptr<rclcpp::SerializedMessage> msg) {
    rclcpp::Time timestamp = this->now();
    if(isRecording_){
        recorder_->write(msg, "tf_static", "tf2_msgs/msg/TFMessage", timestamp);
    }
}



