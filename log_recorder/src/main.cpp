#include "log_recorder/log_recorder.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // Declare the node
    auto logRecorder = std::make_shared<LogRecorder>("log_recorder");
    // Init the node
    logRecorder->init(); 
    // Spin the node
    rclcpp::spin(logRecorder);
    rclcpp::shutdown();
    return 0;
}
