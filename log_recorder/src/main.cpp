#include "log_manager/log_manager.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // Declare the node
    auto logManager = std::make_shared<LogManager>("log_manager");
    // Init the node
    logManager->init(); 
    // Spin the node
    rclcpp::spin(logManager);
    rclcpp::shutdown();
    return 0;
}
