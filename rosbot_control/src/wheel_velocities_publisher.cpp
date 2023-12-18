#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/detail/float64_multi_array__struct.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class HoloController : public rclcpp::Node {
public:
    HoloController() : Node("robot_controller") {
        
        // Publisher 
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_velocities_publisher", 10);
    }

    void moveForward() {
        vel_command.data = moveforward; 
        publisher_->publish(vel_command);
        RCLCPP_INFO(get_logger(), "Moving Forward");
    }

    void moveBackward() {
        vel_command.data = movebackward; 
        publisher_->publish(vel_command);
        RCLCPP_INFO(get_logger(), "Moving Backward");
    }

    void moveRight() {
        vel_command.data = moveright; 
        publisher_->publish(vel_command);
        RCLCPP_INFO(get_logger(), "Moving Right");
    }

    void moveLeft() {
        vel_command.data = moveleft; 
        publisher_->publish(vel_command);
        RCLCPP_INFO(get_logger(), "Moving Left");
    }

    void turnRight() {
        vel_command.data = turnright; 
        publisher_->publish(vel_command);
        RCLCPP_INFO(get_logger(), "Turning Right");
    }

    void turnLeft() {
        vel_command.data = turnleft; 
        publisher_->publish(vel_command);
        RCLCPP_INFO(get_logger(), "Turning Left");
    }

private:

    // Publisher Definition
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    // Movement Control message
    std_msgs::msg::Float64MultiArray vel_command;

    // Movement Commands
    std::vector<double> moveforward = {1.0, 1.0, 1.0, 1.0};
    std::vector<double> movebackward = {-1, -1, -1, -1};
    std::vector<double> moveright = {1, -1, 1, -1};
    std::vector<double> moveleft = {-1, 1, -1, 1};
    std::vector<double> turnright = {1, -1, -1, 1};
    std::vector<double> turnleft = {-1, 1, 1, -1};
    std::vector<double> stop = {0, 0, 0, 0};


};

int main(int argc, char** argv) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    auto controller = std::make_shared<HoloController>();

    // Call a function from the YourNode class
    while (bool flag = true)
    {
        controller->moveForward();
    }

    // Your additional code logic goes here

    // Spin to process callbacks (if any) and keep the node alive
    rclcpp::spin(controller);

    // Shutdown the ROS 2 node
    rclcpp::shutdown();

    return 0;
}
