#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/detail/float64_multi_array__struct.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <iostream>
#include <ostream>
#include <thread>  // for std::this_thread::sleep_for

class Controller : public rclcpp::Node {
public:
    Controller() : Node("robot_controller_transform") {
        // Publisher 
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber
        subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/wheel_speed", 10,
                    std::bind(&Controller::wheel_vel_callback, this, std::placeholders::_1));
    }

private:

    void wheel_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {   
        double vel_wheel_1 = msg->data[0];
        double vel_wheel_2 = msg->data[1];
        double vel_wheel_3 = msg->data[2];
        double vel_wheel_4 = msg->data[3];
    }

    // Publisher Definition
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // Subscriber Definition
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;

    // Receive Velocities
    double vel_wheel_1;
    double vel_wheel_2;
    double vel_wheel_3;
    double vel_wheel_4;
     
};

int main(int argc, char** argv) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    auto controller = std::make_shared<Controller>();

    // Spin to process callbacks (if any) and keep the node alive
    rclcpp::spin(controller);

    // Shutdown the ROS 2 node
    rclcpp::shutdown();

    return 0;
}
