#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/detail/float64_multi_array__struct.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <iostream>
#include <ostream>
#include <thread>  // for std::this_thread::sleep_for


typedef std::vector<std::vector<double>> Matrix;
typedef std::vector<double> Vector;

class Controller : public rclcpp::Node {
public:
    Controller() : Node("robot_controller_transform") {
        // Publisher 
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

        // Subscriber
        subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/wheel_speed", 1,
                    std::bind(&Controller::wheel_vel_callback, this, std::placeholders::_1));

    }

private:

    void wheel_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {   
        if (msg->data.size() == 4) {
            u1 = msg->data[0];
            u2 = msg->data[1];
            u3 = msg->data[2];
            u4 = msg->data[3];

            u1 = u1/10;
            u2 = u2/10;
            u3 = u3/10;
            u4 = u4/10;

            vx = (u1 + u2 + u3 + u4) / 4.0;
            wz = (u2 - u4) / 2.0;
            vy = (u1 - u2 + u3 - u4) / 4.0;

            auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
            twist_msg->linear.x = vx;
            twist_msg->linear.y = vy;
            twist_msg->angular.z = wz;

            // RCLCPP_INFO(get_logger(), "X: %f\n Y: %f \n W: %f", vx, vy, wz);

            // Publish the twist message
            publisher_->publish(std::move(twist_msg));

        } else {
            RCLCPP_WARN(get_logger(), "Received invalid wheel speeds message size.");
        }
        
    }

    // Publisher Definition
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // Subscriber Definition
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_;

    // Receive Velocities
    double vx;
    double vy;
    double wz;

    // kinematic model data needed 
    double u1, u2, u3, u4;              // wheel velocities
   
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
