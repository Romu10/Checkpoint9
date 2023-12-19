#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/detail/float64_multi_array__struct.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <iostream>
#include <ostream>
#include <thread>  // for std::this_thread::sleep_for
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>


class TrajectoryController : public rclcpp::Node {
public:
    TrajectoryController() : Node("eight_trajectory_control") {
        // Publisher 
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_speed", 1);

        // Subscriber
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/rosbot_xl_base_controller/odom", 10,
                    std::bind(&TrajectoryController::odometryCallback, this, std::placeholders::_1));

    }


private:

    void getEulerAngles(const geometry_msgs::msg::Quaternion& quaternion) {
        geometry_msgs::msg::Vector3 euler_angles;
        quaternionToEuler(quaternion, euler_angles);
        std::cout << "Roll: " << euler_angles.x << ", Pitch: " << euler_angles.y << ", Yaw: " << euler_angles.z << std::endl;
    }

    void quaternionToEuler(const geometry_msgs::msg::Quaternion& quaternion, geometry_msgs::msg::Vector3& euler_angles) {

        RCLCPP_INFO(get_logger(), "\n q_x: %f\n q_y: %f \n q_z: %f \n q_w: %f", quaternion.x, quaternion.y, quaternion.z, quaternion.w);

        // Normalizar el cuaternión
        tf2::Quaternion tf_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        tf_quaternion.normalize();

        // Obtener los ángulos de Euler
        tf2::Matrix3x3(tf_quaternion).getRPY(euler_angles.x, euler_angles.y, euler_angles.z);
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {   
        quaternion.x = msg->pose.pose.orientation.x;
        quaternion.y = msg->pose.pose.orientation.y;
        quaternion.z = msg->pose.pose.orientation.z;
        quaternion.w = msg->pose.pose.orientation.w;
        RCLCPP_INFO(get_logger(), "\n q_x1: %f\n q_y1: %f \n q_z1: %f \n q_w1: %f", quaternion.x, quaternion.y, quaternion.z, quaternion.w);

        if ((quaternion.z =! 0.0) && (quaternion.w != 0.0))
        {
            getEulerAngles(quaternion);
            RCLCPP_INFO(get_logger(), "\n q_x: %f\n q_y: %f \n q_z: %f \n q_w: %f", quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        }
        else {
            RCLCPP_WARN(get_logger(), "No Data in the Quaternion Variable");
        }

    }

    // Define Quarternion
    geometry_msgs::msg::Quaternion quaternion;
    geometry_msgs::msg::Vector3 euler_angles;

    // Publisher Definition
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

    // Subscriber Definition
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;

};

int main(int argc, char** argv) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    auto controller = std::make_shared<TrajectoryController>();

    // Spin to process callbacks (if any) and keep the node alive
    rclcpp::spin(controller);

    // Shutdown the ROS 2 node
    rclcpp::shutdown();

    return 0;
}
