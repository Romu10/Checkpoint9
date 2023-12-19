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
#include <cmath>

class TrajectoryController : public rclcpp::Node {
public:
    TrajectoryController() : Node("eight_trajectory_control") {
        // Publisher 
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_speed", 1);

        // Subscriber
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/rosbot_xl_base_controller/odom", 10,
                    std::bind(&TrajectoryController::odometryCallback, this, std::placeholders::_1));
        
        // Robot Dimensions
        l = 0.170/2;
        r = 0.100/2;
        w = 0.26969/2;

    }

    void velocity_2_twist(float phi, float dx, float dy) {
        
        R_ = {{1, 0, 0},
               {0,  std::cos(phi), std::sin(phi)},
               {0, -std::sin(phi), std::cos(phi)}};

        v_ = {phi, dx, dy};

        // Validate matrix and vector size for a correct dot product 
        if (R_.size() != 3 || R_[0].size() != 3 || v_.size() != 3) {
            std::cerr << "Error: Matrices must have the correct sizes for dot product calculation." << std::endl;
        }

        // Perform the dot product of the 3x3 matrix and the 3x1 matrix
        result_ = {0.0, 0.0, 0.0};
        for (size_t i = 0; i < result_.size(); ++i) {
            result_[i] = dotProduct(R_[i], v_);
        }

        // Print the result
        std::cout << "Twist message: ";
        for (double value : result_) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }

    std::tuple<double, double, double> getTwistMessage(){
        double wz_ = result_[0];
        double vx_ = result_[1];
        double vy_ = result_[2];
        return std::make_tuple(wz_, vx_, vy_);
    }

    void publishTwist(std::vector<double> publish_result_){
        // Publish in the topic
        vel_command.data = publish_result_;
        publisher_->publish(vel_command);
        RCLCPP_INFO(get_logger(), "Twist Message Published");

    }

    void twist_2_wheels(float wz, float vx, float vy) {
        
        // Estructural properties matrix
        matrix_ = {{(-l-w), 1.0, -1.0},
                    {(l+w), 1.0, 1.0},
                    {(l+w), 1.0, -1.0},
                    {(-l-w), 1.0, 1.0}};

        // Initialize result matrix with same size
        matrix_H_ = matrix_;        
        
        // Start procesing the matrix where matrix_H_ = matrix_/r
        for (size_t i = 0; i < matrix_.size(); ++i) {
            for (size_t j = 0; j < matrix_[i].size(); ++j) {
                matrix_H_[i][j] = matrix_[i][j] / r;
            }
        }
        
        // Initialize the twist in a vector (3x1)
        twist_ = {wz, vx, vy};

        // Validate matrix and vector size for a correct dot product 
        if (matrix_H_.size() != 4 || matrix_H_[0].size() != 3 || twist_.size() != 3) {
            std::cerr << "Error: Matrices must have the correct sizes for dot product calculation." << std::endl;
            // return 0;  // Return 1 in case of an error
        }

        // Perform the dot product of the 3x3 matrix and the 3x1 matrix
        std::vector<double> result = {0.0, 0.0, 0.0, 0.0};
        for (size_t i = 0; i < matrix_H_.size(); ++i) {
            result[i] = dotProduct(matrix_H_[i], twist_);
        }

        // Print the result
        std::cout << "Velocities for each wheel: ";
        for (double value : result) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
        

        // Publish in the topic
        vel_command.data = result;
        publisher_->publish(vel_command);
    }

private:

    double dotProduct(const std::vector<double>& vec1, const std::vector<double>& vec2) {
        if (vec1.size() != vec2.size()) {
            std::cerr << "Error: Vectors must have the same size for dot product calculation." << std::endl;
            return 0.0;  // Return 0 in case of an error
        }

        double result = 0.0;
        for (size_t i = 0; i < vec1.size(); ++i) {
            result += vec1[i] * vec2[i];
        }

        return result;
    }

    void getEulerAngles(const geometry_msgs::msg::Quaternion& quaternion) {
        geometry_msgs::msg::Vector3 euler_angles;
        quaternionToEuler(quaternion, euler_angles);
        std::cout << "Roll: " << euler_angles.x << ", Pitch: " << euler_angles.y << ", Yaw: " << euler_angles.z << std::endl;
    }

    void quaternionToEuler(const geometry_msgs::msg::Quaternion& quaternion, geometry_msgs::msg::Vector3& euler_angles) {

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

        if ((quaternion.z =! 0.0) && (quaternion.w != 0.0))
        {
            getEulerAngles(quaternion);
            // RCLCPP_INFO(get_logger(), "\n q_x: %f\n q_y: %f \n q_z: %f \n q_w: %f", quaternion.x, quaternion.y, quaternion.z, quaternion.w);
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

    // Movement Control message
    std_msgs::msg::Float64MultiArray vel_command;

    // Subscriber Definition
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;

    // Define matrix
    std::vector<std::vector<double>> R_;
    std::vector<double> v_;
    std::vector<double> result_;
    std::vector<std::vector<double>> matrix_;
    std::vector<std::vector<double>> matrix_H_;
    std::vector<double> twist_;

    // kinematic model data needed 
    double r;                               // wheel radius
    double w;                               // hall of track width 
    double l;                               // hall of the wheel base distance 

};

int main(int argc, char** argv) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    auto controller = std::make_shared<TrajectoryController>();

    // Define Movement Variables
    double vx = 0.0, vy = 0.0, phi = 0.0;

    // Define Twist Vector 
    std::vector<double> twist_vector_ = {0.0, 0.0, 0.0};

    // Waypoints  
    std::vector<double> w1 = {0.0, 1.0, -1.0};

    // Waypoint 1
    controller->velocity_2_twist(w1[0], w1[1], w1[2]);
    std::tie(phi, vx, vy) = controller->getTwistMessage();
    twist_vector_ = {phi, vx, vy};
    std::cout << "Pitch: " << phi << ", X: " << vx << ", Y: " << vy << std::endl;
    controller->twist_2_wheels(phi, vx, vy);



    // Spin to process callbacks (if any) and keep the node alive
    rclcpp::spin(controller);

    // Shutdown the ROS 2 node
    rclcpp::shutdown();

    return 0;
}
