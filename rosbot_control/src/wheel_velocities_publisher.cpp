#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/detail/float64_multi_array__struct.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <iostream>
#include <ostream>
#include <thread>  // for std::this_thread::sleep_for

class HoloController : public rclcpp::Node {
public:
    HoloController() : Node("robot_controller") {
        
        // Publisher 
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_speed", 1);

        // Robot Dimensions
        l = 0.170/2;
        r = 0.100/2;
        w = 0.26969/2;
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
    
    void stopIt() {
        vel_command.data = stop; 
        publisher_->publish(vel_command);
        RCLCPP_INFO(get_logger(), "Stop");
    }

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

    void twist_2_wheels(float vx, float vy, float wz) {
        
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
        /*
        std::cout << "Velocities for each wheel: ";
        for (double value : result) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
        */

        // Publish in the topic
        vel_command.data = result;
        publisher_->publish(vel_command);
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

    // kinematic model data needed 
    double u_1, u_2, u_3, u_4;              // wheel velocities
    double r;                               // wheel radius
    double w;                               // hall of track width 
    double l;                               // hall of the wheel base distance 
    
    std::vector<std::vector<double>> matrix_;
    std::vector<std::vector<double>> matrix_H_;
    std::vector<double> twist_;

};

int main(int argc, char** argv) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    auto controller = std::make_shared<HoloController>();

    // Init message
    std::cout << "Initialized wheel velocities publisher node" << std::endl;

    // Go Forward
    auto start_time = std::chrono::steady_clock::now();
    std::cout << "Move forward" << std::endl;
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 3) {
        controller->twist_2_wheels(0.1, 0.0, 0.0);
    }

    // Go backward
    start_time = std::chrono::steady_clock::now();
    std::cout << "Move backward" << std::endl;
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 3) {
        controller->twist_2_wheels(-0.1, 0.0, 0.0);
    }

    // Go left
    start_time = std::chrono::steady_clock::now();
    std::cout << "Move left" << std::endl;
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 3) {
        controller->twist_2_wheels(0.0, -0.1, 0.0);
    }

    // Go right
    start_time = std::chrono::steady_clock::now();
    std::cout << "Move right" << std::endl;
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 3) {
        controller->twist_2_wheels(0.0, 0.1, 0.0);
    }

    // turn right
    start_time = std::chrono::steady_clock::now();
    std::cout << "Turn clockwise" << std::endl;
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 3) {
        controller->twist_2_wheels(0.0, 0.0, -0.5);
    }

    // turn left
    start_time = std::chrono::steady_clock::now();
    std::cout << "Turn counter-clockwise" << std::endl;
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 3) {
        controller->twist_2_wheels(0.0, 0.0, 0.5);
    }

    controller->stopIt();

    // Spin to process callbacks (if any) and keep the node alive
    rclcpp::spin(controller);

    // Shutdown the ROS 2 node
    rclcpp::shutdown();

    return 0;
}
