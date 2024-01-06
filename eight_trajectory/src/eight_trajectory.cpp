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
#include <vector>

class TrajectoryController : public rclcpp::Node {
public:
    TrajectoryController() : Node("eight_trajectory_control") {
        // Publisher 
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_speed", 10);

        // Subscriber
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/rosbot_xl_base_controller/odom", 10,
                    std::bind(&TrajectoryController::odometryCallback, this, std::placeholders::_1));
        
        // Robot Dimensions
        l = 0.170/2;
        r = 0.100/2;
        w = 0.26969/2;

    }

    std::tuple<double, double, double> getTwistMessage(){
        double wz_ = result_[0];
        double vx_ = result_[1];
        double vy_ = result_[2];
        return std::make_tuple(wz_, vx_, vy_);
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

    // Función para calcular la distancia y la orientación entre dos puntos (x1, y1) y (x2, y2)
    std::tuple<double, double> calculateDistanceAndOrientation(std::vector<double> point1, std::vector<double> point2) {

        // Calcular la distancia
        double distance = std::sqrt(std::pow(point2[1] - point1[1], 2) + std::pow(point2[2] - point1[2], 2));

        // Calcular la orientación (yaw)
        double orientation = std::atan2(point2[2] - point1[2], point2[1] - point1[1]);

        // Adjust when yaw get out reach
        if (orientation < -2.30){
            std::cout << "OLD Yaw: " << orientation << std::endl; 
            orientation = -1.5708;
        }
        
        else if (orientation > 2.30){
            orientation = -1.885;
        }
        

        std::cout << "Distance: " << distance << " | Orientation: " << orientation << std::endl; 

        return std::make_tuple(distance, orientation);

    }

    double turnCommand(double orientation, bool turn, bool final){

        double turnVel;

        if (turn){
            turnVel = (orientation - calculate_yaw) * 0.30;
        }
        else if (final){
            turnVel = ((orientation + 0.4927) - calculate_yaw) * 0.70;
            std::cout << "Last Vel: " << turnVel << std::endl; 
        }
        else{
            turnVel = (orientation - calculate_yaw) * 0.10;
        }

        // Stabilize the command 
        if (std::abs(turnVel) < 0.05){
            turnVel = 0.0;
        }

        std::cout << "Turn velocity Command: " << turnVel << std::endl;
        return turnVel;
    }

    double forwardCommand(double act_dist){
        // Calculate the vel command and publish only 5%
        double vel = (act_dist) * 0.01;
        std::cout << "Forward velocity Command: " << vel << std::endl;
        return vel;
    }

    std::vector<double> getRobotPos(){
        std::vector<double> pos_vector = {0.0, 0.0, 0.0};
        pos_vector[0] = calculate_yaw;
        pos_vector[1] = current_pos_x;
        pos_vector[2] = current_pos_y;
        return pos_vector;
    }

    std::vector<double> calculateRelativeCordinates(const std::vector<double> point, std::vector<double> origen) {
        std::vector<double> new_origen = {0.0, 0.0, 0.0};
        new_origen[1] = point[1] - origen[1];
        new_origen[2] = point[2] - origen[2];     

        std::cout << "New Odom Target: " << std::endl;
        for (const auto& element : new_origen) {
            std::cout << element << " ";
        }
        std::cout << std::endl;

        return new_origen;
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

    long double calculateYaw(long double qx, long double qy, long double qz, long double qw) {

        long double yaw = std::atan2(2 * ((qw * qz) + (qx * qy)), 1 - 2 * ((qy * qy) + (qz * qz)));
      
        if (yaw > M_PI) {
            yaw -= 2 * M_PI;
        } else if (yaw < -M_PI) {
            yaw += 2 * M_PI;
        }

        return yaw;
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {   

        quaternion_x = msg->pose.pose.orientation.x;
        quaternion_y = msg->pose.pose.orientation.y;
        quaternion_z = msg->pose.pose.orientation.z;
        quaternion_w = msg->pose.pose.orientation.w;
        
        quaternion.x = quaternion_x;
        quaternion.y = quaternion_y;
        quaternion.z = quaternion_z;
        quaternion.w = quaternion_w;

        current_pos_x = msg->pose.pose.position.x;
        current_pos_y = msg->pose.pose.position.y;

        // Calculate Yaw
        calculate_yaw = calculateYaw(quaternion_x, quaternion_y, quaternion_z, quaternion_w);
        calculated_current_yaw_degree = (calculate_yaw * (180.0 / M_PI));
        std::cout << "Yaw: " << calculate_yaw << " rad" << std::endl;
        

        // Print current pos
        // std::cout << "X position: " << current_pos_x << " m" << std::endl;
        // std::cout << "Y position: " << current_pos_y << " m" << std::endl;
    }

    // Define Quarternion
    geometry_msgs::msg::Quaternion quaternion;
    long double quaternion_x, quaternion_y, quaternion_z, quaternion_w = 0.0;  

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

    // Current Yaw 
    long double current_yaw_rad = 0.0;
    long double current_yaw_degree = 0.0;
    long double calculate_yaw = 0.0;
    long double  calculated_current_yaw_degree = 0.0;   

    // Current Pose 
    double current_pos_x = 0.0;
    double current_pos_y = 0.0;

};

int main(int argc, char *argv[]) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    std::shared_ptr<TrajectoryController> trajectory_controller = std::make_shared<TrajectoryController>();
    
    // Indicate that nodes exists
    RCLCPP_INFO(trajectory_controller->get_logger(), "Trajectory Controller UP...");

    // Create executors
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add the odometry subscriber node to the executor
    executor.add_node(trajectory_controller);

    // Define Movement Variables
    double vx = 0.0, vz = 0.0;

    // Define Flag
    bool flag = false;

    // Time 
    int time; 

    // Define position vector
    std::vector<double> pos_vector = {0.0, 0.0, 0.0};
    std::vector<double> pos_target = {0.0, 0.0, 0.0};

    // Create frecuency Turn Controller
    rclcpp::Rate loop_rate(5);

    // Waypoints               
    //                        phi - x -  y
    std::vector<double> w0 = {0.0, 0.0, 0.0};
    std::vector<double> w1 = {0.0, 1.0, -1.0};
    std::vector<double> w2 = {0.0, 1.0, 1.0};
    std::vector<double> w3 = {0.0, 1.0, 1.0};
    std::vector<double> w4 = {1.5708, 1.0, -1.0};      
    std::vector<double> w5 = {-3.1415, -1.0, -1.0};
    std::vector<double> w6 = {0.0, -1.0, 1.0};
    std::vector<double> w7 = {0.0, -1.0, 1.0};
    std::vector<double> w8 = {0.0, -1.0, -1.0};

    std::vector<std::vector<double>> waypoints = {
        {0.0, 0.0, 0.0},
        {0.0, 1.0, -1.0},           // 1
        {0.0, 1.0, 1.0},            // 2
        {0.0, 1.0, 1.0},            // 3
        {0.0, 1.0, -1.0},           // 4
        {0.0, -1.0, -1.0},           // 5    
        {0.0, -1.0, 1.0},          // 6
        {0.0, -1.0, 1.0},          // 7
        {0.0, -1.0, -1.0}            // 8
    };
    
    for (size_t i = 0; i < waypoints.size(); ++i) {
        auto w0 = waypoints[0];
        auto w1 = waypoints[(i + 1) % waypoints.size()];  // Use the next waypoint as w1

        if (i < 1){
            time = 8;
        }
        else if (i > 3 && i < 5){
            time = 13; 
        }
        else if (i > 6){
            time = 11;
        }
        else{
            time = 10;
        }

        if (i > 7){
            std::cout << "Trajectory END" << std::endl;
            break;
        }
        
        // Waypoint information
        std::cout << "===========================" << std::endl;
        std::cout << "Waypoint " << i + 1 << std::endl;
        std::cout << "===========================" << std::endl;

        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < time && rclcpp::ok()) {
            double distance, orientation, target_orientation;

            if (!flag) {
                std::tie(distance, orientation) = trajectory_controller->calculateDistanceAndOrientation(w0, w1);
                target_orientation = orientation;
                flag = true;
            } else {
                std::tie(distance, orientation) = trajectory_controller->calculateDistanceAndOrientation(w0, w1);
                if (i > 4 && i < 7){
                    vz = trajectory_controller->turnCommand(target_orientation, true, false);
                }
                else if (i > 6){
                    vz = trajectory_controller->turnCommand(target_orientation, false, true);
                }
                else {
                    vz = trajectory_controller->turnCommand(target_orientation, false, false);
                }
                vx = trajectory_controller->forwardCommand(distance);
                trajectory_controller->twist_2_wheels(vz, vx, 0.0);
            }

            loop_rate.sleep();
            executor.spin_some();

        }
        flag = false;
    }

    // Shutdown the ROS 2 node
    rclcpp::shutdown();

    return 0;
}
