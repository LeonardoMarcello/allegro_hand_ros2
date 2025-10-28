#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "bhand/BHand.h"
#include "allegro_utils/thunder_ahand_finger.h"
#include "allegro_utils/kalman_filter_joint.hpp"
#include <eigen3/Eigen/Dense>

class DummyNode : public rclcpp::Node
{
public:
    DummyNode();

private:

    void timer_callback();
    void setJointDesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void setJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);


    rclcpp::TimerBase::SharedPtr timer_;

    // Forward joint command (sensor_msgs/JointState).
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_gc_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lib_cmd_pub_;

    // Handles external joint command (sensor_msgs/JointState).
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_des_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;


    std::string jointNames[16] =
        {
                "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",
                "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",
                "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",
                "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0"
        };

    double jointLimits [16][2] =
        {
            {-0.3,  0.3},    // Index 0
            {-0.01, 1.6},    // Index 1
            {-0.07, 1.86},   // Index 2
            {-0.02, 2.01},   // Index 3

            //{-0.26, 0.26},   // Middle 0
            //{-0.21, 1.79},   // Middle 1
            //{-0.12, 1.86},   // Middle 2
            //{-0.21, 1.85},   // Middle 3

            {-0.23, 0.23},   // Middle 0
            {-0.09, 1.76},   // Middle 1
            {-0.02, 1.83},   // Middle 2
            {-0.06, 1.82},   // Middle 3

            {-0.26, 0.29},   // Ring 0
            {-0.21, 1.79},   // Ring 1
            {-0.12, 1.86},   // Ring 2
            {-0.21, 1.85},   // Ring 3

            { 0.00, 1.78},   // Thumb 0
            {-0.26, 1.65},   // Thumb 1
            {-0.05, 1.85},   // Thumb 2
            {-0.09, 1.80}    // Thumb 3
        };


    sensor_msgs::msg::JointState joint_cmd;
    sensor_msgs::msg::JointState joint_gc_cmd;
    sensor_msgs::msg::JointState joint_des;

    std::mutex *mutex;
    size_t count;
    BHand *pBHand = NULL;
    long frame = 0;

    kalman_filter_joint::KalmanFilterJoint kf;
    thunder_ahand_finger ahand_index;
    Eigen::VectorXd q_encoder;
    Eigen::VectorXd q;
	Eigen::VectorXd dq;
	Eigen::VectorXd ddq;
	Eigen::VectorXd q_r;
	Eigen::VectorXd dq_r;
	Eigen::VectorXd ddq_r;
	Eigen::VectorXd params;




    double pB_current_position[16] = {0.0};
    double pB_previous_position[16] = {0.0};

    double pB_current_position_filtered[16] = {0.0};
    double pB_previous_position_filtered[16] = {0.0};

    double pB_current_velocity[16] = {0.0};
    double pB_previous_velocity[16] = {0.0};
    double pB_current_velocity_filtered[16] = {0.0};

    double pB_torque[16] = {0.0};

};
