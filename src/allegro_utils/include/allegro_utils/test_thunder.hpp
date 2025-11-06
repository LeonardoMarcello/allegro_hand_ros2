#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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

    sensor_msgs::msg::JointState joint_cmd;
    sensor_msgs::msg::JointState joint_des;

    std::mutex *mutex;
    size_t count;

    kalman_filter_joint::KalmanFilterJoint kf[16];
    thunder_ahand_finger ahand_index;
    Eigen::VectorXd hat_pi;
    Eigen::VectorXd q_encoder;
    Eigen::VectorXd q;
	Eigen::VectorXd dq;
	Eigen::VectorXd ddq;
	Eigen::VectorXd q_r;
	Eigen::VectorXd dq_r;
	Eigen::VectorXd ddq_r;
	Eigen::VectorXd params;

};
