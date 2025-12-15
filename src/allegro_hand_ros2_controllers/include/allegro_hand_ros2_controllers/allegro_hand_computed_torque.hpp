#ifndef COMPUTED_TORQUE_HPP_
#define COMPUTED_TORQUE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <Eigen/Core>
#include <chrono>
#include <mutex>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "allegro_hand_ros2_controllers/thunder_ahand_finger.h"
#include "allegro_hand_ros2_controllers/thunder_ahand_thumb.h"

# define DOF_JOINTS 16
namespace allegro_hand_computed_torque
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using CmdMsg = sensor_msgs::msg::JointState;

class ComputedTorque_Controller : public controller_interface::ControllerInterface
{
    public:
        ComputedTorque_Controller(){};
        ~ComputedTorque_Controller(){};

        CallbackReturn on_init() override;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::return_type update(
            const rclcpp::Time & time, const rclcpp::Duration & period
        ) override;

    private:
        realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>> rt_command_ptr_;
        rclcpp::Subscription<CmdMsg>::SharedPtr joints_cmd_sub_;
        std::string logger_name_;

        std::string jointNames[DOF_JOINTS] =
            {
                "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",     // index
                "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",     // middle
                "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",   // ring
                "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0"  // thumb
            };
 
        // control gains
        double Kp[DOF_JOINTS] = {};
        double Kd[DOF_JOINTS] = {};


        // desired trajectory
        double q_des[DOF_JOINTS] = {};
        double qd_des[DOF_JOINTS] = {};
        double qdd_des[DOF_JOINTS] = {};
        double torque_des[DOF_JOINTS] = {};
        sensor_msgs::msg::JointState joint_cmd_;

        thunder_ahand_finger thunder_index_handle_;
        thunder_ahand_finger thunder_middle_handle_;
        thunder_ahand_finger thunder_ring_handle_;
        thunder_ahand_thumb thunder_thumb_handle_;

};

}  // namespace allegro_hand_computed_torque

#endif  // COMPUTED_TORQUE_HPP_