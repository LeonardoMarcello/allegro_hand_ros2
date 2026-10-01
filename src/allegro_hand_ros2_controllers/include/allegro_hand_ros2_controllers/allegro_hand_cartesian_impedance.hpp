#ifndef CARTESIAN_IMPEDANCE_HPP_
#define CARTESIAN_IMPEDANCE_HPP_

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
#include "geometry_msgs/msg/pose_array.hpp"

#include "thunder_ahand/thunder_ahand_finger.h"
#include "thunder_ahand/thunder_ahand_thumb.h"

#include "bhand/BHand.h"

# define DOF_JOINTS 16
# define NUM_OF_FINGERS 4
namespace allegro_hand_cartesian_impedance
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using CmdMsg = geometry_msgs::msg::PoseArray;

class CartesianImpedance_Controller : public controller_interface::ControllerInterface
{
    public:
        CartesianImpedance_Controller(){};
        ~CartesianImpedance_Controller(){};

        CallbackReturn on_init() override;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::return_type update(
            const rclcpp::Time & , const rclcpp::Duration & period
        ) override;

    private:
        realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>> rt_command_ptr_;
        rclcpp::Subscription<CmdMsg>::SharedPtr joints_cmd_sub_;
        rclcpp::Publisher<CmdMsg>::SharedPtr controller_pub_;
        std::string logger_name_;

        std::string jointNames[DOF_JOINTS] =
            {
                "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",     // index
                "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",     // middle
                "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",   // ring
                "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0"  // thumb
            };
 
        // control gains
        //double Kp[DOF_JOINTS] = {};
        //double Kd[DOF_JOINTS] = {};
        std::vector<Eigen::Vector4d> Kp; 
        std::vector<Eigen::Vector4d> Kv;
        std::vector<Eigen::Vector3d> cartesan_stiffness; 
        std::vector<Eigen::Vector3d> cartesian_damping;


        // desired trajectory
        double x_des[3*NUM_OF_FINGERS] = {};
        double torque_des[3*NUM_OF_FINGERS] = {};
        CmdMsg joint_cmd_;

        thunder_ahand_finger thunder_index_handle_;
        thunder_ahand_finger thunder_middle_handle_;
        thunder_ahand_finger thunder_ring_handle_;
        thunder_ahand_thumb thunder_thumb_handle_;



        rclcpp::Publisher<CmdMsg>::SharedPtr pBHand_controller_pub_;
        size_t count;
        BHand *pBHand = NULL;
        long frame = 0;
        double pB_current_position[16] = {0.0};
        double pB_previous_position[16] = {0.0};

        double pB_current_position_filtered[16] = {0.0};
        double pB_previous_position_filtered[16] = {0.0};

        double pB_current_velocity[16] = {0.0};
        double pB_previous_velocity[16] = {0.0};
        double pB_current_velocity_filtered[16] = {0.0};

        double pB_torque[16] = {0.0};


};

}  // namespace allegro_hand_cartesian_impedance

#endif  // CARTESIAN_IMPEDANCE_HPP_