#ifndef ALLEGRO_HAND_HW_INT_HPP
#define ALLEGRO_HAND_HW_INT_HPP
#include <atomic>
#include "hardware_interface/system.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/logger.hpp"

#include <std_srvs/srv/set_bool.hpp>
#include "allegro_hand_interfaces/srv/save_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


// #include <allegro_hand_driver/AllegroHandDrv.h>
#include <allegro_hand_hw_interface/AllegroHandDrv.h>
using namespace allegro;
#include "bhand/BHand.h"
#include <allegro_hand_hw_interface/kalman_filter_joint.hpp>

#define HW_IF_PRESSURE "pressure"
#define ALLEGRO_CONTROL_TIME_INTERVAL 0.002

namespace allegro_hand_hw_interface{

    enum ControlMode{
        OFF,
        POSITION,
        EFFORT
    };

    class AllegroHandV5: public hardware_interface::SystemInterface{
            public:
                AllegroHandV5();
                ~AllegroHandV5();
                CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
                CallbackReturn on_configure(const rclcpp_lifecycle::State& ) override;
                CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_error(const rclcpp_lifecycle::State&) override;

                std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
                std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

                hardware_interface::return_type read(const rclcpp::Time & , const rclcpp::Duration & period) override;
                hardware_interface::return_type write(const rclcpp::Time & , const rclcpp::Duration & period) override;

            private:
                double wrapAngleError(double q1, double q2, bool is_rad = true); // compute the angular error between dq = q1 - q2, considering the periodicity of the angle
 
                ControlMode controlMode = ControlMode::EFFORT;
                std::string whichHand;  // Right or left hand.
                std::string whichType;  // non-Geared or Geared hand.

                std::string jointNames[DOF_JOINTS] =
                    {
                        "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",     // index
                        "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",     // middle
                        "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",   // ring
                        "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0"  // thumb
                    };

                std::string pressureSensorNames[DOF_JOINTS] =  {"index_sensor", "middle_sensor", "ring_sensor", "thumb_sensor"};

                // BHand library
                size_t count;
                BHand *pBHand = NULL;
                long frame = 0;


                // Hand state
                kalman_filter_joint::KalmanFilterJoint kf[16];
                double position_offset[DOF_JOINTS] = {0.0};
                double current_joint_temperature[DOF_JOINTS] = {0.0};

                double current_position[DOF_JOINTS] = {0.0};
                double previous_position[DOF_JOINTS] = {0.0};

                double current_position_filtered[DOF_JOINTS] = {0.0};
                double previous_position_filtered[DOF_JOINTS] = {0.0};

                double current_velocity[DOF_JOINTS] = {0.0};
                double previous_velocity[DOF_JOINTS] = {0.0};
                double current_velocity_filtered[DOF_JOINTS] = {0.0};

                double desired_torque[DOF_JOINTS] = {0.0};
                double desired_position[DOF_JOINTS] = {0.0};

                double tactile_sensor[4] = {0.0};

                //int status_interval = 0;


                // ROS stuff
                std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

                // CAN device
                allegro::AllegroHandDrv *canDevice;
                std::string can_ch;

                std::mutex *mutex;


                // other
                bool pressure_req_ = true;
                bool first_cycle_ = true;
                bool new_data_ = false;          // a complete CAN frame was read in this cycle
                double elap_time = 0.0;   // time elapsed since the last complete CAN frame
                int num_state_interfaces ;

                // Flags
                int lEmergencyStop = 0;             // can check
                rclcpp::Node::SharedPtr activate_node_;
                rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr activate_service_;
                rclcpp::Service<allegro_hand_interfaces::srv::SavePose>::SharedPtr save_pose_service_;
                double position_snapshot_[DOF_JOINTS] = {0.0};   // last measured position, shared with service thread (guarded by mutex)
                bool position_received_ = false;                 // true once a complete CAN frame was read
                bool savePose(const std::string & name, const std::string & file_path, std::string & message);
                void sendZeroTorque();   // write zero torque to the hand (waits for a complete CAN frame)
                void stopServiceThread();   // stop allegro_hand_service executor and destroy its node
                std::atomic_bool active_{false};
                rclcpp::executors::SingleThreadedExecutor executor_;
                std::thread executor_thread_;

    };


} // namespace allegro_hand_hw_interface



#endif // ALLEGRO_HAND_HW_INT_HPP