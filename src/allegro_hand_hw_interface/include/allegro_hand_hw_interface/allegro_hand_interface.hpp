#ifndef ALLEGRO_HAND_HW_INT_HPP
#define ALLEGRO_HAND_HW_INT_HPP
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
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

#include "tf2/tf2/LinearMath/Quaternion.h"
#include "tf2_ros/tf2_ros/transform_broadcaster.h"

#include <allegro_hand_driver/AllegroHandDrv.h>
using namespace allegro;
#include "bhand/BHand.h"

#define HW_IF_PRESSURE "pressure"
#define ALLEGRO_CONTROL_TIME_INTERVAL 0.002

namespace allegro_hand_hw_interface{
    class AllegroHand_Interface: public hardware_interface::SystemInterface{
            public:
                AllegroHand_Interface();
                ~AllegroHand_Interface();
                CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
                CallbackReturn on_configure(const rclcpp_lifecycle::State& ) override;
                CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;
                CallbackReturn on_error(const rclcpp_lifecycle::State&) override;

                std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
                std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

                hardware_interface::return_type read(const rclcpp::Time & , const rclcpp::Duration & ) override;
                hardware_interface::return_type write(const rclcpp::Time & , const rclcpp::Duration & ) override;

            private:
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

                double position_offset[DOF_JOINTS] = {0.0};

                double current_position[DOF_JOINTS] = {0.0};
                double previous_position[DOF_JOINTS] = {0.0};

                double current_position_filtered[DOF_JOINTS] = {0.0};
                double previous_position_filtered[DOF_JOINTS] = {0.0};

                double current_velocity[DOF_JOINTS] = {0.0};
                double previous_velocity[DOF_JOINTS] = {0.0};
                double current_velocity_filtered[DOF_JOINTS] = {0.0};

                double desired_torque[DOF_JOINTS] = {0.0};


                double current_joint_temperature[DOF_JOINTS] = {0.0};
                double tactile_sensor[4] = {0.0};

                int status_interval = 0;


                // ROS stuff
                std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

                // CAN device
                allegro::AllegroHandDrv *canDevice;
                std::string can_ch;
                std::mutex *mutex;
                BHand *pBHand = NULL;

                // Flags
                int lEmergencyStop = 0;
                long frame = 0;

                double motion_time = 1.0;
                double force_get = 2.0f;

                // ROS Time
                rclcpp::Time tstart;
                rclcpp::Time tnow;
                double dt;

                // other
                bool pressure_req_ = true;
                bool first_cycle_ = true;
                int num_state_interfaces ;


    };


} // namespace allegro_hand_hw_interface



#endif // ALLEGRO_HAND_HW_INT_HPP