#ifndef PROJECT_ALLEGRO_NODE_TORQUE_HPP
#define PROJECT_ALLEGRO_NODE_TORQUE_HPP


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "allegro_node.h"
// Forward class declaration.
class BHand;

// Joint-space torque control of the Allegro hand.
//
// Listens to allegroHand/torque_cmd topic and sets the desired joint torques
// directly.
//
// Author: Leo
//
class AllegroNodeTorque : public AllegroNode {
  public:
    AllegroNodeTorque(const std::string node_name);

    ~AllegroNodeTorque();

    void initController(const std::string &whichHand, const std::string &whichType);

    void computeDesiredTorque();

    void libCmdCallback(const std_msgs::msg::String::SharedPtr msg);

    void setTorqueCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    void doIt(bool polling);

  protected:
    // Handles external joint command (sensor_msgs/JointState).
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr torque_cmd_sub;

    // Handles defined on/off Torque commands (std_msgs/String).
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lib_cmd_sub;

    // Initialize BHand (custom controller do not use it)
    //BHand *pBHand = nullptr;

    // Desired joint torques (inherited from allegro_node)
    // double desired_torque[DOF_JOINTS] = {0.0};

    // switch on/off control torque
    bool controlTorque = false;
};

#endif //PROJECT_ALLEGRO_NODE_TORQUE_HPP
