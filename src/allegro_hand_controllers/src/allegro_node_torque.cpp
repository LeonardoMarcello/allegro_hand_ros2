#include "allegro_node_torque.h"
#include "candrv/candrv.h"
#include "allegro_hand_driver/AllegroHandDrv.h"
#include <iostream>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

int flag = 0;

// The only topic specific to the 'torque' controller is the envelop torque.
const std::string TORQUE_TOPIC = "allegroHand/torque_cmd";
std::string pkg1_path;
std::string data_path;

AllegroNodeTorque::AllegroNodeTorque(const std::string nodeName)
        : AllegroNode(nodeName){//,pBHand(nullptr) {
  initController(whichHand,whichType);

  torque_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
          TORQUE_TOPIC, 3, std::bind(&AllegroNodeTorque::setTorqueCallback, this, std::placeholders::_1));
  lib_cmd_sub = this->create_subscription<std_msgs::msg::String>(
          LIB_CMD_TOPIC, 1, std::bind(&AllegroNodeTorque::libCmdCallback, this, std::placeholders::_1));
  }

AllegroNodeTorque::~AllegroNodeTorque() {
  RCLCPP_INFO(this->get_logger(), "Torque controller node is shutting down");
  //delete pBHand;
}

void AllegroNodeTorque::libCmdCallback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "CTRL: Heard: [%s]", msg->data.c_str());
  const std::string lib_cmd = msg->data.c_str();
  
  // Only turns torque control on/off: listens to 'save' (space-bar) or 'on'
  // (not published).
  if (lib_cmd.compare("on") == 0 || lib_cmd.compare("save") == 0) {
    RCLCPP_INFO(this->get_logger(), "Torque control is on.");
    //pBHand->SetMotionType(eMotionType_JOINT_PD);
    controlTorque = true;
  }
  else if (lib_cmd.compare("off") == 0) {
    RCLCPP_INFO(this->get_logger(), "Torque control is off.");
    //pBHand->SetMotionType(eMotionType_NONE);
    controlTorque = false;
  }

}

// Called when a desired joint torque message is received
void AllegroNodeTorque::setTorqueCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {

  mutex->lock();
  for (int i = 0; i < DOF_JOINTS; i++){
    desired_torque[i] = msg->effort[i];
    // to do: add saturation!!
  }
  mutex->unlock();

  //controlTorque = true;
}

void AllegroNodeTorque::computeDesiredTorque() {
  if(!controlTorque) {
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_torque[i] = 0.0;
  }
  // When controlTorque is true, there is no need to do anything (desired_torque
  // is already set in the callback).
}

void AllegroNodeTorque::initController(const std::string &whichHand, const std::string &whichType) {
  // Initialize BHand controller
  if (whichHand == "left") {
    //pBHand = new BHand(eHandType_Left);
    RCLCPP_WARN(this->get_logger(), "CTRL: Left Allegro Hand controller initialized.");
  } else {
    //pBHand = new BHand(eHandType_Right);
    RCLCPP_WARN(this->get_logger(), "CTRL: Right Allegro Hand controller initialized.");
  }


  if(whichType == "A"){
    //pBHand->GetType(eHardwareType_A);
    RCLCPP_WARN(this->get_logger(), "CTRL: A-Type Allegro Hand controller initialized.");
  } 
  else{
    //pBHand->GetType(eHardwareType_B);
    RCLCPP_WARN(this->get_logger(), "CTRL: B-Type Allegro Hand controller initialized.");
  } 

  //pBHand->SetTimeInterval(ALLEGRO_CONTROL_TIME_INTERVAL);
  //pBHand->SetMotionType(eMotionType_NONE);

  // sets initial deactivation
  controlTorque = false;
  for (int i = 0; i < DOF_JOINTS; i++)
    desired_torque[i] = 0.0;

  RCLCPP_INFO(this->get_logger(), "***************************************");
  RCLCPP_INFO(this->get_logger(), "     Joint Torque Control Method       ");
  RCLCPP_INFO(this->get_logger(), "---------------------------------------");
  RCLCPP_INFO(this->get_logger(), "    Only 'O' (off), 'S' (on) work.     ");
  RCLCPP_INFO(this->get_logger(), "***************************************");
}

void AllegroNodeTorque::doIt(bool polling) {
  auto this_node = std::shared_ptr<AllegroNodeTorque>(this);
  int control_cycle = (int)(1/ALLEGRO_CONTROL_TIME_INTERVAL);
  rclcpp::Rate rate(control_cycle);
  if (polling) {
    RCLCPP_INFO(this->get_logger(), " Polling = true.");
    while (rclcpp::ok()) {
      updateController();
      rclcpp::spin_some(this_node);
      rate.sleep();
    }
  } else {

    RCLCPP_INFO(this->get_logger(), "Polling = false.");
    // Timer callback
    rclcpp::TimerBase::SharedPtr timer = startTimerCallback();
    rclcpp::spin(this_node);
  }
}

int main(int argc, char **argv) {
  auto clean_argv = rclcpp::init_and_remove_ros_arguments(argc, argv); 

  bool polling = false;
  if (clean_argv.size() > 1 && clean_argv[1] == std::string("true")) {
    polling = true;
  }
  //printf("Start controller with polling = %d\n", polling);
  //bool is_sim = std::find(clean_argv.begin(), clean_argv.end(), "--sim") != clean_argv.end();
  AllegroNodeTorque allegroNode("allegro_node_torque");
  allegroNode.doIt(polling);
}
