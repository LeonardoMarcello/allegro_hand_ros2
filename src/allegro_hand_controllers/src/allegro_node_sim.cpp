#include "allegro_node_sim.h"
#include <iostream>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>


#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

const std::string jointNames[DOF_JOINTS] =
      {
              "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",
              "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",
              "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",
              "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0"
      };

const double home_pose[DOF_JOINTS] =
        {
                // Default (HOME) position (degrees), set at system start if
                // no 'initial_position.yaml' parameter is loaded.
                0.0, -10.0, 45.0, 45.0,  0.0, -10.0, 45.0, 45.0,
                5.0, -5.0, 50.0, 45.0, 60.0, 25.0, 15.0, 45.0
        };


const std::string initialPosition[DOF_JOINTS] =
        {
                "~initial_position/j00", "~initial_position/j01",
                "~initial_position/j02",
                "~initial_position/j03",
                "~initial_position/j10", "~initial_position/j11",
                "~initial_position/j12",
                "~initial_position/j13",
                "~initial_position/j20", "~initial_position/j21",
                "~initial_position/j22",
                "~initial_position/j23",
                "~initial_position/j30", "~initial_position/j31",
                "~initial_position/j32",
                "~initial_position/j33"
        };



AllegroNodeSim::AllegroNodeSim(const std::string nodeName)
  : AllegroNode(nodeName, true)
{
  initController(whichHand, whichType);
}

AllegroNodeSim::~AllegroNodeSim() {
  RCLCPP_INFO(this->get_logger(), "Sim controller node is shutting down");
  rclcpp::shutdown();
}

void AllegroNodeSim::computeDesiredTorque() {
  // Just set current = desired.
  for (int idx = 0; idx < DOF_JOINTS; ++idx) {
    current_position[idx] = desired_joint_state.position[idx];
  }
}

void AllegroNodeSim::initController(const std::string &whichHand, const std::string &whichType) {
  // Initialize BHand controller
  if (whichHand == "left") {
    pBHand = new BHand(eHandType_Left);
    RCLCPP_WARN(this->get_logger(), "CTRL: Left Allegro Hand controller initialized.");
  } else {
    pBHand = new BHand(eHandType_Right);
    RCLCPP_WARN(this->get_logger(), "CTRL: Right Allegro Hand controller initialized.");
  }


  if(whichType == "A"){
    pBHand->GetType(eHardwareType_A);
    RCLCPP_WARN(this->get_logger(), "CTRL: A-Type Allegro Hand controller initialized.");
  } 
  else{
    pBHand->GetType(eHardwareType_B);
    RCLCPP_WARN(this->get_logger(), "CTRL: B-Type Allegro Hand controller initialized.");
  } 

  // set initial position via initial_position.yaml or to default values
  if (this->has_parameter("initial_position")) {
    RCLCPP_INFO(this->get_logger(), "CTRL: Initial Pose loaded from param server.");
    double tmp;
    mutex->lock();
    desired_joint_state.position.resize(DOF_JOINTS);
    for (int i = 0; i < DOF_JOINTS; i++) {
      this->get_parameter(initialPosition[i], tmp);
      desired_joint_state.position[i] = DEGREES_TO_RADIANS(tmp);
    }
    mutex->unlock();
  }
  else {
    RCLCPP_WARN(this->get_logger(),"CTRL: Initial position not loaded.");
    RCLCPP_WARN(this->get_logger(),"Check launch file is loading /parameters/initial_position.yaml");
    RCLCPP_WARN(this->get_logger(),"Loading Home position instead...");

    // Home position
    mutex->lock();
    desired_joint_state.position.resize(DOF_JOINTS);
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_joint_state.position[i] = DEGREES_TO_RADIANS(home_pose[i]);
    mutex->unlock();
  }

  RCLCPP_INFO(this->get_logger(), "***************************************");
  RCLCPP_INFO(this->get_logger(), "     Simulation Control Method         ");
  RCLCPP_INFO(this->get_logger(), "---------------------------------------");
  RCLCPP_INFO(this->get_logger(), "      Direct Joint Remapping           ");
  RCLCPP_INFO(this->get_logger(), "***************************************");
}

void AllegroNodeSim::updateController() {

  // Calculate loop time;
  tnow = get_clock()->now();
  dt = ALLEGRO_CONTROL_TIME_INTERVAL;//1e-9 * (tnow - tstart).nanoseconds();

  // When running gazebo, sometimes the loop gets called *too* often and dt will
  // be zero. Ensure nothing bad (like divide-by-zero) happens because of this.
  if(dt <= 0) {
    RCLCPP_DEBUG_STREAM_THROTTLE(rclcpp::get_logger("allegro_node"), *get_clock(), 1000, "AllegroNode::updateController dt is zero.");
    return;
  }

  tstart = tnow;

  // back-up previous joint positions:
  for (int i = 0; i < DOF_JOINTS; i++) {
    previous_position[i] = current_position[i];
    previous_position_filtered[i] = current_position_filtered[i];
    previous_velocity[i] = current_velocity[i];
  }


  // low-pass filtering:
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_position_filtered[i] = current_position[i];
    current_velocity[i] =
            (current_position[i] - previous_position[i]) / dt;
    current_velocity_filtered[i] =  current_velocity[i];;
  }

  // calculate control torque:
  computeDesiredTorque();

  // publish joint positions to ROS topic:
  publishData();

  frame++;
}

void AllegroNodeSim::doIt(bool polling) {
  auto this_node = std::shared_ptr<AllegroNodeSim>(this);
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
  AllegroNodeSim allegroNode("allegro_node_sim");
  allegroNode.doIt(polling);
}