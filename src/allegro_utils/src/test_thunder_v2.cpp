#include "allegro_utils/test_thunder_v2.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

DummyNode::DummyNode()
: Node("dummy_node")
  //kf(1e-1, 1e-7) // n_of_joint, process noise, measurement noise
  //kf(16, 1e-1, 1e-7*Eigen::MatrixXd::Identity(16,16)) // n_of_joint, process noise, measurement noise
  //kf(16, 1*Eigen::MatrixXd::Identity(48,48), 1e-4*Eigen::MatrixXd::Identity(16,16)) // n_of_joint, process noise, measurement noise
{
    RCLCPP_INFO(this->get_logger(), "Dummy node started!");
    mutex = new std::mutex();
    count = 0;
    // subscribers
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "allegroHand_0/joint_states", 3, std::bind(&DummyNode::setJointCallback, this, std::placeholders::_1));
    joint_des_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "allegroHand_0/joint_cmd", 3, std::bind(&DummyNode::setJointDesCallback, this, std::placeholders::_1));

    // publishers
    cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "allegroHand_0/torque_cmd", 10);
    lib_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
        "allegroHand_0/lib_cmd", 10);

    // prepare msg to command effort
    joint_cmd.name.resize(16);
    joint_cmd.position.resize(16);
    joint_cmd.velocity.resize(16);
    joint_cmd.effort.resize(16);
    // prepare msg to read traject
    joint_des.name.resize(16);
    joint_des.position.resize(16);
    joint_des.velocity.resize(16);
    joint_des.effort.resize(16);
    // init vectors
	  q_encoder = Eigen::VectorXd::Zero(16);
	  q = Eigen::VectorXd::Zero(16);
	  dq = Eigen::VectorXd::Zero(16);
	  ddq = Eigen::VectorXd::Zero(16);
	  q_r = Eigen::VectorXd::Zero(16);
	  dq_r = Eigen::VectorXd::Zero(16);
	  ddq_r = Eigen::VectorXd::Zero(16);

    // init kalman (used for thunder controller)
    for (int i = 0; i < 16; ++i){
        //kf[i] = kalman_filter_joint::KalmanFilterJoint(1e-2, 1e-7);
        kf[i] = kalman_filter_joint::KalmanFilterJoint(10, 1e-7); // ok
        //kf[i] = kalman_filter_joint::KalmanFilterJoint(1, 1e-7);
    }
    // init thunder robot
    std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_hand_ros2_controllers");
    std::string conf_path = pkg_path + "/config/thunder/";
    thunder_index_handle_.load_par(conf_path + "ahand_index_identification_par.yaml");
    thunder_middle_handle_.load_par(conf_path + "ahand_middle_identification_par.yaml");
    thunder_ring_handle_.load_par(conf_path + "ahand_ring_identification_par.yaml");
    thunder_thumb_handle_.load_par(conf_path + "ahand_thumb_identification_par.yaml");


    RCLCPP_INFO(this->get_logger(), "Loaded thunder form %s",conf_path.c_str());

    thunder_controller_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "allegroHand_0/thunder_torque_cmd", 10);

    // init their library
    pBHand = new BHand(eHandType_Right);
    pBHand->GetType(eHardwareType_B);
    pBHand->SetTimeInterval(0.002);
    pBHand->SetMotionType(eMotionType_JOINT_PD);
    pBHand_controller_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "allegroHand_0/pBHand_torque_cmd", 10);


    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2),
        std::bind(&DummyNode::timer_callback, this));
}

void DummyNode::timer_callback(){

    double dt = 0.002;

    // filtering joint position
    for (int i = 0; i < 16; ++i) {
      kf[i].prediction(dt);
      kf[i].update(q_encoder(i));
      kf[i].get_q(q(i)); kf[i].get_dq(dq(i)); kf[i].get_ddq(ddq(i));
    }
    // Safe interrupt
    auto dq_abs = dq.cwiseAbs();
    if (*std::max_element(dq_abs.begin(), dq_abs.end()) > 2) {
      std_msgs::msg::String lib_cmd;
      lib_cmd.data = "off";
      //lib_cmd_pub_->publish(lib_cmd);
      RCLCPP_WARN(this->get_logger(), "Maximum joint speed exceeded. Safe controller interrupt.");
    }

    // Their Library --------------------------------------------------------------
    // 1) back-up previous joint positions:
    for (int i = 0; i < 16; i++) {
      pB_previous_position[i] = pB_current_position[i];
      pB_previous_position_filtered[i] = pB_current_position_filtered[i];
      pB_previous_velocity[i] = pB_current_velocity[i];
    }
    // 2) update
    double desired_position[16];
    for (int i = 0; i < 16; ++i) {
      pB_current_position[i] = q_encoder(i);
      desired_position[i] = q_r(i);
    }
    // 3) filtering
    for (int i = 0; i < 16; ++i) {
      pB_current_position_filtered[i] = pB_current_position[i];
      pB_current_velocity[i] =
                (pB_current_position[i] - pB_previous_position[i]) / dt;
      pB_current_velocity_filtered[i] =  pB_current_velocity[i];
      // pB_current_position_filtered[i] = (0.6 * pB_current_position_filtered[i]) +
      //                                 (0.198 * pB_previous_position[i]) +
      //                                 (0.198 * pB_current_position[i]);
      // pB_current_velocity[i] = (pB_current_position_filtered[i] - pB_previous_position_filtered[i]) / 0.002;
      // pB_current_velocity_filtered[i] = (0.6 * pB_current_velocity_filtered[i]) +
      //                                 (0.198 * pB_previous_velocity[i]) +
      //                                 (0.198 * pB_current_velocity[i]);
      // pB_current_velocity[i] = (pB_current_position[i] - pB_previous_position[i]) / 0.002;
    }
    // 4) apply controller
    pBHand->SetJointPosition(pB_current_position);
    pBHand->SetJointDesiredPosition(desired_position);
    pBHand->UpdateControl(static_cast<double>(frame) * dt);
    pBHand->GetJointTorque(pB_torque);
    frame +=1 ;




    // Thunder Library --------------------------------------------------------------
    thunder_index_handle_.set_q(q.segment(0, 4));      thunder_index_handle_.set_dq(dq.segment(0, 4));
    thunder_index_handle_.set_dqr(dq_r.segment(0, 4)); thunder_index_handle_.set_ddqr(ddq_r.segment(0, 4));


    thunder_middle_handle_.set_q(q.segment(4, 4));      thunder_middle_handle_.set_dq(dq.segment(4, 4));
    thunder_middle_handle_.set_dqr(dq_r.segment(4, 4)); thunder_middle_handle_.set_ddqr(ddq_r.segment(4, 4));


    thunder_ring_handle_.set_q(q.segment(8, 4));      thunder_ring_handle_.set_dq(dq.segment(8, 4));
    thunder_ring_handle_.set_dqr(dq_r.segment(8, 4)); thunder_ring_handle_.set_ddqr(ddq_r.segment(8, 4));

    thunder_thumb_handle_.set_q(q.segment(12, 4));      thunder_thumb_handle_.set_dq(dq.segment(12, 4));
    thunder_thumb_handle_.set_dqr(dq_r.segment(12, 4)); thunder_thumb_handle_.set_ddqr(ddq_r.segment(12, 4));


    // compute joint errors
    Eigen::VectorXd e = q_r - q;
    Eigen::VectorXd de = dq_r - dq;

    // set gains
    Eigen::MatrixXd kp = Eigen::MatrixXd::Zero(4,4);
    Eigen::MatrixXd kd = Eigen::MatrixXd::Zero(4,4);
    kp.diagonal()  << 1, 1, 1, 1;
    kd.diagonal()  << 0.04, 0.15, 0.04, 0.04;

    // compute torque
    Eigen::VectorXd thunder_tau(16);
    thunder_tau.segment(0, 4)  =  kp*e.segment( 0, 4) + kd*de.segment( 0, 4) + thunder_index_handle_.get_G() ;
    thunder_tau.segment(4, 4)  =  kp*e.segment( 4, 4) + kd*de.segment( 4, 4) + thunder_middle_handle_.get_G();
    thunder_tau.segment(8, 4)  =  kp*e.segment( 8, 4) + kd*de.segment( 8, 4) + thunder_ring_handle_.get_G()  ;
    thunder_tau.segment(12, 4) =  kp*e.segment(12, 4) + kd*de.segment(12, 4) + thunder_thumb_handle_.get_G() ;


    // create msg
    joint_cmd.header.stamp = get_clock()->now();
    for (int i = 0; i < 16; i++) {
      joint_cmd.name[i] = jointNames[i];
      joint_cmd.position[i] = q(i);
      joint_cmd.velocity[i] = dq(i);
      joint_cmd.effort[i] = pB_torque[i];
    }

    // Log Publisher --------------------------------------------------------------
    sensor_msgs::msg::JointState pB_torque_msgs;
    pB_torque_msgs.header.stamp = get_clock()->now();
    pB_torque_msgs.name.resize(16);
    pB_torque_msgs.position.resize(16);
    pB_torque_msgs.velocity.resize(16);
    pB_torque_msgs.effort.resize(16);
    for (int i = 0; i < 16; i++) {
      pB_torque_msgs.name[i] = jointNames[i];
      pB_torque_msgs.position[i] = pB_current_position_filtered[i];
      pB_torque_msgs.velocity[i] = pB_current_velocity_filtered[i];
      pB_torque_msgs.effort[i] = pB_torque[i];
    }


    sensor_msgs::msg::JointState thunder_torque_msgs;
    thunder_torque_msgs.header.stamp = get_clock()->now();
    thunder_torque_msgs.name.resize(16);
    thunder_torque_msgs.position.resize(16);
    thunder_torque_msgs.velocity.resize(16);
    thunder_torque_msgs.effort.resize(16);
    for (int i = 0; i < 16; i++) {
      thunder_torque_msgs.name[i] = jointNames[i];
      thunder_torque_msgs.position[i] = q(i);
      thunder_torque_msgs.velocity[i] = dq(i);
      thunder_torque_msgs.effort[i] = thunder_tau(i);
    }
    // Print efforts to console
    //std::ostringstream oss;
    //oss << "Efforts: [";
    //for (size_t i = 0; i < joint_cmd.effort.size(); ++i) {
    //  oss << joint_cmd.effort[i];
    //  if (i < joint_cmd.effort.size() - 1) oss << ", ";
    //}
    //oss << "]";
    //RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

    // Publish torque
    cmd_pub_->publish(joint_cmd);
    pBHand_controller_pub_->publish(pB_torque_msgs);
    thunder_controller_pub_->publish(thunder_torque_msgs);
}

void DummyNode::setJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  mutex->lock();
  for (int i = 0; i < 16; i++){
      q_encoder(i) = msg->position[i];
  }
  mutex->unlock();
}
void DummyNode::setJointDesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  mutex->lock();
  count = 0;
  mutex->unlock();
  for (int i = 0; i < 16; i++){
    //joint_des[i] = msg->position[i];
    q_r(i) = msg->position[i];
    try{
        dq_r(i) = msg->velocity.at(i);
    }catch(const std::out_of_range&) {
        dq_r(i) = 0.0;
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyNode>());
  rclcpp::shutdown();
  return 0;
}
