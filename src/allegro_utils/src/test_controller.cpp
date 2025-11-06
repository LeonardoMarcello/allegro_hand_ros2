#include "allegro_utils/test_controller.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

DummyNode::DummyNode()
: Node("dummy_node"),pBHand(nullptr)
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
    cmd_gc_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "allegroHand_0_GC/torque_cmd", 10);
    lib_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
        "allegroHand_0/lib_cmd", 10);

    // prepare msg to command effort
    joint_gc_cmd.name.resize(16);
    joint_gc_cmd.position.resize(16);
    joint_gc_cmd.effort.resize(16);

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

	  q_r = Eigen::VectorXd::Zero(4);
	  dq_r = Eigen::VectorXd::Zero(4);
	  ddq_r = Eigen::VectorXd::Zero(4);

    // init kalman
    for (int i = 0; i < 16; ++i){
        //kf[i] = kalman_filter_joint::KalmanFilterJoint(1e-1, 1e-7);
        //kf[i] = kalman_filter_joint::KalmanFilterJoint(10, 1e-7); // ok
        kf[i] = kalman_filter_joint::KalmanFilterJoint(25, 1e-7);
    }

    // init robot
    std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_utils");
    std::string conf_path = pkg_path + "/config/thunder/ahand_finger_conf.yaml";
    ahand_index.load_conf(conf_path); // or load_par_REG()

    // init their library
    pBHand = new BHand(eHandType_Right);
    pBHand->GetType(eHardwareType_B);
    pBHand->SetTimeInterval(0.002);
    pBHand->SetMotionType(eMotionType_JOINT_PD);


    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2),
        std::bind(&DummyNode::timer_callback, this));
}

void DummyNode::timer_callback(){
    //auto start = std::chrono::steady_clock::now();

    // Their Library
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
      if (i>3 && i <=7){
        desired_position[i] = q_r(i-4);
      }else{
        desired_position[i] = 0.0;
      }
    }
    // 2) filtering
    for (int i = 0; i < 16; ++i) {
      pB_current_position_filtered[i] = (0.6 * pB_current_position_filtered[i]) +
                                      (0.198 * pB_previous_position[i]) +
                                      (0.198 * pB_current_position[i]);
      pB_current_velocity[i] = (pB_current_position_filtered[i] - pB_previous_position_filtered[i]) / 0.002;
      pB_current_velocity_filtered[i] = (0.6 * pB_current_velocity_filtered[i]) +
                                      (0.198 * pB_previous_velocity[i]) +
                                      (0.198 * pB_current_velocity[i]);
      pB_current_velocity[i] = (pB_current_position[i] - pB_previous_position[i]) / 0.002;
    }
    // 3) apply controller
    pBHand->SetJointPosition(pB_current_position);
    pBHand->SetJointDesiredPosition(desired_position);
    pBHand->UpdateControl(static_cast<double>(frame) * 0.002);
    pBHand->GetJointTorque(pB_torque);
    frame +=1 ;
    // 4) Get Gravity comp
    Eigen::VectorXd pB_tau_all = Eigen::VectorXd::Zero(16);
    Eigen::VectorXd pB_tau = Eigen::VectorXd::Zero(16);
    Eigen::VectorXd pB_G = Eigen::VectorXd::Zero(16);
    double pB_kp[16] = {1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1};
    double pB_kd[16] = {0.04,0.15,0.04,0.04, 0.04,0.15,0.04,0.04, 0.04,0.15,0.04,0.04, 0.04,0.15,0.04,0.04};
    // create msg
    joint_gc_cmd.header.stamp = get_clock()->now();
    for (int i = 0; i < 16; ++i) {
      joint_gc_cmd.name[i] = jointNames[i];
      pB_tau_all(i) = pB_torque[i];
      pB_tau(i) = pB_kp[i]*(desired_position[i]-pB_current_position[i]) - pB_kd[i]*pB_current_velocity[i];
      pB_G(i) = pB_torque[i] - pB_tau(i);
      // send msg
      joint_gc_cmd.position[i] = pB_current_position[i];
      joint_gc_cmd.effort[i] = pB_tau_all(i);
    }
    cmd_gc_pub_->publish(joint_gc_cmd);


    //auto their_end = std::chrono::steady_clock::now();
    //std::chrono::duration<double> elapsed = their_end - start;
    //RCLCPP_INFO(this->get_logger(), "Elapsed time Their: %.3f seconds", elapsed.count());

    // filtering joint position---------------------------------------------------------------------------------
    for (int i = 0; i < 16; ++i) {
      kf[i].prediction(0.002);
      kf[i].update(q_encoder(i));
      kf[i].get_q(q(i)); kf[i].get_dq(dq(i)); kf[i].get_ddq(ddq(i));
      //mutex->unlock();
    }

    ///auto filtering_end = std::chrono::steady_clock::now();
    ///elapsed = filtering_end - their_end;
    ///RCLCPP_INFO(this->get_logger(), "Elapsed time Filtering (Update): %.3f seconds", elapsed.count());


    // update robot model
    count += 1;
    ahand_index.setArguments(q.segment(4, 4), dq.segment(4, 4), dq_r, ddq_r);

    // Safety interrupt
    auto T_0_ee = ahand_index.get_T_0_ee();
    double x = T_0_ee(0,3);
    double y = T_0_ee(1,3);
    double z = T_0_ee(2,3);
    if (x - 0.14 < 0.186 && z < 0){
        std_msgs::msg::String lib_cmd;
        lib_cmd.data = "off";
        lib_cmd_pub_->publish(lib_cmd);
        RCLCPP_WARN(this->get_logger(), "End-effector limits exceeded at x coord: [%f, %f, %f]", x, y, z);
    }
    //for (int i = 0; i < 16; i++) {
    for (int i = 4; i < 8; i++) {
      if (q(i) < jointLimits[i][0] || q(i) > jointLimits[i][1]){
        std_msgs::msg::String lib_cmd;
        lib_cmd.data = "off";
        lib_cmd_pub_->publish(lib_cmd);
        RCLCPP_WARN(this->get_logger(), "Maximum joint limits exceeded (joint %d). Safe controller interrupt.", i);
      }else if(dq(i) < -17 || dq(i) > 17){
        std_msgs::msg::String lib_cmd;
        lib_cmd.data = "off";
        //lib_cmd_pub_->publish(lib_cmd);
        RCLCPP_WARN(this->get_logger(), "Maximum joint speed exceeded (joint %d). Safe controller interrupt.", i);
      }
    }


    //auto check_end = std::chrono::steady_clock::now();
    //elapsed = check_end - filtering_end;
    //RCLCPP_INFO(this->get_logger(), "Elapsed time Check: %.3f seconds", elapsed.count());

    // Dynamic
    Eigen::MatrixXd M = ahand_index.get_M();
    Eigen::MatrixXd C = ahand_index.get_C();
    Eigen::MatrixXd G = ahand_index.get_G();
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(4,4);
    D.diagonal()  <<  0.0025, 0.0025, 0.000625, 0.000625;
    Eigen::VectorXd fs(4);
    fs  <<  0.001875*tanh(40*dq(4)), 0.00125*tanh(50*dq(5)), 0.00125*tanh(50*dq(6)), 0.00625*tanh(10*dq(7));

    // Errors
    Eigen::MatrixXd e = q_r - q.segment(4, 4);
    Eigen::MatrixXd de = dq_r - dq.segment(4, 4);

    // Gains
    Eigen::MatrixXd kp = Eigen::MatrixXd::Zero(4,4);
    Eigen::MatrixXd kd = Eigen::MatrixXd::Zero(4,4);
    /*
    PD:
      Kp = [1, 1, 1, 1];
      Kd = [0.04, 0.15, 0.04, 0.04];
    GC:
      Kd = [0.004*sqrt(80), 0.008*sqrt(90), 0.008*sqrt(90), 0.004*sqrt(80)];
    */
    kp.diagonal()  << 1,    1.2*1,         .95*1,       .92*1;
    kd.diagonal()  << 0.04, 1.2*0.15,   .95*0.04,     .88*0.04;

    // computed Torque law
    Eigen::MatrixXd tau_CT = M*(ddq_r + kp*e + kd*de) + C*dq.segment(4, 4) + G;

    //Eigen::MatrixXd tau_PD = G + D*dq.segment(4, 4) + fs;//kp*e + kd*de; // to do: compensazione di gravità e attriti
    //Eigen::MatrixXd tau_PD = G; // to do: compensazione di gravità e attriti
    //Eigen::MatrixXd tau_PD = kp*e + kd*de + pB_G.segment(4, 4); // to do: compensazione di gravità e attriti
    //Eigen::MatrixXd tau_PD = kp*e - kd*dq.segment(4, 4); // to do: compensazione di gravità e attriti
    //Eigen::MatrixXd tau_PD = kp*e - kd*dq.segment(4, 4) + pB_G.segment(4, 4); // to do: compensazione di gravità e attriti
    Eigen::MatrixXd tau_PD = pB_tau.segment(4, 4) + pB_G.segment(4, 4); // to do: compensazione di gravità e attriti

    // create msg
    joint_cmd.header.stamp = get_clock()->now();
    for (int i = 0; i < 16; i++) {
      joint_cmd.name[i] = jointNames[i];
      joint_cmd.position[i] = q(i);
      joint_cmd.velocity[i] = dq(i);
      if (i>3 && i <=7 && count < 200){
        joint_cmd.effort[i] = tau_PD(i-4);
      }else{
        joint_cmd.effort[i] = 0.0;
      }
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


    //auto end = std::chrono::steady_clock::now();
    //elapsed = end - check_end;
    //RCLCPP_INFO(this->get_logger(), "Elapsed Thunder time: %.3f seconds", elapsed.count());
    //elapsed = end - start;
    //RCLCPP_INFO(this->get_logger(), "Elapsed Total time: %.3f seconds", elapsed.count());
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
  for (int i = 0; i < 16; i++){
    //joint_des[i] = msg->position[i];
    if (i>3 && i <=7){
      q_r(i-4) = msg->position[i];
      try {
          dq_r(i - 4) = msg->velocity.at(i);  // use .at() to throw on invalid index
      }
      catch (const std::out_of_range& e) {
          dq_r(i - 4) = 0.0;
      }
    }
  }
  mutex->unlock();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyNode>());
  rclcpp::shutdown();
  return 0;
}
