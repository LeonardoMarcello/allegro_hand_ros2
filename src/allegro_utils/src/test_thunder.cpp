#include "allegro_utils/test_thunder.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

DummyNode::DummyNode()
: Node("dummy_node"),
  kf(16, 1e-1, 1e-7*Eigen::MatrixXd::Identity(16,16)) // n_of_joint, process noise, measurement noise
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
        "allegroHand_fake/torque_cmd", 10);
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
	  q_r = Eigen::VectorXd::Zero(4);
	  dq_r = Eigen::VectorXd::Zero(4);
	  ddq_r = Eigen::VectorXd::Zero(4);
    // init robot
    std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_utils");
    std::string conf_path = pkg_path + "/config/thunder/ahand_finger_conf.yaml";
    ahand_index.load_conf(conf_path); // or load_par_REG()


  timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2),
        std::bind(&DummyNode::timer_callback, this));
}

void DummyNode::timer_callback(){
    // filtering joint position
    kf.prediction(0.002);
    mutex->lock();
    kf.update(q_encoder);
    count += 1;
    mutex->unlock();
    kf.get_q(q);kf.get_dq(dq);kf.get_ddq(ddq);

    // Safe interrupt
    auto dq_abs = dq.cwiseAbs();
    if (*std::max_element(dq_abs.begin(), dq_abs.end()) > 2) {
      std_msgs::msg::String lib_cmd;
      lib_cmd.data = "off";
      //lib_cmd_pub_->publish(lib_cmd);
      RCLCPP_WARN(this->get_logger(), "Maximum joint speed exceeded. Safe controller interrupt.");
    }

    // update robot model
    ahand_index.setArguments(q.segment(4, 4), dq.segment(4, 4), dq_r, ddq_r);

    // Dynamics
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
    kp.diagonal()  << 1, 1, 1, 1;
    kd.diagonal()  << 0.04, 0.15, 0.04, 0.04;

    // computed Torque law
    Eigen::MatrixXd tau_CT = M*(ddq_r + kp*e + kd*de) + C*dq.segment(4, 4) + G;

    Eigen::MatrixXd tau_PD = G + D*dq.segment(4, 4) + fs;//kp*e + kd*de; // to do: compensazione di gravità e attriti
    Eigen::MatrixXd tau_GC = G; // to do: compensazione di gravità e attriti

    // create msg
    joint_cmd.header.stamp = get_clock()->now();
    for (int i = 0; i < 16; i++) {
      joint_cmd.name[i] = jointNames[i];
      joint_cmd.position[i] = q(i);
      joint_cmd.velocity[i] = dq(i);
      if (i>3 && i <=7){// && count < 200){
        joint_cmd.effort[i] = tau_GC(i-4);
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
    if (i>3 && i <=7){
      q_r(i-4) = msg->position[i];
      dq_r(i-4) = msg->velocity[i];
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
