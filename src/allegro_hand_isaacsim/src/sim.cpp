#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TrajectoryToJointState : public rclcpp::Node
{
public:
  TrajectoryToJointState()
  : Node("moveit_jointstate_publisher")
  {
    // Declare use_sim_time parameter and set it (default false)
    bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    this->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));

    traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/execute_trajectory/trajectory", 10,
      std::bind(&TrajectoryToJointState::trajectoryCallback, this, std::placeholders::_1));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("allegroHand/joint_cmd", 10);

    timer_ = this->create_wall_timer(10ms, std::bind(&TrajectoryToJointState::publishJointState, this));
  }

private:
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  trajectory_msgs::msg::JointTrajectory current_trajectory_;
  rclcpp::Time start_time_;
  size_t index_ = 0;
  bool executing_ = false;

  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu points", msg->points.size());
    current_trajectory_ = *msg;
    start_time_ = this->now();
    index_ = 0;
    executing_ = true;
  }

  void publishJointState()
  {
    if (!executing_ || current_trajectory_.points.empty())
      return;

    rclcpp::Duration elapsed = this->now() - start_time_;
    double t_sec = elapsed.seconds();

    while (index_ < current_trajectory_.points.size()) {
      const auto& pt = current_trajectory_.points[index_];
      double pt_time = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;

      if (pt_time <= t_sec) {
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.name = current_trajectory_.joint_names;
        js.position = pt.positions;
        if (!pt.velocities.empty())
          js.velocity = pt.velocities;
        if (!pt.effort.empty())
          js.effort = pt.effort;

        joint_state_pub_->publish(js);
        index_++;
      } else {
        break;
      }
    }

    if (index_ >= current_trajectory_.points.size()) {
      executing_ = false;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryToJointState>());
  rclcpp::shutdown();
  return 0;
}
