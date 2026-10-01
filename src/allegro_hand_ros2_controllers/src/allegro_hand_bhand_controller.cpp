#include "allegro_hand_ros2_controllers/allegro_hand_bhand_controller.hpp"


#include "pluginlib/class_list_macros.hpp"

#include "controller_interface/helpers.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>


using namespace std::chrono;
using namespace std::chrono_literals;
namespace allegro_hand_bhand_controller
{

    CallbackReturn BHand_Controller::on_init()
    {
        // declaring parameters
        try
        {
            //kp,kd,staturation,rate
            //declare_parameters();
        }
         catch(const std::exception & e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"Exception thrown during declaration of joints name with message: %s", e.what());
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_node()->get_logger(),"initialize succesfully");
        return CallbackReturn::SUCCESS;
    };


    CallbackReturn BHand_Controller::on_configure(const rclcpp_lifecycle::State & )
    {

        // get parameters
        if (!(get_node()->has_parameter("gains.finger.kp") &&
              get_node()->has_parameter("gains.finger.kd") &&
              get_node()->has_parameter("gains.thumb.kp") &&
              get_node()->has_parameter("gains.thumb.kd"))){
                RCLCPP_ERROR(get_node()->get_logger(),"Could not load controller gains.");
                return CallbackReturn::ERROR;
        }
        
        Kp.clear();
        Kd.clear();
        auto kp = get_node()->get_parameter("gains.finger.kp").as_double_array();
        auto kd = get_node()->get_parameter("gains.finger.kd").as_double_array();
        if (kp.size()!=4 || kd.size()!=4){
                RCLCPP_ERROR(get_node()->get_logger(),"Invalid number of gains.");
                return CallbackReturn::ERROR;
        }
        Kp.emplace_back(kp[0], kp[1], kp[2], kp[3]);    // index
        Kd.emplace_back(kd[0], kd[1], kd[2], kd[3]);
        Kp.emplace_back(kp[0], kp[1], kp[2], kp[3]);    // middle
        Kd.emplace_back(kd[0], kd[1], kd[2], kd[3]);
        Kp.emplace_back(kp[0], kp[1], kp[2], kp[3]);    // ring
        Kd.emplace_back(kd[0], kd[1], kd[2], kd[3]);


        kp = get_node()->get_parameter("gains.finger.kp").as_double_array();
        kd = get_node()->get_parameter("gains.finger.kd").as_double_array();
        if (kp.size()!=4 || kd.size()!=4){
                RCLCPP_ERROR(get_node()->get_logger(),"Invalid number of gains.");
                return CallbackReturn::ERROR;
        }
        Kp.emplace_back(kp[0], kp[1], kp[2], kp[3]);    // thumb
        Kd.emplace_back(kd[0], kd[1], kd[2], kd[3]);


        //set qos protocol
        rclcpp::QoS out_qos(10),in_qos(10);
        if(get_node()->has_parameter("BestEffort_QOS") &&
            get_node()->get_parameter("BestEffort_QOS").as_bool())
        {
            out_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
            in_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        }
        else
        {
            out_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
            in_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        }
        //set the in qos for deadline miss
        // duration<double,std::milli>  deadmis_to_ = dur;

        // in_qos.deadline(deadmis_to_);
        // rclcpp::SubscriptionOptions sub_opt;
        // sub_opt.event_callbacks.deadline_callback = 
        // [this](rclcpp::QOSDeadlineRequestedInfo & event) 
        // {
        //     this->dl_miss_count_ ++;
        //     if(dl_miss_count_ > 10)
        //     {
        //         std::lock_guard<std::mutex> l_g(var_mutex_);
        //         c_stt_ = Controller_State::INACTIVE;
        //     }
        //     RCLCPP_WARN(this->get_node()->get_logger(),"Passed deadline");
        // };

        //create subscriber to traject command
        joints_cmd_sub_ = get_node()->create_subscription<CmdMsg>(
            //"~/command",
            "allegroHand_0/joint_cmd",
            in_qos,
            [this](const CmdMsg::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });


        //create conroller publisher
        controller_pub_ = get_node()->create_publisher<CmdMsg>(
            "~/command_torque",
            out_qos);

        // init their library
        pBHand = new BHand(eHandType_Right);
        pBHand->GetType(eHardwareType_B);
        pBHand->SetTimeInterval(0.002);
        pBHand->SetMotionType(eMotionType_JOINT_PD);

        // set up tf
        // hand_frame_name_ = "ahand";
        // hand_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
        // hand_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*hand_tf_buffer_);


        RCLCPP_INFO(get_node()->get_logger(),"configure succesfully");
        return CallbackReturn::SUCCESS;
    }


    CallbackReturn BHand_Controller::on_activate(const rclcpp_lifecycle::State & )
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BHand_Controller::on_deactivate(const rclcpp_lifecycle::State & )
    {
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>>(nullptr);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BHand_Controller::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

   controller_interface::InterfaceConfiguration BHand_Controller::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration stt_int_cnf;
        stt_int_cnf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for( auto & it : jointNames)
        {
            stt_int_cnf.names.push_back(it + "/" + hardware_interface::HW_IF_POSITION);
            stt_int_cnf.names.push_back(it + "/" + hardware_interface::HW_IF_VELOCITY);
            stt_int_cnf.names.push_back(it + "/" + hardware_interface::HW_IF_EFFORT);
        }
        return stt_int_cnf;
    }

    controller_interface::InterfaceConfiguration BHand_Controller::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration cmd_int_cnf;
        cmd_int_cnf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(auto &it : jointNames)
        {
            cmd_int_cnf.names.push_back(it + "/" + hardware_interface::HW_IF_EFFORT);
        }
        return cmd_int_cnf;
    }

    controller_interface::return_type BHand_Controller::update(
                const rclcpp::Time & , const rclcpp::Duration & period
            )
    {
        // position command
        auto joint_cmd_ptr = rt_command_ptr_.readFromRT();

        // no position command received yet
        if (!joint_cmd_ptr || !(*joint_cmd_ptr)) {
            return controller_interface::return_type::OK;
        }
        // bad position command received
        if ((*joint_cmd_ptr)->name.size() != DOF_JOINTS) {
            RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                                "command size (%zu) does not match number of joints (%u)", 
                                (*joint_cmd_ptr)->name.size(), DOF_JOINTS);
            return controller_interface::return_type::ERROR;
        }else{
            joint_cmd_ = *(*joint_cmd_ptr);
        }
        // read state_interfaces_ : [pos_joint0, vel_joint0, eff_joint0, pos_joint1, ...]
        Eigen::VectorXd q = Eigen::VectorXd::Zero(DOF_JOINTS);
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(DOF_JOINTS);
        Eigen::VectorXd q_r = Eigen::VectorXd::Zero(DOF_JOINTS);
        Eigen::VectorXd dq_r = Eigen::VectorXd::Zero(DOF_JOINTS);
        Eigen::VectorXd ddq_r = Eigen::VectorXd::Zero(DOF_JOINTS);
        for (size_t i = 0; i < DOF_JOINTS; i++) {
            q(i) = state_interfaces_[i * 3].get_value();
            dq(i) = state_interfaces_[i * 3 + 1].get_value();
            q_r(i) = joint_cmd_.position.at(i);
            try{
               dq_r(i) = joint_cmd_.velocity.at(i);
            }catch(const std::out_of_range & e){
                dq_r(i) = 0.0;
            }
        }

        double desired_position[16];
        for (size_t i = 0; i < DOF_JOINTS; i++) {
            pB_current_position[i] = q(i);
            desired_position[i] = q_r(i);
        }
        double dt = period.seconds();
        pBHand->SetJointPosition(pB_current_position);
        pBHand->SetJointDesiredPosition(desired_position);
        // if (enable_hand_pose){
        //     try {
        //         auto t = hand_tf_buffer_->lookupTransform(
        //                                     hand_frame_name_.c_str(), "world",
        //                                     tf2::TimePointZero);

        //         // tf2::fromMsg(t.transform.rotation, q);
        //         // double roll, pitch, yaw;
        //         // hand_pose_quaternion.getRPY(roll, pitch, yaw);
        //         // pBHand->SetOrientation(roll,pitch,yaw);

        //     }catch (const tf2::TransformException & ex) {
        //         RCLCPP_INFO(
        //             get_node()->get_logger(), "Could not transform %s to %s: %s",
        //             hand_frame_name_.c_str(), "world", ex.what());
        //     }
        // }

        pBHand->UpdateControl(static_cast<double>(frame) * dt);
        pBHand->GetJointTorque(pB_torque);
        frame +=1 ;


        // write on command_interfaces_ : [eff_joint0, eff_joint1, ...]
        CmdMsg pb_controller;
        pb_controller.name.resize(DOF_JOINTS);
        pb_controller.effort.resize(DOF_JOINTS);
        pb_controller.header.stamp = get_node()->now();
        for (size_t i = 0; i < DOF_JOINTS; i++) {
            double effort_command = pB_torque[i];
            pb_controller.name[i] = jointNames[i];
            pb_controller.effort[i] = effort_command;
            command_interfaces_[i].set_value(effort_command);
        }
        controller_pub_->publish(pb_controller);




        return controller_interface::return_type::OK;
    }


}; // namespace allegro_hand_bhand_controller
PLUGINLIB_EXPORT_CLASS(
    allegro_hand_bhand_controller::BHand_Controller, controller_interface::ControllerInterface
);