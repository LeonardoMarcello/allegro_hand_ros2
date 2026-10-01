#include "allegro_hand_ros2_controllers/allegro_hand_cartesian_impedance.hpp"


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
namespace allegro_hand_cartesian_impedance
{
    CallbackReturn CartesianImpedance_Controller::on_init()
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


    CallbackReturn CartesianImpedance_Controller::on_configure(const rclcpp_lifecycle::State & )
    {

        // get parameters
        if (!(get_node()->has_parameter("gains.finger.kp") &&
              get_node()->has_parameter("gains.finger.kd") &&
              get_node()->has_parameter("gains.thumb.kp") &&
              get_node()->has_parameter("gains.thumb.kd"))){
                RCLCPP_ERROR(get_node()->get_logger(),"Could not load controller gains.");
                return CallbackReturn::ERROR;
        }
        // load the 4 gain to applay at each finger
        Kp.clear();
        Kd.clear();
        auto kp = get_node()->get_parameter("gains.finger.kp").as_double_array();
        auto kd = get_node()->get_parameter("gains.finger.kd").as_double_array();
        if (kp.size()!=4 || kd.size()!=4){
                RCLCPP_ERROR(get_node()->get_logger(),"Invalid number of gains.");
                return CallbackReturn::ERROR;
        }
        // assign gains
        Kp.emplace_back(kp[0], kp[1], kp[2], kp[3]);    // index
        Kd.emplace_back(kd[0], kd[1], kd[2], kd[3]);
        Kp.emplace_back(kp[0], kp[1], kp[2], kp[3]);    // middle
        Kd.emplace_back(kd[0], kd[1], kd[2], kd[3]);
        Kp.emplace_back(kp[0], kp[1], kp[2], kp[3]);    // ring
        Kd.emplace_back(kd[0], kd[1], kd[2], kd[3]);


        // load the 4 gain for thumb
        kp = get_node()->get_parameter("gains.thumb.kp").as_double_array();
        kd = get_node()->get_parameter("gains.thumb.kd").as_double_array();
        if (kp.size()!=4 || kd.size()!=4){
                RCLCPP_ERROR(get_node()->get_logger(),"Invalid number of gains.");
                return CallbackReturn::ERROR;
        }
        Kp.emplace_back(kp[0], kp[1], kp[2], kp[3]);    // thumb
        Kd.emplace_back(kd[0], kd[1], kd[2], kd[3]);



        // load hand models
        std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_hand_ros2_controllers");
        std::string conf_path = pkg_path + "/config/thunder/";
        thunder_index_handle_.load_par(conf_path + "ahand_index_identification_par.yaml");
        thunder_middle_handle_.load_par(conf_path + "ahand_middle_identification_par.yaml");
        thunder_ring_handle_.load_par(conf_path + "ahand_ring_identification_par.yaml");
        thunder_thumb_handle_.load_par(conf_path + "ahand_thumb_identification_par.yaml");


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
            "allegroHand_0/cartesian_cmd",
            in_qos,
            [this](const CmdMsg::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });


        //create conroller publisher
        controller_pub_ = get_node()->create_publisher<CmdMsg>(
            "~/command_torque",
            out_qos);


        RCLCPP_INFO(get_node()->get_logger(),"configure succesfully");
        return CallbackReturn::SUCCESS;
    }


    CallbackReturn CartesianImpedance_Controller::on_activate(const rclcpp_lifecycle::State & )
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CartesianImpedance_Controller::on_deactivate(const rclcpp_lifecycle::State & )
    {
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>>(nullptr);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CartesianImpedance_Controller::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

   controller_interface::InterfaceConfiguration CartesianImpedance_Controller::state_interface_configuration() const
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

    controller_interface::InterfaceConfiguration CartesianImpedance_Controller::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration cmd_int_cnf;
        cmd_int_cnf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(auto &it : jointNames)
        {
            cmd_int_cnf.names.push_back(it + "/" + hardware_interface::HW_IF_EFFORT);
        }
        return cmd_int_cnf;
    }

    controller_interface::return_type CartesianImpedance_Controller::update(
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
        for (size_t i = 0; i < DOF_JOINTS; i++) {
            q(i) = state_interfaces_[i * 3].get_value();
            dq(i) = state_interfaces_[i * 3 + 1].get_value();
        }

        // Load fingertip command
        Eigen::VectorXd index_x_r = Eigen::VectorXd::Zero(3*);
        index_x_r(0) = joint_cmd_.poses.at(0).position(0);
        index_x_r(1) = joint_cmd_.poses.at(0).position(1);
        index_x_r(2) = joint_cmd_.poses.at(0).position(2);

        Eigen::VectorXd middle_x_r = Eigen::VectorXd::Zero(3);
        middle_x_r(0) = joint_cmd_.poses.at(1).position(0);
        middle_x_r(1) = joint_cmd_.poses.at(1).position(1);
        middle_x_r(2) = joint_cmd_.poses.at(1).position(2);

        Eigen::VectorXd ring_x_r = Eigen::VectorXd::Zero(3);
        ring_x_r(0) = joint_cmd_.poses.at(2).position(0);
        ring_x_r(1) = joint_cmd_.poses.at(2).position(1);
        ring_x_r(2) = joint_cmd_.poses.at(2).position(2);

        Eigen::VectorXd thumb_x_r = Eigen::VectorXd::Zero(3);
        thumb_x_r(0) = joint_cmd_.poses.at(3).position(0);
        thumb_x_r(1) = joint_cmd_.poses.at(3).position(1);
        thumb_x_r(2) = joint_cmd_.poses.at(3).position(2);




        thunder_index_handle_.set_q(q.segment(0, 4));      thunder_index_handle_.set_dq(dq.segment(0, 4));
        thunder_index_handle_.set_dqr(dq_r.segment(0, 4)); thunder_index_handle_.set_ddqr(ddq_r.segment(0, 4));


        thunder_middle_handle_.set_q(q.segment(4, 4));      thunder_middle_handle_.set_dq(dq.segment(4, 4));
        thunder_middle_handle_.set_dqr(dq_r.segment(4, 4)); thunder_middle_handle_.set_ddqr(ddq_r.segment(4, 4));


        thunder_ring_handle_.set_q(q.segment(8, 4));      thunder_ring_handle_.set_dq(dq.segment(8, 4));
        thunder_ring_handle_.set_dqr(dq_r.segment(8, 4)); thunder_ring_handle_.set_ddqr(ddq_r.segment(8, 4));

        thunder_thumb_handle_.set_q(q.segment(12, 4));      thunder_thumb_handle_.set_dq(dq.segment(12, 4));
        thunder_thumb_handle_.set_dqr(dq_r.segment(12, 4)); thunder_thumb_handle_.set_ddqr(ddq_r.segment(12, 4));

        // compute tip errors
        Eigen::VectorXd index_e = index_x_r - thunder_index_handle_.get_T_w_ee().col(3).head<3>();
        Eigen::VectorXd middle_e = index_x_r - thunder_index_handle_.get_T_w_ee().col(3).head<3>();
        Eigen::VectorXd ring_e = index_x_r - thunder_index_handle_.get_T_w_ee().col(3).head<3>();
        Eigen::VectorXd thumb_e = index_x_r - thunder_index_handle_.get_T_w_ee().col(3).head<3>();


        Eigen::VectorXd tau(DOF_JOINTS);
        // ----------------------------------
        // CONTROL LAW IMPLEMENTATION
        //
        // Add here the desired contro Law
        // ----------------------------------
        // ...
        Eigen::Matrix<double, 3, 4> J = thunder_index_handle_.get_J_ee().topRows<3>();
        Eigen::Matrix<double, 4, 3> JT = J.transpose();
        Eigen::Matrix<double, 3, 4> JTpinv = JT.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d N = I - (JT * JTpinv);
        // Task
        Eigen::Vector4d tau_task = JT * (
            cartesian_stiffness_i.asDiagonal() * error_ee -
            cartesian_damping_i.asDiagonal() * (J * finger.get_dq())
        );
        Eigen::Vector4d tau_null = N * (-kp_i.asDiagonal() * finger.get_q() - kv_i.asDiagonal() * finger.get_dq());
        // Gravity Compensation
        Eigen::Vector4d tau_gravity = thunder_index_handle_.get_G();
        // Control
        tau.segment(0, 4)  =  tau_task + tau_null + tau_gravity;
        // ---------------------
        J = thunder_middle_handle_.get_J_ee().topRows<3>();
        JT = J.transpose();
        JTpinv = JT.completeOrthogonalDecomposition().pseudoInverse();
        N = I - (JT * JTpinv);
        tau_task = JT * (
            cartesian_stiffness_i.asDiagonal() * error_ee -
            cartesian_damping_i.asDiagonal() * (J * finger.get_dq())
        );
        tau_null = N * (-kp_i.asDiagonal() * finger.get_q() - kv_i.asDiagonal() * finger.get_dq());
        tau_gravity = thunder_middle_handle_.get_G();
        tau.segment(4, 4)  =  tau_task + tau_null + tau_gravity;
        // ---------------------
        J = thunder_ring_handle_.get_J_ee().topRows<3>();
        JT = J.transpose();
        JTpinv = JT.completeOrthogonalDecomposition().pseudoInverse();
        N = I - (JT * JTpinv);
        tau_task = JT * (
            cartesian_stiffness_i.asDiagonal() * error_ee -
            cartesian_damping_i.asDiagonal() * (J * finger.get_dq())
        );
        tau_null = N * (-kp_i.asDiagonal() * finger.get_q() - kv_i.asDiagonal() * finger.get_dq());
        tau_gravity = thunder_ring_handle_.get_G();
        tau.segment(8, 4)  =  tau_task + tau_null + tau_gravity;
        // ---------------------
        J = thunder_thumb_handle_.get_J_ee().topRows<3>();
        JT = J.transpose();
        JTpinv = JT.completeOrthogonalDecomposition().pseudoInverse();
        N = I - (JT * JTpinv);
        tau_task = JT * (
            cartesian_stiffness_i.asDiagonal() * error_ee -
            cartesian_damping_i.asDiagonal() * (J * finger.get_dq())
        );
        tau_null = N * (-kp_i.asDiagonal() * finger.get_q() - kv_i.asDiagonal() * finger.get_dq());
        tau_gravity = thunder_thumb_handle_.get_G();
        tau.segment(12, 4)  =  tau_task + tau_null + tau_gravity;

        // ...

        // write on command_interfaces_ : [eff_joint0, eff_joint1, ...]
        sensor_msgs::msg::JointStates tau_controller;
        tau_controller.name.resize(DOF_JOINTS);
        tau_controller.position.resize(DOF_JOINTS);
        tau_controller.effort.resize(DOF_JOINTS);
        tau_controller.header.stamp = get_node()->now();
        for (size_t i = 0; i < DOF_JOINTS; i++) {
            double effort_command = tau_GC(i);
            double effort_msg = tau(i);
            tau_controller.name[i] = jointNames[i];
            tau_controller.position[i] = q(i);
            tau_controller.effort[i] = effort_command;
            command_interfaces_[i].set_value(effort_command);
        }
        controller_pub_->publish(tau_controller);

        return controller_interface::return_type::OK;
    }


}; // namespace allegro_hand_cartesian_impedance
PLUGINLIB_EXPORT_CLASS(
    allegro_hand_cartesian_impedance::CartesianImpedance_Controller, controller_interface::ControllerInterface
);