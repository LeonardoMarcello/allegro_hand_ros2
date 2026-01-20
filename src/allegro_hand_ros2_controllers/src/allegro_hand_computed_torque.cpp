#include "allegro_hand_ros2_controllers/allegro_hand_computed_torque.hpp"


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
namespace allegro_hand_computed_torque
{
    CallbackReturn ComputedTorque_Controller::on_init()
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


    CallbackReturn ComputedTorque_Controller::on_configure(const rclcpp_lifecycle::State & )
    {

        // get parameters
        if (!(get_node()->has_parameter("kp1") &&
            get_node()->has_parameter("kp2") &&
            get_node()->has_parameter("kp3") &&
            get_node()->has_parameter("kd1") &&
            get_node()->has_parameter("kd2") &&
            get_node()->has_parameter("kd3"))){
                RCLCPP_ERROR(get_node()->get_logger(),"Could not load controller gains.");
                return CallbackReturn::ERROR;
        }
        double kp1 = get_node()->get_parameter("kp1").as_double();
        double kp2 = get_node()->get_parameter("kp2").as_double();
        double kp3 = get_node()->get_parameter("kp3").as_double();
        Kp[0] = kp1;  Kp[1] = kp2;  Kp[2] = kp2;  Kp[3] = kp3;  // index
        Kp[4] = kp1;  Kp[5] = kp2;  Kp[6] = kp2;  Kp[7] = kp3;  // middle
        Kp[8] = kp1;  Kp[9] = kp2;  Kp[10] = kp2; Kp[11] = kp3; // ring
        Kp[12] = kp1; Kp[13] = kp2; Kp[14] = kp2; Kp[15] = kp3; // thumb

        double kd1 = get_node()->get_parameter("kd1").as_double();
        double kd2 = get_node()->get_parameter("kd2").as_double();
        double kd3 = get_node()->get_parameter("kd3").as_double();
        Kd[0] = kd1;  Kd[1] = kd2;  Kd[2] = kd2;  Kd[3] = kd3;  // index
        Kd[4] = kd1;  Kd[5] = kd2;  Kd[6] = kd2;  Kd[7] = kd3;  // middle
        Kd[8] = kd1;  Kd[9] = kd2;  Kd[10] = kd2; Kd[11] = kd3; // ring
        Kd[12] = kd1; Kd[13] = kd2; Kd[14] = kd2; Kd[15] = kd3; // thumb

        // if (!(get_node()->has_parameter("dt"))){
        //     RCLCPP_ERROR(get_node()->get_logger(),"Could not load controller dt.");
        //     return CallbackReturn::ERROR;
        // }
        // auto dt = get_node()->get_parameter("dt").as_int(); // in [ms]

        // milliseconds dur{dt + 5};


        // load hand models
        std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_hand_ros2_controllers");
        std::string conf_path = pkg_path + "/config/thunder/";
        thunder_index_handle_.load_conf(conf_path + "ahand_index_conf.yaml"); // or load_par_REG()
        thunder_middle_handle_.load_conf(conf_path + "ahand_middle_conf.yaml"); // or load_par_REG()
        thunder_ring_handle_.load_conf(conf_path + "ahand_ring_conf.yaml"); // or load_par_REG()
        //thunder_thumb_handle_.load_conf(conf_path + "ahand_thumb_conf.yaml"); // NON esiste in nuovo thunder


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

        //create subscriber to traject
        joints_cmd_sub_ = get_node()->create_subscription<CmdMsg>(
            "~/command",
            in_qos,
            [this](const CmdMsg::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });


        RCLCPP_INFO(get_node()->get_logger(),"configure succesfully");
        return CallbackReturn::SUCCESS;
    }


    CallbackReturn ComputedTorque_Controller::on_activate(const rclcpp_lifecycle::State & )
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ComputedTorque_Controller::on_deactivate(const rclcpp_lifecycle::State & )
    {
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>>(nullptr);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ComputedTorque_Controller::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

   controller_interface::InterfaceConfiguration ComputedTorque_Controller::state_interface_configuration() const
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

    controller_interface::InterfaceConfiguration ComputedTorque_Controller::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration cmd_int_cnf;
        cmd_int_cnf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(auto &it : jointNames)
        {
            cmd_int_cnf.names.push_back(it + "/" + hardware_interface::HW_IF_EFFORT);
        }
        return cmd_int_cnf;
    }

    controller_interface::return_type ComputedTorque_Controller::update(
                const rclcpp::Time & time, const rclcpp::Duration & 
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
            //dq_r(i) = joint_cmd_.velocity.at(i);
            dq_r(i) = 0.0;
        }

        Eigen::VectorXd tau(DOF_JOINTS);
        thunder_index_handle_.setArguments(q.segment(0, 4),dq.segment(0, 4),dq_r.segment(0, 4), ddq_r.segment(0, 4));
        thunder_middle_handle_.setArguments(q.segment(4, 4), dq.segment(4, 4), dq_r.segment(4, 4), ddq_r.segment(4, 4));
        thunder_ring_handle_.setArguments(q.segment(8, 4), dq.segment(8, 4), dq_r.segment(8, 4), ddq_r.segment(8, 4));
        thunder_thumb_handle_.set_q(q.segment(12, 4));      thunder_thumb_handle_.set_dq(dq.segment(12, 4));
        thunder_thumb_handle_.set_dqr(dq_r.segment(12, 4)); thunder_thumb_handle_.set_ddqr(ddq_r.segment(12, 4));

        // compute joint errors
        Eigen::VectorXd e = q_r - q;
        Eigen::VectorXd de = dq_r - dq;

        // ----------------------------------
        // CONTROL LAW IMPLEMENTATION
        //
        // Add here the desired contro Law
        // ----------------------------------
        // ...
        Eigen::SparseMatrix<double> beta;
        Eigen::SparseMatrix<double> beta_pinv;
        finger_beta::load_beta(beta);
        finger_beta::load_beta_pinv(beta_pinv);
        auto hat_pi_reduced = finger_beta::load_pi_hat();


        Eigen::SparseMatrix<double> thumb_beta;
        Eigen::SparseMatrix<double> thumb_beta_pinv;
        thumb_beta::load_beta(thumb_beta);
        thumb_beta::load_beta_pinv(thumb_beta_pinv);
        auto thumb_hat_pi_reduced = thumb_beta::load_pi_hat();



        Eigen::MatrixXd Yr = thunder_index_handle_.get_reg_G();
        Eigen::MatrixXd Y_Ir = 0*q.segment(0, 4).asDiagonal();
        Eigen::MatrixXd Y_rIr(Yr.rows(), Yr.cols() + Y_Ir.cols());
        Y_rIr << Yr, Y_Ir;
        tau.segment(0, 4) =  Y_rIr*beta_pinv*hat_pi_reduced.segment(0,24);



        Yr = thunder_middle_handle_.get_reg_G();
        Y_Ir = 0*q.segment(4, 4).asDiagonal();
        Y_rIr << Yr, Y_Ir;
        tau.segment(4, 4) =  Y_rIr*beta_pinv*hat_pi_reduced.segment(0,24);



        Yr = thunder_ring_handle_.get_reg_G();
        Y_Ir = 0*q.segment(8, 4).asDiagonal();
        Y_rIr << Yr, Y_Ir;
        tau.segment(8, 4) =  Y_rIr*beta_pinv*hat_pi_reduced.segment(0,24);

        Eigen::MatrixXd thumb_Yr = thunder_thumb_handle_.get_reg_G();
        Eigen::MatrixXd thumb_Y_Ir = 0*q.segment(12, 4).asDiagonal();
        Eigen::MatrixXd thumb_Y_rIr(thumb_Yr.rows(), thumb_Yr.cols() + thumb_Y_Ir.cols());
        thumb_Y_rIr << thumb_Yr, thumb_Y_Ir;
        tau.segment(12, 4) =  thumb_Y_rIr*thumb_beta_pinv*thumb_hat_pi_reduced.segment(0,24);
        // ...

        // write on command_interfaces_ : [eff_joint0, eff_joint1, ...]
        for (size_t i = 0; i < DOF_JOINTS; i++) {
            double effort_command = tau(i);
            // if (i>=12){
            //     RCLCPP_INFO(get_node()->get_logger(), "Thumb Joint %zu: Computed Torque: %f", i, effort_command);
            //     effort_command = 0.0; // disable thumb torque for safety
            // }
            command_interfaces_[i].set_value(effort_command);
        }

        return controller_interface::return_type::OK;
    }


}; // namespace allegro_hand_computed_torque
PLUGINLIB_EXPORT_CLASS(
    allegro_hand_computed_torque::ComputedTorque_Controller, controller_interface::ControllerInterface
);