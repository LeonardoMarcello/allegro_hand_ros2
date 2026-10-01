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

        // double kp1 = get_node()->get_parameter("kp1").as_double();
        // double kp2 = get_node()->get_parameter("kp2").as_double();
        // double kp3 = get_node()->get_parameter("kp3").as_double();
        // double kp4 = get_node()->get_parameter("kp4").as_double();
        // Kp.emplace_back(kp1, kp2, kp3, kp4);    // index
        // Kp.emplace_back(kp1, kp2, kp3, kp4);    // middle
        // Kp.emplace_back(kp1, kp2, kp3, kp4);    // ring
        // Kp.emplace_back(kp1, kp2, kp3, kp4);    // thumb
        // // Kp[0] = kp1;  Kp[1] = kp2;  Kp[2] = kp3;  Kp[3] = kp4;  // index 
        // // Kp[4] = kp1;  Kp[5] = kp2;  Kp[6] = kp3;  Kp[7] = kp4;  // middle
        // // Kp[8] = kp1;  Kp[9] = kp2;  Kp[10] = kp3; Kp[11] = kp4; // ring  
        // // Kp[12] = kp1; Kp[13] = kp2; Kp[14] = kp3; Kp[15] = kp4; // thumb 

        // double kd1 = get_node()->get_parameter("kd1").as_double();
        // double kd2 = get_node()->get_parameter("kd2").as_double();
        // double kd3 = get_node()->get_parameter("kd3").as_double();
        // double kd4 = get_node()->get_parameter("kd4").as_double();
        // Kd.emplace_back(kd1, kd2, kd3, kd4);    // index
        // Kd.emplace_back(kd1, kd2, kd3, kd4);    // middle
        // Kd.emplace_back(kd1, kd2, kd3, kd4);    // ring
        // Kd.emplace_back(kd1, kd2, kd3, kd4);    // thumb
        // // Kd[0] = kd1;  Kd[1] = kd2;  Kd[2] = kd3;  Kd[3] = kd4;  // index
        // // Kd[4] = kd1;  Kd[5] = kd2;  Kd[6] = kd3;  Kd[7] = kd4;  // middle
        // // Kd[8] = kd1;  Kd[9] = kd2;  Kd[10] = kd3; Kd[11] = kd4; // ring
        // // Kd[12] = kd1; Kd[13] = kd2; Kd[14] = kd3; Kd[15] = kd4; // thumb

        // RCLCPP_INFO(get_node()->get_logger(),"Fingers controller gain:");
        // for (int i=0; i<4;i++){
        //     RCLCPP_INFO(get_node()->get_logger(),"Kp: %f, %f, %f, %f", Kp[i](0), Kp[i](1), Kp[i](2), Kp[i](3));
        //     RCLCPP_INFO(get_node()->get_logger(),"Kd: %f, %f, %f, %f", Kd[i](0), Kd[i](1), Kd[i](2), Kd[i](3));
        // }
        // if (!(get_node()->has_parameter("dt"))){
        //     RCLCPP_ERROR(get_node()->get_logger(),"Could not load controller dt.");
        //     return CallbackReturn::ERROR;
        // }
        // auto dt = get_node()->get_parameter("dt").as_int(); // in [ms]

        // milliseconds dur{dt + 5};

        // Log controller gains
        RCLCPP_INFO(get_node()->get_logger(), "Fingers controller gains loaded:");
        for (size_t i = 0; i < Kp.size(); ++i) {
            // Assuming Kp[i] and Kd[i] support index operator or providing values
            // Adjust the index/method access based on your specific Type definition
            RCLCPP_INFO(get_node()->get_logger(), 
                "[%zu] Kp: %f, %f, %f, %f | Kd: %f, %f, %f, %f", 
                i, Kp[i](0), Kp[i](1), Kp[i](2), Kp[i](3), 
                Kd[i](0), Kd[i](1), Kd[i](2), Kd[i](3));
        }

        // load hand models
        std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_hand_ros2_controllers");
        std::string conf_path = pkg_path + "/config/thunder/";
        thunder_index_handle_.load_par(conf_path + "ahand_index_identification_par.yaml");
        thunder_middle_handle_.load_par(conf_path + "ahand_middle_identification_par.yaml");
        thunder_ring_handle_.load_par(conf_path + "ahand_ring_identification_par.yaml");
        thunder_thumb_handle_.load_par(conf_path + "ahand_thumb_identification_par.yaml");
        // std::string conf_path = pkg_path + "/config/thunder/wonik_parameters";
        // thunder_index_handle_.load_par(conf_path + "ahand_index_par.yaml");
        // thunder_middle_handle_.load_par(conf_path + "ahand_middle_par.yaml");
        // thunder_ring_handle_.load_par(conf_path + "ahand_ring_par.yaml");
        // thunder_thumb_handle_.load_par(conf_path + "ahand_thumb_par.yaml");


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

        //create pbHand conroller publisher
        pBHand_controller_pub_ = get_node()->create_publisher<CmdMsg>(
            "~/pbhand_command_torque",
            out_qos);

        //create pbHand conroller publisher
        error_pub_ = get_node()->create_publisher<CmdMsg>(
            "~/controller_error",
            out_qos);


        // Instance of Controller mode 
        set_controller_mode_service_ = get_node()->create_service<std_srvs::srv::SetBool>(
            "~/set_controller_mode",
            [this](
                const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response)
            {
                if (request->data) {
                    control_mode_.store(ControllerMode::GRAVITY_COMP);
                    response->message = "Controller mode changed to: GRAVITY_COMP";
                } else {
                    control_mode_.store(ControllerMode::RUN);
                    response->message = "Controller mode changed to: RUN (Computed Torque)";
                }
                response->success = true;
                RCLCPP_INFO(
                    get_node()->get_logger(),
                    "%s",
                    response->message.c_str());
            }
        );
        // Tf2 listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_timer_ = get_node()->create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() {
                try {
                    auto transform = tf_buffer_->lookupTransform("world", "palm_link", tf2::TimePointZero);
                    Eigen::Quaterniond q(
                        transform.transform.rotation.w,
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z
                    );
                    rt_hand_orientation_.writeFromNonRT(q);
                } catch (const tf2::TransformException &) {
                    // Silenzia gli errori se la TF non è ancora disponibile
                }
            });



        // init their library
        pBHand = new BHand(eHandType_Right);
        pBHand->GetType(eHardwareType_B);
        pBHand->SetTimeInterval(0.002);
        pBHand->SetMotionType(eMotionType_JOINT_PD);


        RCLCPP_INFO(get_node()->get_logger(),"configure succesfully");
        return CallbackReturn::SUCCESS;
    }


    CallbackReturn ComputedTorque_Controller::on_activate(const rclcpp_lifecycle::State & )
    {
        control_mode_ = allegro_hand_computed_torque::ControllerMode::RUN;
        has_valid_cmd_ = false;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ComputedTorque_Controller::on_deactivate(const rclcpp_lifecycle::State & )
    {
        // release the hand: zero torque on all joints
        for (auto & cmd : command_interfaces_) {
            cmd.set_value(0.0);
        }
        rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdMsg>>(nullptr);
        control_mode_ = allegro_hand_computed_torque::ControllerMode::STOP;
        has_valid_cmd_ = false;
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
                const rclcpp::Time & , const rclcpp::Duration & period
            )
    {
        // Parse command
        // position command
        auto joint_cmd_ptr = rt_command_ptr_.readFromRT();
        if (joint_cmd_ptr && *joint_cmd_ptr) {
            if ((*joint_cmd_ptr)->position.size() == DOF_JOINTS) {
                joint_cmd_ = *(*joint_cmd_ptr);
                has_valid_cmd_ = true;
            } else {
                // bad position command received: ignore it, keep last valid one
                RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                                    "command size (%zu) does not match number of joints (%u)",
                                    (*joint_cmd_ptr)->position.size(), DOF_JOINTS);
            }
        }
        // no position command received yet (Modality Gravity Compensation)
        const ControllerMode mode = has_valid_cmd_ ? control_mode_.load() : ControllerMode::GRAVITY_COMP;
        // read state_interfaces_ : [pos_joint0, vel_joint0, eff_joint0, pos_joint1, ...]
        Eigen::VectorXd q = Eigen::VectorXd::Zero(DOF_JOINTS);
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(DOF_JOINTS);
        Eigen::VectorXd q_r = Eigen::VectorXd::Zero(DOF_JOINTS);
        Eigen::VectorXd dq_r = Eigen::VectorXd::Zero(DOF_JOINTS);
        Eigen::VectorXd ddq_r = Eigen::VectorXd::Zero(DOF_JOINTS);
        for (size_t i = 0; i < DOF_JOINTS; i++) {
            q(i) = state_interfaces_[i * 3].get_value();
            dq(i) = state_interfaces_[i * 3 + 1].get_value();
            q_r(i) = has_valid_cmd_ ? joint_cmd_.position[i] : q(i);
            dq_r(i) = (has_valid_cmd_ && joint_cmd_.velocity.size() == DOF_JOINTS) ? joint_cmd_.velocity[i] : 0.0;
        }

        // read hand orientation
        Eigen::Quaterniond hand_quat = Eigen::Quaterniond::Identity();
        auto hand_quat_ptr = rt_hand_orientation_.readFromRT();
        if (hand_quat_ptr != nullptr) {
            hand_quat = *hand_quat_ptr;
        }

        //--------------- pbHand
        double desired_position[16];
        // rotate with pbhand
        for (size_t i = 0; i < DOF_JOINTS; i++) {
            pB_current_position[i] = q(i);
            desired_position[i] = q_r(i);
        }
        double dt = period.seconds();
        pBHand->SetJointPosition(pB_current_position);
        pBHand->SetJointDesiredPosition(desired_position);
        pBHand->UpdateControl(static_cast<double>(frame) * dt);
        pBHand->GetJointTorque(pB_torque);
        frame +=1 ;
        //--------------- pbHand

        Eigen::VectorXd tau(DOF_JOINTS);
        Eigen::VectorXd tau_GC(DOF_JOINTS);

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

        // rotate hand (rotate gravity accordingly to world-palm_link transform)
        Eigen::Vector3d gravity_world(0.0, 0.0, -9.81);
        //Eigen::Vector3d palm_gravity = hand_quat.inverse() * gravity_world;

        // ----------------------------------
        // CONTROL LAW IMPLEMENTATION
        //
        // Add here the desired contro Law
        // ----------------------------------
        // ...

        tau.segment(0, 4)  =  thunder_index_handle_.get_G()  + Kp[0].asDiagonal()*e.segment( 0, 4) + Kd[0].asDiagonal()*de.segment( 0, 4);
        tau.segment(4, 4)  =  thunder_middle_handle_.get_G() + Kp[1].asDiagonal()*e.segment( 4, 4) + Kd[1].asDiagonal()*de.segment( 4, 4);
        tau.segment(8, 4)  =  thunder_ring_handle_.get_G()   + Kp[2].asDiagonal()*e.segment( 8, 4) + Kd[2].asDiagonal()*de.segment( 8, 4);
        tau.segment(12, 4) =  thunder_thumb_handle_.get_G()  + Kp[3].asDiagonal()*e.segment(12, 4) + Kd[3].asDiagonal()*de.segment(12, 4);

        tau_GC.segment(0, 4) =  thunder_index_handle_.get_G();
        tau_GC.segment(4, 4) =  thunder_middle_handle_.get_G();
        tau_GC.segment(8, 4) =  thunder_ring_handle_.get_G();
        tau_GC.segment(12, 4) =  thunder_thumb_handle_.get_G();
        // ...

        // write on command_interfaces_ : [eff_joint0, eff_joint1, ...]
        CmdMsg tau_controller;
        tau_controller.name.resize(DOF_JOINTS);
        tau_controller.position.resize(DOF_JOINTS);
        tau_controller.effort.resize(DOF_JOINTS);
        tau_controller.header.stamp = get_node()->now();
        for (size_t i = 0; i < DOF_JOINTS; i++) {
            double effort_command = 0.0;
            if (mode == allegro_hand_computed_torque::ControllerMode::GRAVITY_COMP) {
                effort_command = tau_GC(i);
            } else if (mode == allegro_hand_computed_torque::ControllerMode::RUN) {
                effort_command = tau(i);
            }
            tau_controller.name[i] = jointNames[i];
            tau_controller.position[i] = q(i);
            tau_controller.effort[i] = effort_command;
            command_interfaces_[i].set_value(effort_command);
        }
        controller_pub_->publish(tau_controller);

        // controller error
        CmdMsg error_controller;
        error_controller.name.resize(DOF_JOINTS);
        error_controller.position.resize(DOF_JOINTS);
        error_controller.header.stamp = get_node()->now();
        for (size_t i = 0; i < DOF_JOINTS; i++) {
            error_controller.name[i] = jointNames[i];
            error_controller.position[i] = e(i);
        }
        error_pub_->publish(error_controller);

        CmdMsg pb_controller;
        pb_controller.name.resize(DOF_JOINTS);
        pb_controller.effort.resize(DOF_JOINTS);
        pb_controller.header.stamp = get_node()->now();
        for (size_t i = 0; i < DOF_JOINTS; i++) {
            double effort_command = pB_torque[i];
            pb_controller.name[i] = jointNames[i];
            pb_controller.effort[i] = effort_command;
            // command_interfaces_[i].set_value(effort_command);
        }
        pBHand_controller_pub_->publish(pb_controller);


        return controller_interface::return_type::OK;
    }


}; // namespace allegro_hand_computed_torque
PLUGINLIB_EXPORT_CLASS(
    allegro_hand_computed_torque::ComputedTorque_Controller, controller_interface::ControllerInterface
);