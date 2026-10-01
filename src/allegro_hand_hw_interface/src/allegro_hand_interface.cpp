#include "allegro_hand_hw_interface/allegro_hand_interface.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <yaml-cpp/yaml.h>

#define LOGGER_NAME "allegro_hand_interface"

namespace allegro_hand_hw_interface{



    AllegroHandV5::AllegroHandV5(){
        mutex = new std::mutex();

        // Initialize values: joint names should match URDF, desired torque and
        // velocity are both zero.
        std::memset(desired_torque, 0.0, sizeof(desired_torque));
        std::memset(current_velocity, 0.0, sizeof(current_velocity));
        std::memset(current_position_filtered, 0.0, sizeof(current_position_filtered));
        std::memset(current_velocity_filtered, 0.0, sizeof(current_velocity_filtered));
    }
    AllegroHandV5::~AllegroHandV5()
    {
        stopServiceThread();
        sendZeroTorque();
        if (canDevice) delete canDevice;
        delete mutex;
        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),"AllegroHand interface deleted");
    };


    CallbackReturn AllegroHandV5::on_init(const hardware_interface::HardwareInfo & info)
    {
        // Initialize CAN device
        canDevice = new allegro::AllegroHandDrv();
        can_ch = info.hardware_parameters.at("device");
        whichHand = info.hardware_parameters.at("handness");
        whichType = info.hardware_parameters.at("type");
        
        auto it = info.hardware_parameters.find("position_control");
        if (it != info.hardware_parameters.end()) {
            // 3. Set to true only if the string value is "true"
            if (it->second == "true" || it->second == "1") {
                controlMode = ControlMode::POSITION;
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Using Position Control");
            }
        }

        // Initialize  BHand library (Position Controller)
        if (whichHand == "left") pBHand = new BHand(eHandType_Left);
        else                     pBHand = new BHand(eHandType_Right);
        pBHand->GetType(whichType == "A" ? eHardwareType_A : eHardwareType_B);
        pBHand->SetTimeInterval(ALLEGRO_CONTROL_TIME_INTERVAL);
        pBHand->SetMotionType(eMotionType_JOINT_PD);

        // iniitialize Joint Kalman Filter
        std::memset(desired_torque, 0.0, sizeof(desired_torque));
        std::memset(current_velocity, 0.0, sizeof(current_velocity));
        std::memset(current_position_filtered, 0.0, sizeof(current_position_filtered));
        std::memset(current_velocity_filtered, 0.0, sizeof(current_velocity_filtered));
        for (int i = 0; i < DOF_JOINTS; i++) {
            kf[i] = kalman_filter_joint::KalmanFilterJoint(std::stod(info.hardware_parameters.at("kalman_filter_process_noise")),
                                                            std::stod(info.hardware_parameters.at("kalman_filter_measurement_noise")));
        }

        // set if pressure is requested (Default = true)
        it = info.hardware_parameters.find("pressure");
        if (it != info.hardware_parameters.end()) {
            pressure_req_ = (it->second == "true" || it->second == "True" || it->second == "1");
        } else {
            RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                        "Parameter 'pressure' not found, defaulting to True.");
            pressure_req_ = true;
        }


        return CallbackReturn::SUCCESS;
    };

    CallbackReturn AllegroHandV5::on_configure(const rclcpp_lifecycle::State& )
    {
        // setup the communication thread, request comms info
        if (canDevice->init(can_ch)) {
                usleep(3000);
        }
        else {
            delete canDevice;
            canDevice = 0;
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Cannot initialize CAN device");
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "CAN device initialized");


        // Create ROS2 activate/deactivate service
        activate_node_ = std::make_shared<rclcpp::Node>("allegro_hand_service");
        activate_service_ = activate_node_->create_service<std_srvs::srv::SetBool>(
            "activate",
            [this](std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response) 
            {
                active_ = request->data;
                response->success = true;
                response->message = active_ ? "ON" : "OFF";
                RCLCPP_INFO(activate_node_->get_logger(), "Security flag toggled: %s", active_ ? "ENABLED" : "DISABLED");
            });

        // Create ROS2 save pose service
        save_pose_service_ = activate_node_->create_service<allegro_hand_interfaces::srv::SavePose>(
            "save_pose",
            [this](const std::shared_ptr<allegro_hand_interfaces::srv::SavePose::Request> request,
                std::shared_ptr<allegro_hand_interfaces::srv::SavePose::Response> response)
            {
                response->success = savePose(request->name, request->file_path, response->message);
                if (response->success) RCLCPP_INFO(activate_node_->get_logger(), "%s", response->message.c_str());
                else                   RCLCPP_ERROR(activate_node_->get_logger(), "%s", response->message.c_str());
            });
        executor_.add_node(activate_node_);
        executor_thread_ = std::thread([this]() { executor_.spin(); });


        return CallbackReturn::SUCCESS;
    };

    CallbackReturn AllegroHandV5::on_cleanup(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Pass cleanup");

        return CallbackReturn::SUCCESS;
    };

    CallbackReturn AllegroHandV5::on_activate(const rclcpp_lifecycle::State&)
    {
        first_cycle_ = true;    // re-check hand and hold current pose on next valid frame
        active_ = true;
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Pass activate");
        return CallbackReturn::SUCCESS;
    };

    CallbackReturn AllegroHandV5::on_deactivate(const rclcpp_lifecycle::State&)
    {
        //write zero torque to the hand
        active_ = false;
        sendZeroTorque();

        // Stop the executor thread
        stopServiceThread();



        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Pass Deactvate");
        return CallbackReturn::SUCCESS;
    };

    CallbackReturn AllegroHandV5::on_shutdown(const rclcpp_lifecycle::State&)
    {
        //write zero torque to the hand
        active_ = false;
        sendZeroTorque();
        stopServiceThread();
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Pass Shutdown");
        return CallbackReturn::SUCCESS;
    };

    CallbackReturn AllegroHandV5::on_error(const rclcpp_lifecycle::State&)
    {
        //write zero torque to the hand
        active_ = false;
        sendZeroTorque();
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Pass error");
        return CallbackReturn::SUCCESS;
    };

    std::vector<hardware_interface::StateInterface> AllegroHandV5::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (int i=0; i<DOF_JOINTS; i++){
            try{
                state_interfaces.emplace_back(
                    jointNames[i],
                    hardware_interface::HW_IF_POSITION,
                    &current_position_filtered[i]
                );
                state_interfaces.emplace_back(
                    jointNames[i],
                    hardware_interface::HW_IF_VELOCITY,
                    &current_velocity_filtered[i]
                );
                state_interfaces.emplace_back(
                    jointNames[i],
                    hardware_interface::HW_IF_EFFORT,
                    &desired_torque[i]
                );
            }
            catch(std::logic_error &err)
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Raised error %s during the state interface exporting ID:%d bus:%d",err.what(),i,0);
                assert(false);
            }
        }
        num_state_interfaces = DOF_JOINTS*3;

        if(pressure_req_){
            for(int i=0; i<4; i++){
                try{
                    state_interfaces.emplace_back(pressureSensorNames[i], HW_IF_PRESSURE, &tactile_sensor[i]);
                    num_state_interfaces += 1;
                }
                catch(std::logic_error &err)
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Raised error %s during the state interface exporting ID:%d bus:%d",err.what(),i,0);
                    assert(false);
                }
            }
        }
        return state_interfaces;
    };

    std::vector<hardware_interface::CommandInterface> AllegroHandV5::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        std::vector<std::string> int_type;
        for (int i=0; i<DOF_JOINTS; i++){
            // position controller interface
            try{
                command_interfaces.emplace_back(
                    jointNames[i],
                    hardware_interface::HW_IF_POSITION,
                    &desired_position[i]
                );
            }
            catch(std::logic_error &err)
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Raised error %s during the command interface exporting ID:%d bus:%d",err.what(),i,0);
                assert(false);
            }

            // effort controller interface
            try{
                command_interfaces.emplace_back(
                    jointNames[i],
                    hardware_interface::HW_IF_EFFORT,
                    &desired_torque[i]
                );
            }
            catch(std::logic_error &err)
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Raised error %s during the command interface exporting ID:%d bus:%d",err.what(),i,0);
                assert(false);
            }
        }
        return command_interfaces;
    };

    hardware_interface::return_type AllegroHandV5::read(const rclcpp::Time & , const rclcpp::Duration & period)
    {
        /*
        // Calculate loop time;
        tnow = get_clock()->now();;
        dt = ALLEGRO_CONTROL_TIME_INTERVAL;//1e-9 * (tnow - tstart).nanoseconds();

        // When running gazebo, sometimes the loop gets called *too* often and dt will
        // be zero. Ensure nothing bad (like divide-by-zero) happens because of this.
        if(dt <= 0) {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"AllegroNode::updateController dt is zero.");
            canDevice = 0;
            return hardware_interface::return_type::ERROR;
        }*/

        //tstart = tnow;
        //dt = ALLEGRO_CONTROL_TIME_INTERVAL;//1e-9 * (tnow - tstart).nanoseconds();

        elap_time += period.seconds();
        double dt = elap_time;

        if (canDevice){
            // try to update joint positions through CAN comm:
            lEmergencyStop = canDevice->readCANFrames();

            // check if all positions are updated:
            if (lEmergencyStop == 0 && canDevice->isJointInfoReady()){
                // back-up previous joint positions:
                for (int i = 0; i < DOF_JOINTS; i++) {
                    previous_position[i] = current_position[i];
                    previous_position_filtered[i] = current_position_filtered[i];
                    previous_velocity[i] = current_velocity[i];
                }

                // update joint positions after logging previous step:
                canDevice->getJointInfo(current_position);
                for (int i = 0; i < DOF_JOINTS; i++) {
                    current_position[i] = wrapAngleError(current_position[i], 0);
                }
                {
                    std::lock_guard<std::mutex> lock(*mutex);
                    std::memcpy(position_snapshot_, current_position, sizeof(position_snapshot_));
                    position_received_ = true;
                }
                pBHand->SetJointPosition(current_position);
                // a) Finite Difference:
                if(controlMode == ControlMode::POSITION){
                    for (int i = 0; i < DOF_JOINTS; i++) {
                        current_position_filtered[i] = current_position[i];
                        current_velocity[i] = (current_position[i] - previous_position[i]) / dt;
                        current_velocity_filtered[i] =  current_velocity[i];
                    }
                }else{
                    // b) Low-pass filtering:
                    for (int i = 0; i < DOF_JOINTS; i++) {
                        current_position_filtered[i] = (0.6 * current_position_filtered[i]) +
                                                    (0.198 * previous_position[i]) +
                                                    (0.198 * current_position[i]);
                        current_velocity[i] = (current_position_filtered[i] - previous_position_filtered[i]) / dt;
                        current_velocity_filtered[i] = (0.6 * current_velocity_filtered[i]) +
                                                    (0.198 * previous_velocity[i]) +
                                                    (0.198 * current_velocity[i]);
                        current_velocity[i] = (current_position[i] - previous_position[i]) / dt;
                    }
                    // c) Kalman filtering:
                    // for (int i = 0; i < DOF_JOINTS; i++) {
                    //     kf[i].prediction(dt);
                    //     kf[i].update(current_position[i]);
                    //     kf[i].get_q(current_position_filtered[i]);
                    //     kf[i].get_dq(current_velocity[i]);
                    // }
                }
                // Store pressure values if needed
                if (pressure_req_){
                    for (int i = 0; i < 4; i++) {
                        tactile_sensor[i] = static_cast<double>(fingertip_sensor[i]);
                    }
                }

                // flag reset is done in write(), after the torque is sent
                new_data_ = true;
                elap_time = 0.0;
            }
        }

        if (first_cycle_ && new_data_){
            // check handedness
            if(whichHand.compare("left") == 0){
                if(canDevice->RIGHT_HAND){
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"WRONG HANDEDNESS DETECTED!");
                    delete canDevice;
                    canDevice = nullptr;
                    return hardware_interface::return_type::ERROR;
                }
            }else{
                if(!canDevice->RIGHT_HAND){
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"WRONG HANDEDNESS DETECTED!");
                    delete canDevice;
                    canDevice = nullptr;
                    return hardware_interface::return_type::ERROR;
                }
            }
            // check hand type 
            if(whichType.compare("A") == 0){
                if(!canDevice->HAND_TYPE_A){
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"WRONG TYPE DETECTED!");
                    delete canDevice;
                    canDevice = nullptr;
                    return hardware_interface::return_type::ERROR;
                }
            }else{
                if(canDevice->HAND_TYPE_A){
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"WRONG TYPE DETECTED!");
                    delete canDevice;
                    canDevice = nullptr;
                    return hardware_interface::return_type::ERROR;
                }
            }
            // hold on current position when position controller active 
            if(controlMode == ControlMode::POSITION){
                std::memcpy(desired_position, current_position, sizeof(desired_position));
            }
            first_cycle_ = false;
        }


        // Stop program when Allegro Hand is switched off
        if (lEmergencyStop < 0) {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Allegro Hand Node is Shutting Down! (Emergency Stop)");
            rclcpp::shutdown();
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    };

    hardware_interface::return_type AllegroHandV5::write(const rclcpp::Time & , const rclcpp::Duration & )
    {
        // Like allegro_node: compute/send only when a complete new CAN frame is available
        if (!canDevice || !new_data_) return hardware_interface::return_type::OK;
        new_data_ = false;

        // Check if using position controller then exploits bHand library
        if(controlMode == ControlMode::POSITION){
            pBHand->SetJointDesiredPosition(desired_position);
            pBHand->UpdateControl(static_cast<double>(frame) * ALLEGRO_CONTROL_TIME_INTERVAL);
            pBHand->GetJointTorque(desired_torque);
        }
        frame+=1;

        // check if hand is activate
        if (!active_) {
            //RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Allegro Hand disabled");
            std::memset(desired_torque, 0.0, sizeof(desired_torque));
        }

        // set & write torque to each joint:
        canDevice->setTorque(desired_torque);
        lEmergencyStop = canDevice->writeJointTorque();
        canDevice->resetJointInfoReady();

        // Stop program when Allegro Hand is switched off
        if (lEmergencyStop < 0) {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Allegro Hand Node is Shutting Down! (Emergency Stop)");
            rclcpp::shutdown();
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    };

    void AllegroHandV5::stopServiceThread()
    {
        executor_.cancel();
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
        if (activate_node_) {
            executor_.remove_node(activate_node_);
            activate_service_.reset();
            save_pose_service_.reset();
            activate_node_.reset();
        }
    }

    void AllegroHandV5::sendZeroTorque()
    {
        if (!canDevice) return;
        std::memset(desired_torque, 0.0, sizeof(desired_torque));

        // _writeDevices() only sends when a complete frame was received: wait for it (max ~20 ms)
        for (int k = 0; k < 40 && !canDevice->isJointInfoReady(); k++) {
            canDevice->readCANFrames();
            usleep(500);
        }
        canDevice->setTorque(desired_torque);
        canDevice->writeJointTorque();
        canDevice->resetJointInfoReady();
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Zero torque sent to the hand");
    }

    bool AllegroHandV5::savePose(const std::string & name, const std::string & file_path, std::string & message)
    {
        if (name.empty() || file_path.empty()) {
            message = "Pose name and file path must not be empty";
            return false;
        }

        // copy last measured position
        double pose[DOF_JOINTS];
        {
            std::lock_guard<std::mutex> lock(*mutex);
            if (!position_received_) {
                message = "No joint position received from the hand yet";
                return false;
            }
            std::memcpy(pose, position_snapshot_, sizeof(pose));
        }

        // add/replace the pose in the YAML file, keeping the other stored poses
        try {
            YAML::Node poses;
            if (std::filesystem::exists(file_path)) poses = YAML::LoadFile(file_path);
            poses[name]["position"] = std::vector<double>(pose, pose + DOF_JOINTS);
            poses[name]["position"].SetStyle(YAML::EmitterStyle::Flow);

            auto dir = std::filesystem::path(file_path).parent_path();
            if (!dir.empty()) std::filesystem::create_directories(dir);
            std::ofstream out(file_path);
            out << poses << "\n";
        } catch (const std::exception & e) {
            message = std::string("Failed to save pose: ") + e.what();
            return false;
        }

        message = "Pose '" + name + "' saved to " + file_path;
        return true;
    }

    double AllegroHandV5::wrapAngleError(double q1, double q2, bool is_rad)
    {
        double dq = q1 - q2;

        if (is_rad) {
            // Range: (-PI, PI]
            return std::atan2(std::sin(dq), std::cos(dq));
        } else {
            // Convert to rad, wrap, then convert back to deg
            const double deg2rad = M_PI / 180.0;
            const double rad2deg = 180.0 / M_PI;
            
            double dq_rad = dq * deg2rad;
            double wrapped_rad = std::atan2(std::sin(dq_rad), std::cos(dq_rad));
            
            return wrapped_rad * rad2deg;
        }
    }

} // namespace allegro_hand_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  allegro_hand_hw_interface::AllegroHandV5,
  hardware_interface::SystemInterface)