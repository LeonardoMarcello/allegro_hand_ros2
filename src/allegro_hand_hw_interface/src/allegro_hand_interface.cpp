#include "allegro_hand_hw_interface/allegro_hand_interface.hpp"
#include <cmath>

#define LOGGER_NAME "allegro_hand_interface"

namespace allegro_hand_hw_interface{



    AllegroHand_Interface::AllegroHand_Interface()
    {
        mutex = new std::mutex();

        // Initialize values: joint names should match URDF, desired torque and
        // velocity are both zero.
        for (int i = 0; i < DOF_JOINTS; i++) {
            desired_torque[i] = 0.0;
            current_velocity[i] = 0.0;
            current_position_filtered[i] = 0.0;
            current_velocity_filtered[i] = 0.0;
        }
    }
    AllegroHand_Interface::~AllegroHand_Interface()
    {
        if (canDevice) delete canDevice;
        delete mutex;
        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),"AllegroHand interface deleted");
    };


    CallbackReturn AllegroHand_Interface::on_init(const hardware_interface::HardwareInfo & info)
    {
        // Initialize CAN device
        canDevice = new allegro::AllegroHandDrv();
        can_ch = info.hardware_parameters.at("device");
        whichHand = info.hardware_parameters.at("handness");
        whichType = info.hardware_parameters.at("type");

        return CallbackReturn::SUCCESS;
    };

    CallbackReturn AllegroHand_Interface::on_configure(const rclcpp_lifecycle::State& )
    {
        // setup the communication thread
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
        return CallbackReturn::SUCCESS;
    };

    CallbackReturn AllegroHand_Interface::on_cleanup(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Pass cleanup");

        return CallbackReturn::SUCCESS;
    };

    CallbackReturn AllegroHand_Interface::on_activate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Pass activate");
        return CallbackReturn::SUCCESS;
    };

    CallbackReturn AllegroHand_Interface::on_deactivate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Pass Deactvate");
        return CallbackReturn::SUCCESS;
    };

    CallbackReturn AllegroHand_Interface::on_shutdown(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Pass Shutdown");
        return CallbackReturn::SUCCESS;
    };

    CallbackReturn AllegroHand_Interface::on_error(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),"Pass error");
        return CallbackReturn::SUCCESS;
    };

    std::vector<hardware_interface::StateInterface> AllegroHand_Interface::export_state_interfaces()
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
                    state_interfaces.emplace_back(pressureSensorNames[i] ,HW_IF_PRESSURE, &tactile_sensor[i]);
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

    std::vector<hardware_interface::CommandInterface> AllegroHand_Interface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        std::vector<std::string> int_type;
        for (int i=0; i<DOF_JOINTS; i++){
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

    hardware_interface::return_type AllegroHand_Interface::read(const rclcpp::Time & , const rclcpp::Duration & )
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

        tstart = tnow;

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

                // update joint positions:
                canDevice->getJointInfo(current_position);

                // low-pass filtering:
                for (int i = 0; i < DOF_JOINTS; i++) {
                    current_position_filtered[i] = current_position[i];
                    current_velocity[i] =
                            (current_position[i] - previous_position[i]) / dt;
                    current_velocity_filtered[i] =  current_velocity[i];;
                }
            }
        }

        if (first_cycle_){
            // check handedness
            if(whichHand.compare("left") == 0){
                if(canDevice->RIGHT_HAND){
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"WRONG HANDEDNESS DETECTED!");
                    canDevice = 0;
                    return hardware_interface::return_type::ERROR;
                }
            }else{
                if(!canDevice->RIGHT_HAND){
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"WRONG HANDEDNESS DETECTED!");
                    canDevice = 0;
                    return hardware_interface::return_type::ERROR;
                }
            }
            // check hand type 
            if(whichType.compare("A") == 0){
                if(!canDevice->HAND_TYPE_A){
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"WRONG TYPE DETECTED!");
                    canDevice = 0;
                    return hardware_interface::return_type::ERROR;
                }
            }else{
                if(canDevice->HAND_TYPE_A){
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"WRONG TYPE DETECTED!");
                    canDevice = 0;
                    return hardware_interface::return_type::ERROR;
                }
            }
            first_cycle_ = false;
        }


        // Stop program when Allegro Hand is switched off
        if (lEmergencyStop < 0) {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Allegro Hand Node is Shutting Down! (Emergency Stop)");
            rclcpp::shutdown();
        }

        return hardware_interface::return_type::OK;
    };

    hardware_interface::return_type AllegroHand_Interface::write(const rclcpp::Time & , const rclcpp::Duration & )
    {
        // set & write torque to each joint:
        canDevice->setTorque(desired_torque);
        lEmergencyStop = canDevice->writeJointTorque();

        // Stop program when Allegro Hand is switched off
        if (lEmergencyStop < 0) {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),"Allegro Hand Node is Shutting Down! (Emergency Stop)");
            rclcpp::shutdown();
        }

        return hardware_interface::return_type::OK;
    };


} // namespace allegro_hand_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  allegro_hand_hw_interface::AllegroHand_Interface,
  hardware_interface::SystemInterface)