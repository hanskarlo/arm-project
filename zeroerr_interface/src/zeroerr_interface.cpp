#include "zeroerr_interface/zeroerr_interface.h"


ZeroErrInterface::ZeroErrInterface() : Node("zeroerr_interface")
{
    // Initialize vector variables (avoid segfault)
    joint_states_.name.resize(NUM_JOINTS);
    for (uint i = 0; i < NUM_JOINTS; i++)
    {
        std::string joint_name = "j";
        joint_name.append(std::to_string(i + 1));

        joint_states_.name[i] = joint_name;
    }
    joint_states_.position.assign(NUM_JOINTS, 0.0);
    joint_states_enc_counts_.assign(NUM_JOINTS, 0.0);
    joint_commands_.assign(NUM_JOINTS, 0.0);

    arm_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("arm/state", 10);

    arm_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "arm/command",
        10,
        std::bind(&ZeroErrInterface::arm_cmd_cb_, this, std::placeholders::_1));

    if (!init_())
    {
        RCLCPP_ERROR(this->get_logger(), "Exiting");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Initialization successful!\n");

    stamp_ = this->now().seconds();

    // Create cyclic data exchange timer
    cyclic_pdo_timer_ = this->create_wall_timer(
        CYCLIC_DATA_PERIOD,
        std::bind(&ZeroErrInterface::cyclic_pdo_loop_, this));
        
    // Create joint state publisher timer
    joint_state_pub_timer_ = this->create_wall_timer(
        JOINT_STATE_PERIOD,
        std::bind(&ZeroErrInterface::joint_state_pub_, this));
}


ZeroErrInterface::~ZeroErrInterface()
{
    RCLCPP_INFO(this->get_logger(), "Releasing master...\n");
    ecrt_release_master(master);
}


/**
 * @brief Configures PDOs and joint parameters, activates EtherCAT master and 
 * allocates process data domain memory.
 * 
 * @return true successful initialization,
 * @return false if initialization failed
 */
bool ZeroErrInterface::init_()
{
    RCLCPP_INFO(this->get_logger(), "Starting...\n");


    master = ecrt_request_master(0);
    if (!master)
    {
        RCLCPP_ERROR(this->get_logger(), "Requesting master 0 failed.\n");
        return false;
    }


    if (!configure_pdos_()) return false;


    // RCLCPP_INFO(this->get_logger(), "Creating SDO requests...\n");
    // for (uint i = 0; i < NUM_JOINTS; i++)
    // {
    //     if (!(sdo[i] = ecrt_slave_config_create_sdo_request(joint_slave_configs[i], 0x603F, 0, sizeof(uint16_t)))) {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to create SDO request.\n");
    //         return false;
    //     }
    //     ecrt_sdo_request_timeout(sdo[i], 500); // ms
    // }


    // if (!set_drive_parameters_()) return false;


    RCLCPP_INFO(this->get_logger(), "Activating master...\n");
    if (ecrt_master_activate(master))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to activate master!\n");
        return false;
    }


    if (!(domain_pd = ecrt_domain_data(domain)))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get domain process data!\n");
        return false;
    }


    return true;
}


/**
 * @brief Configures joints' process data objects (PDOs) and creates 
 * process data domain (see ec_defines.h for domain layout)
 * 
 * @return true for successful configuration,
 * @return false for failed configuration
 */
bool ZeroErrInterface::configure_pdos_()
{
    RCLCPP_INFO(this->get_logger(), "Registering domain...\n");
    if (!(domain = ecrt_master_create_domain(master)))
    {
        RCLCPP_ERROR(this->get_logger(), "Domain creation failed!\n");
        return false;
    }

    

    RCLCPP_INFO(this->get_logger(), "Configuring PDOs...\n");
    for (uint i = 0; i < NUM_JOINTS; i++)
    {
        
        if (!(joint_slave_configs[i] = ecrt_master_slave_config(
                  master,
                  0, i,
                  ZEROERR_EROB)))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get slave configuration for joint %d.\n", i);
            return false;
        }


        if (ecrt_slave_config_pdos(joint_slave_configs[i], EC_END, erob_syncs_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure PDOs for joint %d.\n", i);
            return false;

        }
    }
    RCLCPP_INFO(this->get_logger(), "Configured PDOs!\n");

    RCLCPP_INFO(this->get_logger(), "Registering PDO entries...\n");
    if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs_))
    {
        RCLCPP_ERROR(this->get_logger(), "PDO entry registration failed!\n");
        return false;
    }

    return true;
}


/**
 * @brief Sets drive motion profile and sync manager (SM) parameters for all joints
 * sequentially.
 * 
 * @return true for successful parameter changes,
 * @return false if any parameter change fails
 */
bool ZeroErrInterface::set_drive_parameters_()
{
    uint32_t abort_code;
    size_t result_size;
    uint32_t data_32;
    // uint16_t data_16;
    uint8_t data_8;

    for (uint i = 0; i < NUM_JOINTS; i++)
    {
        // Set max profile velocity
        data_32 = (i < 3) ? EROB_110H120_MAX_SPEED : EROB_70H100_MAX_SPEED;
        if (ecrt_master_sdo_download(
                master,
                i,
                MAX_PROFILE_VELOCITY,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to download Max profile velocity for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                MAX_PROFILE_VELOCITY,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to upload Max profile velocity for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed max profile velocity: %u counts/s for j%i", data_32, i);


        // Profile velocity
        data_32 = (i < 3) ? (EROB_110H120_MAX_SPEED / 10) : (EROB_70H100_MAX_SPEED / 10);
        if (ecrt_master_sdo_download(
                master,
                i,
                PROFILE_VELOCITY,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change profile velocity for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                PROFILE_VELOCITY,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change profile velocity for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed profile velocity: %u for j%d", data_32, i);


        // Profile acceleration
        data_32 = (i < 3) ? (EROB_110H120_MAX_ADCEL / 10) : (EROB_70H100_MAX_ADCEL / 10);
        if (ecrt_master_sdo_download(
                master,
                i,
                PROFILE_ACCELERATION,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change profile acceleration for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                PROFILE_ACCELERATION,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change profile acceleration for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed profile acceleration: %u for j%d", data_32, i);


        // Profile deceleration
        data_32 = (i < 3) ? (EROB_110H120_MAX_ADCEL / 10) : (EROB_70H100_MAX_ADCEL / 10);
        if (ecrt_master_sdo_download(
                master,
                i,
                PROFILE_DECELERATION,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change profile deceleration for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                PROFILE_DECELERATION,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change profile deceleration for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed profile deceleration: %u for j%d", data_32, i);
        

        // Position following window
        data_32 = 10000;
        if (ecrt_master_sdo_download(
                master,
                i,
                POS_FOLLOW_WINDOW,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change Position following window for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                POS_FOLLOW_WINDOW,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change Position following window for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed Position following window %u for j%d", data_32, i);




        //* Set parameters for CSP mode
        // Set mode to CSP (0x8) mode
        data_8 = 0x08;
        if (ecrt_master_sdo_download(
                master,
                i,
                MODE_OF_OPERATION,
                &data_8,
                sizeof(data_8),
                &abort_code))
        {
            RCLCPP_INFO(this->get_logger(), "Failed to change mode of operation for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                MODE_OF_OPERATION,
                &data_8,
                sizeof(data_8),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change mode of operation for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed mode of operation 0x%x for j%d", data_8, i);

        // Set target position to current position
        if (ecrt_master_sdo_upload(
                master,
                i,
                POS_ACTUAL_INDEX,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read current position (0x6064) for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Current position: %u (counts) for j%d", data_32, i);

        if (ecrt_master_sdo_download(
                master,
                i,
                TARGET_POS_INDEX,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change target position (0x607A) for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                TARGET_POS_INDEX,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change target position (0x607A) for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Target position: %u (counts) for j%d\n\n", data_32, i);
    

        // Setup DC-Synchronization 
        ecrt_slave_config_dc(
            joint_slave_configs[i], 
            ASSIGN_ACTIVATE, 
            PERIOD_NS, 
            4400000, 0, 0);
    
    }


    return true;
}


/**
 * @brief Transitions joints through CiA402 Drive Profile State Machine.
 * 
 * @return true if all joints reached Operation Enabled state,
 * @return false if still in progress, or failed
 */
bool ZeroErrInterface::state_transition_()
{
    uint16_t status_word;
    uint16_t control_word;
    

    // Read status + control words
    status_word = EC_READ_U16(domain_pd + status_word_offset[joint_no_]);
    control_word = EC_READ_U16(domain_pd + ctrl_word_offset[joint_no_]);


    //* CiA 402 PDS FSA commissioning
    if ((status_word & 0b01001111) == 0b00000000)
    {
        if (driveState[joint_no_] != NOT_READY)
        {
            driveState[joint_no_] = NOT_READY;
            // RCLCPP_INFO(this->get_logger(), " J%d State: Not ready", joint_no_);
        }
    }
    else if ((status_word & 0b01001111) == 0b01000000)
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no_], (control_word & 0b01111110) | 0b00000110);
        
        if (driveState[joint_no_] != SWITCH_ON_DISABLED)
        {
            driveState[joint_no_] = SWITCH_ON_DISABLED;
            // RCLCPP_INFO(this->get_logger(), " J%d State: Switch on disabled", joint_no_);
        }
    }
    else if ((status_word & 0b01101111) == 0b00100001)
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no_], (control_word & 0b01110111) | 0b00000111);
        
        if (driveState[joint_no_] != READY)
        {
            driveState[joint_no_] = READY;
            // RCLCPP_INFO(this->get_logger(), " J%d State: Ready to switch on", joint_no_);
        }
    }
    else if ((status_word & 0b01101111) == 0b00100011)
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no_], (control_word & 0b01111111) | 0b00001111);

        if (driveState[joint_no_] != SWITCHED_ON)
        {
            driveState[joint_no_] = SWITCHED_ON;
            // RCLCPP_INFO(this->get_logger(), " J%d State: Switched on", joint_no_);
        }
    }
    else if ((status_word & 0b01101111) == 0b00100111)
    {
        if (driveState[joint_no_] != OPERATION_ENABLED)
        {
            driveState[joint_no_] = OPERATION_ENABLED;
            RCLCPP_INFO(this->get_logger(), " J%d State: Operation enabled!", joint_no_);
            
            //* Current joint reached Operation Enabled, enable next joint
        }
        joint_no_++;    
    }
    else if ((status_word & 0b01101111) == 0b00000111)
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no_], (control_word & 0b01111111) | 0b00001111);
        driveState[joint_no_] = QUICK_STOP_ACTIVE;
        RCLCPP_INFO(this->get_logger(), " J%d State: Quick stop active", joint_no_);
    }
    else if ((status_word & 0b01001111) == 0b00001111)
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no_], 0x0080);
        driveState[joint_no_] = FAULT_REACTION_ACTIVE;
        RCLCPP_INFO(this->get_logger(), " J%d State: Fault reaction active", joint_no_);
    }
    else if ((status_word & 0b01001111) == 0b00001000)
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no_], (control_word & 0b11111111) | 0b10000000);

        if (driveState[joint_no_] != FAULT)
        {
            driveState[joint_no_] = FAULT;
            RCLCPP_INFO(this->get_logger(), " J%d State: Fault", joint_no_);
        }
    }

    if (joint_no_ == NUM_JOINTS) // All joints reached Operation Enabled
    {
        joint_no_ = 0;
        return true;
    }
    else
        return false;
}


/**
 * @brief Read selected SDO from selected joint.
 * 
 * @param joint_no_ Joint to read SDO from
 */
void ZeroErrInterface::read_sdos(int joint_no_)
{
    switch (ecrt_sdo_request_state(sdo[joint_no_])) {
        case EC_REQUEST_UNUSED: // request was not used yet
            ecrt_sdo_request_read(sdo[joint_no_]); // trigger first read
            break;
        case EC_REQUEST_BUSY:
            RCLCPP_INFO(this->get_logger(), "Still busy...\n");
            break;
        case EC_REQUEST_SUCCESS:
            RCLCPP_INFO(this->get_logger(), "Error (0x603F): 0x%04X\n",
                    EC_READ_U16(ecrt_sdo_request_data(sdo[joint_no_])));
            ecrt_sdo_request_read(sdo[joint_no_]); // trigger next read
            break;
        case EC_REQUEST_ERROR:
            RCLCPP_INFO(this->get_logger(), "Failed to read SDO!\n");
            ecrt_sdo_request_read(sdo[joint_no_]); // retry reading
            break;
    }
}


/**
 * @brief Cyclic process data object exchange loop.
 * 
 * @note Frequency is controlled through CYCLIC_DATA_PERIOD member. Try not to change.
 * 
 * @warning Maintain consistent data exchange timing to avoid EtherCAT sync issues.
 * 
 */
void ZeroErrInterface::cyclic_pdo_loop_()
{
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    // check process data state (optional)
    // check_domain_state_();

    // Read joint states
    for (uint i = 0; i < NUM_JOINTS; i++)
        joint_states_enc_counts_[i] = EC_READ_S32(domain_pd + actual_pos_offset[i]);


    if (counter_)
    {
        counter_--;
    }
    else    // Do below every 1s
    {
        counter_ = ( 1 / CYCLIC_DATA_PERIOD.count() );

        // check for master state (optional)
        // check_master_state();

        // check for islave configuration state(s) (optional)
        // if (!operational_)
            // operational_ = check_slave_config_states_(joint_no__);
        

        // RCLCPP_INFO(this->get_logger(), "Hi");

        // read process data SDO
        // read_sdos(joint_no__);
    }


    // If all joints reached EtherCAT OP state
    if (joints_OP_)
    {
        // Transit joints through CiA402 PDS FSA
        joints_op_enabled_ = state_transition_();
    }        
    else
    {
        joints_OP_ = check_slave_config_states_();

        //* If not all joints reached OP state for 4s, retry
        if ((this->now().seconds() - stamp_) >= 4)
        {
            RCLCPP_INFO(this->get_logger(), "Not all joints reached OP, retrying");
            ecrt_master_reset(master);

            // receive process data
            ecrt_master_receive(master);
            ecrt_domain_process(domain);

            stamp_ = this->now().seconds();
        }
    }


    // If all joints reached CiA402 Drive State Operation Enabled
    if (joints_op_enabled_)
    {
        RCLCPP_INFO(this->get_logger(), "Writing target pos %u", joint_commands_[5]);
        for (uint i = 0; i < NUM_JOINTS; i++)
        {
            EC_WRITE_S32(domain_pd + target_pos_offset[i], joint_commands_[i]);
        }
    }



    // Sync every cycle
    clock_gettime(CLOCK_TO_USE, &time_ns);
    ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time_ns));
    ecrt_master_sync_slave_clocks(master);



    // send process data
    ecrt_domain_queue(domain);
    ecrt_master_send(master);
}

double ZeroErrInterface::convert_count_to_rad_(int32_t counts)
{
    double radians;

    if ( (counts > 0) ? (counts > MAX_COUNT) : (abs(counts) > MAX_COUNT))
    {
        double scaled_counts = (counts > 0) ? (MAX_COUNT - (counts % MAX_COUNT)) : -(MAX_COUNT - (abs(counts) % MAX_COUNT));
        radians = COUNT_TO_RAD(scaled_counts);
    }
    else
    {
        radians = COUNT_TO_RAD(counts);
    }

    return radians;
}

/**
 * @brief Joint state publisher callback.
 * 
 * @note Frequency is controlled through JOINT_STATE_PERIOD member.
 * 
 */
void ZeroErrInterface::joint_state_pub_()
{
    joint_states_.header.stamp = this->now();

    for (uint i = 0; i < NUM_JOINTS; i++)
    {
        joint_states_.position[i] = convert_count_to_rad_(joint_states_enc_counts_[i]);
    }

    arm_state_pub_->publish(joint_states_);
}


/**
 * @brief Outputs state of EtherCAT master.
 * 
 */
void ZeroErrInterface::check_master_state_()
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        RCLCPP_INFO(this->get_logger(), "%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        RCLCPP_INFO(this->get_logger(), "AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        RCLCPP_INFO(this->get_logger(), "Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}


/**
 * @brief Outputs process data domain state.
 * 
 */
void ZeroErrInterface::check_domain_state_()
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain, &ds);

    if (ds.working_counter != domain_state.working_counter)
        RCLCPP_INFO(this->get_logger(), "Domain: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain_state.wc_state)
        RCLCPP_INFO(this->get_logger(), "Domain: State %u.\n", ds.wc_state);

    domain_state = ds;
}


/**
 * @brief Check the EtherCAT slave state of each joint sequentially.
 * 
 * @return true when all joints reach OP state,
 * @return false when joints havent reached OP state yet
 */
bool ZeroErrInterface::check_slave_config_states_()
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(joint_slave_configs[joint_no_], &s);
    
    // if (s.al_state != joint_ec_states[joint_no_].al_state)
    //     RCLCPP_INFO(this->get_logger(), "J%d: State 0x%02X.\n", joint_no_, s.al_state);
    // if (s.online != joint_ec_states[joint_no_].online)
    //     RCLCPP_INFO(this->get_logger(), "J%d: %s.\n", joint_no_, s.online ? "online" : "offline");
    if (s.operational != joint_ec_states[joint_no_].operational)
        RCLCPP_INFO(this->get_logger(), "J%d: %soperational.", joint_no_, 
            s.operational ? "" : "Not ");    

    joint_ec_states[joint_no_] = s;


    // Current joint is operational -- check next joint
    if (joint_ec_states[joint_no_].operational)
        joint_no_++; 


    if (joint_no_ == 6) // All joints operational
    {
        // Reset joint tracker
        joint_no_ = 0;
        return true;
    }
    else
    {
        return false;
    }
}


/**
 * @brief Updates joint commands via arm command topic.
 * 
 * @param arm_cmd arm command JointState message 
 */
void ZeroErrInterface::arm_cmd_cb_(sensor_msgs::msg::JointState::UniquePtr arm_cmd)
{
    // RCLCPP_INFO(this->get_logger(), "arm_cmd_cb fired");

    for (uint i = 0; i < arm_cmd->position.size(); i++) {
        joint_commands_[i] = RAD_TO_COUNT( arm_cmd->position[i] );
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ZeroErrInterface>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}