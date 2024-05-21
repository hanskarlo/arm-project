#include "arm_ethercat_interface/arm_ethercat_interface.h"
#include "realtime_tools/thread_priority.hpp"


ZeroErrInterface::ZeroErrInterface() : Node("arm_ethercat_interface")
{

    normal_prio_cbg_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );


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

    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
    options.callback_group = normal_prio_cbg_;

    arm_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "arm/command",
        10,
        std::bind(&ZeroErrInterface::arm_cmd_cb_, this, std::placeholders::_1),
        options);


    if (!init_())
    {
        RCLCPP_ERROR(this->get_logger(), "Exiting");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Initialization successful!\n");

    stamp_ = this->now().seconds();


    // Create cyclic data exchange timer

    CYCLIC_DATA_PERIOD = std::chrono::milliseconds( MSEC_PER_SEC / FREQUENCY );
    RCLCPP_INFO(this->get_logger(), "Cyclic data exchange running at %dms", (MSEC_PER_SEC / FREQUENCY));


    cyclic_pdo_timer_ = this->create_wall_timer(
        CYCLIC_DATA_PERIOD,
        std::bind(&ZeroErrInterface::cyclic_pdo_loop_, this),
        high_prio_cbg_);
        
    // Create joint state publisher timer
    joint_state_pub_timer_ = this->create_wall_timer(
        JOINT_STATE_PERIOD,
        std::bind(&ZeroErrInterface::joint_state_pub_, this),
        normal_prio_cbg_);
    
    clock_gettime(CLOCK_TO_USE, &wakeupTime);
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


    if (!set_drive_parameters_()) return false;


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

    for (uint i = 0; i < NUM_JOINTS; i++)
    {
        // Set target velocity
        uint32_t initial_target_vel = 0;
        if (ecrt_master_sdo_download(
                master,
                i,
                TARGET_VELOCITY,
                (uint8_t *)&initial_target_vel,
                sizeof(initial_target_vel),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to download target velocity for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                TARGET_VELOCITY,
                (uint8_t *)&initial_target_vel,
                sizeof(initial_target_vel),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to upload target velocity for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed target velocity: %u counts/s for j%i", initial_target_vel, i);


        // Set max velocity
        uint32_t max_velocity = (i < 3) ? EROB_110H120_MAX_SPEED : EROB_70H100_MAX_SPEED;
        if (ecrt_master_sdo_download(
                master,
                i,
                MAX_VELOCITY,
                (uint8_t *)&max_velocity,
                sizeof(max_velocity),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to download Max velocity for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                MAX_VELOCITY,
                (uint8_t *)&max_velocity,
                sizeof(max_velocity),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to upload Max velocity for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed max velocity: %u counts/s for j%i", max_velocity, i);


        // Set max profile velocity
        uint32_t max_profile_velocity = (i < 3) ? EROB_110H120_MAX_SPEED : EROB_70H100_MAX_SPEED;
        if (ecrt_master_sdo_download(
                master,
                i,
                MAX_PROFILE_VELOCITY,
                (uint8_t *)&max_profile_velocity,
                sizeof(max_profile_velocity),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to download Max profile velocity for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                MAX_PROFILE_VELOCITY,
                (uint8_t *)&max_profile_velocity,
                sizeof(max_profile_velocity),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to upload Max profile velocity for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed max profile velocity: %u counts/s for j%i", max_profile_velocity, i);


        // Profile velocity
        uint32_t profile_velocity = (i < 3) ? (EROB_110H120_MAX_SPEED) : (EROB_70H100_MAX_SPEED);
        if (ecrt_master_sdo_download(
                master,
                i,
                PROFILE_VELOCITY,
                (uint8_t *)&profile_velocity,
                sizeof(profile_velocity),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change profile velocity for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                PROFILE_VELOCITY,
                (uint8_t *)&profile_velocity,
                sizeof(profile_velocity),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change profile velocity for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed profile velocity: %u for j%d", profile_velocity, i);


        // Profile acceleration
        uint32_t profile_adcel = (i < 3) ? (EROB_110H120_MAX_ADCEL) : (EROB_70H100_MAX_ADCEL);
        if (ecrt_master_sdo_download(
                master,
                i,
                PROFILE_ACCELERATION,
                (uint8_t *)&profile_adcel,
                sizeof(profile_adcel),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change profile acceleration for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                PROFILE_ACCELERATION,
                (uint8_t *)&profile_adcel,
                sizeof(profile_adcel),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change profile acceleration for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed profile acceleration: %u for j%d", profile_adcel, i);


        // Profile deceleration
        if (ecrt_master_sdo_download(
                master,
                i,
                PROFILE_DECELERATION,
                (uint8_t *)&profile_adcel,
                sizeof(profile_adcel),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change profile deceleration for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                PROFILE_DECELERATION,
                (uint8_t *)&profile_adcel,
                sizeof(profile_adcel),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change profile deceleration for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed profile deceleration: %u for j%d", profile_adcel, i);
        

        // Position following error window
        uint32_t pos_follow_err_window = 1000000000;
        if (ecrt_master_sdo_download(
                master,
                i,
                POS_FOLLOW_ERR_WINDOW,
                (uint8_t *)&pos_follow_err_window,
                sizeof(pos_follow_err_window),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change Position following window for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                POS_FOLLOW_ERR_WINDOW,
                (uint8_t *)&pos_follow_err_window,
                sizeof(pos_follow_err_window),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change Position following window for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed Position following error window %u for j%d", pos_follow_err_window, i);


        // Position window
        uint32_t pos_window = pos_follow_err_window;
        if (ecrt_master_sdo_download(
                master,
                i,
                POS_WINDOW,
                (uint8_t *)&pos_window,
                sizeof(pos_window),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change position window for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                POS_WINDOW,
                (uint8_t *)&pos_window,
                sizeof(pos_window),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change position window for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed position window %u for j%d", pos_window, i);


        // Position following timeout
        //? This also changes POS_FOLLOW_ERR_TIMEOUT (0x6066, 0) parameter
        uint16_t pos_window_timeout = 5000; //ms
        if (ecrt_master_sdo_download(
                master,
                i,
                POS_WINDOW_TIMEOUT,
                (uint8_t *)&pos_window_timeout,
                sizeof(pos_window_timeout),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change position window timeout for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                POS_WINDOW_TIMEOUT,
                (uint8_t *)&pos_window_timeout,
                sizeof(pos_window_timeout),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change position window timeout for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed position window timeout %ums for j%d", pos_window_timeout, i);



        //* Set parameters for CSP mode
        // Set mode to CSP (0x8) mode
        uint8_t mode = 0x08;
        if (ecrt_master_sdo_download(
                master,
                i,
                MODE_OF_OPERATION,
                &mode,
                sizeof(mode),
                &abort_code))
        {
            RCLCPP_INFO(this->get_logger(), "Failed to change mode of operation for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                MODE_OF_OPERATION,
                &mode,
                sizeof(mode),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change mode of operation for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed mode of operation 0x%x for j%d", mode, i);


        // Read current position
        int32_t current_pos;
        if (ecrt_master_sdo_upload(
                master,
                i,
                POS_ACTUAL_INDEX,
                (uint8_t *)&current_pos,
                sizeof(current_pos),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read current position (0x6064) for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Current position: %d (counts) for j%d", current_pos, i);


        // Set joint commands to current positions
        joint_commands_[i] = current_pos;


        // Velocity following error window
        uint32_t vel_follow_err_window = 150000;
        if (ecrt_master_sdo_download(
                master,
                i,
                VELOCITY_FOLLOW_ERR_WINDOW,
                (uint8_t *)&vel_follow_err_window,
                sizeof(vel_follow_err_window),
                &abort_code))
        {
            RCLCPP_INFO(this->get_logger(), "Failed to velocity following error window for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                VELOCITY_FOLLOW_ERR_WINDOW,
                (uint8_t *)&vel_follow_err_window,
                sizeof(vel_follow_err_window),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to velocity following error window for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed velocity following error window %u for j%d\n", vel_follow_err_window, i);
    



        // Setup DC-Synchronization 
        ecrt_slave_config_dc(
            joint_slave_configs[i], 
            ASSIGN_ACTIVATE, 
            SYNC0_CYCLE, 
            SYNC0_SHIFT, 
            0, 0);
    
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
    int32_t current_pos = EC_READ_S32(domain_pd + actual_pos_offset[joint_no_]);
    int32_t target_pos = EC_READ_S32(domain_pd + target_pos_offset[joint_no_]);
    

    // Read status + control words
    status_word = EC_READ_U16(domain_pd + status_word_offset[joint_no_]);
    control_word = EC_READ_U16(domain_pd + ctrl_word_offset[joint_no_]);


    //* CiA 402 PDS FSA commissioning
    if ((status_word & 0b01001111) == 0b00000000)
    {
        if (driveState[joint_no_] != NOT_READY)
        {
            driveState[joint_no_] = NOT_READY;
            RCLCPP_INFO(this->get_logger(), " J%d State: Not ready", joint_no_);
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
            RCLCPP_INFO(this->get_logger(), " J%d State: Switched on", joint_no_);

            if (current_pos != target_pos)
            {
                RCLCPP_ERROR(this->get_logger(), "target pos != current pos, fixing...");
                EC_WRITE_S32(domain_pd + target_pos_offset[joint_no_], current_pos);
                joint_commands_[joint_no_] = current_pos;
            }
        }
    }
    else if ((status_word & 0b01101111) == 0b00100111)
    {
        if (driveState[joint_no_] != OPERATION_ENABLED)
        {
            driveState[joint_no_] = OPERATION_ENABLED;
            RCLCPP_INFO(this->get_logger(), " J%d State: Operation enabled!\n", joint_no_);
            
            //* Current joint reached Operation Enabled, enable next joint
        }
        joint_no_++;    
    }
    // else if ((status_word & 0b01101111) == 0b00000111)
    // {
    //     EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no_], (control_word & 0b01111111) | 0b00001111);
    //     driveState[joint_no_] = QUICK_STOP_ACTIVE;
    //     RCLCPP_INFO(this->get_logger(), " J%d State: Quick stop active", joint_no_);
    // }
    // else if ((status_word & 0b01001111) == 0b00001111)
    // {
    //     EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no_], 0x0080);
    //     driveState[joint_no_] = FAULT_REACTION_ACTIVE;
    //     RCLCPP_INFO(this->get_logger(), " J%d State: Fault reaction active", joint_no_);
    // }
    else if ((status_word & 0b01001111) == 0b00001000)
    {
        if (driveState[joint_no_] != FAULT)
        {
            driveState[joint_no_] = FAULT;
            RCLCPP_INFO(this->get_logger(), " J%d State: Fault (0x%x)", joint_no_, status_word);
        }

        EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no_], (control_word & 0b11111111) | 0b10000000);

        // if (!(joint_no_ == 5 && joints_op_enabled_))
        // {
        //     RCLCPP_INFO(this->get_logger(), "Clearing fault for J%d", joint_no_);
        //     EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no_], (control_word & 0b11111111) | 0b10000000);
        // }
        // else
        // {
        //     joint_no_ = 5;
        //     return true;
        // }

    }

    if (joint_no_ == NUM_JOINTS) // All joints reached Operation Enabled
    {
        joint_no_ = 0;
        // joint_no_ = 5;
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

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
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
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    ecrt_master_application_time(master, TIMESPEC2NS(t));
    // ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));
    // wakeupTime = timespec_add(wakeupTime, cycletime);
    // clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

    // loop_start_time_ = (unsigned long) this->now().nanoseconds();
    
    // latency_ns_ = loop_start_time_ - wakeup_time_;
    // period_ns_ = loop_start_time_ - loop_last_start_time;
    // exec_ns_ = loop_end_time_ - loop_last_start_time;
    // loop_last_start_time = loop_start_time_;

    // if (latency_ns_ > latency_max_ns_) {
    //     latency_max_ns_ = (loop_start_time_ > wakeup_time_) ? latency_ns_ : -latency_ns_;
    // }
    // if (latency_ns_ < latency_min_ns_) {
    //     latency_min_ns_ = latency_ns_;
    // }
    // if (period_ns_ > period_max_ns_) {
    //     period_max_ns_ = period_ns_;
    // }
    // if (period_ns_ < period_min_ns_) {
    //     period_min_ns_ = period_ns_;
    // }
    // if (exec_ns_ > exec_max_ns_) {
    //     exec_max_ns_ = exec_ns_;
    // }
    // if (exec_ns_ < exec_min_ns_) {
    //     exec_min_ns_ = exec_ns_;
    // }


    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain);


    // check process data state (optional)
    // check_domain_state_();

    // Read joint states
    // for (uint i = 0; i < NUM_JOINTS; i++)
    //     joint_states_enc_counts_[i] = EC_READ_S32(domain_pd + actual_pos_offset[i]);


    // if (counter_)
    // {
        // counter_--;
    // }
    // else    // Do below every 1s
    // {
        // counter_ = ( 1000 / CYCLIC_DATA_PERIOD.count() );
        // counter_ = FREQUENCY;
// 
        // check for master state (optional)
        // check_master_state();
// 
        // check for slave configuration state(s) (optional)
        // if (!operational_)
        //     operational_ = check_slave_config_states_(joint_no__);
        // 
        // read process data SDO
        // read_sdos(joint_no__);
// 
        //* Measure timing
        // RCLCPP_INFO(this->get_logger(), "period     %luns ... %luns",
        //         period_min_ns_, period_max_ns_);
        // RCLCPP_INFO(this->get_logger(), "exec       %luns ... %luns",
        //         exec_min_ns_, exec_max_ns_);
        // RCLCPP_INFO(this->get_logger(), "latency    %luns ... %luns\n",
        //         latency_min_ns_, latency_max_ns_);
            // 
        // period_max_ns_ = 0;
        // period_min_ns_ = (0xffffffffffffffff);
        // exec_max_ns_ = 0;
        // exec_min_ns_ = (0xffffffffffffffff);
        // latency_max_ns_ = 0;
        // latency_min_ns_ = (0xffffffffffffffff);
    // }


    // If all joints reached EtherCAT OP state
    if (joints_OP_)
    {
        // Transit joints through CiA402 PDS FSA
        joints_op_enabled_ = state_transition_();

        // for (uint i = 0; i < NUM_JOINTS; i++)
            // joint_states_enc_counts_[i] = EC_READ_S32(domain_pd + actual_pos_offset[i]);
        
        joint_states_enc_counts_[joint_no_] = EC_READ_S32(domain_pd + actual_pos_offset[joint_no_]);
    }        
    else
    {
        joints_OP_ = check_slave_config_states_();

        //* If not all joints reached OP state for 10s, retry
        if ((this->now().seconds() - stamp_) >= 10)
        {
            RCLCPP_INFO(this->get_logger(), "Not all joints reached OP, retrying");
            ecrt_master_reset(master);

            joint_no_ = 0;

            // receive process data
            ecrt_master_receive(master);
            ecrt_domain_process(domain);

            stamp_ = this->now().seconds();
        }
    }


    // If all joints reached CiA402 Drive State Operation Enabled
    if (joints_op_enabled_)
    {
        //* Write target positions in TxPDOs from MoveIt joint commands
        for (uint i = 0; i < NUM_JOINTS; i++)
        {
            EC_WRITE_S32(domain_pd + target_pos_offset[i], joint_commands_[i]);
        }   


        //* Uncomment to zero actuator
        // uint joint_index = 4; // Change index to choose which joint to zero
        // int32_t current_pos = EC_READ_S32(domain_pd + actual_pos_offset[joint_index]);
        // int32_t target_pos = current_pos;
        // uint32_t delta = 250;
        // if (abs(current_pos) > delta)
        // {
        //     if (current_pos < 0)
        //         target_pos += delta;
        //     else if (current_pos > 0)
        //         target_pos -= delta;
        //     EC_WRITE_S32(domain_pd + target_pos_offset[joint_index], target_pos);
        // }


        //* Uncomment to jog J6 between [-180, 180]
        // int32_t current_pos = EC_READ_S32(domain_pd + actual_pos_offset[5]);
        // int32_t target_pos = current_pos;
        // if (!toggle)
        // {
        //     if (current_pos < 262144)
        //     {
        //         target_pos += 8000;
        //         EC_WRITE_S32(domain_pd + target_pos_offset[5], target_pos);
        //     }
        //     else
        //         toggle = true;
        // }
        // else
        // {
        //     if (current_pos > -262144)
        //     {
        //         target_pos -= 8000;
        //         EC_WRITE_S32(domain_pd + target_pos_offset[5], target_pos);
        //     }
        //     else
        //         toggle = false;
        // }

    }




    // Sync every cycle
    // Write application time to master
    //
    // It is a good idea to use the target time (not the measured time) as
    // application time, because it is more stable.
    //
    // ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));

    if (sync_ref_counter)
    {
        sync_ref_counter--;
    }
    else
    {
        sync_ref_counter = 1;
        clock_gettime(CLOCK_MONOTONIC, &time_ns);
        ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time_ns));
    }
    // ecrt_master_sync_reference_clock(master);
    ecrt_master_sync_slave_clocks(master);

    // send process data
    ecrt_domain_queue(domain);
    ecrt_master_send(master);

    // loop_end_time_ = (unsigned long) this->now().nanoseconds();
    // wakeup_time_ = loop_end_time_ + PERIOD_NS;

    // clock_gettime(CLOCK_TO_USE, &wakeupTime);
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
        joint_states_.position[i] = COUNT_TO_RAD(joint_states_enc_counts_[i]);
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

rclcpp::CallbackGroup::SharedPtr ZeroErrInterface::get_high_prio_callback_group()
{
    high_prio_cbg_ = get_node_base_interface()->get_default_callback_group();
    return high_prio_cbg_;  // the default callback group.
}

rclcpp::CallbackGroup::SharedPtr ZeroErrInterface::get_normal_prio_callback_group()
{
    return normal_prio_cbg_;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor high_prio_exec_;
    rclcpp::executors::SingleThreadedExecutor normal_prio_exec_;

    auto node = std::make_shared<ZeroErrInterface>();

    high_prio_exec_.add_callback_group(
        node->get_high_prio_callback_group(), node->get_node_base_interface()
    );

    normal_prio_exec_.add_callback_group(
        node->get_normal_prio_callback_group(), node->get_node_base_interface()
    );

    // Create a thread for each of the two executors ...
    auto high_prio_thread = std::thread(
        [&]() {

            //* Set thread priority
            if (!realtime_tools::configure_sched_fifo(sched_get_priority_max(SCHED_FIFO) - 19))
                RCLCPP_WARN(node->get_logger(), "Couldnt enable FIFO RT Scheduling!");
            
            cpu_set_t mask;
            int ret;
            CPU_ZERO(&mask);
            CPU_SET(0, &mask);
            ret = sched_setaffinity(0, sizeof(mask), &mask);

            if (ret != 0){
                RCLCPP_WARN(node->get_logger(), "Couldn't assign to core 0");
            }
            
            high_prio_exec_.spin();
            
        });

    auto low_prio_thread = std::thread(
        [&]() {
            normal_prio_exec_.spin();
        });

    //* Set main thread priority
    // struct sched_param param = {};
    // param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 20;
    // RCLCPP_INFO(node->get_logger(), "Using priority %i.\n", param.sched_priority);
    // if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    //     RCLCPP_ERROR(node->get_logger(), "sched_setscheduler failed\n");
    // }

    // rclcpp::spin(node);

    while (rclcpp::ok());

    rclcpp::shutdown();

    return 0;
}
