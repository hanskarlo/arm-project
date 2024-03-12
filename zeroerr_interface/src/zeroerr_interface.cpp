#include "zeroerr_interface/zeroerr_interface.h"

ZeroErrInterface::ZeroErrInterface() : Node("zeroerr_interface")
{
    // Initialize vector variables (avoid segfault)
    joint_states_.name.resize(NUM_JOINTS);
    joint_states_.position.assign(NUM_JOINTS, 0.0);
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

    stamp = this->now().seconds();

    // Create cyclic data exchange timer
    cyclic_pdo_timer_ = this->create_wall_timer(
        1ms,
        std::bind(&ZeroErrInterface::cyclic_pdo_loop_, this));
        
    counter = 0;

    // Create joint state publisher timer
    joint_state_pub_timer_ = this->create_wall_timer(
        10ms,
        std::bind(&ZeroErrInterface::joint_state_pub_, this));
}


ZeroErrInterface::~ZeroErrInterface()
{
    RCLCPP_INFO(this->get_logger(), "Releasing master..\n");
    ecrt_release_master(master);
}


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


bool ZeroErrInterface::set_drive_parameters()
{
    uint32_t abort_code;
    size_t result_size;
    uint32_t data_32;
    uint16_t data_16;
    uint8_t data_8;

    for (uint i = 0; i < NUM_JOINTS; i++)
    {
        // Set max velocity
        data_32 = (i < 3) ? EROB_110H120_MAX_SPEED : EROB_70H100_MAX_SPEED;
        if (ecrt_master_sdo_download(
                master,
                i,
                MAX_VELOCITY,
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
                MAX_VELOCITY,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to upload Max profile velocity for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed max profile velocity: %u counts/s for j%i", data_32, i);


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


        // Max acceleration
        data_32 = (i < 3) ? EROB_110H120_MAX_ADCEL : EROB_70H100_MAX_ADCEL;
        if (ecrt_master_sdo_download(
                master,
                i,
                MAX_ACCELERATION,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change Max acceleration for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                MAX_ACCELERATION,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change Max acceleration for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed max acceleration: %u for j%d", data_32, i);


        // Max deceleration
        data_32 = (i < 3) ? EROB_110H120_MAX_ADCEL : EROB_70H100_MAX_ADCEL;
        if (ecrt_master_sdo_download(
                master,
                i,
                MAX_DECELERATION,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to download Max deceleration for j%d", i);
            return false;
        }

        if (ecrt_master_sdo_upload(
                master,
                i,
                MAX_DECELERATION,
                (uint8_t *)&data_32,
                sizeof(data_32),
                &result_size,
                &abort_code))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to upload Max deceleration for j%d", i);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Changed max deceleration: %u for j%d", data_32, i);


        // Profile velocity
        data_32 = (i < 3) ? (EROB_110H120_MAX_SPEED / 2) : (EROB_70H100_MAX_SPEED / 2);
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
        data_32 = (i < 3) ? (EROB_110H120_MAX_ADCEL / 2) : (EROB_70H100_MAX_ADCEL / 2);
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
        data_32 = (i < 3) ? (EROB_110H120_MAX_ADCEL / 2) : (EROB_70H100_MAX_ADCEL / 2);
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
        data_32 = 100000;
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
    }


    return true;
}


bool ZeroErrInterface::init_()
{
    bool ret = false;

    RCLCPP_INFO(this->get_logger(), "Starting...\n");


    master = ecrt_request_master(0);
    if (!master)
    {
        RCLCPP_ERROR(this->get_logger(), "Requesting master 0 failed.\n");
        return false;
    }


    ret = configure_pdos_();
    if (!ret)
        return ret;

    RCLCPP_INFO(this->get_logger(), "Creating SDO requests...\n");
    for (uint i = 0; i < NUM_JOINTS; i++)
    {
        if (!(sdo[i] = ecrt_slave_config_create_sdo_request(joint_slave_configs[i], 0x603F, 0, sizeof(uint16_t)))) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create SDO request.\n");
            return false;
        }
        ecrt_sdo_request_timeout(sdo[i], 500); // ms
    }

    ret = set_drive_parameters();
    if (!ret)
        return ret;


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


void ZeroErrInterface::state_transition_(int joint_no)
{
    uint16_t status_word;
    uint16_t control_word;
    
    // Read status + control words
    status_word = EC_READ_U16(domain_pd + status_word_offset[joint_no]);
    control_word = EC_READ_U16(domain_pd + ctrl_word_offset[joint_no]);


    //* CiA 402 PDS FSA commissioning
    if ((status_word & 0b01001111) == 0b00000000)
    {
        if (driveState[joint_no] != NOT_READY)
        {
            driveState[joint_no] = NOT_READY;
            RCLCPP_INFO(this->get_logger(), " J%d State: Not ready", joint_no);
        }
    }
    else if ((status_word & 0b01001111) == 0b01000000)
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no], (control_word & 0b01111110) | 0b00000110);
        driveState[joint_no] = SWITCH_ON_DISABLED;
        RCLCPP_INFO(this->get_logger(), " J%d State: Switch on disabled", joint_no);
    }
    else if ((status_word & 0b01101111) == 0b00100001)
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no], (control_word & 0b01110111) | 0b00000111);
        driveState[joint_no] = READY;
        RCLCPP_INFO(this->get_logger(), " J%d State: Ready to switch on", joint_no);
    }
    else if ((status_word & 0b01101111) == 0b00100011)
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no], (control_word & 0b01111111) | 0b00001111);
        driveState[joint_no] = SWITCHED_ON;
        RCLCPP_INFO(this->get_logger(), " J%d State: Switched on", joint_no);
    }
    else if ((status_word & 0b01101111) == 0b00100111)
    {
        if (driveState[joint_no] != OPERATION_ENABLED)
        {
            driveState[joint_no] = OPERATION_ENABLED;
            RCLCPP_INFO(this->get_logger(), " J%d State: Operation enabled!", joint_no);
            joint_no_++;
        }
    }
    else if ((status_word & 0b01101111) == 0b00000111)
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no], (control_word & 0b01111111) | 0b00001111);
        driveState[joint_no] = QUICK_STOP_ACTIVE;
        RCLCPP_INFO(this->get_logger(), " J%d State: Quick stop active", joint_no);
    }
    else if ((status_word & 0b01001111) == 0b00001111)
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no], 0x0080);
        driveState[joint_no] = FAULT_REACTION_ACTIVE;
        RCLCPP_INFO(this->get_logger(), " J%d State: Fault reaction active", joint_no);
    }
    else if ((status_word & 0b01001111) == 0b00001000)
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset[joint_no], (control_word & 0b11111111) | 0b10000000);

        if (driveState[joint_no] != FAULT)
        {
            driveState[joint_no] = FAULT;
            RCLCPP_INFO(this->get_logger(), " J%d State: Fault", joint_no);
        }
    }


}

void ZeroErrInterface::read_sdos(int joint_no)
{
    switch (ecrt_sdo_request_state(sdo[joint_no])) {
        case EC_REQUEST_UNUSED: // request was not used yet
            ecrt_sdo_request_read(sdo[joint_no]); // trigger first read
            break;
        case EC_REQUEST_BUSY:
            RCLCPP_INFO(this->get_logger(), "Still busy...\n");
            break;
        case EC_REQUEST_SUCCESS:
            RCLCPP_INFO(this->get_logger(), "Error (0x603F): 0x%04X\n",
                    EC_READ_U16(ecrt_sdo_request_data(sdo[joint_no])));
            ecrt_sdo_request_read(sdo[joint_no]); // trigger next read
            break;
        case EC_REQUEST_ERROR:
            RCLCPP_INFO(this->get_logger(), "Failed to read SDO!\n");
            ecrt_sdo_request_read(sdo[joint_no]); // retry reading
            break;
    }
}

void ZeroErrInterface::cyclic_pdo_loop_()
{
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    // check process data state (optional)
    check_domain_state();

    // Read joint states
    for (uint i = 0; i < NUM_JOINTS; i++)
        joint_states_.position[i] = EC_READ_U32(domain_pd + actual_pos_offset[i]);


    if (counter)
    {
        counter--;
    }
    else
    { // do this at 1 Hz
        counter = 1000;

        // check for master state (optional)
        // check_master_state();

        // check for islave configuration state(s) (optional)
        if (!operational)
            operational = check_slave_config_states(joint_no_);

        // status_word[i] = EC_READ_U16(domain_pd + status_word[i]_offset);
        // RCLCPP_INFO(this->get_logger(), "Status Word: 0x%x", status_word[i]);

        // RCLCPP_INFO(this->get_logger(), "Hi");

        // read process data SDO
        // read_sdos(joint_no_);
    }


    if (operational)
    {
        // CiA402 PDS FSA
        state_transition_(joint_no_);

        //* Check all motors are in Operation Enabled state (PDS FSA)
        // uint8_t state_counter = 0;
        // for (uint i = 0; i < NUM_JOINTS; i++)
        // {
        //     if (driveState[i] == OPERATION_ENABLED)
        //         state_counter++;
        // }

        // if (state_counter == NUM_JOINTS)
        // {
        //     for (uint i = 0; i < NUM_JOINTS; i++)
        //     {
        //         EC_WRITE_U32(domain_pd + target_pos_offset[i], joint_commands_[i]);
        //     }
        // }
    }        
    else
    {
        if ((this->now().seconds() - stamp) >= 5)
        {
            RCLCPP_INFO(this->get_logger(), "Resetting master");
            ecrt_master_reset(master);
            stamp = this->now().seconds();
        }
    }

    // send process data
    ecrt_domain_queue(domain);
    ecrt_master_send(master);
}

void ZeroErrInterface::joint_state_pub_()
{
    joint_states_.header.stamp = this->now();

    for (uint i = 0; i < NUM_JOINTS; i++)
    {
        std::string joint_name = "j";
        joint_name.append(std::to_string(i + 1));

        joint_states_.name[i] = joint_name;
    }

    arm_state_pub_->publish(joint_states_);
}

void ZeroErrInterface::check_master_state()
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

void ZeroErrInterface::check_domain_state()
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain, &ds);

    if (ds.working_counter != domain_state.working_counter)
        RCLCPP_INFO(this->get_logger(), "Domain: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain_state.wc_state)
        RCLCPP_INFO(this->get_logger(), "Domain: State %u.\n", ds.wc_state);

    domain_state = ds;
}

bool ZeroErrInterface::check_slave_config_states(int joint_no)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(joint_slave_configs[joint_no], &s);
    
    if (s.al_state != joint_ec_states[joint_no].al_state)
        RCLCPP_INFO(this->get_logger(), "J%d: State 0x%02X.\n", joint_no, s.al_state);
    if (s.online != joint_ec_states[joint_no].online)
        RCLCPP_INFO(this->get_logger(), "J%d: %s.\n", joint_no, s.online ? "online" : "offline");
    if (s.operational != joint_ec_states[joint_no].operational)
        RCLCPP_INFO(this->get_logger(), "J%d: %soperational.\n", joint_no, 
            s.operational ? "" : "Not ");    

    joint_ec_states[joint_no] = s;


    if (joint_ec_states[joint_no].operational)
        joint_no_++; 

    if (joint_no_ == 6) // All joints operational
    {
        joint_no_ = 0;
        return true;
    }
    else
        return false;
}

void ZeroErrInterface::arm_cmd_cb_(sensor_msgs::msg::JointState::UniquePtr arm_cmd)
{
    for (uint i = 0; i < arm_cmd->position.size(); i++)
    {
        // Radians -> encoder counts
        joint_commands_[i] = (arm_cmd->position[i] * 524288) / (2 * 3.14159265);
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