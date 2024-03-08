#include "zeroerr_interface/zeroerr_interface.h"
#include "zeroerr_interface/ec_defines.h"

ZeroErrInterface::ZeroErrInterface() : Node("zeroerr_interface")
{
    counter = 0;

    init_();

    // Create cyclic data exchange timer
    cyclic_pdo_timer_ = this->create_wall_timer(
        10ms, 
        std::bind(&ZeroErrInterface::cyclic_pdo_loop_, this)
    );

}

ZeroErrInterface::~ZeroErrInterface(){ ecrt_release_master(master); }


void ZeroErrInterface::init_()
{
           
    
}

void ZeroErrInterface::cyclic_pdo_loop_()
{
    uint16_t status_word, control_word;

    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    // check process data state (optional)
    check_domain_state();

    if (counter) {
        counter--;
    } else { // do this at 1 Hz
        counter = 100;

        // check for master state (optional)
        check_master_state();

        // check for islave configuration state(s) (optional)
        check_slave_config_states();

        status_word = EC_READ_U16(domain_pd + status_word_offset);
        // RCLCPP_INFO(this->get_logger(), "Status Word: %x", status_word);

        // read process data SDO
        // read_sdo();

    }

    // Update/read status + control words
    status_word = EC_READ_U16(domain_pd + status_word_offset);
    control_word = EC_READ_U16(domain_pd + ctrl_word_offset);

    // CiA 402 PDS FSA commissioning
    if ((status_word & 0b01001111) == 0b00000000) 
    {
        if (driveState != NOT_READY)
        {
            driveState = NOT_READY;
            RCLCPP_INFO(this->get_logger(), "State: Not ready");
        }
    } 
    else if ((status_word & 0b01001111) == 0b01000000) 
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset, (control_word & 0b01111110) | 0b00000110);
        driveState = SWITCH_ON_DISABLED;
        RCLCPP_INFO(this->get_logger(), "State: Switch on disabled");
    } 
    else if ((status_word & 0b01101111) == 0b00100001) 
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset, (control_word & 0b01110111) | 0b00000111);
        driveState = READY;
        RCLCPP_INFO(this->get_logger(), "State: Ready to switch on");
    } 
    else if ((status_word & 0b01101111) == 0b00100011) 
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset, (control_word & 0b01111111) | 0b00001111);
        driveState = SWITCHED_ON;
        RCLCPP_INFO(this->get_logger(), "State: Switched on");
    } 
    else if ((status_word & 0b01101111) == 0b00100111) 
    {
        if (driveState != OPERATION_ENABLED)
        {
            driveState = OPERATION_ENABLED;
            RCLCPP_INFO(this->get_logger(), "State: Operation enabled!");
        }
    } 
    else if ((status_word & 0b01101111) == 0b00000111) 
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset, (control_word & 0b01111111) | 0b00001111);
        driveState = QUICK_STOP_ACTIVE;
        RCLCPP_INFO(this->get_logger(), "State: Quick stop active");
    } 
    else if ((status_word & 0b01001111) == 0b00001111) 
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset, 0x0080);
        driveState = FAULT_REACTION_ACTIVE; 
        RCLCPP_INFO(this->get_logger(), "State: Fault reaction active");
    } 
    else if ((status_word & 0b01001111) == 0b00001000) 
    {
        EC_WRITE_U16(domain_pd + ctrl_word_offset, (control_word & 0b11111111) | 0b10000000);
        
        if (driveState != FAULT)
        {
            driveState = FAULT;
            RCLCPP_INFO(this->get_logger(), "State: Fault");
        }
    }


    // send process data
    ecrt_domain_queue(domain);
    ecrt_master_send(master);
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

void ZeroErrInterface::check_slave_config_states()
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(erob_config, &s);

    if (s.al_state != erob_state.al_state)
        RCLCPP_INFO(this->get_logger(), "Joint0: State 0x%02X.\n", s.al_state);
    if (s.online != erob_state.online)
        RCLCPP_INFO(this->get_logger(), "Joint0: %s.\n", s.online ? "online" : "offline");
    if (s.operational != erob_state.operational)
        RCLCPP_INFO(this->get_logger(), "Joint0: %soperational.\n",
                s.operational ? "" : "Not ");

    erob_state = s;
}





int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ZeroErrInterface>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
