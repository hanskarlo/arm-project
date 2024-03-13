#include <ecrt.h>

#define NUM_JOINTS 6

#define EROB_70H100_MAX_SPEED   262144 // counts/s
#define EROB_70H100_MAX_ADCEL   873813 // counts/s^2

#define EROB_110H120_MAX_SPEED  145927 // counts/s
#define EROB_110H120_MAX_ADCEL  486423 // counts/s^2

#define JOINT1_ALIAS_POS  0, 0
#define JOINT2_ALIAS_POS  0, 1
#define JOINT3_ALIAS_POS  0, 2
#define JOINT4_ALIAS_POS  0, 3
#define JOINT5_ALIAS_POS  0, 4
#define JOINT6_ALIAS_POS  0, 5

// Vendor ID, Product ID
#define ZEROERR_EROB    0x5a65726f, 0x00029252

// RxPDO object index, subindex
#define TARGET_POS_INDEX    0x607A, 0
#define DIGI_OUT_INDEX      0x60FE, 0
#define CTRL_WORD_INDEX     0x6040, 0

// TxPDO object index, subindex
#define POS_ACTUAL_INDEX    0x6064, 0
#define DIGI_INPUT_INDEX    0x60FD, 0
#define STATUS_WORD_INDEX   0x6041, 0  

// Other objects in dictionary (index, subindex)
#define SM2_SYNC_TYPE           0x1C32, 1
#define SM3_SYNC_TYPE           0x1C33, 1
#define ERROR_CODE              0x603F, 0
#define MAX_PROFILE_VELOCITY    0x607F, 0
#define MAX_VELOCITY            0x6080, 0
#define MAX_ACCELERATION        0x60C5, 0
#define MAX_DECELERATION        0x60C6, 0
#define PROFILE_VELOCITY        0x6081, 0
#define PROFILE_ACCELERATION    0x6083, 0
#define PROFILE_DECELERATION    0x6084, 0
#define POS_FOLLOW_WINDOW       0x6065, 0
#define MODE_OF_OPERATION       0x6060, 0


//* Ethercat variables
// EtherCAT
ec_master_t *master = NULL;
ec_master_state_t master_state = {};
ec_domain_t *domain = NULL;
ec_domain_state_t domain_state = {};
ec_slave_config_t *erob_config = NULL;
ec_slave_config_state_t erob_state = {};
ec_slave_config_t *joint_slave_configs[NUM_JOINTS];
ec_slave_config_state_t joint_ec_states[NUM_JOINTS];
static ec_sdo_request_t *sdo[NUM_JOINTS];

unsigned long counter;

// CiA 402 PDS FSA States
typedef enum{
    DEFAULT,
    NOT_READY,
    SWITCH_ON_DISABLED,
    READY,
    SWITCHED_ON,
    OPERATION_ENABLED,
    FAULT_REACTION_ACTIVE,
    QUICK_STOP_ACTIVE,
    FAULT
}DriveState;
DriveState driveState[NUM_JOINTS] = {DEFAULT, DEFAULT, DEFAULT, DEFAULT, DEFAULT, DEFAULT};

// process data
static uint8_t *domain_pd;

// RxPDO entry offsets
static unsigned int target_pos_offset[NUM_JOINTS];
static unsigned int ctrl_word_offset[NUM_JOINTS];

// TxPDO entry offsets
static unsigned int actual_pos_offset[NUM_JOINTS];
static unsigned int status_word_offset[NUM_JOINTS];

const static ec_pdo_entry_reg_t domain_regs_[] = {
    {JOINT1_ALIAS_POS, ZEROERR_EROB, POS_ACTUAL_INDEX,  &actual_pos_offset[0], NULL},
    {JOINT1_ALIAS_POS, ZEROERR_EROB, STATUS_WORD_INDEX, &status_word_offset[0], NULL},
    {JOINT1_ALIAS_POS, ZEROERR_EROB, TARGET_POS_INDEX,  &target_pos_offset[0], NULL},
    {JOINT1_ALIAS_POS, ZEROERR_EROB, CTRL_WORD_INDEX,   &ctrl_word_offset[0], NULL},
    {JOINT2_ALIAS_POS, ZEROERR_EROB, POS_ACTUAL_INDEX,  &actual_pos_offset[1], NULL},
    {JOINT2_ALIAS_POS, ZEROERR_EROB, STATUS_WORD_INDEX, &status_word_offset[1], NULL},
    {JOINT2_ALIAS_POS, ZEROERR_EROB, TARGET_POS_INDEX,  &target_pos_offset[1], NULL},
    {JOINT2_ALIAS_POS, ZEROERR_EROB, CTRL_WORD_INDEX,   &ctrl_word_offset[1], NULL},
    {JOINT3_ALIAS_POS, ZEROERR_EROB, POS_ACTUAL_INDEX,  &actual_pos_offset[2], NULL},
    {JOINT3_ALIAS_POS, ZEROERR_EROB, STATUS_WORD_INDEX, &status_word_offset[2], NULL},
    {JOINT3_ALIAS_POS, ZEROERR_EROB, TARGET_POS_INDEX,  &target_pos_offset[2], NULL},
    {JOINT3_ALIAS_POS, ZEROERR_EROB, CTRL_WORD_INDEX,   &ctrl_word_offset[2], NULL},
    {JOINT4_ALIAS_POS, ZEROERR_EROB, POS_ACTUAL_INDEX,  &actual_pos_offset[3], NULL},
    {JOINT4_ALIAS_POS, ZEROERR_EROB, STATUS_WORD_INDEX, &status_word_offset[3], NULL},
    {JOINT4_ALIAS_POS, ZEROERR_EROB, TARGET_POS_INDEX,  &target_pos_offset[3], NULL},
    {JOINT4_ALIAS_POS, ZEROERR_EROB, CTRL_WORD_INDEX,   &ctrl_word_offset[3], NULL},
    {JOINT5_ALIAS_POS, ZEROERR_EROB, POS_ACTUAL_INDEX,  &actual_pos_offset[4], NULL},
    {JOINT5_ALIAS_POS, ZEROERR_EROB, STATUS_WORD_INDEX, &status_word_offset[4], NULL},
    {JOINT5_ALIAS_POS, ZEROERR_EROB, TARGET_POS_INDEX,  &target_pos_offset[4], NULL},
    {JOINT5_ALIAS_POS, ZEROERR_EROB, CTRL_WORD_INDEX,   &ctrl_word_offset[4], NULL},
    {JOINT6_ALIAS_POS, ZEROERR_EROB, POS_ACTUAL_INDEX,  &actual_pos_offset[5], NULL},
    {JOINT6_ALIAS_POS, ZEROERR_EROB, STATUS_WORD_INDEX, &status_word_offset[5], NULL},
    {JOINT6_ALIAS_POS, ZEROERR_EROB, TARGET_POS_INDEX,  &target_pos_offset[5], NULL},
    {JOINT6_ALIAS_POS, ZEROERR_EROB, CTRL_WORD_INDEX,   &ctrl_word_offset[5], NULL},
    {} //! Terminate with empty struct
};


ec_pdo_entry_info_t erob_pdo_entries_[] = {
    {TARGET_POS_INDEX, 32},
    {DIGI_OUT_INDEX, 32},
    {CTRL_WORD_INDEX, 16},
    {POS_ACTUAL_INDEX, 32},
    {DIGI_INPUT_INDEX, 32},
    {STATUS_WORD_INDEX, 16}
};

static ec_pdo_info_t erob_pdos_[] = {
    {0x1600, 3, erob_pdo_entries_},
    {0x1A00, 3, erob_pdo_entries_ + 3}
};

static ec_sync_info_t erob_syncs_[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, erob_pdos_, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, erob_pdos_ + 1, EC_WD_DISABLE},
    {0xff}
};