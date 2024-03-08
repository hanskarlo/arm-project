#include <ecrt.h>
    
#define JOINT1_ALIAS_POS  0, 0

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

// EtherCAT
ec_master_t *master = NULL;
ec_master_state_t master_state = {};
ec_domain_t *domain = NULL;
ec_domain_state_t domain_state = {};
ec_slave_config_t *erob_config = NULL;
ec_slave_config_state_t erob_state = {};


//* Ethercat variables
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
DriveState driveState = DEFAULT;

// process data
static uint8_t *domain_pd;

// RxPDO entry offsets
static unsigned int target_pos_offset;
static unsigned int d_out_offset;
static unsigned int ctrl_word_offset;

// TxPDO entry offsets
static unsigned int actual_pos_offset;
static unsigned int d_input_offset;
static unsigned int status_word_offset;


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