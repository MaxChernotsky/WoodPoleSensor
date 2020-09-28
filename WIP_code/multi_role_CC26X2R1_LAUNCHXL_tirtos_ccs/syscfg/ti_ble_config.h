/*
 *  ======== ti_ble_config.h ========
 *  Configured BLE module definitions
 *
 *  DO NOT EDIT - This file is generated by the SysConfig tool.
 */

#include <bcomdef.h>
#include <gapgattserver.h>
#include <simple_gatt_profile.h>
#include <gap_advertiser.h>
#include <gapbondmgr.h>
#include <ti_radio_config.h>

/*********************************************************************
 * RF Settings
 */

#define RF_FE_MODE_AND_BIAS            RF_FE_DIFFERENTIAL | RF_FE_INT_BIAS

// Default Tx Power Index
#define DEFAULT_TX_POWER               HCI_EXT_TX_POWER_0_DBM

/*********************************************************************
 * General Role Configuration
 */

extern uint8_t attDeviceName[GAP_DEVICE_NAME_LEN];

//Random Address
extern uint8_t * pRandomAddress;

// Address mode of the local device
// Note: When using the DEFAULT_ADDRESS_MODE as ADDRMODE_RANDOM or 
// ADDRMODE_RP_WITH_RANDOM_ID, GAP_DeviceInit() should be called with 
// it's last parameter set to a static random address
#define DEFAULT_ADDRESS_MODE                  ADDRMODE_RP_WITH_PUBLIC_ID

// How often to read current RPA (in ms)
#define READ_RPA_PERIOD                       3000

// Maximum number of BLE connections. It should be set based on the
// device GAP role. Here're some recommended values:
//      * Central:     8
//      * Peripheral:  8
//      * Observer:    0
//      * Broadcaster: 0
// Note: When the GAP role includes Peripheral and no v4.1 Controller features
//       are configured, MAX_NUM_BLE_CONNS must not be greater than 1
#define MAX_NUM_BLE_CONNS                      8

// Maximum number of BLE HCI PDUs. If the maximum number connections (above)
// is set to 0 then this number should also be set to 0.
#define MAX_NUM_PDU                   		    5

// Maximum size in bytes of the BLE HCI PDU. Valid range: 27 to 255
// The maximum ATT_MTU is MAX_PDU_SIZE - 4.
#define MAX_PDU_SIZE                  		    69

/*********************************************************************
 * Bond Manager Configuration
 */

#define GAP_BONDINGS_MAX                      10
#define GAP_CHAR_CFG_MAX                      4

extern uint8_t pairMode;
extern uint8_t mitm;
extern uint8_t ioCap;
extern uint8_t bonding;
extern uint8_t secureConnection;
extern uint8_t autoSyncWL;
extern uint8_t eccReGenPolicy;
extern uint8_t KeySize;
extern uint8_t removeLRUBond;
extern uint8_t bondFailAction;
extern uint8_t KeyDistList;
extern uint8_t eccDebugKeys;

extern void setBondManagerParameters();

/*********************************************************************
 * Central Role Configuration
 */

//Init PHY Parameters

// Default PHY for initiating
#define DEFAULT_INIT_PHY                      INIT_PHY_1M

// Default minimum connection interval (in 1.25ms)
#define INIT_PHYPARAM_MIN_CONN_INT    		  80

// Default maximum connection interval (in 1.25ms)
#define INIT_PHYPARAM_MAX_CONN_INT    		  80

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   3000

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

/*********************************************************************
 * Observer Role Configuration
 */

//Primary PHY Parameters

// Default PHY for scanning
#define DEFAULT_SCAN_PHY                        SCAN_PRIM_PHY_CODED

//Default Scan type (Active/Passive)
#define DEFAULT_SCAN_TYPE                       SCAN_TYPE_ACTIVE

// Default scan interval (in 625 us ticks)
#define DEFAULT_SCAN_INTERVAL                   400

// Default scan window   (in 625 us ticks)
#define DEFAULT_SCAN_WINDOW                     400

// Default scan duration in 10 ms
#define DEFAULT_SCAN_DURATION                   200

// Advertising report fields to keep in the list
#define ADV_RPT_FIELDS                          (SCAN_ADVRPT_FLD_ADDRESS | SCAN_ADVRPT_FLD_ADDRTYPE)

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID            true

/*********************************************************************
 * Peripheral Role Configuration
 */

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION       GAP_UPDATE_REQ_PASS_TO_APP

// Delay (in ms) after connection establishment before sending a parameter update requst
#define SEND_PARAM_UPDATE_DELAY                 6000

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update
// request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT           600

/*********************************************************************
 * Broadcaster Role Configuration
 */

// Advertisement Set Number 1
extern GapAdv_params_t advParams1;
extern uint8_t advData1[18];
extern uint8_t advData2[20];
extern uint8_t advData3[17];
extern uint8_t advData4[15];

extern uint8_t scanResData1[21];
