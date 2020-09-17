/******************************************************************************

@file  multi_role.c

@brief This file contains the multi_role sample application for use
with the CC2650 Bluetooth Low Energy Protocol Stack.

Group: WCS, BTS
Target Device: cc13x2_26x2

******************************************************************************

 Copyright (c) 2013-2020, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

******************************************************************************


*****************************************************************************/

/*********************************************************************
* INCLUDES
*/
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>

#if !(defined __TI_COMPILER_VERSION__)
#include <intrinsics.h>
#endif

#include <ti/drivers/utils/List.h>
#include <ti/common/cc26xx/uartlog/UartLog.h>  // Comment out if using xdc Log

#include <ti/display/AnsiColor.h>


#include <icall.h>
#include "util.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>


#include <devinfoservice.h>
#include <simple_gatt_profile.h>

#include <ti_drivers_config.h>
#include <board_key.h>


#include "ti_ble_config.h"
#include "multi_role_menu.h"
#include "multi_role.h"

#include <ti/sysbios/family/arm/cc26xx/Seconds.h>

/*********************************************************************
 * MACROS
 */

#define UTIL_ARRTOHEX_REVERSE     1
#define UTIL_ARRTOHEX_NO_REVERSE  0
/*********************************************************************
* CONSTANTS
*/



// Application events
#define MR_EVT_CHAR_CHANGE         1
#define MR_EVT_KEY_CHANGE          2
#define MR_EVT_ADV_REPORT          3
#define MR_EVT_SCAN_ENABLED        4
#define MR_EVT_SCAN_DISABLED       5
#define MR_EVT_SVC_DISC            6
#define MR_EVT_ADV                 7
#define MR_EVT_PAIRING_STATE       8
#define MR_EVT_PASSCODE_NEEDED     9
#define MR_EVT_SEND_PARAM_UPDATE   10
#define MR_EVT_PERIODIC            11
#define MR_EVT_READ_RPA            12
#define MR_EVT_INSUFFICIENT_MEM    13
#define MR_EVT_TIMESYNC            14
#define MR_EVT_SECONDSSET          15
#define MR_EVT_PERIODICDATA        16

// Internal Events for RTOS application
#define MR_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define MR_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define MR_ALL_EVENTS                        (MR_ICALL_EVT           | \
                                              MR_QUEUE_EVT)

// Supervision timeout conversion rate to miliseconds
#define CONN_TIMEOUT_MS_CONVERSION            10

// Task configuration
#define MR_TASK_PRIORITY                     1
#ifndef MR_TASK_STACK_SIZE
#define MR_TASK_STACK_SIZE                   1024
#endif

// Discovery states
typedef enum {
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
} discState_t;


// address string length is an ascii character for each digit +
#define MR_ADDR_STR_SIZE     15

// How often to perform periodic event (in msec)
#define MR_PERIODIC_EVT_PERIOD               5000

#define CONNINDEX_INVALID  0xFF

// Spin if the expression is not true
#define MULTIROLE_ASSERT(expr) if (!(expr)) multi_role_spin();




char ownDevAlpha = 'A';
char ownDevNum = '2';

char targetDevAlpha = 'A';
char targetDevNum = '3';



int foundCorrectDev = 0;

char manuToPrint[100];

//advertisement timeSync variable
uint32_t timePreAdv = 0; //preadv values
uint32_t ntimePreAdv = 0;
uint32_t timePostAdv = 0; //postadv values
uint32_t ntimePostAdv = 0;
uint32_t timeDiff = 0; //timediff values
uint32_t ntimeDiff = 0;

//scanning timeSync variable
uint32_t timePreScan = 0; //prescan values
uint32_t ntimePreScan = 0;
uint32_t timePostScan = 0; //postscan values
uint32_t ntimePostScan = 0;
uint32_t timeDiffScan = 0; //timediff values
uint32_t ntimeDiffScan = 0;



//TX vals
uint32_t ticksPreAdv = 0; // divide by 10 to get milliseconds
uint32_t ticksPostAdv = 0;


//RX vals
uint32_t ticksPreScan = 0;
uint32_t ticksPostScan = 0;


uint32_t originalClockValue = 5000;
uint32_t combinedTickDelay = 0;


//boolean to specify if advertising the time
bool timeServer = false;

//boolean to specify if advertising the ticks
bool tickServer = false;

//struct to hold the current time
// ts.secs = seconds elapsed since 1970
// ts.nsecs = nano seconds within the second based on RTC
Seconds_Time ts;

//global variable to count the number of iterations in detemrining the accruate time difference
int count = 0;

//variable to be used to isolate the parameters needed to set the time accurately
bool timeClient = false;

/*********************************************************************
* TYPEDEFS
*/

// App event passed from profiles.
typedef struct
{
  uint8_t event;    // event type
  void *pData;   // event data pointer
} mrEvt_t;

// Container to store paring state info when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} mrPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} mrPasscodeData_t;

// Scanned device information record
typedef struct
{
  uint8_t addrType;         // Peer Device's Address Type
  uint8_t addr[B_ADDR_LEN]; // Peer Device Address
  uint8_t txPower;
  uint8_t rssi;
  uint16_t dataLen;
  char* manuData[30];
} scanRec_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;
  uint8_t data[];
} mrClockEventData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} mrGapAdvEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
  List_Elem elem;
  uint16_t  connHandle;
} mrConnHandleEntry_t;

// Connected device information
typedef struct
{
  uint16_t         connHandle;           // Connection Handle
  uint8_t          addr[B_ADDR_LEN];     // Peer Device Address
  uint8_t          charHandle;           // Characteristic Handle
  Clock_Struct*    pUpdateClock;         // pointer to clock struct
  uint8_t          discState;            // Per connection deiscovery state
} mrConnRec_t;

/*********************************************************************
* GLOBAL VARIABLES
*/

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
* LOCAL VARIABLES
*/

#define APP_EVT_EVENT_MAX  0x13
char *appEventStrings[] = {
  "APP_ZERO             ",
  "APP_CHAR_CHANGE      ",
  "APP_KEY_CHANGE       ",
  "APP_ADV_REPORT       ",
  "APP_SCAN_ENABLED     ",
  "APP_SCAN_DISABLED    ",
  "APP_SVC_DISC         ",
  "APP_ADV              ",
  "APP_PAIRING_STATE    ",
  "APP_SEND_PARAM_UPDATE",
  "APP_PERIODIC         ",
  "APP_READ_RPA         ",
  "APP_INSUFFICIENT_MEM ",
};

/*********************************************************************
* LOCAL VARIABLES
*/

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct clkPeriodic;
// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

//clock instance for timesync events
static Clock_Struct clkTimeSync;

static Clock_Struct clkSecondsSet;

static Clock_Struct clkPeriodicData;

// Memory to pass periodic event to clock handler
mrClockEventData_t periodicUpdateData =
{
  .event = MR_EVT_PERIODIC
};

// Memory to pass RPA read event ID to clock handler
mrClockEventData_t argRpaRead =
{
  .event = MR_EVT_READ_RPA
};

mrClockEventData_t timeSyncClk =
{
 .event = MR_EVT_TIMESYNC
};

mrClockEventData_t secondssetClk =
{
 .event = MR_EVT_SECONDSSET
};

//clock structure for the periodic interval Data set
mrClockEventData_t periodicDataClk =
{
 .event = MR_EVT_PERIODICDATA
};







// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct mrTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(mrTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t mrTaskStack[MR_TASK_STACK_SIZE];

// Maximim PDU size (default = 27 octets)
static uint16 mrMaxPduSize;

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
// Number of scan results filtered by Service UUID
static uint8_t numScanRes = 0;

// Scan results filtered by Service UUID
static scanRec_t scanList[DEFAULT_MAX_SCAN_RES];
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Value to write
static uint8_t charVal = 0;

// Number of connected devices
static uint8_t numConn = 0;

// Connection handle of current connection
static uint16_t mrConnHandle = LINKDB_CONNHANDLE_INVALID;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Per-handle connection info
static mrConnRec_t connList[MAX_NUM_BLE_CONNS];

// Advertising handles
static uint8 advHandle;
static uint8 advHandleTime;
static uint8 advHandleTicks;

static bool mrIsAdvertising = false;
// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};

// Initiating PHY
static uint8_t mrInitPhy = INIT_PHY_1M;

/*********************************************************************
* LOCAL FUNCTIONS
*/
static void multi_role_init(void);
static void multi_role_scanInit(void);
static void multi_role_advertInit(void);
static void multi_role_taskFxn(UArg a0, UArg a1);

static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg);
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg);
static void multi_role_processAppMsg(mrEvt_t *pMsg);
static void multi_role_processCharValueChangeEvt(uint8_t paramID);
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void multi_role_processPasscode(mrPasscodeData_t *pData);
static void multi_role_processPairState(mrPairStateData_t* pairingEvent);
static void multi_role_processGapMsg(gapEventHdr_t *pMsg);
static void multi_role_processParamUpdate(uint16_t connHandle);
static void multi_role_processAdvEvent(mrGapAdvEventData_t *pEventData);

static void multi_role_charValueChangeCB(uint8_t paramID);
static status_t multi_role_enqueueMsg(uint8_t event, void *pData);
static void multi_role_handleKeys(uint8_t keys);
static uint16_t multi_role_getConnIndex(uint16_t connHandle);
static void multi_role_keyChangeHandler(uint8_t keys);
static uint8_t multi_role_addConnInfo(uint16_t connHandle, uint8_t *pAddr,
                                      uint8_t role);
static void multi_role_performPeriodicTask(void);
static void multi_role_clockHandler(UArg arg);
static uint8_t multi_role_clearConnListEntry(uint16_t connHandle);
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
static void multi_role_addScanInfo(uint8_t *pAddr, uint8_t addrType, uint8_t txPower, uint8_t rssi, uint16_t dataLen, char *receivedData);
static bool multi_role_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                      uint16_t dataLen);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID
static uint8_t multi_role_removeConnInfo(uint16_t connHandle);
static void multi_role_startSvcDiscovery(void);
#ifndef Display_DISABLE_ALL
static char* multi_role_getConnAddrStr(uint16_t connHandle);
#endif
static void multi_role_advCB(uint32_t event, void *pBuf, uintptr_t arg);
static void multi_role_scanCB(uint32_t evt, void* msg, uintptr_t arg);
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs, uint32_t numComparison);
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status);
static void multi_role_updateRPA(void);

static char * util_arrtohex(uint8_t const *src, uint8_t src_len, uint8_t *dst, uint8_t dst_len, uint8_t reverse);

static void multi_role_timeSend(void);
static void multi_role_timeIsolation(void);

static void multi_role_tickSend(void);
static void multi_role_tickIsolation(void);

/*********************************************************************
 * EXTERN FUNCTIONS
*/
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
* PROFILE CALLBACKS
*/

// Simple GATT Profile Callbacks
static simpleProfileCBs_t multi_role_simpleProfileCBs =
{
  multi_role_charValueChangeCB // Characteristic value change callback
};

// GAP Bond Manager Callbacks
static gapBondCBs_t multi_role_BondMgrCBs =
{
  multi_role_passcodeCB, // Passcode callback
  multi_role_pairStateCB                  // Pairing state callback
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
 * @fn      multi_role_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void multi_role_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
* @fn      multi_role_createTask
*
* @brief   Task creation function for multi_role.
*
* @param   None.
*
* @return  None.
*/
void multi_role_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = mrTaskStack;
  taskParams.stackSize = MR_TASK_STACK_SIZE;
  taskParams.priority = MR_TASK_PRIORITY;

  Task_construct(&mrTask, multi_role_taskFxn, &taskParams, NULL);
}

/*********************************************************************
* @fn      multi_role_init
*
* @brief   Called during initialization and contains application
*          specific initialization (ie. hardware initialization/setup,
*          table initialization, power up notification, etc), and
*          profile initialization/setup.
*
* @param   None.
*
* @return  None.
*/
static void multi_role_init(void)
{
  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- init ", MR_TASK_PRIORITY);
  // Create the menu
  //multi_role_build_menu();
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Open Display.
  //dispHandle = Display_open(Display_Type_ANY, NULL);
  Log_info0("Board Booting up...");


  //output current device data
  //Log_info1("Channel: %d", ownChannel);
  Log_info0("Current Device ID shown in ASCII:");
  Log_info2("Current Device ID (alpha, num): %d, %d", (int)ownDevAlpha, (int)ownDevNum);



  //initialise Clocks

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clock for internal periodic events.
  Util_constructClock(&clkPeriodic, multi_role_clockHandler,
                      MR_PERIODIC_EVT_PERIOD, 0, false,
                      (UArg)&periodicUpdateData);

  //create one-shot clock for timesync based on call from advert data
  Util_constructClock(&clkTimeSync, multi_role_clockHandler, 1500, 0, false, (UArg)&timeSyncClk);
  Util_constructClock(&clkSecondsSet, multi_role_clockHandler, 1500, 0, false, (UArg)&secondssetClk);

  //create periodic clock for periodic data sync but do not start it now
  Util_constructClock(&clkPeriodicData, multi_role_clockHandler, 20000, 0, false, (UArg)&periodicDataClk);


  // Init key debouncer
  Board_initKeys(multi_role_keyChangeHandler);

  // Initialize Connection List
  multi_role_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (void *)attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

    // Pass all parameter update requests to the app for it to decide
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
  }

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  {
    // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    // Some brand smartphone is essentially needing 251/2120, so we set them here.
  #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
  #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    // This API is documented in hci.h
    // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client, used by GAPBondMgr to look for RPAO characteristic for network privacy
  GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Setup the GAP Bond Manager
  setBondManagerParameters();

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

  // Setup the SimpleProfile Characteristic Values
  // For more information, see the GATT and GATTServApp sections in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  {
    uint8_t charValue1 = 1;
    uint8_t charValue2 = 2;
    uint8_t charValue3 = 3;
    uint8_t charValue4 = 4;
    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                               &charValue1);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
                               &charValue2);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
                               &charValue3);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &charValue4);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                               charValue5);
  }

  // Register callback with SimpleGATTprofile
  SimpleProfile_RegisterAppCBs(&multi_role_simpleProfileCBs);

  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&multi_role_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- call GAP_DeviceInit", GAP_PROFILE_PERIPHERAL | GAP_PROFILE_CENTRAL);
  //Initialize GAP layer for Peripheral and Central role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL | GAP_PROFILE_CENTRAL, selfEntity,
                 addrMode, &pRandomAddress);

  Seconds_set(1598997542);

}

/*********************************************************************
* @fn      multi_role_taskFxn
*
* @brief   Application task entry point for the multi_role.
*
* @param   a0, a1 - not used.
*
* @return  None.
*/
static void multi_role_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  multi_role_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, MR_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8_t safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = multi_role_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & MR_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          mrEvt_t *pMsg = (mrEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            multi_role_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
* @fn      multi_role_processStackMsg
*
* @brief   Process an incoming stack message.
*
* @param   pMsg - message to process
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : Stack msg status=%d, event=0x%x\n", pMsg->status, pMsg->event);

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      //multi_role_processRoleEvent((gapMultiRoleEvent_t *)pMsg);
      multi_role_processGapMsg((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      // Process GATT message
        Log_info0("In GATT_MSG_EVENT");
      safeToDealloc = multi_role_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch (pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
          break;

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

        // HCI Commands Events
        case HCI_COMMAND_STATUS_EVENT_CODE:
          {
            hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;
            switch ( pMyMsg->cmdOpcode )
            {
              case HCI_LE_SET_PHY:
                {
                  if (pMyMsg->cmdStatus ==
                      HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
                  {
                    //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0,"PHY Change failure, peer does not support this");
                      Log_info0("PHY Change failure, peer does not support this");
                  }
                  else
                  {
                      //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0,"PHY Update Status: 0x%02x",DipMyMsg->cmdStatus);
                      Log_info1("PHY Update Status 0x%02x", pMyMsg->cmdStatus);
                  }
                }
                break;
              case HCI_DISCONNECT:
                break;

              default:
                {
                  //Display_printf(dispHandle, MR_ROW_NON_CONN, 0,"Unknown Cmd Status: 0x%04x::0x%02x",pMyMsg->cmdOpcode, pMyMsg->cmdStatus);
                    Log_info2("Unknown Cmd Status: 0x%04x::0x%02x",pMyMsg->cmdOpcode, pMyMsg->cmdStatus);
                }
              break;
            }
          }
          break;

        // LE Events
        case HCI_LE_EVENT_CODE:
        {
          hciEvt_BLEPhyUpdateComplete_t *pPUC
            = (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

          if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
          {
            if (pPUC->status != SUCCESS)
            {

              //Display_printf(dispHandle, MR_ROW_ANY_CONN, 0,"%s: PHY change failure",multi_role_getConnAddrStr(pPUC->connHandle));
                Log_info0("PHY change failure");
            }
            else
            {
              //Display_printf(dispHandle, MR_ROW_ANY_CONN, 0,"%s: PHY updated to %s",multi_role_getConnAddrStr(pPUC->connHandle),
              // Only symmetrical PHY is supported.
              // rxPhy should be equal to txPhy.
                             //(pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1 Mbps" :
                             //(pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? "2 Mbps" :
                             //(pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "Coded" : "Unexpected PHY Value");
                Log_info1("PHY updated to %s",
                          (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1 Mbps" :
                        (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? "2 Mbps" :
                        (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "Coded" : "Unexpected PHY Value");
            }
          }

          break;
        }

        default:
          break;
      }

      break;
    }

    case L2CAP_SIGNAL_EVENT:
      // place holder for L2CAP Connection Parameter Reply
      break;

    default:
      // Do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      multi_role_processGapMsg
 *
 * @brief   GAP message processing function.
 *
 * @param   pMsg - pointer to event message structure
 *
 * @return  none
 */
static void multi_role_processGapMsg(gapEventHdr_t *pMsg)
{
  switch (pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;
      Log_info0("Ready to process GAP functions");
      printf("Hello\n");
      BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_DEVICE_INIT_DONE_EVENT", 0);
      if(pPkt->hdr.status == SUCCESS)
      {
        // Store the system ID
        uint8_t systemId[DEVINFO_SYSTEM_ID];

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = pPkt->devAddr[0];
        systemId[1] = pPkt->devAddr[1];
        systemId[2] = pPkt->devAddr[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = pPkt->devAddr[5];
        systemId[6] = pPkt->devAddr[4];
        systemId[5] = pPkt->devAddr[3];

        // Set Device Info Service Parameter
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- start advert %d,%d\n", 0, 0);


        //Setup and start advertising
        multi_role_advertInit();

        //disable advertising
        GapAdv_disable(advHandle);
        GapAdv_disable(advHandleTicks);
        GapAdv_disable(advHandleTime);

      }

      //Setup scanning
      multi_role_scanInit();

      //enable scanning indefinitely
      numScanRes = 0;
      GapScan_enable(0, 0, 0);

      mrMaxPduSize = pPkt->dataPktLen;


      //Display initialized state status
      //Display_printf(dispHandle, MR_ROW_NUM_CONN, 0, "Num Conns: %d", numConn);
      //Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "Initialized");
      //Display_printf(dispHandle, MR_ROW_MYADDRSS, 0, "Multi-Role Address: %s",(char *)Util_convertBdAddr2Str(pPkt->devAddr));

      Log_info1("Multi-Role Address: %s", (uintptr_t)(char *)Util_convertBdAddr2Str(pPkt->devAddr));
      break;
    }

    case GAP_CONNECTING_CANCELLED_EVENT:
    {


        Log_info0("Connection attempt cancelled");
      //Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "Conneting attempt cancelled");


      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      uint16_t connHandle = ((gapEstLinkReqEvent_t*) pMsg)->connectionHandle;
      uint8_t role = ((gapEstLinkReqEvent_t*) pMsg)->connRole;
      uint8_t* pAddr      = ((gapEstLinkReqEvent_t*) pMsg)->devAddr;
      uint8_t  connIndex;
      uint8_t* pStrAddr;
      //uint8_t i;
      //uint8_t numConnectable = 0;

      BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_LINK_ESTABLISHED_EVENT", 0);
      // Add this connection info to the list
      connIndex = multi_role_addConnInfo(connHandle, pAddr, role);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      connList[connIndex].charHandle = 0;

      Util_startClock(&clkPeriodic);

      pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

      Log_info1("Connected to %s", (uintptr_t)pStrAddr);
      Log_info1("No. of Conns: %d", numConn);

      //Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "Connected to %s", pStrAddr);
      //Display_printf(dispHandle, MR_ROW_NUM_CONN, 0, "Num Conns: %d", numConn);


      Log_info0("Setting connHandle...");
      mrConnHandle  = connList[0].connHandle;

      multi_role_enqueueMsg(MR_EVT_SVC_DISC, NULL);

      if (numConn < MAX_NUM_BLE_CONNS)
      {
        // Start advertising since there is room for more connections
        GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
        //Log_info0("Advertising re-enabled due to space for connections");
      }

      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      uint16_t connHandle = ((gapTerminateLinkEvent_t*) pMsg)->connectionHandle;
      uint8_t connIndex;
      //uint32_t itemsToEnable = MR_ITEM_STARTDISC | MR_ITEM_ADVERTISE | MR_ITEM_PHY;
      //uint8_t* pStrAddr;
      //uint8_t i;
      //uint8_t numConnectable = 0;

      BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : GAP msg: status=%d, opcode=%s\n", 0, "GAP_LINK_TERMINATED_EVENT");
      // Mark this connection deleted in the connected device list.
      connIndex = multi_role_removeConnInfo(connHandle);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      //pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

      //Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "%s is disconnected",pStrAddr);
      //Display_printf(dispHandle, MR_ROW_NUM_CONN, 0, "Num Conns: %d", numConn);


      // Start advertising since there is room for more connections
      GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);


      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      gapUpdateLinkParamReqReply_t rsp;
      gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsg;

      rsp.connectionHandle = pReq->req.connectionHandle;
      rsp.signalIdentifier = pReq->req.signalIdentifier;

      // Only accept connection intervals with slave latency of 0
      // This is just an example of how the application can send a response
      if(pReq->req.connLatency == 0)
      {
        rsp.intervalMin = pReq->req.intervalMin;
        rsp.intervalMax = pReq->req.intervalMax;
        rsp.connLatency = pReq->req.connLatency;
        rsp.connTimeout = pReq->req.connTimeout;
        rsp.accepted = TRUE;
      }
      else
      {
        rsp.accepted = FALSE;
      }

      // Send Reply
      VOID GAP_UpdateLinkParamReqReply(&rsp);

      break;
    }

     case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;
        Log_info0("in gap link param update");

        // Get the address from the connection handle
        linkDBInfo_t linkInfo;
        if (linkDB_GetInfo(pPkt->connectionHandle, &linkInfo) ==  SUCCESS)
        {

          if(pPkt->status == SUCCESS)
          {
              Log_info2("Updated params for %s, connTimeout: %d", (uintptr_t)Util_convertBdAddr2Str(linkInfo.addr),linkInfo.connTimeout*CONN_TIMEOUT_MS_CONVERSION);
              //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0,"Updated: %s, connTimeout:%d",Util_convertBdAddr2Str(linkInfo.addr),linkInfo.connTimeout*CONN_TIMEOUT_MS_CONVERSION);
          }
          else
          {
              //Log_warning2("Update Failed 0x%02x: %s", pPkt->opcode, (uintptr_t)Util_convertBdAddr2Str(linkInfo.addr));
              // Display the address of the connection update failure
            //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0,"Update Failed 0x%h: %s", pPkt->opcode,Util_convertBdAddr2Str(linkInfo.addr));
          }
        }
        // Check if there are any queued parameter updates
        mrConnHandleEntry_t *connHandleEntry = (mrConnHandleEntry_t *)List_get(&paramUpdateList);
        if (connHandleEntry != NULL)
        {
          // Attempt to send queued update now
          multi_role_processParamUpdate(connHandleEntry->connHandle);

          // Free list element
          ICall_free(connHandleEntry);
        }




        break;
      }

#if defined ( NOTIFY_PARAM_UPDATE_RJCT )
     case GAP_LINK_PARAM_UPDATE_REJECT_EVENT:
     {
       linkDBInfo_t linkInfo;
       gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

       // Get the address from the connection handle
       linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

       // Display the address of the connection update failure
       Display_printf(dispHandle, MR_ROW_CUR_CONN, 0,
                      "Peer Device's Update Request Rejected 0x%h: %s", pPkt->opcode,
                      Util_convertBdAddr2Str(linkInfo.addr));

       break;
     }
#endif

    default:
      break;
  }
}

/*********************************************************************
* @fn      multi_role_scanInit
*
* @brief   Setup initial device scan settings.
*
* @return  None.
*/
static void multi_role_scanInit(void)
{
  uint8_t temp8;
  uint16_t temp16;

  // Setup scanning
  // For more information, see the GAP section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/

  // Register callback to process Scanner events
  GapScan_registerCb(multi_role_scanCB, NULL);

  // Set Scanner Event Mask
  GapScan_setEventMask(GAP_EVT_SCAN_ENABLED | GAP_EVT_SCAN_DISABLED |
                       GAP_EVT_ADV_REPORT);

  // Set Scan PHY parameters
  GapScan_setPhyParams(DEFAULT_SCAN_PHY, SCAN_TYPE_ACTIVE,
                       SCAN_PARAM_DFLT_INTERVAL, SCAN_PARAM_DFLT_WINDOW);

  // Set Advertising report fields to keep
  temp16 = ADV_RPT_FIELDS;
  GapScan_setParam(SCAN_PARAM_RPT_FIELDS, &temp16);
  // Set Scanning Primary PHY
  temp8 = DEFAULT_SCAN_PHY;
  GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &temp8);
  // Set LL Duplicate Filter
  temp8 = SCAN_FLT_DUP_ENABLE;
  GapScan_setParam(SCAN_PARAM_FLT_DUP, &temp8);

  // Set PDU type filter -
  // Only 'Connectable' and 'Complete' packets are desired.
  // It doesn't matter if received packets are
  // whether Scannable or Non-Scannable, whether Directed or Undirected,
  // whether Scan_Rsp's or Advertisements, and whether Legacy or Extended.
  temp16 = SCAN_FLT_PDU_CONNECTABLE_ONLY | SCAN_FLT_PDU_COMPLETE_ONLY;
  GapScan_setParam(SCAN_PARAM_FLT_PDU_TYPE, &temp16);

  // Set initiating PHY parameters
  GapInit_setPhyParam(DEFAULT_INIT_PHY, INIT_PHYPARAM_CONN_INT_MIN,
					  INIT_PHYPARAM_MIN_CONN_INT);
  GapInit_setPhyParam(DEFAULT_INIT_PHY, INIT_PHYPARAM_CONN_INT_MAX,
					  INIT_PHYPARAM_MAX_CONN_INT);
}

/*********************************************************************
* @fn      multi_role_scanInit
*
* @brief   Setup initial advertisment and start advertising from device init.
*
* @return  None.
*/
static void multi_role_advertInit(void)
{
  uint8_t status = FAILURE;
  // Setup and start Advertising
  // For more information, see the GAP section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/


  BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- call GapAdv_create set=%d,%d\n", 1, 0);
  // Create Advertisement set #1 and assign handle
  GapAdv_create(&multi_role_advCB, &advParams1,
                &advHandle);

  // Load advertising data for set #1 that is statically allocated by the app
  GapAdv_loadByHandle(advHandle, GAP_ADV_DATA_TYPE_ADV,
                      sizeof(advData1), advData1);


  // Set event mask for set #1
  GapAdv_setEventMask(advHandle,
                      GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                      GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                      GAP_ADV_EVT_MASK_SET_TERMINATED);

  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GapAdv_enable", 0);


  // Enable legacy advertising for set #1
  status = GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

  //setup advertisement set 2

    BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- call GapAdv_create set=%d,%d\n", 2, 0);


    // Create Advertisement set #2 and assign handle
    status = GapAdv_create(&multi_role_advCB, &advParams1, &advHandleTime);

    // Load advertising data for set #2 that is statically allocated by the app
    status = GapAdv_loadByHandle(advHandleTime, GAP_ADV_DATA_TYPE_ADV, sizeof(advData2), advData2);

    // Set event mask for set #2
    GapAdv_setEventMask(advHandleTime,
                        GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                        GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                        GAP_ADV_EVT_MASK_SET_TERMINATED);

    BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GapAdv_enable", 0);
    // Enable legacy advertising for set #2
    status = GapAdv_enable(advHandleTime, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

    BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- call GapAdv_create set=%d,%d\n", 3, 0);


    // Create Advertisement set #2 and assign handle
    status = GapAdv_create(&multi_role_advCB, &advParams1, &advHandleTicks);

    // Load advertising data for set #2 that is statically allocated by the app
    status = GapAdv_loadByHandle(advHandleTicks, GAP_ADV_DATA_TYPE_ADV, sizeof(advData3), advData3);

    // Set event mask for set #2
    GapAdv_setEventMask(advHandleTicks,
                        GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                        GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                        GAP_ADV_EVT_MASK_SET_TERMINATED);

    BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GapAdv_enable", 0);
    // Enable legacy advertising for set #2
    status = GapAdv_enable(advHandleTicks, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);



    Log_info0("Initialise Advertising...");





  if(status != SUCCESS)
  {
    mrIsAdvertising = false;
    //Display_printf(dispHandle, MR_ROW_ADVERTIS, 0, "Error: Failed to Start Advertising!");
  }

  if (addrMode > ADDRMODE_RANDOM)
  {
    multi_role_updateRPA();

    // Create one-shot clock for RPA check event.
    Util_constructClock(&clkRpaRead, multi_role_clockHandler,
                        READ_RPA_PERIOD, 0, true,
                        (UArg) &argRpaRead);
  }
}

/*********************************************************************
 * @fn      multi_role_advCB
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void multi_role_advCB(uint32_t event, void *pBuf, uintptr_t arg)
{
  mrGapAdvEventData_t *pData = ICall_malloc(sizeof(mrGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(multi_role_enqueueMsg(MR_EVT_ADV, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}


/*********************************************************************
* @fn      multi_role_processGATTMsg
*
* @brief   Process GATT messages and events.
*
* @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
*/
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // Get connection index from handle
  uint8_t connIndex = multi_role_getConnIndex(pMsg->connHandle);
  MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  Log_info0("in processGATTMsg");

  if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
      Log_info1("FC Violation: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
      Log_info1("MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }


  // Messages from GATT server
  if (linkDB_Up(pMsg->connHandle))
  {
    if ((pMsg->method == ATT_READ_RSP) ||
        ((pMsg->method == ATT_ERROR_RSP) &&
         (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {
        //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
          Log_info1("Read Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a successful read, display the read value
        //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
          Log_info1("Read RSP: 0x%02x", pMsg->msg.readRsp.pValue[0]);
      }

    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {

      if (pMsg->method == ATT_ERROR_RSP)
      {
        //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
      }
      else
      {
        // After a succesful write, display the value that was written and
        // increment value
        //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Write sent: %d", charVal);
      }

    }
    else if (connList[connIndex].discState != BLE_DISC_STATE_IDLE)
    {
      multi_role_processGATTDiscEvent(pMsg);
    }
  } // Else - in case a GATT message came after a connection has dropped, ignore it.

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      multi_role_processParamUpdate
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static void multi_role_processParamUpdate(uint16_t connHandle)
{
  gapUpdateLinkParamReq_t req;
  uint8_t connIndex;

  req.connectionHandle = connHandle;
  req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
  req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
  req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
  req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

  connIndex = multi_role_getConnIndex(connHandle);
  MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Deconstruct the clock object
  Clock_destruct(connList[connIndex].pUpdateClock);
  // Free clock struct
  ICall_free(connList[connIndex].pUpdateClock);
  connList[connIndex].pUpdateClock = NULL;

  // Send parameter update
  bStatus_t status = GAP_UpdateLinkParamReq(&req);

  // If there is an ongoing update, queue this for when the udpate completes
  if (status == bleAlreadyInRequestedMode)
  {
    mrConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(mrConnHandleEntry_t));
    if (connHandleEntry)
    {
      connHandleEntry->connHandle = connHandle;

      List_put(&paramUpdateList, (List_Elem *)connHandleEntry);
    }
  }
}

/*********************************************************************
* @fn      multi_role_processAppMsg
*
* @brief   Process an incoming callback from a profile.
*
* @param   pMsg - message to process
*
* @return  None.
*/
static void multi_role_processAppMsg(mrEvt_t *pMsg)
{
  bool safeToDealloc = TRUE;

  if (pMsg->event <= APP_EVT_EVENT_MAX)
  {
    BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=%s\n", 0, appEventStrings[pMsg->event]);
  }
  else
  {
    BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=0x%x\n", 0, pMsg->event);
  }

  switch (pMsg->event)
  {
    case MR_EVT_CHAR_CHANGE:
    {
      multi_role_processCharValueChangeEvt(*(uint8_t*)(pMsg->pData));
      break;
    }

    case MR_EVT_KEY_CHANGE:
    {
      multi_role_handleKeys(*(uint8_t *)(pMsg->pData));
      break;
    }

    case MR_EVT_ADV_REPORT:
    {
      GapScan_Evt_AdvRpt_t* pAdvRpt = (GapScan_Evt_AdvRpt_t*) (pMsg->pData);
      Log_info0("In Advertising Report...");

      Seconds_getTime(&ts);
      timePreScan = ts.secs;
      ntimePreScan = ts.nsecs;

      ticksPreScan = Clock_getTicks();
      Log_info1("current clock ticks: %d", ticksPreScan);

      Log_info1("prescan time: %d", timePreScan);
      Log_info1("prescan ntime: %d", ntimePreScan);







#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
      if (multi_role_findSvcUuid(SIMPLEPROFILE_SERV_UUID, pAdvRpt->pData, pAdvRpt->dataLen))
      {
          Log_info0("");
          Log_info0(ANSI_COLOR(FG_GREEN) "Showing Devices with 0xFFF0:" ANSI_COLOR(ATTR_RESET));

          //receive incoming advertDataReport and save as char array
          const char* outputData = Util_convertBytes2Str(pAdvRpt->pData, pAdvRpt->dataLen);

          //copy all the data in a new array to be modified later on
          char allData[60];
          strcpy(allData, outputData);

          //save only manufacturer (custom) data
          char manuDataOnly[25];
          strncpy(manuDataOnly, allData+33, sizeof(allData));

          Log_info1("All Data Printout: %s", (uintptr_t)outputData);

          //adjust data to print in uartLog - UNUSED ATM
          memcpy(manuToPrint, manuDataOnly, sizeof(manuDataOnly)+1);
          manuToPrint[sizeof(manuDataOnly)+1] = '\0';

          //add received advert report to the struct
          multi_role_addScanInfo(pAdvRpt->addr, pAdvRpt->addrType, pAdvRpt->txPower, pAdvRpt->rssi, pAdvRpt->dataLen, &manuToPrint);
          Log_info1("Received AdvertData: " ANSI_COLOR(FG_GREEN) "%s" ANSI_COLOR(ATTR_RESET),(uintptr_t)manuToPrint);

          //disable scan after finding first device
          //need to alter this to run only when looking for timeServer advData


          //determine if the advertising report is from the correct device
          bool correctDevice = isCorrectDevice(manuDataOnly, ownDevAlpha, ownDevNum);

          //disable advertising once the correct device is being broadcasted
          if (correctDevice) {
              Log_info0("Correct Device Found");
              GapScan_disable();
          }

          else {
              Log_info0("Incorrect Device Found");
          }




        //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Discovered: %s",Util_convertBdAddr2Str(pAdvRpt->addr));
      }
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
      //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Discovered: %s",Util_convertBdAddr2Str(pAdvRpt->addr));
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

      // Free scan payload data
      if (pAdvRpt->pData != NULL)
      {
        ICall_free(pAdvRpt->pData);
      }
      break;
    }

    case MR_EVT_SCAN_ENABLED:
    {
        Log_info0("Scanning has started: Discovering...");
        printf("in scan enabled\n");

        // Disable everything but "Stop Discovering" on the menu


      break;
    }

    case MR_EVT_SCAN_DISABLED:
    {
      uint8_t numReport;
      uint8_t i;
      static uint8_t* pAddrs = NULL;
      uint8_t* pAddrTemp;

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
      numReport = numScanRes;
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
      GapScan_Evt_AdvRpt_t advRpt;

      numReport = ((GapScan_Evt_End_t*) (pMsg->pData))->numReport;
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID



      // Allocate buffer to display addresses
      pAddrs = ICall_malloc(numReport * MR_ADDR_STR_SIZE);
      if (pAddrs == NULL)
      {
        numReport = 0;
      }


      if (pAddrs != NULL)
      {
        pAddrTemp = pAddrs;
        for (i = 0; i < numReport; i++, pAddrTemp += MR_ADDR_STR_SIZE)
        {
  #if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
          // Get the address from the list, convert it to string, and
          // copy the string to the address buffer
          memcpy(pAddrTemp, Util_convertBdAddr2Str(scanList[i].addr),
                 MR_ADDR_STR_SIZE);
          //Log_info1("Datalen: %d", (int)scanList[i].dataLen);
  #else // !DEFAULT_DEV_DISC_BY_SVC_UUID
          // Get the address from the report, convert it to string, and
          // copy the string to the address buffer
          GapScan_getAdvReport(i, &advRpt);
          memcpy(pAddrTemp, Util_convertBdAddr2Str(advRpt.addr),
                 MR_ADDR_STR_SIZE);
  #endif // DEFAULT_DEV_DISC_BY_SVC_UUID


        }//end for loop


      }//end pAddrs if statement


      Log_info1("Scanning stopped: numReport = %d", numReport);




      //will run when timeClient is needed
      //will isolate the time and delay from the advertData
      if (timeClient){

          Log_info0("Isolating received timestamp:");

          //the latest advReport will be the one that matches the value
          multi_role_tickIsolation();

      }//end for loop for timeClient





      break;
    }

    case MR_EVT_SVC_DISC:
    {
        Log_info0("Service Discovery");
      multi_role_startSvcDiscovery();
      break;
    }

    case MR_EVT_ADV:
    {
      multi_role_processAdvEvent((mrGapAdvEventData_t*)(pMsg->pData));
      break;
    }

    case MR_EVT_PAIRING_STATE:
    {
      multi_role_processPairState((mrPairStateData_t*)(pMsg->pData));
      break;
    }

    case MR_EVT_PASSCODE_NEEDED:
    {
      multi_role_processPasscode((mrPasscodeData_t*)(pMsg->pData));
      break;
    }

    case MR_EVT_SEND_PARAM_UPDATE:
    {
      // Extract connection handle from data
      uint16_t locConnHandle = *(uint16_t *)(((mrClockEventData_t *)pMsg->pData)->data);
      multi_role_processParamUpdate(locConnHandle);
      safeToDealloc = FALSE;
      break;
    }

    case MR_EVT_PERIODIC:
    {
      multi_role_performPeriodicTask();
      break;
    }

    case MR_EVT_READ_RPA:
    {
      multi_role_updateRPA();
      break;
    }


    case MR_EVT_INSUFFICIENT_MEM:
    {
      // We are running out of memory.
      //Display_printf(dispHandle, MR_ROW_ANY_CONN, 0, "Insufficient Memory");
        Log_info0("Insufficient Memory");

      // We might be in the middle of scanning, try stopping it.
      GapScan_disable();
      break;
    }

    case MR_EVT_TIMESYNC:
    {
        //when the timesync event has been called, print to screen
        Log_info0("TimeSync successfully called");
        Seconds_set(1599767013);
        Util_startClock(&clkSecondsSet);
        break;
    }

    case MR_EVT_SECONDSSET:
    {
        Seconds_getTime(&ts);
        Log_info2("Seconds: %d, nSecs: %d", ts.secs, ts.nsecs);

     break;
    }

    //simulation for the 10 minute event
    case MR_EVT_PERIODICDATA:
    {

        Log_info1("Periodic Data Simulation Value");

        //will run after the initial time sync has run




        break;
    }

    default:
      // Do nothing.
      break;
  }

  if ((safeToDealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      multi_role_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void multi_role_processAdvEvent(mrGapAdvEventData_t *pEventData)
{



  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
      BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GAP_EVT_ADV_START_AFTER_ENABLE", 0);
      mrIsAdvertising = true;

      //

      //Log_info1("Adv set %d Enabled", (uintptr_t)(uint8_t *)pEventData->pBuf);
      //Display_printf(dispHandle, MR_ROW_ADVERTIS, 0, "Adv Set %d Enabled",*(uint8_t *)(pEventData->pBuf));
      Log_info1("Adv Set %d Enabled", *(uint8_t *)(pEventData->pBuf));



      if ((tickServer==true) && (count<1))
      {
          //get current tick value of clock
          ticksPostAdv = Clock_getTicks();
          Log_info1("ticksPostAdv %d", ticksPostAdv);

          //determine tick difference
          uint32_t ticksDiffAdv = ticksPostAdv - ticksPreAdv;

          //convert diff into hex
          char tempTickHexDelay [4];
          sprintf(tempTickHexDelay, "%X", ticksDiffAdv);
          printf("timeTickHexDelay %s\n", tempTickHexDelay);


          //need to include the current clock values in advertising
          Log_info1("Scan Delay (RX+TX): %d", combinedTickDelay);

          //will later add the combinedTickDelay value to the Tx tick value here and advertise

          int increment = 0;

          //load advertising handle to update advertising data
          GapAdv_prepareLoadByHandle(advHandleTicks, GAP_ADV_FREE_OPTION_DONT_FREE);
          size_t tempSize = 2;

          //add each value to the advData
          for (int i = 13; i < 15; i++) {
              char tempChar[2];
              long int tempLong = 0;
              strncpy(tempChar, tempTickHexDelay + increment, tempSize);
              Log_info1("tempChar %s", (uintptr_t)tempChar);

              tempLong = strtol(tempChar, 0, 16);
              Log_info1("tempLong %d", tempLong);
              char toAdvData = tempLong;

              advData3[i] = toAdvData;
              increment = increment+2;
          }//end for loop

          GapAdv_loadByHandle(advHandleTicks, GAP_ADV_DATA_TYPE_ADV, sizeof(advData3), advData3);
          //timeServer=false;
          Log_info2("Count[%d]: Diff: %d", count, ticksDiffAdv);
          count = count+1;

      }//end if for tickServer isolation



      if ((timeServer==true) && (count<1))
            {
                Seconds_getTime(&ts);
                timePostAdv = ts.secs;
                ntimePostAdv = ts.nsecs;
                timeDiff = timePostAdv - timePreAdv;
                ntimeDiff = ntimePostAdv - ntimePreAdv;

                //update advert data with delay information

                char tempHexDelay[8];
                int increment = 0;

                sprintf(tempHexDelay, "%X", ntimeDiff);

                printf("nTime (hex): %s\n", tempHexDelay);

                GapAdv_prepareLoadByHandle(advHandleTime, GAP_ADV_FREE_OPTION_DONT_FREE);
                size_t tempSize = 2;

                //add each value to the advData
                for (int i = 17; i < 20; i++) {
                    char tempChar[2];
                    long int tempLong = 0;
                    strncpy(tempChar, tempHexDelay + increment, tempSize);
                    //printf("AdvData[%d]: %s\n", i, tempChar);
                    tempLong = strtol(tempChar, 0, 16);
                    char toAdvData = tempLong;

                    advData2[i] = toAdvData;
                    increment = increment+2;
                }//end for loop

                GapAdv_loadByHandle(advHandleTime, GAP_ADV_DATA_TYPE_ADV, sizeof(advData2), advData2);
                //timeServer=false;
                Log_info2("Count[%d]: Diff: %d", count, ntimeDiff);
                count = count+1;

            }//end if for time server functions


      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
      mrIsAdvertising = false;
      //Display_printf(dispHandle, MR_ROW_ADVERTIS, 0, "Adv Set %d Disabled",*(uint8_t *)(pEventData->pBuf));
      Log_info1("Adv Set %d Disabled", *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_START:
      //Display_printf(dispHandle, MR_ROW_ADVERTIS, 0, "Adv Started %d Enabled",*(uint8_t *)(pEventData->pBuf));
        Log_info1("Adv Started: %d Set Enable", *(uint8_t *)(pEventData->pBuf));


      break;

    case GAP_EVT_ADV_END:
      //Display_printf(dispHandle, MR_ROW_ADVERTIS, 0, "Adv Ended %d Disabled",*(uint8_t *)(pEventData->pBuf));
        Log_info1("Adv Ended: %d Disabled", *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
    {
      mrIsAdvertising = false;
#ifndef Display_DISABLE_ALL
      GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);
#endif
      //Display_printf(dispHandle, MR_ROW_ADVERTIS, 0, "Adv Set %d disabled after conn %d",advSetTerm->handle, advSetTerm->connHandle );
    }
    break;

    case GAP_EVT_SCAN_REQ_RECEIVED:
      break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
      break;

    default:
      break;
  }

  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
/*********************************************************************
 * @fn      multi_role_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool multi_role_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                      uint16_t dataLen)
{
  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;

  if (dataLen > 0)
  {
    pEnd = pData + dataLen - 1;

    // While end of data not reached
    while (pData < pEnd)
    {
      // Get length of next AD item
      adLen = *pData++;
      if (adLen > 0)
      {
        adType = *pData;

        // If AD type is for 16-bit service UUID
        if ((adType == GAP_ADTYPE_16BIT_MORE) ||
            (adType == GAP_ADTYPE_16BIT_COMPLETE))
        {
          pData++;
          adLen--;

          // For each UUID in list
          while (adLen >= 2 && pData < pEnd)
          {
            // Check for match
            if ((pData[0] == LO_UINT16(uuid)) && (pData[1] == HI_UINT16(uuid)))
            {
              // Match found
              return TRUE;
            }

            // Go to next
            pData += 2;
            adLen -= 2;
          }

          // Handle possible erroneous extra byte in UUID list
          if (adLen == 1)
          {
            pData++;
          }
        }
        else
        {
          // Go to next item
          pData += adLen;
        }
      }
    }
  }

  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      multi_role_addScanInfo
 *
 * @brief   Add a device to the scanned device list
 *
 * @return  none
 */
static void multi_role_addScanInfo(uint8_t *pAddr, uint8_t addrType, uint8_t txPower, uint8_t rssi, uint16_t dataLen, char *receivedData)
{
  uint8_t i;

  // If result count not at max
  if (numScanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < numScanRes; i++)
    {
      if (memcmp(pAddr, scanList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }

    // Add addr to scan result list
    memcpy(scanList[numScanRes].addr, pAddr, B_ADDR_LEN);
    scanList[numScanRes].addrType = addrType;
    scanList[numScanRes].txPower = txPower;
    scanList[numScanRes].rssi = rssi;
    scanList[numScanRes].dataLen = dataLen;
    strcpy(scanList[numScanRes].manuData, receivedData);

    Log_info4(ANSI_COLOR(FG_BLUE) "Dev No: %d" ANSI_COLOR(ATTR_RESET)  ", txPower: %d, rssi %d, BT ADDR: " ANSI_COLOR(FG_GREEN) "%s" ANSI_COLOR(ATTR_RESET), numScanRes+1, txPower, rssi, (uintptr_t)Util_convertBdAddr2Str(pAddr));

    // Increment scan result count
    numScanRes++;
  }
}
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

/*********************************************************************
 * @fn      multi_role_scanCB
 *
 * @brief   Callback called by GapScan module
 *
 * @param   evt - event
 * @param   msg - message coming with the event
 * @param   arg - user argument
 *
 * @return  none
 */
void multi_role_scanCB(uint32_t evt, void* pMsg, uintptr_t arg)
{
  uint8_t event;

  if (evt & GAP_EVT_ADV_REPORT)
  {
    event = MR_EVT_ADV_REPORT;
  }
  else if (evt & GAP_EVT_SCAN_ENABLED)
  {
    event = MR_EVT_SCAN_ENABLED;
  }
  else if (evt & GAP_EVT_SCAN_DISABLED)
  {
    event = MR_EVT_SCAN_DISABLED;
  }
  else if (evt & GAP_EVT_INSUFFICIENT_MEMORY)
  {
    event = MR_EVT_INSUFFICIENT_MEM;
  }
  else
  {
    return;
  }

  if(multi_role_enqueueMsg(event, pMsg) != SUCCESS)
  {
    ICall_free(pMsg);
  }

}

/*********************************************************************
* @fn      multi_role_charValueChangeCB
*
* @brief   Callback from Simple Profile indicating a characteristic
*          value change.
*
* @param   paramID - parameter ID of the value that was changed.
*
* @return  None.
*/
static void multi_role_charValueChangeCB(uint8_t paramID)
{
  uint8_t *pData;

  // Allocate space for the event data.
  if ((pData = ICall_malloc(sizeof(uint8_t))))
  {
    *pData = paramID;

    // Queue the event.
    if(multi_role_enqueueMsg(MR_EVT_CHAR_CHANGE, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      multi_role_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static status_t multi_role_enqueueMsg(uint8_t event, void *pData)
{
  uint8_t success;
  mrEvt_t *pMsg = ICall_malloc(sizeof(mrEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return(bleMemAllocError);
}

/*********************************************************************
 * @fn      multi_role_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void multi_role_processCharValueChangeEvt(uint8_t paramId)
{
  uint8_t newValue;

  switch(paramId)
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);

      //Display_printf(dispHandle, MR_ROW_CHARSTAT, 0, "Char 1: %d", (uint16_t)newValue);
      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);

      //Display_printf(dispHandle, MR_ROW_CHARSTAT, 0, "Char 3: %d", (uint16_t)newValue);
      break;

    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      multi_role_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void multi_role_performPeriodicTask(void)
{
  uint8_t valueToCopy;

  // Call to retrieve the value of the third characteristic in the profile
  if (SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &valueToCopy) == SUCCESS)
  {
    // Call to set that value of the fourth characteristic in the profile.
    // Note that if notifications of the fourth characteristic have been
    // enabled by a GATT client device, then a notification will be sent
    // every time this function is called.
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &valueToCopy);
  }
}

/*********************************************************************
 * @fn      multi_role_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void multi_role_updateRPA(void)
{
  uint8_t* pRpaNew;

  // Read the current RPA.
  pRpaNew = GAP_GetDevAddress(FALSE);

  if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
  {
    // If the RPA has changed, update the display
    //Display_printf(dispHandle, MR_ROW_RPA, 0, "RP Addr: %s",Util_convertBdAddr2Str(pRpaNew));
    memcpy(rpa, pRpaNew, B_ADDR_LEN);
  }
}

/*********************************************************************
 * @fn      multi_role_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void multi_role_clockHandler(UArg arg)
{
  mrClockEventData_t *pData = (mrClockEventData_t *)arg;

  if (pData->event == MR_EVT_PERIODIC)
  {
    // Start the next period
    Util_startClock(&clkPeriodic);

    // Send message to perform periodic task
    multi_role_enqueueMsg(MR_EVT_PERIODIC, NULL);
  }
  else if (pData->event == MR_EVT_READ_RPA)
  {
    // Start the next period
    Util_startClock(&clkRpaRead);

    // Send message to read the current RPA
    multi_role_enqueueMsg(MR_EVT_READ_RPA, NULL);
  }
  else if (pData->event == MR_EVT_SEND_PARAM_UPDATE)
  {
    // Send message to app
    multi_role_enqueueMsg(MR_EVT_SEND_PARAM_UPDATE, pData);
  }
  else if (pData->event == MR_EVT_TIMESYNC)
  {
      //send message to app
      multi_role_enqueueMsg(MR_EVT_TIMESYNC, NULL);
  }
  else if (pData->event == MR_EVT_SECONDSSET)
  {
      //send message to app
      multi_role_enqueueMsg(MR_EVT_SECONDSSET, NULL);
   }
  else if (pData->event == MR_EVT_PERIODICDATA)
  {
      multi_role_enqueueMsg(MR_EVT_PERIODICDATA, NULL);
  }

}

/*********************************************************************
* @fn      multi_role_keyChangeHandler
*
* @brief   Key event handler function
*
* @param   a0 - ignored
*
* @return  none
*/
static void multi_role_keyChangeHandler(uint8_t keys)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = keys;

    multi_role_enqueueMsg(MR_EVT_KEY_CHANGE, pValue);
  }
}

/*********************************************************************
* @fn      multi_role_handleKeys
*
* @brief   Handles all key events for this device.
*
* @param   keys - bit field for key events. Valid entries:
*                 HAL_KEY_SW_2
*                 HAL_KEY_SW_1
*
* @return  none
*/
static void multi_role_handleKeys(uint8_t keys)
{
  uint32_t rtnVal = 0;
  if (keys & KEY_LEFT)
  {
    // Check if the key is still pressed
    if (PIN_getInputValue(CONFIG_PIN_BTN1) == 0)
    {


        //multi_role_serviceDiscovery(0);

        foundCorrectDev = 0;
        timeClient = true;
        GapAdv_disable(advHandle);
        GapAdv_disable(advHandleTicks);
        GapAdv_disable(advHandleTime);

        multi_role_doDiscoverDevices();

        //left button handler
    }
  }
  else if (keys & KEY_RIGHT)
  {
    // Check if the key is still pressed
    rtnVal = PIN_getInputValue(CONFIG_PIN_BTN2);
    if (rtnVal == 0)
    {
      //right button handler
      //multi_role_doConnect(0);
        multi_role_tickSend();

    }
  }
}

/*********************************************************************
* @fn      multi_role_processGATTDiscEvent
*
* @brief   Process GATT discovery event
*
* @param   pMsg - pointer to discovery event stack message
*
* @return  none
*/
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
  uint8_t connIndex = multi_role_getConnIndex(pMsg->connHandle);
  MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  Log_info1("discState in discEvent: %d", connList[connIndex].discState);

  if (connList[connIndex].discState == BLE_DISC_STATE_MTU)
  {
    // MTU size response received, discover simple service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                         HI_UINT16(SIMPLEPROFILE_SERV_UUID) };

      connList[connIndex].discState = BLE_DISC_STATE_SVC;

      // Discovery simple service
      VOID GATT_DiscPrimaryServiceByUUID(pMsg->connHandle, uuid,
                                         ATT_BT_UUID_SIZE, selfEntity);
    }
  }
  else if (connList[connIndex].discState == BLE_DISC_STATE_SVC)
  {
    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0)
    {
      svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
    }

    // If procedure complete
    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) &&
         (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
      if (svcStartHdl != 0)
      {
        attReadByTypeReq_t req;

        // Discover characteristic
        connList[connIndex].discState = BLE_DISC_STATE_CHAR;

        req.startHandle = svcStartHdl;
        req.endHandle = svcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        VOID GATT_DiscCharsByUUID(pMsg->connHandle, &req, selfEntity);
      }
    }
  }
  else if (connList[connIndex].discState == BLE_DISC_STATE_CHAR)
  {
    // Characteristic found, store handle
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) &&
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {
      uint8_t connIndex = multi_role_getConnIndex(mrConnHandle);

      // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
      MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

      // Store the handle of the simpleprofile characteristic 1 value
      connList[connIndex].charHandle
        = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[3],
                       pMsg->msg.readByTypeRsp.pDataList[4]);

      //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Simple Svc Found");
      Log_info0("Simple svc found");

    }

    connList[connIndex].discState = BLE_DISC_STATE_IDLE;
  }
}

/*********************************************************************
* @fn      multi_role_getConnIndex
*
* @brief   Translates connection handle to index
*
* @param   connHandle - the connection handle
*
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
*/
static uint16_t multi_role_getConnIndex(uint16_t connHandle)
{
  uint8_t i;
  // Loop through connection
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    // If matching connection handle found
    if (connList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  // Not found if we got here
  return(MAX_NUM_BLE_CONNS);
}

#ifndef Display_DISABLE_ALL
/*********************************************************************
 * @fn      multi_role_getConnAddrStr
 *
 * @brief   Return, in string form, the address of the peer associated with
 *          the connHandle.
 *
 * @return  A null-terminated string of the address.
 *          if there is no match, NULL will be returned.
 */
static char* multi_role_getConnAddrStr(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return Util_convertBdAddr2Str(connList[i].addr);
    }
  }

  return NULL;
}
#endif

/*********************************************************************
 * @fn      multi_role_clearConnListEntry
 *
 * @brief   clear device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t multi_role_clearConnListEntry(uint16_t connHandle)
{
  uint8_t i;
  // Set to invalid connection index initially
  uint8_t connIndex = MAX_NUM_BLE_CONNS;

  if(connHandle != LINKDB_CONNHANDLE_ALL)
  {
    connIndex = multi_role_getConnIndex(connHandle);
    // Get connection index from handle
    if(connIndex >= MAX_NUM_BLE_CONNS)
    {
      return bleInvalidRange;
    }
  }

  // Clear specific handle or all handles
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL))
    {
      connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
      connList[i].charHandle = 0;
      connList[i].discState  =  0;
    }
  }

  return SUCCESS;
}


/************************************************************************
* @fn      multi_role_pairStateCB
*
* @param   connHandle - the connection handle
*
* @param   state - pairing state
*
* @param   status - status of pairing state
*
* @return  none
*/
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status)
{
  mrPairStateData_t *pData = ICall_malloc(sizeof(mrPairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if (multi_role_enqueueMsg(MR_EVT_PAIRING_STATE, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
* @fn      multi_role_passcodeCB
*
* @brief   Passcode callback.
*
* @param   deviceAddr - pointer to device address
*
* @param   connHandle - the connection handle
*
* @param   uiInputs - pairing User Interface Inputs
*
* @param   uiOutputs - pairing User Interface Outputs
*
* @param   numComparison - numeric Comparison 20 bits
*
* @return  none
*/
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs,
                                  uint32_t numComparison)
{
  mrPasscodeData_t *pData = ICall_malloc(sizeof(mrPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData)
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if (multi_role_enqueueMsg(MR_EVT_PASSCODE_NEEDED, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
* @fn      multi_role_processPairState
*
* @brief   Process the new paring state.
*
* @param   pairingEvent - pairing event received from the stack
*
* @return  none
*/
static void multi_role_processPairState(mrPairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;
  uint8_t status = pPairData->status;

  switch (state)
  {
    case GAPBOND_PAIRING_STATE_STARTED:
      //Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Pairing started");
        Log_info0("Pairing started");
      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
      if (status == SUCCESS)
      {
        linkDBInfo_t linkInfo;

        Log_info0("Pairing success");
        //Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Pairing success");

        if (linkDB_GetInfo(pPairData->connHandle, &linkInfo) == SUCCESS)
        {
          // If the peer was using private address, update with ID address
          if ((linkInfo.addrType == ADDRTYPE_PUBLIC_ID ||
               linkInfo.addrType == ADDRTYPE_RANDOM_ID) &&
              !Util_isBufSet(linkInfo.addrPriv, 0, B_ADDR_LEN))

          {
            // Update the address of the peer to the ID address
            //Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "Addr updated: %s",Util_convertBdAddr2Str(linkInfo.addr));
              Log_info1("Dev using RPA, Addr Updated to: %s", (uintptr_t)Util_convertBdAddr2Str(linkInfo.addr));
            // Update the connection list with the ID address
            uint8_t i = multi_role_getConnIndex(pPairData->connHandle);

            MULTIROLE_ASSERT(i < MAX_NUM_BLE_CONNS);
            memcpy(connList[i].addr, linkInfo.addr, B_ADDR_LEN);
          }
        }
      }
      else
      {
        //Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Pairing fail: %d", status);
          Log_info1("Pairing fail: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
      if (status == SUCCESS)
      {
        //Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Encryption success");
          Log_info0("Encryption success");
      }
      else
      {
        //Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Encryption failed: %d", status);
          Log_info1("Encryption failed: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
      if (status == SUCCESS)
      {
        //Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Bond save success");
          Log_info0("Bond save success");
      }
      else
      {
        //Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Bond save failed: %d", status);
          Log_info1("Bond save failed: %d", status);
      }

      break;

    default:
      break;
  }
}

/*********************************************************************
* @fn      multi_role_processPasscode
*
* @brief   Process the Passcode request.
*
* @return  none
*/
static void multi_role_processPasscode(mrPasscodeData_t *pData)
{
  // Display passcode to user
  if (pData->uiOutputs != 0)
  {
    //Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Passcode: %d",B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pData->connHandle, SUCCESS,
                         B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      multi_role_startSvcDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void multi_role_startSvcDiscovery(void)
{
  uint8_t connIndex = multi_role_getConnIndex(mrConnHandle);
  Log_info1("Conn index: %d", connIndex);

  // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
  MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  attExchangeMTUReq_t req;

  // Initialize cached handles
  svcStartHdl = svcEndHdl = 0;

  connList[connIndex].discState = BLE_DISC_STATE_MTU;

  Log_info1("discState: %d", connList[connIndex].discState);

  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = mrMaxPduSize - L2CAP_HDR_SIZE;
  Log_info1("Client RX MTU: %d", (int)req.clientRxMTU);


  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(mrConnHandle, &req, selfEntity);
  Log_info0("End of svcDiscovery");
}

/*********************************************************************
* @fn      multi_role_addConnInfo
*
* @brief   add a new connection to the index-to-connHandle map
*
* @param   connHandle - the connection handle
*
* @param   addr - pointer to device address
*
* @return  index of connection handle
*/
static uint8_t multi_role_addConnInfo(uint16_t connHandle, uint8_t *pAddr,
                                      uint8_t role)
{
  uint8_t i;
  mrClockEventData_t *paramUpdateEventData;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;
      memcpy(connList[i].addr, pAddr, B_ADDR_LEN);
      numConn++;

      // If a peripheral, start the clock to send a connection parameter update
      if(role == GAP_PROFILE_PERIPHERAL)
      {
        // Allocate data to send through clock handler
        paramUpdateEventData = ICall_malloc(sizeof(mrClockEventData_t) +
                                            sizeof(uint16_t));
        if(paramUpdateEventData)
        {
          // Set clock data
          paramUpdateEventData->event = MR_EVT_SEND_PARAM_UPDATE;
          *((uint16_t *)paramUpdateEventData->data) = connHandle;

          // Create a clock object and start
          connList[i].pUpdateClock
            = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

          if (connList[i].pUpdateClock)
          {
            Util_constructClock(connList[i].pUpdateClock,
                                multi_role_clockHandler,
                                SEND_PARAM_UPDATE_DELAY, 0, true,
                                (UArg) paramUpdateEventData);
          }
        }
        else
        {
          // Memory allocation failed
          MULTIROLE_ASSERT(false);
        }
      }

      break;
    }
  }

  return i;
}

/*********************************************************************
 * @fn      multi_role_clearPendingParamUpdate
 *
 * @brief   clean pending param update request in the paramUpdateList list
 *
 * @param   connHandle - connection handle to clean
 *
 * @return  none
 */
void multi_role_clearPendingParamUpdate(uint16_t connHandle)
{
  List_Elem *curr;

  for (curr = List_head(&paramUpdateList); curr != NULL; curr = List_next(curr)) 
  {
    if (((mrConnHandleEntry_t *)curr)->connHandle == connHandle)
    {
      List_remove(&paramUpdateList, curr);
    }
  }
}

/*********************************************************************
 * @fn      multi_role_removeConnInfo
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t multi_role_removeConnInfo(uint16_t connHandle)
{
  uint8_t connIndex = multi_role_getConnIndex(connHandle);

  if(connIndex < MAX_NUM_BLE_CONNS)
  {
    Clock_Struct* pUpdateClock = connList[connIndex].pUpdateClock;

    if (pUpdateClock != NULL)
    {
      // Stop and destruct the RTOS clock if it's still alive
      if (Util_isActive(pUpdateClock))
      {
        Util_stopClock(pUpdateClock);
      }

      // Destruct the clock object
      Clock_destruct(pUpdateClock);
      // Free clock struct
      ICall_free(pUpdateClock);
    }
    // Clear pending update requests from paramUpdateList
    multi_role_clearPendingParamUpdate(connHandle);
    // Clear Connection List Entry
    multi_role_clearConnListEntry(connHandle);
    numConn--;
  }

  return connIndex;
}

/*********************************************************************
* @fn      multi_role_doDiscoverDevices
*
* @brief   Respond to user input to start scanning
*
* @param   index - not used
*
* @return  TRUE since there is no callback to use this value
*/
void multi_role_doDiscoverDevices(void)
{
    Log_info0("Left Button Pressed");


#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
  // Scanning for DEFAULT_SCAN_DURATION x 10 ms.
  // The stack does not need to record advertising reports
  // since the application will filter them by Service UUID and save.

  // Reset number of scan results to 0 before starting scan
  numScanRes = 0;
  GapScan_enable(0, DEFAULT_SCAN_DURATION, 0);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
  // Scanning for DEFAULT_SCAN_DURATION x 10 ms.
  // Let the stack record the advertising reports as many as up to DEFAULT_MAX_SCAN_RES.
  GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID


  //return (true);
}

/*********************************************************************
 * @fn      multi_role_doStopDiscovering
 *
 * @brief   Stop on-going scanning
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool multi_role_doStopDiscovering(uint8_t index)
{
  (void) index;

  GapScan_disable();

  return (true);
}

/*********************************************************************
 * @fn      multi_role_doCancelConnecting
 *
 * @brief   Cancel on-going connection attempt
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool multi_role_doCancelConnecting(uint8_t index)
{
  (void) index;

  GapInit_cancelConnect();

  return (true);
}

/*********************************************************************
* @fn      multi_role_doConnect
*
* @brief   Respond to user input to form a connection
*
* @param   index - index as selected from the mrMenuConnect
*
* @return  TRUE since there is no callback to use this value
*/
bool multi_role_doConnect(uint8_t index)
{
  // Temporarily disable advertising
    Log_info0("");
    Log_info0("Disabling Advertising...");
  GapAdv_disable(advHandle);

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
  Log_info2("Attempting Connection with Device: %d with BD Addr: %s", index, (uintptr_t)Util_convertBdAddr2Str(scanList[index].addr));
  GapInit_connect(scanList[index].addrType & MASK_ADDRTYPE_ID,
                  scanList[index].addr, mrInitPhy, 0);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
  GapScan_Evt_AdvRpt_t advRpt;

  GapScan_getAdvReport(index, &advRpt);

  GapInit_connect(advRpt.addrType & MASK_ADDRTYPE_ID,
                  advRpt.addr, mrInitPhy, 0);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

  // Re-enable advertising
  Log_info0("Re-enabling Advertising...");
  GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);



  //Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "Connecting...");

  return (true);
}

/*********************************************************************
 * @fn      multi_role_doSelectConn
 *
 * @brief   Select a connection to communicate with
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool multi_role_doSelectConn(uint8_t index)
{


    Log_info0("in doselectconn");
  // index cannot be equal to or greater than MAX_NUM_BLE_CONNS
  MULTIROLE_ASSERT(index < MAX_NUM_BLE_CONNS);

  mrConnHandle  = connList[index].connHandle;

  if (connList[index].charHandle == 0)
  {
    // Initiate service discovery
      Log_info1("Initiating service discovery with device %d", index);
    multi_role_enqueueMsg(MR_EVT_SVC_DISC, NULL);
  }




  return (true);
}

/*********************************************************************
 * @fn      multi_role_doGattRead
 *
 * @brief   GATT Read
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool multi_role_doGattRead(uint8_t index)
{
  attReadReq_t req;
  uint8_t connIndex = multi_role_getConnIndex(mrConnHandle);

  // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
  MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  req.handle = connList[connIndex].charHandle;
  GATT_ReadCharValue(mrConnHandle, &req, selfEntity);

  return (true);
}

/*********************************************************************
 * @fn      multi_role_doGattWrite
 *
 * @brief   GATT Write
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool multi_role_doGattWrite(uint8_t index)
{
  status_t status;
  uint8_t charVals[4] = { 0x00, 0x55, 0xAA, 0xFF }; // Should be consistent with
                                                    // those in scMenuGattWrite
  attWriteReq_t req;

  req.pValue = GATT_bm_alloc(mrConnHandle, ATT_WRITE_REQ, 1, NULL);

  if ( req.pValue != NULL )
  {
    uint8_t connIndex = multi_role_getConnIndex(mrConnHandle);

    // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
    MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    req.handle = connList[connIndex].charHandle;
    req.len = 1;
    charVal = charVals[index];
    req.pValue[0] = charVal;
    req.sig = 0;
    req.cmd = 0;

    status = GATT_WriteCharValue(mrConnHandle, &req, selfEntity);
    if ( status != SUCCESS )
    {
      GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
    }
  }

  return (true);
}

/*********************************************************************
* @fn      multi_role_doConnUpdate
*
* @brief   Respond to user input to do a connection update
*
* @param   index - index as selected from the mrMenuConnUpdate
*
* @return  TRUE since there is no callback to use this value
*/
bool multi_role_doConnUpdate(uint8_t index)
{
  gapUpdateLinkParamReq_t params;

  (void) index; //may need to get the real connHandle?

  params.connectionHandle = mrConnHandle;
  params.intervalMin = DEFAULT_UPDATE_MIN_CONN_INTERVAL;
  params.intervalMax = DEFAULT_UPDATE_MAX_CONN_INTERVAL;
  params.connLatency = DEFAULT_UPDATE_SLAVE_LATENCY;

  linkDBInfo_t linkInfo;
  if (linkDB_GetInfo(mrConnHandle, &linkInfo) == SUCCESS)
  {
    if (linkInfo.connTimeout == DEFAULT_UPDATE_CONN_TIMEOUT)
    {
      params.connTimeout = DEFAULT_UPDATE_CONN_TIMEOUT + 200;
    }
    else
    {
      params.connTimeout = DEFAULT_UPDATE_CONN_TIMEOUT;
    }

    GAP_UpdateLinkParamReq(&params);

    //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Param update Request:connTimeout =%d",params.connTimeout*CONN_TIMEOUT_MS_CONVERSION);
  }
  else
  {

    //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0,"update :%s, Unable to find link information",Util_convertBdAddr2Str(linkInfo.addr));
  }

  return (true);
}

/*********************************************************************
 * @fn      multi_role_doConnPhy
 *
 * @brief   Set Connection PHY preference.
 *
 * @param   index - item number in MRMenu_connPhy list
 *
 * @return  always true
 */
bool multi_role_doConnPhy(uint8_t index)
{
  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX. For more information, see the LE 2M PHY section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  // Note PHYs are already enabled by default in build_config.opt in stack project.
  HCI_LE_SetPhyCmd(mrConnHandle, 0, MRMenu_connPhy[index].value, MRMenu_connPhy[index].value, 0);

  //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Connection PHY preference: %s",TBM_GET_ACTION_DESC(&mrMenuConnPhy, index));

  return (true);
}

/*********************************************************************
 * @fn      multi_role_doSetInitPhy
 *
 * @brief   Set initialize PHY preference.
 *
 * @param   index - item number in MRMenu_initPhy list
 *
 * @return  always true
 */
bool multi_role_doSetInitPhy(uint8_t index)
{
  mrInitPhy = MRMenu_initPhy[index].value;
  //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Initialize PHY preference: %s",TBM_GET_ACTION_DESC(&mrMenuInitPhy, index));

  return (true);
}

/*********************************************************************
 * @fn      multi_role_doSetScanPhy
 *
 * @brief   Set PHYs for scanning.
 *
 * @param   index - item number in MRMenu_scanPhy list
 *
 * @return  always true
 */
bool multi_role_doSetScanPhy(uint8_t index)
{
  // Set scanning primary PHY
  GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &MRMenu_scanPhy[index].value);

  //Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "Primary Scan PHY: %s",TBM_GET_ACTION_DESC(&mrMenuScanPhy, index));

  return (true);
}

/*********************************************************************
 * @fn      multi_role_doSetAdvPhy
 *
 * @brief   Set advertise PHY preference.
 *
 * @param   index - item number in MRMenu_advPhy list
 *
 * @return  always true
 */
bool multi_role_doSetAdvPhy(uint8_t index)
{
  uint16_t props;
  GapAdv_primaryPHY_t phy;
  bool isAdvActive = mrIsAdvertising;
  
  switch (MRMenu_advPhy[index].value)
  {
    case MR_ADV_LEGACY_PHY_1_MBPS:
        props = GAP_ADV_PROP_CONNECTABLE | GAP_ADV_PROP_SCANNABLE | GAP_ADV_PROP_LEGACY;
        phy = GAP_ADV_PRIM_PHY_1_MBPS;     
    break;
    case MR_ADV_EXT_PHY_1_MBPS:
        props = GAP_ADV_PROP_CONNECTABLE;
        phy = GAP_ADV_PRIM_PHY_1_MBPS;
    break;
    case MR_ADV_EXT_PHY_CODED:
        props = GAP_ADV_PROP_CONNECTABLE;
        phy = GAP_ADV_PRIM_PHY_CODED_S2;
    break;
    default:
        return (false);
  }
  if (isAdvActive)
  {
    // Turn off advertising
    GapAdv_disable(advHandle);
  }
  GapAdv_setParam(advHandle,GAP_ADV_PARAM_PROPS,&props);
  GapAdv_setParam(advHandle,GAP_ADV_PARAM_PRIMARY_PHY,&phy);
  GapAdv_setParam(advHandle,GAP_ADV_PARAM_SECONDARY_PHY,&phy);
  if (isAdvActive)
  {
    // Turn on advertising
    GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
  }
  
  //Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Advertise PHY preference: %s",TBM_GET_ACTION_DESC(&mrMenuAdvPhy, index));

  return (true);
}

/*********************************************************************
* @fn      multi_role_doDisconnect
*
* @brief   Respond to user input to terminate a connection
*
* @param   index - index as selected from the mrMenuConnUpdate
*
* @return  always true
*/
bool multi_role_doDisconnect(uint8_t index)
{
  (void) index;

  // Disconnect
  GAP_TerminateLinkReq(mrConnHandle, HCI_DISCONNECT_REMOTE_USER_TERM);

  return (true);
}

/*********************************************************************
* @fn      multi_role_doAdvertise
*
* @brief   Respond to user input to terminate a connection
*
* @param   index - index as selected from the mrMenuConnUpdate
*
* @return  always true
*/
bool multi_role_doAdvertise(uint8_t index)
{
  (void) index;

  // If we're currently advertising
  if (mrIsAdvertising)
  {
    // Turn off advertising
    GapAdv_disable(advHandle);
  }
  // If we're not currently advertising
  else
  {
    if (numConn < MAX_NUM_BLE_CONNS)
    {
      // Start advertising since there is room for more connections
      GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
    }
    else
    {
      //Display_printf(dispHandle, MR_ROW_ADVERTIS, 0,"At Maximum Connection Limit, Cannot Enable Advertisment");
    }
  }

  return (true);
}


char * util_arrtohex(uint8_t const *src, uint8_t src_len,
                     uint8_t *dst, uint8_t dst_len, uint8_t reverse)
{
    char hex[] = "0123456789ABCDEF";
    uint8_t *pStr = dst;
    uint8_t avail = dst_len - 1;
    int8_t inc = 1;
    if(reverse)
    {
        src = src + src_len - 1;
        inc = -1;
    }

    memset(dst, 0, avail);

    while(src_len && avail > 3)
    {
        if(avail < dst_len - 1)
        {
            *pStr++ = ':';
            avail -= 1;
        }

        *pStr++ = hex[*src >> 4];
        *pStr++ = hex[*src & 0x0F];
        src += inc;
        avail -= 2;
        src_len--;
    }

    if(src_len && avail)
    {
        *pStr++ = ':'; // Indicate not all data fit on line.
    }
    return((char *)dst);
}

static void multi_role_timeSend(void) {

    GapAdv_disable(advHandle);
    GapAdv_disable(advHandleTime);
    //timePreAdv = Seconds_getTime(*ts);
    Seconds_getTime(&ts);
    timePreAdv = ts.secs;
    ntimePreAdv = ts.nsecs;

    char tempHexTime[9];
    char tempHexnTime[8];

    int increment = 0;
    count = 0;

    sprintf(tempHexTime, "%X", timePreAdv);
    sprintf(tempHexnTime, "%X", ntimePreAdv);

    printf("Time (hex): %s\n", tempHexTime);
    printf("nTime (hex): %s\n", tempHexnTime);

    GapAdv_prepareLoadByHandle(advHandleTime, GAP_ADV_FREE_OPTION_DONT_FREE);
    size_t tempSize = 2;

    //add each value to the advData
    for (int i = 13; i < 17; i++) {
        char tempChar[2];
        long int tempLong = 0;
        strncpy(tempChar, tempHexTime + increment, tempSize);
        //printf("AdvData[%d]: %s\n", i, tempChar);
        tempLong = strtol(tempChar, 0, 16);
        char toAdvData = tempLong;

        advData2[i] = toAdvData;
        increment = increment+2;
    }//end for loop


    GapAdv_loadByHandle(advHandleTime, GAP_ADV_DATA_TYPE_ADV, sizeof(advData2), advData2);

    Log_info1("Current Time: %d", timePreAdv);
    Log_info1("nanoseconds: %d", ntimePreAdv);
    timeServer = true;
    GapAdv_enable(advHandleTime, GAP_ADV_ENABLE_OPTIONS_USE_MAX_EVENTS, 2);


}//end multi_role_timeSend function

static void multi_role_tickSend (void){
    //function to be called when ticks need to be sent

    //ensure all other advertising sets are disabled
    GapAdv_disable(advHandle);
    GapAdv_disable(advHandleTime);
    GapAdv_disable(advHandleTicks);

    //Util_startClock(&clkTimeSync);

    //get preAdv clock time - as advertising starts
    ticksPreAdv = Clock_getTicks();
    Log_info1("ticksPreAdv %d", ticksPreAdv);

    tickServer = true;

    //enable advertising
    GapAdv_enable(advHandleTicks, GAP_ADV_ENABLE_OPTIONS_USE_MAX_EVENTS, 1);
    count = 0;


}//end multi_role_tickSend


//function to perfrom timestamp isolation

static void multi_role_timeIsolation(void) {

    //for the initial function assume the device broadcasting the timestamp is the initial device in scanList

    //isolate the manufacture data - look at making this a singular function call
    //manufacture data is isolated in the scanList structure

    //Util_startClock(&clkTimeSync);

    //variable to temporary hold the manufacturer data to be edited
    char tempData[30];
    strcpy(tempData, scanList[0].manuData);

    //remove the colons found in the received data
    //might be able to remove this command if you take pAdvRpt value
    Util_removeChar(tempData, ':');
    Log_info1("Removed colon: %s", (uintptr_t)tempData);

    printf("altered %s\n", tempData);

    //variables to hold the received timestamp and nanosecond delay value
    char receivedTimestamp [9];
    char rxDelay[7];
    size_t testvals = 8;

    //fill the variables accordingly
    strncpy(receivedTimestamp, tempData+4, testvals);
    strncpy(rxDelay, tempData+12, sizeof(rxDelay));

    //printf("rxDelay %s\n", rxDelay);
    printf("receivedTimestamp %s\n", receivedTimestamp);

    //convert from the hex value to decimal values
    uint32_t timeStamp = strtol(receivedTimestamp, 0, 16);
    uint32_t timeDelay = strtol(rxDelay, 0, 16);

    printf("timestamp value in dec: %d", timeStamp);

    char testPrint[50];
    memcpy(testPrint, receivedTimestamp, sizeof(receivedTimestamp)+1);
    testPrint[sizeof(receivedTimestamp)+1] = '\0';

    Log_info1("testPrint %s", (uintptr_t)testPrint);

    Log_info1("Received TimeStamp %d", timeStamp);
    Log_info1("Received Delay %d",timeDelay);


    //get current device time to compare delays

    //update Seconds struct containing received time and nsecs
    Seconds_getTime(&ts);

    //save current secs and nsecs
    timeDiffScan = ts.secs - timePreScan; //timediff values
    ntimeDiffScan = ts.nsecs - timePreScan;


    Log_info1("ntimePostScan %d", ts.nsecs);
    Log_info1("ntimePreScan %d", ntimePreScan);

    Log_info1("RX secs diff: %d", timeDiffScan);
    Log_info1("RX nsecs diff: %d", ntimeDiffScan);

    //combined tx and rx delay
    uint32_t combinedDelays = timeDelay + ntimeDiffScan;

    Log_info1("Combined delay: %d", combinedDelays);

    //set current time based on delays
    ts.secs = timeStamp + timeDiffScan;
    ts.nsecs = combinedDelays;
    Seconds_setTime(&ts);

    Log_info1("received timeStamp: %d", ts.secs);

}//end multi_role_timeIsolation function


static void multi_role_tickIsolation (void) {
    //new function to call when doing tick isolation for syncing

    int index = numScanRes;
    //need to isolate the incoming tick tx delay

    //variable to temporary hold the manufacturer data to be edited
    char tempData[10];
    strcpy(tempData, scanList[index].manuData);

    //remove the colon and '0' found in the received data
    Util_removeChar(tempData, ':');
    //Util_removeChar(tempData, '0');
    Log_info0("Removed colon");
    printf("removed colon value: %s\n", tempData);


    //isolate the txDelay value
    char txDelayChar[6];
    size_t txDelayCount = 4;
    //printf("txDelay isolation %s\n", txDelayChar);
    //Util_removeChar(tempData, '0');

    strncpy(txDelayChar, tempData+4, txDelayCount);
    printf("txDelay isolation: %s\n", txDelayChar);
    Util_removeChar(txDelayChar, '0');
    printf("txDelay remove 0: %s\n", txDelayChar);


    //add terminating value
    txDelayChar[5] = '\0';

    printf("after terminating value %s\n", txDelayChar);

    uint32_t txDelay = strtol(txDelayChar, 0, 16);

    Log_info1("Received TX Delay: %d", txDelay);
    printf("received tx delay: %d\n", txDelay);

    //tick RX delay calculation
    ticksPostScan = Clock_getTicks();
    uint32_t ticksDiffScan = ticksPostScan - ticksPreScan;

    //save the combined delay to a global variable to be used in the advertising phase
    combinedTickDelay = txDelay + ticksDiffScan;

    //log information for debugging purposes
    Log_info1("Combined Tick Delay: %d", combinedTickDelay);
    Log_info1("current clock ticks: %d", ticksPostScan);
    Log_info1("RX tick diff: %d", ticksDiffScan);


    //start clock
    uint32_t startingTimeClock = originalClockValue - combinedTickDelay;
    Log_info2("Starting Clock with adjustment (%d) to: %d",combinedTickDelay, startingTimeClock);

    //update the clock period accordingly
    Util_restartClock(&clkTimeSync, startingTimeClock);

    Log_info0("Begin advertising phase of device");
    multi_role_tickSend();

}//end multi_role_tickIsolation


/*********************************************************************
*********************************************************************/
