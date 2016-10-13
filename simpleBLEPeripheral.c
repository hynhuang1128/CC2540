
/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED ?AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_Clock.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
//#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

#include "npi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*********************************************************************
 * MACROS
 */
/*imperial version or metric version */
#define IMPERIAL_VERSION
//#define METRIC_VERSION

#define FIFO_SIZE 6
#define FIFOFULL 0
#define FIFOEmpty 1
#define FIFOR_OK 2
#define uint8_t unsigned char

#define ledSit P0_1
#define ledStand P0_4
#define ledBle P0_0
#define motorVab P1_5
#define onDuty P0_7
   
#define onSit 5
#define onStand 7
#define onLeave 1

#ifdef IMPERIAL_VERSION
#define VER_THERSHOLD_LOW 250
#define VER_THERSHOLD_HIGH 500
#define VER_HIGT_JUDGE 400
#else
#define VER_THERSHOLD_LOW 630
#define VER_THERSHOLD_HIGH 1270
#define VER_HIGT_JUDGE 900
#endif
/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                  30

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8//80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8//800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

/*********************************************************************
 * TYPEDEFS
 */

struct FIFOqueue{
    uint8_t front;       			//
    uint8_t rear;				//
    uint8_t count;				//
    uint8_t dat[FIFO_SIZE];		        //
};

struct nodesMap{                               //
    union {
        uint32 timeStamp;
        uint8 ts[4];
    };
    uint8 status;
}dataNode[200];

//typedef struct nodesMap *node;
//typedef struct FIFOqueue *Queue;

//node dataNode[200];

uint8 nodeCount = 0;

uint8 sendFlag = 0;

uint8 currentStatus = 0;
uint8 previousStatus = -1;

uint8 ble_connect_status = 0;

uint8 setValueDiffer = 17;
uint16 hightJudge_Value = 900;
uint16 hightHighThershold_Value = 1270;
uint16 hightLowThershold_Value = 630;
/*********************************************************************
 * GLOBAL VARIABLES
 */
void QueueInit(struct FIFOqueue *Queue);    //
uint8_t QueueIn(struct FIFOqueue *Queue , uint8_t sdata);  //
uint8_t QueueOut(struct FIFOqueue *Queue , uint8_t *sdata); //

/*
*  
*
**/
void QueueInit(struct FIFOqueue *Queue)
{
	Queue->rear = 0;
	Queue->front = Queue->rear;									//
	Queue->count = 0;									
	memset(Queue->dat, 0, sizeof(Queue->dat));
}
/*
*  
*
**/
uint8_t QueueIn(struct FIFOqueue *Queue,uint8_t sdata) //
{
	if((Queue->front == Queue->rear) && (Queue->count == FIFO_SIZE))
	{
            for(int i=0;i<Queue->count;i--){
                Queue->dat[i] = Queue->dat[i+1];
            }
            Queue->dat[Queue->count] = sdata;
            return FIFOFULL;
	}
	else
	{
		Queue->dat[Queue->rear] = sdata;
		Queue->rear = (Queue->rear + 1) % FIFO_SIZE;
		Queue->count += 1;
		return FIFOR_OK;
	}
}
/*
* 
*
**/
uint8_t QueueOut(struct FIFOqueue *Queue, uint8_t *sdata)
{
	if((Queue->front == Queue->rear) && (Queue->count == 0))
	{
		return FIFOEmpty;
	}
	else
	{
		*sdata = Queue->dat[Queue->front];                 //
		Queue->front = (Queue->front + 1) % FIFO_SIZE;
		Queue->count -= 1;
		return FIFOR_OK;
	}
}

struct FIFOqueue FIFO;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x6,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x39,//'9' 
   0x2d,

  0x41,//'A'
  0x2d,
  0x4D,//'M'
 


  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );

#if defined( CC2540_MINIDK )||( WEBEE_BOARD )
//static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
#endif

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

static void NpiSerialCallback( uint8 port, uint8 events );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */

 uint8 ble_Value[15];
 uint8 ble_num=0;
 uint8 ctrol_value[6];
 uint8 set_Flag=0; 
 uint32 higt_Value;
 uint32 set_Value;

uint8 timesVab = 0;
uint16 freqVab = 0;
uint8 vabStartFlag = 0;

uint8 startVabTime = 30;
uint8 endVabTime = 15;
uint8 vabCount = 15;

void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;
  //Npi Uart Init
  NPI_InitTransport(NpiSerialCallback);
  NPI_WriteTransport("Hello World\n",12);
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLEPeripheral_TaskID );

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4[SIMPLEPROFILE_CHAR4_LEN] = { 2, 3, 4, 5, 6};
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    uint8 charValue6[SIMPLEPROFILE_CHAR6_LEN] = { 0xaa, 0xbb, 0xc3, 0xd4, 0xe5 };
    uint8 charValue7[SIMPLEPROFILE_CHAR7_LEN] = { 1, 2, 3, 4, 5 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, SIMPLEPROFILE_CHAR6_LEN, charValue6 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR7, SIMPLEPROFILE_CHAR7_LEN, charValue7 );
  }


#if defined( CC2540_MINIDK )

  SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLEPeripheral_TaskID );

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  //P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  //P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  //P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

 // P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  //P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low

#endif // #if defined( CC2540_MINIDK )
  P0SEL = 0x6c;
  P0DIR |= 0x13;
  
  P1SEL = 0; // Configure Port 1 as GPIO
  P1DIR |= 0X2F;
  P1 = 0X0F;
#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  //HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
 // HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
 char flag_count = 4; 

uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

    return ( events ^ SBP_START_DEVICE_EVT );
  }


  
  if ( events & SBP_PERIODIC_EVT )// 30ms
  {
    flag_count++;
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }
      //             //ctrl+k or ctrl+shift+k
    if(set_Flag==1){
             if(set_Value > hightLowThershold_Value && set_Value < hightHighThershold_Value){
               if(higt_Value<set_Value-10)
             { P1 =0X0B;    
             }
             else if(higt_Value>(set_Value+10))
                     {P1 =0X07; //down
                     }
             else{
               P1 =0X0F;
               set_Flag=0;
             } 
             } 
             else{
               P1 =0X0F;} 
    }
    // Perform periodic application task
    performPeriodicTask();

    return (events ^ SBP_PERIODIC_EVT);
  }

#if defined ( PLUS_BROADCASTER )
  if ( events & SBP_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ SBP_ADV_IN_CONNECTION_EVT);
  }
#endif // PLUS_BROADCASTER

  
  
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  #if defined( CC2540_MINIDK )||( WEBEE_BOARD )
/*    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;*/
  #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}

#if defined( CC2540_MINIDK )||( WEBEE_BOARD )
/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
/*static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_1 )    //S1
  {
    NPI_WriteTransport("KEY K1\n",7);
  }
  
  if ( keys & HAL_KEY_SW_2 )    //S2
  {
    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
    }
  }
}*/
#endif // #if defined( CC2540_MINIDK )

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_CONNECTED:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          ble_connect_status = 1;
          ledBle = 0;
      }
      break;

    case GAPROLE_WAITING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          ble_connect_status = 0;
          ledStand = 1;
          ledSit = 1;
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

UTCTimeStruct *Ti;
uint32 timestamp = 0;
uint32 timeStampG = 0;
uint8 sendCount = 0;
/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */

static void performPeriodicTask( void )
{
  uint8 status_buf[7];
  //  uint8 valueToCopy;
  uint8 values[20]={1,2,3,4,5,6,7,8};
  //  uint8 stat;
  
  if(vabStartFlag){
  freqVab++;
  if(freqVab < 30){
      motorVab = 1;
  }else if(freqVab < 45){
      motorVab = 0;
  }else{
      freqVab = 0;
      timesVab++;
  }
  if(timesVab > 15){
      timesVab = 0;
      freqVab = 0;
      vabStartFlag = 0;
  }
}

  // Call to retrieve the value of the third characteristic in the profile
  //  stat = SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &valueToCopy);
  //判斷當前姿態是否有改變，改變的話將狀態和時間戳存入到緩存區內
//  if(ble_connect_status){
    if(!onDuty){
        if(higt_Value > hightJudge_Value){
          currentStatus = onStand;
          ledStand = 0;
          ledSit = 1;
        }else{
          currentStatus = onSit;
          ledStand = 1;
          ledSit = 0; 
        }
      }else{
        currentStatus = onLeave;
          ledStand = 1;
          ledSit = 1;
      }
//  }else{
//      ledBle = 1;
//  }
  if(previousStatus != currentStatus){
    previousStatus = currentStatus;
    //獲得當前的時間戳
    osalTimeUpdate();  
    timeStampG = osal_getClock();
    dataNode[nodeCount].timeStamp = timeStampG;
    dataNode[nodeCount].status = currentStatus;
    nodeCount++;
      status_buf[0]=0x06;
      status_buf[1]=0x55;
      status_buf[2]=0xaa;
      status_buf[3]=0xff;
      status_buf[4]=0x00;
      status_buf[5]=currentStatus;
      status_buf[6]=0xdd;
      SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR7,SIMPLEPROFILE_CHAR7_LEN,status_buf );
    //如果數據溢出
    if(nodeCount > 200){
      for(int i = 0; i<200 ; i++){
        dataNode[i] = dataNode[i+1];
      }
      nodeCount = 0;
    }
  }
/*  if(previousStatus != currentStatus){
    if(onDuty){
      if(higt_value > 900){
        currentStatus = onStand;
      }else{
        currentStatus = onSit
      }
    }else{
      currentStatus = onLeave;
    }
    previousStatus = currentStatus;
  } */
  //  if( stat == SUCCESS )
  //  {
    /*
     * Call to set that value of the fourth characteristic in the profile. Note
     * that if notifications of the fourth characteristic have been enabled by
     * a GATT client device, then a notification will be sent every time this
     * function is called.
     */
//數據發送標誌位，如果發送的話、直接把200個數據全部發送完。  
/*    osalTimeUpdate();  
    osal_ConvertUTCTime(Ti,osal_getClock());
    timestamp = osal_getClock();
    dataNode[0]->timeStamp = timestamp;
    dataNode[0]->status = onDuty;
    values[0]=0x55;
    values[1]=0xaa;
    values[2]=dataNode[0]->ts[0];
    values[3]=dataNode[0]->ts[1];
    values[4]=dataNode[0]->ts[2];
    values[5]=dataNode[0]->ts[3];
    values[6]=dataNode[0]->status;
    values[19]=0xdd;*/
  if(sendFlag){
    values[0] = 0x55;
    values[1] = 0xaa;
    values[2] = sendCount/3;     //包計數
    for(int i = 0 ; i < 15 ; i = i+5){
      values[i+3] = dataNode[sendCount].status;
      values[i+4] = dataNode[sendCount].ts[3];
      values[i+5] = dataNode[sendCount].ts[2];
      values[i+6] = dataNode[sendCount].ts[1];
      values[i+7] = dataNode[sendCount].ts[0];
      sendCount++;
    }
    values[19] = 0xdd;
    if(values[2] == 66){
      sendCount = 0;
      sendFlag = 0;
    }

    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, values);
    //  }  
  }
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */

static void simpleProfileChangeCB( uint8 paramID )
{  
  uint8 newValue;
  uint8 newChar6Value[32];
  uint8 char6cb[SIMPLEPROFILE_CHAR6_LEN];
  uint32 timestampc;
  QueueInit(&FIFO);
  
  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );
      NPI_WriteTransport(&newValue,1 );
      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;
    
    case SIMPLEPROFILE_CHAR6:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR6, newChar6Value );
      //HalLcdWriteStringValue( "Char 6:", (uint16)(newChar6Value[0]), 10,  HAL_LCD_LINE_3 );
      //NPI_WriteTransport(&newChar6Value[1],newChar6Value[0] );
     if( newChar6Value[0] > 95 )
      {
        //NPI_WriteTransport(&newChar6Value[1],14);
        //NPI_WriteTransport("...\n",4 ); 
      }
      else
      {
        //NPI_WriteTransport(&newChar6Value[1],6);
        
        for(uint8 i=0;i<newChar6Value[0];i++)
        {
          ble_num++; 
          QueueIn(&FIFO , newChar6Value[1+i]);
          //ble_Value[ble_num++]=newChar6Value[1+i];
        }
 
        if(ble_num == 6){
          ble_num = 0;
        for(uint8 i=0;i<7;i++)
        {
          QueueOut(&FIFO , &ble_Value[i]);
 //         //ble_Value[ble_num++]=newChar6Value[1+i];
        }
        
          NPI_WriteTransport(&ble_Value[0],6);
//        for(uint8 i=0;i<newChar6Value[0];i++)
//        {
        if(ble_Value[0]==0x55&&ble_Value[5]==0xdd)
        {
          if(ble_Value[2]==0x01)         //up
          {
            P1 =0X0B;
            set_Flag=0;
          }
           else if(ble_Value[2]==0x02)  //stop
           {P1 =0X0F; 
           set_Flag=0;
           set_Value=0;
           }
           else if(ble_Value[2]==0x03)  //down
           {
            P1 =0X07;
            set_Flag=0; 
           }
           else if(ble_Value[2]==0x04)  //set
           {
             set_Value=ble_Value[3]*256+ble_Value[4];
             set_Flag=1;
           }
           else if(ble_Value[2]==0x05)
           {
            char6cb[0]=0x55;
            char6cb[1]=0xaa;
            char6cb[2]=higt_Value/256;
            char6cb[3]=higt_Value%256;
            char6cb[4]=0x00;
            char6cb[5]=0xdd;
            SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6,SIMPLEPROFILE_CHAR6_LEN,char6cb ); 
           }
           else if(ble_Value[2]==0x06){
             if(ble_Value[3] != 0){
                 vabStartFlag = 1;
             }else{
                 timesVab = 0;
                 freqVab = 0;
                 motorVab = 0;
                 vabStartFlag = 0;
             }
            /*timestampc = ble_Value[3]*16777216+ble_Value[4]*65536+ble_Value[5]*256+ble_Value[6];
            osal_setClock( timestampc );
            char6cb[0]=timestampc%256;
            char6cb[1]=timestampc/256%256;
            char6cb[2]=timestampc/65536%256;
            char6cb[3]=timestampc/16777216%256;
            char6cb[4]=0;
            char6cb[5]=0xdd;
            SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6,SIMPLEPROFILE_CHAR6_LEN,char6cb ); */
           }
           else if(ble_Value[2]==0x07){
             sendFlag = 1; 
             osalTimeUpdate();
             timeStampG = osal_getClock();
             char6cb[0]=0x55;
             char6cb[4]=timeStampG%256;
             char6cb[3]=timeStampG/256%256;
             char6cb[2]=timeStampG/65536%256;
             char6cb[1]=timeStampG/16777216%256;
             char6cb[5]=0xdd;
             SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6,SIMPLEPROFILE_CHAR6_LEN,char6cb ); 
           }
           else if(ble_Value[2]==0x08){
             //reserved
           }
           else
           {P1 =0X0F; }
           SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6,SIMPLEPROFILE_CHAR6_LEN,char6cb ); 
//          }
        }
        }              
      }
      
      break;
      
    default:
      // should not reach here!
      break;
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

uint16 higt_Previous = 0;
uint8 higt_Change = 0;

static void NpiSerialCallback( uint8 port, uint8 events )
{
  (void)port;
  uint8 numBytes = 0;
  uint8 buf[128];
  uint8 ble_buf[7];
 
    if (events & HAL_UART_RX_TIMEOUT)   //??????????
    {
      numBytes = NPI_RxBufLen();       //??????????????ж??????
      if(numBytes==4)
      { 
        if ( numBytes >= SIMPLEPROFILE_CHAR7_LEN ) buf[0] = SIMPLEPROFILE_CHAR7_LEN-1;
        else buf[0] = numBytes;
        NPI_ReadTransport(&buf[1],buf[0]);    //????????????
        ble_buf[0]=0x06;
        ble_buf[1]=0x55;
        ble_buf[2]=0xaa;
        if(buf[2] != 0x01){
            ble_buf[3] = 0x01;
            ble_buf[3] |= (buf[2] << 4);
        }else{
            ble_buf[3] = 0x01;
        }
        ble_buf[4]=buf[3];
        ble_buf[5]=buf[4];
        ble_buf[6]=0xdd;
/*      if(higt_Previous != higt_Value){
        higt_change = 1;
        higt_Previous = higt_Value;
      }*/
        higt_Value=buf[3]*256+buf[4];
        if(flag_count > 3 && higt_Previous != higt_Value){
          SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR7,SIMPLEPROFILE_CHAR7_LEN,ble_buf );
          higt_Previous = higt_Value;
          flag_count = 0;
          //higt_change = 0;
        }
      
      }
  }

}


/*********************************************************************
*********************************************************************/