/******************************************************************************
  Filename:       TestRoutine.c
  Revised:        $Date: 2012-03-07 01:04:58 -0800 (Wed, 07 Mar 2012) $
  Revision:       $Revision: 29656 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller3
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
******************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 5 seconds.  The application will also
  receives "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "TestRoutine.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "hal_adc.h"

#include <stdio.h>

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif  

/* PWM */
#include "../usr_api/pwm.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t TestRoutine_ClusterList[TestRoutine_MAX_CLUSTERS] =
{
  TestRoutine_CLUSTERID
};

const SimpleDescriptionFormat_t TestRoutine_SimpleDesc =
{
  TestRoutine_ENDPOINT,              //  int Endpoint;
  TestRoutine_PROFID,                //  uint16 AppProfId[2];
  TestRoutine_DEVICEID,              //  uint16 AppDeviceId[2];
  TestRoutine_DEVICE_VERSION,        //  int   AppDevVer:4;
  TestRoutine_FLAGS,                 //  int   AppFlags:4;
  TestRoutine_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)TestRoutine_ClusterList,  //  byte *pAppInClusterList;
  TestRoutine_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)TestRoutine_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in TestRoutine_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t TestRoutine_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte TestRoutine_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // TestRoutine_Init() is called.
devStates_t TestRoutine_NwkState;


byte TestRoutine_TransID;  // This is the unique message ID (counter)

afAddrType_t TestRoutine_DstAddr;

static uint8  finishFlag = 0;  //器件初始化完成標誌 

enum pwmDir{ pwmInactive, pwmIncreasing, pwmDecreasing };
byte pwmDirect = pwmInactive;

uint16 adcVal0=0;
uint16 adcVal1=0;
uint16 adcVal0Prev = 0xFFFF;
uint16 adcVal1Prev = 0xFFFF;

byte    ledToggle=0;
byte    speedDir=0;
uint16  speed=20;
uint16  speedAbs;
byte    vdir=2;
byte    JoyStickOp = 0;
byte    motor_state = 0;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void TestRoutine_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void TestRoutine_HandleKeys( byte shift, byte keys );
static void TestRoutine_MessageMSGCB( afIncomingMSGPacket_t *pckt );
//聲明發送信息函數
static void TestRoutine_SendKeyCommand(uint8 index);
static void TestRoutine_SendTheMessage(byte* Data, uint16 Len);

#if defined( IAR_ARMCM3_LM )
static void TestRoutine_ProcessRtosMessage( void );
#endif

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      TestRoutine_Init
 *
 * @brief   Initialization function for the Generic App Task.
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

static void AppUartInit(void)
{
  halUARTCfg_t uartConfig;

  /* UART Configuration */
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = HAL_UART_BR_9600;
  uartConfig.flowControl          = HAL_UART_FLOW_OFF;
  uartConfig.flowControlThreshold = 64;
  uartConfig.rx.maxBufSize        = 128;
  uartConfig.tx.maxBufSize        = 128;
  uartConfig.idleTimeout          = 6;
  uartConfig.intEnable            = TRUE;

  uartConfig.callBackFunc         = NULL;   //這裡不用回調

  HalUARTOpen (HAL_UART_PORT_0, &uartConfig);
#if (LCD_DISPLAY_DETAIL == TRUE)
  HalLcdBufferAdd("Uart0 init");
#endif   
}

void TestRoutine_Init( uint8 task_id )
{
  TestRoutine_TaskID = task_id;
  TestRoutine_NwkState = DEV_INIT;
  TestRoutine_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  
  TestRoutine_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  TestRoutine_DstAddr.endPoint = 0;
  TestRoutine_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  TestRoutine_epDesc.endPoint = TestRoutine_ENDPOINT;
  TestRoutine_epDesc.task_id = &TestRoutine_TaskID;
  TestRoutine_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&TestRoutine_SimpleDesc;
  TestRoutine_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &TestRoutine_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( TestRoutine_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "TestRoutine", HAL_LCD_LINE_1 );
#endif

  ZDO_RegisterForZDOMsg( TestRoutine_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( TestRoutine_TaskID, Match_Desc_rsp );

#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiatr
  RTOS_RegisterApp( task_id, TestRoutine_RTOS_MSG_EVT );
#endif
  AppUartInit();
  
  osal_start_reload_timer( TestRoutine_TaskID, TestRoutine_ADCReadTimer, 300);
}

#define MOTOR_DISABLED  0
#define MOTOR_STANDBY   1
#define MOTOR_FORWARD   2
#define MOTOR_BACKWARD  3
#define MOTOR_BREAK     4

#define VDIR_FORWARD    1
#define VDIR_BACKWARD   0

#define motor_forward()  { vdir = VDIR_FORWARD; st(P0_6 = 1;); }
#define motor_backward() { vdir = VDIR_BACKWARD; st(P0_6 = 0;); }

#define motor_enable()   { motor_state = MOTOR_STANDBY; }
#define motor_disable()  { motor_state = MOTOR_DISABLED; }

#define VDIR_PIN    P0_6
#define BREAK_PIN   P0_5

#define JOYSTICK_UP    0
#define JOYSTICK_DOWN  1

#define motor_break()   { motor_state = MOTOR_BREAK; st(P0_5 = 0;); }
#define motor_unbreak() { motor_state = MOTOR_STANDBY; st(P0_5 = 1;); }

#define MIN_PWM_SPEED    10
#define MAX_PWM_SPEED   600



/*********************************************************************
 * @fn      TestRoutine_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 TestRoutine_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;
  byte adcBuff[10] = {'A', 'D', 'C', 0, 0, 0, 0, 0};

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( TestRoutine_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          TestRoutine_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          TestRoutine_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          TestRoutine_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          TestRoutine_NwkState = (devStates_t)(MSGpkt->hdr.status);
          //只有終端設備才可以有發送的權力
          if ((TestRoutine_NwkState == DEV_END_DEVICE)) {          
            finishFlag = 1;
            HalLcdWriteString("Key up = Send Data",HAL_LCD_LINE_8);
          }
          else if (TestRoutine_NwkState == DEV_NWK_DISC) {
            HalLcdWriteString("Disc. PAN..",HAL_LCD_LINE_8);
          }
          else if (TestRoutine_NwkState == DEV_COORD_STARTING) {          
            HalLcdWriteString("Waiting req..",HAL_LCD_LINE_8);      
            motor_init();
          }
          else {
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( TestRoutine_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }    
  // Send a message out - This event is generated by a timer
  //  (setup in TestRoutine_Init()).
  
  static const int ADC_SEND_INTERVAL = 1;
  
  if (TestRoutine_NwkState == DEV_END_DEVICE) {
    /* Start t*/
    
    if (events & TestRoutine_ADCReadTimer) {
      // Start and read ADC value
      P0DIR &= ~0xa0;
      HalAdcInit(); 
      adcVal0 = HalAdcRead(HAL_ADC_CHANNEL_5, HAL_ADC_RESOLUTION_12);
      adcVal1 = HalAdcRead(HAL_ADC_CHANNEL_7, HAL_ADC_RESOLUTION_12);         
            
      if (true/*((adcVal0Prev > adcVal0) && (adcVal0Prev - adcVal0) >= ADC_SEND_INTERVAL) ||
          ((adcVal0Prev < adcVal0) && (adcVal0 - adcVal0Prev) >= ADC_SEND_INTERVAL) ||
          ((adcVal1Prev > adcVal1) && (adcVal1Prev - adcVal1) >= ADC_SEND_INTERVAL) ||
          ((adcVal1Prev < adcVal1) && (adcVal1 - adcVal1Prev) >= ADC_SEND_INTERVAL)*/) {
                
        adcVal0Prev = adcVal0;
        adcVal1Prev = adcVal1;
        /* Send message to the coordinator */
        HalLedBlink(HAL_LED_1, 1, 60, 600);      
        adcBuff[4] = adcVal0 >> 8;
        adcBuff[5] = (byte)adcVal0;
        adcBuff[6] = adcVal1 >> 8;
        adcBuff[7] = (byte)adcVal1;      
        
        if (DEV_END_DEVICE == TestRoutine_NwkState) {
          TestRoutine_SendTheMessage(adcBuff, sizeof(adcBuff));
        }        
        ////else if (DEV_ZB_COORD == TestRoutine_NwkState) {               
          ////HalLcdWriteStringValue( "adc value 0: ", adcVal0, 10, HAL_LCD_LINE_7 );        
          ////HalLcdWriteStringValue( "adc value 1: ", adcVal1, 10, HAL_LCD_LINE_8 );        
        ////}
        
      }
    }
 }  
  
#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & TestRoutine_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    TestRoutine_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ TestRoutine_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}


void motor_init(void)
{
  P0DIR |= 0X20;
  P0SEL &= ~0x20;
  P0DIR |= 0X40;
  P0SEL &= ~0x40;   
 
  pwmPulse(MIN_PWM_SPEED);
  speed = MIN_PWM_SPEED;
  
  motor_forward();
  motor_break();
  motor_disable(); 
}
/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      TestRoutine_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void TestRoutine_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            TestRoutine_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            TestRoutine_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            TestRoutine_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      TestRoutine_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void TestRoutine_HandleKeys( uint8 shift, uint8 keys )
{
  // Shift is used to make each button/switch dual purpose.
  uint8 keyId = 0;
  
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      keyId = 4;
    }
    if ( keys & HAL_KEY_SW_2 )
    {
       keyId = 5;
    }
    if ( keys & HAL_KEY_SW_3 )
    {
      keyId = 6;
    }
    if ( keys & HAL_KEY_SW_4 )
    {
      keyId = 7;
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      keyId = 0;
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      keyId = 1;
    }

    if ( keys & HAL_KEY_SW_3 )
    {
      keyId = 2;
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      keyId = 3;
    }
  }
  
  //用於指示已經觸發發送信息      
  if( finishFlag > 0 )
  {
    TestRoutine_SendKeyCommand( keyId );
    HalLedBlink(HAL_LED_1, 1, 60, 600);
  }  
}

void change_motor_speed(uint16 Ref)
{
  uint8 increase = 0;
  Ref = (Ref >> 4) - 10;   // Convert adc to about 0 ~ 54 increament
  
  if (VDIR_FORWARD == vdir) {
    if (JoyStickOp == JOYSTICK_UP) {
      increase = 1;
    }
    else {
      increase = 0;
    }
  }
  else {
    if (JoyStickOp == JOYSTICK_UP) {
      increase = 1;      
    }
    else {
      increase = 0;
    }
  }  
  if (increase) {
    if ((speed + Ref) <= MAX_PWM_SPEED) {
      speed += Ref;
    }
  }
  else {
    if ((speed >= (MIN_PWM_SPEED + Ref))) {
      speed -= Ref;
    }
  }
  setFreq(speed);
  
  HalLcdWriteStringValue( "speed: ", speed, 10, HAL_LCD_LINE_5 );     
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      TestRoutine_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
#define abs_sub(x, y) (x > y ? (x - y) : (y - x))

static void TestRoutine_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint16 adc_middle_dist;
  
  switch ( pkt->clusterId )
  {
    case TestRoutine_CLUSTERID:
      // "the" message
      ////HalLedBlink(HAL_LED_2,1,40,500);
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );         
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      if (is_data(pkt->cmd.Data, "ADC")) {     
        HalLcdWriteString((char*)pkt->cmd.Data, HAL_LCD_LINE_8);
        adcVal0 = ((uint16)pkt->cmd.Data[4] << 8) + pkt->cmd.Data[5];
        adcVal1 = ((uint16)pkt->cmd.Data[6] << 8) + pkt->cmd.Data[7];
        // PWM Control
        P0DIR |= 0X20;
        P0SEL &= ~0x20;
        P0DIR |= 0X40;
        P0SEL &= ~0x40;        
        
        adc_middle_dist = abs_sub(adcVal1, 998);
        
        if (adc_middle_dist >= 180) 
        {          
          if (adcVal1 >= 1118) {
            if (motor_state != MOTOR_BACKWARD && motor_state == MOTOR_FORWARD) {
              // Wake up and go front
              motor_enable();
              motor_unbreak();
              motor_forward();
            }
            change_motor_speed(adc_middle_dist);            
          }
          else if (adcVal1 <= 878) {
            // Decrease or increase speed
            if (motor_state != MOTOR_BACKWARD && motor_state != MOTOR_FORWARD) {
              // Wake up and go front
              motor_enable();
              motor_unbreak();              
              motor_backward();
            }
            change_motor_speed(adc_middle_dist);
          }                     
        }
        else {
          /* The joystick adc value is too small, do nothing */
        }
        
        HalLcdWriteStringValue( "pwm: ", adc_middle_dist, 10, HAL_LCD_LINE_6 );                     
        
        HalLcdWriteStringValue( "adc value 0: ", adcVal0, 10, HAL_LCD_LINE_7 );
        HalLcdWriteStringValue( "adc value 1: ", adcVal1, 10, HAL_LCD_LINE_8 );
      }
      if (is_data(pkt->cmd.Data, "Key1")) {
        motor_break();
      }
      
      if (is_data(pkt->cmd.Data, "S-Key1")) {
        if (pwmDirect == pwmDecreasing) {
          ////osal_stop_timerEx(TestRoutine_TaskID, TestRoutine_DecreasePWM_EVT);
          HalUARTWrite(HAL_UART_PORT_0, "Stop", sizeof("Stop"));          
        }                   
        ////osal_start_reload_timer( TestRoutine_TaskID, TestRoutine_IncreasePWM_EVT, 32); 
        HalLcdWriteString("Accelerating.. ", HAL_LCD_LINE_8);  
        HalUARTWrite(HAL_UART_PORT_0, "Inc", sizeof("Inc"));
        pwmDirect = pwmIncreasing;
      }      
      else if (is_data(pkt->cmd.Data, "S-Key3")) {
        if (pwmDirect == pwmIncreasing) {
          ////osal_stop_timerEx(TestRoutine_TaskID, TestRoutine_IncreasePWM_EVT);
          HalUARTWrite(HAL_UART_PORT_0, "Stop", sizeof("Stop"));            
        }          
        HalLcdWriteString("Decelerating.. ", HAL_LCD_LINE_8);  
        HalUARTWrite(HAL_UART_PORT_0, "Dec", sizeof("Dec"));
        ////osal_start_reload_timer( TestRoutine_TaskID, TestRoutine_DecreasePWM_EVT, 32);
        pwmDirect = pwmDecreasing;
      }      
      else {
        if (pwmDirect == pwmIncreasing) {
          ////osal_stop_timerEx(TestRoutine_TaskID, TestRoutine_IncreasePWM_EVT);
          HalUARTWrite(HAL_UART_PORT_0, "Stop", sizeof("Stop"));
          HalLcdWriteString("Stopped Increasing",HAL_LCD_LINE_8);          
        }
        else if (pwmDirect == pwmDecreasing) {
          ////osal_stop_timerEx(TestRoutine_TaskID, TestRoutine_DecreasePWM_EVT);
          HalUARTWrite(HAL_UART_PORT_0, "Stop", sizeof("Stop"));
          HalLcdWriteString("Stopped Decreasing",HAL_LCD_LINE_8);  
        }
        else {
        }
        pwmDirect = pwmInactive;
      }         
      break;
  }
}

/*********************************************************************
 * @fn      TestRoutine_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */

/*
 *  Send string message via uni cast with 16bit address.
 */
static void TestRoutine_SendTheMessage(byte* Data, uint16 Len)
{
  //設置為16位網絡地址，即單播模式
  TestRoutine_DstAddr.addrMode = afAddr16Bit;
  //設置目的地址
  TestRoutine_DstAddr.addr.shortAddr = 0x0000;               //ox0000為協調器網絡地址
  TestRoutine_DstAddr.endPoint = TestRoutine_ENDPOINT;       //遠程設備端點號

  if ( AF_DataRequest( &TestRoutine_DstAddr, &TestRoutine_epDesc,
                       TestRoutine_CLUSTERID,
                       Len,
                       Data,
                       &TestRoutine_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS)
  {
    // Successfully requested to be sent.
    HalLedBlink(HAL_LED_2, 1, 40, 500);
  }
  else
  {
    // Error occurred in request to send.
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_FLASH );
  }

}

static void TestRoutine_SendKeyCommand( uint8 index )
{

  char *theMessageData[8] = {"Key1","Key2", "Key3", "Key4", 
                             "S-Key1","S-Key2", "S-Key3", "S-Key4"};
  
  TestRoutine_SendTheMessage((byte*)theMessageData[index], 
                             osal_strlen(theMessageData[index]) + 1);
}


#if defined( IAR_ARMCM3_LM )
/*********************************************************************
 * @fn      TestRoutine_ProcessRtosMessage
 *
 * @brief   Receive message from RTOS queue, send response back.
 *
 * @param   none
 *
 * @return  none
 */
static void TestRoutine_ProcessRtosMessage( void )
{
  osalQueue_t inMsg;

  if ( osal_queue_receive( OsalQueue, &inMsg, 0 ) == pdPASS )
  {
    uint8 cmndId = inMsg.cmnd;
    uint32 counter = osal_build_uint32( inMsg.cbuf, 4 );

    switch ( cmndId )
    {
      case CMD_INCR:
        counter += 1;  /* Increment the incoming counter */
                       /* Intentionally fall through next case */

      case CMD_ECHO:
      {
        userQueue_t outMsg;

        outMsg.resp = RSP_CODE | cmndId;  /* Response ID */
        osal_buffer_uint32( outMsg.rbuf, counter );    /* Increment counter */
        osal_queue_send( UserQueue1, &outMsg, 0 );  /* Send back to UserTask */
        break;
      }
      
      default:
        break;  /* Ignore unknown command */    
    }
  }
}
#endif

/*********************************************************************
 */
