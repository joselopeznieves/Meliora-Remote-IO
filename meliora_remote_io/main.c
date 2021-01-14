// Standard includes
#include <stdlib.h>

// simplelink includes
#include "simplelink.h"


// driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "interrupt.h"
#include "rom_map.h"
#include "prcm.h"
#include "uart.h"
#include "timer.h"

// common interface includes
#include "network_if.h"
#ifndef NOTERM
#include "uart_if.h"
#endif

#include "button_if.h"
#include "gpio_if.h"
#include "timer_if.h"
#include "common.h"
#include "utils.h"


#include "sl_mqtt_client.h"

// application specific includes
#include "pin_mux.h"

#include "modbus.h"

//#include "channel_interface.h"

typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    SOCKET_CREATE_ERROR = -0x7D0,
    BIND_ERROR = SOCKET_CREATE_ERROR - 1,
    LISTEN_ERROR = BIND_ERROR -1,
    SOCKET_OPT_ERROR = LISTEN_ERROR -1,
    CONNECT_ERROR = SOCKET_OPT_ERROR -1,
    ACCEPT_ERROR = CONNECT_ERROR - 1,
    SEND_ERROR = ACCEPT_ERROR -1,
    RECV_ERROR = SEND_ERROR -1,
    SOCKET_CLOSE_ERROR = RECV_ERROR -1,
    DEVICE_NOT_IN_STATION_MODE = SOCKET_CLOSE_ERROR - 1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

#define APPLICATION_VERSION 	"1.4.0"

/*Operate Lib in MQTT 3.1 mode.*/
#define MQTT_3_1_1              false /*MQTT 3.1.1 */
#define MQTT_3_1                true /*MQTT 3.1*/

#define WILL_TOPIC              "Client"
#define WILL_MSG                "Client Stopped"
#define WILL_QOS                QOS2
#define WILL_RETAIN             false

/*Defining Broker IP address and port Number*/
#define SERVER_ADDRESS           "mqtt.eclipseprojects.io"
#define SERVER_IP_ADDRESS        "192.168.178.67"
#define PORT_NUMBER              1883
#define SECURED_PORT_NUMBER      8883
#define LOOPBACK_PORT            1882

#define MAX_BROKER_CONN         1

#define SERVER_MODE             MQTT_3_1
/*Specifying Receive time out for the Receive task*/
#define RCV_TIMEOUT             30

/*Background receive task priority*/
#define TASK_PRIORITY           3

/* Keep Alive Timer value*/
#define KEEP_ALIVE_TIMER        25

/*Clean session flag*/
#define CLEAN_SESSION           true

/*Retain Flag. Used in publish message. */
#define RETAIN                  1

/*Defining Publish Topic*/
#define PUB_TOPIC_AI              "/cc3200/Meliora/vai"
#define PUB_TOPIC_AO              "/cc3200/Meliora/vao"

/*Defining Number of topics*/
#define TOPIC_COUNT             10

/*Defining Subscription Topic Values*/
#define TOPIC_DI                  "/cc3200/Meliora/di"
#define TOPIC_DO                  "/cc3200/Meliora/do"
#define TOPIC_AI                  "/cc3200/Meliora/ai"
#define TOPIC_AO                  "/cc3200/Meliora/ao"
#define TOPIC_AI_AS               "/cc3200/Meliora/ai/autoscalling"
#define TOPIC_AI_SI               "/cc3200/Meliora/ai/slopeintercept"
#define TOPIC_AO_SI               "/cc3200/Meliora/ao/slopeintercept"
#define TOPIC_UDMA                "/cc3200/Meliora/user"
#define TOPIC_AI_FLAG             "/cc3200/Meliora/flagvai"
#define TOPIC_AO_FLAG             "/cc3200/Meliora/flagvao"
char* const CHANNELS[4] = {TOPIC_DI, TOPIC_DO, TOPIC_AI, TOPIC_AO};
char* const AS_SI[3] = {TOPIC_AI_AS, TOPIC_AI_SI, TOPIC_AO_SI};
char* const FLAG[3] = {TOPIC_UDMA, TOPIC_AI_FLAG, TOPIC_AO_FLAG};

/*Defining QOS levels*/
#define QOS0                    0
#define QOS1                    1
#define QOS2                    2

/*Spawn task priority and OSI Stack Size*/
#define OSI_STACK_SIZE          2048

typedef struct connection_config{
    SlMqttClientCtxCfg_t broker_config;
    void *clt_ctx;
    unsigned char *client_id;
    unsigned char *usr_name;
    unsigned char *usr_pwd;
    bool is_clean;
    unsigned int keep_alive_time;
    SlMqttClientCbs_t CallBAcks;
    int num_topics;
    char *topic[TOPIC_COUNT];
    unsigned char qos[TOPIC_COUNT];
    SlMqttWill_t will_params;
    bool is_connected;
}connect_config;

typedef enum
{
    BROKER_DISCONNECTION,
    ANALOG_INPUTS,
    ANALOG_OUTPUTS
}events;

typedef struct
{
	void * hndl;
	events event;
}event_msg;

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void
Mqtt_Recv(void *app_hndl, const char  *topstr, long top_len, const void *payload,
          long pay_len, bool dup,unsigned char qos, bool retain);
static void sl_MqttEvt(void *app_hndl,long evt, const void *buf,
                       unsigned long len);
static void sl_MqttDisconnect(void *app_hndl);
void maskChannels(int* masks, char* cmd);
void BoardInit(void);
void MqttClient(void *pvParameters);
void ModbusTask( void *pvParameters);
void ConnectToAP(void *pvParameters);
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#ifdef USE_FREERTOS
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#endif

unsigned short g_usTimerInts;
/* AP Security Parameters */
SlSecParams_t SecurityParams = {0};

/*Message Queue*/
OsiMsgQ_t g_PBQueue;

/* connection configuration */
connect_config usr_connect_config[] =
{
    {
        {
            {
                SL_MQTT_NETCONN_URL,
                SERVER_ADDRESS,
                PORT_NUMBER,
                0,
                0,
                0,
                NULL
            },
            SERVER_MODE,
            true,
        },
        NULL,
        "user1",
        NULL,
        NULL,
        true,
        KEEP_ALIVE_TIMER,
        {Mqtt_Recv, sl_MqttEvt, sl_MqttDisconnect},
        TOPIC_COUNT,
        {TOPIC_DI, TOPIC_DO, TOPIC_AI, TOPIC_AO, TOPIC_AI_AS, TOPIC_AI_SI, TOPIC_AO_SI, TOPIC_UDMA, TOPIC_AI_FLAG, TOPIC_AO_FLAG},
        {QOS2, QOS2, QOS2, QOS2, QOS2, QOS2, QOS2, QOS2, QOS2, QOS2},
        {WILL_TOPIC,WILL_MSG,WILL_QOS,WILL_RETAIN},
        false
    }
};

/* library configuration */
SlMqttClientLibCfg_t Mqtt_Client={
    0,
    TASK_PRIORITY,
    30,
    true,
    (long(*)(const char *, ...))UART_PRINT
};


void *app_hndl = (void*)usr_connect_config;

int coil[4] = {0,0,0,0};
int discrete[4] = {0,0,0,0};
int holding[4] = {0,0,0,0};
int input[4] = {0,0,0,0};

OsiSyncObj_t sync_obj;
OsiTaskHandle handle;

extern volatile unsigned long g_ulStatus;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//****************************************************************************
//! Defines Mqtt_Pub_Message_Receive event handler.
//! Client App needs to register this event handler with sl_ExtLib_mqtt_Init 
//! API. Background receive task invokes this handler whenever MQTT Client 
//! receives a Publish Message from the broker.
//!
//!\param[out]     topstr => pointer to topic of the message
//!\param[out]     top_len => topic length
//!\param[out]     payload => pointer to payload
//!\param[out]     pay_len => payload length
//!\param[out]     retain => Tells whether its a Retained message or not
//!\param[out]     dup => Tells whether its a duplicate message or not
//!\param[out]     qos => Tells the Qos level
//!
//!\return none
//****************************************************************************
static void
Mqtt_Recv(void *app_hndl, const char  *topstr, long top_len, const void *payload,
                       long pay_len, bool dup,unsigned char qos, bool retain)
{

    char *output_str=(char*)malloc(top_len+1);
    memset(output_str,'\0',top_len+1);
    strncpy(output_str, (char*)topstr, top_len);
    output_str[top_len]='\0';


    if(strncmp(output_str,TOPIC_DO, top_len) == 0)
    {
        maskChannels(coil, (char*) payload);
    }
    else if(strncmp(output_str,TOPIC_DI, top_len) == 0)
    {
        maskChannels(discrete, (char*) payload);
    }
    else if(strncmp(output_str,TOPIC_AO, top_len) == 0)
    {
        maskChannels(holding, (char*) payload);
    }
    else if(strncmp(output_str,TOPIC_AI, top_len) == 0)
    {
        maskChannels(input, (char*) payload);
    }
    else if((strncmp(output_str,TOPIC_AI_AS, top_len) == 0))
    {
        saveAutoScaling((char*) payload);
    }
    else if(strncmp(output_str,TOPIC_AO_SI, top_len) == 0)
    {

    }
    else if(strncmp(output_str,TOPIC_AI_SI, top_len) == 0)
    {

    }
    else if(strncmp(output_str, TOPIC_UDMA, top_len) == 0)
    {
        save_udma((char*) payload);
    }
    else if(strncmp(output_str,TOPIC_AO_FLAG, top_len) == 0)
    {
        if(strncmp((char*) payload, "1", 1) == 0) {
            event_msg msg;

            msg.event = ANALOG_OUTPUTS;
            msg.hndl = NULL;
            //
            // write message indicating publish message
            //
            osi_MsgQWrite(&g_PBQueue,&msg,OSI_NO_WAIT);
        }
    }
//    else if(strncmp(output_str,TOPIC_AI_FLAG, top_len) == 0)
//    {
//        if(strncmp((char*) payload, "1", 1) == 0) {
//            event_msg msg;
//
//            msg.event = ANALOG_INPUTS;
//            msg.hndl = NULL;
//            //
//            // write message indicating publish message
//            //
//            osi_MsgQWrite(&g_PBQueue,&msg,OSI_NO_WAIT);
//        }
//    }

    readMask(coil, discrete, holding, input);

    free(output_str);

    return;
}

//****************************************************************************
//! Defines sl_MqttEvt event handler.
//! Client App needs to register this event handler with sl_ExtLib_mqtt_Init
//! API. Background receive task invokes this handler whenever MQTT Client
//! receives an ack(whenever user is in non-blocking mode) or encounters an error.
//!
//!
//! \return none
//****************************************************************************
static void
sl_MqttEvt(void *app_hndl, long evt, const void *buf,unsigned long len)
{
    switch(evt)
    {
      case SL_MQTT_CL_EVT_PUBACK:
        break;

      case SL_MQTT_CL_EVT_SUBACK:
        break;

      case SL_MQTT_CL_EVT_UNSUBACK:
        break;

      default:
        break;

    }
}

//****************************************************************************
//
//! callback event in case of MQTT disconnection
//!
//! \param app_hndl is the handle for the disconnected connection
//!
//! return none
//
//****************************************************************************
static void
sl_MqttDisconnect(void *app_hndl)
{
    connect_config *local_con_conf;
    event_msg msg;
    local_con_conf = app_hndl;
    msg.hndl = app_hndl;
    msg.event = BROKER_DISCONNECTION;

    local_con_conf->is_connected = false;
    //
    // write message indicating publish message
    //
    osi_MsgQWrite(&g_PBQueue,&msg,OSI_NO_WAIT);

}

void maskChannels(int* masks, char* cmd)
{
    int i;
    for(i = 3; i >=0; i--) {
        masks[i] = cmd[3-i] - '0';
    }
}


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void BoardInit(void)
{
    /* In case of TI-RTOS vector table is initialize by OS itself */
    #ifndef USE_TIRTOS
    //
    // Set vector table base
    //
    #if defined(ccs)
        IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    #endif
    #if defined(ewarm)
        IntVTableBaseSet((unsigned long)&__vector_table);
    #endif
    #endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}


//*****************************************************************************
//
//! Task implementing MQTT client communication to other web client through
//!    a broker
//!
//! \param  none
//!
//! This function
//!    1. Initializes network driver and connects to the default AP
//!    2. Initializes the mqtt library and set up MQTT connection configurations
//!    3. set up the button events and their callbacks(for publishing)
//!    4. handles the callback signals
//!
//! \return None
//!
//*****************************************************************************
void MqttClient(void *pvParameters)
{
    osi_SyncObjWait(&sync_obj, OSI_WAIT_FOREVER);
    osi_SyncObjSignal(&sync_obj);
    long lRetVal = -1;
    int iCount = 0;
    int iNumBroker = 0;
    int iConnBroker = 0;
    event_msg RecvQue;

    connect_config *local_con_conf = (connect_config *)app_hndl;
    //
    // Register Push Button Handlers
    //
//    Button_IF_Init(pushButtonInterruptHandler2,pushButtonInterruptHandler3);
    
    //
    // Initialze MQTT client lib
    //
    lRetVal = sl_ExtLib_MqttClientInit(&Mqtt_Client);
    if(lRetVal != 0)
    {
        // lib initialization failed
        LOOP_FOREVER();
    }
    
    /******************* connection to the broker ***************************/
    iNumBroker = sizeof(usr_connect_config)/sizeof(connect_config);
    if(iNumBroker > MAX_BROKER_CONN)
    {
        LOOP_FOREVER();
    }

connect_to_broker:
    while(iCount < iNumBroker)
    {
        //create client context
        local_con_conf[iCount].clt_ctx =
        sl_ExtLib_MqttClientCtxCreate(&local_con_conf[iCount].broker_config,
                                      &local_con_conf[iCount].CallBAcks,
                                      &(local_con_conf[iCount]));

        //
        // Set Client ID
        //
        sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                            SL_MQTT_PARAM_CLIENT_ID,
                            local_con_conf[iCount].client_id,
                            strlen((char*)(local_con_conf[iCount].client_id)));

        //
        // Set will Params
        //
        if(local_con_conf[iCount].will_params.will_topic != NULL)
        {
            sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                                    SL_MQTT_PARAM_WILL_PARAM,
                                    &(local_con_conf[iCount].will_params),
                                    sizeof(SlMqttWill_t));
        }

        //
        // setting username and password
        //
        if(local_con_conf[iCount].usr_name != NULL)
        {
            sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                                SL_MQTT_PARAM_USER_NAME,
                                local_con_conf[iCount].usr_name,
                                strlen((char*)local_con_conf[iCount].usr_name));

            if(local_con_conf[iCount].usr_pwd != NULL)
            {
                sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                                SL_MQTT_PARAM_PASS_WORD,
                                local_con_conf[iCount].usr_pwd,
                                strlen((char*)local_con_conf[iCount].usr_pwd));
            }
        }

        //
        // connectin to the broker
        //
        if((sl_ExtLib_MqttClientConnect((void*)local_con_conf[iCount].clt_ctx,
                            local_con_conf[iCount].is_clean,
                            local_con_conf[iCount].keep_alive_time) & 0xFF) != 0)
        {
            
            //delete the context for this connection
            sl_ExtLib_MqttClientCtxDelete(local_con_conf[iCount].clt_ctx);
            
            break;
        }
        else
        {
            local_con_conf[iCount].is_connected = true;
            iConnBroker++;
        }

        //
        // Subscribe to topics
        //

        if(sl_ExtLib_MqttClientSub((void*)local_con_conf[iCount].clt_ctx,
                                   CHANNELS,
                                   local_con_conf[iCount].qos, 4) < 0)
        {
            sl_ExtLib_MqttClientDisconnect(local_con_conf[iCount].clt_ctx);
            local_con_conf[iCount].is_connected = false;

            //delete the context for this connection
            sl_ExtLib_MqttClientCtxDelete(local_con_conf[iCount].clt_ctx);
            iConnBroker--;
            break;
        }
        else if(sl_ExtLib_MqttClientSub((void*)local_con_conf[iCount].clt_ctx,
                                   AS_SI,
                                   local_con_conf[iCount].qos, 3) < 0)
        {

            sl_ExtLib_MqttClientDisconnect(local_con_conf[iCount].clt_ctx);
            local_con_conf[iCount].is_connected = false;

            //delete the context for this connection
            sl_ExtLib_MqttClientCtxDelete(local_con_conf[iCount].clt_ctx);
            iConnBroker--;
            break;
        }
        else if(sl_ExtLib_MqttClientSub((void*)local_con_conf[iCount].clt_ctx,
                                   FLAG,
                                   local_con_conf[iCount].qos, 2) < 0)
        {
            sl_ExtLib_MqttClientDisconnect(local_con_conf[iCount].clt_ctx);
            local_con_conf[iCount].is_connected = false;

            //delete the context for this connection
            sl_ExtLib_MqttClientCtxDelete(local_con_conf[iCount].clt_ctx);
            iConnBroker--;
            break;
        }
        iCount++;
    }

    if(iConnBroker < 1)
    {
        //
        // no succesful connection to broker
        //
        goto end;
    }

    iCount = 0;

    for(;;)
    {
        osi_MsgQRead( &g_PBQueue, &RecvQue, OSI_WAIT_FOREVER);
        
        if(ANALOG_INPUTS == RecvQue.event)
        {
            unsigned char* message = sendRegisterValues(0);
            //
            // send publish message
            //
            sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
                    PUB_TOPIC_AI, message,strlen((char*)message),QOS2,RETAIN);
        }
        else if(ANALOG_OUTPUTS == RecvQue.event)
        {
            unsigned char* message = sendRegisterValues(1);
            //
            // send publish message
            //

            sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
                    PUB_TOPIC_AO,message,strlen((char*) message),QOS2,RETAIN);
        }
        else if(BROKER_DISCONNECTION == RecvQue.event)
        {
            iConnBroker--;
            /* Derive the value of the local_con_conf or clt_ctx from the message */
			sl_ExtLib_MqttClientCtxDelete(((connect_config*)(RecvQue.hndl))->clt_ctx);
            
            if(!IS_CONNECTED(g_ulStatus))
            {
                
                while(!(IS_CONNECTED(g_ulStatus)) || !(IS_IP_ACQUIRED(g_ulStatus)))
                {
                    osi_Sleep(10);
                }
                goto connect_to_broker;
                
            }
            if(iConnBroker < 1)
            {
                //
                // device not connected to any broker
                //
                goto connect_to_broker;
            }
        }
    }
end:
    //
    // Deinitializating the client library
    //
    sl_ExtLib_MqttClientExit();
    
    LOOP_FOREVER();
}

//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! This function
//!    1. Invokes the SLHost task
//!    2. Invokes the MqttClient
//!
//! \return None
//!
//*****************************************************************************

void ModbusTask( void *pvParameters){
    osi_SyncObjWait(&sync_obj, OSI_WAIT_FOREVER);
    osi_SyncObjSignal(&sync_obj);

    SlSockAddrIn_t  sAddr;
    SlSockAddrIn_t  sLocalAddr;
    int             iCounter;
    int             iAddrSize;
    int             iSockID;
    int             iStatus;
    int             iNewSockID;
    long            lNonBlocking = 1;
    int             iTestBufLen;
    #define BUF_SIZE            1400
    char g_cBsdBuf[BUF_SIZE];
    unsigned short usPort = 502;

reset_tcp:

    // filling the buffer
    for (iCounter=0 ; iCounter<BUF_SIZE ; iCounter++)
    {
        g_cBsdBuf[iCounter] = (char)(iCounter % 10);
    }

    iTestBufLen  = BUF_SIZE;

    //filling the TCP server socket address
    sLocalAddr.sin_family = SL_AF_INET;
    sLocalAddr.sin_port = sl_Htons((unsigned short)usPort);
    sLocalAddr.sin_addr.s_addr = 0;

    // creating a TCP socket
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if( iSockID < 0 )
    {
        // error
        goto reset_tcp;
    }

    iAddrSize = sizeof(SlSockAddrIn_t);

    // binding the TCP socket to the TCP server address
    iStatus = sl_Bind(iSockID, (SlSockAddr_t *)&sLocalAddr, iAddrSize);
    if( iStatus < 0 )
    {
        // error
        sl_Close(iSockID);
        goto reset_tcp;
    }

    // putting the socket for listening to the incoming TCP connection
    iStatus = sl_Listen(iSockID, 0);
    if( iStatus < 0 )
    {
        sl_Close(iSockID);
        goto reset_tcp;
    }

    // setting socket option to make the socket as non blocking
    iStatus = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
                            &lNonBlocking, sizeof(lNonBlocking));
    if( iStatus < 0 )
    {
        sl_Close(iSockID);
        goto reset_tcp;
    }
    iNewSockID = SL_EAGAIN;

    // waiting for an incoming TCP connection
    while( iNewSockID < 0 )
    {
        // accepts a connection form a TCP client, if there is any
        // otherwise returns SL_EAGAIN
        iNewSockID = sl_Accept(iSockID, ( struct SlSockAddr_t *)&sAddr,
                                (SlSocklen_t*)&iAddrSize);
        if( iNewSockID == SL_EAGAIN )
        {
           MAP_UtilsDelay(10000);
        }
        else if( iNewSockID < 0 )
        {
            // error
            sl_Close(iNewSockID);
            sl_Close(iSockID);
            goto reset_tcp;
        }
    }

    // waits for 1000 packets from the connected TCP client
    while (true)
    {
        iStatus = sl_Recv(iNewSockID, g_cBsdBuf, iTestBufLen, 0);
        if( iStatus <= 0 )
        {
          // error
          sl_Close(iNewSockID);
          sl_Close(iSockID);
          goto reset_tcp;
        }

        int i;
        char* response = clientHandler(g_cBsdBuf);
        int len = response[0];
        for(i = 0; i < len; i++)
            g_cBsdBuf[i] = response[i+1];
        iStatus = sl_Send(iNewSockID, g_cBsdBuf, len, 0);
        if( iStatus <= 0 )
        {
          // error
          sl_Close(iNewSockID);
          sl_Close(iSockID);
          goto reset_tcp;
        }
    }
}

void ConnectToAP(void *pvParameters){

    osi_SyncObjCreate(&sync_obj);

    long lRetVal = -1;
    unsigned char policyVal;


    //
    // Reset The state of the machine
    //
    Network_IF_ResetMCUStateMachine();

    //
    // Start the driver
    //
    lRetVal = Network_IF_InitDriver(ROLE_STA);
    if(lRetVal < 0)
    {
       LOOP_FOREVER();
    }

    // Initialize AP security params
    SecurityParams.Key = (signed char *)SECURITY_KEY;
    SecurityParams.KeyLen = strlen(SECURITY_KEY);
    SecurityParams.Type = SECURITY_TYPE;

    //
    // Connect to the Access Point
    //
    lRetVal = Network_IF_ConnectAP(SSID_NAME, SecurityParams);
    if(lRetVal < 0)
    {
       LOOP_FOREVER();
    }

    lRetVal = sl_WlanProfileAdd(SSID_NAME,strlen(SSID_NAME),0,&SecurityParams,0,1,0);

    //set AUTO policy
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                      SL_CONNECTION_POLICY(1,0,0,0,0),
                      &policyVal, 1 /*PolicyValLen*/);

    osi_SyncObjSignal(&sync_obj);
    osi_TaskDelete(&handle);
}

void main()
{ 
    long lRetVal = -1;
    //
    // Initialize the board configurations
    //
    BoardInit();

    //
    // Pinmux for UART
    //
    PinMuxConfig();

    //
    // Configuring UART
    //
    InitTerm();

    InitPWMModules();
    ADCInit();

    //
    // Start the SimpleLink Host
    //
    lRetVal = VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }
    //
    // Start the MQTT Client task
    //
    osi_MsgQCreate(&g_PBQueue,"PBQueue",sizeof(event_msg),10);

    lRetVal = osi_TaskCreate(ConnectToAP,
                            (const signed char *)"Task0",
                            OSI_STACK_SIZE, NULL, 3, &handle );
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    lRetVal = osi_TaskCreate(ModbusTask, (const signed char *)"MODBUS",
                            OSI_STACK_SIZE, NULL, 1, NULL );
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    lRetVal = osi_TaskCreate(MqttClient, (const signed char *)"MQTT",
                            OSI_STACK_SIZE, NULL, 1, NULL );

    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }
    //
    // Start the task scheduler
    //
    osi_start();
}

