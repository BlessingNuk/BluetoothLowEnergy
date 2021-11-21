/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : App/p2p_routeur_app.c
 * Description        : P2P Router Application
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "app_ble.h"
#include "p2p_routeur_app.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

typedef struct{
    /**
     * state of the Connection
     */
    APP_BLE_ConnStatus_t state;

    /**
     * connection handle
     */
    uint16_t connHandle;

    /**
     * handle of the Led Button service
     */
    uint16_t P2PServiceHandle;

    /**
     * end handle of the Data Transfer service
     */
    uint16_t P2PServiceEndHandle;

    /**
     * handle of the Tx characteristic
     *
     */
    uint16_t P2PTxCharHdle;

    /**
     * handle of the client configuration
     * descriptor of Tx characteristic
     */
    uint16_t P2PLedDescHandle;

    /**
     * handle of the client configuration
     * descriptor of new alert characteristic
     */

    uint16_t  P2PClientCharHdle;                  /**< Characteristic handle */
    uint16_t  P2PClientDescHandle;                    /**< Characteristic handle */

    uint8_t data_buffer[];
}P2P_ClientContext_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
#define UNPACK_2_BYTE_PARAMETER(ptr)  \
        (uint16_t)((uint16_t)(*((uint8_t *)ptr))) |   \
        (uint16_t)((((uint16_t)(*((uint8_t *)ptr + 1))) << 8))
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

PLACE_IN_SECTION("BLE_APP_CONTEXT") static P2P_ClientContext_t aP2PClientContext[BLE_CFG_CLT_MAX_NBR_CB];
//PLACE_IN_SECTION("BLE_APP_CONTEXT") static uint8_t device_data_buffer[] = {9};

/**
 * END of Section BLE_APP_CONTEXT
 */
/* USER CODE BEGIN PV */
//uint8_t the_data[] = {6, 9}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

static SVCCTL_EvtAckStatus_t Client_Event_Handler(void *pckt);
static tBleStatus Client_Update_Char(uint16_t UUID, uint8_t Service_Instance, uint8_t *pPayload);
static void Client_Update_Service( void );
void P2P_Router_APP_Init(void);
void P2P_Client_App_Notification(P2P_Client_App_Notification_evt_t *pNotification);
void P2P_Client_Init(void);
void add_simple_service(void);
uint8_t get_device_data();
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/

/**
 * @brief  End Device service Notification
 * @param  GATT Notification (Opcode & Data)
 * @retval None
 */
void EDS_STM_App_Notification(EDS_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN EDS_STM_App_Notification_1 */

/* USER CODE END EDS_STM_App_Notification_1 */
        switch(pNotification->EDS_Evt_Opcode)
    {
        /* USER CODE BEGIN EDS_Evt_Opcode */

        /* USER CODE END EDS_Evt_Opcode */
        case EDS_STM_NOTIFY_ENABLED_EVT:
#if(CFG_DEBUG_APP_TRACE != 0)
            APP_DBG_MSG("-- APPLICATION ROUTER : ENDDEVICEMGT NOTIFICATION ENABLED\r\n");
#endif
            /* USER CODE BEGIN EDS_STM_NOTIFY_ENABLED_EVT */

            /* USER CODE END EDS_STM_NOTIFY_ENABLED_EVT */
            break;

        case EDS_STM_NOTIFY_DISABLED_EVT:
#if(CFG_DEBUG_APP_TRACE != 0)
            APP_DBG_MSG("-- APPLICATION ROUTER : ENDDEVICEMGT NOTIFICATION DISABLED\r\n");
#endif
            /* USER CODE BEGIN EDS_STM_NOTIFY_DISABLED_EVT */

            /* USER CODE END EDS_STM_NOTIFY_DISABLED_EVT */
            break;

        default:
            /* USER CODE BEGIN EDS_Evt_Opcode_default */

            /* USER CODE END EDS_Evt_Opcode_default */
            break;
    }
/* USER CODE BEGIN EDS_STM_App_Notification_2 */

/* USER CODE END EDS_STM_App_Notification_2 */
    return;
}
/**
 * @brief  P2P service Notification
 * @param  GATT Notification (Opcode & Data)
 * @retval None
 */
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_STM_App_Notification_1 */

/* USER CODE END P2PS_STM_App_Notification_1 */
    switch(pNotification->P2P_Evt_Opcode)
    {
        /* USER CODE BEGIN P2P_Evt_Opcode */

        /* USER CODE END P2P_Evt_Opcode */
        case P2PS_STM__NOTIFY_ENABLED_EVT:
#if(CFG_DEBUG_APP_TRACE != 0)
            APP_DBG_MSG("-- APPLICATION ROUTER : BUTTON NOTIFICATION ENABLED\n\r");
#endif
            /* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */
            	printf("notification enabled..\n\r");
            /* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
            break;

        case P2PS_STM_NOTIFY_DISABLED_EVT:
#if(CFG_DEBUG_APP_TRACE != 0)
            APP_DBG_MSG("-- APPLICATION ROUTER : BUTTON NOTIFICATION DISABLED\n\r");
#endif
            /* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */
            printf("notification disabled..\n\r");
            /* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
            break;

        case P2PS_STM_READ_EVT:
#if(CFG_DEBUG_APP_TRACE != 0)
            APP_DBG_MSG("-- APPLICATION ROUTER : LED READ STATUS\n\r");
#endif
            /* USER CODE BEGIN P2PS_STM_READ_EVT */
            printf("READ event happened\n\r");
            /* USER CODE END P2PS_STM_READ_EVT */
            break;

        case P2PS_STM_WRITE_EVT: //SEEMS LIKE SERVER SIDE
#if(CFG_DEBUG_APP_TRACE != 0)
            APP_DBG_MSG("-- APPLICATION ROUTER : WRITE LED CONFIG TO ALL END DEVICE \n\r");
#endif
            /* USER CODE BEGIN P2PS_STM_WRITE_EVT */
            printf("\n\r...DATA RECEIVED...\n\r\n");

            for(int i=0;i<pNotification->DataTransfered.Length;i++){
            	  aP2PClientContext->data_buffer[i]= pNotification->DataTransfered.pPayload[i];
            	  printf("\n\r...Data received is: %d...\n\r", aP2PClientContext->data_buffer[i]);
              }
            terminate_connection(pNotification->ConnectionHandle);

            /* USER CODE END P2PS_STM_WRITE_EVT */
            break;

        default:
            /* USER CODE BEGIN P2P_Evt_Opcode_default */

            /* USER CODE END P2P_Evt_Opcode_default */
        break;
    }
/* USER CODE BEGIN P2PS_STM_App_Notification_2 */

/* USER CODE END P2PS_STM_App_Notification_2 */
    return;
}

/**
 * @brief  Router Application initialisation
 * @param  None
 * @retval None
 */
void P2P_Router_APP_Init(void)
{
    /* USER CODE BEGIN P2P_Router_APP_Init_1 */

    /* USER CODE END P2P_Router_APP_Init_1 */

    UTIL_SEQ_RegTask( 1<< CFG_TASK_SEARCH_SERVICE_ID, UTIL_SEQ_RFU, Client_Update_Service );
   // UTIL_SEQ_RegTask( 1<< CFG_TASK_DISCONN_DEV_1_ID, UTIL_SEQ_RFU, terminate_connection );

    /* USER CODE BEGIN P2P_Router_APP_Init_2 */
    /**
       * Initialize LedButton Service
       */
    /* USER CODE END P2P_Router_APP_Init_2 */

    return;
}


/**
 * @brief  End Device Management
 * @param  None
 * @retval None
 */
void P2PR_APP_End_Device_Mgt_Connection_Update( P2PR_APP_Device_Status_t *p_device_status )
{
/* USER CODE BEGIN P2PR_APP_End_Device_Mgt_Connection_Update_1 */
	//uint8_t buffer[]={7,4};
/* USER CODE END P2PR_APP_End_Device_Mgt_Connection_Update_1 */
    if( (p_device_status->Device1_Status) & (0x81) )
    {
    /* USER CODE BEGIN Device1_Status */
    	//Client_Update_Char(P2P_NOTIFY_CHAR_UUID, 0, buffer);
    //	aP2PClientContext->state = APP_BLE_UPDATE_CHARACS;

    /* USER CODE END Device1_Status */
    }

#if (CFG_P2P_DEMO_MULTI != 0 )
    /* USER CODE BEGIN CFG_P2P_DEMO_MULTI */

    /* USER CODE END CFG_P2P_DEMO_MULTI */
#endif
/* USER CODE BEGIN P2PR_APP_End_Device_Mgt_Connection_Update_2 */

/* USER CODE END P2PR_APP_End_Device_Mgt_Connection_Update_2 */
    return;
}

/**
 * @brief  Notification from the Server
 * @param  Client Notification from Server side
 * @retval None
 */
void P2P_Client_App_Notification(P2P_Client_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN P2P_Client_App_Notification_1 */

/* USER CODE END P2P_Client_App_Notification_1 */
    switch(pNotification->P2P_Client_Evt_Opcode)
    {
    /* USER CODE BEGIN P2P_Client_Evt_Opcode */

    /* USER CODE END P2P_Client_Evt_Opcode */
        default:
    /* USER CODE BEGIN P2P_Client_Evt_Opcode_default */

    /* USER CODE END P2P_Client_Evt_Opcode_default */
            break;

    }
/* USER CODE BEGIN P2P_Client_App_Notification_2 */

/* USER CODE END P2P_Client_App_Notification_2 */
    return;
}

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void P2P_Client_Init(void)
{
    uint8_t index =0;
/* USER CODE BEGIN P2P_Client_Init_1 */

/* USER CODE END P2P_Client_Init_1 */

    for(index = 0; index < BLE_CFG_CLT_MAX_NBR_CB; index++)
        aP2PClientContext[index].state= APP_BLE_IDLE;
    /**
     *    Register the event handler to the BLE controller
     */
    SVCCTL_RegisterCltHandler(Client_Event_Handler);
/* USER CODE BEGIN P2P_Client_Init_2 */

/* USER CODE END P2P_Client_Init_2 */
    return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */
/*************************************************************
 *
 * PUBLIC FUNCTIONS
 *
 *************************************************************/
void P2PR_APP_Init( void )
{
    P2P_Router_APP_Init();
    P2P_Client_Init();
    /* USER CODE BEGIN P2PR_APP_Init */
   // add_simple_service();
    /* USER CODE END P2PR_APP_Init */
    return;
}

/* USER CODE BEGIN FD_PUBLIC_FUNCTIONS */
//void add_simple_service(void)
//{
//	uint8_t SERVICE_UUID[16] = {0x7C,0xBA,0xA5,0x1A,0x2F,0xFF,0x11,0xEC,0x85,0x15,0xAA,0xC3,0x19,0xAA,0x34,0xC8};
//	Service_UUID_t service_uuid;
//	memccpy(service_uuid.Service_UUID_128,SERVICE_UUID,16);
//	uint16_t my_service_handle;
//    aci_gatt_add_service(UUID_TYPE_128,&service_uuid , PRIMARY_SERVICE, 7, &my_service_handle);
//    return;
//}

/* USER CODE END FD_PUBLIC_FUNCTIONS */
/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Client_Update_Service( void )
{
    /* USER CODE BEGIN Client_Update_Service_1 */

    /* USER CODE END Client_Update_Service_1 */
  //  uint16_t enable = 0x0001;
    uint8_t buffer[]={7,4};
    uint8_t index;

    index = 0;
    while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
            (aP2PClientContext[index].state != APP_BLE_IDLE))
    {

        switch(aP2PClientContext[index].state)
        {
        /* USER CODE BEGIN aP2PClientContext */

        /* USER CODE END aP2PClientContext */

            case APP_BLE_UPDATE_CHARACS:
            {
            //	aP2PClientContext[index].state = APP_BLE_CONNECTED;
            	Client_Update_Char(P2P_NOTIFY_CHAR_UUID, 0, buffer);
            	printf("\n\r...TRANSMISSION COMPLETE...\n\r");

            }
            break;
            default:
            /* USER CODE BEGIN aP2PClientContext_default */

            /* USER CODE END aP2PClientContext_default */
                break;
        }
        index++;
    }
    /* USER CODE BEGIN Client_Update_Service_2 */
  //  terminate_connection(aP2PClientContext->connHandle);
    /* USER CODE END Client_Update_Service_2 */
    return;
}

/**
 * @brief  Feature Characteristic update
 * @param  pFeatureValue: The address of the new value to be written
 * @retval None
 */
//THIS IS CLIENT SIDE
static tBleStatus Client_Update_Char(uint16_t UUID, uint8_t Service_Instance, uint8_t *pPayload)
{
    /* USER CODE BEGIN Client_Update_Char_1 */
//	uint8_t buffer[]={7,4};
    /* USER CODE END Client_Update_Char_1 */
    tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
    uint8_t index;

    index = 0;
    while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
            (aP2PClientContext[index].state != APP_BLE_IDLE))
    {
        /* USER CODE BEGIN Client_Update_Char_2 */
    	switch(UUID)
    	    {
    	      case P2P_NOTIFY_CHAR_UUID: /* SERVER RX -- so CLIENT TX */
    	      ret = aci_gatt_write_without_resp(aP2PClientContext[index].connHandle,
    	                                         aP2PClientContext[index].P2PServiceEndHandle,
    	                                         2, /* charValueLen */
    	                                         (uint8_t *)  pPayload);
    	        break;
    	      default:
    	        break;
    	    }
        /* USER CODE END Client_Update_Char_2 */
        index++;
    }
    /* USER CODE BEGIN Client_Update_Char_3 */
    if(ret==BLE_STATUS_SUCCESS){
    	printf("\n\r...TRANSMIT/SEND DATA SUCCESS...\n\r");
    	for(int i=0;i<2;i++){
		printf("\n\r...Data sent is: %d...\n\r",pPayload[i]);
	  }

    }
    /* USER CODE END Client_Update_Char_3 */
    return ret;
}/* end Client_Update_Char() */

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Client_Event_Handler(void *Event)
{
    /* USER CODE BEGIN Client_Event_Handler_1 */

    /* USER CODE END Client_Event_Handler_1 */
    SVCCTL_EvtAckStatus_t return_value;
    hci_event_pckt *event_pckt;
    evt_blecore_aci *blecore_evt;
    P2P_Client_App_Notification_evt_t Notification;

    return_value = SVCCTL_EvtNotAck;
    event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

    switch(event_pckt->evt)
    {
        case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
        {
            /* USER CODE BEGIN EVT_VENDOR */

            /* USER CODE END EVT_VENDOR */
            blecore_evt = (evt_blecore_aci*)event_pckt->data;
            switch(blecore_evt->ecode)
            {
            /* USER CODE BEGIN ecode */

            /* USER CODE END ecode */
                case ACI_ATT_READ_BY_GROUP_TYPE_RESP_VSEVT_CODE:
                {
                    /* USER CODE BEGIN EVT_BLUE_ATT_READ_BY_GROUP_TYPE_RESP */

                    /* USER CODE END EVT_BLUE_ATT_READ_BY_GROUP_TYPE_RESP */
                    aci_att_read_by_group_type_resp_event_rp0 *pr = (void*)blecore_evt->data;
                    uint8_t numServ, i, idx;
                    uint16_t uuid, handle;

                    uint8_t index;
                    handle = pr->Connection_Handle;
                    index = 0;
                    while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                            (aP2PClientContext[index].state != APP_BLE_IDLE)/* &&
                        (aP2PClientContext[index].connHandle != handle)*/)
                    {
                        APP_BLE_ConnStatus_t status;

                        status = APP_BLE_Get_Client_Connection_Status(aP2PClientContext[index].connHandle);
                        if((aP2PClientContext[index].state == APP_BLE_CONNECTED)&&
                                (status == APP_BLE_IDLE))
                        {
                            /* Handle deconnected */

                            aP2PClientContext[index].state = APP_BLE_IDLE;
                            aP2PClientContext[index].connHandle = 0xFFFF;
                            break;
                        }
                        index++;
                    }

                    if(index < BLE_CFG_CLT_MAX_NBR_CB)
                    {
                        aP2PClientContext[index].connHandle= handle;
                        numServ = (pr->Data_Length) / pr->Attribute_Data_Length;

                        /* the event data will be
                         * 2bytes start handle
                         * 2bytes end handle
                         * 2 or 16 bytes data
                         * we are interested only if the UUID is 16 bit.
                         * So check if the data length is 6
                         * we are interested only if the UUID is 128 bit.
                         * So check if the data length is 20
                         */
#if (UUID_128BIT_FORMAT==1)
                        if (pr->Attribute_Data_Length == 20)
                        {
                            idx = 16;
#else
                            if (pr->Attribute_Data_Length == 6)
                            {
                                idx = 4;
#endif
                                for (i=0; i<numServ; i++)
                                {
                                    uuid = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx]);
                                    if(uuid == P2P_SERVICE_UUID)
                                    {
#if(CFG_DEBUG_APP_TRACE != 0)
                                        APP_DBG_MSG("-- GATT : P2P_SERVICE_UUID FOUND - connection handle 0x%x \n", aP2PClientContext[index].connHandle);
#endif
                                        #if (UUID_128BIT_FORMAT==1)
                                        aP2PClientContext[index].P2PServiceHandle = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx-16]);
                                        aP2PClientContext[index].P2PServiceEndHandle = UNPACK_2_BYTE_PARAMETER (&pr->Attribute_Data_List[idx-14]);
#else
                                        aP2PClientContext[index].P2PServiceHandle = UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx-4]);
                                        aP2PClientContext[index].P2PServiceEndHandle = UNPACK_2_BYTE_PARAMETER (&pr->Attribute_Data_List[idx-2]);
#endif
                                        aP2PClientContext[index].state = APP_BLE_DISCOVER_CHARACS ;
                                    }
                                    idx += 6;
                                }
                            }
                        }
                    }
                    break;

                case ACI_ATT_READ_BY_TYPE_RESP_VSEVT_CODE:
                {
                    /* USER CODE BEGIN EVT_BLUE_ATT_READ_BY_TYPE_RESP */

                    /* USER CODE END EVT_BLUE_ATT_READ_BY_TYPE_RESP */
                    aci_att_read_by_type_resp_event_rp0 *pr = (void*)blecore_evt->data;

                    uint8_t idx;
                    uint16_t uuid, handle;

                    /* the event data will be
                     * 2 bytes start handle
                     * 1 byte char properties
                     * 2 bytes handle
                     * 2 or 16 bytes data
                     */

                    uint8_t index;

                    index = 0;
                    while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                            (aP2PClientContext[index].connHandle != pr->Connection_Handle))
                        index++;

                    if(index < BLE_CFG_CLT_MAX_NBR_CB)
                    {

                        /* we are interested in only 16 bit UUIDs */
#if (UUID_128BIT_FORMAT==1)
                        idx = 17;
                        if (pr->Handle_Value_Pair_Length == 21)
#else
                            idx = 5;
                        if (pr->Handle_Value_Pair_Length == 7)
#endif
                        {
                            pr->Data_Length -= 1;
                            while(pr->Data_Length > 0)
                            {
                                uuid = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx]);
                                /* store the characteristic handle not the attribute handle */
#if (UUID_128BIT_FORMAT==1)
                                handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx-14]);
#else
                                handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx-2]);
#endif
                                if(uuid == P2P_WRITE_CHAR_UUID)
                                {
#if(CFG_DEBUG_APP_TRACE != 0)
                                    APP_DBG_MSG("-- GATT : LED_CHAR_UUID FOUND - connection handle 0x%x\n", aP2PClientContext[index].connHandle);
#endif
                                    aP2PClientContext[index].state = APP_BLE_DISCOVER_LED_CHAR_DESC;
                                    aP2PClientContext[index].P2PTxCharHdle = handle;
                                }

                                else if(uuid == P2P_NOTIFY_CHAR_UUID)
                                {
#if(CFG_DEBUG_APP_TRACE != 0)
                                    APP_DBG_MSG("-- GATT : BUTTON_CHAR_UUID FOUND  - connection handle 0x%x\n", aP2PClientContext[index].connHandle);
#endif
                                    aP2PClientContext[index].state = APP_BLE_DISCOVER_BUTTON_CHAR_DESC;
                                    aP2PClientContext[index].P2PClientCharHdle = handle;
                                }
#if (UUID_128BIT_FORMAT==1)
                                pr->Data_Length -= 21;
                                idx += 21;
#else
                                pr->Data_Length -= 7;
                                idx += 7;
#endif
                            }
                        }
                    }
                }
                break;

                case ACI_ATT_FIND_INFO_RESP_VSEVT_CODE:
                {
                    /* USER CODE BEGIN EVT_BLUE_ATT_FIND_INFORMATION_RESP */

                    /* USER CODE END EVT_BLUE_ATT_FIND_INFORMATION_RESP */
                    aci_att_find_info_resp_event_rp0 *pr = (void*)blecore_evt->data;

                    uint8_t numDesc, idx, i;
                    uint16_t uuid, handle;

                    /*
                     * event data will be of the format
                     * 2 bytes handle
                     * 2 bytes UUID
                     */

                    uint8_t index;

                    index = 0;
                    while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                            (aP2PClientContext[index].connHandle != pr->Connection_Handle))

                        index++;

                    if(index < BLE_CFG_CLT_MAX_NBR_CB)
                    {

                        numDesc = (pr->Event_Data_Length) / 4;
                        /* we are interested only in 16 bit UUIDs */
                        idx = 0;
                        if (pr->Format == UUID_TYPE_16)
                        {
                            for (i=0; i<numDesc; i++)
                            {
                                handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_UUID_Pair[idx]);
                                uuid = UNPACK_2_BYTE_PARAMETER(&pr->Handle_UUID_Pair[idx+2]);

                                if(uuid == CLIENT_CHAR_CONFIG_DESCRIPTOR_UUID)
                                {
#if(CFG_DEBUG_APP_TRACE != 0)
                                    APP_DBG_MSG("-- GATT : CLIENT_CHAR_CONFIG_DESCRIPTOR_UUID- connection handle 0x%x\n", aP2PClientContext[index].connHandle);
#endif
                                    if( aP2PClientContext[index].state == APP_BLE_DISCOVER_BUTTON_CHAR_DESC)
                                    {

                                        aP2PClientContext[index].P2PClientDescHandle = handle;
                                        aP2PClientContext[index].state = APP_BLE_ENABLE_NOTIFICATION_BUTTON_DESC;

                                    }
                                }
                                idx += 4;
                            }
                        }
                    }
                }
                break; /*ACI_ATT_FIND_INFO_RESP_VSEVT_CODE*/

                case ACI_GATT_NOTIFICATION_VSEVT_CODE:
                {
                    /* USER CODE BEGIN EVT_BLUE_GATT_NOTIFICATION */

                    /* USER CODE END EVT_BLUE_GATT_NOTIFICATION */
                    aci_gatt_notification_event_rp0 *pr = (void*)blecore_evt->data;
                    uint8_t index;

                    index = 0;
                    while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                            (aP2PClientContext[index].connHandle != pr->Connection_Handle))
                        index++;

                    if(index < BLE_CFG_CLT_MAX_NBR_CB)
                    {

                        if ( (pr->Attribute_Handle == aP2PClientContext[index].P2PClientCharHdle) &&
                                (pr->Attribute_Value_Length == (2)) )
                        {
#if(CFG_DEBUG_APP_TRACE != 0)
                            APP_DBG_MSG("-- GATT : BUTTON CHARACTERISTICS RECEIVED_EVT - connection handle 0x%x\n", aP2PClientContext[index].connHandle);
#endif
                            Notification.P2P_Client_Evt_Opcode = P2P_BUTTON_INFO_RECEIVED_EVT;
                            Notification.DataTransfered.Length = pr->Attribute_Value_Length;
                            Notification.DataTransfered.pPayload = &pr->Attribute_Value[0];

                            P2P_Client_App_Notification(&Notification);

                            /* INFORM APPLICATION BUTTON IS PUSHED BY END DEVICE */

                        }
                    }
                }
                break;/* end ACI_GATT_NOTIFICATION_VSEVT_CODE */

                case ACI_GATT_PROC_COMPLETE_VSEVT_CODE:
                {
                    /* USER CODE BEGIN EVT_BLUE_GATT_PROCEDURE_COMPLETE */
                		//printf("\nACI_GATT_PROC_COMPLETE_VSEVT_CODE\n\r");

                    /* USER CODE END EVT_BLUE_GATT_PROCEDURE_COMPLETE */
                    aci_gatt_proc_complete_event_rp0 *pr = (void*)blecore_evt->data;
                  //  terminate_connection(pr->Connection_Handle);
#if(CFG_DEBUG_APP_TRACE != 0)
                    APP_DBG_MSG("-- GATT : ACI_GATT_PROC_COMPLETE_VSEVT_CODE \n");
                    APP_DBG_MSG("\n");
#endif
                    uint8_t index;
                    index = 0;
                    while((index < BLE_CFG_CLT_MAX_NBR_CB) &&
                            (aP2PClientContext[index].connHandle != pr->Connection_Handle))
                        index++;

                    if(index < BLE_CFG_CLT_MAX_NBR_CB) ///connected now changing char
                    {
                    	aP2PClientContext->state = APP_BLE_UPDATE_CHARACS;
                    	UTIL_SEQ_SetTask(  1<<CFG_TASK_SEARCH_SERVICE_ID, CFG_SCH_PRIO_0 );
                 //   	terminate_connection(pr->Connection_Handle);
                    }
                 //   terminate_connection(pr->Connection_Handle);



                }
                break; /*ACI_GATT_PROC_COMPLETE_VSEVT_CODE*/
                default:
                    /* USER CODE BEGIN ecode_default */

                    /* USER CODE END ecode_default */
                    break;
                }
            }

            break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

        default:
            /* USER CODE BEGIN evt_default */

            /* USER CODE END evt_default */
            break;
        }
/* USER CODE BEGIN Client_Event_Handler_2 */

       // terminate_connection(pr->Connection_Handle);
/* USER CODE END Client_Event_Handler_2 */
        return(return_value);
    }/* end BLE_CTRL_Event_Acknowledged_Status_t */

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */

/* USER CODE END FD_LOCAL_FUNCTIONS */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
