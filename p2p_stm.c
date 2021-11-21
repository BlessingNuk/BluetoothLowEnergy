/**
  ******************************************************************************
  * @file    p2p_stm.c
  * @author  MCD Application Team
  * @brief   Peer to Peer Service (Custom STM)
  ******************************************************************************
  * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include "common_blesvc.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct{

  uint16_t	data_transfer_service_handle;
  uint16_t	data_transfer_tx_char_handle;
  uint16_t	data_transfer_rx_char_handle;
  uint8_t 	rcv_data[2]; //ka multihop we gonna send this to send data

  //uint16_t	data_transfer_service_handle;				        /**< Service handle */
 // uint16_t	data_transfer_tx_char_handle;	  /**< Characteristic handle */
  //uint16_t	data_transfer_rx_char_handle;	/**< Characteristic handle */
#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
  uint16_t  RebootReqCharHdle;                /**< Characteristic handle */
#endif
}PeerToPeerContext_t;

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED  1

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)


/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/**
 * Reboot Characteristic UUID
 * 0000fe11-8e22-4541-9d4c-21edae82ed19
 */
#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
#if (UUID_128_SUPPORTED == 1)
static const uint8_t BM_REQ_CHAR_UUID[16] = {0x19, 0xed, 0x82, 0xae,
                                       0xed, 0x21, 0x4c, 0x9d,
                                       0x41, 0x45, 0x22, 0x8e,
                                       0x11, 0xFE, 0x00, 0x00};
#else
static const uint8_t BM_REQ_CHAR_UUID[2] = {0x11, 0xFE};
#endif
#endif

/**
 * START of Section BLE_DRIVER_CONTEXT
 */
PLACE_IN_SECTION("BLE_DRIVER_CONTEXT") static PeerToPeerContext_t aPeerToPeerContext;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */
/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t PeerToPeer_Event_Handler(void *pckt);


/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Hardware Characteristics Service */
/*
 The following 128bits UUIDs have been generated from the random UUID
 generator:
 D973F2E0-B19E-11E2-9E96-0800200C9A66: Service 128bits UUID
 D973F2E1-B19E-11E2-9E96-0800200C9A66: Characteristic_1 128bits UUID
 D973F2E2-B19E-11E2-9E96-0800200C9A66: Characteristic_2 128bits UUID
 #define DATA_TRANSFER_SERVICE_UUID                                     (0xFE80)
 #define DATA_TRANSFER_TX_CHAR_UUID                                     (0xFE81)
 #define DATA_TRANSFER_RX_CHAR_UUID                                     (0xFE82)
 */
#define COPY_DATA_TRANSFER_SERVICE_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0xfe,0x40,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_DATA_TRANSFER_TX_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0xfe,0x41,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_DATA_TRANSFER_RX_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0xfe,0x42,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)



/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t PeerToPeer_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blecore_aci *blecore_evt;
  aci_gatt_attribute_modified_event_rp0    * attribute_modified;
  P2PS_STM_App_Notification_evt_t Notification;

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch(event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
    {
      blecore_evt = (evt_blecore_aci*)event_pckt->data;
      switch(blecore_evt->ecode)
      {
        case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
       {
          attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;
//            if(attribute_modified->Attr_Handle == (aPeerToPeerContext.data_transfer_rx_char_handle + 2))
//            {
//              /**
//               * Descriptor handle
//               */
//              return_value = SVCCTL_EvtAckFlowEnable;
//              /**
//               * Notify to application
//               */
//              if(attribute_modified->Attr_Data[0] & COMSVC_Notification)
//              {
//                Notification.P2P_Evt_Opcode = P2PS_STM__NOTIFY_ENABLED_EVT;
//                P2PS_STM_App_Notification(&Notification);
//              }
//              else
//              {
//                Notification.P2P_Evt_Opcode = P2PS_STM_NOTIFY_DISABLED_EVT;
//                P2PS_STM_App_Notification(&Notification);
//              }
//            }
            if(attribute_modified->Attr_Handle == (aPeerToPeerContext.data_transfer_rx_char_handle + 1)){
            	//printf("modified\n\r");
            	receive_data(attribute_modified->Attr_Data, attribute_modified->Attr_Data_Length); //my method
            	Notification.P2P_Evt_Opcode = P2PS_STM_WRITE_EVT;
            	Notification.DataTransfered.pPayload = attribute_modified->Attr_Data;
            	Notification.DataTransfered.Length = attribute_modified->Attr_Data_Length;
            	Notification.ConnectionHandle = attribute_modified->Connection_Handle;
            	P2PS_STM_App_Notification(&Notification);
            }
            else if(attribute_modified->Attr_Handle == (aPeerToPeerContext.data_transfer_tx_char_handle + 2)){
              return_value = SVCCTL_EvtAckFlowEnable;
			  /**
			   * Notify to application
			   */
			  if(attribute_modified->Attr_Data[0] & COMSVC_Notification)
			  {
				Notification.P2P_Evt_Opcode = P2PS_STM__NOTIFY_ENABLED_EVT;
				P2PS_STM_App_Notification(&Notification);
			  }
			  else
			  {
				Notification.P2P_Evt_Opcode = P2PS_STM_NOTIFY_DISABLED_EVT;
				P2PS_STM_App_Notification(&Notification);
			  }

            }
//            else if(attribute_modified->Attr_Handle == (aPeerToPeerContext.data_transfer_tx_char_handle + 1))
//            {
//              BLE_DBG_P2P_STM_MSG("-- GATT : LED CONFIGURATION RECEIVED\n");
//              Notification.P2P_Evt_Opcode = P2PS_STM_WRITE_EVT;
//              Notification.DataTransfered.Length=attribute_modified->Attr_Data_Length;
//              Notification.DataTransfered.pPayload=attribute_modified->Attr_Data;
//              P2PS_STM_App_Notification(&Notification);
//            }
#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
            else if(attribute_modified->Attr_Handle == (aPeerToPeerContext.RebootReqCharHdle + 1))
            {
              BLE_DBG_P2P_STM_MSG("-- GATT : REBOOT REQUEST RECEIVED\n");
              Notification.P2P_Evt_Opcode = P2PS_STM_BOOT_REQUEST_EVT;
              Notification.DataTransfered.Length=attribute_modified->Attr_Data_Length;
              Notification.DataTransfered.pPayload=attribute_modified->Attr_Data;
              P2PS_STM_App_Notification(&Notification);
            }
#endif
        }
        break;
//        case ACI_ATT_READ_BLOB_RESP_VSEVT_CODE:
//        	printf("yeah\n\r");
//        	Notification.P2P_Evt_Opcode = P2PS_STM_READ_EVT;
//        	P2PS_STM_App_Notification(&Notification);
//        	break;

        default:
          break;
      }
    }
    break; /* HCI_HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE_SPECIFIC */

    default:
      break;
  }

  return(return_value);
}/* end SVCCTL_EvtAckStatus_t */


/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void P2PS_STM_Init(void)
{
 
  Char_UUID_t  uuid16;
//  uint8_t data_buffer[] = {3};

  /**
   *	Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(PeerToPeer_Event_Handler);
  
    /**
     *  Peer To Peer Service
     *
     * Max_Attribute_Records = 2*no_of_char + 1
     * service_max_attribute_record = 1 for Peer To Peer service +
     *                                2 for P2P Write characteristic +
     *                                2 for P2P Notify characteristic +
     *                                1 for client char configuration descriptor +
     *                                
     */
    COPY_DATA_TRANSFER_SERVICE_UUID(uuid16.Char_UUID_128);
    aci_gatt_add_service(UUID_TYPE_128,
                      (Service_UUID_t *) &uuid16,
                      PRIMARY_SERVICE,
                      8,
                      &(aPeerToPeerContext.data_transfer_service_handle));

    /**
     *  Add LED/ tx Characteristic
     */
//    COPY_DATA_TRANSFER_TX_CHAR_UUID(uuid16.Char_UUID_128);
//    aci_gatt_add_char(aPeerToPeerContext.data_transfer_service_handle,
//                      UUID_TYPE_128, &uuid16,
//                      2,
//					  CHAR_PROP_NOTIFY,
//                      ATTR_PERMISSION_NONE,
//					  0,
//                      10, /* encryKeySize */
//                      1, /* isVariable */
//                      &(aPeerToPeerContext.data_transfer_tx_char_handle));

    /**
     *   Add Button/ rx Characteristic
     */
    COPY_DATA_TRANSFER_RX_CHAR_UUID(uuid16.Char_UUID_128);
    aci_gatt_add_char(aPeerToPeerContext.data_transfer_service_handle,
                      UUID_TYPE_128, &uuid16,
                      2,
					  CHAR_PROP_WRITE_WITHOUT_RESP|CHAR_PROP_READ|CHAR_PROP_WRITE,
                      ATTR_PERMISSION_NONE,
                      GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
                      10, /* encryKeySize */
                      1, /* isVariable: 1 */
                      &(aPeerToPeerContext.data_transfer_rx_char_handle));
//    aci_gatt_update_char_value(aPeerToPeerContext.data_transfer_service_handle,
//   						 aPeerToPeerContext.data_transfer_rx_char_handle,
//   						  0, /* charValOffset */
//   						 3, /* charValueLen */
//   						 data_buffer);

 
#if(BLE_CFG_OTA_REBOOT_CHAR != 0)      
    /**
     *  Add Boot Request Characteristic
     */
    aci_gatt_add_char(aPeerToPeerContext.data_transfer_service_handle,
                      BM_UUID_LENGTH,
                      (Char_UUID_t *)BM_REQ_CHAR_UUID,
                      BM_REQ_CHAR_SIZE,
                      CHAR_PROP_WRITE_WITHOUT_RESP,
                      ATTR_PERMISSION_NONE,
                      GATT_NOTIFY_ATTRIBUTE_WRITE,
                      10,
                      0,
                      &(aPeerToPeerContext.RebootReqCharHdle));
#endif    

    
  return;
}

/**
 * @brief  Characteristic update
 * @param  UUID: UUID of the characteristic
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 * 
 */

tBleStatus receive_data(uint8_t * data_buffer, uint8_t no_bytes)
{
  //printf("\n\r...DATA HAS\n\r");
  tBleStatus result = BLE_STATUS_INVALID_PARAMS;
  for(int i=0;i<no_bytes;i++){
	  aPeerToPeerContext.rcv_data[i] = data_buffer[i];
	 // printf("data is this is in p2p_stm file: %d \n\r",aPeerToPeerContext.rcv_data[i]);
  }

  return result; //return rcv_data
}

tBleStatus send_data(uint8_t * data_buffer, uint8_t no_bytes)
{
  tBleStatus result = BLE_STATUS_INVALID_PARAMS;

 result = aci_gatt_update_char_value(aPeerToPeerContext.data_transfer_service_handle,
						 aPeerToPeerContext.data_transfer_rx_char_handle,
						  0, /* charValOffset */
						 2, /* charValueLen */
						 data_buffer);


  return result;
}

tBleStatus P2PS_STM_App_Update_Char(uint16_t UUID, uint8_t *pPayload)
{
  tBleStatus result = BLE_STATUS_INVALID_PARAMS;
  switch(UUID)
  {
    case P2P_NOTIFY_CHAR_UUID:

     result = aci_gatt_update_char_value(aPeerToPeerContext.data_transfer_service_handle,
                             aPeerToPeerContext.data_transfer_rx_char_handle,
                              0, /* charValOffset */
                             2, /* charValueLen */
                             (uint8_t *)  pPayload);
    case P2P_WRITE_CHAR_UUID:
	result = aci_gatt_update_char_value(aPeerToPeerContext.data_transfer_service_handle,
								 aPeerToPeerContext.data_transfer_tx_char_handle,
								  0, /* charValOffset */
								 2, /* charValueLen */
								 (uint8_t *)  pPayload);

      break;

    default:
      break;
  }

  return result;
}/* end P2PS_STM_Init() */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
