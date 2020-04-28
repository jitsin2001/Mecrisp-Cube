/**
 *  @brief
 *      Cable Replacement Client CRC (for GAP Central Role).
 *
 *      Using CMSIS-RTOS queues as buffers.
 *  @file
 *      uart.c
 *  @author
 *      Peter Schmid, peter@spyr.ch
 *  @date
 *      2020-02-19
 *  @remark
 *      Language: C, STM32CubeIDE GCC
 *  @copyright
 *      Peter Schmid, Switzerland
 *
 *      This project Mecrsip-Cube is free software: you can redistribute it
 *      and/or modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation, either version 3 of
 *      the License, or (at your option) any later version.
 *
 *      Mecrsip-Cube is distributed in the hope that it will be useful, but
 *      WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *      General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with Mecrsip-Cube. If not, see http://www.gnu.org/licenses/.
 */

// System include files
// ********************
#include "cmsis_os.h"
#include <stdio.h>

// Application include files
// *************************
#include "app_common.h"
#include "main.h"
#include "crc_app.h"
#include "app_ble.h"

#include "dbg_trace.h"
#include "ble.h"
#include "crs_stm.h"


// Rx/Tx Buffer Length
// *******************
#define CRC_TX_BUFFER_LENGTH	1024
#define CRC_RX_BUFFER_LENGTH	1024


// Private function prototypes
// ***************************
static void CRC_Thread(void *argument);

// Global Variables
// ****************

// RTOS resources
// **************

// Definitions for CRC thread
static osThreadId_t CRC_ThreadId;
static const osThreadAttr_t crc_ThreadAttr = {
		.name = "CRC_Thread",
		.priority = (osPriority_t) osPriorityHigh,
		.stack_size = 512 * 2
};

static osMutexId_t CRC_MutexID;
const osMutexAttr_t CRC_MutexAttr = {
		NULL,				// no name required
		osMutexPrioInherit,	// attr_bits
		NULL,				// memory for control block
		0U					// size for control block
};

// Definitions for TxQueue
static osMessageQueueId_t CRC_TxQueueId;
static const osMessageQueueAttr_t crc_TxQueue_attributes = {
		.name = "CRC_TxQueue"
};

// Definitions for RxQueue
static osMessageQueueId_t CRC_RxQueueId;
static const osMessageQueueAttr_t crc_RxQueue_attributes = {
		.name = "CRC_RxQueue"
};


/* Private typedef -----------------------------------------------------------*/
typedef enum {
	CRC_UNINITIALIZED                                 = 0,
	CRC_INITIALIZED                                   = 1,
	CRC_IDLE                                          = 2,
	CRC_DISCOVER_CHARACS                              = 4,
	CRC_DISCOVER_DESC                                 = 5,
	CRC_READ_TX                                       = 6,
	CRC_READ_RX                                       = 7,
	CRC_READ_RX_CCC                                   = 8,
	CRC_WRITE_TX                                      = 9,
	CRC_ENABLE_RX_NOTIFICATION                        = 10,
	CRC_DISABLE_RX_NOTIFICATION                       = 11,
	CRC_CONNECTED                                     = 12,
	CRC_CONNECTED_ADVERTISING                         = 13
}  CRC_SVC_ProfileState;

typedef struct {
	/**
	 * state of the CRC
	 * state machine
	 */
	CRC_SVC_ProfileState state;

	/**
	 * connection handle
	 */
	uint16_t connHandle;

	/**
	 * handle of the CRC service
	 */
	uint16_t ServiceHandle;

	/**
	 * end handle of the CRC service
	 */
	uint16_t ServiceEndHandle;

	/**
	 * handle of the TX characteristic
	 *
	 */
	uint16_t TXCharHdle;

	/**
	 * handles of the RX characteristic
	 *
	 */
	uint16_t RXCharHdle;

	/**
	 * handles of the RX Client Characteristic Configuration descriptor
	 *
	 */
	uint16_t RXCCCDescHdle;

}CRC_Context_t;


/* Private defines -----------------------------------------------------------*/
#define UNPACK_2_BYTE_PARAMETER(ptr)  \
        (uint16_t)((uint16_t)(*((uint8_t *)ptr))) |   \
        (uint16_t)((((uint16_t)(*((uint8_t *)ptr + 1))) << 8))

#define READ_RX                                                                0
#define READ_TX                                                                1
#define READ_RX_CCC_DESC                                                       2
          
#define WRITE_TX                                                               0
#define WRITE_ENABLE_RX_NOTIFICATION                                           1
#define WRITE_DISABLE_RX_NOTIFICATION                                          2

#define RX_BUFFER_SIZE                                                        64
#define MAX_STRING_SIZE                                                       20

#define TX_CHAR                                                                1
#define RX_CHAR                                                                2


/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("BLE_APP_CONTEXT") static CRC_Context_t CRC_Context[CFG_MAX_CONNECTION];
static uint8_t waitForComplete;
static char szString[MAX_STRING_SIZE+1];


/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

// Public Functions
// ****************

/**
 * @brief  Feature Characteristic write
 * @param  index: characteristic index
 * @retval None
 */
static void CRCAPP_Write_Char(uint8_t index) {
	uint8_t con_index = 0;

	con_index = 0;
	while((con_index < CFG_MAX_CONNECTION) &&
			(CRC_Context[con_index].state != CRC_IDLE)) {
		switch(index) {
		case WRITE_TX:
			APP_DBG_MSG("WRITE_TX\n");
			if(CRC_Context[con_index].TXCharHdle > 0) {
				CRC_Context[con_index].state = CRC_WRITE_TX;
				APP_DBG_MSG("CRC_CONNECTED -> CRC_WRITE_TX\n");
				//	UTIL_SEQ_SetTask( 1<<CFG_TASK_CRC_DISCOVERY_REQ_ID, CFG_SCH_PRIO_0);
			}
			break;

		case WRITE_ENABLE_RX_NOTIFICATION:
			APP_DBG_MSG("WRITE_ENABLE_RX_NOTIFICATION\n");
			if(CRC_Context[con_index].RXCharHdle > 0) {
				CRC_Context[con_index].state = CRC_ENABLE_RX_NOTIFICATION;
				APP_DBG_MSG("CRC_CONNECTED -> CRC_ENABLE_RX_NOTIFICATION\n");
//				UTIL_SEQ_SetTask( 1<<CFG_TASK_CRC_DISCOVERY_REQ_ID, CFG_SCH_PRIO_0);
				// ??
			}
			break;

		case WRITE_DISABLE_RX_NOTIFICATION:
			APP_DBG_MSG("WRITE_DISABLE_RX_NOTIFICATION\n");
			if(CRC_Context[con_index].RXCharHdle > 0) {
				CRC_Context[con_index].state = CRC_DISABLE_RX_NOTIFICATION;
				APP_DBG_MSG("CRC_CONNECTED -> CRC_DISABLE_RX_NOTIFICATION\n");
//				UTIL_SEQ_SetTask( 1<<CFG_TASK_CRC_DISCOVERY_REQ_ID, CFG_SCH_PRIO_0);
				// ??
			}
			break;

		default:
			break;
		}
		con_index++;
	}

}/* end CRCAPP_Write_Char() */



/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t CRCAPP_Event_Handler(void *Event) {
	SVCCTL_EvtAckStatus_t return_value;
	hci_event_pckt *event_pckt;
	evt_blue_aci *blue_evt;

	return_value = SVCCTL_EvtNotAck;
	event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

	switch(event_pckt->evt) {
	case EVT_VENDOR:
		blue_evt = (evt_blue_aci*)event_pckt->data;

		APP_DBG_MSG("EVT_VENDOR: 0x%x!\n",
				blue_evt->ecode);

		switch(blue_evt->ecode) {
		case EVT_BLUE_ATT_READ_BY_GROUP_TYPE_RESP: {
			aci_att_read_by_group_type_resp_event_rp0 *pr = (void*)blue_evt->data;
			uint8_t numServ, i, idx;
			uint16_t handle;
			uint8_t index;

			handle = pr->Connection_Handle;
			index = 0;
			while((index < CFG_MAX_CONNECTION) &&
					(CRC_Context[index].state != CRC_IDLE)) {
				APP_BLE_ConnStatus_t status;

//				status = APP_BLE_Get_Client_Connection_Status(CRC_Context[index].connHandle);
				// Server_Connection ??
				APP_DBG_MSG("Handle 0x%x status: %d !\n",
						CRC_Context[index].connHandle, status);
				if(((CRC_Context[index].state == CRC_CONNECTED) ||
						(CRC_Context[index].state == CRC_CONNECTED_ADVERTISING))&&
						(status == APP_BLE_IDLE)) {
					/* Handle deconnected */
					APP_DBG_MSG("Handle 0x%x no more connected, connection table updated !\n",
							CRC_Context[index].connHandle);
					CRC_Context[index].state = CRC_IDLE;
					CRC_Context[index].connHandle = 0xFFFF;
					waitForComplete = 1;
					break;
				}
				index++;
			}

			if(index < CFG_MAX_CONNECTION) {
				CRC_Context[index].connHandle = handle;

				APP_DBG_MSG("New handle 0x%x in connection table index: %d!\n",
						CRC_Context[index].connHandle, index);
				numServ = pr->Data_Length / pr->Attribute_Data_Length;

				/* the event data will be
				 * 2bytes start handle
				 * 2bytes end handle
				 * 2 or 16 bytes data
				 * we are interested only if the UUID is 128 bits.
				 * So check if the data length is 20
				 */
				if (pr->Attribute_Data_Length == 20) {
					idx = 16;
					for (i = 0; i < numServ; i++) {
						uint8_t j;
						uint8_t service_uuid[] = { CRS_STM_UUID128 };
						uint8_t result = TRUE;

						for(j = 0; j < pr->Attribute_Data_Length - 4; j++) {
							if(pr->Attribute_Data_List[j+4] != service_uuid[(pr->Attribute_Data_Length-5) - j]) {
								result = FALSE;
								APP_DBG_MSG("Service UUID is not a Cable Replacement Service\n");
								break;
							}
						}
						if(result == TRUE) {
							APP_DBG_MSG("EVT_BLUE_ATT_READ_BY_GROUP_TYPE_RESP, first index in CRC_IDLE state: %d\n", index);

							CRC_Context[index].ServiceHandle =
									UNPACK_2_BYTE_PARAMETER(&pr->Attribute_Data_List[idx-16]);
							CRC_Context[index].ServiceEndHandle =
									UNPACK_2_BYTE_PARAMETER (&pr->Attribute_Data_List[idx-14]);
							APP_DBG_MSG("*************************************************************************************************************************\n");
							APP_DBG_MSG("* Found Cable Replacement service start handle 0x%04x end handle 0x%04x\n",
									CRC_Context[index].ServiceHandle,
									CRC_Context[index].ServiceEndHandle);
							APP_DBG_MSG("*************************************************************************************************************************\n");
							waitForComplete = 1;
						}
						idx += 20;
					}
				}
			} else {
				APP_DBG_MSG("EVT_BLUE_ATT_READ_BY_GROUP_TYPE_RESP, failed no free index in connection table !\n");
			}
		}
		break; /*EVT_BLUE_ATT_READ_BY_GROUP_TYPE_RESP*/

		case EVT_BLUE_ATT_READ_BY_TYPE_RESP: {
			aci_att_read_by_type_resp_event_rp0 *pr = (void*)blue_evt->data;
			uint8_t idx;
			uint16_t handle;
			uint8_t index;

			index = 0;
			while((index < CFG_MAX_CONNECTION) &&
					(CRC_Context[index].connHandle != pr->Connection_Handle)) {
				index++;
			}

			if(index < CFG_MAX_CONNECTION) {
				handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[0]);
				if((CRC_Context[index].state == CRC_DISCOVER_CHARACS) &&
						(handle >= CRC_Context[index].ServiceHandle) &&
						(handle <= CRC_Context[index].ServiceEndHandle)) {
					/* Event for CRC Client */
					APP_DBG_MSG("EVT_BLUE_ATT_READ_BY_TYPE_RESP\n");

					/* the event data will be
					 * 2 bytes start handle
					 * 1 byte char properties
					 * 2 bytes handle
					 * 2 or 16 bytes data
					 */
					idx = 17;
					/* we are interested in only 128 bits UUIDs */
					if (pr->Handle_Value_Pair_Length == 21) {
						while(pr->Data_Length > 0) {
							uint8_t tx_uuid[] = { CRS_STM_TX_UUID128 };
							uint8_t rx_uuid[] = { CRS_STM_RX_UUID128 };
							uint8_t i, result = TX_CHAR | RX_CHAR;

							/* store the characteristic handle not the attribute handle */
							handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx-14]);
							for(i = 0; i < pr->Handle_Value_Pair_Length - 5; i++) {
								if(pr->Handle_Value_Pair_Data[i+5] != tx_uuid[(pr->Handle_Value_Pair_Length-6) - i]) {
									APP_DBG_MSG("Characteristic UUID is not a Tx UUID Characteristic\n");
									APP_DBG_MSG("pr->Handle_Value_Pair_Data[i+5]: 0x%x != tx_uuid[(pr->Handle_Value_Pair_Length-6) - i]: 0x%x\n",
											pr->Handle_Value_Pair_Data[i+5],
											tx_uuid[(pr->Handle_Value_Pair_Length-6) - i]);
									result &= RX_CHAR;    /* Not a TX Characteristic */
								}
								if(pr->Handle_Value_Pair_Data[i+5] != rx_uuid[(pr->Handle_Value_Pair_Length-6) - i]) {
									APP_DBG_MSG("Characteristic UUID is not a Rx UUID Characteristic\n");
									APP_DBG_MSG("pr->Handle_Value_Pair_Data[i+5]: 0x%x != rx_uuid[(pr->Handle_Value_Pair_Length-6) - i]: 0x%x\n",
											pr->Handle_Value_Pair_Data[i+5],
											rx_uuid[(pr->Handle_Value_Pair_Length-6) - i]);
									result &= TX_CHAR;   /* Not a RX Characteristic */
								}
								if(result == 0) {
									break;
								}
							}
							if(result == TX_CHAR) {
								APP_DBG_MSG("*************************************************************************************************************************\n");
								APP_DBG_MSG("TX start handle 0x%04x\n", UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx-17]));
								APP_DBG_MSG("TX properties   0x%02x\n", pr->Handle_Value_Pair_Data[idx-15]);
								APP_DBG_MSG("TX handle       0x%04x\n", handle);
								APP_DBG_MSG("TX uuid           0x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
										tx_uuid[0], tx_uuid[1], tx_uuid[2],  tx_uuid[3],  tx_uuid[4],  tx_uuid[5],  tx_uuid[6],  tx_uuid[7],
										tx_uuid[8] ,tx_uuid[9], tx_uuid[10], tx_uuid[11], tx_uuid[12], tx_uuid[13], tx_uuid[14], tx_uuid[15]);
								APP_DBG_MSG("*************************************************************************************************************************\n");
								CRC_Context[index].TXCharHdle = handle;
							}

							if(result == RX_CHAR) {
								APP_DBG_MSG("*************************************************************************************************************************\n");
								APP_DBG_MSG("RX start handle 0x%04x\n", UNPACK_2_BYTE_PARAMETER(&pr->Handle_Value_Pair_Data[idx-17]));
								APP_DBG_MSG("RX properties   0x%02x\n", pr->Handle_Value_Pair_Data[idx-15]);
								APP_DBG_MSG("RX handle       0x%04x\n", handle);
								APP_DBG_MSG("RX uuid           0x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
										rx_uuid[0], rx_uuid[1], rx_uuid[2],  rx_uuid[3],  rx_uuid[4],  rx_uuid[5],  rx_uuid[6],  rx_uuid[7],
										rx_uuid[8] ,rx_uuid[9], rx_uuid[10], rx_uuid[11], rx_uuid[12], rx_uuid[13], rx_uuid[14], rx_uuid[15]);
								APP_DBG_MSG("*************************************************************************************************************************\n");
								CRC_Context[index].RXCharHdle = handle;
							}
							pr->Data_Length -= 21;
							idx += 21;
						} /* end while(pr->Event_Data_Length > 0) */
					} /* end if (pr->Handle_Value_Pair_Data == 21) */
				} /* end if(CRC_Context[index].state ... */
			} else {
				APP_DBG_MSG("EVT_BLUE_ATT_READ_BY_TYPE_RESP, failed handle not found in connection table !\n");
			} /* if(index < CFG_MAX_CONNECTION) */
		}
		break; /*EVT_BLUE_ATT_READ_BY_TYPE_RESP*/

		case EVT_BLUE_ATT_FIND_INFORMATION_RESP: {
			aci_att_find_info_resp_event_rp0 *pr = (void*)blue_evt->data;
			uint8_t numDesc, idx, i;
			uint16_t uuid, handle;
			uint8_t index;

			index = 0;
			while((index < CFG_MAX_CONNECTION) &&
					(CRC_Context[index].connHandle != pr->Connection_Handle)) {
				index++;
			}

			if(index < CFG_MAX_CONNECTION) {
				handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_UUID_Pair[0]);
				if((CRC_Context[index].state == CRC_DISCOVER_DESC) &&
						(handle >= CRC_Context[index].ServiceHandle) &&
						(handle <= CRC_Context[index].ServiceEndHandle)) {
					/* Event for CRC Client */
					APP_DBG_MSG("EVT_BLUE_ATT_FIND_INFORMATION_RESP\n");

					waitForComplete = 1;
					/*
					 * event data will be of the format
					 * 2 bytes handle
					 * 2 bytes UUID
					 */

					numDesc = (pr->Event_Data_Length) / 4;
					APP_DBG_MSG("CRC Number of descriptors: %d\n", numDesc);

					/* we are interested only in 16 bit UUIDs */
					idx = 0;
					if (pr->Format == UUID_TYPE_16) {
						for (i = 0; i < numDesc; i++) {
							handle = UNPACK_2_BYTE_PARAMETER(&pr->Handle_UUID_Pair[idx]);
							uuid = UNPACK_2_BYTE_PARAMETER(&pr->Handle_UUID_Pair[idx+2]);

							APP_DBG_MSG("UUID: 0x%x Handle: 0x%x\n", uuid, handle);

							if(uuid == CLIENT_CHAR_CONFIG_DESCRIPTOR_UUID) {
								if( (CRC_Context[index].RXCharHdle)+1 == handle) {
									APP_DBG_MSG("*************************************************************************************************************************\n");
									APP_DBG_MSG("RX Client Characteristic Configuration Desc handle 0x%04x\n", handle);
									APP_DBG_MSG("RX Client Characteristic Configuration Desc uuid   0x%04x\n", uuid);
									APP_DBG_MSG("*************************************************************************************************************************\n");
									CRC_Context[index].RXCCCDescHdle = handle;
								}
							} /* end if(uuid == CLIENT_CHAR_CONFIG_DESCRIPTOR_UUID) */
							idx += 4;
						} /* end for (i = 0; i < numDesc; i++) */
					} /* end if (pr->Format == UUID_TYPE_16) */
				} /* end if(CRC_Context[index].connHandle == pr->Connection_Handle) */
			} else {
				APP_DBG_MSG("EVT_BLUE_ATT_FIND_INFORMATION_RESP, failed handle not found in connection table !\n");
			}
		}
		break; /*EVT_BLUE_ATT_FIND_INFORMATION_RESP*/

		case EVT_BLUE_ATT_READ_RESP: {
			aci_att_read_resp_event_rp0 *pr = (void*)blue_evt->data;
			uint8_t index;

			index = 0;
			while((index < CFG_MAX_CONNECTION) &&
					(CRC_Context[index].connHandle != pr->Connection_Handle)) {
				index++;
			}

			if(index < CFG_MAX_CONNECTION) {
				if(CRC_Context[index].state == CRC_READ_TX) {
					/* Event for CR Client */
					APP_DBG_MSG("EVT_BLUE_ATT_READ_RESP\n");
					waitForComplete = 1;
					APP_DBG_MSG("*************************************************************************************************************************\n");
					APP_DBG_MSG("CRC TX 0x%x:\n",
							CRC_Context[index].TXCharHdle);
					APP_DBG_MSG("*************************************************************************************************************************\n");
				} else if(CRC_Context[index].state == CRC_READ_RX) {
					/* Event for CR Client */
					APP_DBG_MSG("EVT_BLUE_ATT_READ_RESP\n");
					waitForComplete = 1;
					APP_DBG_MSG("*************************************************************************************************************************\n");
					APP_DBG_MSG("CRC RX 0x%x:\n",
							CRC_Context[index].RXCharHdle);
					APP_DBG_MSG("*************************************************************************************************************************\n");
				} else if(CRC_Context[index].state == CRC_READ_RX_CCC) {
					/* Event for CR Client */
					APP_DBG_MSG("EVT_BLUE_ATT_READ_RESP\n");
					waitForComplete = 1;
					APP_DBG_MSG("*************************************************************************************************************************\n");
					APP_DBG_MSG("CRC RX CCC 0x%x: 0x%04x\n",
							CRC_Context[index].RXCCCDescHdle,
							pr->Attribute_Value[0] + (pr->Attribute_Value[1] << 8));
					APP_DBG_MSG("*************************************************************************************************************************\n");
				}
			} else {
				APP_DBG_MSG("EVT_BLUE_ATT_READ_RESP, failed handle not found in connection table !\n");
			}
		}
		break; /*EVT_BLUE_ATT_READ_RESP*/

		case EVT_BLUE_GATT_NOTIFICATION: {
			aci_gatt_notification_event_rp0 *pr = (void*)blue_evt->data;
			uint8_t index;

			index = 0;
			while((index < CFG_MAX_CONNECTION) &&
					(CRC_Context[index].connHandle != pr->Connection_Handle)) {
				index++;
			}

			if(index < CFG_MAX_CONNECTION) {
				if((pr->Attribute_Handle >= CRC_Context[index].ServiceHandle) &&
						(pr->Attribute_Handle <= CRC_Context[index].ServiceEndHandle)) {
					/* Event for CRC Client */
					APP_DBG_MSG("EVT_BLUE_GATT_NOTIFICATION on connection handle 0x%x\n",
							pr->Connection_Handle);
					waitForComplete = 1;
					if(pr->Attribute_Handle == CRC_Context[index].RXCharHdle) {
						uint8_t text[21], i;

						/* the event buffer will have the data as follows:
						 * category ID - 1 byte
						 * number of alerts for that category - 1 byte
						 * text string information - optional(varying number of bytes : the rest
						 * of the available bytes as indicated by length has to be considered
						 * as the string
						 */
						APP_DBG_MSG("pr->Attribute_Value_Length %d\n", pr->Attribute_Value_Length);
						for(i = 0; i < pr->Attribute_Value_Length; i++) {
							text[i] = pr->Attribute_Value[i];
						}
						text[pr->Attribute_Value_Length] = '\0';
						APP_DBG_MSG("*************************************************************************************************************************\n");
						APP_DBG_MSG("%s\n", &(text[0]));
						APP_DBG_MSG("*************************************************************************************************************************\n");
//						PrintPcCrt(PosXRx, PosYRx, "%s", &(text[0]));
						// ??
					}
				}
			} else {
				APP_DBG_MSG("EVT_BLUE_GATT_NOTIFICATION, failed handle not found in connection table !\n");
			}
		}
		break; /*EVT_BLUE_GATT_INDICATION*/

		case EVT_BLUE_GATT_PROCEDURE_COMPLETE: {
			aci_gatt_proc_complete_event_rp0 *pr = (void*)blue_evt->data;

			if(waitForComplete != 0) {
				uint8_t index;

				index = 0;
				while((index < CFG_MAX_CONNECTION) &&
						(CRC_Context[index].connHandle != pr->Connection_Handle)) {
					index++;
				}

				if(index < CFG_MAX_CONNECTION) {
					waitForComplete = 0;
					/* Event for CR Client */
					APP_DBG_MSG("EVT_BLUE_GATT_PROCEDURE_COMPLETE\n");

					switch(CRC_Context[index].state) {
					case CRC_IDLE: {
						CRC_Context[index].state = CRC_DISCOVER_CHARACS;
						APP_DBG_MSG("CRC_IDLE -> CRC_DISCOVER_CHARACS\n");
//						UTIL_SEQ_SetTask( 1<<CFG_TASK_CRC_DISCOVERY_REQ_ID, CFG_SCH_PRIO_0);
						// ??
					}
					break;

					case CRC_DISCOVER_CHARACS: {
						CRC_Context[index].state = CRC_DISCOVER_DESC;
						APP_DBG_MSG("CRC_DISCOVER_CHARACS -> CRC_DISCOVER_DESC\n");
//						UTIL_SEQ_SetTask( 1<<CFG_TASK_CRC_DISCOVERY_REQ_ID, CFG_SCH_PRIO_0);
						// ??
					}
					break;

					case CRC_DISCOVER_DESC: {
						CRC_Context[index].state = CRC_ENABLE_RX_NOTIFICATION;
						APP_DBG_MSG("CRC_DISCOVER_DESC -> CRC_ENABLE_RX_NOTIFICATION\n");
//						UTIL_SEQ_SetTask( 1<<CFG_TASK_CRC_DISCOVERY_REQ_ID, CFG_SCH_PRIO_0);
						// ??
					}
					break;

					case CRC_READ_TX: {
						CRC_Context[index].state = CRC_CONNECTED;
						APP_DBG_MSG("CRC_READ_TX -> CRC_CONNECTED\n");
//						UTIL_SEQ_SetTask( 1<<CFG_TASK_CRC_DISCOVERY_REQ_ID, CFG_SCH_PRIO_0);
						// ??
					}
					break;

					case CRC_READ_RX: {
						CRC_Context[index].state = CRC_CONNECTED;
						APP_DBG_MSG("CRC_READ_RX -> CRC_CONNECTED\n");
//						UTIL_SEQ_SetTask( 1<<CFG_TASK_CRC_DISCOVERY_REQ_ID, CFG_SCH_PRIO_0);
						// ??
					}
					break;


					case CRC_READ_RX_CCC: {
						CRC_Context[index].state = CRC_CONNECTED;
						APP_DBG_MSG("CRC_READ_RX_CCC -> CRC_CONNECTED\n");
//						UTIL_SEQ_SetTask( 1<<CFG_TASK_CRC_DISCOVERY_REQ_ID, CFG_SCH_PRIO_0);
						// ??
					}
					break;

					case CRC_WRITE_TX: {
						CRC_Context[index].state = CRC_CONNECTED;
						APP_DBG_MSG("CRC_WRITE_TX -> CRC_CONNECTED\n");
//						UTIL_SEQ_SetTask( 1<<CFG_TASK_CRC_DISCOVERY_REQ_ID, CFG_SCH_PRIO_0);
						// ??
					}
					break;

					case CRC_ENABLE_RX_NOTIFICATION: {
						CRC_Context[index].state = CRC_CONNECTED;
						APP_DBG_MSG("CRC_ENABLE_RX_NOTIFICATION -> CRC_CONNECTED\n");
//						UTIL_SEQ_SetTask( 1<<CFG_TASK_CRC_DISCOVERY_REQ_ID, CFG_SCH_PRIO_0);
						// ??
					}
					break;

					case CRC_DISABLE_RX_NOTIFICATION: {
						CRC_Context[index].state = CRC_CONNECTED;
						APP_DBG_MSG("CRC_DISABLE_RX_NOTIFICATION -> CRC_CONNECTED\n");
//						UTIL_SEQ_SetTask( 1<<CFG_TASK_CRC_DISCOVERY_REQ_ID, CFG_SCH_PRIO_0);
						// ??
					}
					break;

					default:
						break;
					} // switch CRC_Context[index].state
				} else {
					APP_DBG_MSG("EVT_BLUE_GATT_PROCEDURE_COMPLETE failed, not found handle in connection table !\n");
				}
			}
		}
		break; /*EVT_BLUE_GATT_PROCEDURE_COMPLETE*/

		case EVT_BLUE_GATT_ERROR_RESP: {
			aci_gatt_error_resp_event_rp0 *pr = (void*)blue_evt->data;
			uint8_t index;

			index = 0;
			while((index < CFG_MAX_CONNECTION) &&
					(CRC_Context[index].connHandle != pr->Connection_Handle)) {
				index++;
			}

			if(index < CFG_MAX_CONNECTION) {
				if((CRC_Context[index].state == CRC_DISCOVER_CHARACS) &&
						(waitForComplete == 0) &&
						(pr->Attribute_Handle >= CRC_Context[index].ServiceHandle) &&
						(pr->Attribute_Handle <= CRC_Context[index].ServiceEndHandle)) {
					APP_DBG_MSG("EVT_BLUE_GATT_ERROR_RESP\n");
					waitForComplete = 1;
				} else if((CRC_Context[index].state == CRC_IDLE) &&
						(waitForComplete == 0) &&
						(pr->Attribute_Handle >= CRC_Context[index].ServiceHandle) &&
						(pr->Attribute_Handle <= CRC_Context[index].ServiceEndHandle)) {
					APP_DBG_MSG("EVT_BLUE_GATT_ERROR_RESP\n");
					waitForComplete = 1;
				}
			} else {
				APP_DBG_MSG("EVT_BLUE_GATT_ERROR_RESP, not found handle in connection table !\n");
			}
		}
		break; /*EVT_BLUE_GATT_ERROR_RESP*/

		default:
			break;
	}
	break; /* HCI_EVT_VENDOR_SPECIFIC */

	default:
		break;
	}

	return(return_value);
}/* end CRCAPP_Event_Handler */


// Public Functions
// ****************

/**
 *  @brief
 *      Initializes the CRC.
 *  @return
 *      None
 */
void CRCAPP_Init(void) {
	uint8_t index;

	// Create the queue(s)
	// creation of TxQueue
	CRC_TxQueueId = osMessageQueueNew(CRC_TX_BUFFER_LENGTH, sizeof(uint8_t),
			&crc_TxQueue_attributes);
	if (CRC_TxQueueId == NULL) {
		Error_Handler();
	}
	// creation of RxQueue
	CRC_RxQueueId = osMessageQueueNew(CRC_RX_BUFFER_LENGTH, sizeof(uint8_t),
			&crc_RxQueue_attributes);
	if (CRC_RxQueueId == NULL) {
		Error_Handler();
	}

	CRC_MutexID = osMutexNew(&CRC_MutexAttr);
	if (CRC_MutexID == NULL) {
		Error_Handler();
	}

	// creation of CRC_TxThread
	CRC_ThreadId = osThreadNew(CRC_Thread, NULL, &crc_ThreadAttr);
	if (CRC_ThreadId == NULL) {
		Error_Handler();
	}

	waitForComplete = 1;

	for(index = 0; index < CFG_MAX_CONNECTION; index++) {
		CRC_Context[index].state = CRC_IDLE;
		CRC_Context[index].connHandle = 0xFFFF;
	}

	/**
	 *	Register the event handler to the BLE controller
	 */
	SVCCTL_RegisterCltHandler(CRCAPP_Event_Handler);
}


/**
 * @brief  Service update
 * @param  None
 * @retval None
 */
void CRCAPP_Update_Service() {
	tBleStatus result;
	uint8_t index;

	index = 0;
	while ((index < CFG_MAX_CONNECTION) &&
			(CRC_Context[index].state != CRC_IDLE)) {
		switch (CRC_Context[index].state) {
		case CRC_CONNECTED: {
			APP_DBG_MSG("CRC_CONNECTED\n");
			if (APP_BLE_Get_Client_Connection_Status(CRC_Context[index].connHandle) == APP_BLE_IDLE) {
				APP_DBG_MSG("Handle deconnected !\n");
				CRC_Context[index].state = CRC_IDLE;
				CRC_Context[index].connHandle = 0xFFFF;
			}
		}
		break;

		case CRC_DISCOVER_CHARACS:{
			APP_DBG_MSG("CRC_DISCOVER_CHARACS\n");

			result = aci_gatt_disc_all_char_of_service(CRC_Context[index].connHandle,
					CRC_Context[index].ServiceHandle,
					CRC_Context[index].ServiceEndHandle);

			if ( result == BLE_STATUS_SUCCESS ) {
				APP_DBG_MSG("All characteristics discovery sent successfully \n");
			} else {
				APP_DBG_MSG("All characteristics discovery sending failed with result: 0x%x\n", result);
				if (result == BLE_STATUS_NOT_ALLOWED) {
					CRC_Context[index].state = CRC_IDLE; /* redo a CRC_DISCOVER_CHARACS */
					waitForComplete = 1;
					APP_DBG_MSG("BLE_STATUS_NOT_ALLOWED do another CRC_DISCOVER_CHARACS\n");
				}
			}
		}
		break;

		case CRC_DISCOVER_DESC: {
			APP_DBG_MSG("CRC_DISCOVER_DESC\n");

			result = aci_gatt_disc_all_char_desc(CRC_Context[index].connHandle,
					CRC_Context[index].ServiceHandle,
					CRC_Context[index].ServiceEndHandle);

			if ( result == BLE_STATUS_SUCCESS ) {
				APP_DBG_MSG("Descriptors discovery sent successfully \n");
			} else {
				APP_DBG_MSG("Descriptors discovery sending failed \n");
				if (result == BLE_STATUS_NOT_ALLOWED) {
					CRC_Context[index].state = CRC_DISCOVER_CHARACS; /* redo a CRC_DISCOVER_DESC */
					waitForComplete = 1;
					APP_DBG_MSG("BLE_STATUS_NOT_ALLOWED do another CRC_DISCOVER_DESC\n");
				}
			}
		}
		break;

		case CRC_READ_RX: {
			APP_DBG_MSG("CRC_READ_TEMPERATURE_TYPE\n");

			result = aci_gatt_read_char_value(CRC_Context[index].connHandle,
					CRC_Context[index].RXCharHdle);
			if ( result == BLE_STATUS_SUCCESS ) {
				APP_DBG_MSG("Read CRC Temperature Type sent successfully \n");
			} else {
				APP_DBG_MSG("Read CRC RX sending failed \n");
				if (result == BLE_STATUS_NOT_ALLOWED) {
					CRC_Context[index].state = CRC_READ_RX; /* redo a CRC_READ_RX */
					waitForComplete = 1;
					APP_DBG_MSG("BLE_STATUS_NOT_ALLOWED do another CRC_READ_RX\n");
				}
			}
		}
		break;

		case CRC_READ_TX: {
			APP_DBG_MSG("CRC_READ_TX\n");

			result = aci_gatt_read_char_value(CRC_Context[index].connHandle,
					CRC_Context[index].TXCharHdle);
			if ( result == BLE_STATUS_SUCCESS ) {
				APP_DBG_MSG("Read CRC TX sent Successfully \n");
			} else {
				APP_DBG_MSG("Read TX sent Failed \n");
				if (result == BLE_STATUS_NOT_ALLOWED) {
					CRC_Context[index].state = CRC_READ_TX; /* redo a CRC_READ_TX */
					waitForComplete = 1;
					APP_DBG_MSG("BLE_STATUS_NOT_ALLOWED do another CRC_READ_TX\n");
				}
			}
		}
		break;

		case CRC_READ_RX_CCC: {
			APP_DBG_MSG("CRC_READ_TEMPERATURE_MEASUREMENT_CCC\n");

			result = aci_gatt_read_char_desc(CRC_Context[index].connHandle,
					CRC_Context[index].RXCCCDescHdle);
			if ( result == BLE_STATUS_SUCCESS ) {
				APP_DBG_MSG("Read CRC RX CCCC sent successfully \n");
			} else  {
				APP_DBG_MSG("Read RX sending failed \n");
				if(result == BLE_STATUS_NOT_ALLOWED) {
					CRC_Context[index].state = CRC_READ_RX_CCC; /* redo a CRC_READ_RX_CCC */
					waitForComplete = 1;
					APP_DBG_MSG("BLE_STATUS_NOT_ALLOWED do another CRC_READ_RX_CCC\n");
				}
			}
		}
		break;

		case CRC_WRITE_TX: {
			APP_DBG_MSG("CRC_WRITE_TX\n");

			result = aci_gatt_write_without_resp(CRC_Context[index].connHandle,
					CRC_Context[index].TXCharHdle,
					strlen(szString),
					(uint8_t *) &szString[0]);

			if ( result == BLE_STATUS_SUCCESS ) {
				APP_DBG_MSG("Write CRC TX sent Successfully \n");
				CRC_Context[index].state = CRC_CONNECTED;
				APP_DBG_MSG("CRC_WRITE_TX -> CRC_CONNECTED\n");
//				UTIL_SEQ_SetTask( 1<<CFG_TASK_CRC_DISCOVERY_REQ_ID, CFG_SCH_PRIO_0);
				// ??
			} else {
				APP_DBG_MSG("Write TX sent Failed \n");
				if (result == BLE_STATUS_NOT_ALLOWED) {
					CRC_Context[index].state = CRC_WRITE_TX; /* redo a CRC_WRITE_TX */
					waitForComplete = 1;
					APP_DBG_MSG("BLE_STATUS_NOT_ALLOWED do another CRC_WRITE_TX\n");
				}
			}
		}
		break;

		case CRC_ENABLE_RX_NOTIFICATION: {
			uint8_t notification[2] = {0x01, 0x00};

			APP_DBG_MSG("CRC_ENABLE_RX_NOTIFICATION\n");

			result = aci_gatt_write_char_desc(CRC_Context[index].connHandle,
					CRC_Context[index].RXCCCDescHdle,
					2,
					(uint8_t *) &notification[0]);

			if( result == BLE_STATUS_SUCCESS ) {
				waitForComplete = 1;
				APP_DBG_MSG("Enable CRC RX Notification Sent Successfully \n");
			} else {
				APP_DBG_MSG("Enable CRC RX Notification Sent Failed \n");
				if (result == BLE_STATUS_NOT_ALLOWED) {
					CRC_Context[index].state = CRC_ENABLE_RX_NOTIFICATION; /* redo a CRC_ENABLE_RX_NOTIFICATION */
					waitForComplete = 1;
					APP_DBG_MSG("BLE_STATUS_NOT_ALLOWED do another CRC_ENABLE_RX_NOTIFICATION\n");
				}
			}
		}
		break;

		case CRC_DISABLE_RX_NOTIFICATION: {
			uint8_t notification[2] = {0x00, 0x00};

			APP_DBG_MSG("CRC_DISABLE_RX_NOTIFICATION\n");

			result = aci_gatt_write_char_desc(CRC_Context[index].connHandle,
					CRC_Context[index].RXCCCDescHdle,
					2,
					(uint8_t *) &notification[0]);
			if ( result == BLE_STATUS_SUCCESS ) {
				waitForComplete = 1;
				APP_DBG_MSG("Disable CRC RX Notification Sent Successfully \n");
			} else {
				APP_DBG_MSG("Disable CRC RX Notification Sent Failed \n");
				if (result == BLE_STATUS_NOT_ALLOWED) {
					CRC_Context[index].state = CRC_DISABLE_RX_NOTIFICATION; /* redo a CRC_DISABLE_RX_NOTIFICATION */
					waitForComplete = 1;
					APP_DBG_MSG("BLE_STATUS_NOT_ALLOWED do another CRC_DISABLE_RX_NOTIFICATION\n");
				}
			}
		}
		break;

		default:
			break;
		}
		index++;
	}
}


/**
 *  @brief
 *		Reads a char from the CRS Rx (serial in). Blocking until char is
 *      ready.
 *  @return
 *      Return the character read as an unsigned char cast to an int or EOF on
 *      error.
 */
int CRCAPP_getc(void) {
	uint8_t c;
	if (osMessageQueueGet(CRC_RxQueueId, &c, NULL, osWaitForever) == osOK) {
		return c;
	} else {
		Error_Handler();
		return EOF;
	}
}


/**
 *  @brief
 *		There is a character in the queue (key pressed).
 *  @return
 *		TRUE if a character has been received.
 */
int CRCAPP_RxReady(void) {
	if (osMessageQueueGetCount(CRC_RxQueueId) == 0) {
		return FALSE;
	} else {
		return TRUE;
	}
}


/**
 *  @brief
 *      Writes a char to the CRS Tx (serial out). Blocking until char can be
 *      written into the queue.
 *  @param[in]
 *      c  char to write
 *  @return
 *      Return EOF on error, 0 on success.
 */
int CRCAPP_putc(int c) {
	if (osMessageQueuePut(CRC_TxQueueId, &c, 0, osWaitForever) == osOK) {
		return 0;
	} else {
		Error_Handler();
		return EOF;
	}
}


/**
 *  @brief
 *      Tx queue ready for next char.
 *  @return
 *      FALSE if the buffer is full.
 */
int CRCAPP_TxReady(void) {
	if (osMessageQueueGetSpace(CRC_TxQueueId) > 0) {
		return TRUE;
	} else {
		return FALSE;
	}
}



/**
 *  @brief
 *      Notification from CRS_Event_Handler.
 *
 *      Callback routine from crs_stm.c
 *  @return
 *      none
 */
void CRSAPP_Notification(CRSAPP_Notification_evt_t *pNotification) {
	uint8_t buffer;
	uint32_t i;
	osStatus_t status;

	switch(pNotification->CRS_Evt_Opcode) {
	case CRS_WRITE_EVT:
		APP_DBG_MSG("CRS_WRITE_EVT: Data received: %s \n", pNotification->DataTransfered.pPayload);
		pNotification->DataTransfered.pPayload[pNotification->DataTransfered.Length] = '\0';

		for (i=0; i<pNotification->DataTransfered.Length; i++) {
			buffer = pNotification->DataTransfered.pPayload[i];
			status = osMessageQueuePut(CRC_RxQueueId, &buffer, 0, 0);
			if (status != osOK) {
				// can't put char into queue
				Error_Handler();
				break;
			}
		}
		break;

	case CRS_NOTIFY_ENABLED_EVT:
		APP_DBG_MSG("CRS_NOTIFY_ENABLED_EVT\n");
		break;

	case CRS_NOTIFY_DISABLED_EVT:
		APP_DBG_MSG("CRS_NOTIFY_DISABLED_EVT\n");
		break;

	default:
		break;
	}
}


/**
  * @brief
  * 	Function implementing the CRC thread.
  * @param
  * 	argument: Not used
  * @retval
  * 	None
  */
static void CRC_Thread(void *argument) {
	uint8_t buffer[CRS_MAX_DATA_LEN];
	uint32_t count;
	uint8_t i;
	tBleStatus status;

	// Infinite loop
	for(;;) {
		// blocked till a character is in the Tx queue
		if (osMessageQueueGet(CRC_TxQueueId, &buffer[0], 0, osWaitForever) == osOK) {
			count = osMessageQueueGetCount(CRC_TxQueueId);
			if (count > CRS_MAX_DATA_LEN-2) {
				count = CRS_MAX_DATA_LEN-2;
			}
			for (i=1; i<count+1; i++) {
				osMessageQueueGet(CRC_TxQueueId, &buffer[i], 0, osWaitForever);
			}
			buffer[count+1] = '\0';
			// send the characters
//			status = CRSAPP_Update_Char(CRC_RX_CHAR_UUID, (uint8_t *)&buffer[0]);
			CRCAPP_Write_Char(index);
			if (status != BLE_STATUS_SUCCESS) {
				// can't send char
				Error_Handler();
			}
		} else {
			// can't write to the queue
			Error_Handler();
		}
	}
}



