/**
 ******************************************************************************
  * File Name          : App/ble_conf.h
  * Description        : Configuration file for BLE Middleware.
  *
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLE_CONF_H
#define BLE_CONF_H

#include "app_conf.h"

/******************************************************************************
 *
 * BLE SERVICES CONFIGURATION
 * blesvc
 *
 ******************************************************************************/

/**
 * This setting shall be set to '1' if the device needs to support the Peripheral Role
 * In the MS configuration, both BLE_CFG_PERIPHERAL and BLE_CFG_CENTRAL shall be set to '1'
 */
#if (GATT_CLIENT == 1)
#define BLE_CFG_PERIPHERAL                                                     0
#else
#define BLE_CFG_PERIPHERAL                                                     1
#endif

/**
 * This setting shall be set to '1' if the device needs to support the Central Role
 * In the MS configuration, both BLE_CFG_PERIPHERAL and BLE_CFG_CENTRAL shall be set to '1'
 */
#if (GATT_CLIENT == 1)
#define BLE_CFG_CENTRAL                                                        1
#else
#define BLE_CFG_CENTRAL                                                        0
#endif


/**
 * There is one handler per service enabled
 * Note: There is no handler for the Device Information Service
 *
 * This shall take into account all registered handlers
 * (from either the provided services or the custom services)
 */
#define BLE_CFG_SVC_MAX_NBR_CB                                                 7

#define BLE_CFG_CLT_MAX_NBR_CB                                                 0

/******************************************************************************
 * Device Information Service (DIS)
 ******************************************************************************/
/**< Options: Supported(1) or Not Supported(0) */
#define BLE_CFG_DIS_MANUFACTURER_NAME_STRING                                   1
#define BLE_CFG_DIS_MODEL_NUMBER_STRING                                        1
#define BLE_CFG_DIS_SERIAL_NUMBER_STRING                                       1
#define BLE_CFG_DIS_HARDWARE_REVISION_STRING                                   1
#define BLE_CFG_DIS_FIRMWARE_REVISION_STRING                                   1
#define BLE_CFG_DIS_SOFTWARE_REVISION_STRING                                   1
#define BLE_CFG_DIS_SYSTEM_ID                                                  1
#define BLE_CFG_DIS_IEEE_CERTIFICATION                                         0
#define BLE_CFG_DIS_PNP_ID                                                     0

/**
 * device information service characteristic lengths
 */
#define BLE_CFG_DIS_SYSTEM_ID_LEN_MAX                                        (8)
#define BLE_CFG_DIS_MODEL_NUMBER_STRING_LEN_MAX                              (32)
#define BLE_CFG_DIS_SERIAL_NUMBER_STRING_LEN_MAX                             (32)
#define BLE_CFG_DIS_FIRMWARE_REVISION_STRING_LEN_MAX                         (32)
#define BLE_CFG_DIS_HARDWARE_REVISION_STRING_LEN_MAX                         (32)
#define BLE_CFG_DIS_SOFTWARE_REVISION_STRING_LEN_MAX                         (32)
#define BLE_CFG_DIS_MANUFACTURER_NAME_STRING_LEN_MAX                         (32)
#define BLE_CFG_DIS_IEEE_CERTIFICATION_LEN_MAX                               (32)
#define BLE_CFG_DIS_PNP_ID_LEN_MAX                                           (7)

/******************************************************************************
 * Heart Rate Service (HRS)
 ******************************************************************************/
#define BLE_CFG_HRS_BODY_SENSOR_LOCATION_CHAR               1/**< BODY SENSOR LOCATION CHARACTERISTIC */
#define BLE_CFG_HRS_ENERGY_EXPENDED_INFO_FLAG               1/**< ENERGY EXTENDED INFO FLAG */
#define BLE_CFG_HRS_ENERGY_RR_INTERVAL_FLAG                 1/**< Max number of RR interval values - Shall not be greater than 9 */

/******************************************************************************
 * GAP Service - Apprearance
 ******************************************************************************/

#define BLE_CFG_UNKNOWN_APPEARANCE                  (0)
#define BLE_CFG_HR_SENSOR_APPEARANCE                (832)
#define BLE_CFG_GAP_APPEARANCE                      (BLE_CFG_HR_SENSOR_APPEARANCE)

/******************************************************************************
 * Over The Air Feature (OTA) - STM Proprietary
 ******************************************************************************/
#define BLE_CFG_OTA_REBOOT_CHAR         0/**< REBOOT OTA MODE CHARACTERISTIC */

/******************************************************************************
 * Cable Replacement Service STM (CRS STM)
 ******************************************************************************/
/**< Options: Supported(1) or Not Supported(0) */
#define CRS_STM_UUID128    0x00, 0x00, 0xfe, 0x60, 0xcc, 0x7a, 0x48, 0x2a, 0x98, 0x4a, 0x7f, 0x2e, 0xd5, 0xb3, 0xe5, 0x8f
#define CRS_STM_TX_UUID128 0x00, 0x00, 0xfe, 0x61, 0x8e, 0x22, 0x45, 0x41, 0x9d, 0x4c, 0x21, 0xed, 0xae, 0x82, 0xed, 0x19
#define CRS_STM_RX_UUID128 0x00, 0x00, 0xfe, 0x62, 0x8e, 0x22, 0x45, 0x41, 0x9d, 0x4c, 0x21, 0xed, 0xae, 0x82, 0xed, 0x19

#endif /*BLE_CONF_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
