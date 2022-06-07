/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
*
* Bluetooth LE beacon sample
*
* During initialization the app configures stack to send advertisement
* packets.  Non-connectable undirected advertisements are used.
*
* Features demonstrated
*  - configuring vendor specific advertisements
*  - configuring GPIO P0 as an active HIGH indication to pause advertising
*    for as long as the GPIO is high. When the beacon app is co-located
*    with a WiFi device, the WiFi device can assert P0 for the duration
*    of the WiFi activity so the beacon can be turned off to prevent
*    interference.
*
* To demonstrate the app, work through the following steps.
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. Monitor advertisement packets on over the air sniffer
* 4. Push and hold the user button on the eval board and check that advertisements stop
* 5. Release the user button and check that advertisements resume
*
*/
#include "spar_utils.h"
#include "bleprofile.h"
#include "bleapp.h"
#include "gpiodriver.h"
#include "string.h"
#include "stdio.h"
#include "platform.h"
#include "mybeacon.h"
#include "additional_advertisement_control.h"
#include "bleapputils.h"
#include "sparcommon.h"

/******************************************************
 *               Function Prototypes
 ******************************************************/

static void mybeacon_create(void);
extern void blecm_setTxPowerInADV(int);
/******************************************************
 *               Variables Definitions
 ******************************************************/

const UINT8 mybeacon_uuid[16] = {0x87, 0xf8, 0xac, 0xb5, 0x9a, 0xd0, 0x29, 0xa7, 0x20, 0x4c, 0xf0, 0x80, 0xe2, 0xb5, 0x5c, 0x0e};
UINT16 mymeacon_sequence_number = 0;

// Following structure defines UART configuration
const BLE_PROFILE_PUART_CFG mybeacon_puart_cfg =
{
    /*.baudrate   =*/ 115200,
    /*.txpin      =*/ PUARTENABLE | GPIO_PIN_UART_TX,
    /*.rxpin      =*/ PUARTENABLE | GPIO_PIN_UART_RX,
};

void advertisement_packet_configuration(UINT8 type);

/******************************************************
 *               Function Definitions
 ******************************************************/
// Application initialization
APPLICATION_INIT()
{
    bleapp_set_cfg(NULL,
                   0,
                   NULL,
                   (void *)&mybeacon_puart_cfg,
                   NULL,
                   mybeacon_create);
    BLE_APP_ENABLE_TRACING_ON_PUART();
}

// Create ANCS client device
void mybeacon_create(void)
{

    ble_trace0("\rmybeacon_create()");
    ble_trace0(bleprofile_p_cfg->ver);

    // dump the database to debug uart.
    legattdb_dumpDb();

    ble_tracen((char *)mybeacon_uuid, 16);

    bleprofile_Init(bleprofile_p_cfg);

    // mybeacon_sequence_number global variable is equal to zero for this initialization call
    //
    advertisement_packet_configuration(0);

    // Configure P0 as active high input to pause advertisements.
    bleprofile_configureGpioForSkippingRf(GPIO_PIN_P0, GPIO_PIN_INPUT_HIGH);

    // register with LE stack to be called 2.5msec before the advertisement event
    bleprofile_notifyAdvPacketTransmissions(&advertisement_packet_configuration, 2500);

    // Set TX power for advertisement packets at 4dBm
    blecm_setTxPowerInADV(4);

    bleapputils_changeLPOSource(LPO_MIA_LPO, FALSE, 500);

    // Start advertisements
    blecm_startAdv(
        HCIULP_ADV_NONCONNECTABLE_EVENT,                // non-connectable undirected advertisement
        160,                                            // adv interval 100 msec
        HCIULP_ADV_CHANNEL_MAP_MASK,                    // all channels
        HCIULP_PUBLIC_ADDRESS,                          // int advAdrType,
        HCIULP_ADV_FILTER_POLICY_ACCEPT_LIST_NOT_USED,  // int advFilterPolicy,
        HCIULP_PUBLIC_ADDRESS,                          // int initiatorAdrType,
        NULL);                                          // UINT8* initiatorAdr
}

// this function is called 2.5 msec before the advertisement event.  In this sample
// just bump the sequence number and modify advertisement data.
void advertisement_packet_configuration(UINT8 type)
{
    if (type == 0)
    {
        const uint8_t ADV_FIELD_SIZE = 2;
        BLE_ADV_FIELD adv[ADV_FIELD_SIZE];

        // Format advertisement data.  Data consists of 2 fields.  Standard Advertisement flags
        // and Infineon Vendor specific data.  The vendor specific data consists of 16 byte
        // UUID, 2 byte type and 2 byte sequence number.

        // flags
        adv[0].len     = 1 + 1;
        adv[0].val     = ADV_FLAGS;
        adv[0].data[0] = LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED;

        adv[1].len     = 22 + 1;
        adv[1].val     = ADV_MANUFACTURER_DATA; // (AD_TYPE == 0xff)
        adv[1].data[0] = 0x09;  // Infineon  (Company Identifier 2 bytes)
        adv[1].data[1] = 0x00;

        BT_MEMCPY(&adv[1].data[2], mybeacon_uuid, 16);

        adv[1].data[18] = MYBEACON_TYPE & 0xff;
        adv[1].data[19] = (MYBEACON_TYPE >> 8) & 0xff;
        adv[1].data[20] = ++mymeacon_sequence_number & 0xff;
        adv[1].data[21] = (mymeacon_sequence_number >> 8) & 0xff;

        bleprofile_GenerateADVData(adv, ADV_FIELD_SIZE);
    }
}
