/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
* Bluetooth LE Beacon Device
*
* This file provides definitions and function prototypes for My Beacon device
*
*/
#ifndef MY_BEACON_H
#define MY_BEACON_H

// Please note that all UUIDs need to be reversed when publishing in the database

#ifdef WIN32
// {0E5CB5E2-80F0-4C20-A729-D09AB5ACF887}
static const GUID GUID_MY_BEACON = {0x0e5cb5e2, 0x80f0, 0x4c20, { 0xa7, 0x29, 0xd0, 0x9a, 0xb5, 0xac, 0xf8, 0x87 } };
#endif
#define UUID_MY_BEACON              0x87, 0xf8, 0xac, 0xb5, 0x9a, 0xd0, 0x29, 0xa7, 0x20, 0x4c, 0xf0, 0x80, 0xe2, 0xb5, 0x5c, 0x0e

#define MYBEACON_TYPE				0x01

#define     HCIULP_ADV_NONCONNECTABLE_EVENT                                 0x03
#define     HCIULP_ADV_CHANNEL_37                                           0x01
#define     HCIULP_ADV_CHANNEL_38                                           0x02
#define     HCIULP_ADV_CHANNEL_39                                           0x04

#define     HCIULP_ADV_CHANNEL_MAP_MASK                                     (HCIULP_ADV_CHANNEL_37 | HCIULP_ADV_CHANNEL_38 | HCIULP_ADV_CHANNEL_39)

#define     HCIULP_PUBLIC_ADDRESS                                           0x00

#define     HCIULP_ADV_FILTER_POLICY_ACCEPT_LIST_NOT_USED                    0x00    // filter accept list not used

#endif
