/*******************************************************************************
* File Name: light_dimmable.c
*
* Description: This file contains mesh lightness server implementation.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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
*******************************************************************************/

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "cy_retarget_io.h"
#include "cybsp_bt_config.h"
#include "wiced_bt_types.h"
#include "wiced_bt_stack.h"

#include "wiced_bt_types.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_bt_mesh_core.h"
#include "wiced_bt_mesh_models.h"
#include "mesh_cfg.h"
#include "mesh_app.h"
#include "board.h"

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static void mesh_lightness_server_message_handler(uint8_t element_idx,
                                                 uint16_t event, void *p_data);
static void mesh_light_set_brightness(uint8_t element_idx,
                                    wiced_bt_mesh_light_lightness_status_t *p_status);

/*******************************************************************************
 * Variables Definitions
 ******************************************************************************/

/* Structure to keep the light state of lightness server. */
typedef struct
{
    uint8_t  brightness_level;
    uint32_t  remaining_time;
} mesh_lightness_state_t;

/* Application state ON/OFF */
mesh_lightness_state_t app_state;

/*******************************************************************************
 * Function Name: mesh_lightness_server_model_init
 *******************************************************************************
 * Summary:
 *  mesh lightness server model initialization.
 *
 * Parameters:
 *  wiced_bool_t is_provisioned : provisioned status
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void mesh_lightness_server_model_init(wiced_bool_t is_provisioned)
{

    wiced_bt_mesh_model_light_lightness_server_init(MESH_LIGHTNESS_SERVER_ELEMENT_INDEX,
                                    mesh_lightness_server_message_handler, is_provisioned);
}


/*******************************************************************************
 * Function Name: mesh_lightness_server_message_handler
 *******************************************************************************
 * Summary:
 *  lightness server mesh handler.
 *
 * Parameters:
 *  uint8_t element_idx : element id
 *  uint16_t event : event type
 *  uint16_t *p_data : p_data
 * Return:
 *  void
 *
 ******************************************************************************/
void mesh_lightness_server_message_handler(uint8_t element_idx, uint16_t event, void *p_data)
{

    switch (event)
    {
    case WICED_BT_MESH_LIGHT_LIGHTNESS_STATUS:
        mesh_light_set_brightness(element_idx, (wiced_bt_mesh_light_lightness_status_t *)p_data);
        break;

    default:
        printf("Mesh lightness server unknown event:%d\r\n", event);
        break;
    }

}

/*******************************************************************************
 * Function Name: mesh_light_set_brightness
 *******************************************************************************
 * Summary:
 * This function set the brightness level on change is received over mesh.
 *
 * Parameters:
 *  uint8_t element_idx : element id
 *  wiced_bt_mesh_light_lightness_status_t p_status : lightness status
 * Return:
 *  void
 *
 ******************************************************************************/
void mesh_light_set_brightness(uint8_t element_idx, wiced_bt_mesh_light_lightness_status_t *p_status)
{

    printf("Mesh lightness server set level: actual:%d linear:%d remaining:%ld\r\n",
                                                        p_status->lightness_actual_present,
                                                        p_status->lightness_linear_present,
                                                        p_status->remaining_time);

    app_state.brightness_level = (uint8_t)((uint32_t)p_status->lightness_actual_present * 100 / 65535);
    app_state.remaining_time = p_status->remaining_time;

    board_led_set_brightness(USER_LED2, app_state.brightness_level);

}


/* [] END OF FILE */