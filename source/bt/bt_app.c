/*******************************************************************************
* File Name: bt_app.c
*
* Description: This file contains the task that handles bluetooth events and
* notifications.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cy_retarget_io.h"
#include "cycfg_gap.h"
#include "cybsp_bt_config.h"
#include "cycfg_gatt_db.h"
#include "cycfg_bt_settings.h"
#include "wiced_bt_types.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_stack.h"
#include "bt_app.h"
#include "bt_utils.h"
#include "board.h"
#include "sensor.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void bt_app_init(void);
wiced_bt_gatt_status_t bt_app_gatt_event_cb(wiced_bt_gatt_evt_t event,
                                            wiced_bt_gatt_event_data_t *p_data);
wiced_bt_gatt_status_t bt_app_gatt_conn_status_cb(wiced_bt_gatt_connection_status_t 
                                                                    *p_conn_status);
wiced_bt_gatt_status_t bt_app_gatt_req_cb(wiced_bt_gatt_attribute_request_t *p_attr_req);
wiced_bt_gatt_status_t bt_app_gatt_req_write_value(uint16_t attr_handle,
                                                    uint8_t *p_val, uint16_t len);
wiced_bt_gatt_status_t bt_app_gatt_req_write_handler(uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_write_req_t *p_write_req,
                                                uint16_t len_req);
wiced_bt_gatt_status_t bt_app_gatt_req_read_handler(uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_read_t *p_read_req,
                                                uint16_t len_req);
wiced_bt_gatt_status_t bt_app_gatt_req_read_by_type_handler (uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_read_by_type_t *p_read_req,
                                                uint16_t len_requested);
static gatt_db_lookup_table_t *bt_app_find_by_handle(uint16_t handle);
static void* bt_app_alloc_buffer(int len);
static void  bt_app_free_buffer(uint8_t *p_event_data);
static void  bt_print_bd_address(wiced_bt_device_address_t bdadr);
void bt_app_restore_config(void);

/*******************************************************************************
 * Structures
 ******************************************************************************/

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Variable to denote notification received for bt task. */
uint8_t notify_enabled;
uint8_t notify_motion_sensor;
uint8_t notify_temperature_sensor;
uint8_t notify_index;
const uint16_t notify_interval_value[4u] = {NOTIFY_TIME_100MS, NOTIFY_TIME_500MS,
                                    NOTIFY_TIME_1000MS, NOTIFY_TIME_5000MS};

/* Holds the connection ID */
volatile uint16_t bt_connection_id = 0;

/**
 * Typdef for function used to free allocated buffer to stack
 */
typedef void (*pfn_free_buffer_t)(uint8_t *);

/*******************************************************************************
 * Function Name: bt_app_init
 *******************************************************************************
 * Summary:
 *   This function handles application level initialization tasks and is 
 *   called from the BT management callback once the LE stack enabled event 
 *   (BTM_ENABLED_EVT) is triggered This function is executed in the
 *    BTM_ENABLED_EVT management callback.
 *
 * Parameters:
 *   None
 *
 * Return:
 *  None
 *
 ******************************************************************************/
void bt_app_init(void)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    /* Register with BT stack to receive GATT callback */
    status = wiced_bt_gatt_register(bt_app_gatt_event_cb);
    printf("GATT event handler registration status: %d \r\n",status);

    /* Initialize GATT Database */
    status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    printf("GATT database initialization status: %d \r\n",status);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(FALSE, FALSE);

    /* Set Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, 
                                                            cy_bt_adv_packet_data);

    /* Start Undirected LE Advertisements on device startup.
     * The corresponding parameters are contained in 'app_bt_cfg.c' */
    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

    /* Failed to start advertisement. Stop program execution */
    if (WICED_BT_SUCCESS != result)
    {
        printf("Failed to start advertisement! \r\n");
        CY_ASSERT(0);
    }

    /* Set user led status */
    board_led_set_blink(USER_LED1, BLINK_SLOW);
    board_led_set_state(USER_LED2, LED_OFF);
}

/*******************************************************************************
* Function Name: bt_app_management_cb
********************************************************************************
* Summary:
*   This is a Bluetooth stack event handler function to receive management events
*   from the LE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : LE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to LE management event 
*                                                 structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*******************************************************************************/
wiced_result_t bt_app_management_cb(wiced_bt_management_evt_t event,
                                   wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_device_address_t local_bda = {0x00, 0xA0, 0x50, 0x02, 0x04, 0x08};

    printf("Bluetooth app management callback: 0x%x\r\n", event);

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Initilize the flash memory */
            if(CY_RSLT_SUCCESS == flash_memory_init())
            {
                printf("Flash memory initialized! \r\n");
            }
            /* Restore the notification data*/
            bt_app_restore_config();

            /* Bluetooth Controller and Host Stack Enabled */
            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                wiced_bt_set_local_bdaddr(local_bda, BLE_ADDR_PUBLIC);
                wiced_bt_dev_read_local_addr(local_bda);
                printf("Bluetooth local device address: ");
                bt_print_bd_address(local_bda);

                /* Perform application-specific initialization */
                bt_app_init();
            }
            else
            {
                printf("Bluetooth enable failed, status = %d \r\n",
                                                p_event_data->enabled.status);
            }
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

            /* Advertisement State Changed */
            printf("Bluetooth advertisement state change: 0x%x\r\n",
                                     p_event_data->ble_advert_state_changed);
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            printf("Bluetooth connection parameter update status:%d\n \
                    parameter interval: %d ms\n \
                    parameter latency: %d ms\n \
                    parameter timeout: %d ms\r\n",
                    p_event_data->ble_connection_param_update.status,
                    p_event_data->ble_connection_param_update.conn_interval,
                    p_event_data->ble_connection_param_update.conn_latency,
                    p_event_data->ble_connection_param_update.supervision_timeout);
            result = WICED_SUCCESS;
            break;

        case BTM_BLE_PHY_UPDATE_EVT:
            /* Print the updated Bluetooth physical link*/
            printf("Bluetooth phy update selected TX - %dM\r\nBluetooth phy update selected RX - %dM\r\n",
                    p_event_data->ble_phy_update_event.tx_phy,
                    p_event_data->ble_phy_update_event.rx_phy);
            break;

        case BTM_PIN_REQUEST_EVT:
        case BTM_PASSKEY_REQUEST_EVT:
             result = WICED_BT_ERROR;
             break;

        default:
            printf("Bluetooth unhandled event: 0x%x \r\n", event);
            break;
    }
    return result;
}

/*******************************************************************************
* Function Name: bt_app_gatt_event_cb
********************************************************************************
* Summary:
*   This function handles GATT events from the BT stack.
*
* Parameters:
*   wiced_bt_gatt_evt_t event                : LE GATT event code of one byte length
*   wiced_bt_gatt_event_data_t *p_event_data : Pointer to LE GATT event structures
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_event_cb(wiced_bt_gatt_evt_t event,
                                        wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_event_data->attribute_request;
    /* Call the appropriate callback function based on the GATT event type, and 
     * pass the relevant event parameters to the callback function */
    switch (event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            status = bt_app_gatt_conn_status_cb(&p_event_data->connection_status );
            if(WICED_BT_GATT_SUCCESS != status)
            {
               printf("GATT connection status failed: 0x%x\r\n", status);
            }
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            status = bt_app_gatt_req_cb(p_attr_req);
            break;

        case GATT_GET_RESPONSE_BUFFER_EVT:
            p_event_data->buffer_request.buffer.p_app_rsp_buffer =
            bt_app_alloc_buffer(p_event_data->buffer_request.len_requested);
            p_event_data->buffer_request.buffer.p_app_ctxt = (void *)bt_app_free_buffer;
            status = WICED_BT_GATT_SUCCESS;
            break;

            /* GATT buffer transmitted event,
             * check \ref wiced_bt_gatt_buffer_transmitted_t*/
        case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            pfn_free_buffer_t pfn_free =
                (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function
             * to free it. */
            if (pfn_free)
                pfn_free(p_event_data->buffer_xmitted.p_app_data);

            status = WICED_BT_GATT_SUCCESS;
        }
            break;

        default:
            status = WICED_BT_GATT_SUCCESS;
            break;
    }

    return status;
}

/*******************************************************************************
* Function Name: bt_app_gatt_req_cb
********************************************************************************
* Summary:
*   This function handles GATT server events from the BT stack.
*
* Parameters:
*  wiced_bt_gatt_attribute_request_t p_attr_req : Pointer to GATT connection status
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_cb(wiced_bt_gatt_attribute_request_t *p_attr_req)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    switch ( p_attr_req->opcode )
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
             /* Attribute read request */
            status = bt_app_gatt_req_read_handler(p_attr_req->conn_id,
                                                  p_attr_req->opcode,
                                                  &p_attr_req->data.read_req,
                                                  p_attr_req->len_requested);
             break;

        case GATT_REQ_READ_BY_TYPE:
            status = bt_app_gatt_req_read_by_type_handler(p_attr_req->conn_id,
                                                            p_attr_req->opcode,
                                                          &p_attr_req->data.read_by_type,
                                                          p_attr_req->len_requested);
            break;

        case GATT_REQ_READ_MULTI:
            break;

        case GATT_REQ_MTU:
            status = wiced_bt_gatt_server_send_mtu_rsp(p_attr_req->conn_id,
                                                       p_attr_req->data.remote_mtu,
                                                       CY_BT_MTU_SIZE);
             break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
             /* Attribute write request */
             status = bt_app_gatt_req_write_handler(p_attr_req->conn_id,
                                                    p_attr_req->opcode,
                                                    &p_attr_req->data.write_req,
                                                    p_attr_req->len_requested);

             if ((GATT_REQ_WRITE == p_attr_req->opcode) &&
                 (WICED_BT_GATT_SUCCESS == status ))
             {
                 wiced_bt_gatt_write_req_t *p_write_request = &p_attr_req->data.write_req;
                 wiced_bt_gatt_server_send_write_rsp(p_attr_req->conn_id,
                                                     p_attr_req->opcode,
                                                     p_write_request->handle);
             }
             break;
        case GATT_HANDLE_VALUE_CONF:
        case GATT_HANDLE_VALUE_NOTIF:
             break;

        default:
            printf("bt_app_gatt: unhandled GATT request: %d\r\n", p_attr_req->opcode);
            break;
    }

    return status;
}

/*******************************************************************************
 * Function Name : bt_app_gatt_req_read_by_type_handler
 * *****************************************************************************
 * Summary :
 *    Process read-by-type request from peer device
 *
 * Parameters:
 *  uint16_t                      conn_id       : Connection ID
 *  wiced_bt_gatt_opcode_t        opcode        : LE GATT request type opcode
 *  wiced_bt_gatt_read_by_type_t  p_read_req    : Pointer to read request 
 *                                                containing the handle to read
 *  uint16_t                      len_req        : Length of data requested
 *
 * Return:
 *  wiced_bt_gatt_status_t  : LE GATT status
 ******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_read_by_type_handler(uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_read_by_type_t *p_read_req,
                                                uint16_t len_req)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t last_handle = 0;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = bt_app_alloc_buffer(len_req);
    uint8_t pair_len = 0;
    int used_len = 0;

    if (NULL == p_rsp)
    {
        printf("bt_app_gatt:no memory found, len_req: %d!!\r\n",len_req);
        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            attr_handle,
                                            WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, 
     * between the start and end handles */
    while (WICED_TRUE)
    {
        last_handle = attr_handle;
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle,
                                                        p_read_req->e_handle,
                                                        &p_read_req->uuid);
        if (0 == attr_handle)
            break;

        if ( NULL == (puAttribute = bt_app_find_by_handle(attr_handle)))
        {
            printf("bt_app_gatt:found type but no attribute for %d \r\n",last_handle);
            wiced_bt_gatt_server_send_error_rsp(conn_id,
                                                opcode,
                                                p_read_req->s_handle,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            bt_app_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used_len,
                                                                len_req - used_len,
                                                                &pair_len,
                                                                attr_handle,
                                                                puAttribute->cur_len,
                                                                puAttribute->p_data);
        if (0 == filled)
        {
            break;
        }
        used_len += filled;

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (0 == used_len)
    {
        printf("bt_app_gatt:attr not found start_handle: 0x%04x  end_handle: 0x%04x \
                                                        type: 0x%04x\r\n",
                                                        p_read_req->s_handle,
                                                        p_read_req->e_handle,
                                                        p_read_req->uuid.uu.uuid16);

        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            p_read_req->s_handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        bt_app_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    return wiced_bt_gatt_server_send_read_by_type_rsp(conn_id,
                                                      opcode,
                                                      pair_len,
                                                      used_len,
                                                      p_rsp,
                                                      (void *)bt_app_free_buffer);
}


/*******************************************************************************
* Function Name: bt_app_gatt_req_write_value
********************************************************************************
* Summary:
* This function handles writing to the attribute handle in the GATT database
* using the data passed from the BT stack. The value to write is stored in a
* buffer whose starting address is passed as one of the function parameters
*
* Parameters:
*  uint16_t attr_handle      : GATT attribute handle
*  uint8_t p_val            : Pointer to Bluetooth GATT write request value
*  uint16_t len              : length of GATT write request
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_write_value(uint16_t attr_handle,
                                                    uint8_t *p_val, uint16_t len)
{
    wiced_bt_gatt_status_t gatt_status  = WICED_BT_GATT_INVALID_HANDLE;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;

    /* Check for a matching handle entry */
    for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {

        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in external lookup table */
            isHandleInTable = WICED_TRUE;

            /* Check if the buffer has space to store the data */
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);

            if (validLen)
            {

                /* Value fits within the supplied buffer; copy over the value */
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                gatt_status = WICED_BT_GATT_SUCCESS;

                /* Add code for any action required when this attribute is written.
                 * In this case, we Initialize the characteristic value */

                switch ( attr_handle )
                {
                case HDLD_SENSOR_HUB_MOTION_SENSOR_CLIENT_CHAR_CONFIG:
                    if ( len != 2 )
                    {
                        return WICED_BT_GATT_INVALID_ATTR_LEN;
                    }

                    if(GATT_CLIENT_CONFIG_NOTIFICATION ==
                               app_sensor_hub_motion_sensor_client_char_config[0])
                    {
                        notify_motion_sensor = NOTIFIY_ON;
                    }
                    else
                    {
                        notify_motion_sensor = NOTIFIY_OFF;
                    }
                    break;

                case HDLD_SENSOR_HUB_TEMPERATURE_SENSOR_CLIENT_CHAR_CONFIG:
                    if ( len != 2 )
                    {
                        return WICED_BT_GATT_INVALID_ATTR_LEN;
                    }
                    if(GATT_CLIENT_CONFIG_NOTIFICATION ==
                               app_sensor_hub_temperature_sensor_client_char_config[0])
                    {
                        notify_temperature_sensor = NOTIFIY_ON;
                    }
                    else
                    {
                        notify_temperature_sensor = NOTIFIY_OFF;
                    }
                    break;

                }

                if((app_sensor_hub_motion_sensor_client_char_config[0] ||
                        app_sensor_hub_temperature_sensor_client_char_config[0]) ==
                                                    GATT_CLIENT_CONFIG_NOTIFICATION)
                {
                    notify_enabled = NOTIFIY_ON;
                    board_led_set_state(USER_LED2, LED_ON);
                }
                else
                {
                    notify_enabled = NOTIFIY_OFF;
                    board_led_set_state(USER_LED2, LED_OFF);
                }

            }
            else
            {
                /* Value to write does not meet size constraints */
                gatt_status = WICED_BT_GATT_INVALID_HANDLE;
                printf("GATT write request to invalid handle: 0x%x\r\n", attr_handle);
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        /* TODO: Add code to read value for handles not contained within
         * generated lookup table. This is a custom logic that depends on the
         * application, and is not used in the current application. If the value
         * for the current handle is successfully written in the below code
         * snippet, then set the result using: res = WICED_BT_GATT_SUCCESS; */
        switch(attr_handle)
        {
            default:
                /* The write operation was not performed for the
                    * indicated handle */
                gatt_status = WICED_BT_GATT_WRITE_NOT_PERMIT;
                printf("GATT write request to invalid handle: 0x%x\n", attr_handle);
                break;
        }
    }
    return gatt_status;
}

/*******************************************************************************
* Function Name: bt_app_gatt_req_write_handler
********************************************************************************
* Summary:
*   This function handles Write Requests received from the client device
*
* Parameters:
*  uint16_t conn_id       : Connection ID
*  wiced_bt_gatt_opcode_t opcode        : LE GATT request type opcode
*  wiced_bt_gatt_write_req_t p_write_req   : Pointer to LE GATT write request
*  uint16_t len_req       : length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_write_handler(uint16_t conn_id,
                                                wiced_bt_gatt_opcode_t opcode,
                                                wiced_bt_gatt_write_req_t *p_write_req,
                                                uint16_t len_req)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    printf("bt_app_gatt_write_handler: conn_id:%d handle:0x%x offset:%d len:%d\r\n",
                                                            conn_id, 
                                                            p_write_req->handle, 
                                                            p_write_req->offset, 
                                                            p_write_req->val_len );

    /* Attempt to perform the Write Request */
    status = bt_app_gatt_req_write_value(p_write_req->handle,
                                         p_write_req->p_val,
                                         p_write_req->val_len);

    if(WICED_BT_GATT_SUCCESS != status)
    {
        printf("bt_app_gatt:GATT set attr status : 0x%x\n", status);
    }

    return (status);
}

/*******************************************************************************
* Function Name: bt_app_gatt_req_read_handler
********************************************************************************
* Summary:
*   This function handles Read Requests received from the client device
*
* Parameters:
* conn_id       : Connection ID
* opcode        : LE GATT request type opcode
* p_read_req    : Pointer to read request containing the handle to read
* len_req       : length of data requested
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_req_read_handler( uint16_t conn_id,
                                                    wiced_bt_gatt_opcode_t opcode,
                                                    wiced_bt_gatt_read_t *p_read_req,
                                                    uint16_t len_req)
{
    gatt_db_lookup_table_t  *puAttribute;
    int          attr_len_to_copy;
    uint8_t     *from;
    int          to_send;

    puAttribute = bt_app_find_by_handle(p_read_req->handle);
    if (NULL == puAttribute)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    attr_len_to_copy = puAttribute->cur_len;

    printf("bt_app_gatt_read_handler: conn_id:%d handle:0x%x offset:%d len:%d\r\n",
                                                    conn_id, p_read_req->handle,
                                                    p_read_req->offset,
                                                    attr_len_to_copy);

    if (p_read_req->offset >= puAttribute->cur_len)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }

    to_send = MIN(len_req, attr_len_to_copy - p_read_req->offset);
    from = ((uint8_t *)puAttribute->p_data) + p_read_req->offset;

    switch ( p_read_req->handle )
    {
    case HDLC_SENSOR_HUB_MOTION_SENSOR_VALUE:
        /* Read motion sensor data from motion sensor */
        get_motion_data();
        memcpy(&app_sensor_hub_motion_sensor[0], &app_data.data.accel, 6u);
        memcpy(&app_sensor_hub_motion_sensor[6], &app_data.data.gyro, 6u);
        app_sensor_hub_motion_sensor[12] = app_data.orientation;
        break;

    case HDLC_SENSOR_HUB_TEMPERATURE_SENSOR_VALUE:
        /* Read temperture sensor data from thermistor */
        get_temperature();
        memcpy(&app_sensor_hub_temperature_sensor[0], &app_data.temperature, 4u);
        break;
    }

    /* No need for context, as buff not allocated */
    return wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send,
                                                                    from, NULL);

}

/*******************************************************************************
* Function Name: bt_app_gatt_conn_status_cb
********************************************************************************
* Summary:
*   This callback function handles connection status changes.
*
* Parameters:
*   wiced_bt_gatt_connection_status_t *p_conn_status  : Pointer to data that 
*                                                       has connection details
*
* Return:
*  wiced_bt_gatt_status_t: Status codes in wiced_bt_gatt_status_e
*
*******************************************************************************/
wiced_bt_gatt_status_t bt_app_gatt_conn_status_cb(wiced_bt_gatt_connection_status_t 
                                                                    *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_result_t result = WICED_BT_ERROR;

    if ( NULL != p_conn_status )
    {
        if (p_conn_status->connected)
        {
            /* Device has connected */
            printf("Bluetooth connected with device address:" );
            bt_print_bd_address(p_conn_status->bd_addr);
            printf("Bluetooth device connection id: 0x%x\r\n", p_conn_status->conn_id );
            /* Store the connection ID */
            bt_connection_id = p_conn_status->conn_id;
            board_led_set_state(USER_LED1, LED_OFF);
        }
        else
        {
            /* Device has disconnected */
            printf("Bluetooth disconnected with device address:" );
            bt_print_bd_address(p_conn_status->bd_addr);
            printf("Bluetooth device connection id: 0x%x\r\n", p_conn_status->conn_id );
            /* Set the connection id to zero to indicate disconnected state */
            bt_connection_id = 0;
            /* Restart the advertisements */
            result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
            /* Failed to start advertisement. Stop program execution */
            if (CY_RSLT_SUCCESS != result)
            {
                CY_ASSERT(0);
            }
            board_led_set_blink(USER_LED1, BLINK_SLOW);
        }
        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}

/*******************************************************************************
 * Function Name: bt_app_free_buffer
 *******************************************************************************
 * Summary:
 *  This function frees up the memory buffer
 *
 * Parameters:
 *  uint8_t *p_data: Pointer to the buffer to be free
 * 
 * Return:
 *  None
 *
 ******************************************************************************/
void bt_app_free_buffer(uint8_t *p_buf)
{
    vPortFree(p_buf);
}

/*******************************************************************************
 * Function Name: bt_app_alloc_buffer
 *******************************************************************************
 * Summary:
 *  This function allocates a memory buffer.
 *
 *
 * Parameters:
 *  int len: Length to allocate
 * 
 * Return:
 *  None
 *
 ******************************************************************************/
void* bt_app_alloc_buffer(int len)
{
    return pvPortMalloc(len);
}

/*******************************************************************************
 * Function Name : bt_app_find_by_handle
 * *****************************************************************************
 * Summary :
 *    Find attribute description by handle
 *
 * Parameters:
 *  uint16_t handle    handle to look up
 *
 * Return:
 *  gatt_db_lookup_table_t   pointer containing handle data
 * 
 ******************************************************************************/
gatt_db_lookup_table_t  *bt_app_find_by_handle(uint16_t handle)
{
    for (uint8_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (handle == app_gatt_db_ext_attr_tbl[i].handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}

/*******************************************************************************
* Function Name: bt_app_send_notification
********************************************************************************
* Summary: Sends GATT notification.
*
 * Parameters:
 *  uint8_t index   : index of the sensor
*
* Return:
*  None
*
*******************************************************************************/
void bt_app_send_notification(uint8_t index)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    switch(index)
    {

    case MOTION_SENSOR:

       if((GATT_CLIENT_CONFIG_NOTIFICATION == \
                            app_sensor_hub_motion_sensor_client_char_config[0])
                            && (0 != bt_connection_id))
        {

        status = wiced_bt_gatt_server_send_notification(
                                bt_connection_id,
                                HDLC_SENSOR_HUB_MOTION_SENSOR_VALUE,
                                app_gatt_db_ext_attr_tbl[2].cur_len,
                                app_gatt_db_ext_attr_tbl[2].p_data,NULL);

        if(WICED_BT_GATT_SUCCESS != status)
        {
            printf("Sending motion sensor notification failed\r\n");
        }

        }
        break;

    case TEMPERATURE_SENSOR:

       if((GATT_CLIENT_CONFIG_NOTIFICATION == \
                                        app_sensor_hub_temperature_sensor_client_char_config[0])
                                        && (0 != bt_connection_id))
        {
           status = wiced_bt_gatt_server_send_notification(
                                        bt_connection_id,
                                        HDLC_SENSOR_HUB_TEMPERATURE_SENSOR_VALUE,
                                        app_gatt_db_ext_attr_tbl[4].cur_len,
                                        app_gatt_db_ext_attr_tbl[4].p_data,NULL);
           if(WICED_BT_GATT_SUCCESS != status)
           {
               printf("Sending temperature sensor notification failed\r\n");
           }

        }
        break;

    }

}

/*******************************************************************************
* Function Name: bt_app_restore_config
********************************************************************************
* Summary:
* This function restore the config data from the flash memory.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void bt_app_restore_config(void)
{

    /* Read notification interval time from flash*/
    if(0u == flash_memory_read((uint8_t*)"NOTIFY_INX", 1u, &notify_index))
    {
        notify_index = 0;
    }
    printf("Notify interval values is set to %d ms \r\n",
                                     notify_interval_value[notify_index]);
}

/*******************************************************************************
* Function Name: bt_print_bd_address
********************************************************************************
* Summary: This is the utility function that prints the address of the 
*          Bluetooth device
*
* Parameters:
*  wiced_bt_device_address_t bdaddr : Bluetooth address
*
* Return:
*  None
*
*******************************************************************************/
void bt_print_bd_address(wiced_bt_device_address_t bdadr)
{
    for(uint8_t i=0;i<BD_ADDR_LEN-1;i++)
    {
        printf("%02X:",bdadr[i]);
    }
    printf("%02X\n",bdadr[BD_ADDR_LEN-1]);
}

/* END OF FILE [] */
