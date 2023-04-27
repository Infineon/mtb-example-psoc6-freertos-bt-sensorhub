/*******************************************************************************
* File Name: bt_utils.c
*
* Description: This file contains flash memory access and helper functions.
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
#include "cy_retarget_io.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "bt_utils.h"
#include "mtb_kvstore.h"
#include "stdlib.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define AUXILIARY_FLASH_BLOCK                (1)
#define AUXILIARY_FLASH_LENGTH               (16) /* Hexadecimal */
#define FLASH_CONFIG_MAX_LEN                 (256)

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
uint32_t bd_read_size(void* context, uint32_t addr);
uint32_t bd_program_size(void* context, uint32_t addr);
uint32_t bd_erase_size(void* context, uint32_t addr);
cy_rslt_t bd_read(void* context, uint32_t addr, uint32_t length, uint8_t* buf);
cy_rslt_t bd_program(void* context, uint32_t addr, uint32_t length, const uint8_t* buf);
cy_rslt_t bd_erase(void* context, uint32_t addr, uint32_t length);

/*******************************************************************************
* Global Variables
*******************************************************************************/
static cyhal_flash_t flash_obj;
static cyhal_flash_block_info_t flash_block_info;
cyhal_flash_info_t flash_info;

mtb_kvstore_t kv_store_obj;

/*Kvstore block device*/

mtb_kvstore_bd_t block_device =
{
    .read         = bd_read,
    .program      = bd_program,
    .erase        = bd_erase,
    .read_size    = bd_read_size,
    .program_size = bd_program_size,
    .erase_size   = bd_erase_size,
    .context      = &flash_obj,
};

/*******************************************************************************
* Function Name: flash_memory_init
********************************************************************************
* Summary:
* This function Initialize the flash memory.
*
* Parameters:
*  None
*
* Return:
*  cy_rslt_t : returns the result status.
*
*******************************************************************************/
cy_rslt_t flash_memory_init(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /*Define the space to be used Storage*/
    uint32_t length = 0u;
    uint32_t start_addr = 0;

    result = cyhal_flash_init(&flash_obj);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("Internal flash initialization failed! \n");
        CY_ASSERT(0);
    }

    cyhal_flash_get_info(&flash_obj, &flash_info);
    flash_block_info = flash_info.blocks[AUXILIARY_FLASH_BLOCK];

    length = flash_block_info.page_size * AUXILIARY_FLASH_LENGTH;

    start_addr = flash_block_info.start_address + flash_block_info.size - length;

    /*Initialize kv-store library*/
    result = mtb_kvstore_init(&kv_store_obj, start_addr, length, &block_device);
    /*Check if the kv-store initialization was successfull*/
    if (CY_RSLT_SUCCESS !=  result)
    {
        printf("Kv-store init failed with error code = %x\r\n", (int)result);
    }
    return result;
}

/*******************************************************************************
* Function Name: flash_memory_read
********************************************************************************
* Summary:
* This function reads data from flash memory.
*
* Parameters:
*  config_item_id : index of data
*  len :data length
*  buf : data buffer
*
* Return:
*  uint32_t : returns the length of data size.
*
*******************************************************************************/
uint32_t flash_memory_read(uint8_t* key, uint32_t len, uint8_t* buf)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if (len >= FLASH_CONFIG_MAX_LEN)
    {
        printf("Flash read failed reached maximum length %d!", (uint16_t)len);
        return 0u;
    }

    result = mtb_kvstore_read(&kv_store_obj, (const char*)key, (uint8_t*)buf, &len);
    if(CY_RSLT_SUCCESS != result)
    {

        /*Suppress the ITEM NOT FOUND error message */
        if(MTB_KVSTORE_ITEM_NOT_FOUND_ERROR == result)
        {
            return 0u;
        }
        printf("Flash read failed with error code : 0x%x\r\n", (int)result);

    }
    return (len);
}

/*******************************************************************************
* Function Name: flash_memory_write
********************************************************************************
* Summary:
* This function write data to flash memory.
*
* Parameters:
*  config_item_id : index of data
*  len :data length
*  buf : data buffer
*
* Return:
*  uint32_t : returns the length of data size.
*
*******************************************************************************/
uint32_t flash_memory_write(uint8_t* key, uint32_t len, uint8_t* buf)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if (len >= FLASH_CONFIG_MAX_LEN)
    {
        printf("Flash write failed reached maximum length %d!", (uint16_t)len);
        return 0;
    }

    result = mtb_kvstore_write(&kv_store_obj, (const char*)key, (uint8_t*)buf, len);
    if(CY_RSLT_SUCCESS != result)
    {
        /*Suppress the ITEM NOT FOUND error message */
        if(MTB_KVSTORE_ITEM_NOT_FOUND_ERROR == result)
        {
            return 0u;
        }
        printf("Flash write failed with error code: 0x%x\r\n", (int)result);
        return 0;
    }
    return (len);
}

/*******************************************************************************
* Function Name: flash_memory_delete
********************************************************************************
* Summary:
* This function delete data from the flash memory.
*
* Parameters:
*  config_item_id : index of data
*
* Return:
*  uint32_t : returns the status.
*
*******************************************************************************/
uint32_t flash_memory_delete(uint8_t* key)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = mtb_kvstore_delete(&kv_store_obj, (const char*)key);
    if(CY_RSLT_SUCCESS != result)
    {
        /*Suppress the ITEM NOT FOUND error message */
        if(MTB_KVSTORE_ITEM_NOT_FOUND_ERROR == result)
        {
            return 0u;
        }
        printf("Flash delete failed with error code: 0x%x\r\n", (int)result);

        return 0u;
    }
    return 1u;
}

/*******************************************************************************
* Function Name: flash_memory_reset
********************************************************************************
* Summary:
* This function reset data from the flash memory.
*
* Parameters:
*  None
*
* Return:
*  uint32_t : returns the status.
*
*******************************************************************************/
uint32_t flash_memory_reset(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = mtb_kvstore_reset(&kv_store_obj);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("Flash reset failed with error code: 0x%x\r\n", (int)result);
        return 0u;
    }
    return 1u;
}

/*******************************************************************************
* Function Name: bd_read_size
********************************************************************************
* Summary:
* This function read block data size.
*
* Parameters:
*  context : block data context
*  addr : block data address
*
* Return:
*  uint32_t : TRUE.
*
*******************************************************************************/
uint32_t bd_read_size(void* context, uint32_t addr)
{
    (void)context;
    (void)addr;
    return 1u;
}

/*******************************************************************************
* Function Name: bd_program_size
********************************************************************************
* Summary:
* This function program the block.
*
* Parameters:
*  context : block data context
*  addr : block data address
*
* Return:
*  uint32_t : block page size.
*
*******************************************************************************/
uint32_t bd_program_size(void* context, uint32_t addr)
{
    (void)context;
    if(flash_info.block_count != 0)
        return flash_block_info.page_size;
    else
        return CY_RSLT_TYPE_ERROR;
}

/*******************************************************************************
* Function Name: bd_erase_size
********************************************************************************
* Summary:
* This function erase the block data.
*
* Parameters:
*  context : block data context
*  addr : block data address
*
* Return:
*  uint32_t : block sector size.
*
*******************************************************************************/
uint32_t bd_erase_size(void* context, uint32_t addr)
{
    (void)context;
    if(flash_info.block_count != 0)
        return flash_block_info.sector_size;
    else
        return CY_RSLT_TYPE_ERROR;
}

/*******************************************************************************
* Function Name: bd_read
********************************************************************************
* Summary:
* This function read block data.
*
* Parameters:
*  context : block data context
*  addr : block data address
*
* Return:
*  uint32_t : TRUE.
*
*******************************************************************************/
cy_rslt_t bd_read(void* context, uint32_t addr, uint32_t length, uint8_t* buf)
{
    (void)context;
    return cyhal_flash_read(&flash_obj, addr, buf, length);
}

/*******************************************************************************
* Function Name: bd_program
********************************************************************************
* Summary:
* This function program the block data.
*
* Parameters:
*  context : block data context
*  addr : block data address
*  length : block data length
*  buf : block data buffer
*
* Return:
*  cy_rslt_t : status.
*
*******************************************************************************/
cy_rslt_t bd_program(void* context, uint32_t addr, uint32_t length, const uint8_t* buf)
{
    (void)context;
    uint32_t prog_size = bd_program_size(context, addr);
    CY_ASSERT(0 == (length % prog_size));
    volatile cy_rslt_t result = CY_RSLT_SUCCESS;
    for(uint32_t loc = addr; result == CY_RSLT_SUCCESS && loc < addr + length;
         loc += prog_size, buf += prog_size)
    {
        result = cyhal_flash_program(&flash_obj, loc, (const uint32_t*)buf);
    }
    return result;
}


/*******************************************************************************
* Function Name: bd_erase
********************************************************************************
* Summary:
* This function erase the block data.
*
* Parameters:
*  context : block data context
*  addr : block data address
*  length : block data length
*
* Return:
*  cy_rslt_t : status.
*
*******************************************************************************/
cy_rslt_t bd_erase(void* context, uint32_t addr, uint32_t length)
{
    (void)context;
    uint32_t erase_size = bd_erase_size(context, addr);
    CY_ASSERT(0 == (length % erase_size));
    cy_rslt_t result = CY_RSLT_SUCCESS;
    for(uint32_t loc = addr; result == CY_RSLT_SUCCESS && loc < addr + length; loc += erase_size)
    {
        result = cyhal_flash_erase(&flash_obj, loc);
    }
    return result;
}

/* [] END OF FILE */
