/******************************************************************************
* File Name: sensor.h
*
* Description: This file is the public interface of sensor.c
*
* Related Document: See README.md
*
*******************************************************************************
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
 * Include guard
 ******************************************************************************/
#ifndef SENSOR_H_
#define SENSOR_H_

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "timers.h"
#include "mtb_bmi160.h"
#include "mtb_thermistor_ntc_gpio.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* Sensor index values */
enum
{
    MOTION_SENSOR,
    TEMPERATURE_SENSOR,
};

/*******************************************************************************
 * Data structure and enumeration
 ******************************************************************************/
/* Structure used for storing APP data */
typedef struct
{
    mtb_bmi160_data_t data;
    uint8_t orientation;
    uint32 temperature;
} sensor_data_t;

/*******************************************************************************
* Variable Definitions
*******************************************************************************/
extern sensor_data_t app_data;

/******************************************************************************
 * Function Definitions
 ******************************************************************************/
void sensor_task(void* pvParameters);
void get_motion_data(void);
void get_temperature(void);
void get_orientation(uint8_t *orientation);

#endif /* SENSOR_H_ */