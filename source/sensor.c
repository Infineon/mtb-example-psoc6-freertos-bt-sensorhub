/*******************************************************************************
* File Name:   sensor.c
*
* Description: This file contains the sensor tasks and drivers
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
*******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include "inttypes.h"
#include "bt_app.h"
#include "bt_utils.h"
#include "board.h"
#include "sensor.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* I2C Clock frequency in Hz (1MHZ) */
#define I2C_CLK_FREQ_HZ                         (1000000u)

/* Thermistor pin definition in CY8CKIT_028_EPD Shield*/
/** Pin for the Thermistor VDD signal */
#define PIN_THERM_VDD                           (CYBSP_A0)
/** Pin for the Thermistor Output option1 signal */
#define PIN_THERM_OUT1                          (CYBSP_A1)
/** Pin for the Thermistor Output option2 signal */
#define PIN_THERM_OUT2                          (CYBSP_A2)
/** Pin for the Thermistor Ground signal */
#define PIN_THERM_GND                           (CYBSP_A3)

/* Thermistor Constants */
/** Resistance of the reference resistor */
#define SENSOR_THERM_R_REF                         (float)(10000)
/** Beta constant of the (NCP18XH103F03RB) thermistor (3380 Kelvin).See the
 * thermistor datasheet for more details. */
#define SENSOR_THERM_B_CONST                       (float)(3380)
/** Resistance of the thermistor is 10K at 25 degrees C (from datasheet)
 * Therefore R0 = 10000 Ohm, and T0 = 298.15 Kelvin, which gives
 * R_INFINITY = R0 e^(-B_CONSTANT / T0) = 0.1192855 */
#define SENSOR_THERM_R_INFINITY                    (float)(0.1192855)

/*******************************************************************************
 * Structures
 ******************************************************************************/
/* HAL structure for I2C */
cyhal_i2c_t mi2c;
cyhal_i2c_cfg_t mi2c_cfg = {
    .is_slave = false,
    .address = 0,
    .frequencyhal_hz = I2C_CLK_FREQ_HZ
};

/* Structure for thermistor */
mtb_thermistor_ntc_gpio_cfg_t thermistor_cfg = {
    .r_ref = SENSOR_THERM_R_REF,
    .b_const = SENSOR_THERM_B_CONST,
    .r_infinity = SENSOR_THERM_R_INFINITY,
};

/*******************************************************************************
* Global constants
*******************************************************************************/
/* Orientation types:
 * Indicates which edge of the board is pointing towards the ceiling/sky
 */
typedef enum
{
    ORIENTATION_NULL            = 0,    /* Default orientation state used for initialization purposes */
    ORIENTATION_TOP_EDGE        = 1,    /* Top edge of the board points towards the ceiling */
    ORIENTATION_BOTTOM_EDGE     = 2,    /* Bottom edge of the board points towards the ceiling */
    ORIENTATION_LEFT_EDGE       = 3,    /* Left edge of the board (USB connector side) points towards the ceiling */
    ORIENTATION_RIGHT_EDGE      = 4,    /* Right edge of the board points towards the ceiling */
    ORIENTATION_DISP_UP         = 5,    /* Display faces up (towards the sky/ceiling) */
    ORIENTATION_DISP_DOWN       = 6     /* Display faces down (towards the ground) */
} orientation_t;

const char* orientation[] = {"UNKNOWN", "TOP_EDGE", "BOTTON_EDGE", "LEFT_EDGE",
                                        "RIGHT_EDGE", "DISP_UP", "DISP_DOWN"};

/*******************************************************************************
* Global Variables
*******************************************************************************/
mtb_bmi160_t motion_sensor;

mtb_thermistor_ntc_gpio_t thermistor;

mtb_bmi160_data_t acc_data;

cyhal_adc_t adc;

/* Application data objects */
sensor_data_t app_data;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
/*******************************************************************************
* Function Name: sensor_task
********************************************************************************
* Summary:
*  The function initialize the sensor IO's and interface.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
* Return
*  None
*
*******************************************************************************/
void sensor_task(void* param)
{
    /* Status variable to indicate the result of various operations */
    cy_rslt_t result;

    /* Remove warning for unused parameter */
    (void)param;

    /* Initialize i2c for motion sensor */
    result = cyhal_i2c_init(&mi2c, (cyhal_gpio_t) CYBSP_I2C_SDA,
                            (cyhal_gpio_t) CYBSP_I2C_SCL, NULL);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("Sensor:failed to init I2C!\r\n");
        CY_ASSERT(0u);
    }
    result = cyhal_i2c_configure(&mi2c, &mi2c_cfg);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("Sensor:failed to configure I2C!\r\n");
        CY_ASSERT(0u);
    }
    printf("Sensor:I2C1 initialization done!\r\n");

    /* Initialize motion sensor */
    result = mtb_bmi160_init_i2c(&motion_sensor, &mi2c, MTB_BMI160_DEFAULT_ADDRESS);

    if(CY_RSLT_SUCCESS != result)
    {
        printf("Failed to init the motion sensor!\r\n");
        CY_ASSERT(0u);
    }
    printf("Sensor:motion sensor initialization done!\r\n");
    
    /* Intialize the adc */
    result = cyhal_adc_init(&adc, PIN_THERM_OUT1, NULL);

    if(CY_RSLT_SUCCESS != result)
    {
        printf("Sensor:failed to configure ADC!\r\n");
        CY_ASSERT(0u);
    }
    printf("Sensor:ADC initialization done!\r\n");

    /* Initialize thermistor */
    result = mtb_thermistor_ntc_gpio_init(&thermistor, &adc,
                                          PIN_THERM_GND,
                                          PIN_THERM_VDD,
                                          PIN_THERM_OUT1,
                                          &thermistor_cfg,
                                          MTB_THERMISTOR_NTC_WIRING_VIN_R_NTC_GND);

    if(CY_RSLT_SUCCESS != result)
    {
        printf("Sensor:failed to init the thermistor!\r\n");
        CY_ASSERT(0u);
    }
    printf("Sensor:thermistor initialization done!\r\n");

    for(;;)
    {
        /* Check for a notification status. */

        if( NOTIFIY_ON == notify_motion_sensor)
        {
            get_motion_data();
            /* Motion data to be send*/
            memcpy(&app_sensor_hub_motion_sensor[0], &app_data.data.accel, 6u);
            memcpy(&app_sensor_hub_motion_sensor[6], &app_data.data.gyro, 6u);
            app_sensor_hub_motion_sensor[12] = app_data.orientation;
            bt_app_send_notification(MOTION_SENSOR);
        }

        if(NOTIFIY_ON == notify_temperature_sensor)
        {
            get_temperature();
            /* Temperature data to be send*/
            memcpy(&app_sensor_hub_temperature_sensor[0], &app_data.temperature, 4u);

            bt_app_send_notification(TEMPERATURE_SENSOR);
        }

        /* Notification Delay */
        vTaskDelay(notify_interval_value[notify_index]);

    }
}

/*******************************************************************************
* Function Name: get_motion_data
********************************************************************************
* Summary:
*  The function reads the motion sensor data and copy to the structure.
*
* Parameters:
*  None
*
* Return
*  None
*
*******************************************************************************/
void get_motion_data(void)
{
    /* Block the I2C resource while reading the motion sensor data */
    //xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
    mtb_bmi160_read(&motion_sensor, &acc_data);
    /* Release the I2C resource after reading the motion sensor data */
    //xSemaphoreGive(i2c_semaphore);
    printf("Motion Sensor:accel: X:%6d Y:%6d Z:%6d\r\n", acc_data.accel.x, 
                                           acc_data.accel.y, 
                                           acc_data.accel.z);

    printf("Motion Sensor:gyro : X:%6d Y:%6d Z:%6d\r\n", acc_data.gyro.x, 
                                               acc_data.gyro.y, 
                                               acc_data.gyro.z);

    memcpy(&app_data.data,&acc_data,sizeof(mtb_bmi160_data_t));

    get_orientation(&app_data.orientation);
    printf("Motion Sensor:orientation = %s\r\n", orientation[app_data.orientation]);
}

/*******************************************************************************
* Function Name: get_temperature
********************************************************************************
* Summary:
*  The function reads the thermistor values and copies to the structure.
*
* Parameters:
*  None
*
* Return
*  None
*
*******************************************************************************/
void get_temperature(void)
{
    float temp;
    temp = mtb_thermistor_ntc_gpio_get_temp(&thermistor);
    app_data.temperature = (uint32_t) (temp * 100);
    printf("Thermistor Sensor:temperature (Celsius) = %f\n\r", (double)temp);
}

/*******************************************************************************
* Function Name: get_orientation
********************************************************************************
* Summary:
*  The function reads the thermistor values and copies to the structure.
*
* Parameters:
*  uint8_t *orientation : pointer to get orientation values 
*
* Return
*  None
*
*******************************************************************************/
void get_orientation(uint8_t *orientation)
{
    /* Variables used to store absolute values of the accelerometer data */
    uint16_t abs_x, abs_y, abs_z;

    /* Clear the previous orientation data */
    *orientation = ORIENTATION_NULL;

    /* Get the absolute values of the accelerations along each axes */
    abs_x = abs(acc_data.accel.x);
    abs_y = abs(acc_data.accel.y);
    abs_z = abs(acc_data.accel.z);

    /* Z axis (perpendicular to face of the display) is most aligned with gravity. */
    if ((abs_z > abs_x) && (abs_z > abs_y))
    {
        if (acc_data.accel.z < 0)
        {
            /* Display faces down (towards the ground) */
            *orientation = ORIENTATION_DISP_DOWN;
        }
        else
        {
            /* Display faces up (towards the sky/ceiling) */
            *orientation = ORIENTATION_DISP_UP;
        }
    }
    /* Y axis (parallel with shorter edge of board) is most aligned with gravity. */
    else if ((abs_y > abs_x) && (abs_y > abs_z))
    {
        if (acc_data.accel.y > 0)
        {
            /* Display has an inverted landscape orientation */
            *orientation = ORIENTATION_BOTTOM_EDGE;
        }
        else
        {
            /* Display has landscape orientation */
            *orientation = ORIENTATION_TOP_EDGE;
        }
    }
    /* X axis (parallel with longer edge of board) is most aligned with gravity. */
    else
    {
        if (acc_data.accel.x < 0)
        {
            /* Display has an inverted portrait orientation */
            *orientation = ORIENTATION_RIGHT_EDGE;
        }
        else
        {
            /* Display has portrait orientation */
            *orientation = ORIENTATION_LEFT_EDGE;
        }
    }
}

/* END OF FILE [] */
