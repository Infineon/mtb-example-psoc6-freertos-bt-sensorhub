/*******************************************************************************
* File Name: board.c
*
* Description: This file contains board supported API's.
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
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include "board.h"
#include "sensor.h"
#include "bt_utils.h"
#include "bt_app.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define PWM_FREQUENCY_1KHZ              (1000u)

#define PWM_DUTY_CYCLE_0                (0.0)
#define PWM_DUTY_CYCLE_50               (50.0)
#define PWM_DUTY_CYCLE_100              (100.0)

#define BUTTON_LONGPRESS_INTERVAL       (5000u)
/* Interrupt priority for the GPIO connected to the user button */
#define BUTTON_INTERRUPT_PRIORITY       (7u)

#define BOARD_TASK_PRIORITY             (configMAX_PRIORITIES - 1u)
#define BOARD_TASK_STACK_SIZE           (512u)

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void board_led_init(void);
void board_button_init(void);
static void button_interrupt_callback(void *handler_arg, cyhal_gpio_event_t event);
/*******************************************************************************
* Extern Variables
*******************************************************************************/

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* FreeRTOS task handle for board task. Button task is used to handle button
 * events */
TaskHandle_t  board_task_handle;

/* PWM object */
cyhal_pwm_t pwm_obj[USER_LED_MAX];

/* Variables to keep the button timings. */
uint64_t button_pushed_time = 0u;
uint32_t button_previous_value = 1u;
uint32_t button_pushed_duration = 0u;

/* Button interrupt config data */
cyhal_gpio_callback_data_t gpio_cb_data =
{
    .callback = button_interrupt_callback,
    .callback_arg = NULL,
    .pin = NC,
    .next = NULL
};

/*******************************************************************************
* Function Name: board_init
********************************************************************************
*
* Summary:
*   Initialize the board with LED's and Buttons
*
* Parameters:
*   None
*
* Return:
*   cy_rslt_t  Result status
*
*******************************************************************************/
cy_rslt_t board_init(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    BaseType_t rtos_result;

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                                CY_RETARGET_IO_BAUDRATE);

    board_led_init();

    board_button_init();

    /* Create Button Task for processing board events */
    rtos_result = xTaskCreate(board_task,"Board Task", BOARD_TASK_STACK_SIZE,
                            NULL, BOARD_TASK_PRIORITY, &board_task_handle);
    if( pdPASS != rtos_result)
    {
        printf("Failed to create board task.\r\n");
        CY_ASSERT(0u);
    }

    return result;
}


/*******************************************************************************
* Function Name: board_led_init
********************************************************************************
*
* Summary:
*   Initialize the leds with PWM
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void board_led_init(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the PWM for USER_LED1 */
    result = cyhal_pwm_init_adv(&pwm_obj[USER_LED1], CYBSP_USER_LED1, NC,
                                     CYHAL_PWM_RIGHT_ALIGN, true, 0u, true, NULL);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM1 init failed with error code: %lu\r\n", (unsigned long) result);
    }

    /* Start the PWM */
    result = cyhal_pwm_start(&pwm_obj[USER_LED1]);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM1 start failed with error code: %lu\r\n", (unsigned long) result);
    }

    /* Initialize the PWM for USER_LED2*/
    result = cyhal_pwm_init_adv(&pwm_obj[USER_LED2], CYBSP_USER_LED2, NC, 
                                        CYHAL_PWM_RIGHT_ALIGN, true, 0u, true, NULL);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM2 init failed with error code: %lu\r\n", (unsigned long) result);
    }

    /* Start the PWM */
    result = cyhal_pwm_start(&pwm_obj[USER_LED2]);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM2 start failed with error code: %lu\r\n", (unsigned long) result);

    }

}


/*******************************************************************************
* Function Name: board_led_set_brightness
********************************************************************************
*
* Summary:
*   Set the led brightness over PWM
*
* Parameters:
*   index: index of LED
*   value: PWM duty cycle value
*
* Return:
*   None
*
*******************************************************************************/
void board_led_set_brightness(uint8_t index, uint8_t value)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cyhal_pwm_set_duty_cycle(&pwm_obj[index], value, PWM_FREQUENCY_1KHZ);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM set duty cycle failed with error code: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
}


/*******************************************************************************
* Function Name: board_led_set_state
********************************************************************************
*
* Summary:
*   Set the led state over PWM
*
* Parameters:
*   index: index of LED
*   value: ON/OFF state
*
* Return:
*   None
*
*******************************************************************************/
void board_led_set_state(uint8_t index, bool value)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cyhal_pwm_set_duty_cycle(&pwm_obj[index],
                                value?PWM_DUTY_CYCLE_0:PWM_DUTY_CYCLE_100, PWM_FREQUENCY_1KHZ);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM set duty cycle failed with error code: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
}


/*******************************************************************************
* Function Name: board_led_set_blink
********************************************************************************
*
* Summary:
*   Set the led frequency for PWM
*
* Parameters:
*   index: index of LED
*   value: value of PWM frequency
*
* Return:
*   None
*
*******************************************************************************/
void board_led_set_blink(uint8_t index, uint8_t value)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cyhal_pwm_set_duty_cycle(&pwm_obj[index], PWM_DUTY_CYCLE_50, value);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM set duty cycle failed with error code: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
}

/*******************************************************************************
* Function Name: board_button_init
********************************************************************************
*
* Summary:
*   Initialize the button with Interrupt
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void board_button_init(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the GPIO for user button */
    result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);

    if (CY_RSLT_SUCCESS != result)
    {
        printf("GPIO initialization failed! \r\n");
    }

    /* Configure GPIO interrupt. */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &gpio_cb_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_BOTH,
                                BUTTON_INTERRUPT_PRIORITY, true);
}

/*******************************************************************************
* Function Name: board_task
********************************************************************************
*
* Summary:
*   This task initialize the board and button events
*
* Parameters:
*   void *pvParameters: Not used
*
* Return:
*   None
*
*******************************************************************************/
void board_task(void *pvParameters)
{
    static uint32_t notify_value;

    for(;;)
     {
         /* Block till a notification is received. */
         xTaskNotifyWait(0, 0, &notify_value, portMAX_DELAY);

         switch(notify_value)
         {
         case BUTTON_PRESS:
             break;
         case BUTTON_PRESSED:
             printf("User button (SW2) pressed \r\n");
             if(++notify_index >= 4u) notify_index = 0;
             printf("Notify interval values is set to %d ms \r\n",
                                              notify_interval_value[notify_index]);
            /* Write notification interval time to flash*/
             flash_memory_write((uint8_t*)"NOTIFY_INX", 1u, (uint8_t*)&notify_index);
             break;
         case BUTTON_LONGPRESSED:
             printf("Factory reset the flash\r\n");
             flash_memory_reset();
             break;
         }
     }
}

/*******************************************************************************
* Function Name: button_interrupt_callback
********************************************************************************
* Summary:
*   GPIO interrupt handler.
*
* Parameters:
*  *handler_arg : Not used
*  event : Not used
*
*  Return:
*   None
*
*******************************************************************************/
static void button_interrupt_callback(void *handler_arg, cyhal_gpio_event_t event)
{
    uint32_t value =  cyhal_gpio_read(CYBSP_USER_BTN);
    uint32_t current_time =  xTaskGetTickCountFromISR();
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    if (value == button_previous_value)
    {
        return;
    }
    button_previous_value = value;

    if (value == CYBSP_BTN_PRESSED)
    {
        button_pushed_time = current_time;
        xTaskNotifyFromISR(board_task_handle, (uint32_t) BUTTON_PRESS,
                            eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);
        return;
    }

    /* Button is released */
    button_pushed_duration = current_time - button_pushed_time;
    xTaskNotifyFromISR(board_task_handle,
           (uint32_t) (button_pushed_duration < BUTTON_LONGPRESS_INTERVAL)
           ?BUTTON_PRESSED:BUTTON_LONGPRESSED,
           eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);
}

/* [] END OF FILE */
