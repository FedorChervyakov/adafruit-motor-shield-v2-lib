/*
 * =====================================================================================
 *
 *       Filename:  motor_shield.c
 *
 *    Description:  Driver for the Adafruit Motor Shield v2
 *
 *        Version:  1.0
 *        Created:  24/09/19 22:54:55
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Fedor Chervyakov (), fchervyakov@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include "motor_shield.h"

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  MS_struct_Init
 *  Description:  Return MotorShield_dev_t initialized with passed settings
 * =====================================================================================
 */
MotorShield_dev_t MS_struct_Init(pca9685_dev pca9685, uint8_t driver1_mode, uint8_t driver2_mode)
{
    MotorShield_dev_t MS;
    tb6612_dev_t tb6612_1, tb6612_2;

    /* Initialize H-bridge structures with corresponding pca9685 outputs */
    /* TB6612 number 1 (IC1 on the schematic) */
    tb6612_1.H_A = (H_bridge_t) { .PWM_pin = PCA9685_LED8, 
                                  .IN1_pin = PCA9685_LED10,
                                  .IN2_pin = PCA9685_LED9 };
    tb6612_1.H_B = (H_bridge_t) { .PWM_pin = PCA9685_LED13,
                                  .IN1_pin = PCA9685_LED11,
                                  .IN2_pin = PCA9685_LED12};

    /* TB6612 number 2 (IC3 on the schematic) */
    tb6612_2.H_A = (H_bridge_t) { .PWM_pin = PCA9685_LED2,
                                  .IN1_pin = PCA9685_LED4,
                                  .IN2_pin = PCA9685_LED3 };
    tb6612_2.H_B = (H_bridge_t) { .PWM_pin = PCA9685_LED7,
                                  .IN1_pin = PCA9685_LED5,
                                  .IN2_pin = PCA9685_LED6 };

    /* Initialize TB6612 structures with passed modes */
    tb6612_1.mode = driver1_mode;
    tb6612_2.mode = driver2_mode;

    /* Initialize MotorShield_dev_t structure */
    MS.pca9685 = pca9685;
    MS.tb6612_1 = tb6612_1;
    MS.tb6612_2 = tb6612_2;

    return MS;
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  MS_Init
 *  Description:  Initialize the MotorShield. i.e. initialize the pca9685
 * =====================================================================================
 */
int8_t MS_Init(MotorShield_dev_t *MS)
{
    int8_t status = MS_OK;
    
    status = pca9685_init(&MS->pca9685);
    if (status != PCA9685_OK)
    {
        return MS_E_INIT_FAIL;
    }

    return status;
}		/* -----  end of function MS_Init  ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  MS_DC_drive
 *  Description:  Configure single H-bridge controller
 * =====================================================================================
 */
int8_t MS_DC_drive(MotorShield_dev_t *MS, uint8_t motor,
                   uint8_t mode, uint16_t duty_cycle)
{
    int8_t status = MS_OK;                      /* Status variable */
    uint16_t duty_cycles[3];                    /* PWM, IN1, IN2 */
    uint16_t sorted_duty[3];                    /* Duty cycles in pin order */
    uint8_t s_pin;                              /* Lowest pin in sequence */

    /* Check if specified DC motor is not available */
    if (!((((motor == MS_DC_MOTOR_1) || (motor == MS_DC_MOTOR_2)) \
        && (MS->tb6612_1.mode == TB6612_2DC_MODE)) \
        || (((motor == MS_DC_MOTOR_3) || (motor == MS_DC_MOTOR_4)) \
        && (MS->tb6612_2.mode == TB6612_2DC_MODE))))
    {
        return MS_E_INVALID_MOTOR;
    }
    
    /* Initialize duty cycles for PWM, IN1, and IN2 pins */
    duty_cycles[0] = duty_cycle;
    switch (mode)
    {
        case MS_CW:
            /* IN1: Low;  IN2: High */
            duty_cycles[1] = 0;
            duty_cycles[2] = 4096;
            break;
        case MS_CCW:
            /* IN1: High; IN2: Low  */
            duty_cycles[1] = 4096;
            duty_cycles[2] = 0;
            break;
        case MS_STOP:
            /* IN1: Low;  IN2: Low  */
            duty_cycles[1] = 0;
            duty_cycles[2] = 0;
            break;
        case MS_SHORT_BRAKE:
            /* IN1: High; IN2: High */
            duty_cycles[1] = 4096;
            duty_cycles[2] = 4096;
            break;
        default:
            /* Same as MS_STOP case */
            /* IN1: Low;  IN2: Low  */
            duty_cycles[1] = 0;
            duty_cycles[2] = 0;
            break;
    }

    /* Sort duty cycles in pin order */
    switch (motor) 
    {
        case MS_DC_MOTOR_1:
            sorted_duty[0] = duty_cycles[0];    /* Pin 8,  PWM */
            sorted_duty[1] = duty_cycles[2];    /* Pin 9,  IN2 */
            sorted_duty[2] = duty_cycles[1];    /* Pin 10, IN1 */
            s_pin = PCA9685_LED8;
            break;
        case MS_DC_MOTOR_2:
            sorted_duty[0] = duty_cycles[1];    /* Pin 11, IN1 */
            sorted_duty[1] = duty_cycles[2];    /* Pin 12, IN2 */
            sorted_duty[2] = duty_cycles[0];    /* Pin 13, PWM */
            s_pin = PCA9685_LED11;
            break;
        case MS_DC_MOTOR_3:
            sorted_duty[0] = duty_cycles[0];    /* Pin 2,  PWM */
            sorted_duty[1] = duty_cycles[2];    /* Pin 3,  IN2 */
            sorted_duty[2] = duty_cycles[1];    /* Pin 4,  IN1 */
            s_pin = PCA9685_LED2;
            break;
        case MS_DC_MOTOR_4:
            sorted_duty[0] = duty_cycles[1];    /* Pin 5,  IN1 */
            sorted_duty[1] = duty_cycles[2];    /* Pin 6,  IN2 */
            sorted_duty[2] = duty_cycles[0];    /* Pin 7,  PWM */
            s_pin = PCA9685_LED5;
            break;
    }

    /* Write output config registers */
    status = pca9685_setPins(MS->pca9685, s_pin, (uint16_t *) sorted_duty, 3);
    if (status != PCA9685_OK)
    {
        return MS_E_DRIVE_FAIL;
    }

    return status;
}		/* -----  end of function MS_DC_drive  ----- */
