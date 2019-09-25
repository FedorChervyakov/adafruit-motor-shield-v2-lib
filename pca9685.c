/*
 * =====================================================================================
 *
 *       Filename:  pca9685.c
 *
 *    Description:  Driver for PCA9685 i2c led controller
 *
 *        Version:  1.0
 *        Created:  24/09/19 23:29:40
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Fedor Chervyakov (), fchervyakov@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include "pca9685.h"

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  duty_to_registers
 *  Description:  Covert 12-bit duty cycle to ON_L ON_H OFF_L OFF_H registers
 * =====================================================================================
 */
void duty_to_registers (uint8_t *reg, uint16_t duty_cycle)
{
    uint8_t on_l, on_h, off_l, off_h;

    if (duty_cycle == 0)
    { 
        /* Full OFF */
        on_l  = 0x00;
        on_h  = 0x00;
        off_l = 0x00;
        off_h = 1 << 4;
    }
    else if (duty_cycle < 4096) 
    {
        /* PWM operation, no delay */
        on_l = 0x00;
        on_h = 0x00;
        off_l = (uint8_t) duty_cycle;
        off_h = (uint8_t) (duty_cycle >> 8);
    }
    else
    {
        /* Duty cycle > 4095 -> out full ON */
        on_l = 0x00;
        on_h = 1 << 4;
        off_l = 0x00;
        off_h = 0x00;
    }

    reg[0] = on_l;
    reg[1] = on_h;
    reg[2] = off_l;
    reg[3] = off_h;

    return;
}		/* -----  end of function duty_to_registers  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  pca9685_init
 *  Description:  Initialize PCA9685. i.e. reset and configure autoincrement
 * =====================================================================================
 */
int8_t pca9685_init(pca9685_dev *dev)
{
    int8_t status = PCA9685_OK;

    /* Reset the device */ 
    status = pca9685_reset(dev);
    if (status != PCA9685_OK) 
    {
        return status;
    }

    /* Configure the device */

    return status;
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  pca9685_reset
 *  Description:  Perform software reset of PCA9685 by calling 0x00 address in read mode
 * =====================================================================================
 */
int8_t pca9685_reset(pca9685_dev *dev)
{
    /* SWRST procedure is described in the datasheet    */
    int8_t status = PCA9685_OK;

    /* Call software reset i2c address with dummy data as per datasheet  */
    status = dev->read(PCA9685_I2C_SOFTWARE_RESET_ADDRESS, (uint8_t) 0x06, NULL, 0);
    if (status != PCA9685_OK) 
    {
        return PCA9685_E_COMM_FAIL;
    }

    return status;
}


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  pca9685_setAll
 *  Description:  Configure all pins at once
 * =====================================================================================
 */
int8_t pca9685_setAll(pca9685_dev *dev, uint16_t duty_cycle)
{
    int8_t status = PCA9685_OK;                 /* Status variable */
    uint8_t reg[4] = {0}                        /* Output control registers */
    
    /* Convert duty cycle to registers */
    duty_to_registers((uint8_t *) reg, duty_cycle);

    /* Write to ALL_LED registers */
    status = dev->write(dev->dev_id, PCA9685_ALL_LED_ON_L, (uint8_t *) reg, 4);
    if (status != PCA9685_OK)
    {
        return PCA9685_E_COMM_FAIL;
    }

    return status;
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  pca9685_setPin
 *  Description:  Configure single output pin (0 to 15)
 * =====================================================================================
 */
int8_t pca9685_setPin( pca9685_dev *dev, uint8_t pin, uint16_t duty_cycle)
{
    int8_t status = PCA9685_OK;
    uint8_t reg[4] = {0};

    duty_to_registers((uint8_t *) reg, duty_cycle);

    status = dev->write(dev->dev_id, PCA9685_LED0_ON_L + 4*pin, (uint8_t *) reg, 4);
    if (status != PCA9685_OK)
    {
        return PCA9685_E_COMM_FAIL;
    }

    return status;
}		/* -----  end of function pca9685_setPin   ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  pca9685_setPrescaler
 *  Description:  Set PWM frequency
 * =====================================================================================
 */
int8_t pca9685_setPrescaler( pca9685_dev *dev, uint16_t PWM_frequency)
{
    int8_t status = PCA9685_OK;

    uint8_t prescale_value = (uint8_t) ((float) 25E6 / (float) (4096 * PWM_frequency));
    prescale_value--;

    status = dev->write(dev->dev_id, PCA9685_PRE_SCALE, (uint8_t *) &prescale_value, 1);
    if (status != PCA9685_OK)
    {
        return PCA9685_E_COMM_FAIL;
    }

    return status;
}		/* -----  end of function pca9685_setPrescaler  ----- */
