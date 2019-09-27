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
    if (duty_cycle == 0)
    { 
        /* Full OFF */
        reg[0] = 0x00;
        reg[1] = 0x00;
        reg[2] = 0x00;
        reg[3] = 1 << 4;
    }
    else if (duty_cycle < 4096) 
    {
        /* PWM operation, no delay */
        reg[0] = 0x00;
        reg[1] = 0x00;
        reg[2] = (uint8_t) duty_cycle;
        reg[3] = (uint8_t) (duty_cycle >> 8);
    }
    else
    {
        /* Duty cycle > 4095 -> out full ON */
        reg[0] = 0x00;
        reg[1] = 1 << 4;
        reg[2] = 0x00;
        reg[3] = 0x00;
    }

    return;
}		/* -----  end of function duty_to_registers  ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  pca9685_init_struct
 *  Description:  Returns a pca9685_dev structure initialized with passed arguments
 * =====================================================================================
 */
pca9685_dev pca9685_init_struct( uint8_t dev_id, /* I2C address */
                                 pca9685_i2c_com_fptr_t i2c_write_fp,
                                 pca9685_i2c_com_fptr_t i2c_read_fp,
                                 pca9685_delay_fptr_t   delay_ms_fp,
                                 uint16_t frequency)
{
    pca9685_dev dev = { .dev_id = dev_id, .read = i2c_read_fp,
                        .write = i2c_write_fp, .delay = delay_ms_fp,
                        .frequency = frequency };

    return dev;
}		/* -----  end of function pca9685_init_struct  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  pca9685_init
 *  Description:  Initialize PCA9685. i.e. reset, configure autoincrement and prescaler
 * =====================================================================================
 */
int8_t pca9685_init(struct pca9685_dev *dev)
{
    int8_t status = PCA9685_OK;

    /* Reset the device */ 
    status = pca9685_reset(dev);
    if (status != PCA9685_OK) 
    {
        return PCA9685_E_COMM_FAIL;    
    }

    /* Configure the prescaler */
    status = pca9685_setPrescaler(dev, dev->frequency);
    if (status != PCA9685_OK)
    {
        return PCA9685_E_COMM_FAIL;    
    }

    /* Configure MODE1 register
     *
     * Disable restart, use internal clock,
     * autoincrement enabled, normal mode (oscillator on),
     * device does not respond to SUB1,2,3 and ALLCALL
     * I2C addresses
     */
    uint8_t mode1 = PCA9685_MODE1_RESTART | PCA9685_MODE1_AI;

    status = dev->write(dev->dev_id, PCA9685_MODE1, (uint8_t *) &mode1, 1);
    if (status != PCA9685_OK)
    {
        return PCA9685_E_COMM_FAIL;
    }

    return status;
}		/* -----  end of function pca9685_init  ---------- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  pca9685_reset
 *  Description:  Perform software reset of PCA9685 by calling 0x00 address in read mode
 * =====================================================================================
 */
int8_t pca9685_reset(struct pca9685_dev *dev)
{
    /* SWRST procedure is described in the datasheet    */
    int8_t status = PCA9685_OK;

    /* Call general call i2c address with swrst address as per datasheet  */
    status = dev->read(PCA9685_I2C_GENERAL_CALL_ADDRESS, \
                       PCA9685_I2C_SOFTWARE_RESET_ADDRESS, \
                       NULL, 0);
    if (status != PCA9685_OK) 
    {
        return PCA9685_E_COMM_FAIL;
    }

    return status;
}		/* -----  end of function pca9685_reset  --------- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  pca9685_setAll
 *  Description:  Configure all pins at once
 * =====================================================================================
 */
int8_t pca9685_setAll(struct pca9685_dev *dev, uint16_t duty_cycle)
{
    int8_t status = PCA9685_OK;                 /* Status variable */
    uint8_t reg[4] = {0};                       /* Output control registers */
    
    /* Convert duty cycle to registers */
    duty_to_registers((uint8_t *) reg, duty_cycle);

    /* Write to ALL_LED registers */
    status = dev->write(dev->dev_id, PCA9685_ALL_LED_ON_L, (uint8_t *) reg, 4);
    if (status != PCA9685_OK)
    {
        return PCA9685_E_COMM_FAIL;
    }

    return status;
}		/* -----  end of function pca9685_setAll  -------- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  pca9685_setPin
 *  Description:  Configure single output pin (0 to 15)
 * =====================================================================================
 */
int8_t pca9685_setPin(struct pca9685_dev *dev, uint8_t pin, uint16_t duty_cycle)
{
    int8_t status = PCA9685_OK;                 /* Status variable */
    uint8_t reg[4] = {0};                       /* Output control registers */

    /* Convert duty cycle to registers */
    duty_to_registers((uint8_t *) reg, duty_cycle);

    /* Write to 4 registers that correspond to a patricular pin */
    status = dev->write(dev->dev_id, PCA9685_LED0_ON_L + 4*pin, (uint8_t *) reg, 4);
    if (status != PCA9685_OK)
    {
        return PCA9685_E_COMM_FAIL;
    }

    return status;
}		/* -----  end of function pca9685_setPin   ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  pca9685_setPins
 *  Description:  Configure consecutive output pins
 * =====================================================================================
 */
int8_t pca9685_setPins (struct pca9685_dev *dev, uint8_t pin,
                         uint16_t *duty_cycles, uint8_t len )
{
    int8_t status = PCA9685_OK;                 /* Status variable */
    uint8_t reg[64] = {0};                      /* Output control registers */

    /* Compute registers for each duty cycle */
    for (uint8_t i=0; i < len; i++)
    {
        duty_to_registers((uint8_t *) &reg[i*4], *(duty_cycles+i));
    }

    /* Write to registers corresponding to a patricular sequence of pins */
    status = dev->write(dev->dev_id, PCA9685_LED0_ON_L + 4*pin, (uint8_t *) reg, 4*len);
    if (status != PCA9685_OK)
    {
        return PCA9685_E_COMM_FAIL;
    }
    return status;
}		/* -----  end of function pca9685_setPins  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  pca9685_setPrescaler
 *  Description:  Set PWM frequency
 * =====================================================================================
 */
int8_t pca9685_setPrescaler(struct pca9685_dev *dev, uint16_t PWM_frequency)
{
    int8_t status = PCA9685_OK;

    /* Compute prescale value */
    uint8_t prescale_value = (uint8_t) ((float) 25E6 / (float) (4096 * PWM_frequency));
    prescale_value--;

    uint8_t mode1_old = 0;

    /* Read contents of mode1 register */
    status = dev->read(dev->dev_id, PCA9685_PRE_SCALE, (uint8_t *) &mode1_old, 1);
    if (status != PCA9685_OK)
    {
        return PCA9685_E_COMM_FAIL;
    }

    uint8_t mode1_new = mode1_old | PCA9685_MODE1_SLEEP; /* set the sleep bit */

    /* Write to mode1 register with sleep bit set */
    status = dev->write(dev->dev_id, PCA9685_PRE_SCALE, (uint8_t *) &mode1_new, 1);
    if (status != PCA9685_OK)
    {
        return PCA9685_E_COMM_FAIL;
    }

    /* Write the prescale value to the prescaler register */
    status = dev->write(dev->dev_id, PCA9685_PRE_SCALE, (uint8_t *) &prescale_value, 1);
    if (status != PCA9685_OK)
    {
        return PCA9685_E_COMM_FAIL;
    }

    /* Restore contents of mode1 register (possibly waking up)*/
    status = dev->write(dev->dev_id, PCA9685_PRE_SCALE, (uint8_t *) &mode1_old, 1);
    if (status != PCA9685_OK)
    {
        return PCA9685_E_COMM_FAIL;
    }

    return status;
}		/* -----  end of function pca9685_setPrescaler  ----- */
