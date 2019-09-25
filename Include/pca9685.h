/*
 * =====================================================================================
 *
 *       Filename:  pca9685.h
 *
 *    Description:  Header file for the PCA9685 i2c led controller driver
 *
 *        Version:  1.0
 *        Created:  25/09/19 00:31:51
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Fedor Chervyakov (), fchervyakov@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef __PCA9685_H_
#define __PCA9685_H_

/*-----------------------------------------------------------------------------
 *  Header includes
 *-----------------------------------------------------------------------------*/
#include "pca9685_defs.h"

/*-----------------------------------------------------------------------------
 *  Function prototypes
 *-----------------------------------------------------------------------------*/
void duty_to_registers (uint8_t *reg, uint16_t duty_cycle);
int8_t pca9685_init(pca9685_dev *dev);
int8_t pca9685_reset(pca9685_dev *dev);
int8_t pca9685_setAll(pca9685_dev *dev, uint16_t duty_cycle);
int8_t pca9685_setPin(pca9685_dev *dev, uint8_t pin, uint16_t duty_cycle);
int8_t pca9685_setPrescaler(pca9685_dev *dev, uint16_t PWM_frequency);
#endif