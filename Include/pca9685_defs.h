/*
 * =====================================================================================
 *
 *       Filename:  PCA9685_defs.h
 *
 *    Description:  Defines for the PCA9685 i2c led controller
 *                  onboard an Adafruit Motor Shield v2
 *
 *        Version:  1.0
 *        Created:  24/09/19 20:37:06
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Fedor Chervyakov, fchervyakov@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef __PCA9685_DEFS_H_
#define __PCA9685_DEFS_H__


/*-----------------------------------------------------------------------------
 *  Header includes
 *-----------------------------------------------------------------------------*/
 
#include <stdint.h>
#include <stddef.h>

/*-----------------------------------------------------------------------------
 *  Common macros
 *-----------------------------------------------------------------------------*/
#define CONCAT_BYTES(msb, lsb)  (((uint16_t) msb << 8) | (uint16_t) lsb)

/*-----------------------------------------------------------------------------
 *  I2C Addresses
 *-----------------------------------------------------------------------------*/
/* Addresses active on startup */
#define PCA9685_I2C_LED_ALL_CALL_ADDRESS   ((uint8_t) 0xE0)
#define PCA9685_I2C_SOFTWARE_RESET_ADDRESS ((uint8_t) 0x06) /*register address*/
#define PCA9564_I2C_SLAVE_ADDRESS          ((uint8_t) 0x00)
#define PCA9665_I2C_SLAVE_ADDRESS          ((uint8_t) 0xE0)

/* General Call I2C address which is used as Software Reset address */
#define PCA9685_I2C_GENERALL_CALL_ADDRESS  ((uint8_t) 0x00)

/* Slave address select bits */
#define PCA9685_A0 (0)
#define PCA9685_A1 (0 << 1)
#define PCA9685_A2 (0 << 2)
#define PCA9685_A3 (0 << 3)
#define PCA9685_A4 (0 << 4)
#define PCA9685_A5 (1 << 5)

/* 8-bit Slave address */
#define PCA9685_I2C_SLAVE_ADDRESS ((uint8_t)((1 << 6) & PCA9685_A5 & PCA9685_A4\
                     & PCA9685_A3 & PCA9685_A2 & PCA9685_A1 & PCA9685_A0) << 1)

/*-----------------------------------------------------------------------------
 *  Registers
 *-----------------------------------------------------------------------------*/

#define PCA9685_MODE1       ((uint8_t) 0x00)
#define PCA9685_MODE2       ((uint8_t) 0x01)
#define PCA9685_SUBADR1     ((uint8_t) 0x02)
#define PCA9685_SUBADR2     ((uint8_t) 0x03)
#define PCA9685_SUBADR3     ((uint8_t) 0x04)
#define PCA9685_ALLCALLADR  ((uint8_t) 0x05)

/* output and brightness control registers for each LED  */
#define LED_CONTROL_BASE    ((uint8_t) 0x06)
#define PCA9685_LED0_ON_L   ((uint8_t) (LED_CONTROL_BASE + 0x0))
#define PCA9685_LED0_ON_H   ((uint8_t) (LED_CONTROL_BASE + 0x1))
#define PCA9685_LED0_OFF_L  ((uint8_t) (LED_CONTROL_BASE + 0x2))
#define PCA9685_LED0_OFF_H  ((uint8_t) (LED_CONTROL_BASE + 0x3))
#define PCA9685_LED1_ON_L   ((uint8_t) (LED_CONTROL_BASE + 0x4))
#define PCA9685_LED1_ON_H   ((uint8_t) (LED_CONTROL_BASE + 0x5))
#define PCA9685_LED1_OFF_L  ((uint8_t) (LED_CONTROL_BASE + 0x6))
#define PCA9685_LED1_OFF_H  ((uint8_t) (LED_CONTROL_BASE + 0x7))
#define PCA9685_LED2_ON_L   ((uint8_t) (LED_CONTROL_BASE + 0x8))
#define PCA9685_LED2_ON_H   ((uint8_t) (LED_CONTROL_BASE + 0x9))
#define PCA9685_LED2_OFF_L  ((uint8_t) (LED_CONTROL_BASE + 0xa))
#define PCA9685_LED2_OFF_H  ((uint8_t) (LED_CONTROL_BASE + 0xb))
#define PCA9685_LED3_ON_L   ((uint8_t) (LED_CONTROL_BASE + 0xc))
#define PCA9685_LED3_ON_H   ((uint8_t) (LED_CONTROL_BASE + 0xd))
#define PCA9685_LED3_OFF_L  ((uint8_t) (LED_CONTROL_BASE + 0xe))
#define PCA9685_LED3_OFF_H  ((uint8_t) (LED_CONTROL_BASE + 0xf))
#define PCA9685_LED4_ON_L   ((uint8_t) (LED_CONTROL_BASE + 0x10))
#define PCA9685_LED4_ON_H   ((uint8_t) (LED_CONTROL_BASE + 0x11))
#define PCA9685_LED4_OFF_L  ((uint8_t) (LED_CONTROL_BASE + 0x12))
#define PCA9685_LED4_OFF_H  ((uint8_t) (LED_CONTROL_BASE + 0x13))
#define PCA9685_LED5_ON_L   ((uint8_t) (LED_CONTROL_BASE + 0x14))
#define PCA9685_LED5_ON_H   ((uint8_t) (LED_CONTROL_BASE + 0x15))
#define PCA9685_LED5_OFF_L  ((uint8_t) (LED_CONTROL_BASE + 0x16))
#define PCA9685_LED5_OFF_H  ((uint8_t) (LED_CONTROL_BASE + 0x17))
#define PCA9685_LED6_ON_L   ((uint8_t) (LED_CONTROL_BASE + 0x18))
#define PCA9685_LED6_ON_H   ((uint8_t) (LED_CONTROL_BASE + 0x19))
#define PCA9685_LED6_OFF_L  ((uint8_t) (LED_CONTROL_BASE + 0x1a))
#define PCA9685_LED6_OFF_H  ((uint8_t) (LED_CONTROL_BASE + 0x1b))
#define PCA9685_LED7_ON_L   ((uint8_t) (LED_CONTROL_BASE + 0x1c))
#define PCA9685_LED7_ON_H   ((uint8_t) (LED_CONTROL_BASE + 0x1d))
#define PCA9685_LED7_OFF_L  ((uint8_t) (LED_CONTROL_BASE + 0x1e))
#define PCA9685_LED7_OFF_H  ((uint8_t) (LED_CONTROL_BASE + 0x1f))
#define PCA9685_LED8_ON_L   ((uint8_t) (LED_CONTROL_BASE + 0x20))
#define PCA9685_LED8_ON_H   ((uint8_t) (LED_CONTROL_BASE + 0x21))
#define PCA9685_LED8_OFF_L  ((uint8_t) (LED_CONTROL_BASE + 0x22))
#define PCA9685_LED8_OFF_H  ((uint8_t) (LED_CONTROL_BASE + 0x23))
#define PCA9685_LED9_ON_L   ((uint8_t) (LED_CONTROL_BASE + 0x24))
#define PCA9685_LED9_ON_H   ((uint8_t) (LED_CONTROL_BASE + 0x25))
#define PCA9685_LED9_OFF_L  ((uint8_t) (LED_CONTROL_BASE + 0x26))
#define PCA9685_LED9_OFF_H  ((uint8_t) (LED_CONTROL_BASE + 0x27))
#define PCA9685_LED10_ON_L  ((uint8_t) (LED_CONTROL_BASE + 0x28))
#define PCA9685_LED10_ON_H  ((uint8_t) (LED_CONTROL_BASE + 0x29))
#define PCA9685_LED10_OFF_L ((uint8_t) (LED_CONTROL_BASE + 0x2a))
#define PCA9685_LED10_OFF_H ((uint8_t) (LED_CONTROL_BASE + 0x2b))
#define PCA9685_LED11_ON_L  ((uint8_t) (LED_CONTROL_BASE + 0x2c))
#define PCA9685_LED11_ON_H  ((uint8_t) (LED_CONTROL_BASE + 0x2d))
#define PCA9685_LED11_OFF_L ((uint8_t) (LED_CONTROL_BASE + 0x2e))
#define PCA9685_LED11_OFF_H ((uint8_t) (LED_CONTROL_BASE + 0x2f))
#define PCA9685_LED12_ON_L  ((uint8_t) (LED_CONTROL_BASE + 0x30))
#define PCA9685_LED12_ON_H  ((uint8_t) (LED_CONTROL_BASE + 0x31))
#define PCA9685_LED12_OFF_L ((uint8_t) (LED_CONTROL_BASE + 0x32))
#define PCA9685_LED12_OFF_H ((uint8_t) (LED_CONTROL_BASE + 0x33))
#define PCA9685_LED13_ON_L  ((uint8_t) (LED_CONTROL_BASE + 0x34))
#define PCA9685_LED13_ON_H  ((uint8_t) (LED_CONTROL_BASE + 0x35))
#define PCA9685_LED13_OFF_L ((uint8_t) (LED_CONTROL_BASE + 0x36))
#define PCA9685_LED13_OFF_H ((uint8_t) (LED_CONTROL_BASE + 0x37))
#define PCA9685_LED14_ON_L  ((uint8_t) (LED_CONTROL_BASE + 0x38))
#define PCA9685_LED14_ON_H  ((uint8_t) (LED_CONTROL_BASE + 0x39))
#define PCA9685_LED14_OFF_L ((uint8_t) (LED_CONTROL_BASE + 0x3a))
#define PCA9685_LED14_OFF_H ((uint8_t) (LED_CONTROL_BASE + 0x3b))
#define PCA9685_LED15_ON_L  ((uint8_t) (LED_CONTROL_BASE + 0x3c))
#define PCA9685_LED15_ON_H  ((uint8_t) (LED_CONTROL_BASE + 0x3d))
#define PCA9685_LED15_OFF_L ((uint8_t) (LED_CONTROL_BASE + 0x3e))
#define PCA9685_LED15_OFF_H ((uint8_t) (LED_CONTROL_BASE + 0x3f))

/* output and brightness control registers for ALL LEDs */
#define PCA9685_ALL_LED_ON_L  ((uint8_t) 0xFA)
#define PCA9685_ALL_LED_ON_H  ((uint8_t) 0xFB)
#define PCA9685_ALL_LED_OFF_L ((uint8_t) 0xFC)
#define PCA9685_ALL_LED_OFF_H ((uint8_t) 0xFD)

#define PCA9685_PRE_SCALE     ((uint8_t) 0xFE)
#define PCA9685_TestMode      ((uint8_t) 0xFF)

/*-----------------------------------------------------------------------------
 *  MODE1 register bits
 *-----------------------------------------------------------------------------*/
#define PCA9685_MODE1_RESTART ((uint8_t) 1 << 7)
#define PCA9685_MODE1_EXTCLK  ((uint8_t) 1 << 6)
#define PCA9685_MODE1_AI      ((uint8_t) 1 << 5)
#define PCA9685_MODE1_SLEEP   ((uint8_t) 1 << 4)
#define PCA9685_MODE1_SUB1    ((uint8_t) 1 << 3)
#define PCA9685_MODE1_SUB2    ((uint8_t) 1 << 2)
#define PCA9685_MODE1_SUB3    ((uint8_t) 1 << 1)
#define PCA9685_MODE1_ALLCALL ((uint8_t) 1)

/*-----------------------------------------------------------------------------
 *  MODE2 register bits
 *-----------------------------------------------------------------------------*/
#define PCA9685_MODE2_INVRT   ((uint8_t) 1 << 4)
#define PCA9685_MODE2_OCH     ((uint8_t) 1 << 3)
#define PCA9685_MODE2_OUTDRV  ((uint8_t) 1 << 2)
#define PCA9685_MODE2_OUTNE_1 ((uint8_t) 1 << 1)
#define PCA9685_MODE2_OUTNE_0 ((uint8_t) 1)

/*-----------------------------------------------------------------------------
 *  LED driver output pins
 *-----------------------------------------------------------------------------*/
#define PCA9685_LED0  ((uint8_t) 0)
#define PCA9685_LED1  ((uint8_t) 1)
#define PCA9685_LED2  ((uint8_t) 2)
#define PCA9685_LED3  ((uint8_t) 3)
#define PCA9685_LED4  ((uint8_t) 4)
#define PCA9685_LED5  ((uint8_t) 5)
#define PCA9685_LED6  ((uint8_t) 6)
#define PCA9685_LED7  ((uint8_t) 7)
#define PCA9685_LED8  ((uint8_t) 8)
#define PCA9685_LED9  ((uint8_t) 9)
#define PCA9685_LED10 ((uint8_t) 10)
#define PCA9685_LED11 ((uint8_t) 11)
#define PCA9685_LED12 ((uint8_t) 12)
#define PCA9685_LED13 ((uint8_t) 13)
#define PCA9685_LED14 ((uint8_t) 14)
#define PCA9685_LED15 ((uint8_t) 15)

/*-----------------------------------------------------------------------------
 *  Error codes
 *-----------------------------------------------------------------------------*/
#define PCA9685_OK          ((int8_t)  0)
#define PCA9685_E_COMM_FAIL ((int8_t) -1)
#define PCA9685_E_INIT_FAIL ((int8_t) -2)

/*-----------------------------------------------------------------------------
 *  Type definitions
 *-----------------------------------------------------------------------------*/
// Function pointer typedefs
typedef int8_t (*pca9685_i2c_com_fptr_t) \
        (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
typedef void (*pca9685_delay_fptr_t) (uint16_t delay_ms);

/*-----------------------------------------------------------------------------
 *  PCA9685 device structure
 *-----------------------------------------------------------------------------*/
typedef struct pca9685_dev
{
    uint8_t dev_id;                             /* Device address */
    pca9685_i2c_com_fptr_t read;                /* I2C read function pointer */
    pca9685_i2c_com_fptr_t write;               /* I2C write function pointer */
    pca9685_delay_fptr_t delay;                 /* Delay function pointer */
    uint16_t frequency;                         /* PWM output frequency */
};
#endif
