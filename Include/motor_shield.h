/*
 * =====================================================================================
 *
 *       Filename:  motor_shield.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  25/09/19 18:30:12
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Fedor Chervyakov (), fchervyakov@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef __MOTOR_SHIELD_H_
#define __MOTOR_SHIELD_H_

/*-----------------------------------------------------------------------------
 *  Header includes
 *-----------------------------------------------------------------------------*/
#include "pca9685.h"

/*-----------------------------------------------------------------------------
 *  Function prototypes
 *-----------------------------------------------------------------------------*/
struct MotorShield_dev MS_struct_Init(struct pca9685_dev *pca9685, uint8_t driver1_mode, uint8_t driver2_mode);
int8_t MS_Init(struct MotorShield_dev *MS);
int8_t MS_DC_drive(struct MotorShield_dev *MS, uint8_t motor, uint8_t mode, uint16_t duty_cycle);

/*-----------------------------------------------------------------------------
 *  TB6612 mode defines
 *-----------------------------------------------------------------------------*/
#define TB6612_STEPPER_MODE  ((uint8_t) 1)
#define TB6612_2DC_MODE      ((uint8_t) 0)

/*-----------------------------------------------------------------------------
 *  DC Motor defines
 *-----------------------------------------------------------------------------*/
#define MS_DC_MOTOR_1        ((uint8_t) 0)
#define MS_DC_MOTOR_2        ((uint8_t) 1)
#define MS_DC_MOTOR_3        ((uint8_t) 2)
#define MS_DC_MOTOR_4        ((uint8_t) 3)

/*-----------------------------------------------------------------------------
 *  H-bridge mode defines
 *-----------------------------------------------------------------------------*/
#define MS_CW                ((uint8_t) 0)
#define MS_CCW               ((uint8_t) 1)
#define MS_SHORT_BRAKE       ((uint8_t) 2)
#define MS_STOP              ((uint8_t) 3)

/*-----------------------------------------------------------------------------
 *  Motor Shield error codes
 *-----------------------------------------------------------------------------*/
#define MS_OK                ((int8_t)  0)
#define MS_E_INIT_FAIL       ((int8_t) -1)
#define MS_E_INVALID_MOTOR   ((int8_t) -2)
#define MS_E_DRIVE_FAIL      ((int8_t) -3)


/*-----------------------------------------------------------------------------
 *  Typedefs
 *-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
 *  Motor shield structure
 *-----------------------------------------------------------------------------*/
struct MotorShield_dev
{
    struct pca9685_dev *pca9685;
    struct tb6612_dev *tb6612_1;
    struct tb6612_dev *tb6612_2;
};

/*-----------------------------------------------------------------------------
 *  Structure representing a single TB6612 device
 *-----------------------------------------------------------------------------*/
struct tb6612_dev
{
    uint8_t mode;                       /* Mode: single stepper or two DC motors */
    struct H_bridge *H_A;                     /* H-bridge A  */
    struct H_bridge *H_B;                     /* H-bridge B  */
};

/*-----------------------------------------------------------------------------
 *  Structure representing a single H-bridge of TB6612
 *-----------------------------------------------------------------------------*/
struct H_bridge
{
    uint8_t PWM_pin;
    uint8_t IN1_pin;
    uint8_t IN2_pin;
};

#endif
