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
 *  Typedefs
 *-----------------------------------------------------------------------------*/
typedef struct MotorShield_dev MotorShield_dev;
/*-----------------------------------------------------------------------------
 *  Function prototypes
 *-----------------------------------------------------------------------------*/
int8_t MS_Init(MotorShield_dev *MS);
int8_t MS_DC_drive(MotorShield_dev *MS, uint8_t motor, uint8_t mode, uint16_t duty_cycle);

/*-----------------------------------------------------------------------------
 *  TB6612 (driver) mode defines
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
 *  Motor shield structure
 *-----------------------------------------------------------------------------*/
struct MotorShield_dev
{
    struct pca9685_dev *pca9685;
    uint8_t driver1_mode;
    uint8_t driver2_mode;
};

#endif
