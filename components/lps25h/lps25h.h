/*
 * lps25h.h
 *
 *  Created on: 2 gru 2019
 *      Author: Piokli
 */

#ifndef COMPONENTS_LPS25H_LPS25H_H_
#define COMPONENTS_LPS25H_LPS25H_H_

#include <driver/i2c.h>
#include <esp_err.h>
#include "../i2c_helper/i2c_helper.h"

#define LPS25H_ADDR 0x5C				// LSM6DS33 address
#define LPS25H_WHO_ID 0xBD				// sensor's ID for checking if connection is correct

static enum LPS25H_regAddr
{
	REF_P_XL          = 0x08,
	REF_P_L           = 0x09,
	REF_P_H           = 0x0A,

	WHO_AM_I          = 0x0F,
	RES_CONF          = 0x10,

	CTRL_REG1         = 0x20,
	CTRL_REG2         = 0x21,
	CTRL_REG3         = 0x22,
	CTRL_REG4         = 0x23,
	INT_CFG           = 0x24,
	INT_SOURCE        = 0x25,

	STATUS_REG        = 0x27,
	PRESS_POUT_XL     = 0x28,           // may be a typo -> PRESS_OUT_XL
	PRESS_OUT_L       = 0x29,
	PRESS_OUT_H       = 0x2A,
	TEMP_OUT_L        = 0x2B,
	TEMP_OUT_H        = 0x2C,

	FIFO_CTRL         = 0x2E,
	FIFO_STATUS       = 0x2F,
	THIS_P_L          = 0x30,
	THIS_P_H          = 0x31,

	RPDS_L            = 0x39,
	RPDS_H            = 0x3A,
};

#endif /* COMPONENTS_LPS25H_LPS25H_H_ */
