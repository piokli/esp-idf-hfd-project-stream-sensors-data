/*
 * i2c_helper.h
 *
 *  Created on: 2 gru 2019
 *      Author: Piokli
 */

#ifndef COMPONENTS_I2C_HELPER_I2C_HELPER_H_
#define COMPONENTS_I2C_HELPER_I2C_HELPER_H_

#include <esp_err.h>
#include <driver/i2c.h>

#define I2C_MASTER_NUM 1 				// as it was default in I2C's example sdkconfig.h; it is master's port number - maybe it's because there can be more
#define I2C_MASTER_SDA_IO 23			// gpio number for i2c slave data
#define I2C_MASTER_SCL_IO 22			// gpio number for i2c slave clock
#define I2C_MASTER_FREQ_HZ 100000		// default as in sdkconfig.h of I2C's example
#define I2C_MASTER_RX_BUF_DISABLE 0		// I2C master doesn't need buffer
#define I2C_MASTER_TX_BUF_DISABLE 0 	// I2C master doesn't need buffer

#define ACK_CHECK_EN 0x1				// I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0				// I2C master will not check ack from slave
#define ACK_VAL 0x0                     // I2C ack value
#define NACK_VAL 0x1                    // I2C nack value

#define WRITE_BIT I2C_MASTER_WRITE      // I2C master write
#define READ_BIT I2C_MASTER_READ        // I2C master read

esp_err_t i2c_master_init(void);

#endif /* COMPONENTS_I2C_HELPER_I2C_HELPER_H_ */
