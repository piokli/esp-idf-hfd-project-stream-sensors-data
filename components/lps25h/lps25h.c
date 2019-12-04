/*
 * lps25h.c
 *
 *  Created on: 2 gru 2019
 *      Author: Piokli
 */

#include "lps25h.h"
#include <driver/i2c.h>
#include "../i2c_helper/i2c_helper.h"


esp_err_t lps25h_test_connection(void)	// initialize or rather check
{
	uint8_t sensor_id;

	int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LPS25H_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, LPS25H_WHO_AM_I_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LPS25H_I2C_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &sensor_id, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

	if (sensor_id == LPS25H_WHO_ID)
	{
		printf("LPS25H: im ok\n");
		return ret; //ESP_OK;
	}
	return ESP_ERR_INVALID_RESPONSE;
}

esp_err_t lps25h_read_press()
{
	//press = 0;
	int ret;

	uint8_t is_ready;
	uint8_t p_h, p_l, p_xl;
	uint32_t pressure;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LPS25H_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, LPS25H_STATUS_REG_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        //return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LPS25H_I2C_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &is_ready, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if(is_ready & LPS25H_PRESS_DATA_AVAIABLE)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (LPS25H_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, LPS25H_PRESS_OUT_H_ADDR, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) {
            //return ret;
        }
        vTaskDelay(30 / portTICK_RATE_MS);
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (LPS25H_I2C_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, &p_h, ACK_VAL);
        i2c_master_read_byte(cmd, &p_l, ACK_VAL);
        i2c_master_read_byte(cmd, &p_xl, NACK_VAL);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);

        pressure = ((p_h << 16) | (p_l << 8) | p_xl);
        printf("Pressure: %f\n", pressure / 4096.0);
        //printf("Pressure: %lu\n", (unsigned long)pressure);
    }
    else
    {
    	printf("*not ready*\n");
    }

    return ret;
}

esp_err_t lps25h_complete_setup(void)
{
	int16_t data;
	uint8_t settings = 0;

	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LPS25H_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, LPS25H_CTRL_REG1_ADDR, ACK_CHECK_EN);
    settings = LPS25H_POWER_UP | LPS25H_DATA_OUTPUT_RATE_12_5_HZ;
    i2c_master_write_byte(cmd, settings, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        //return ret;
    }
    // this part just for debug
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LPS25H_I2C_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    printf("CTRL_REG1: %X\n", data);
    return ret;
}
