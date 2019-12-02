/*
 * lsm6ds33_lps25h_i2c.c
 *
 *  Created on: 30 lis 2019
 *      Author: Piokli
 */

#include <math.h>
#include <driver/i2c.h>
#include "lsm6ds33_lps25h_i2c.h"

esp_err_t LSM6DS33_read_acc(struct vector *a)
{
	int ret;

	uint8_t ax_l;
	uint8_t ax_h;
	uint8_t ay_l;
	uint8_t ay_h;
	uint8_t az_l;
	uint8_t az_h;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DS33_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, OUTX_L_XL, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DS33_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &ax_l, ACK_VAL);
    i2c_master_read_byte(cmd, &ax_h, ACK_VAL);
    i2c_master_read_byte(cmd, &ay_l, ACK_VAL);
    i2c_master_read_byte(cmd, &ay_h, ACK_VAL);
    i2c_master_read_byte(cmd, &az_l, ACK_VAL);
    i2c_master_read_byte(cmd, &az_h, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    a->x = (int16_t)((ax_h << 8) | ax_l);
    a->y = (int16_t)((ay_h << 8) | ay_l);
    a->z = (int16_t)((az_h << 8) | az_l);

    return ret;
}

/**
 * Reads and writes x, y, z gyroscope values to gyro vector
 */
esp_err_t LSM6DS33_read_gyro(struct vector *g)
{
	int ret;

	uint8_t gx_l;
	uint8_t gx_h;
	uint8_t gy_l;
	uint8_t gy_h;
	uint8_t gz_l;
	uint8_t gz_h;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DS33_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, OUTX_L_G, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DS33_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &gx_l, ACK_VAL);
    i2c_master_read_byte(cmd, &gx_h, ACK_VAL);
    i2c_master_read_byte(cmd, &gy_l, ACK_VAL);
    i2c_master_read_byte(cmd, &gy_h, ACK_VAL);
    i2c_master_read_byte(cmd, &gz_l, ACK_VAL);
    i2c_master_read_byte(cmd, &gz_h, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    g->x = (int16_t)((gx_h << 8) | gx_l);
    g->y = (int16_t)((gy_h << 8) | gy_l);
    g->z = (int16_t)((gz_h << 8) | gz_l);

    return ret;
}

float vector_magnitude(struct vector *v)
{
	float mag = sqrt((v->x * v->x) + (v->y * v->y) + (v->z * v->z));
	return mag;
}

void vector_normalize(struct vector *a)
{
	float mag = sqrt((a->x * a->x) + (a->y * a->y) + (a->z * a->z));
	a->x /= mag;
	a->y /= mag;
	a->z /= mag;
}

esp_err_t write_reg(enum LSM6DS33_regAddr reg, uint8_t value)
{
	int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DS33_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

uint8_t read_reg(enum LSM6DS33_regAddr reg, uint8_t *data_h, uint8_t *data_l)
{
	int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DS33_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DS33_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_l, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t LSM6DS33_test_connection(void)	// initialize or rather check
{
	uint8_t sensor_data_h, sensor_data_l;
	read_reg(WHO_AM_I, &sensor_data_h, &sensor_data_l);
	if (sensor_data_h == LSM6DS33_WHO_ID)
	{
		return ESP_OK;
	}
	return ESP_ERR_INVALID_RESPONSE;
}

esp_err_t LSM6DS33_default_setup(void)	// sensor setup and read loop
{
    int ret;
    vTaskDelay(30 / portTICK_RATE_MS);													// LSM6 boots up for about 20 miliseconds?
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();										// start cmd link to queue commands
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LSM6DS33_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    //i2c_master_write_byte(cmd, CTRL9_XL, ACK_CHECK_EN);									// Acc X, Y, Z axes enabled-- what? doesnt work?
    //i2c_master_write_byte(cmd, 0x38, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, CTRL1_XL, ACK_CHECK_EN);									// Acc = 416 Hz (High performance mode)
    i2c_master_write_byte(cmd, 0x60, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, CTRL3_C, ACK_CHECK_EN);									// auto increment register
    i2c_master_write_byte(cmd, 0x04, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
