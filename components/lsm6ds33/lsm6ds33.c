/*
 * lsm6ds33.c
 *
 *  Created on: 30 lis 2019
 *      Author: Piokli
 */

#include "lsm6ds33.h"

#include <math.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include "esp_log.h"

static const char* TAG = "lsm6ds33";

esp_err_t lsm6ds33_test_connection(void)
{
	uint8_t get_id;

	esp_err_t ret = i2c_helper_read_reg(LSM6DS33_I2C_ADDR, LSM6DS33_WHO_AM_I_ADDR, &get_id, 1);
	if (ret != ESP_OK) {
        return ret;
    }
    if (get_id != LSM6DS33_WHO_ID)
    {
    	ESP_LOGW(TAG, "Failed to connect to LSM6DS33!");
    	return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Connected to LSM6DS33");

    return ESP_OK;
}

esp_err_t lsm6ds33_default_setup(void)
{
	esp_err_t ret;

	uint8_t ctrl1_xl_setup = LSM6DS33_ACC_DATA_RATE_104_HZ |
			                 LSM6DS33_ACC_FULL_SCALE_2_G |
							 LSM6DS33_ACC_AA_BANDWIDTH_200_HZ;

	uint8_t ctrl2_g_setup = LSM6DS33_GYRO_DATA_RATE_104_HZ |
			                LSM6DS33_GYRO_FULL_SCALE_250_DPS;

	// ctrl3_c_setup; ctrl4_c_setup; ctrl5_c_setup;
	// ctrl6_c_setup(acc hi-perf); ctr7_c_setup(gyro hi-perf);
	// ctrl8_xl_setup; ctr9_xl_setup(acc axis en); ctr10_c_setup(gyro axis en)
	// ^these are all *ctrl registers, using defaults for now
	// there are also plenty more of settings for internal functions and interrupts

	ret = i2c_helper_write_reg(LSM6DS33_I2C_ADDR, LSM6DS33_CTRL1_XL_ADDR, &ctrl1_xl_setup, 1);
	if (ret != ESP_OK) {
	    return ret;
	}
	ret = i2c_helper_write_reg(LSM6DS33_I2C_ADDR, LSM6DS33_CTRL2_G_ADDR, &ctrl2_g_setup, 1);

	return ret;
}

esp_err_t lsm6ds33_read_acc_raw(struct vector *a)
{
	uint8_t buff_size = 2;
	uint8_t *ax = malloc(sizeof(uint8_t) * buff_size);
	uint8_t *ay = malloc(sizeof(uint8_t) * buff_size);
	uint8_t *az = malloc(sizeof(uint8_t) * buff_size);

	esp_err_t ret = i2c_helper_read_reg(LSM6DS33_I2C_ADDR, LSM6DS33_OUTX_L_XL_ADDR, ax, buff_size);
    if (ret != ESP_OK) {
        return ret;
    }
	ret = i2c_helper_read_reg(LSM6DS33_I2C_ADDR, LSM6DS33_OUTY_L_XL_ADDR, ay, buff_size);
    if (ret != ESP_OK) {
        return ret;
    }
	ret = i2c_helper_read_reg(LSM6DS33_I2C_ADDR, LSM6DS33_OUTZ_L_XL_ADDR, az, buff_size);

    a->x = (int16_t)((ax[1] << 8) | ax[0]);
    a->y = (int16_t)((ay[1] << 8) | ay[0]);
    a->z = (int16_t)((az[1] << 8) | az[0]);

    free(ax);
    free(ay);
    free(az);

    return ret;
}

esp_err_t lsm6ds33_read_gyro_raw(struct vector *g)
{
	uint8_t buff_size = 2;
	//uint8_t gx[buff_size]; //error: passing incompatible pointer type
	//uint8_t gy[buff_size];
	//uint8_t gz[buff_size];
	uint8_t *gx = malloc(sizeof(uint8_t) * buff_size);
	uint8_t *gy = malloc(sizeof(uint8_t) * buff_size);
	uint8_t *gz = malloc(sizeof(uint8_t) * buff_size);

	esp_err_t ret = i2c_helper_read_reg(LSM6DS33_I2C_ADDR, LSM6DS33_OUTX_L_G_ADDR, gx, buff_size);
	i2c_helper_read_reg(LSM6DS33_I2C_ADDR, LSM6DS33_OUTY_L_G_ADDR, gy, buff_size);
	i2c_helper_read_reg(LSM6DS33_I2C_ADDR, LSM6DS33_OUTZ_L_G_ADDR, gz, buff_size);

    g->x = (int16_t)((gx[1] << 8) | gx[0]);
    g->y = (int16_t)((gy[1] << 8) | gy[0]);
    g->z = (int16_t)((gz[1] << 8) | gz[0]);

    free(gx);
    free(gy);
    free(gz);

    return ret;
}

void lsm6ds33_vector_normalise(struct vector *a)
{
	float mag = sqrt((a->x * a->x) + (a->y * a->y) + (a->z * a->z));
	a->x /= mag;
	a->y /= mag;
	a->z /= mag;
}

float lsm6ds33_vector_magnitude_of(struct vector v)
{
	float mag = sqrt((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
	return mag;
}
