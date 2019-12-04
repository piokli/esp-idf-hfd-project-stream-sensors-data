/*
 * lsm6ds33.h
 *
 *  Created on: 30 lis 2019
 *      Author: Piokli
 */

#ifndef COMPONENTS_LSM6DS33_LSM6DS33_H_
#define COMPONENTS_LSM6DS33_LSM6DS33_H_

#include <driver/i2c.h>
#include <esp_err.h>
#include "../i2c_helper/i2c_helper.h"

#define LSM6DS33_ADDR 0x6B				// LSM6DS33 address
#define LSM6DS33_WHO_ID 0x69			// sensor's ID for checking if connection is correct


/**
 * Structure for better keeping the x, y and z values of accelerometer and gyroscope
 */
struct vector {
	float x;
	float y;
	float z;
};

/**
 *
 */
float vector_magnitude(struct vector *v);

/**
 *
 */
void vector_normalize(struct vector *a);

/**
 * Reads and writes x, y, z accelerometer values to acc vector
 *
 * @TODO should check status reg maybe in the future?
 */
esp_err_t LSM6DS33_read_acc(struct vector *a);

/**
 * Reads and writes x, y, z gyroscope values to gyro vector
 */
esp_err_t LSM6DS33_read_gyro(struct vector *g);


/**
 *
 */
esp_err_t LSM6DS33_test_connection(void);

/**
 * @TODO must analyse and add setup for gyro
 */
esp_err_t LSM6DS33_default_setup(void);


#endif /* COMPONENTS_LSM6DS33_LSM6DS33_H_ */
