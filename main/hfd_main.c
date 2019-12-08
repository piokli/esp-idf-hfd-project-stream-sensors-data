/* Hfd main source
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <math.h>

#include "../components/i2c_helper/i2c_helper.h"
#include "../components/lps25h/lps25h.h"
#include "../components/lsm6ds33/lsm6ds33.h"

void app_main()
{
	i2c_helper_master_init();
	lsm6ds33_test_connection();
	lps25h_test_connection();

	lsm6ds33_default_setup();
	lps25h_default_setup();

	uint32_t pressure_data;
	struct vector acc_data;
	struct vector gyro_data;

	while(1)
	{
		lps25h_read_press_raw(&pressure_data);
		printf("\nPressure: %.2f\n", pressure_data / 4096.0);

		lsm6ds33_read_acc_raw(&acc_data);
		printf("Acc mag: %.2f\n", lsm6ds33_vector_magnitude_of(acc_data));
		lsm6ds33_vector_normalise(&acc_data);
		printf("acc.x = %.2f acc.y = %.2f acc.z = %.2f\n", acc_data.x, acc_data.y, acc_data.z);

		lsm6ds33_read_gyro_raw(&gyro_data);
		printf("Gyro mag: %.2f\n", lsm6ds33_vector_magnitude_of(gyro_data));
		lsm6ds33_vector_normalise(&gyro_data);
		printf("gyro.x = %.2f gyro.y = %.2f gyro.z = %.2f\n", gyro_data.x, gyro_data.y, gyro_data.z);

		vTaskDelay(100 / portTICK_PERIOD_MS);
	}

}
