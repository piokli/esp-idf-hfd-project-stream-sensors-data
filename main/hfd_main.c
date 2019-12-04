/* Hfd main source
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../components/i2c_helper/i2c_helper.h"
#include "../components/lps25h/lps25h.h"
#include "../components/lsm6ds33/lsm6ds33.h"


void app_main()
{
	i2c_master_init();
	lps25h_complete_setup();
	vTaskDelay(100 / portTICK_PERIOD_MS);

	while(1)
	{
		lps25h_read_press();
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	/*
	while(1)
	{
		LSM6DS33_test_connection();
		LPS25H_test_connection();
	    printf("Hello world!\n");
	    vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	*/
}
