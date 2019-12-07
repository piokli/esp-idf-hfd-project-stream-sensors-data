/* Hfd main source
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "../components/i2c_helper/i2c_helper.h"
#include "../components/lps25h/lps25h.h"
#include "../components/lsm6ds33/lsm6ds33.h"


void app_main()
{
	i2c_helper_master_init();
	lsm6ds33_test_connection();
	lps25h_test_connection();

	while(1)
	{
		printf("Works\n");
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}

}
