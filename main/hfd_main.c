/* Hfd main source
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../components/lsm6ds33_lps25h_i2c/lsm6ds33_lps25h_i2c.h"


void app_main()
{
	i2c_master_init();

	while(1)
	{
	    printf("Hello world!\n");
	    vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
