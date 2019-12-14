/* Hfd main source
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <math.h>
#include "nvs_flash.h"

#include "../components/i2c_helper/i2c_helper.h"
#include "../components/lps25h/lps25h.h"
#include "../components/lsm6ds33/lsm6ds33.h"
#include "../components/wifi_station/wifi_station.h"
#include "../components/tcp_client_helper/tcp_client_helper.h"

QueueHandle_t xQueue;

void send_sensors_data_task(void *pvParameters)
{
	struct vector acc_data;
	float acc_mag;
	uint32_t tick;

	//char string_buff[128];
	char *string_buff;

	while(1)
	{
		tick = xTaskGetTickCount();
		lsm6ds33_read_acc_raw(&acc_data);
		acc_mag = lsm6ds33_vector_magnitude_of(acc_data);
		lsm6ds33_vector_normalise(&acc_data);
		//tutaj u¿ywam snprintfa i wysy³am stringa do xQueue
		string_buff = (char*)malloc(128);// * sizeof(char));
		//a co je¿eli nie odbiorê danej z kolejki i nie zrobie free wtedy?? @TODO naprawic to trzeba bedzie

		//printf("Sent acc_mag: %.2f, acc_x: %.2f, acc_y: %.2f, acc_z: %.2f\n", acc_mag, acc_data.x, acc_data.y, acc_data.z);
		snprintf(string_buff, 128, "%u, %.2f, %.2f, %.2f, %.2f\n", tick, acc_mag, acc_data.x, acc_data.y, acc_data.z);
		printf("sent: %s", string_buff);

		//xQueueSendToBack(xQueue, &acc_mag, 0); //send too queue
		xQueueSendToBack(xQueue, &string_buff, 0);

		vTaskDelay(20 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void send_sensors_data_receive_task(void *pvParameters)
{
	//char string_buff[128];
	char* string_buff;

	while(1)
	{
		//xQueueSendToBack(xQueue, &acc_mag, 0); //send too queue
		xQueueReceive(xQueue, &string_buff, 10000);
		printf("recv: %s",  string_buff);
		free(string_buff);

		//vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void test(void *pvParameters)
{
	size_t memo;
	while(1)
	{
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		memo = xPortGetFreeHeapSize();
		printf("Free memory: %d\n", memo);
	}
}

void app_main()
{
    //Initialize NVS (for wifi_init to use)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wifi, I2C and connect to sensors
    vTaskDelay(100 / portTICK_PERIOD_MS);
	wifi_init_sta();
	i2c_helper_master_init();
	lsm6ds33_test_connection();

	// Setup sensors
	lsm6ds33_default_setup();

	// TASKS
	xQueue = xQueueCreate(10, sizeof(char*));
	if(xQueue != NULL)
	{
		xTaskCreate(send_sensors_data_task, "send_sensors_data", 4096, NULL, 5, NULL);
		//xTaskCreate(send_sensors_data_receive_task, "send_sensors_data_receive", 4096, NULL, 5, NULL);
		xTaskCreate(tcp_client_task, "tcp_client", 4096, (void*)xQueue, 5, NULL);
		xTaskCreate(test, "test", 4096, (void*)xQueue, 0, NULL);
	}


	/*
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
	*/

}
