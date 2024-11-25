/* Hfd main source
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <math.h>
#include "nvs_flash.h"
#include "driver/gpio.h"
// #include "driver/adc.h"

#include "../components/i2c_helper/i2c_helper.h"
#include "../components/lps25h/lps25h.h"
#include "../components/lsm6ds33/lsm6ds33.h"
#include "../components/wifi_station/wifi_station.h"
#include "../components/tcp_client_helper/tcp_client_helper.h"

QueueHandle_t xQueue;
static const char *TAG_main = "main";


//////////////////////////////////////////////

#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0

static int Button_Pressed = 0;

static char tag[] = "test_button_intr";
static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
	ESP_LOGI(tag, "Button intr task started!");

    gpio_config_t io_conf;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_LOW_LEVEL;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    uint32_t last_tick = xTaskGetTickCount();
    uint32_t io_num;
    uint32_t cnt = 0;
    //const char *msg_buff = "0, 0, 0, 0, 0, 0";

    while(1) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
        	//ESP_LOGD(tag, "Interrupt recieved at: GPIO[%d]", io_num);
        	uint32_t current_tick = xTaskGetTickCount();

        	if (current_tick - last_tick > 10) {
        		ESP_LOGI(tag, "GPIO[%lu] intr, bttn pressed %lu time", io_num, cnt);

        		/*
        		if (!xQueueSendToBack(xQueue, &msg_buff, 0)) {
        			ESP_LOGE(tag, "Couldnt send to queue");
        		} else {
        			ESP_LOGW(tag, "QEUEEEE GUTT");
        		}
        		*/

        		Button_Pressed = 1;
        		vTaskDelay(100 / portTICK_PERIOD_MS);
        		Button_Pressed = 0;
        		vTaskDelay(20 / portTICK_PERIOD_MS);

        		cnt++;
        	}
        	last_tick = current_tick;
        }
    }
	vTaskDelete(NULL);
}

/////////////////////////////////////////////////////////////


void send_sensors_data_task(void *pvParameters)
{
	ESP_LOGI(TAG_main, "Hello");

	struct vector acc_data;
	struct vector gyro_data;
	float acc_mag;
	float gyro_mag;
	uint32_t press_data;
	char *string_buff;

	vTaskDelay(1000 / portTICK_PERIOD_MS); //dla zasady

	while(1)
	{
		ESP_LOGI(TAG_main, "Aj am in da loop!");

		lsm6ds33_read_acc_raw(&acc_data);
		lsm6ds33_read_gyro_raw(&gyro_data);
		lps25h_read_press_raw(&press_data);

		lsm6ds33_vector_calculate_acc_raw(&acc_data);
		lsm6ds33_vector_calculate_gyro_raw(&gyro_data);

		acc_mag = lsm6ds33_vector_magnitude_of(acc_data);
		gyro_mag = lsm6ds33_vector_magnitude_of(gyro_data);

		//tutaj u�ywam snprintfa i wysy�am stringa do xQueue
		string_buff = (char*)malloc(128);// * sizeof(char));
		//a co je�eli nie odbior� danej z kolejki i nie zrobie free wtedy?? @TODO naprawic to trzeba bedzie

		snprintf(string_buff, 128, "%lu, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %lu\n", esp_log_timestamp()  /* esp_log_timestamp() tick / portTICK_PERIOD_MS*/, acc_mag, acc_data.x, acc_data.y, acc_data.z, gyro_mag, gyro_data.x, gyro_data.y, gyro_data.z, (unsigned long)press_data);
		printf("sent: %s", string_buff);

		printf("%d, %d, %d\n", sizeof(*string_buff), strlen(string_buff), sizeof(&string_buff));

		xQueueSendToBack(xQueue, &string_buff, 1000);//portMAX_DELAY);
		vTaskDelay(pdMS_TO_TICKS(20)); // vTaskDelay(pdMS_TO_TICKS(20));  == vTaskDelay(20 / portTICK_PERIOD_MS);

		if (Button_Pressed == 1) {
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}

		//free(string_buff);
	}
	vTaskDelete(NULL);
}

void test(void *pvParameters)
{
	size_t memo;
	while(1)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
		memo = xPortGetFreeHeapSize();
		printf("Free memory: %d\n", memo);
	}
	vTaskDelete(NULL);
}

void blinky(void *pvParameters)
{
	const int blink_gpio = 5;
	esp_rom_gpio_pad_select_gpio(blink_gpio);
	gpio_set_direction(blink_gpio, GPIO_MODE_OUTPUT);

	while(1)
	{
	    /* Blink off (output low) */;
	    gpio_set_level(blink_gpio, 0);
	    vTaskDelay(1000 / portTICK_PERIOD_MS);
	    /* Blink on (output high) */
	    gpio_set_level(blink_gpio, 1);
	    vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}


// void check_battery(void *pvParameters)
// {
// 	//gpio_set_direction(35, GPIO_MODE_INPUT);
// 	float vbat;
// 	gpio_num_t gpio_num;

// 	adc1_config_width(ADC_WIDTH_BIT_12);
// 	adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11); //max 3.9V
// 	while(1)
// 	{
// 		int val = adc1_get_raw(ADC1_CHANNEL_7);
// 		adc1_pad_get_io_num(ADC1_CHANNEL_7, &gpio_num);

// 		//vbat = gpio_get_level(35);
// 		vbat = (3.9 * val / 4096) * 2;
// 		ESP_LOGI(TAG_main, "Battery: %f V, at GPIO%d\n", vbat, gpio_num);
// 		vTaskDelay(100 / portTICK_PERIOD_MS);
// 		//printf("Battery: %d V\n", vbat);
// 	}
// 	vTaskDelete(NULL);
// }


void app_main()
{
	//esp_log_level_set(tag, ESP_LOG_DEBUG);

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
	lps25h_test_connection();
	lsm6ds33_test_connection();

	// Set up sensors
	lsm6ds33_default_setup();
	lps25h_default_setup();

	// TASKS
	xQueue = xQueueCreate(10, sizeof(char*));
	if(xQueue != NULL)
	{
		xTaskCreate(blinky, "blinky", 1024, NULL, 0, NULL); // "I'm alive!!!"
		// xTaskCreate(check_battery, "check_battery", 2048, NULL, 0, NULL);
	    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 1, NULL);
		xTaskCreate(send_sensors_data_task, "send_sensors_data", 4096, NULL, 2, NULL); //sending data through xQueue to tcp_client_task
		xTaskCreate(tcp_client_task, "tcp_client", 4096, (void*)xQueue, 3, NULL);
	}
}
