/*
 * tcp_client_helper.c
 *
 *  Created on: 9 gru 2019
 *      Author: Piokli
 */

#include "tcp_client_helper.h"

static const char *TAG = "tcp client";

void tcp_client_task(void *pvParameters)
{
	//float mag;
	//uint32_t tick;

	//char *text_to_send;
	//text_to_send = (char*)pvParameters;

	QueueHandle_t getQueue = pvParameters;
	char* tx_buffer;

    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {
    	ESP_LOGI(TAG, "Gonna be waiting for established wi-fi");
    	xEventGroupWaitBits(s_wifi_event_group, BIT0, 1, 0, 15000 / portTICK_RATE_MS);
    	ESP_LOGI(TAG, "wifi is established");

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", HOST_IP_ADDR, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Successfully connected");

        while (1) {
        	printf("got it\n");
        	xQueueReceive(getQueue, &tx_buffer, 1000);
        	//printf("Received mag: %.2f\n", mag);
        	//printf("hejkaaaa\n");
        	//tick = xTaskGetTickCount();
        	//sprintf(tx_buffer, "%u, %f\n", tick, mag); // na t¹ chwilê oszukany czas - powinienem przekazywac strukture z danymi wszystkimi
        	//albo sam wskaznik do sformatowanego juz stringa z danymi
        	//no³p, struktura z danymi ok ale nie wysy³am wskaŸnik do niej tylko stringa ju¿ gotowego (ewentualnie wskaŸnik do stringa, szybsze i lepsze)

        	// below i am trying to send my data and not that "const payload"
            int err = send(sock, tx_buffer, strlen(tx_buffer), 0); //text_to_send, strlen(text_to_send), 0);//payload, strlen(payload), 0);                                // send data
            free(tx_buffer);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            //vTaskDelay(10 / portTICK_PERIOD_MS);
            //free(tx_buffer); //it is automaticly freed


            //when i disable recv and if else then tcp stops sending after a while and program crashes of memory leaking
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
            }



            //vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

