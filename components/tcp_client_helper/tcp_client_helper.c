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

    //char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {
        struct sockaddr_in dest_addr;
        inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

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
        	if (!xQueueReceive(getQueue, &tx_buffer, portMAX_DELAY)) {
        		ESP_LOGE(TAG, "QUEUE NOT OK");
                break;
        	} else {
        		ESP_LOGI(TAG, "QUEUE OK");
        	}

            int err = send(sock, tx_buffer, strlen(tx_buffer), 0); //MSG_DONTWAIT);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            free(tx_buffer);

            //vTaskDelay(10 / portTICK_PERIOD_MS);
            //free(tx_buffer); //it is automaticly freed... or is it?

/*
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
*/


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

