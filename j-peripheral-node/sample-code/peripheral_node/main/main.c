/* udp_perf Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/




/*
udp_perf example

Using this example to test udp throughput performance.
esp<->esp or esp<->ap

step1:
    init wifi as AP/STA using config SSID/PASSWORD.

step2:
    create a udp server/client socket using config PORT/(IP).
    if server: wating for the first message of client.
    if client: sending a packet to server first.

step3:
    send/receive data to/from each other.
    you can see the info in serial output.
*/


#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "udp_perf.h"
#include "bme280.h"
#include <esp_mqtt.h>


//this task establish a UDP connection and receive data from UDP
static void udp_conn(void *pvParameters)
{
    ESP_LOGI(TAG, "task udp_conn start.");
    /*wating for connecting to AP*/
    xEventGroupWaitBits(udp_event_group, WIFI_CONNECTED_BIT,false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "sta has connected to ap.");
    
    /*create udp socket*/
    int socket_ret;
    ESP_LOGI(TAG, "create udp client after 5s...");
    vTaskDelay(5000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "create_udp_client.");
    socket_ret = create_udp_client();

    if(socket_ret == ESP_FAIL) {
	ESP_LOGI(TAG, "create udp socket error,stop.");
	vTaskDelete(NULL);
    }
    
    /*create a task to tx/rx data*/
    TaskHandle_t tx_task,rx_task,mqtt_task;
    xTaskCreate(&send_data, "send_data", 4096, NULL, 4, &tx_task);
    xTaskCreate(&recv_data, "recv_data", 4096, NULL, 6, &rx_task);
    xTaskCreate(&mqtt_publish_sensor_data, "mqtt_publish_sensor_data", 4096, NULL, 7, &mqtt_task);

    /*waiting udp connected success*/
    xEventGroupWaitBits(udp_event_group, UDP_CONNCETED_SUCCESS,false, true, portMAX_DELAY);
    int bps;
    while (1) {
	total_data = 0;
	vTaskDelay(3000 / portTICK_RATE_MS);//every 3s
	bps = total_data / 3;

	if (total_data <= 0) {
	    int err_ret = check_connected_socket();
	    if (err_ret == -1) {  //-1 reason: low level netif error
		ESP_LOGW(TAG, "udp send & recv stop.\n");
		break;
	    }
	}


	ESP_LOGI(TAG, "udp send %d byte per sec! total pack: %d \n", bps, success_pack);

    }
    close_socket();
    vTaskDelete(tx_task);
    vTaskDelete(rx_task);
    vTaskDelete(mqtt_task);
    vTaskDelete(NULL);
}

static void
mqtt_status_cb(esp_mqtt_status_t status)
{
    switch (status)
    {
    case ESP_MQTT_STATUS_CONNECTED:

        esp_mqtt_subscribe("hello", 0);
        break;
    case ESP_MQTT_STATUS_DISCONNECTED:
        break;
    }
}

/* ************************************************************************* *
 * The MQTT Message callback function. When a message is received, print a
 * message containing the topic, payload, and the length of the payload to the
 * terminal.
 * *************************************************************************/
static void
mqtt_message_cb(const char *topic, uint8_t *payload, size_t len)
{
    printf("incoming\t%s:%s (%d)\n", topic, payload, (int)len);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );


    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    configure_i2s();
    i2c_example_master_init();
    esp_mqtt_init(mqtt_status_cb, mqtt_message_cb, 256, 2000);
    //i2c_master_sensor_config(I2C_EXAMPLE_MASTER_NUM);
    xTaskCreate(&udp_conn, "udp_conn", 4096, NULL, 5, NULL);
}
