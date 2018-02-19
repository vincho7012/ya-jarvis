/* udp_perf Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "driver/i2c.h"

#include "esp_system.h"
#include "nvs_flash.h"
//#include "freertos/semphr.h"
//#include "freertos/queue.h"

#include <esp_mqtt.h>
#include "udp_perf.h"
#include "bme280.h"



struct bme280_t bme280 = {
	.bus_write = BME280_I2C_bus_write,
	.bus_read = BME280_I2C_bus_read,
	.dev_addr = BME280_I2C_ADDRESS2,
	.delay_msec = BME280_delay_msek
};


/* FreeRTOS event group to signal when we are connected to WiFi and ready to start UDP test*/
EventGroupHandle_t udp_event_group;
const char *MQTT_TAG = "MQTT_SAMPLE";

static int mysocket,mysocket1;

static struct sockaddr_in remote_addr,remote_addr1;
static unsigned int socklen,socklen1;

int total_data = 0;
int success_pack = 0;


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
    	esp_mqtt_stop();
        esp_wifi_connect();
        xEventGroupClearBits(udp_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
    	ESP_LOGI(TAG, "event_handler:SYSTEM_EVENT_STA_GOT_IP!");
    	ESP_LOGI(TAG, "got ip:%s\n",
		ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
    	xEventGroupSetBits(udp_event_group, WIFI_CONNECTED_BIT);
    	esp_mqtt_start(MQTT_HOST, MQTT_PORT, "esp-mqtt", MQTT_USER, MQTT_PASS);
        break;
    default:
        break;
    }
    return ESP_OK;
}


//wifi_init_sta
void wifi_init_sta()
{
    udp_event_group = xEventGroupCreate();
    
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_DEFAULT_SSID,
            .password = EXAMPLE_DEFAULT_PWD
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s \n",
	    EXAMPLE_DEFAULT_SSID,EXAMPLE_DEFAULT_PWD);
}
//create a udp client socket. return ESP_OK:success ESP_FAIL:error
esp_err_t create_udp_client()
{
    ESP_LOGI(TAG, "create_udp_client()");
    ESP_LOGI(TAG, "sending to %s:%d",
	    EXAMPLE_DEFAULT_SERVER_IP, EXAMPLE_DEFAULT_SEND_PORT);
    ESP_LOGI(TAG, "receiving from %s:%d",
    	    EXAMPLE_DEFAULT_SERVER_IP, EXAMPLE_DEFAULT_RECV_PORT);
    mysocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (mysocket < 0) {
    	show_socket_error_reason(mysocket);
	return ESP_FAIL;
    }
    mysocket1 = socket(AF_INET, SOCK_DGRAM, 0);
    if (mysocket1 < 0) {
    	show_socket_error_reason(mysocket1);
	return ESP_FAIL;
    }
    /*for client remote_addr is also server_addr*/
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(EXAMPLE_DEFAULT_SEND_PORT);
    remote_addr.sin_addr.s_addr = inet_addr(EXAMPLE_DEFAULT_SERVER_IP);
    remote_addr1.sin_family = AF_INET;
    remote_addr1.sin_port = htons(EXAMPLE_DEFAULT_RECV_PORT);
    //remote_addr1.sin_addr.s_addr = inet_addr(EXAMPLE_DEFAULT_SERVER_IP);
    remote_addr1.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(mysocket1, (struct sockaddr *)&remote_addr1, sizeof(remote_addr1)) < 0) {
        	show_socket_error_reason(mysocket1);
    	close(mysocket1);
    	return ESP_FAIL;
        }

    return ESP_OK;
}


//send data task
void send_data(void *pvParameters)
{
    ESP_LOGI(TAG, "task send_data start!\n");
    int len;
    char databuff[EXAMPLE_DEFAULT_PKTSIZE];
    
    /*send&receive first packet*/
    socklen = sizeof(remote_addr);
    memset(databuff, EXAMPLE_PACK_BYTE_IS, EXAMPLE_DEFAULT_PKTSIZE);

    ESP_LOGI(TAG, "first sendto:");
    len = sendto(mysocket, databuff, EXAMPLE_DEFAULT_PKTSIZE, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));


    if (len > 0) {
	ESP_LOGI(TAG, "transfer data with %s:%u\n",
		inet_ntoa(remote_addr.sin_addr), ntohs(remote_addr.sin_port));
	xEventGroupSetBits(udp_event_group, UDP_CONNCETED_SUCCESS);
    } else {
    	show_socket_error_reason(mysocket);
	close(mysocket);
	vTaskDelete(NULL);
    } /*if (len > 0)*/
    
    vTaskDelay(500 / portTICK_RATE_MS);
    //i2c_master_sensor_config(I2C_EXAMPLE_MASTER_NUM);
    ESP_LOGI(TAG, "start count1!\n");
    while(1) {

	len = sendto(mysocket, databuff, EXAMPLE_DEFAULT_PKTSIZE, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));

	vTaskDelay(5000 / portTICK_RATE_MS);//every 5s

	//i2c_master_read_sensor( I2C_EXAMPLE_MASTER_NUM, sensor_regdata);

	if (len > 0) {
	    total_data += len;
	    success_pack++;
	} else {
	    if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG) {
		show_socket_error_reason(mysocket);
	    }
	} /*if (len > 0)*/
    } /*while(1)*/
}

//recv data task
void recv_data(void *pvParameters)
{
    ESP_LOGI(TAG, "task recv_data start!\n");

    int len;
    char databuff[EXAMPLE_DEFAULT_PKTSIZE];

    /*send&receive first packet*/
    socklen1 = sizeof(remote_addr1);
    memset(databuff, EXAMPLE_PACK_BYTE_IS, EXAMPLE_DEFAULT_PKTSIZE);




    ESP_LOGI(TAG, "start count!\n");
    while(1) {

	len = recvfrom(mysocket1, databuff, EXAMPLE_DEFAULT_PKTSIZE, 0, (struct sockaddr *)&remote_addr1, &socklen1);
	i2s_start(I2S_NUM);
	i2s_write_bytes(I2S_NUM, databuff, len, portMAX_DELAY);
	vTaskDelay(3000 / portTICK_RATE_MS);
	i2s_stop(I2S_NUM);
	ESP_LOGW(TAG, "%s udp packege received.\n",databuff);
	if (len > 0) {
	    total_data += len;
	    success_pack++;
	} else {
	    if (LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG) {
		show_socket_error_reason(mysocket);
	    }
	} /*if (len > 0)*/
    } /*while(1)*/
}
void mqtt_publish_sensor_data(void *pvParameters)
{
	i2c_master_sensor_config(I2C_EXAMPLE_MASTER_NUM);
	bme280_init(&bme280);
	s32 v_uncomp_pressure_s32;
	s32 v_uncomp_temperature_s32;
	s32 v_uncomp_humidity_s32;
	vTaskDelay(100 / portTICK_RATE_MS);
	while(1) {
		bme280_read_uncomp_pressure_temperature_humidity(
						&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

		ESP_LOGI(TAG,"%.2f degC / %.3f hPa / %.3f %%",
			bme280_compensate_temperature_double(v_uncomp_temperature_s32),
			bme280_compensate_pressure_double(v_uncomp_pressure_s32)/100, // Pa -> hPa
			bme280_compensate_humidity_double(v_uncomp_humidity_s32));
		char *payload = "hello world";
		esp_mqtt_publish("test", (void *)payload, (int)strlen(payload), 0, false);
		vTaskDelay(5000 / portTICK_RATE_MS);//every 5s
	}
}

int get_socket_error_code(int socket)
{
    int result;
    u32_t optlen = sizeof(int);
    if(getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen) == -1) {
	ESP_LOGE(TAG, "getsockopt failed");
	return -1;
    }
    return result;
}

int show_socket_error_reason(int socket)
{
    int err = get_socket_error_code(socket);
    ESP_LOGW(TAG, "socket error %d %s", err, strerror(err));
    return err;
}

int check_connected_socket()
{
    int ret;
    ESP_LOGD(TAG, "check connect_socket");
    ret = get_socket_error_code(mysocket);
    if(ret != 0) {
    	ESP_LOGW(TAG, "socket error %d %s", ret, strerror(ret));
    }
    return ret;
}

void close_socket()
{
    close(mysocket);
    close(mysocket1);
}

void configure_i2s()
{
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,                                  // Only TX
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = 16,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           //2-channels
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .dma_buf_count = 6,
        .dma_buf_len = 60,
        .use_apll = 0,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1                                //Interrupt level 1
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = 26,
        .ws_io_num = 25,
        .data_out_num = 22,
        .data_in_num = -1                                                       //Not used
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
}

void i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t i2c_master_sensor_config(i2c_port_t i2c_num)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH280_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xF4, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xB7, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_master_stop(cmd);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
    	ESP_LOGI(TAG, "error in i2c\n");
        return ret;
    }

    return ret;
}

s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;

	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = SUCCESS;
	} else {
		iError = FAIL;
	}
	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

	if (cnt > 1) {
		i2c_master_read(cmd, reg_data, cnt-1, ACK_VAL);
	}
	i2c_master_read_byte(cmd, reg_data+cnt-1, NACK_VAL);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = SUCCESS;
	} else {
		iError = FAIL;
	}

	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

void BME280_delay_msek(u32 msek)
{
	vTaskDelay(msek/portTICK_PERIOD_MS);
}


