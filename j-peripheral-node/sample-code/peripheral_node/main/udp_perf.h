
/* udp_perf Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#ifndef __UDP_PERF_H__
#define __UDP_PERF_H__



#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "bme280.h"

/*test options*/
#define EXAMPLE_ESP_WIFI_MODE_AP CONFIG_UDP_PERF_WIFI_MODE_AP //TRUE:AP FALSE:STA
#define EXAMPLE_ESP_UDP_MODE_SERVER CONFIG_UDP_PERF_SERVER //TRUE:server FALSE:client
#define EXAMPLE_ESP_UDP_PERF_TX CONFIG_UDP_PERF_TX //TRUE:send FALSE:receive
#define EXAMPLE_PACK_BYTE_IS 97 //'a'
/*AP info and tcp_server info*/
#define EXAMPLE_DEFAULT_SSID CONFIG_UDP_PERF_WIFI_SSID
#define EXAMPLE_DEFAULT_PWD CONFIG_UDP_PERF_WIFI_PASSWORD
#define EXAMPLE_DEFAULT_RECV_PORT CONFIG_UDP_PERF_SERVER_RECV_PORT
#define EXAMPLE_DEFAULT_SEND_PORT CONFIG_UDP_PERF_SERVER_SEND_PORT
#define EXAMPLE_DEFAULT_PKTSIZE CONFIG_UDP_PERF_PKT_SIZE
#define EXAMPLE_MAX_STA_CONN 1 //how many sta can be connected(AP mode)

#ifdef CONFIG_UDP_PERF_SERVER_IP
#define EXAMPLE_DEFAULT_SERVER_IP CONFIG_UDP_PERF_SERVER_IP
#else
#define EXAMPLE_DEFAULT_SERVER_IP "192.168.4.1"
#endif /*CONFIG_UDP_PERF_SERVER_IP*/


#define I2S_NUM         (0)
#define SAMPLE_RATE     (36000)

#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_SCL_IO          19               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          18               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define BH280_SENSOR_ADDR                  0x77             /*!< slave address for BH1750 sensor */
#define BH280_CMD_START                    0xFA             /*!< Command to set measure mode */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

#define TAG "udp_perf:"

/* FreeRTOS event group to signal when we are connected to WiFi and ready to start UDP test*/
extern EventGroupHandle_t udp_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define UDP_CONNCETED_SUCCESS BIT1

extern int total_data;
extern int success_pack;


//using esp as station
void wifi_init_sta();


//create a udp client socket. return ESP_OK:success ESP_FAIL:error
esp_err_t create_udp_client();

//send data task
void send_data(void *pvParameters);

//recv data task
void recv_data(void *pvParameters);

//task for publishing sensor data
void mqtt_publish_sensor_data(void *pvParameters);

//get socket error code. return: error code
int get_socket_error_code(int socket);

//show socket error code. return: error code
int show_socket_error_reason(int socket);

//check connected socket. return: error code
int check_connected_socket();

//close all sockets
void close_socket();

//configure i2s
void configure_i2s();

void i2c_example_master_init();

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

void BME280_delay_msek(u32 msek);

esp_err_t i2c_master_sensor_config(i2c_port_t i2c_num);

#ifdef __cplusplus
}
#endif


#endif /*#ifndef __UDP_PERF_H__*/

