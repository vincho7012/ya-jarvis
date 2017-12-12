#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s.h"

#include "lwip/sockets.h"

#define I2C_MASTER_SCL_IO    19    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    18    /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */

#define WM8731_I2C_ADDR  0x1A    /*!< slave address for WM8731 sensor */
#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define WM8731_LINVOL   0x00
#define WM8731_RINVOL   0x01
#define WM8731_LOUT1V   0x02
#define WM8731_ROUT1V   0x03
#define WM8731_APANA    0x04
#define WM8731_APDIGI   0x05
#define WM8731_PWR      0x06
#define WM8731_IFACE    0x07
#define WM8731_SRATE    0x08
#define WM8731_ACTIVE   0x09
#define WM8731_RESET	0x0f

#define WM8731_SAMPLING_FREQ_8K		8000u
#define WM8731_SAMPLING_FREQ_32K	32000u
#define WM8731_SAMPLING_FREQ_44K1	44100u
#define WM8731_SAMPLING_FREQ_48K	48000u
#define WM8731_SAMPLING_FREQ_88K2	88200u
#define WM8731_SAMPLING_FREQ_96K	96000u

#define WM8731_MCLK 12000000u //6144000u//12000000u


//#define CODEC_IS_MASTER


#define SAMPLE_RATE     (36000)
#define I2S_NUM         (0)
#define WAVE_FREQ_HZ    (100)
#define PI 3.14159265

#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)


typedef struct wm8731_cmd_entry_s {
	unsigned int reg_addr;
	unsigned int reg_val;
} wm8731_cmd_entry_t;

typedef struct _sampling_ctrl {
         long mclk;
         long rate;
         unsigned int fs;
         unsigned char sr:4;
         unsigned char bosr:1;
         unsigned char usb:1;
 } wm8731_sampling_ctrl_t;

wm8731_sampling_ctrl_t sampling_ctrl_tbl[] = {
	/* 48k */
	{12288000, 48000, 256, 0x0, 0x0, 0x0},
	{18432000, 48000, 384, 0x0, 0x1, 0x0},
	{12000000, 48000, 250, 0x0, 0x0, 0x1},
    {6144000,  48000, 256, 0x7, 0x0, 0x0},

	/* 32k */
	{12288000, 32000, 384, 0x6, 0x0, 0x0},
	{18432000, 32000, 576, 0x6, 0x1, 0x0},
	{12000000, 32000, 375, 0x6, 0x0, 0x1},

	/* 8k */
	{12288000, 8000, 1536, 0x3, 0x0, 0x0},
	{18432000, 8000, 2304, 0x3, 0x1, 0x0},
	{11289600, 8000, 1408, 0xb, 0x0, 0x0},
	{16934400, 8000, 2112, 0xb, 0x1, 0x0},
	{12000000, 8000, 1500, 0x3, 0x0, 0x1},

	/* 96k */
	{12288000, 96000, 128, 0x7, 0x0, 0x0},
	{18432000, 96000, 192, 0x7, 0x1, 0x0},
	{12000000, 96000, 125, 0x7, 0x0, 0x1},

	/* 44.1k */
	{11289600, 44100, 256, 0x8, 0x0, 0x0},
	{16934400, 44100, 384, 0x8, 0x1, 0x0},
	{12000000, 44100, 272, 0x8, 0x1, 0x1},

	/* 88.2k */
	{11289600, 88200, 128, 0xf, 0x0, 0x0},
	{16934400, 88200, 192, 0xf, 0x1, 0x0},
	{12000000, 88200, 136, 0xf, 0x1, 0x1},
};

wm8731_cmd_entry_t wm8731_cmd_entries[] = {
    { WM8731_PWR,		0x80 },
	{ WM8731_RESET, 	0x00 },
	{ WM8731_ACTIVE, 	0x00 },
	{ WM8731_APANA, 	0x12 },	// disable boost, disable mute, disable bypass, select DAC, disable side tone, -6dB attn.
	{ WM8731_APDIGI, 	0x00 },
	{ WM8731_PWR, 		0x00 }, //0x60 },	//Microphone Input an Bias Power Down 1 = Enable Power Down
	{ WM8731_IFACE, 	0x02 },//0x02 },  // I2S mode, 16 bits, Slave Mode.
	//i2c_write(WM8731_SRATE, ((0 << 7) | (0 << 6) | (sampling_ctrl_tbl[pwm8731_handle->index_samp].sr << 2) |
	//						 (sampling_ctrl_tbl[pwm8731_handle->index_samp].bosr << 1) |
	//						 (sampling_ctrl_tbl[pwm8731_handle->index_samp].usb)));
	{ WM8731_LINVOL, 	0x17 },
	{ WM8731_RINVOL, 	0x17 },
	{ WM8731_LOUT1V, 	0x7F },
	{ WM8731_ROUT1V, 	0x7F },
	{ WM8731_ACTIVE, 	0x01 },
};

//VIV
unsigned int sample_val_arr[SAMPLE_PER_CYCLE] = { 0 };

/**
 * @brief i2c master initialization
 */
void i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t wm8731_config(i2c_port_t i2c_num, wm8731_cmd_entry_t* cmd_entries, unsigned int entries_count, unsigned int relaxation_time)
{
	i2c_cmd_handle_t cmd;
    int ret;

#ifdef CODEC_IS_MASTER
	int index_samp = -1;
    unsigned long mclk = WM8731_MCLK;
    unsigned long sampling_freq = WM8731_SAMPLING_FREQ_32K;

    // Find the index in the table containing correct settings
    while(!(sampling_ctrl_tbl[++index_samp].rate == sampling_freq &&
    		  sampling_ctrl_tbl[index_samp].mclk == mclk));

    printf(">>> %d entries, %d index_samp\n", entries_count, index_samp);
#endif // CODEC_IS_MASTER

    for (unsigned int i = 0; i < entries_count; i++)
    {
    	unsigned char byte1 = (cmd_entries[i].reg_addr << 1) | (cmd_entries[i].reg_val >> 8);
    	unsigned char byte2 = cmd_entries[i].reg_val & 0xff;

    	cmd = i2c_cmd_link_create();
   	    i2c_master_start(cmd);
    	i2c_master_write_byte(cmd, WM8731_I2C_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    	i2c_master_write_byte(cmd, byte1, ACK_CHECK_EN);
    	i2c_master_write_byte(cmd, byte2, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_FAIL) {
            return ESP_FAIL;
        }
        vTaskDelay(relaxation_time / portTICK_RATE_MS);
    }

#ifdef CODEC_IS_MASTER
	cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, WM8731_I2C_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, (WM8731_SRATE << 1), ACK_CHECK_EN);
	i2c_master_write_byte(
        cmd,
        ((0 << 7) | (0 << 6) | (sampling_ctrl_tbl[index_samp].sr << 2) |
        (sampling_ctrl_tbl[index_samp].bosr << 1) |
        (sampling_ctrl_tbl[index_samp].usb)), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return ESP_FAIL;
    }
    vTaskDelay(relaxation_time / portTICK_RATE_MS);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, WM8731_I2C_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, (WM8731_ACTIVE << 1), ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x01, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret == ESP_FAIL) {
		return ESP_FAIL;
	}
	vTaskDelay(relaxation_time / portTICK_RATE_MS);
#endif //CODEC_IS_MASTER

    return ESP_OK;
}





struct media_bus_t
{
	struct
	{
		TaskHandle_t* thread_id;
		int running;
	} sender;

	struct
	{
		TaskHandle_t* thread_id;
		int running;
	} receiver;

	int sock;
	struct sockaddr_in addr;
} media_bus;

struct mbus_cfg_t
{
	char* my_ip_addr;
	unsigned short my_listening_port;
	char* sendto_ip_addr;
	unsigned short sendto_port;
} mbus_cfg = {
	.my_ip_addr = "192.168.1.102",
	.my_listening_port = 37773,
	.sendto_ip_addr = "192.168.1.101",
	.sendto_port = 27772,
};

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

static const char *TAG = "example";

/* CA cert, taken from wpa2_ca.pem
   Client cert, taken from wpa2_client.crt
   Client key, taken from wpa2_client.key

   The PEM, CRT and KEY file were provided by the person or organization
   who configured the AP with wpa2 enterprise.

   To embed it in the app binary, the PEM, CRT and KEY file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
*/

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "d2racing",
            .password = "cisco123",
            .bssid_set = false
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void wifi_task(void *pvParameters)
{
    tcpip_adapter_ip_info_t ip;
    memset(&ip, 0, sizeof(tcpip_adapter_ip_info_t));
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    while (1) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        if (tcpip_adapter_get_ip_info(ESP_IF_WIFI_STA, &ip) == 0) {
            ESP_LOGI(TAG, "~~~~~~~~~~~");
            ESP_LOGI(TAG, "IP:"IPSTR, IP2STR(&ip.ip));
            ESP_LOGI(TAG, "MASK:"IPSTR, IP2STR(&ip.netmask));
            ESP_LOGI(TAG, "GW:"IPSTR, IP2STR(&ip.gw));
            ESP_LOGI(TAG, "~~~~~~~~~~~");
        }
    }
}


static void mbus_snd_task(void *p)
{
    int i = 0;
    struct sockaddr_in toAddr;
    struct timeval usertimeout;
    int ret = 0;

    LWIP_UNUSED_ARG(p);

    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

    memset(&toAddr, 0, sizeof(struct sockaddr_in));
    toAddr.sin_family = AF_INET;
    //VIV OLD toAddr.sin_addr.s_addr = inet_addr("192.168.1.101");
    inet_aton(mbus_cfg.sendto_ip_addr, &toAddr.sin_addr);
    //toAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    toAddr.sin_port = htons(mbus_cfg.sendto_port);
    toAddr.sin_len = sizeof(struct sockaddr_in);
    while (1) {
        usertimeout.tv_sec = 1;
        usertimeout.tv_usec = 0;
/*******************************start means the start of a frame ********************************************/
        ret = sendto(media_bus.sock, sample_val_arr, sizeof(sample_val_arr), 0, (struct sockaddr*)&toAddr, sizeof(toAddr));
        if (ret < 0) {
            perror("send failed reason");
            //ESP_LOGI(TAG, "send failed reason %d", ret);
        }
        ESP_LOGI(TAG, "send success\n")
        vTaskDelay(5000 / portTICK_RATE_MS);
        //ets_delay_us(100000);
#if 0
/*******************************start contents of a frame ********************************************/
        for(i=0; i < LINES; i++) {
            FD_ZERO(&rdfds);
            FD_CLR(sock, &rdfds);
            FD_SET(sock, &rdfds);
            ret = select(sock+1, NULL, &rdfds, NULL, &usertimeout);
            if (ret > 0) {

                if (FD_ISSET(sock, &rdfds)) {

                    ret = sendto(sock, image[i], SEND_BUF_LEN, 0, (struct sockaddr*)&toAddr, sizeof(toAddr));
                    if (ret < 0) {
                        perror("send failed reason");
                        /*
                        ets_delay_us(500);
                        ret=sendto(sock,image[i],SEND_BUF_LEN,0,(struct sockaddr*)&toAddr,sizeof(toAddr));
                        */
                    }
                }
            }
        }
/*******************************end means the end of a frame ********************************************/
        sendto(sock, end, SEND_END_LEN, 0, (struct sockaddr*)&toAddr, sizeof(toAddr));
#endif
    }
}

static void mbus_rcv_task(void *p)
{
    int i = 0;
    struct sockaddr_in fromAddr;
    //VIV struct sockaddr_storage fromAddr;
	int size;
	int bufsize = 2000; //VIV
	unsigned char buf[bufsize+1];
    int ret = 0;

    unsigned int fromAddrLen = sizeof(fromAddr);

    LWIP_UNUSED_ARG(p);

    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

    ESP_LOGI(TAG, "mbus_rcv_task\n")

//VIV
    memset(&fromAddr, 0, sizeof(struct sockaddr_in));
    //fromAddr.sin_family = AF_INET;
    ////VIV OLD toAddr.sin_addr.s_addr = inet_addr("192.168.1.101");
    //inet_aton(mbus_cfg.sendto_ip_addr, &fromAddr.sin_addr);
    //fromAddr.sin_port = htons(mbus_cfg.sendto_port);
    //fromAddr.sin_len = sizeof(struct sockaddr_in);

    while (1) {
        //lwip_recvfrom(int s, void *mem, size_t len, int flags,
        //              struct sockaddr *from, socklen_t *fromlen)

    	size = recvfrom(media_bus.sock, buf, bufsize, 0 /*MSG_DONTWAIT*/, (struct sockaddr*)&fromAddr, (socklen_t *)&fromAddrLen);

    	ESP_LOGI(TAG, "After recvfrom >>>>>>>\n")

		if (size < 0) {
			if ((size == -EAGAIN)) {
				// do nothing
			} else if (size != -EAGAIN) {
				ESP_LOGE(TAG, "Error on receiving - error = %d\n", size);
				perror("receive failed reason");
			}
		} else if (size > 0) {
			ESP_LOGI(TAG, "Received %d bytes from %d.%d.%d.%d @ %u\n",
					size,
					(fromAddr.sin_addr.s_addr&0xFF),
					((fromAddr.sin_addr.s_addr&0xFF00)>>8),
					((fromAddr.sin_addr.s_addr&0xFF0000)>>16),
					((fromAddr.sin_addr.s_addr&0xFF000000)>>24),
					ntohs(fromAddr.sin_port));
			// Print data
			//unsigned int i = 0;
			//for (i = 0; i < size; i++) {
			//	ESP_LOGI(TAG, "%x ", buf[i]);
			//}
		} else {
			// Ignore size = 0
		}

    	vTaskDelay(10 / portTICK_RATE_MS);
    }

//close_and_out:
//    close(sock);
//    vTaskDelete(NULL);
}

static int media_bus_init(struct mbus_cfg_t* cfg)
{
	int ret = 0;
	struct sockaddr_in myAddr;
	int reuse = 1;
	struct timeval tv;
	tv.tv_sec = 3;  /* 3 Seconds Time-out */
	tv.tv_usec = 0;

	memset(&media_bus, 0, sizeof(struct media_bus_t));

	ret = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(ret < 0) {
		ESP_LOGI(TAG, "socket err");
		perror("");
		goto exit;
	}
	media_bus.sock = ret;

	ESP_LOGI(TAG, "socket %d successfully created\n", media_bus.sock);

	/*set the socket options*/
	ret = setsockopt(media_bus.sock, SOL_SOCKET, SO_RCVTIMEO, (char *) &tv, sizeof(struct timeval));
	if(ret < 0) {
		ESP_LOGI(TAG, "setsockopt SO_RCVTIMEO err");
		perror("");
		goto exit;
	}
	ESP_LOGI(TAG, "setsockopt SO_RCVTIMEO success\n");

	ret = setsockopt(media_bus.sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
	if(ret < 0) {
		ESP_LOGI(TAG, "setsockopt SO_REUSEADDR err");
		perror("");
		goto exit;
	}
	ESP_LOGI(TAG, "setsockopt SO_REUSEADDR success\n");

	memset(&myAddr, 0, sizeof(struct sockaddr_in));
	myAddr.sin_family      = AF_INET;
	//VIV OLD myAddr.sin_addr.s_addr = inet_addr(INADDR_RECV); //VIV htonl(INADDR_RECV); //VIV htonl(INADDR_ANY);
	inet_aton(cfg->my_ip_addr, &myAddr.sin_addr);
	myAddr.sin_port        = htons(cfg->my_listening_port);
	myAddr.sin_len = sizeof(struct sockaddr_in);

	ret = bind(media_bus.sock, (struct sockaddr *)&myAddr, sizeof(myAddr));
	if (ret < 0)
	{
		ESP_LOGI(TAG, "could not bind or connect to socket, error = %d\n", ret);
		perror("");
		goto close_and_out;
	}

	ESP_LOGI(TAG, "listening on port %d, socket %u\n", cfg->my_listening_port, media_bus.sock);

	xTaskCreate(&mbus_rcv_task, "mbus_rcv_task", 4096, NULL, 5, NULL); //VIV media_bus.receiver.thread_id);
	xTaskCreate(&mbus_snd_task, "mbus_snd_task", 4096, NULL, 5, NULL); //VIV media_bus.sender.thread_id);



#if 0
	/* start kernel receiver thread */
	media_bus->receiver.thread_id = kthread_run((void *)ksocket_rcv_loop, (void*)cfg, MODULE_NAME);
	if (IS_ERR(media_bus->receiver.thread_id))
	{
		printk(KERN_INFO MODULE_NAME": Unable to start kernel thread\n");
		err = -ENOMEM;
		goto rcv_out;
	}

	printk(KERN_INFO MODULE_NAME": Thread %p successfully started\n", media_bus->receiver.thread_id);

	/* start kernel sender thread */
	media_bus->sender.thread_id = kthread_run((void *)ksocket_snd_loop, (void*)cfg, MODULE_NAME);
	if (IS_ERR(media_bus->sender.thread_id))
	{
		printk(KERN_INFO MODULE_NAME": Unable to start kernel thread\n");
		err = -ENOMEM;
		goto snd_out;
	}

	printk(KERN_INFO MODULE_NAME": Thread %p successfully started\n", media_bus->sender.thread_id);
	goto exit; // Graceful exit

snd_out:
	media_bus->sender.thread_id = NULL;
	media_bus->sender.running = 0;
	// Stop the receiver also
	kthread_stop(media_bus->receiver.thread_id);

rcv_out:
	media_bus->receiver.thread_id = NULL;
	media_bus->receiver.running = 0;

close_socket:
	sock_release(media_bus->sock);
	media_bus->sock = NULL;

	close_socket:
	kfree(media_bus);
	media_bus = NULL;

exit:
	return err;
#endif


close_and_out:
//snd_out:
//rcv_out:
//close_socket:
//cleanup:
exit:

	return 0;
}


void app_main(void)
{
    int ret;

    nvs_flash_init();

    //vTaskDelay(500 / portTICK_RATE_MS);

	i2c_master_init();

    //vTaskDelay(500 / portTICK_RATE_MS);

    ret = wm8731_config(I2C_MASTER_NUM, wm8731_cmd_entries, sizeof(wm8731_cmd_entries) / sizeof(wm8731_cmd_entry_t), 1);

    printf("*******************\n");
    printf("WM8731 initialization\n");
    printf("*******************\n");
    if (ret == ESP_OK) {
    	printf("Codec initialization success!\n");
    } else {
        printf("No ack, codec not connected...skip...\n");
    }

    vTaskDelay(500 / portTICK_RATE_MS);

    unsigned int i, sample_val;

    float sin_float, triangle_float, triangle_step = 65536.0 / SAMPLE_PER_CYCLE;
    //for 36Khz sample rates, we create 100Hz sine wave, every cycle need 36000/100 = 360 samples (4-bytes each sample)
    //using 6 buffers, we need 60-samples per buffer
    //2-channels, 16-bit each channel, total buffer is 360*4 = 1440 bytes
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX, //VIV I2S_MODE_MASTER | I2S_MODE_TX,                                  // Only TX
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = 16,                                                  //16-bit per channel
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           //2-channels
        .communication_format = I2S_COMM_FORMAT_I2S, //VIV | I2S_COMM_FORMAT_I2S_MSB,
        .dma_buf_count = 6,
        .dma_buf_len = 60,                                                      //
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1                                //Interrupt level 1
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = 23,
        .ws_io_num = 21,
        .data_out_num = 22,
        .data_in_num = -1                                                       //Not used
    };

    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
//VIV    i2s_set_pin(I2S_NUM, NULL);

    triangle_float = -32767;

    for(i = 0; i < SAMPLE_PER_CYCLE; i++) {
        sin_float = sin(i * PI / 180.0);
        if(sin_float >= 0)
            triangle_float += triangle_step;
        else
            triangle_float -= triangle_step;
        sin_float *= 32767;

        //VIV
        //sample_val = 0;
        //sample_val += (short)triangle_float;
        //sample_val = sample_val << 16;
        //sample_val += (short) sin_float;

        sample_val_arr[i] = 0;
        sample_val_arr[i] += (short)triangle_float;
        sample_val_arr[i] = sample_val_arr[i] << 16;
        sample_val_arr[i] += (short) sin_float;

        //printf("0x%x\n", sample_val);

        //VIV i2s_push_sample(I2S_NUM, (char *)&sample_val, portMAX_DELAY);
    }

    //VIV
    i2s_write_bytes(I2S_NUM, sample_val_arr, sizeof(sample_val_arr), portMAX_DELAY);

    initialise_wifi();
    xTaskCreate(&wifi_task, "wifi_task", 4096, NULL, 5, NULL);

    media_bus_init(&mbus_cfg);
}

