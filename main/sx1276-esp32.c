#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <ctype.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"

// #include "esp_event.h"
#include "esp_flash.h"
#include "esp_log.h"
//#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "i2c_interface.h"
#include "sht31_driver.h"
#include "lora.h"

static const char *TAG = "LoRa sensor";

#define TIMEOUT 100

float temperature;
float humi;

void SHT31_Task(void *pvParameter)
{
    uint8_t unit = 0;    
    uint16_t temp;

    SHT31_Init();

    ESP_ERROR_CHECK_WITHOUT_ABORT(SHT31_DisableHeater());

	while(1)
	{

        ESP_ERROR_CHECK_WITHOUT_ABORT(SHT31_ReadTempHumi(&temp, &humi, unit));
        temperature = temp / 10.0;
        ESP_LOGI(TAG, "temp before = %.1f, humi = %.2f\n", temperature, humi);
        vTaskDelay(2000 / portTICK_PERIOD_MS); 
        
    }
}

void task_primary(void *pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start");
	uint8_t buf[256]; // Maximum Payload size of SX1276/77/78/79 is 255
	while(1) {
		TickType_t nowTick = xTaskGetTickCount();
		int send_len = sprintf((char *)buf, "Hello World!! %" PRIu32 " Temperature: %.1f", nowTick, temperature);

		lora_send_packet(buf, send_len);
		ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent:[%.*s]", send_len, send_len, buf);

		bool waiting = true;
		TickType_t startTick = xTaskGetTickCount();
		while(waiting) {
			lora_receive(); // put into receive mode
			if(lora_received()) {
				int rxLen = lora_receive_packet(buf, sizeof(buf));
                int rssi = lora_packet_rssi();
                int snr = lora_packet_snr();
				TickType_t currentTick = xTaskGetTickCount();
				TickType_t diffTick = currentTick - startTick;
				ESP_LOGI(pcTaskGetName(NULL), "%d byte packet received:[%.*s] with RSSI: %d   SNR: %d", rxLen, rxLen, buf, rssi, snr);
				ESP_LOGI(pcTaskGetName(NULL), "Response time is %"PRIu32" MillSecs", diffTick * portTICK_PERIOD_MS);
				waiting = false;
			}
			TickType_t currentTick = xTaskGetTickCount();
			TickType_t diffTick = currentTick - startTick;
			ESP_LOGD(pcTaskGetName(NULL), "diffTick=%"PRIu32, diffTick);
			if (diffTick > TIMEOUT) {
				ESP_LOGW(pcTaskGetName(NULL), "Response timeout");
				waiting = false;
			}
			vTaskDelay(1); // Avoid WatchDog alerts
		} // end waiting
		vTaskDelay(pdMS_TO_TICKS(5000));
	} // end while
}

static esp_err_t esp_storage_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}
void app_main(void)
{
    esp_storage_init();

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_init(I2C_NUM_0));
    SHT31_Init();
    xTaskCreate(&SHT31_Task, "sht31_task", 2048, NULL, 5, NULL);

    if (lora_init() == 0) {
		ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
		while(1) {
			vTaskDelay(1);
		}
	}

	ESP_LOGI(pcTaskGetName(NULL), "Frequency is 915MHz");
	lora_set_frequency(915e6); // 915MHz

    lora_enable_crc();

    int cr = 1;
	int bw = 7;
	int sf = 7;

    lora_set_coding_rate(cr);
	ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", cr);

	lora_set_bandwidth(bw);
	ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bw);

	lora_set_spreading_factor(sf);
	ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);

    xTaskCreate(&task_primary, "PRIMARY", 1024*3, NULL, 5, NULL);
}
