#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "rom/ets_sys.h"
#include "rom/crc.h"
#include "driver/gpio.h"

#include "esp_node.h"
#include "sensor_service.h"

#define TAG "CLIENT"

TaskHandle_t ClientTaskHandle = NULL;

static void vTaskSendQuery(void* pvParameters);

void vTaskSendQuery(void* pvParameters)
{
    esp_sensor_t query_obj;
    ESPNowGenerateQuery((uint8_t*)&query_obj, READ_SENSOR_VALUE, HUMIDITY_SENSOR);
    uint8_t peerIndex = 0;
    node_mac_t peerMac = { 0 };

    while(1)
    {
        if(GetPeerCount() > 0)
        {
            GetPeerMAC(peerIndex, &peerMac);

            ESP_LOGD(TAG, "peer(%d) --- %x:%x:%x:%x:%x:%x\r\n", (uint32_t)peerIndex,
            peerMac.mac[0], peerMac.mac[1], peerMac.mac[2],
            peerMac.mac[3], peerMac.mac[4], peerMac.mac[5]);
            esp_now_send(peerMac.mac, (uint8_t*)&query_obj, ESPNOW_QUERY_LEN);
            peerIndex++;
            if(GetPeerCount() == peerIndex)
            {
                peerIndex = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000));        
    }
}

void InitESPNowClient()

{
    
    if( xTaskCreate(vTaskSendQuery, "Client", 2*MINIMUM_STACK,
    NULL, 3, &ClientTaskHandle) != pdPASS)
    {
        ESP_LOGE(TAG, "Client initialization failed");
    }
}