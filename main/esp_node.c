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

#define TAG "NODE"
#define RX_QUEUE_LEN 5

#define MINIMUM_STACK configMINIMAL_STACK_SIZE

static uint8_t  EspNowBuffer[ESPNOW_MAX_PAYLOAD_LEN] = { 0 };
static esp_now_peer_info_t PeerList[MAX_SENSOR_NODES] = { { { 0 } } };

const uint8_t BroadcastMAC[ESP_MAC_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

xQueueHandle ESPNowRxQueue;
TaskHandle_t RxProcessHandle = NULL;
TaskHandle_t AdvertiseHandle = NULL;
TaskHandle_t SensorTaskHandle = NULL;

static void ESPNowSendCallback(const uint8_t* mac_address, esp_now_send_status_t status);
static void ESPNowReceiveCallback(const uint8_t* mac_address, const uint8_t *data, int len);

static void vTaskAdvertise(void* pvParameter);
static void vTaskProcessRxdData(void* pvParameters);

static void vTaskSendSensorData(void* pvParameter);

void ESPAddPeer(rxd_obj_t rx_obj)
{
    esp_now_peer_info_t* peer = (esp_now_peer_info_t*)malloc(sizeof(esp_now_peer_info_t));
    uint8_t* buffr = (uint8_t*)&rx_obj.sensor_obj;

    if(peer != NULL)
    {
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        peer->channel = buffr[INDEX_ESPNOW_CHANNEL];
        peer->ifidx = buffr[INDEX_ESPNOW_IF_TYPE];
        peer->encrypt = true;
        memcpy(peer->peer_addr, rx_obj.src_mac, ESP_MAC_LEN);
        ESP_ERROR_CHECK(esp_now_add_peer(peer));
        free(peer);
    }
    else
    {
        ESP_LOGE(TAG, "ESPAddPeer - malloc failed");
    }
}

void InitESPBroadcast()
{
    esp_now_peer_info_t* peer = 
    (esp_now_peer_info_t*)malloc(sizeof(esp_now_peer_info_t));
    if(peer != NULL)
    {
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        peer->channel = ESPNOW_CHANNEL;
        peer->ifidx = ESPNOW_WIFI_IF;
        peer->encrypt = false;
        memcpy(peer->peer_addr, BroadcastMAC, ESP_MAC_LEN);
        ESP_ERROR_CHECK(esp_now_add_peer(peer));
        free(peer);
    }
    else
    {
        ESP_LOGE(TAG, "InitESPBroadcast - malloc failed");
    }
}

void InitESPNow()
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(ESPNowSendCallback));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(ESPNowReceiveCallback));
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)ESP_PMK));
    InitESPBroadcast();
    ESPNowRxQueue = xQueueCreate(RX_QUEUE_LEN, sizeof(rxd_obj_t));
    if(ESPNowRxQueue == NULL)
    {
        ESP_LOGE(TAG, "ESPNowRxQueue initialization failed");
    }

    if( xTaskCreate(vTaskProcessRxdData, "RxProcessor", MINIMUM_STACK,
    NULL, 3, &RxProcessHandle) != pdPASS)
    {
        ESP_LOGE(TAG, "RxProcessor initialization failed");
    }

    if( xTaskCreate(vTaskAdvertise, "AvertiseProcess", MINIMUM_STACK,
    NULL, 3, &AdvertiseHandle) != pdPASS)
    {
        ESP_LOGE(TAG, "Advertisement initialization failed");
    }
}

void ESPNow_Deinit()
{
    esp_now_deinit();
}

static void ESPNowReceiveCallback(const uint8_t* mac_address, const uint8_t *data, int len)
{
    if(mac_address == NULL || data == NULL)
    {
        ESP_LOGE(TAG, "ESPNowReceiveCallback memory error");
        return;
    }

    ESP_LOGD(TAG, "%d bytes of data rx'd", len);
    rxd_obj_t rxObj = { 0 };
    memcpy(&rxObj.sensor_obj, data, len);
    memcpy(rxObj.src_mac, mac_address, ESP_MAC_LEN);
    rxObj.data_len = len;
    
    //
    //  push data to queue
    //
    if(xQueueSend(ESPNowRxQueue, &rxObj, portMAX_DELAY) == pdTRUE)
    {
        ESP_LOGI(TAG, "Sent to queue");
    }
}

static void ESPNowSendCallback(const uint8_t* mac_address, esp_now_send_status_t status)
{
    if(status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(TAG, "ESP_NOW_SEND_SUCCESS");
    }
}

void PackSensorData(esp_sensor_t* sensor, uint8_t* buffer, uint32_t* len)
{
    if(sensor == NULL || buffer == NULL || len == NULL)
    {
        ESP_LOGE(TAG, "PackSensorData memory error");
        return;
    }

    *len = sizeof(esp_sensor_t);
    memcpy(buffer, sensor, *len);
}

void CreateAdvertise(uint8_t* buffer)
{
    if(buffer != NULL)
    {
        buffer[INDEX_MSG_TYPE] = MSG_ADVERTISE;
        buffer[INDEX_DEVICE_TYPE] = SENSOR_NODE;
        buffer[INDEX_ESPNOW_CHANNEL] = ESPNOW_CHANNEL;
        buffer[INDEX_ESPNOW_IF_TYPE] = ESPNOW_WIFI_IF;
        buffer[INDEX_ESPNOW_ADVERT_RSV1] = 0;
        buffer[INDEX_ESPNOW_SENSOR_TYPE] = MOISTURE_SENSOR;
        buffer[INDEX_ESPNOW_ADVERT_RSV2] = 0;
        buffer[INDEX_ESPNOW_SIMULATION] = 1;
    }
    else
    {
        ESP_LOGE(TAG, "CreateAdvertise memory error, aborting vTaskAdvertise");
        vTaskDelete(AdvertiseHandle);
    }
}

void vTaskSendSensorData(void* pvParameter)
{
    /* create a dummy sensor object */
    esp_sensor_t sensor_t1;
    sensor_t1.sensor_type = 0x01;
    uint32_t buff_len = 0;

    while(1)
    {
        sensor_t1.sensor_val.flt = GenerateRandomFloat(25.0f, 26.0f);
        PackSensorData(&sensor_t1, EspNowBuffer, &buff_len);
        esp_now_send(BroadcastMAC, EspNowBuffer, buff_len);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    vTaskDelete(NULL);
}

void vTaskAdvertise(void* pvParameter)
{
    esp_sensor_t device;
    CreateAdvertise((uint8_t*)&device);

    while(1)
    {
        esp_now_send(BroadcastMAC, (uint8_t*)&device, ESPNOW_ADVERT_MSG_LEN);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    vTaskDelete(NULL);
}

void vTaskProcessRxdData(void* pvParameters)
{
    rxd_obj_t rxObj = { 0 };
    esp_sensor_t sensorObj;
    esp_now_peer_num_t peerCount = { 0 };
    uint8_t* ptrU8RxObj = (uint8_t*)&sensorObj;
    sensorObj.sensor_val.u32 = 0;

    ESP_LOGI(TAG, "vTaskProcessRxdData started");
   
    ESP_LOGI(TAG, "sizeof esp_sensor_t %d ||| size of rxd_obj_t %d", sizeof(esp_sensor_t), sizeof(rxd_obj_t));

    while(xQueueReceive(ESPNowRxQueue, &rxObj, portMAX_DELAY) == pdTRUE)
    {
        ESP_LOGI(TAG, "Queue Received Data in Task");
        memcpy(&sensorObj, &rxObj.sensor_obj, rxObj.data_len);

        ESP_LOGD(TAG, "data len %d src mac %x %x %x %x %x %x\r\n",
        rxObj.data_len,
        rxObj.src_mac[0],
        rxObj.src_mac[1],
        rxObj.src_mac[2],
        rxObj.src_mac[3],
        rxObj.src_mac[4],
        rxObj.src_mac[5]);

        ESP_LOGD(TAG, "ESPNOW_SENSOR_TYPE %02x \r\n",
        ptrU8RxObj[INDEX_ESPNOW_SENSOR_TYPE]);

        switch(ptrU8RxObj[INDEX_MSG_TYPE])
        {
            case MSG_UNICAST:
            break;

            case MSG_BROADCAST:
            break;

            case MSG_ADVERTISE:
            esp_now_get_peer_num(&peerCount);
            if(peerCount.total_num <= ESP_NOW_MAX_TOTAL_PEER_NUM
            && !esp_now_is_peer_exist(rxObj.src_mac))
            {
                ESPAddPeer(rxObj);
            }
            ESP_LOGI(TAG, "%d peers present", peerCount.total_num);           
            break;

            default:
            break;
        }
    }
    vTaskDelete(NULL);
}

void InitSensor()
{
    if( xTaskCreate(vTaskSendSensorData, "Sensor1", 2*MINIMUM_STACK,
    NULL, 3, &SensorTaskHandle) != pdPASS)
    {
        ESP_LOGE(TAG, "Sensor1 initialization failed");
    }
}

float GenerateRandomFloat(float min, float max)
{
    srand(time(NULL));
    float scale = rand() / (float) RAND_MAX;
    return min + scale * (max - min);
}
