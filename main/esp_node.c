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

const uint8_t BroadcastMAC[ESP_MAC_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t ESPNowSeq[ESPNOW_DATA_MAX] = { 0, 0 };

xQueueHandle ESPNowRxQueue;
TaskHandle_t RxProcessHandle = NULL;
TaskHandle_t SensorTaskHandle = NULL;

static void ESPNowSendCallback(const uint8_t* mac_address, esp_now_send_status_t status);
static void ESPNowReceiveCallback(const uint8_t* mac_address, const uint8_t *data, int len);
static void vTaskProcessRxdData(void* pvParameters);
static void vTaskSendSensorData(void* pvParameter);

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
        ESP_LOGE(TAG, "InitESPBroadcast() - malloc failed");
    }
}

void InitESPNow()
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(ESPNowSendCallback));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(ESPNowReceiveCallback));
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)ESP_PMK));
    InitESPBroadcast();
    ESPNowRxQueue = xQueueCreate(RX_QUEUE_LEN, sizeof(rxd_obj_t*));
    if(ESPNowRxQueue == NULL)
    {
        ESP_LOGE(TAG, "ESPNowRxQueue initialization failed");
    }

    if( xTaskCreate(vTaskProcessRxdData, "RxProcessor", MINIMUM_STACK,
    NULL, 3, &RxProcessHandle) != pdPASS)
    {
        ESP_LOGE(TAG, "RxProcessor initialization failed");
    }
}

void ESPNow_Deinit()
{
    esp_now_deinit();
}

void ESPNowPrepareData(espnow_tx_param_t *send_param)
{
    espnow_data_t *buf = (espnow_data_t *)send_param->buffer;
    int i = 0;

    assert(send_param->len >= sizeof(espnow_data_t));

    buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? ESPNOW_DATA_BROADCAST : ESPNOW_DATA_UNICAST;
    buf->state = send_param->state;
    buf->seq_num = ESPNowSeq[buf->type]++;
    buf->crc = 0;
    buf->magic = send_param->magic;
    for (i = 0; i < send_param->len - sizeof(espnow_data_t); i++) {
        buf->payload[i] = (uint8_t)esp_random();
    }
    buf->crc = crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
}

int ESPNowParseData(uint8_t *data, uint16_t data_len, uint8_t *state, uint16_t *seq, int *magic)
{
    espnow_data_t *buf = (espnow_data_t *)data;
    uint16_t crc, crc_cal = 0;

    if (data_len < sizeof(espnow_data_t)) {
        ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
        return -1;
    }

    *state = buf->state;
    *seq = buf->seq_num;
    *magic = buf->magic;
    crc = buf->crc;
    buf->crc = 0;
    crc_cal = crc16_le(UINT16_MAX, (uint8_t const *)buf, data_len);

    if (crc_cal == crc) {
        return buf->type;
    }

    return -1;
}

static void ESPNowReceiveCallback(const uint8_t* mac_address, const uint8_t *data, int len)
{
    ESP_LOGD(TAG, "%d bytes of data rx'd", len);
    rxd_obj_t* rxObj = NULL;
    rxObj = malloc(sizeof(rxd_obj_t));
    memcpy(&rxObj->sensor_obj, data, len);
    memcpy(rxObj->src_mac, mac_address, ESP_MAC_LEN);
    rxObj->data_len = len;
    
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
        return;
    }

    *len = sizeof(esp_sensor_t);
    memcpy(buffer, sensor, *len);
}

void CreateAdvertise(uint8_t* buffer)
{

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

}

void vTaskProcessRxdData(void* pvParameters)
{
    rxd_obj_t* rxObj = NULL;
 
    ESP_LOGI(TAG, "vTaskProcessRxdData started");
 
    esp_sensor_t sensorObj;
    sensorObj.sensor_val.u32 = 0;

    ESP_LOGI(TAG, "sizeof esp_sensor_t %d ||| size of rxd_obj_t %d", sizeof(esp_sensor_t), sizeof(rxd_obj_t));

    while(xQueueReceive(ESPNowRxQueue, &rxObj, portMAX_DELAY) == pdTRUE)
    {
        ESP_LOGI(TAG, "Queue Received Data in Task");
        memcpy(&sensorObj, &rxObj->sensor_obj, rxObj->data_len);

        ESP_LOGD(TAG, "data len %d src mac %x %x %x %x %x %x\r\n",
        rxObj->data_len,
        rxObj->src_mac[0],
        rxObj->src_mac[1],
        rxObj->src_mac[2],
        rxObj->src_mac[3],
        rxObj->src_mac[4],
        rxObj->src_mac[5]);

        ESP_LOGD(TAG, "sensorObj %.1f \r\n", sensorObj.sensor_val.flt);

        free(rxObj);
    }
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
