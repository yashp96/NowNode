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

#if ESPNOW_DEV_MODE == ESPNOW_SERVER
#define TAG "SERVER-NODE"
#else
#define TAG "CLIENT-NODE"
#endif

#define RX_QUEUE_LEN 5

static uint8_t MasterAvailable = false;

static uint8_t EspNowBuffer[ESPNOW_MAX_PAYLOAD_LEN] = { 0 };
// static esp_now_peer_info_t PeerList[MAX_SENSOR_NODES] = { { { 0 } } };
static node_mac_t ESPNowPeerMACList[MAX_SENSOR_NODES] = { { { 0 } } };
static uint32_t NumberOfAvailablePeers = 0;

const uint8_t BroadcastMAC[ESP_MAC_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

xQueueHandle ESPNowRxQueue;
xQueueHandle ESPSvcQueryQueue;
xQueueHandle ESPSvcResponseQueue;

TaskHandle_t RxProcessHandle = NULL;
TaskHandle_t AdvertiseHandle = NULL;

static void ESPNowSendCallback(const uint8_t* mac_address, esp_now_send_status_t status);
static void ESPNowReceiveCallback(const uint8_t* mac_address, const uint8_t *data, int len);

static void vTaskAdvertise(void* pvParameter);
static void vTaskProcessRxdData(void* pvParameters);

uint8_t* ESPNow_GetBufferPtr()
{
    return EspNowBuffer;
}

void ESPAddPeer(rxd_obj_t rx_obj)
{
    esp_now_peer_info_t* peer = (esp_now_peer_info_t*)malloc(sizeof(esp_now_peer_info_t));
    uint8_t* buffr = (uint8_t*)&rx_obj.sensor_obj;

    if(peer != NULL)
    {
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        peer->channel = buffr[INDEX_ESPNOW_CHANNEL];
        peer->ifidx = buffr[INDEX_ESPNOW_IF_TYPE];
        peer->encrypt = false;
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

    ESPSvcQueryQueue = xQueueCreate(RX_QUEUE_LEN, sizeof(rxd_obj_t));
    if(ESPSvcQueryQueue == NULL)
    {
        ESP_LOGE(TAG, "ESPSvcQueryQueue initialization failed");
    }

    ESPSvcResponseQueue = xQueueCreate(RX_QUEUE_LEN, sizeof(rxd_obj_t));
    if(ESPSvcResponseQueue == NULL)
    {
        ESP_LOGE(TAG, "ESPSvcResponseQueue initialization failed");
    }

    if( xTaskCreate(vTaskProcessRxdData, "RxProcessor", MINIMUM_STACK*2,
    NULL, 3, &RxProcessHandle) != pdPASS)
    {
        ESP_LOGE(TAG, "RxProcessor initialization failed");
    }

#if ESPNOW_DEV_MODE == ESPNOW_SERVER
    if( xTaskCreate(vTaskAdvertise, "AvertiseProcess", MINIMUM_STACK,
    NULL, 3, &AdvertiseHandle) != pdPASS)
    {
        ESP_LOGE(TAG, "Advertisement initialization failed");
    }
#endif

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
        ESP_LOGI(TAG, "Sent to rx queue");
    }
}

static void ESPNowSendCallback(const uint8_t* mac_address, esp_now_send_status_t status)
{
    if(status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(TAG, "ESP_NOW_SEND_SUCCESS");
    }
}

void ESPNowGenerateQuery(uint8_t* buffer, function_code_t fc, sensor_type_t sensor_type)
{
    if(buffer != NULL)
    {
        buffer[INDEX_MSG_TYPE] = MSG_UNICAST_QUERY;
        buffer[INDEX_DEVICE_TYPE] = ESPNOW_DEVICE_TYPE;
        buffer[INDEX_FUNCTION_CODE] = (uint8_t)fc;
        buffer[INDEX_QUERY_RSV2] = 0xFF;
        buffer[INDEX_QUERY_RSV3] = 0xFF;
        buffer[INDEX_ESPNOW_SENSOR_TYPE] = (uint8_t)sensor_type;
        buffer[INDEX_QUERY_RSV4] = 0xFF;
        buffer[INDEX_ESPNOW_SIMULATION] = 1;
    }
    else
    {
        ESP_LOGE(TAG, "Memory Fault");
    }
}

void CreateAdvertise(uint8_t* buffer)
{
    if(buffer != NULL)
    {
        buffer[INDEX_MSG_TYPE] = MSG_ADVERTISE;
        buffer[INDEX_DEVICE_TYPE] = ESPNOW_DEVICE_TYPE;
        buffer[INDEX_ESPNOW_CHANNEL] = ESPNOW_CHANNEL;
        buffer[INDEX_ESPNOW_IF_TYPE] = ESPNOW_WIFI_IF;
        buffer[INDEX_ESPNOW_ADVERT_RSV1] = 0;
        buffer[INDEX_ESPNOW_SENSOR_TYPE] = ESPNOW_SENSOR_TYPE;
        buffer[INDEX_ESPNOW_ADVERT_RSV2] = 0;
        buffer[INDEX_ESPNOW_SIMULATION] = 1;
    }
    else
    {
        ESP_LOGE(TAG, "CreateAdvertise memory error, aborting vTaskAdvertise");
        vTaskDelete(AdvertiseHandle);
    }
}

#if ESPNOW_DEV_MODE == ESPNOW_SERVER
void vTaskAdvertise(void* pvParameter)
{
    esp_sensor_t device;
    CreateAdvertise((uint8_t*)&device);

    while(1)
    {
        if (MasterAvailable == false)
        {
            esp_now_send(BroadcastMAC, (uint8_t*)&device, ESPNOW_ADVERT_MSG_LEN);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    vTaskDelete(NULL);
}
#endif

void AddPeerMac(int index, uint8_t* src)
{
    if(src == NULL || index < 0 || index >= MAX_SENSOR_NODES)
    {
        return;
    }
    ESPNowPeerMACList[index].mac[0] = src[0];
    ESPNowPeerMACList[index].mac[1] = src[1];
    ESPNowPeerMACList[index].mac[2] = src[2];
    ESPNowPeerMACList[index].mac[3] = src[3];
    ESPNowPeerMACList[index].mac[4] = src[4];
    ESPNowPeerMACList[index].mac[5] = src[5];
    NumberOfAvailablePeers++;
    ESP_LOGD(TAG, "Peer at %x %x %x %x %x %x\r\n",
    ESPNowPeerMACList[index].mac[0], ESPNowPeerMACList[index].mac[1], ESPNowPeerMACList[index].mac[2],
    ESPNowPeerMACList[index].mac[3], ESPNowPeerMACList[index].mac[4], ESPNowPeerMACList[index].mac[5]);
}

void GetPeerMAC(uint8_t index, node_mac_t* mac)
{
    if(mac != NULL && index < MAX_SENSOR_NODES)
    {
        memcpy(mac, &ESPNowPeerMACList[index], sizeof(node_mac_t));
    }
}

uint8_t GetPeerCount()
{
    return NumberOfAvailablePeers;
}

void vTaskProcessRxdData(void* pvParameters)
{
    uint8_t sendAdvertACK = 0;
    rxd_obj_t rxObj = { 0 };
    esp_sensor_t sensorObj;
    esp_now_peer_num_t peerCount = { 0 };
    uint8_t* ptrU8RxObj = (uint8_t*)&sensorObj;
    sensorObj.sensor_val.u32 = 0;

    ESP_LOGI(TAG, "vTaskProcessRxdData started");
   
    ESP_LOGI(TAG, "sizeof esp_sensor_t %d ||| size of rxd_obj_t %d", sizeof(esp_sensor_t), sizeof(rxd_obj_t));

    while(xQueueReceive(ESPNowRxQueue, &rxObj, portMAX_DELAY) == pdTRUE)
    {
        sendAdvertACK = 0;
        ESP_LOGI(TAG, "Queue Received Data in vTaskProcessRxdData");
        memcpy(&sensorObj, &rxObj.sensor_obj, rxObj.data_len);

/*         ESP_LOGD(TAG, "data len %d src mac %x %x %x %x %x %x\r\n", rxObj.data_len,
        rxObj.src_mac[0], rxObj.src_mac[1], rxObj.src_mac[2],
        rxObj.src_mac[3], rxObj.src_mac[4], rxObj.src_mac[5]); */

/*         ESP_LOGD(TAG, "\n BUFFER[0] %02x\n BUFFER[1] %02x\n BUFFER[2] %02x\n BUFFER[3] %02x\n \
BUFFER[4] %02x\n BUFFER[5] %02x\n BUFFER[6] %02x\n BUFFER[7] %02x\n",
        ptrU8RxObj[INDEX_MSG_TYPE], ptrU8RxObj[INDEX_DEVICE_TYPE], ptrU8RxObj[INDEX_ESPNOW_IF_TYPE],
        ptrU8RxObj[INDEX_ESPNOW_CHANNEL], ptrU8RxObj[INDEX_ESPNOW_ADVERT_RSV1], ptrU8RxObj[INDEX_ESPNOW_SENSOR_TYPE],
        ptrU8RxObj[INDEX_ESPNOW_ADVERT_RSV2], ptrU8RxObj[INDEX_ESPNOW_SIMULATION]); */

        switch(ptrU8RxObj[INDEX_MSG_TYPE])
        {
            case MSG_UNICAST_QUERY:
                if(esp_now_is_peer_exist(rxObj.src_mac) 
                /* as well as check if the device is registered as master?? */)
                {
                    if(xQueueSend(ESPSvcQueryQueue, &rxObj, portMAX_DELAY) == pdTRUE)
                    {
                        ESP_LOGI(TAG, "Query Sent to service queue");
                    }
                }
            break;

            case MSG_UNICAST_RESPONSE:
                if(esp_now_is_peer_exist(rxObj.src_mac) 
                /* should we check if device has sent a query?? */)
                {
                    if(xQueueSend(ESPSvcResponseQueue, &rxObj, portMAX_DELAY) == pdTRUE)
                    {
                        ESP_LOGI(TAG, "Response Sent to service queue");
                    }
                }
            break;

            case MSG_BROADCAST:
            break;

            case MSG_ADVERTISE:
                esp_now_get_peer_num(&peerCount);
                if(!esp_now_is_peer_exist(rxObj.src_mac) &&
                ptrU8RxObj[INDEX_DEVICE_TYPE] == SENSOR_NODE &&
                peerCount.total_num < ESP_NOW_MAX_TOTAL_PEER_NUM)
                {
                    ESPAddPeer(rxObj);
                    AddPeerMac(NumberOfAvailablePeers, rxObj.src_mac);
                    ESP_LOGI(TAG, "MSG_ADVERTISE :: Server Registered");
                    sendAdvertACK = 1;
                    ptrU8RxObj[INDEX_MSG_TYPE] = MSG_ADVERTISE_ACK;
                    ptrU8RxObj[INDEX_DEVICE_TYPE] = ESPNOW_DEVICE_TYPE;  
                }                     
                break;  

            case MSG_ADVERTISE_ACK:
                if(!esp_now_is_peer_exist(rxObj.src_mac) &&
                ptrU8RxObj[INDEX_DEVICE_TYPE] != SENSOR_NODE &&
                peerCount.total_num < ESP_NOW_MAX_TOTAL_PEER_NUM)
                {
                    ESPAddPeer(rxObj);
                    MasterAvailable = true;
                    ESP_LOGI(TAG, "MSG_ADVERTISE_ACK :: Client Registered");
                }
                break;

            default:
            break;
        }

        if(sendAdvertACK == 1)
        {
            esp_now_send(rxObj.src_mac, ptrU8RxObj, ESPNOW_ADVERT_MSG_LEN);
            ESP_LOGD(TAG, "Sending MSG_ADVERTISE_ACK");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}
