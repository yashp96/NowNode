/* Part of NowNode for transmitting data */

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "rom/ets_sys.h"
#include "rom/crc.h"
#include "node_defines.h"
#include "driver/gpio.h"

/* defines */
#define TAG "NODE"
#define ESP_PMK "pmk1234567890123"
#define ESP_LMK "lmk1234567890123"

#define ESPNOW_CHANNEL  1

/* variables and constants */
const uint8_t BroadcastMAC[ESP_MAC_LEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static uint16_t ESPNowSeq[ESPNOW_DATA_MAX] = { 0, 0 };

/* Queues and Locks */
xQueueHandle EspNowEventQueue;

/* function prototypes */
void InitWiFi();
void InitESPNow();
static void ESPNowSendCallback(const uint8_t* mac_address, esp_now_send_status_t status);
static void ESPNowReceiveCallback(const uint8_t* mac_address, const uint8_t *data, int len);
void ESPNow_Deinit(espnow_tx_param_t *send_param);
void ESPNowPrepareData(espnow_tx_param_t *send_param);
void vTaskESPNow(void* pvParameter);
void ESPNow_Deinit(espnow_tx_param_t *send_param);

/* function definitions */

void ESPNow_Deinit(espnow_tx_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(EspNowEventQueue);
    esp_now_deinit();
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

void vTaskESPNow(void* pvParameter)
{
    espnow_event_t event;
    uint8_t recv_state = 0;
    uint16_t recv_seq = 0;
    int recv_magic = 0;
    bool is_broadcast = false;
    int ret;

    vTaskDelay(5000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");

    /* Start sending broadcast ESPNOW data. */
    espnow_tx_param_t *send_param = (espnow_tx_param_t *)pvParameter;
    if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
        ESP_LOGE(TAG, "Send error line 91");
        ESPNow_Deinit(send_param);
        vTaskDelete(NULL);
    }

    while (xQueueReceive(EspNowEventQueue, &event, portMAX_DELAY) == pdTRUE) {
        switch (event.id) {
            case ESPNOW_SEND_CB:
            {
                espnow_event_send_cb_t *send_cb = &event.info.send_cb;
                is_broadcast = IS_BROADCAST_ADDR(send_cb->mac_address);

                ESP_LOGD(TAG, "Send data to line 107 "MACSTR", status1: %d", MAC2STR(send_cb->mac_address), send_cb->status);

                if (is_broadcast && (send_param->broadcast == false)) {
                    break;
                }

                if (!is_broadcast) {
                    send_param->count--;
                    if (send_param->count == 0) {
                        ESP_LOGI(TAG, "Send done");
                        ESPNow_Deinit(send_param);
                        vTaskDelete(NULL);
                    }
                }

                /* Delay a while before sending the next data. */
                if (send_param->delay > 0) 
                {
                    vTaskDelay(send_param->delay/portTICK_RATE_MS);
                }

                ESP_LOGI(TAG, "send data to line 128 "MACSTR"", MAC2STR(send_cb->mac_address));

                memcpy(send_param->dest_mac, send_cb->mac_address, ESP_MAC_LEN);
                
                ESPNowPrepareData(send_param);

                /* Send the next data after the previous data is sent. */
                if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                    ESP_LOGE(TAG, "Send error line 131");
                    ESPNow_Deinit(send_param);
                    vTaskDelete(NULL);
                }
                break;
            }
            case ESPNOW_RECV_CB:
            {
                espnow_event_recv_cb_t *recv_cb = &event.info.recv_cb;

                ret = ESPNowParseData(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                free(recv_cb->data);
                if (ret == ESPNOW_DATA_BROADCAST) {
                    ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_address), recv_cb->data_len);

                    /* If MAC address does not exist in peer list, add it to peer list. */
                    if (esp_now_is_peer_exist(recv_cb->mac_address) == false) {
                        esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
                        if (peer == NULL) {
                            ESP_LOGE(TAG, "Malloc peer information fail");
                            ESPNow_Deinit(send_param);
                            vTaskDelete(NULL);
                        }
                        memset(peer, 0, sizeof(esp_now_peer_info_t));
                        peer->channel = ESPNOW_CHANNEL;
                        peer->ifidx = ESPNOW_WIFI_IF;
                        peer->encrypt = true;
                        memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
                        memcpy(peer->peer_addr, recv_cb->mac_address, ESP_MAC_LEN);
                        ESP_ERROR_CHECK( esp_now_add_peer(peer) );
                        free(peer);
                    }

                    /* Indicates that the device has received broadcast ESPNOW data. */
                    if (send_param->state == 0) {
                        send_param->state = 1;
                    }

                    /* If receive broadcast ESPNOW data which indicates that the other device has received
                     * broadcast ESPNOW data and the local magic number is bigger than that in the received
                     * broadcast ESPNOW data, stop sending broadcast ESPNOW data and start sending unicast
                     * ESPNOW data.
                     */
                    if (recv_state == 1) {
                        /* The device which has the bigger magic number sends ESPNOW data, the other one
                         * receives ESPNOW data.
                         */
                        if (send_param->unicast == false && send_param->magic >= recv_magic) {
                    	    ESP_LOGI(TAG, "Start sending unicast data");
                    	    ESP_LOGI(TAG, "send data to 185 "MACSTR"", MAC2STR(recv_cb->mac_address));

                    	    /* Start sending unicast ESPNOW data. */
                            memcpy(send_param->dest_mac, recv_cb->mac_address, ESP_MAC_LEN);
                            ret = ESPNowParseData(recv_cb->data, recv_cb->data_len, &recv_state, &recv_seq, &recv_magic);
                            if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
                                ESP_LOGE(TAG, "Send error line 186");
                                ESPNow_Deinit(send_param);
                                vTaskDelete(NULL);
                            }
                            else {
                                send_param->broadcast = false;
                                send_param->unicast = true;
                            }
                        }
                    }
                }
                else if (ret == ESPNOW_DATA_UNICAST) {
                    ESP_LOGI(TAG, "Receive %dth unicast data from: "MACSTR", len: %d", recv_seq, MAC2STR(recv_cb->mac_address), recv_cb->data_len);

                    /* If receive unicast ESPNOW data, also stop sending broadcast ESPNOW data. */
                    send_param->broadcast = false;
                }
                else {
                    ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_address));
                }
                break;
            }
            default:
                ESP_LOGE(TAG, "Callback type error: %d", event.id);
                break;
        }
    }
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

static void ESPNowSendCallback(const uint8_t* mac_address, esp_now_send_status_t status)
{
    espnow_event_t event;
    espnow_event_send_cb_t* send_cb = &event.info.send_cb;

    if(mac_address == NULL)
    {
        ESP_LOGE(TAG, "mac address invalid");
        return;
    }

    event.id = ESPNOW_SEND_CB;
    memcpy(send_cb->mac_address, mac_address, ESP_MAC_LEN);
    send_cb->status = status;

    if(xQueueSend(EspNowEventQueue, &event, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGE(TAG, "error in EspNowEventQueue");
    }
}

static void ESPNowReceiveCallback(const uint8_t* mac_address, const uint8_t *data, int len)
{
    espnow_event_t event;
    espnow_event_recv_cb_t *recv_cb = &event.info.recv_cb;

    if (mac_address == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    event.id = ESPNOW_RECV_CB;
    memcpy(recv_cb->mac_address, mac_address, ESP_MAC_LEN);

    recv_cb->data = malloc(len);
    if (recv_cb->data == NULL) {
        ESP_LOGE(TAG, "malloc receive data fail");
        return;
    }

    memcpy(recv_cb->data, data, len);
    recv_cb->data_len = len;
    if (xQueueSend(EspNowEventQueue, &event, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "send receive queue fail");
        free(recv_cb->data);
    }
}

void InitESPNow()
{
    espnow_tx_param_t *send_param;
    EspNowEventQueue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    
    if(EspNowEventQueue == NULL)
    {
        ESP_LOGE(TAG, "unable to create EspNowEventQueue");
        return;
    }

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(ESPNowSendCallback));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(ESPNowReceiveCallback));

    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t*)ESP_PMK));

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) 
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vQueueDelete(EspNowEventQueue);
        esp_now_deinit();
        return;
    }

    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, BroadcastMAC, ESP_MAC_LEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    send_param = malloc(sizeof(espnow_tx_param_t));
    memset(send_param, 0, sizeof(espnow_tx_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vQueueDelete(EspNowEventQueue);
        esp_now_deinit();
        return;
    }

    send_param->unicast = false;
    send_param->broadcast = true;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = 200;
    send_param->delay = 1000;
    send_param->len = 200;
    send_param->buffer = malloc(200);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(send_param);
        vQueueDelete(EspNowEventQueue);
        esp_now_deinit();
        return;
    }

    memcpy(send_param->dest_mac, BroadcastMAC, ESP_MAC_LEN);
    ESPNowPrepareData(send_param);
    xTaskCreate(vTaskESPNow, "ESPNow-Task", 2048, send_param, 4, NULL);
    return;
}

void InitWiFi()
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, 0))
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    InitWiFi();
    InitESPNow();

    // while (1)
    // {
    //     ESP_LOGI(TAG, "bla bla bla");
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
}