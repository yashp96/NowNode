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

#define TAG "SENSOR-SERVICE"

TaskHandle_t SensorTaskHandle = NULL;

static void vServiceTask(void* pvParameter);

float GenerateRandomFloat(float min, float max)
{
    srand(time(NULL));
    float scale = rand() / (float) RAND_MAX;
    return min + scale * (max - min);
}

int GenerateRandomInt(int min, int max)
{
    srand(time(NULL));
    int scale = rand() / (int) RAND_MAX;
    return min + scale * (max - min);
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

void vServiceTask(void* pvParameter)
{
    /* create a dummy sensor object */
    esp_sensor_t sensor_t1 = { {0} , {0} };
    uint8_t* ptrU8SensorObj = (uint8_t*)&sensor_t1;

    uint32_t buff_len = 0;

    rxd_obj_t svcObj = { 0 };
    uint8_t* ptrU8SvcObj = (uint8_t*)&svcObj.sensor_obj;

    while(xQueueReceive(ESPSvcQueryQueue, &svcObj, portMAX_DELAY) == pdTRUE)
    {
         ESP_LOGD(TAG, "\n BUFFER[0] %02x\n BUFFER[1] %02x\n BUFFER[2] %02x\n BUFFER[3] %02x\n \
BUFFER[4] %02x\n BUFFER[5] %02x\n BUFFER[6] %02x\n BUFFER[7] %02x\n",
        ptrU8SvcObj[INDEX_MSG_TYPE], ptrU8SvcObj[INDEX_DEVICE_TYPE], ptrU8SvcObj[INDEX_FUNCTION_CODE],
        ptrU8SvcObj[INDEX_QUERY_RSV2], ptrU8SvcObj[INDEX_QUERY_RSV3], ptrU8SvcObj[INDEX_ESPNOW_SENSOR_TYPE],
        ptrU8SvcObj[INDEX_QUERY_RSV4], ptrU8SvcObj[INDEX_ESPNOW_SIMULATION]);


        if (ptrU8SvcObj[INDEX_DEVICE_TYPE] != SENSOR_NODE &&
            ptrU8SvcObj[INDEX_FUNCTION_CODE] == READ_SENSOR_VALUE)
        {
            switch (ptrU8SvcObj[INDEX_ESPNOW_SENSOR_TYPE])
            {
            case TEMPERATURE_SENSOR:
                sensor_t1.sensor_val.flt = GenerateRandomFloat(25.0f, 26.0f);
                break;
            
            case HUMIDITY_SENSOR:
                sensor_t1.sensor_val.flt = GenerateRandomFloat(60.0f, 80.0f);
                break;

            case PRESSURE_SENSOR:
                sensor_t1.sensor_val.u32 = 0;
                break;

            case LUX_SENSOR:
                sensor_t1.sensor_val.u32 = 0;
                break;

            case SOIL_MOISTURE_SENSOR:
                sensor_t1.sensor_val.u32 = (uint32_t)GenerateRandomInt(0, 1023);
                break;

            default:
                break;
            }
        }       

        ptrU8SensorObj[INDEX_MSG_TYPE] = MSG_UNICAST_RESPONSE;
        ptrU8SensorObj[INDEX_DEVICE_TYPE] = SENSOR_NODE;
        ptrU8SensorObj[INDEX_FUNCTION_CODE] = ptrU8SvcObj[INDEX_FUNCTION_CODE];
        ptrU8SensorObj[INDEX_RESPONSDING_SENSOR_TYPE] = ptrU8SvcObj[INDEX_ESPNOW_SENSOR_TYPE];
        
        esp_now_send(svcObj.src_mac, ptrU8SensorObj, sizeof(esp_sensor_t));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}

void InitServer()
{
    if( xTaskCreate(vServiceTask, "ServiceTask", 2*MINIMUM_STACK,
    NULL, 3, &SensorTaskHandle) != pdPASS)
    {
        ESP_LOGE(TAG, "Service Task initialization failed");
    }
}