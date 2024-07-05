#ifndef ESP_NODE_H
#define ESP_NODE_H

#include "espnow_defines.h"

#define ESPNOW_CHANNEL  1

#define ESP_PMK "pmk1234567890123"
#define ESP_LMK "lmk1234567890123"

#define NODE_MAC_LEN    6
#define NODE_DATA_BUFFER_LEN    4

#define ESPNOW_MAX_PAYLOAD_LEN 200

#define MAX_SENSOR_NODES    20

/* TODO: send broadcast message only to advertise self meta data
for actual data use query-response method 
register a node using advertise based on device type */
#define ESPNOW_ADVERT_MSG_LEN       8
#define INDEX_MSG_TYPE              0
#define INDEX_DEVICE_TYPE           1
#define INDEX_ESPNOW_IF_TYPE        2
#define INDEX_ESPNOW_CHANNEL        3
#define INDEX_ESPNOW_ADVERT_RSV1    4
#define INDEX_ESPNOW_SENSOR_TYPE    5
#define INDEX_ESPNOW_ADVERT_RSV2    6
#define INDEX_ESPNOW_SIMULATION     7

typedef enum DEVICE_TYPE
{
    SENSOR_NODE = 1,
    SENSOR_HUB,
    ESPNOW_TO_WIFI_GATEWAY,
    ESPNOW_TO_ESPNOW_GATEWAY,
    UNKNOWN_DEVICE
}
device_type_t;

typedef enum SENSOR_TYPE
{
    TEMPERATURE_SENSOR = 1,
    HUMIDITY_SENSOR,
    PRESSURE_SENSOR,
    LUX_SENSOR,
    MOISTURE_SENSOR
}
sensor_type_t;

typedef enum TX_MSG_TYPE_E
{
    MSG_UNICAST = 0x1B,     //  Unicast Query
    MSG_BROADCAST = 0xFF,   //  Broadcast
    MSG_ADVERTISE = 0xAF,    //  Advertise device
    MSG_ADVERT_ACK = 0xAA   //  Acknowledgment for advertisment message
}
tx_msg_type_e;

typedef union SENSOR_U
{
    uint32_t u32;
    float   flt;
    uint8_t uchar[4];
}sensor_val_t;

typedef struct ESP_SENSOR_S
{
    uint32_t sensor_type;
    sensor_val_t sensor_val;
}
esp_sensor_t;

typedef struct
{
    uint8_t src_mac[NODE_MAC_LEN];
    int data_len;
    esp_sensor_t sensor_obj;
}
rxd_obj_t;

//
//  esp-now functions
//
void InitESPNow();
void ESPNow_Deinit();

//
// simulation functions
//
void InitSensor();

//
//  helper functions
//
float GenerateRandomFloat(float min, float max);

#endif