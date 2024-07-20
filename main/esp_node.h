#ifndef ESP_NODE_H
#define ESP_NODE_H

#include "freertos/semphr.h"
#include "espnow_defines.h"

#define SENSOR_DEVICE

#define ESPNOW_CLIENT   1   // macro for espnow master device code
#define ESPNOW_SERVER    0   // macro for espnow slave device code

#define ESPNOW_DEV_MODE ESPNOW_CLIENT
// #define ESPNOW_DEV_MODE ESPNOW_SERVER

#define ESPNOW_CHANNEL  1

#define ESP_PMK "pmk1234567890123"
#define ESP_LMK "lmk1234567890123"

#define NODE_MAC_LEN    6
#define NODE_DATA_BUFFER_LEN    4

#define ESPNOW_MAX_PAYLOAD_LEN 200

#define MAX_SENSOR_NODES    20

/* BUFFER INDICES */
#define ESPNOW_ADVERT_MSG_LEN       8
#define ESPNOW_QUERY_LEN            8

// common
#define INDEX_MSG_TYPE              0
#define INDEX_DEVICE_TYPE           1
#define INDEX_ESPNOW_SIMULATION     7

// for advertisement
#define INDEX_ESPNOW_IF_TYPE        2
#define INDEX_ESPNOW_CHANNEL        3
#define INDEX_ESPNOW_ADVERT_RSV1    4
#define INDEX_ESPNOW_SENSOR_TYPE    5
#define INDEX_ESPNOW_ADVERT_RSV2    6

// for queries
#define INDEX_FUNCTION_CODE         2
#define INDEX_QUERY_RSV2            3
#define INDEX_QUERY_RSV3            4
#define INDEX_ESPNOW_SENSOR_TYPE    5
#define INDEX_QUERY_RSV4            6

// for responses
#define INDEX_RESPONSDING_SENSOR_TYPE   3


typedef enum ESPNOW_FUNTION_CODES
{
    READ_DIGITAL_INPUT = 0x01,
    READ_DIGITAL_IP = 0x02,
    READ_SENSOR_VALUE = 0x03,
    WRITE_DIGITAL_OUTPUT = 0x04,
    WRITE_ANALOG_OUTPUT = 0x10
}
function_code_t;

typedef enum DEVICE_TYPE
{
    SENSOR_NODE = 1,
    SENSOR_HUB,
    ESPNOW_TO_WIFI_GATEWAY,
    ESPNOW_TO_ESPNOW_GATEWAY,
    UNKNOWN_DEVICE
}
device_type_t;

typedef struct ESPNOW_MAC
{
    uint8_t mac[6];  // Array of 6 bytes for MAC address
}
__attribute__((packed)) node_mac_t;

#if ESPNOW_DEV_MODE == ESPNOW_SERVER
#define ESPNOW_DEVICE_TYPE SENSOR_NODE
#else
#define ESPNOW_DEVICE_TYPE ESPNOW_TO_WIFI_GATEWAY
#endif

typedef enum ESPNOW_SENSOR_TYPE
{
    TEMPERATURE_SENSOR = 1,
    HUMIDITY_SENSOR,
    PRESSURE_SENSOR,
    LUX_SENSOR,
    SOIL_MOISTURE_SENSOR,
    NO_SENSOR
}
sensor_type_t;

#ifdef SENSOR_DEVICE
#define ESPNOW_SENSOR_TYPE SOIL_MOISTURE_SENSOR
#else
#define ESPNOW_SENSOR_TYPE NO_SENSOR
#endif

typedef enum TX_MSG_TYPE_E
{
    MSG_UNICAST_QUERY = 0x1B,     //  Unicast Query
    MSG_UNICAST_RESPONSE = 0x1A,
    MSG_BROADCAST = 0xFF,   //  Broadcast
    MSG_ADVERTISE = 0xAF,    //  Advertise device
    MSG_ADVERTISE_ACK = 0xAA   //  Acknowledgment for advertisment message
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
    uint8_t sensor_type[4];
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
void ESPNowGenerateQuery(uint8_t* buffer, function_code_t fc, sensor_type_t sensor_type);
void GetPeerMAC(uint8_t index, node_mac_t* mac);
uint8_t GetPeerCount();

//
// simulation functions
//


//
//  helper functions
//
uint8_t* ESPNow_GetBufferPtr();

//
//  externs
//
extern xQueueHandle ESPSvcQueryQueue;

#endif