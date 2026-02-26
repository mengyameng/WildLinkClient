#pragma once
#ifndef __NODE_TELEMETRY_H__
#define __NODE_TELEMETRY_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include <stdint.h>

#pragma pack(1)
typedef struct {
    uint8_t id;

    uint8_t need_help;

    char name[10];

    uint8_t heart_rate_min;
    uint8_t heart_rate_max;
    uint8_t heart_rate;

    uint8_t blood_oxygen_low;
    uint8_t blood_oxygen;

    float body_temp_min;
    float body_temp_max;
    float body_temp;

    uint8_t air_humidity;
    float air_temp;
    float air_pressure;

    struct GPSCoord_t {
        float longitude;
        float latitude;
    } gps;

    uint32_t timestamp;
} NodeTelemetry_t;
#pragma pack()

#define NODE_TELEMETRY_ERR_NONE 0
#define NODE_TELEMETRY_ERR_FAILED 1
#define NODE_TELEMETRY_ERR_ALREADY_HAVE_NODE 2
#define NODE_TELEMETRY_ERR_INVALID_NODE_ID 3

uint8_t nodeTelemetry_newNode(uint8_t node_id);
void nodeTelemetry_deleteNode(uint8_t node_id);
uint8_t nodeTelemetry_setLocalNodeId(uint8_t node_id);
NodeTelemetry_t *nodeTelemetry_getLocalNode(void);
NodeTelemetry_t *nodeTelemetry_getNode(uint8_t node_id);
void nodeTelemetry_forEach(void (*callback)(NodeTelemetry_t *node));

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //__NODE_TELEMETRY_H__