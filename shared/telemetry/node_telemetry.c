#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "node_telemetry.h"

#define NODE_MAX 10
// #define NODE_MAX UINT8_MAX

typedef struct {
    NodeTelemetry_t node;
    uint8_t active;
} NodeTelemetryRegistry_t;

static NodeTelemetryRegistry_t nodeList[NODE_MAX] = {0};
static uint8_t localNodeId = 0;

static inline uint8_t nodeTelemetry_checkNodeId(uint8_t node_id);

uint8_t nodeTelemetry_newNode(uint8_t node_id) {
    uint8_t ret = nodeTelemetry_checkNodeId(node_id);
    if (ret != NODE_TELEMETRY_ERR_NONE) {
        return ret;
    }

    if (nodeList[node_id].active) {
        return NODE_TELEMETRY_ERR_ALREADY_HAVE_NODE;
    }

    memset(&(nodeList[node_id]), 0, sizeof(nodeList[node_id]));

    nodeList[node_id].active = 1;
    nodeList[node_id].node.id = node_id;
    snprintf(nodeList[node_id].node.name, sizeof(nodeList[node_id].node.name),
             "0x%" PRIX8, node_id);

    return NODE_TELEMETRY_ERR_NONE;
}

void nodeTelemetry_deleteNode(uint8_t node_id) {
    if (nodeTelemetry_checkNodeId(node_id) != NODE_TELEMETRY_ERR_NONE) {
        return;
    }

    nodeList[node_id].active = 0;
}

uint8_t nodeTelemetry_setLocalNodeId(uint8_t node_id) {
    uint8_t ret = nodeTelemetry_checkNodeId(node_id);
    if (ret != NODE_TELEMETRY_ERR_NONE) {
        return ret;
    }

    nodeTelemetry_deleteNode(localNodeId);
    localNodeId = node_id;
    return nodeTelemetry_newNode(localNodeId);
}

NodeTelemetry_t *nodeTelemetry_getLocalNode(void) {
    return nodeTelemetry_getNode(localNodeId);
}

NodeTelemetry_t *nodeTelemetry_getNode(uint8_t node_id) {
    if (nodeTelemetry_checkNodeId(node_id) != NODE_TELEMETRY_ERR_NONE) {
        return NULL;
    }

    if (!(nodeList[node_id].active)) {
        return NULL;
    }

    return &(nodeList[node_id].node);
}

void nodeTelemetry_forEach(void (*callback)(NodeTelemetry_t *node)) {
    uint8_t idx = 0;
    for (uint8_t i = 0; i < sizeof(nodeList) / sizeof(nodeList[0]); i++) {
        if (nodeList[i].active) {
            callback(&nodeList[i].node);
        }
    }
}

static inline uint8_t nodeTelemetry_checkNodeId(uint8_t node_id) {
    if (node_id > NODE_MAX) {
        return NODE_TELEMETRY_ERR_INVALID_NODE_ID;
    }
    return NODE_TELEMETRY_ERR_NONE;
}
