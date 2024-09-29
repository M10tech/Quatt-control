/*  (c) 2021-2024 HomeAccessoryKid
 *  Intended as a Domoticz publish only feed
 */
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include <string.h>
#include <udplogger.h>
#include "mqtt-client.h" //my local headers
#include "mqtt_client.h" //from ESP idf base

static esp_mqtt_client_handle_t client;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        UDPLUS("MQTT_EVENT_CONNECTED\n");
        break;
    case MQTT_EVENT_DISCONNECTED:
        UDPLUS("MQTT_EVENT_DISCONNECTED\n");
        break;
    case MQTT_EVENT_PUBLISHED:
//         UDPLUS("MQTT_EVENT_PUBLISHED, msg_id=%d\n", event->msg_id);
        break;
    case MQTT_EVENT_ERROR:
        UDPLUS("MQTT_EVENT_ERROR\n");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            UDPLUS("Last errno string (%s)\n", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        UDPLUS("Other event id:%d\n", event->event_id);
        break;
    }
}

static mqtt_config_t *mqttconf;

int mqtt_client_publish(char *format, ...) {
    char msg[mqttconf->msg_len];
    va_list args;
    va_start(args, format);
    int n=vsnprintf(msg, mqttconf->msg_len,format,args);
    va_end(args);
    if (n>=mqttconf->msg_len) return -2; //truncated message
    int res=esp_mqtt_client_enqueue(client, mqttconf->topic, msg, 0, 1, 0, true);
    if (res==-2) {
        return -1; //message queue full
    } else if (res==-1) {
        return -3; //general error
    }
    return n;
}

void mqtt_client_init(mqtt_config_t *config) {
    mqttconf=config;
    char uri[128];
    snprintf(uri,128,"mqtt://%s:%s@%s", mqttconf->user, mqttconf->pass, mqttconf->host);
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = uri,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}
