
#include "mqtt_client.h"
#include "esp_log.h"

esp_mqtt_client_handle_t client = NULL;

static const char *TAG = "MQTT-CLIENT";
bool MQTT_CONNEECTED = 0;

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Event dispatched event_id=%ld", event_id);
    esp_mqtt_event_handle_t event   = event_data;
    esp_mqtt_client_handle_t client = event->client;

    int msg_id;

    switch ((esp_mqtt_event_id_t)event_id)
    {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            MQTT_CONNEECTED = 1;
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            MQTT_CONNEECTED = 0;
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

void Publish(char *radar_data, char *topic)
{
    if (MQTT_CONNEECTED)
    {
        // Convert the data to json 
        esp_mqtt_client_publish(client, topic, radar_data, sizeof(radar_data), 1, 0);

        // ESP_LOGI(TAG, "Published telemetry data: %s", radar_data);
    }
}

void Init_Mqtt(){

    ESP_LOGI(TAG, "STARTING MQTT");
    esp_mqtt_client_config_t mqtt_cfg = 
    {
        .broker.address.uri = "mqtt://broker.emqx.io:1883"
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
    
}