#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_log.h"

#define ESP_WIFI_SSIDAP      "ESP32"
#define ESP_WIFI_PASSAP      "12345678"
#define MAX_STA_CONN        4

static const char *TAG = "wifi_manager";

/* FreeRTOS event group to signal when we are connected properly */
static EventGroupHandle_t wifi_event_group;

static void
event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_AP_STACONNECTED:
            //once connected start mqtt 
            /* code */
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            /* code */
            break;
        case WIFI_EVENT_AP_START:
            break;
        case WIFI_EVENT_AP_STOP:
            break;
       
        default:
            break;
        }
    }
}

void Wifi_AP(void)
{
    wifi_config_t wifi_config_ap = {
        .ap = {.ssid           = ESP_WIFI_SSIDAP,
               .channel        = 1,
               .ssid_len       = strlen(ESP_WIFI_SSIDAP),
               .password       = ESP_WIFI_PASSAP,
               .max_connection = MAX_STA_CONN,
               .authmode       = WIFI_AUTH_WPA_WPA2_PSK},
    };

    if (strlen(ESP_WIFI_PASSAP) == 0)
    {
        wifi_config_ap.ap.authmode = WIFI_AUTH_OPEN;
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config_ap));
    ESP_LOGI(TAG, "wifi_init_ap finished.");
}
void Initialize_Wifi() {

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID,
                    &event_handler,
                    NULL,
                    NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                    IP_EVENT_STA_GOT_IP,
                    &event_handler,
                    NULL,
                    NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    esp_netif_create_default_wifi_ap();

    Wifi_AP();

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi_init_finished.");

}

void Wifi_Init()
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_INIT");
    Initialize_Wifi();
}