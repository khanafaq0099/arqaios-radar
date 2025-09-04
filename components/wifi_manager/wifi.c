#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "string.h"
#include "mqtt.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
// AP CONFIG
#define ESP_WIFI_SSIDAP      "ESP32"
#define ESP_WIFI_PASSAP      "12345678"
#define MAX_STA_CONN        4
// STA CONFIG
#define ESP_WIFI_SSID       "afaq"
#define ESP_WIFI_PASS       "asdf@1234"

static const char *TAG = "wifi_manager";

int connected = 0;

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
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            break;
        case WIFI_EVENT_AP_START:
            break;
        case WIFI_EVENT_AP_STOP:
            break;
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            ESP_LOGI(TAG, "Station started");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "Station Disconnected Retrying SSID:%s , PSWD:%s", ESP_WIFI_SSID, ESP_WIFI_PASS);
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "Station Connected");
            break;
        default:
            break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        switch (event_id)
        {
        case IP_EVENT_STA_GOT_IP:
                ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
                connected = 1;
                    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);

            
            break;
        default:
            break;
        }
    }
    
}
void Wifi_STA(void)
{
    wifi_config_t wifi_config_sta = {
        .sta =
            {
                .threshold.authmode = WIFI_AUTH_WPA2_PSK,
                .pmf_cfg            = {.capable = true, .required = false},
            },
    };

    sprintf((char *)wifi_config_sta.sta.ssid, "%s", ESP_WIFI_SSID);
    sprintf((char *)wifi_config_sta.sta.password, "%s", ESP_WIFI_PASS);
    ESP_LOGI(TAG, "SSID =%s &&&& Pass=%s\n", ESP_WIFI_SSID, ESP_WIFI_PASS);
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_sta));

    ESP_LOGI(TAG, "wifi_init_sta finished.");
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
    esp_netif_create_default_wifi_sta();

    Wifi_STA();

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi_init_finished.");

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | 
                                       WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT)
        Init_Mqtt();
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