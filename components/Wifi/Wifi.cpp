#include "Wifi.h"

#include <cstring>

namespace WIFI 
{   
    #define TAG "Wifi"

    #define WIFI_SSID CONFIG_WIFI_SSID
    #define WIFI_PASSWORD CONFIG_WIFI_PASSWORD
    #define WIFI_MAX_RETRY CONFIG_WIFI_MAX_RETRY

    #define WIFI_CONNECTED_BIT BIT0
    #define WIFI_FAIL_BIT BIT1
    
    EventGroupHandle_t Wifi::_wifi_event_group{};
    const wifi_init_config_t Wifi::_wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t Wifi::_wifi_config = {};

    void Wifi::_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
    {
        static int conn_retry_num = 0;

        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
        {
            esp_wifi_connect();
        } 
        else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
            if (conn_retry_num < WIFI_MAX_RETRY) {
                esp_wifi_connect();
                conn_retry_num++;
                ESP_LOGI(TAG, "retry to connect to the AP");
            } 
            else 
            {
                xEventGroupSetBits(_wifi_event_group, WIFI_FAIL_BIT);
            }
            ESP_LOGI(TAG,"connect to the AP failed");
        } 
        else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
        {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
            conn_retry_num = 0;
            xEventGroupSetBits(_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }


    esp_err_t Wifi::init() 
    {   

        ESP_LOGI(TAG, "Initializing Wifi...");

        esp_err_t status{ESP_OK};

        _wifi_event_group = xEventGroupCreate();

        status = esp_netif_init();

        if(ESP_OK == status)
            status = esp_event_loop_create_default();

        if(ESP_OK == status) 
        {
            esp_netif_create_default_wifi_sta();
            status = esp_wifi_init(&_wifi_init_config); 
        }

        if(ESP_OK == status)
            status = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, (esp_event_handler_t)&_event_handler, NULL, NULL);

        if(ESP_OK == status)
            status = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, (esp_event_handler_t)&_event_handler, NULL, NULL);
        
        if(ESP_OK == status)
            status = esp_wifi_set_mode(WIFI_MODE_STA);

        memcpy(_wifi_config.sta.ssid, WIFI_SSID, sizeof(WIFI_SSID));
        memcpy(_wifi_config.sta.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
        _wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        _wifi_config.sta.pmf_cfg.capable = true;
        _wifi_config.sta.pmf_cfg.required = false;

        if(ESP_OK == status)
            status = esp_wifi_set_config(WIFI_IF_STA, &_wifi_config);

        if(ESP_OK == status)
            status = esp_wifi_start();

        if(ESP_OK == status)
            ESP_LOGI(TAG, "Wifi initialization successful!");
        
        return status;
    }

    esp_err_t Wifi::begin() {

        esp_err_t status = ESP_OK;

        ESP_LOGI(TAG, "Waiting for connection...");
        EventBits_t event_bits = xEventGroupWaitBits(_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

        if (event_bits & WIFI_CONNECTED_BIT)
            ESP_LOGI(TAG, "connected to ap SSID:%s", WIFI_SSID);

        else if (event_bits & WIFI_FAIL_BIT) 
        {
            ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
            status = ESP_FAIL;

        } 
        else 
        {
            ESP_LOGE(TAG, "UNEXPECTED EVENT");
            status = ESP_FAIL;
        }

        return status;
    }
}