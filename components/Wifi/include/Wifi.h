#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_err.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"


class Wifi {
    public:
        Wifi() {};
        esp_err_t init();
        esp_err_t begin();

    private:
        static void _event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

        static EventGroupHandle_t _wifi_event_group;
        const static wifi_init_config_t _wifi_init_config;
        static wifi_config_t _wifi_config;
};