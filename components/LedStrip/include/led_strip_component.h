#pragma once

#include <stdint.h>
#include "driver/rmt.h"
#include "esp_log.h"



typedef struct led_strip_s led_strip_t;

typedef void *led_strip_dev_t;

struct led_strip_config_t{
    uint32_t max_leds;   /*!< Maximum LEDs in a single strip */
    rmt_channel_t dev; /*!< LED strip device (e.g. RMT channel, PWM channel, etc) */  
} ;

struct led_strip_s {

    uint32_t max_leds;
    rmt_channel_t rmt_channel;
    //uint32_t strip_len;
    uint8_t *buffer;
   
    ~led_strip_s();

    esp_err_t set_pixel(uint32_t index, uint32_t red, uint32_t green, uint32_t blue);       // Funktion
   
    esp_err_t refresh(uint32_t timeout_ms);                                                 // Funktion

    esp_err_t clear(uint32_t timeout_ms);                                                   // Funktion

    esp_err_t del();                                                                        // Funktion

    esp_err_t led_strip_new_rmt(const led_strip_config_t *config);

    void init();
};


#define LED_STRIP_DEFAULT_CONFIG(number, dev_hdl) \
    {                                             \
        .max_leds = number,                       \
        .dev = dev_hdl,                           \
    }





