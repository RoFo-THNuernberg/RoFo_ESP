#include <cstdlib>
#include <string>
#include <cstring>
#include "led_strip_component.h"
//#include "esp_log.h"
#include "esp_attr.h"
//#include "driver/rmt.h"
#include <sys/cdefs.h>
#include "sdkconfig.h"

static const char *TAG = "ws2815";


#define WS2815_T0H_NS (350)
#define WS2815_T0L_NS (1000)
#define WS2815_T1H_NS (1000)
#define WS2815_T1L_NS (350)
#define WS2815_RESET_US (280)

static uint32_t ws2815_t0h_ticks = 0;
static uint32_t ws2815_t1h_ticks = 0;
static uint32_t ws2815_t0l_ticks = 0;
static uint32_t ws2815_t1l_ticks = 0;

#define RMT_TX_CHANNEL RMT_CHANNEL_0


static void IRAM_ATTR ws2815_rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size,
        size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    if (src == NULL || dest == NULL) {
        *translated_size = 0;
        *item_num = 0;
        return;
    }
    const rmt_item32_t bit0 = {{{ ws2815_t0h_ticks, 1, ws2815_t0l_ticks, 0 }}}; //Logical 0
    const rmt_item32_t bit1 = {{{ ws2815_t1h_ticks, 1, ws2815_t1l_ticks, 0 }}}; //Logical 1
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t *pdest = dest;
    while (size < src_size && num < wanted_num) {
        for (int i = 0; i < 8; i++) {
            // MSB first
            if (*psrc & (1 << (7 - i))) {
                pdest->val =  bit1.val;
            } else {
                pdest->val =  bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translated_size = size;
    *item_num = num;
}

esp_err_t led_strip_s::set_pixel(uint32_t index, uint32_t red, uint32_t green, uint32_t blue)  // SetColor Funktion
{   
    if(!(index < max_leds))
    {
        ESP_LOGE(TAG, "index out of the maximum number of leds! " );
        return ESP_FAIL;
    }
    uint32_t start = index * 3;
    // In the order of GRB
    buffer[start + 0] = green & 0xFF;
    buffer[start + 1] = red & 0xFF;
    buffer[start + 2] = blue & 0xFF;
    return ESP_OK;
}


esp_err_t led_strip_s::refresh(uint32_t timeout_ms)
{
    if(!(rmt_write_sample(rmt_channel, buffer, max_leds * 3, true) == ESP_OK))
    {
        ESP_LOGE(TAG, "transmit RMT samples failed " );
        return ESP_FAIL;
    }
    return rmt_wait_tx_done(rmt_channel, pdMS_TO_TICKS(timeout_ms));

}

esp_err_t led_strip_s::clear(uint32_t timeout_ms)
{
    memset(buffer, 0 , max_leds * 3);
    return refresh(timeout_ms);
}


led_strip_s::~led_strip_s()
{
    delete[] buffer;
}


esp_err_t led_strip_s::led_strip_new_rmt(const led_strip_config_t *config)
{   
    rmt_channel = config->dev;       
    max_leds = config->max_leds;


    if(!config)
    {
        ESP_LOGE(TAG, "configuration can't be null");
        return ESP_FAIL;
    }
    // 24 bits per leds TODO
    uint32_t buffer_size = max_leds * 3;
    printf("%d BUFFER GROESSE\n", buffer_size);
    buffer = new uint8_t[buffer_size];
    if(!buffer)
    {
        ESP_LOGE(TAG, "request memory for ws2812 failed");
        return ESP_FAIL;
    }

    uint32_t counter_clk_hz = 0;

    if(rmt_get_counter_clock(config->dev, &counter_clk_hz)) 
    {
        ESP_LOGE(TAG, "get rmt counter clock failed");
        return ESP_FAIL;
    } 
    

    //ns -> ticks
    float ratio = (float)counter_clk_hz / 1e9;

    ws2815_t0h_ticks = (uint32_t)(ratio * WS2815_T0H_NS);
    ws2815_t0l_ticks = (uint32_t)(ratio * WS2815_T0L_NS);
    ws2815_t1h_ticks = (uint32_t)(ratio * WS2815_T1H_NS);
    ws2815_t1l_ticks = (uint32_t)(ratio * WS2815_T1L_NS);  

    //set ws2815 to rmt adapter
    rmt_translator_init(config->dev, ws2815_rmt_adapter);   


   


    return ESP_OK;
}

void led_strip_s::init()
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)CONFIG_LED_RMT_TX_GPIO, RMT_TX_CHANNEL);
    config.clk_div = 2;

    //led_strip_s::max_leds = CONFIG_LED_STRIP_LED_NUMBER; // WOher nochmal?

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    
    led_strip_config_t strip_config;
    strip_config.max_leds = CONFIG_LED_STRIP_LED_NUMBER;
    strip_config.dev = config.channel;

    ESP_ERROR_CHECK(led_strip_new_rmt(&strip_config));

    // Clear LED strip (turn off all LEDs)
    clear(100);


}



 