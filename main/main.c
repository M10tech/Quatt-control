/* (c) 2024 M10tech
 * Quatt-control for ESP32
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"
// #include "lcm_api.h"
#include <udplogger.h>

// You must set version.txt file to match github version tag x.y.z for LCM4ESP32 to work

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#define OT0_SEND_PIN  GPIO_NUM_18
#define OT1_SEND_PIN  GPIO_NUM_19
#define BUFF_SIZE   272 //units of uint32=34 bits x2 halfbits x2 uint32 per hb (cause br=4000 and not 2000) x2 slots (stereo)

static i2s_chan_handle_t   tx_chan0;    //I2S tx channel handler
static i2s_chan_handle_t   tx_chan1;    //I2S tx channel handler

#define ONE  0xFFFFFFFF
#define ZERO 0x00000000
#define SETONE(i)   do {dma_buf[i*8+0]=ONE; dma_buf[i*8+1]=ONE; dma_buf[i*8+2]=ONE; dma_buf[i*8+3]=ONE; \
                        dma_buf[i*8+4]=ZERO;dma_buf[i*8+5]=ZERO;dma_buf[i*8+6]=ZERO;dma_buf[i*8+7]=ZERO; \
                        even++;} while(0)
#define SETZERO(i)  do {dma_buf[i*8+0]=ZERO;dma_buf[i*8+1]=ZERO;dma_buf[i*8+2]=ZERO;dma_buf[i*8+3]=ZERO; \
                        dma_buf[i*8+4]=ONE; dma_buf[i*8+5]=ONE; dma_buf[i*8+6]=ONE; dma_buf[i*8+7]=ONE; \
                        } while(0)
void send_OT_frame(int OT_chan, uint32_t payload) {
    int i,j,even=0;
    size_t bytes_loaded;
    uint32_t dma_buf[BUFF_SIZE];
    i2s_chan_handle_t tx_chan=OT_chan?tx_chan1:tx_chan0;
    UDPLUS("CH%dSND:%08lx\n",OT_chan, payload);
    SETONE(0); SETONE(33); //START and STOP
    for (i=30,j=2 ; i>=0 ; i--,j++) { //j==0 is START and j==1 is parity
        if (payload&(1<<i)) SETONE(j); else SETZERO(j);
    }
    if (even%2) SETONE(1); else SETZERO(1); //parity bit
    //transmit the dma_buf once
    ESP_ERROR_CHECK(i2s_channel_preload_data(tx_chan, dma_buf, BUFF_SIZE*sizeof(uint32_t), &bytes_loaded));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan)); //Enable the TX channel
    vTaskDelay(50/portTICK_PERIOD_MS); //message is 34ms so 50ms is enough
    ESP_ERROR_CHECK(i2s_channel_disable(tx_chan)); //Disable the TX channel
}

void i2s_init() { //note that idle voltage is zero and cannot be flipped
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan0, NULL)); //no Rx
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan1, NULL)); //no RX
    i2s_std_config_t tx_std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(4000), //in halfbits per second, but min value higher than 2000 so use 4000
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {.mclk=I2S_GPIO_UNUSED, .bclk=I2S_GPIO_UNUSED, .ws=I2S_GPIO_UNUSED, .din=I2S_GPIO_UNUSED, },
    };
    tx_std_cfg.gpio_cfg.dout=OT0_SEND_PIN;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan0, &tx_std_cfg));
    tx_std_cfg.gpio_cfg.dout=OT1_SEND_PIN;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan1, &tx_std_cfg));
}

void main_task(void *arg) {
    udplog_init(3);
    vTaskDelay(300); //Allow Wi-Fi to connect
    UDPLUS("\n\nQuatt-control\n");

    i2s_init();
    
    while (true) {
        send_OT_frame(0, ZERO);
        send_OT_frame(1, ONE);
        vTaskDelay(400/portTICK_PERIOD_MS);
        send_OT_frame(0, ONE);
        send_OT_frame(1, ZERO);
        vTaskDelay(400/portTICK_PERIOD_MS); 
    }
}    

void app_main(void) {
    printf("app_main-start\n");

    //The code in this function would be the setup for any app that uses wifi which is set by LCM
    //It is all boilerplate code that is also used in common_example code
    esp_err_t err = nvs_flash_init(); // Initialize NVS
    if (err==ESP_ERR_NVS_NO_FREE_PAGES || err==ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); //NVS partition truncated and must be erased
        err = nvs_flash_init(); //Retry nvs_flash_init
    } ESP_ERROR_CHECK( err );

    //TODO: if no wifi setting found, trigger otamain
    
    //block that gets you WIFI with the lowest amount of effort, and based on FLASH
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    esp_netif_config.route_prio = 128;
    esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
    esp_wifi_set_default_wifi_sta_handlers();
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    //end of boilerplate code

    xTaskCreate(main_task,"main",4096,NULL,1,NULL);
    while (true) {
        vTaskDelay(1000); 
    }
    printf("app_main-done\n"); //will never exit here
}
