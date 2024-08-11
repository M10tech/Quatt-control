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
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// You must set version.txt file to match github version tag x.y.z for LCM4ESP32 to work

#define OT0_SEND_PIN  GPIO_NUM_32
#define OT1_SEND_PIN  GPIO_NUM_33
#define OT0_RECV_PIN  GPIO_NUM_34
#define OT1_RECV_PIN  GPIO_NUM_35


static i2s_chan_handle_t   tx_chan0;    //I2S tx channel handler
static i2s_chan_handle_t   tx_chan1;    //I2S tx channel handler

#define BUFF_SIZE   272 //units of uint32=34 bits x2 halfbits x2 uint32 per hb (cause br=4000 and not 2000) x2 slots (stereo)
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


#define  READY 0
#define  START 1
#define  RECV  2
static QueueHandle_t gpio_evt_queue0;
static QueueHandle_t gpio_evt_queue1;
static int      resp_idx0=0, rx_state0=READY;
static int      resp_idx1=0, rx_state1=READY;
static uint32_t response0=0;
static uint32_t response1=0;
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    static uint64_t before0=0;
    static uint64_t before1=0;
    uint64_t now,delta;
    int even,inv_read;
    switch (gpio_num) {
        case OT0_RECV_PIN: {
            now=esp_timer_get_time(),delta=now-before0;
            even=0, inv_read=gpio_get_level(OT0_RECV_PIN);//note that gpio_read gives the inverted value of the symbol
            if (rx_state0==READY) {
                if (inv_read) return;
                rx_state0=START;
                before0=now;
            } else if (rx_state0==START) {
                if (400<delta && delta<650 && inv_read) {
                    resp_idx0=0; response0=0; even=0;
                    rx_state0=RECV;
                } //else error state but might be a new start, so just stay in this state
                before0=now;
            } else if (rx_state0==RECV)  {
                if (900<delta && delta<1150) {
                    if (resp_idx0<32) {
                        response0=(response0<<1)|inv_read;
                        if (inv_read) even++;
                        resp_idx0++;
                        before0=now;
                    } else {
                        if (even%2==0) {
                            if (response0&0x0f000000) resp_idx0=-2; //signal issue reserved bits not zero
                            else {
                                response0&=0x7fffffff; //mask parity bit
                                xQueueSendToBackFromISR(gpio_evt_queue0, (void*)&response0, NULL);
                            }
                        } else resp_idx0=-1; //signal issue parity failure
                        rx_state0=READY;
                    }
                } else if (delta>=1150) { //error state
                    if (inv_read) rx_state0=READY;
                    else {rx_state0=START; before0=now;}
                } //else do nothing so before0+=500 and next transit is a databit
            }
            
            break;}
        case OT1_RECV_PIN: {
            now=esp_timer_get_time(),delta=now-before1;
            even=0, inv_read=gpio_get_level(OT1_RECV_PIN);//note that gpio_read gives the inverted value of the symbol
            if (rx_state1==READY) {
                if (inv_read) return;
                rx_state1=START;
                before1=now;
            } else if (rx_state1==START) {
                if (400<delta && delta<650 && inv_read) {
                    resp_idx1=0; response1=0; even=0;
                    rx_state1=RECV;
                } //else error state but might be a new start, so just stay in this state
                before1=now;
            } else if (rx_state1==RECV)  {
                if (900<delta && delta<1150) {
                    if (resp_idx1<32) {
                        response1=(response1<<1)|inv_read;
                        if (inv_read) even++;
                        resp_idx1++;
                        before1=now;
                    } else {
                        if (even%2==0) {
                            if (response1&0x0f000000) resp_idx1=-2; //signal issue reserved bits not zero
                            else {
                                response1&=0x7fffffff; //mask parity bit
                                xQueueSendToBackFromISR(gpio_evt_queue1, (void*)&response1, NULL);
                            }
                        } else resp_idx1=-1; //signal issue parity failure
                        rx_state1=READY;
                    }
                } else if (delta>=1150) { //error state
                    if (inv_read) rx_state1=READY;
                    else {rx_state1=START; before1=now;}
                } //else do nothing so before1+=500 and next transit is a databit
            }
            break;}
        default:
            break;
    }
}

void task0(void *arg) {
    uint32_t message;
    while (true) {
        if (xQueueReceive(gpio_evt_queue0, &(message), (TickType_t)850/portTICK_PERIOD_MS) == pdTRUE) {
            UDPLUS("OT0: %08lx\n",message);
        } else {
            UDPLUS("!!! NO_RSP OT0:  resp_idx=%d rx_state=%d response=%08lx\n",resp_idx0, rx_state0, response0);
            resp_idx0=0, rx_state0=READY, response0=0;
        }
    }
}

void task1(void *arg) {
    uint32_t message;
    while (true) {
        if (xQueueReceive(gpio_evt_queue1, &(message), (TickType_t)850/portTICK_PERIOD_MS) == pdTRUE) {
            UDPLUS("OT1:%08lx\n",message);
        } else {
            UDPLUS("!!! NO_RSP OT1: resp_idx=%d rx_state=%d response=%08lx\n",resp_idx1, rx_state1, response1);
            resp_idx1=0, rx_state1=READY, response1=0;
        }
    }
}

#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL  ((1ULL<<OT0_RECV_PIN) | (1ULL<<OT1_RECV_PIN))
void OT_recv_init() {
    gpio_evt_queue0 = xQueueCreate(10, sizeof(uint32_t));
    gpio_evt_queue1 = xQueueCreate(10, sizeof(uint32_t));
        
    gpio_config_t io_conf = {}; //zero-initialize the config structure.

    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; //bit mask of the pins
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0; //disable pull-up mode (not sure this is needed, but we have hardware pulldown)
    gpio_config(&io_conf); //configure GPIO with the given settings

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT); //install gpio isr service
    gpio_isr_handler_add(OT0_RECV_PIN, gpio_isr_handler, (void*) OT0_RECV_PIN); //hook isr handler for specific gpio pin
    gpio_isr_handler_add(OT1_RECV_PIN, gpio_isr_handler, (void*) OT1_RECV_PIN); //hook isr handler for specific gpio pin
        
    xTaskCreate(task0,"task0",4096,NULL,1,NULL);
    xTaskCreate(task1,"task1",4096,NULL,1,NULL);
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

    OT_recv_init();
    i2s_init();
    
    while (true) {
        send_OT_frame(0, 0x00120000); //18 CH water pressure
        vTaskDelay(950/portTICK_PERIOD_MS);
        send_OT_frame(0, 0x001a0000); //26 DHW temp
        vTaskDelay(950/portTICK_PERIOD_MS); 
    }
    //TODO: solve this: do not kill this, else the rest goes with it 
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
