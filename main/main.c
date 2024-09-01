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
#include "ds18b20.h"
#include "onewire_bus.h"
#include "math.h" //for NAN
#include "esp_ota_ops.h" //for esp_app_get_description
#include "mqtt-client.h"

// You must set version.txt file to match github version tag x.y.z for LCM4ESP32 to work

#define OTNUM         2 //number of OpenTherm channels, max 3 for ESP32P4
#define HEATPUMP      1
#define BOILER        0
#define OT0_SEND_PIN  GPIO_NUM_32
#define OT1_SEND_PIN  GPIO_NUM_33
#define OT2_SEND_PIN  GPIO_NUM_NC
#define OT0_RECV_PIN  GPIO_NUM_34
#define OT1_RECV_PIN  GPIO_NUM_35
#define OT2_RECV_PIN  GPIO_NUM_NC
#define ONE_WIRE_PIN  GPIO_NUM_25

int idx=68; //the domoticz base index
#define PUBLISH(name) do {int n=mqtt_client_publish("{\"idx\":%d,\"nvalue\":0,\"svalue\":\"%.2f\"}", idx+name##_ix, name##_fv); \
                            if (n<0) UDPLUS("MQTT publish of %s failed because %s\n",#name,MQTT_CLIENT_ERROR(n)); \
                           } while(0)
#define tgt_temp1_fv tgt_temp1.value.float_value
#define tgt_temp1_ix 1
#define tgt_temp2_fv tgt_temp2.value.float_value
#define tgt_temp2_ix 2
#define     S1avg_fv S1avg
#define     S1avg_ix 3
#define     S2avg_fv S2avg
#define     S2avg_ix 4
#define  heat_mod_fv heat_mod
#define  heat_mod_ix 5
#define   heat_sp_fv heat_sp
#define   heat_sp_ix 6
#define   burnerW_fv temp[BW]
#define   burnerW_ix 7
#define   returnW_fv temp[RW]
#define   returnW_ix 8
#define  pressure_fv pressure*10.0
#define  pressure_ix 9
#define     S3avg_fv S3avg
#define     S3avg_ix 10
#define    S3long_fv S3long
#define    S3long_ix 11


#define BEAT 10 //in seconds
#define SENSORS 3
#define S1 0 //   salon temp sensor
#define S2 1 //upstairs temp sensor
#define S3 6 // outdoor temp sensor
#define BW 4 //boiler water temp
#define RW 5 //return water temp
#define DW 8 //domestic home water temp
float temp[16]={85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85}; //using id as a single hex digit, then hardcode which sensor gets which meaning
float S1temp[6],S2temp[6],S3temp[6],S1avg,S2avg,S3avg;

#define SINKBUS(time) do {gpio_set_level(ONE_WIRE_PIN,0);vTaskDelay(time/portTICK_PERIOD_MS);gpio_set_level(ONE_WIRE_PIN,1);}while(0)
void temp_task(void *argv) {
    int ids[SENSORS],fail=0,sensor_count;
    onewire_bus_handle_t bus; // install new 1-wire bus
    onewire_bus_config_t bus_config = {.bus_gpio_num = ONE_WIRE_PIN,};
    onewire_bus_rmt_config_t rmt_config = {.max_rx_bytes = 10,}; //1byte ROM command + 8byte ROM number + 1byte device command
    
    if (onewire_new_bus_rmt(&bus_config, &rmt_config, &bus)!=ESP_OK) UDPLUS("onewire_new_bus_rmt failed\n");
    UDPLUS("1-Wire bus installed on GPIO%d\n", ONE_WIRE_PIN);

    ds18b20_device_handle_t ds18b20s[SENSORS];
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t result = ESP_OK;

    while (true) {
        sensor_count=0;
        if (onewire_new_device_iter(bus, &iter)!=ESP_OK) UDPLUS("onewire_new_device_iter failed\n"); //create 1-wire device iterator, which is used for device search
        do {result = onewire_device_iter_get_next(iter, &next_onewire_device);
            if (result == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
                ds18b20_config_t ds_cfg = {};
                if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20s[sensor_count]) == ESP_OK) {
                    // The DS18B20 address 64-bit and my batch turns out family C on https://github.com/cpetrich/counterfeit_DS18B20
                    // I have manually selected that I have unique ids using the second hex digit of CRC
                    ids[sensor_count]=(next_onewire_device.address>>56)&0xF;
                    UDPLUS("Found a DS18B20[%d], address: %016llX, id: %X\n", sensor_count, next_onewire_device.address, ids[sensor_count]);
                    sensor_count++;
                    if (sensor_count >= SENSORS) break;
                }
            }
        } while (result != ESP_ERR_NOT_FOUND);
        UDPLUS("Found %d DS18B20 device(s)\n", sensor_count);
        if (sensor_count>=SENSORS) break;
        
        for (int i=0; i<sensor_count; i++) ds18b20_del_device(ds18b20s[i]);
        SINKBUS(9000); //long reset one-wire bus
        if (onewire_del_device_iter(iter)!=ESP_OK) UDPLUS("onewire_del_device_iter failed\n");
        if (fail++>50) {
            UDPLUS("restarting because can't find enough sensors\n");
//             mqtt_client_publish("{\"idx\":%d,\"nvalue\":4,\"svalue\":\"Heater No Sensors\"}", idx);
            vTaskDelay(3000/portTICK_PERIOD_MS);
            esp_restart();
        }
        vTaskDelay(BEAT*1000/portTICK_PERIOD_MS);
    } //break out when all SENSORS are found

    float temperature;
    int indx=0;
    while (1) {
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); //the timer loop triggers this 3 x in 10 seconds
        if ((result=ds18b20_trigger_temperature_conversion(ds18b20s[indx])) == ESP_OK) { //has 800ms delay built into the conversion...
            if ((result=ds18b20_get_temperature(ds18b20s[indx], &temperature)) == ESP_OK) { //get temperature from sensors one by one
                temp[ids[indx]] = temperature;
                //UDPLUS("temperature read from DS18B20[%d]: %.4fC\n", indx, temperature);
//                 int n=mqtt_client_publish("{\"idx\":%d,\"nvalue\":0,\"svalue\":\"%.3f\"}", 345, temperature);
//                 UDPLUS("n=%d\n",n);
            } else {
                UDPLUS("ds18b20_get_temperature for id:%d failed %x: %s\n",ids[indx],result,esp_err_to_name(result));
                temp[ids[indx]] = NAN;
                SINKBUS(9000); //long reset one-wire bus
                UDPLUS("after long reset bus\n"); //without depriving a stuck sensor from power for a few seconds, it will stay stuck
            }
        } else {
            UDPLUS("ds18b20_trigger_temperature_conversion for id:%d failed %x: %s\n",ids[indx],result,esp_err_to_name(result));
            temp[ids[indx]] = NAN;
            SINKBUS(9000); //long reset one-wire bus
        }
        indx++; if (indx>=sensor_count) indx=0;
    }
}


int     heat_on=0;
int     stateflg=0,errorflg=0;
float   curr_mod=0,pressure=0;


static int                 ot_send_pin[OTNUM],ot_recv_pin[OTNUM];
static bool                ot_recv_enable[OTNUM];
static i2s_chan_handle_t   tx_chan[OTNUM];    //I2S tx channel handlers
#define BUFF_SIZE   272 //units of uint32=34 bits x2 halfbits x2 uint32 per hb (cause br=4000 and not 2000) x2 slots (stereo)
#define ONE  0xFFFFFFFF
#define ZERO 0x00000000
#define SETONE(i)   do {dma_buf[i*8+0]=ONE; dma_buf[i*8+1]=ONE; dma_buf[i*8+2]=ONE; dma_buf[i*8+3]=ONE; \
                        dma_buf[i*8+4]=ZERO;dma_buf[i*8+5]=ZERO;dma_buf[i*8+6]=ZERO;dma_buf[i*8+7]=ZERO; \
                        even++;} while(0)
#define SETZERO(i)  do {dma_buf[i*8+0]=ZERO;dma_buf[i*8+1]=ZERO;dma_buf[i*8+2]=ZERO;dma_buf[i*8+3]=ZERO; \
                        dma_buf[i*8+4]=ONE; dma_buf[i*8+5]=ONE; dma_buf[i*8+6]=ONE; dma_buf[i*8+7]=ONE; \
                        } while(0)
void send_OT_frame(int ch, uint32_t payload) {
    int i,j,even=0;
    size_t bytes_loaded;
    uint32_t dma_buf[BUFF_SIZE];
    if (ch>=OTNUM) return;
    UDPLUS("CH%dSND:%08lx ",ch, payload);
    SETONE(0); SETONE(33); //START and STOP
    for (i=30,j=2 ; i>=0 ; i--,j++) { //j==0 is START and j==1 is parity
        if (payload&(1<<i)) SETONE(j); else SETZERO(j);
    }
    if (even%2) SETONE(1); else SETZERO(1); //parity bit
    //transmit the dma_buf once
    if (i2s_channel_preload_data(tx_chan[ch], dma_buf, BUFF_SIZE*sizeof(uint32_t), &bytes_loaded)!=ESP_OK) UDPLUS("i2s_channel_preload_data failed\n");
    if (i2s_channel_enable(tx_chan[ch])!=ESP_OK) UDPLUS("i2s_channel_enable failed\n"); //Enable the TX channel
    vTaskDelay(30/portTICK_PERIOD_MS); //message is 34ms
    ot_recv_enable[ch]=true;
    vTaskDelay(20/portTICK_PERIOD_MS); //message is 34ms so total 50ms is enough
    if (i2s_channel_disable(tx_chan[ch])!=ESP_OK) UDPLUS("i2s_channel_disable failed\n"); //Disable the TX channel
}

#define  IDLE  0
#define  START 1
#define  RECV  2
static QueueHandle_t gpio_evt_queue[OTNUM];
static int      resp_idx[OTNUM], rx_state[OTNUM];
static uint32_t response[OTNUM];
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    int ch=-1;
    for (int i=0;i<OTNUM;i++) if (gpio_num==ot_recv_pin[i]) ch=i;
    if (ch==-1) return; //this should not be possible...
    if (ot_recv_enable[ch]==false) return; //don't waste ISR CPU on inactive channel

    static uint64_t before[OTNUM]={};
    uint64_t now[OTNUM],delta[OTNUM];
    int even[OTNUM],read[OTNUM];

    now[ch]=esp_timer_get_time();delta[ch]=now[ch]-before[ch];
    even[ch]=0, read[ch]=gpio_get_level(ot_recv_pin[ch]);
    if (rx_state[ch]==IDLE) {
        if (!read[ch]) return;
        rx_state[ch]=START;
        before[ch]=now[ch];
    } else if (rx_state[ch]==START) {
        if (400<delta[ch] && delta[ch]<650 && (!read[ch])) {
            resp_idx[ch]=0; response[ch]=0; even[ch]=0;
            rx_state[ch]=RECV;
        } //else error state but might be a new start, so just stay in this state
        before[ch]=now[ch];
    } else if (rx_state[ch]==RECV)  {
        if (900<delta[ch] && delta[ch]<1150) {
            if (resp_idx[ch]<32) { //TODO: should also read stop bit, right?
                response[ch]=(response[ch]<<1)|(!read[ch]);
                if (!read[ch]) even[ch]++;
                resp_idx[ch]++;
                before[ch]=now[ch];
            } else { //received all 32 bits
                if (even[ch]%2==0) {
                    if (response[ch]&0x0f000000) resp_idx[ch]=-2; //signal issue reserved bits not zero
                    else {
                        response[ch]&=0x7fffffff; //mask parity bit
                        xQueueSendToBackFromISR(gpio_evt_queue[ch], (void*)&response[ch], NULL);
                    }
                } else resp_idx[ch]=-1; //signal issue parity failure
                rx_state[ch]=IDLE;
                ot_recv_enable[ch]=false;
            }
        } else if (delta[ch]>=1150) { //error state
            if (!read[ch]) rx_state[ch]=IDLE;
            else {rx_state[ch]=START; before[ch]=now[ch];}
        } //else do nothing so before0+=500 and next transit is a databit
    }
}


#define CalcAvg(Sx) do {            Sx##temp[5]=Sx##temp[4];Sx##temp[4]=Sx##temp[3]; \
            Sx##temp[3]=Sx##temp[2];Sx##temp[2]=Sx##temp[1];Sx##temp[1]=Sx##temp[0]; \
            if ( !isnan(temp[Sx]) && temp[Sx]!=85 )         Sx##temp[0]=temp[Sx];    \
            Sx##avg=(Sx##temp[0]+Sx##temp[1]+Sx##temp[2]+Sx##temp[3]+Sx##temp[4]+Sx##temp[5])/6.0; \
        } while(0)
static TaskHandle_t tempTask = NULL;
int timeIndex=0,pump_off_time=0;
// int switch_state=0,retrigger=0;
// int push=-2;
TimerHandle_t xTimer;
void vTimerCallback( TimerHandle_t xTimer ) {
    uint32_t seconds = ( uint32_t ) pvTimerGetTimerID( xTimer );
    vTimerSetTimerID( xTimer, (void*)seconds+1); //136 year to loop
    uint32_t message;
    int switch_on=0;
//     if (gpio_read(SWITCH_PIN)) switch_state--; else switch_state++; //pin is low when switch is on
//     if (switch_state<0) switch_state=0;
//     if (switch_state>3) switch_state=3;
//     switch_on=switch_state>>1;
//     //TODO read recv pin and if it is a ONE, we have an OpenTherm error state
    UDPLUS("St%d Sw%d @%ld ",timeIndex,switch_on,seconds);
//     if (timeIndex==3) { // allow 3 seconds for two automation rules to succeed and repeat every 10 seconds
//         if (pump_off_time) pump_off_time-=10;
//         if (tgt_heat1.value.int_value==2) { //Pump Off rule confirmed
//             cur_heat2.value.int_value= 1;   //confirm we are heating upstairs
//             homekit_characteristic_notify(&cur_heat2,HOMEKIT_UINT8(cur_heat2.value.int_value));
//             tgt_heat1.value.int_value= 3;   //set heater 1 mode back to auto and be ready for another trigger
//             homekit_characteristic_notify(&tgt_heat1,HOMEKIT_UINT8(tgt_heat1.value.int_value)); //TODO: racecondition?
//             pump_off_time=180; //seconds
//             heat_on=1;
//         }
//         if (cur_heat2.value.int_value==2) {//send reminder notify
//             cur_heat2.value.int_value= 0; //to assure it is considered as a new value we first set it to off
//             homekit_characteristic_notify(&cur_heat2,HOMEKIT_UINT8(cur_heat2.value.int_value)); //and notify
//             retrigger=1;
//             if (pump_off_time>10) heat_on=1; //still time left
//         }
//     }
//     if (retrigger && timeIndex==8) { retrigger=0; //retrigger needed 5 seconds offset
//         cur_heat2.value.int_value= 2;
//         homekit_characteristic_notify(&cur_heat2,HOMEKIT_UINT8(cur_heat2.value.int_value));
//     }
    if (timeIndex%3==1) { //trigger at 1, 4 and 7s so more time to finish temp reading and each sensor is read once a BEAT
        xTaskNotifyGive( tempTask ); //temperature measurement start
        vTaskDelay(1); //TODO: is this needed: prevent interference between OneWire and OT-receiver
    }
    switch (timeIndex) { //send commands BOILER
        case 0: //measure temperature
            message=0x00190000; //25 read boiler water temperature
            break;
        case 1: //execute heater decisions
//             if (tgt_heat2.value.int_value==1) { //use on/off switching thermostat
                   message=0x10014b00; //75 deg //1  CH setpoint in deg C
//             } else if (tgt_heat2.value.int_value==3) { //run heater algoritm for floor heating
//                    message=0x10010000|(uint32_t)(heat_sp*256);
//             } else message=0x10010000|(uint32_t)(tgt_temp1.value.float_value*2-1)*256; //range from 19 - 75 deg
            break;
        case 2:
            message=0x100e1100;
//             if (tgt_heat2.value.int_value==1) { //use on/off switching thermostat
//                 if (tgt_temp2.value.float_value>17) message=0x100e0000|(uint32_t)(4*tgt_temp2.value.float_value-52)*256; //18-100%
//                 else message=0x100e1100; //17%
//             } else   message=0x100e6400; //100% //14 max modulation level
            break;
        case 3:
            message=0x00000200;
//             if (tgt_heat2.value.int_value==1) { //use on/off switching thermostat
//                    message=0x00000200|(switch_on?0x100:0x000); //0  enable CH and DHW
//             } else if (tgt_heat2.value.int_value==3) { //run heater algoritm for floor heating
//                    message=0x00000200|(  heat_on?0x100:0x000);
//             } else message=0x00000200|(tgt_heat1.value.int_value<<8);
            break; 
        case 4:
//             if (tgt_heat2.value.int_value==2) { //test BLOR
//                    message=0x10040100; //4.1  BoilerLockOutReset
//             } else message=0x00380000; //56 DHW setpoint write
            message=0x00380000;
            break;
        case 5: message=0x00050000; break; //5  app specific fault flags
        case 6: message=0x00120000; break; //18 CH water pressure
        case 7: message=0x001a0000; break; //26 DHW temp
        case 8: message=0x001c0000; break; //28 return water temp
        case 9: message=0x00110000; break; //17 rel mod level
        default: break;
    }
    send_OT_frame(BOILER, message); //send message to BOILER OT receiver
    //since we want to run two OT channels every second, we only wait for response for 400ms
    if (xQueueReceive(gpio_evt_queue[BOILER], &(message), (TickType_t)400/portTICK_PERIOD_MS) == pdTRUE) {
        UDPLUS("CH%dRSP:%08lx\n",BOILER,message);
        switch (timeIndex) { //check answers
            case 0: temp[BW]=(float)(message&0x0000ffff)/256; break;
            case 3:
//                 heat_mod=0.0;
                stateflg=(message&0x0000007f);
//                 if ((stateflg&0xa)==0xa) {
//                     heat_mod=1.0; //at this point it is a multiplier
//                     cur_heat1.value.int_value=pump_off_time?2:1; //present heater on but pump off as cur_heat1=2 COOL
//                 } else cur_heat1.value.int_value=0;
//                 homekit_characteristic_notify(&cur_heat1,HOMEKIT_UINT8(cur_heat1.value.int_value));
                break;
            case 5: errorflg=       (message&0x00003f00)/256; break;
            case 6: pressure=(float)(message&0x0000ffff)/256; break;
            case 7: temp[DW]=(float)(message&0x0000ffff)/256; break;
            case 8: temp[RW]=(float)(message&0x0000ffff)/256; break;
            case 9:
                curr_mod=(float)(message&0x0000ffff)/256;
//                 if (curr_mod==70.0) heat_mod=0.0; else heat_mod*=curr_mod;
                break;
            default: break;
        }
    } else { //TODO: make this a function
        UDPLUS("!!! CH%d_NO_RSP: resp_idx=%d rx_state=%d response=%08lx\n",BOILER,resp_idx[BOILER], rx_state[BOILER], response[BOILER]);
        resp_idx[BOILER]=0, rx_state[BOILER]=IDLE, response[BOILER]=0; //TODO: is this critical for proper operation?
        ot_recv_enable[BOILER]=false;
    }
    
    if (!timeIndex) {
        CalcAvg(S1); CalcAvg(S2); CalcAvg(S3);
    }
//     
//     //errorflg=(seconds/600)%2; //test trick to change outcome every 10 minutes
//     if (seconds%60==5) {
//         if (errorflg) { //publish a ORANGE (3) ALERT on domoticz
//             if (push>0) {
//                 int n=mqtt_client_publish("{\"idx\":%d,\"nvalue\":3,\"svalue\":\"Heater ERR: 0x%02X\"}", idx, errorflg);
//                 if (n<0) UDPLUS("MQTT publish of ALERT failed because %s\n",MQTT_CLIENT_ERROR(n)); else push--;
//                 if (push==0) push=-2;
//             }
//         } else { //publish a GREY (0) clean ALERT on domoticz
//             if (push<0) {
//                 int n=mqtt_client_publish("{\"idx\":%d,\"nvalue\":0,\"svalue\":\"Heater OK (again) %d\"}", idx, seconds/60);
//                 if (n<0) UDPLUS("MQTT publish of ALERT failed because %s\n",MQTT_CLIENT_ERROR(n)); else push++;
//                 if (push==0) push=3;            
//             }
//         }
//     }
//  if (seconds%3600==3599) { //force a alive report every hour
//      int n=mqtt_client_publish("{\"idx\":%d,\"nvalue\":0,\"svalue\":\"Heater alive %d\"}", idx, seconds/60);    
//  }
//  
    if (seconds%60==50) { //allow 6 temperature measurments to make sure all info is loaded
PUBLISH(S1avg);
PUBLISH(S2avg);
PUBLISH(S3avg);
PUBLISH(burnerW);
PUBLISH(returnW);
PUBLISH(pressure);
//         heat_on=0;
//         cur_heat2.value.int_value=heater(seconds); //sets heat_sp and returns heater result, 0, 1 or 2
//         if (cur_heat2.value.int_value==2 && pump_off_time>90) cur_heat2.value.int_value=1; //do not retrigger rules yet
//         if (cur_heat2.value.int_value==1) heat_on=1;
//         homekit_characteristic_notify(&cur_heat2,HOMEKIT_UINT8(cur_heat2.value.int_value));
    }

    if (timeIndex==3) {
        UDPLUS("S1=%7.4f S2=%7.4f S3=%7.4f PR=%4.2f DW=%4.1f ERR=%02x RW=%4.1f BW=%4.1f POT=%3d ON=%d MOD=%02.0f ST=%02x\n", \
           temp[S1],temp[S2],temp[S3],pressure,temp[DW],errorflg,temp[RW],temp[BW],pump_off_time,heat_on,curr_mod,stateflg);
    }

    timeIndex++; if (timeIndex==BEAT) timeIndex=0;
} //this is a timer that restarts every 1 second


#define ESP_INTR_FLAG_DEFAULT  0
#define ESP_INTR_FLAG_MINE  ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM //| ESP_INTR_FLAG_EDGE
void OT_recv_init() {
    uint64_t gpio_input_pin_sel=0;
    for (int i=0;i<OTNUM;i++) {
        gpio_evt_queue[i] = xQueueCreate(1, sizeof(uint32_t));
        gpio_input_pin_sel=gpio_input_pin_sel | (1ULL<< ot_recv_pin[i]);
    }
    gpio_config_t io_conf = {}; //zero-initialize the config structure.

    io_conf.pin_bit_mask = gpio_input_pin_sel; //bit mask of the pins
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0; //disable pull-up mode (not sure this is needed, but we have hardware pulldown)
    io_conf.pull_down_en = 1; //to cover temporarily floating input
    gpio_config(&io_conf); //configure GPIO with the given settings

    gpio_install_isr_service(ESP_INTR_FLAG_MINE); //install gpio isr service
    for (int i=0;i<OTNUM;i++) {
        gpio_isr_handler_add(ot_recv_pin[i], gpio_isr_handler, (void*) ot_recv_pin[i]); //hook isr handler for specific gpio pin
    }
}

void OT_send_init() { //note that idle voltage is zero and cannot be flipped
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    i2s_std_config_t tx_std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(4000), //in halfbits per second, but min value higher than 2000 so use 4000
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {.mclk=I2S_GPIO_UNUSED, .bclk=I2S_GPIO_UNUSED, .ws=I2S_GPIO_UNUSED, .din=I2S_GPIO_UNUSED, },
    };
    for (int i=0;i<OTNUM;i++) {
        if (i2s_new_channel(&tx_chan_cfg, &tx_chan[i], NULL)!=ESP_OK) UDPLUS("i2s_new_channel failed\n"); //no Rx
        tx_std_cfg.gpio_cfg.dout=ot_send_pin[i];
        if (i2s_channel_init_std_mode(tx_chan[i], &tx_std_cfg)!=ESP_OK) UDPLUS("i2s_channel_init_std_mode failed\n");
    }
}

void OT_init() {
    for (int i=0;i<OTNUM;i++) {
        if (i==0) {ot_recv_pin[i]=OT0_RECV_PIN; ot_send_pin[i]=OT0_SEND_PIN;}
        if (i==1) {ot_recv_pin[i]=OT1_RECV_PIN; ot_send_pin[i]=OT1_SEND_PIN;}
        if (i==2) {ot_recv_pin[i]=OT2_RECV_PIN; ot_send_pin[i]=OT2_SEND_PIN;}
        rx_state[i]=IDLE; resp_idx[i]=0; response[i]=0;
    }
    OT_recv_init();
    OT_send_init();
}

mqtt_config_t mqttconf=MQTT_DEFAULT_CONFIG;
void MQTT_init() {
    mqttconf.host="192.168.178.5";
    mqttconf.user="test";
    mqttconf.pass="test";
//     mqttconf.queue_size=20;
    mqttconf.msg_len   =64; //to fit the ALERT
    mqtt_client_init(&mqttconf);
}

void main_task(void *arg) {
    udplog_init(3);
    vTaskDelay(300); //Allow Wi-Fi to connect
    UDPLUS("\n\nQuatt-control %s\n",esp_app_get_description()->version);
    
    MQTT_init();
    OT_init();
    S1temp[0]=22;S2temp[0]=22;
    xTaskCreatePinnedToCore(temp_task,"Temp", 4096, NULL, 1, &tempTask,1); //TODO: check if really needed to survive stuck sensor
    xTimer=xTimerCreate( "Timer", 1000/portTICK_PERIOD_MS, pdTRUE, (void*)0, vTimerCallback);
    xTimerStart(xTimer, 0);
    //vTaskDelay(1000/portTICK_PERIOD_MS); //Allow inits to settle
    //esp_intr_dump(NULL);
    
    while (true) {
        vTaskDelay(1000/portTICK_PERIOD_MS); 
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

    xTaskCreatePinnedToCore(main_task,"main",4096,NULL,1,NULL,1); //essential to use core 1 for GPIO interrupts
    while (true) {
        vTaskDelay(1000); 
    }
    //TODO: solve this: do not kill this, else the rest goes with it 
    printf("app_main-done\n"); //will never exit here
}
