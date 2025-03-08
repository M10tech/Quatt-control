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
#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include "esp_netif_sntp.h"
#include <udplogger.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "ds18b20.h"
#include "onewire_bus.h"
#include "math.h" //for NAN
#include "esp_ota_ops.h" //for esp_app_get_description
#include "mqtt-client.h"
#include "ping/ping_sock.h"

// You must set version.txt file to match github version tag x.y.z for LCM4ESP32 to work

#define OTNUM         2 //number of OpenTherm channels, max 3 for ESP32P4
#define BOILER        0
#define HEATPUMP      1
#define OT0_SEND_PIN  GPIO_NUM_32
#define OT1_SEND_PIN  GPIO_NUM_33
#define OT2_SEND_PIN  GPIO_NUM_NC
#define OT0_RECV_PIN  GPIO_NUM_34
#define OT1_RECV_PIN  GPIO_NUM_35
#define OT2_RECV_PIN  GPIO_NUM_NC
#define ONE_WIRE_PIN  GPIO_NUM_25
#define ONE_WIRE_GND  GPIO_NUM_26
#define SWITCH_PIN    GPIO_NUM_27

#define DEFAULT1 20.0
#define DEFAULT2 20.0

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
#define   room_sp_fv room_sp
#define   room_sp_ix 6
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
#define  PBurnerw_fv temp[PB]
#define  PBurnerw_ix 61
#define  PReturnw_fv temp[PR]
#define  PReturnw_ix 62

char    *pinger_target=NULL;

/* ============== BEGIN HOMEKIT CHARACTERISTIC DECLARATIONS =============================================================== */
// add this section to make your device OTA capable
// create the extra characteristic &ota_trigger, at the end of the primary service (before the NULL)
// it can be used in Eve, which will show it, where Home does not
// and apply the four other parameters in the accessories_information section

#include "lcm_api.h"
homekit_characteristic_t ota_trigger  = API_OTA_TRIGGER;
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  "X");
homekit_characteristic_t serial       = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, "1");
homekit_characteristic_t model        = HOMEKIT_CHARACTERISTIC_(MODEL,         "Z");
homekit_characteristic_t revision     = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION,  "0.0.0");

// next use these two lines before calling homekit_server_init(&config);
//    int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value,
//                                      &model.value.string_value,&revision.value.string_value);
//    config.accessories[0]->config_number=c_hash;
// end of OTA add-in instructions

void tgt_temp1_set(homekit_value_t value);
void tgt_temp2_set(homekit_value_t value);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverride-init"
homekit_characteristic_t tgt_heat1 = HOMEKIT_CHARACTERISTIC_(TARGET_HEATING_COOLING_STATE,  3 ); //AUTO
homekit_characteristic_t cur_heat1 = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, 0 );
homekit_characteristic_t tgt_temp1 = HOMEKIT_CHARACTERISTIC_(TARGET_TEMPERATURE,     DEFAULT1, .max_value=(float[]){25}, .setter=tgt_temp1_set );
homekit_characteristic_t cur_temp1 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         1.0 );
homekit_characteristic_t dis_temp1 = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS,     0 );

homekit_characteristic_t tgt_heat2 = HOMEKIT_CHARACTERISTIC_(TARGET_HEATING_COOLING_STATE,  1 ); //HEAT -> on/off mode
homekit_characteristic_t cur_heat2 = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATING_COOLING_STATE, 0 );
homekit_characteristic_t tgt_temp2 = HOMEKIT_CHARACTERISTIC_(TARGET_TEMPERATURE,     DEFAULT2, .max_value=(float[]){25}, .setter=tgt_temp2_set );
homekit_characteristic_t cur_temp2 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         2.0 );
homekit_characteristic_t dis_temp2 = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS,     0 );

homekit_characteristic_t cur_temp3 = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE,         3.0 );
#pragma GCC diagnostic pop


void tgt_temp1_set(homekit_value_t value) {
    if (value.format != homekit_format_float) {
        UDPLUO("Invalid target-value format: %d\n", value.format);
        return;
    }
    tgt_temp1.value=value;
    PUBLISH(tgt_temp1);
}

void tgt_temp2_set(homekit_value_t value) {
    if (value.format != homekit_format_float) {
        UDPLUO("Invalid target-value format: %d\n", value.format);
        return;
    }
    tgt_temp2.value=value;
    PUBLISH(tgt_temp2);
}


#define point05 (13/256.0) //these numbers do not get rounded in float operations
#define point1  (26/256.0) //0.1016
float ffactor=0.3046875; //corresponds to 3x26/256
#define HOMEKIT_CHARACTERISTIC_CUSTOM_FACTOR HOMEKIT_CUSTOM_UUID("F0000009")
#define HOMEKIT_DECLARE_CHARACTERISTIC_CUSTOM_FACTOR(_value, ...) \
    .type = HOMEKIT_CHARACTERISTIC_CUSTOM_FACTOR, \
    .description = "HeaterFactor", \
    .format = homekit_format_int, \
    .min_value=(float[]){-100}, \
    .max_value=(float[]) {105}, \
    .min_step  = (float[]) {5}, \
    .permissions = homekit_permissions_paired_read \
                 | homekit_permissions_paired_write, \
    .value = HOMEKIT_INT_(_value), \
    ##__VA_ARGS__
    
void factor_set(homekit_value_t value); 
homekit_characteristic_t factor=HOMEKIT_CHARACTERISTIC_(CUSTOM_FACTOR, 105, .setter=factor_set);
void factor_set(homekit_value_t value) {
    UDPLUS("Factor: %d\n", value.int_value); //factor 30 ~~ 0.3 degrees maximum steps
    ffactor=(value.int_value/5)*point05; //only works if factor is multiple of 5
    factor.value=value;
}

#define HOMEKIT_CHARACTERISTIC_CUSTOM_CHSETPOINT HOMEKIT_CUSTOM_UUID("F0000019")
#define HOMEKIT_DECLARE_CHARACTERISTIC_CUSTOM_CHSETPOINT(_value, ...) \
    .type = HOMEKIT_CHARACTERISTIC_CUSTOM_CHSETPOINT, \
    .description = "CHsetpoint", \
    .format = homekit_format_int, \
    .min_value=(float[])  {20}, \
    .max_value=(float[])  {50}, \
    .min_step  = (float[]) {1}, \
    .permissions = homekit_permissions_paired_read \
                 | homekit_permissions_paired_write, \
    .value = HOMEKIT_INT_(_value), \
    ##__VA_ARGS__
    
void chsetpoint_set(homekit_value_t value); 
homekit_characteristic_t chsetpoint=HOMEKIT_CHARACTERISTIC_(CUSTOM_CHSETPOINT, 30, .setter=chsetpoint_set);
void chsetpoint_set(homekit_value_t value) {
    UDPLUS("CHsetpoint: %d\n", value.int_value);
    chsetpoint.value=value;
}

// void identify_task(void *_args) {
//     vTaskDelete(NULL);
// }

void identify(homekit_value_t _value) {
    UDPLUS("Identify\n");
//    xTaskCreate(identify_task, "identify", 256, NULL, 2, NULL);
}

/* ============== END HOMEKIT CHARACTERISTIC DECLARATIONS ================================================================= */


#define TEMP2HK(n)  do {old_t##n=cur_temp##n.value.float_value; \
                        cur_temp##n.value.float_value=isnan(temp[S##n])?S##n##avg:(float)(int)(temp[S##n]*10+0.5)/10; \
                        if (old_t##n!=cur_temp##n.value.float_value) \
                            homekit_characteristic_notify(&cur_temp##n,HOMEKIT_FLOAT(cur_temp##n.value.float_value)); \
                    } while (0) //TODO: do we need to test for changed values or is that embedded in notify routine?
#define BEAT 10 //in seconds
#define SENSORS 3
#define S1 0 //   salon temp sensor
#define S2 1 //upstairs temp sensor
#define S3 6 // outdoor temp sensor
#define BW 4 //boiler water temp
#define RW 5 //return water temp
#define DW 8 //domestic home water temp
#define PB 2 //heatPump boiler temp
#define PR 3 //heatPump return temp
#define PO 7 //heatPump outside temp
#define PM 9 //heatPump max setpoint temp
float temp[16]={85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85}; //using id as a single hex digit, then hardcode which sensor gets which meaning
float S1temp[8],S2temp[8],S3temp[8],S1avg,S2avg,S3avg;
float S3total=0;
int   S3samples=0;
bool  S3set=false;

#define SINKBUS(time) do {gpio_set_level(ONE_WIRE_GND,1);vTaskDelay(time/portTICK_PERIOD_MS); \
                          gpio_set_level(ONE_WIRE_GND,0);vTaskDelay(1);} while(0)
void temp_task(void *argv) {
    gpio_config_t io_conf = {}; //zero-initialize the config structure
    io_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT_OD; //set as output mode with open drain
    io_conf.pin_bit_mask = 1ULL<<ONE_WIRE_GND; //this will serve as the ground for the one-wire and will float when we reset
    gpio_config(&io_conf); //configure GPIO with the given settings //use gpio_set_drive_capability(x,GPIO_DRIVE_CAP_3)?
    gpio_set_level(ONE_WIRE_GND,0); //enables ground
    
    int ids[SENSORS],fail=0,sensor_count;
    float old_t1,old_t2,old_t3;
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
        SINKBUS(7000); //long reset one-wire bus
        if (onewire_del_device_iter(iter)!=ESP_OK) UDPLUS("onewire_del_device_iter failed\n");
        if (fail++>50) {
            mqtt_client_publish("{\"idx\":%d,\"nvalue\":4,\"svalue\":\"Heater No Sensors\"}", idx);
            UDPLUS("restarting because can't find enough sensors\n");
            vTaskDelay(3000/portTICK_PERIOD_MS); //allow MQTT and UDPlog to flush output
            esp_restart(); //TODO: disable GPIO outputs
        }
        vTaskDelay(BEAT*1000/portTICK_PERIOD_MS);
    } //break out when all SENSORS are found

    float temperature;
    bool been_reset;
    while (1) {
        been_reset=false;
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); //the timer loop triggers this 1 x in 10 seconds
        for (int indx=0;indx<sensor_count; indx++) {
            if ((result=ds18b20_trigger_temperature_conversion(ds18b20s[indx])) == ESP_OK) { //has 800ms delay built into the conversion...
                if ((result=ds18b20_get_temperature(ds18b20s[indx], &temperature)) == ESP_OK) { //get temperature from sensors one by one
                    //UDPLUS("temperature read from DS18B20[%d]: %.4fC\n", indx, temperature);
                    temp[ids[indx]] = (temperature>125.0 || temperature<-55.0)?NAN:temperature;
                } else {
                    UDPLUS("ds18b20_get_temperature for id:%d failed %x: %s\n",ids[indx],result,esp_err_to_name(result));
                    temp[ids[indx]] = NAN;
                    if (been_reset) continue; //do not sink bus twice in a 10s slot
                    SINKBUS(7000); //long reset one-wire bus
                    been_reset=true;
                    UDPLUS("after long reset bus\n"); //without depriving a stuck sensor from power for a few seconds, it will stay stuck
                }
            } else {
                UDPLUS("ds18b20_trigger_temperature_conversion for id:%d failed %x: %s\n",ids[indx],result,esp_err_to_name(result));
                temp[ids[indx]] = NAN;
                if (been_reset) continue; //do not sink bus twice in a 10s slot
                SINKBUS(7000); //long reset one-wire bus
                been_reset=true;
            }
        }
        if (!isnan(temp[S3]) && temp[S3]!=85) {
            S3samples++,S3total+=temp[S3];
            if (S3set==false) { //initialize S3temp with first valid measurement
                S3temp[0]=S3temp[1]=S3temp[2]=S3temp[3]=S3temp[4]=S3temp[5]=S3temp[6]=S3temp[7]=temp[S3];
                S3set=true;
            }
        }
        TEMP2HK(1);
        TEMP2HK(2);
        TEMP2HK(3);
    }
}


// #define RTC_ADDR    0x600013B0
// #define RTC_MAGIC   0xaabecede
float room_sp=DEFAULT1,room_temp=DEFAULT1+point1;
float curr_mod=0,heat_mod=0,pump_mod=0,pressure=0;
int   time_set=0,stateflg=0,pumpstateflg=0,errorflg=0;
int   heat_on=0,csp=30;
float hys1=0.0,hys2=0.0;
int heater(uint32_t seconds) {
    char strtm[32]; // e.g. DST0wd2yd4    5|07:02:00.060303
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_t now=tv.tv_sec;
    struct tm *tm = localtime(&now);
    sprintf(strtm,"DST%dwd%dyd%-3d %2d|%02d:%02d:%02d",tm->tm_isdst,tm->tm_wday,tm->tm_yday,tm->tm_mday,
            tm->tm_hour,tm->tm_min,tm->tm_sec);
    
    int heater1=0,heater2=0;
    float delta1=0.0,delta2=0.0;
    //heater1 logic
    delta1=S1avg-tgt_temp1.value.float_value;
    if (delta1<hys1) {heater1=1; hys1=point1;} else hys1=0.0;
    
    //heater2 logic
    delta2=S2avg-tgt_temp2.value.float_value;
    if (delta2<0.5+hys2) {heater2=1; hys2=0.02;  } else hys2=0.0; //let the CiC regulate this itself above 0.5
    
    //integrated logic for both heaters
    room_temp=S1avg;
    room_sp=heater1?tgt_temp1.value.float_value+hys1:S1avg-delta2;
    if (ffactor<1.05 && (room_sp-room_temp)>ffactor) room_sp=room_temp+ffactor;
    int result=0; if (heater1) result=1; else if (heater2) result=2; //we must inhibit floor heater pump

    //calculate adjusted ChSetPoint aka CSP
    float          csp_target=chsetpoint.value.int_value;
    if (result==1) csp_target=chsetpoint.value.int_value-S3avg*0.5;
    if (result==2) csp_target=chsetpoint.value.int_value-S3avg*2.0;
    if (csp>csp_target+1) csp=(int)csp_target;
    if (csp<csp_target-1) csp=(int)csp_target;
    if (csp<20) csp=20;
    
    //final report
    UDPLUS("S1=%7.4f S2=%7.4f S3=%7.4f Heater@%-4ld  %s => room_sp:%5.2f h1:%d+h2:%d=on:%d csp:%d ST=%02x PST=%02x\n", \
            S1avg,S2avg,S3avg,(seconds+10)/60,strtm,room_sp,heater1,heater2,result,csp,stateflg,pumpstateflg);
    PUBLISH(S1avg);
    PUBLISH(S2avg);
    PUBLISH(heat_mod);
    PUBLISH(room_sp);
    PUBLISH(burnerW);
    PUBLISH(returnW);
    PUBLISH(pressure);
    PUBLISH(S3avg);
    PUBLISH(PBurnerw);
    PUBLISH(PReturnw);
    
    //save state to RTC memory
    //also add crash reason, if known
    //save two setpoints, factor, crashreason
//     uint32_t *dp;         WRITE_PERI_REG(RTC_ADDR+ 4,mode     ); //int
//                           WRITE_PERI_REG(RTC_ADDR+ 8,heat_till); //time_t
//     dp=(void*)&ffactor;   WRITE_PERI_REG(RTC_ADDR+12,*dp      ); //float
//     dp=(void*)&prev_setp; WRITE_PERI_REG(RTC_ADDR+16,*dp      ); //float
//     dp=(void*)&peak_temp; WRITE_PERI_REG(RTC_ADDR+20,*dp      ); //float
//                           WRITE_PERI_REG(RTC_ADDR+24,peak_time); //int
//     dp=(void*)&stable_tgt_temp1;            WRITE_PERI_REG(RTC_ADDR+28,*dp      ); //float
//     dp=(void*)&tgt_temp2.value.float_value; WRITE_PERI_REG(RTC_ADDR+32,*dp      ); //float
//     dp=(void*)&setpoint2; WRITE_PERI_REG(RTC_ADDR+36,*dp      ); //float
//     dp=(void*)&S3total;   WRITE_PERI_REG(RTC_ADDR+40,*dp      ); //float
//                           WRITE_PERI_REG(RTC_ADDR+44,S3samples); //int
//                           WRITE_PERI_REG(RTC_ADDR   ,RTC_MAGIC);
    return result;
}

void init_task(void *argv) {
    vTaskDelay(1000/portTICK_PERIOD_MS);
//     printf("RTC: "); for (int i=0;i<7;i++) printf("%08x ",READ_PERI_REG(RTC_ADDR+i*4)); printf("\n");
//     uint32_t *dp;
// 	if (READ_PERI_REG(RTC_ADDR)==RTC_MAGIC) {
// 	    mode                    =READ_PERI_REG(RTC_ADDR+ 4);
//         heat_till               =READ_PERI_REG(RTC_ADDR+ 8);
//         dp=(void*)&ffactor;  *dp=READ_PERI_REG(RTC_ADDR+12); factor.value.int_value=ffactor;
//         dp=(void*)&prev_setp;*dp=READ_PERI_REG(RTC_ADDR+16);
//         dp=(void*)&peak_temp;*dp=READ_PERI_REG(RTC_ADDR+20);
//         peak_time               =READ_PERI_REG(RTC_ADDR+24);
//         dp=(void*)&(tgt_temp1.value.float_value);*dp=READ_PERI_REG(RTC_ADDR+28);
//         dp=(void*)&(tgt_temp2.value.float_value);*dp=READ_PERI_REG(RTC_ADDR+32);
//         dp=(void*)&setpoint2;*dp=READ_PERI_REG(RTC_ADDR+36);
//         dp=(void*)&S3total;  *dp=READ_PERI_REG(RTC_ADDR+40);
//         S3samples               =READ_PERI_REG(RTC_ADDR+44);
//     }
//     printf("INITIAL prev_setp=%2.1f f=%2.1f peak_time=%2d peak_temp=%2.4f mode=%d heat_till %s",
//             prev_setp,ffactor,peak_time,peak_temp,mode,ctime(&heat_till));
    PUBLISH(tgt_temp1);
    PUBLISH(tgt_temp2);
    //prevents starting heat if no sensor readings would come in
    S1temp[0]=S1temp[1]=S1temp[2]=S1temp[3]=S1temp[4]=S1temp[5]=S1temp[6]=S1temp[7]=tgt_temp1.value.float_value+0.125;
    S2temp[0]=S2temp[1]=S2temp[2]=S2temp[3]=S2temp[4]=S2temp[5]=S2temp[6]=S2temp[7]=tgt_temp2.value.float_value+0.125;
    
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1); tzset();
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&config);
    while (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK) {
        UDPLUS("Still waiting for system time to sync\n");
    }
    time_t ts = time(NULL);
    UDPLUS("TIME SET: %u=%s\n", (unsigned int) ts, ctime(&ts));
    time_set=1;
    vTaskDelete(NULL);
}

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


#define NO_RSP(CHANNEL) do {UDPLUS("CH%d_NO_RSP!!! ",CHANNEL); \
        resp_idx[CHANNEL]=0, rx_state[CHANNEL]=IDLE, response[CHANNEL]=0; \
        ot_recv_enable[CHANNEL]=false; \
        } while(0) //TODO: is resetting these values critical for proper operation?
#define CalcAvg(Sx) do {            Sx##temp[7]=Sx##temp[6];Sx##temp[6]=Sx##temp[5];Sx##temp[5]=Sx##temp[4]; \
            Sx##temp[4]=Sx##temp[3];Sx##temp[3]=Sx##temp[2];Sx##temp[2]=Sx##temp[1];Sx##temp[1]=Sx##temp[0]; \
            if ( !isnan(temp[Sx]) && temp[Sx]!=85 )         Sx##temp[0]=temp[Sx];    \
            Sx##avg=(Sx##temp[0]+Sx##temp[1]+Sx##temp[2]+Sx##temp[3]+Sx##temp[4]+Sx##temp[5]+Sx##temp[6]+Sx##temp[7])/8.0; \
        } while(0)
#define FLOAT2OT(f) (((uint32_t)((f)*256*256.0))>>8)
static TaskHandle_t tempTask = NULL;
int timeIndex=0,switch_state=0,pump_off_time=0,retrigger=0,heat_result=0;
int push=-2;
TimerHandle_t xTimer;
void vTimerCallback( TimerHandle_t xTimer ) {
    uint32_t seconds = ( uint32_t ) pvTimerGetTimerID( xTimer );
    vTimerSetTimerID( xTimer, (void*)seconds+1); //136 year to loop
    uint32_t message;
    int switch_on=0;
    if (gpio_get_level(SWITCH_PIN)) switch_state--; else switch_state++; //pin is low when switch is on
    if (switch_state<0) switch_state=0;
    if (switch_state>3) switch_state=3;
    switch_on=switch_state>>1;
    //TODO read recv pin and if it is a ONE, we have an OpenTherm error state
    UDPLUS("St%d Sw%d @%ld ",timeIndex,switch_on,seconds);
    if (timeIndex==3) { // allow 3 seconds for two automation rules to succeed and repeat every 10 seconds
        if (pump_off_time) pump_off_time-=10;
        if (tgt_heat1.value.int_value==HOMEKIT_TARGET_HEATING_COOLING_STATE_COOL) { //Pump Off rule confirmed
            if (heat_result) cur_heat2.value.int_value= 1;   //confirm we are heating upstairs
            else cur_heat2.value.int_value= 0;   //confirm we are not heating
            homekit_characteristic_notify(&cur_heat2,HOMEKIT_UINT8(cur_heat2.value.int_value));
            tgt_heat1.value.int_value= HOMEKIT_TARGET_HEATING_COOLING_STATE_AUTO;   //set heater 1 mode back to auto and be ready for another trigger
            homekit_characteristic_notify(&tgt_heat1,HOMEKIT_UINT8(tgt_heat1.value.int_value)); //TODO: racecondition?
            pump_off_time=180; //seconds
            if (heat_result) heat_on=1; else heat_on=0;
        }
        if (cur_heat2.value.int_value==2) {//send reminder notify
            cur_heat2.value.int_value= 0; //to assure it is considered as a new value we first set it to off
            homekit_characteristic_notify(&cur_heat2,HOMEKIT_UINT8(cur_heat2.value.int_value)); //and notify
            retrigger=1;
            if (pump_off_time>10) {if (heat_result) heat_on=1; else heat_on=0;} //still time left
        }
    }
    if (retrigger && timeIndex==8) { retrigger=0; //retrigger needed 5 seconds offset
        cur_heat2.value.int_value= 2;
        homekit_characteristic_notify(&cur_heat2,HOMEKIT_UINT8(cur_heat2.value.int_value));
    }
    
    if (timeIndex==0) {
        xTaskNotifyGive( tempTask ); //temperature measurement start
        vTaskDelay(1); //TODO: is this needed: prevent interference between OneWire and OT-receiver
    }
    if (timeIndex==9) { //even with a bus-reset, results should be in and before a heater calculation
        CalcAvg(S1); CalcAvg(S2); CalcAvg(S3);
    }
    
    switch (timeIndex) { //send commands BOILER
        case 0: //measure temperature
            message=0x00190000; //25 read boiler water temperature
            break;
        case 1: //execute heater decisions
            if (tgt_heat2.value.int_value==HOMEKIT_TARGET_HEATING_COOLING_STATE_HEAT) { //use on/off switching thermostat
                   //message=0x10014100; //65 deg //1  CH setpoint in deg C
                   message=0x10010000|((csp+2)*256); //1  CH setpoint in deg C
            } else message=0x10010000|FLOAT2OT(tgt_temp1.value.float_value*2-1); //range from 19 - 75 deg
            break;
        case 2: message=0x100e6400; break; //100% //14 max modulation level
        case 3:
            if (tgt_heat2.value.int_value==HOMEKIT_TARGET_HEATING_COOLING_STATE_HEAT) { //use on/off switching thermostat
                   message=0x00000200|(switch_on?0x100:0x000); //0  enable CH and DHW
            } else message=0x00000200|(tgt_heat1.value.int_value<<8); //0=off, 1=heat, 2=off, 3=heat
            break; 
        case 4: message=0x00380000; break; //56 DHW setpoint read
        case 5: message=0x00050000; break; //5  app specific fault flags
        case 6: message=0x00120000; break; //18 CH water pressure
        case 7: message=0x001a0000; break; //26 DHW temp
        case 8: message=0x001c0000; break; //28 return water temp
        case 9: message=0x00110000; break; //17 rel mod level
        default: break;
    }
    if (seconds>50) send_OT_frame(BOILER, message); //send message to BOILER OT receiver
    //since we want to run two OT channels every second, we only wait for response for 400ms
    if (xQueueReceive(gpio_evt_queue[BOILER], &(message), (TickType_t)400/portTICK_PERIOD_MS) == pdTRUE) {
        UDPLUS("CH%dRSP:%08lx ",BOILER,message);
        switch (timeIndex) { //check answers from BOILER
            case 0: temp[BW]=(float)(message&0x0000ffff)/256; break;
            case 3:
                heat_mod=0.0;
                stateflg=(message&0x0000007f);
                if ((stateflg&0xa)==0xa) {
                    heat_mod=1.0; //at this point it is a multiplier
                    cur_heat1.value.int_value=pump_off_time?2:1; //present heater on but pump off as cur_heat1=2 COOL
                } else cur_heat1.value.int_value=0;
                homekit_characteristic_notify(&cur_heat1,HOMEKIT_UINT8(cur_heat1.value.int_value));
                break;
            case 5: errorflg=       (message&0x00003f00)/256; break;
            case 6: pressure=(float)(message&0x0000ffff)/256; break;
            case 7: temp[DW]=(float)(message&0x0000ffff)/256; break;
            case 8: temp[RW]=(float)(message&0x0000ffff)/256; break;
            case 9:
                curr_mod=(float)(message&0x0000ffff)/256;
                if (curr_mod==70.0) heat_mod=0.0; else heat_mod*=curr_mod;
                break;
            default: break;
        }
    } else {
        NO_RSP(BOILER);
    } //END of BOILER CONTROL
    
    switch (timeIndex) { //send commands HEATPUMP
        case 0: message=0x00194600; break; //25 read boiler water temperature
                //    CMD:00194600    RSP:40191752    25 read boiler water temperature 70 => 23.32
        case 1: message=0x10010000|(heat_on?(csp*256):0x0000); break; //set to chsetpoint when heat_on
                //    CMD:10010000    RSP:50010000     1 CH setpoint in deg C  (now zero, since CH=off)
        case 2: message=0x100e6400; break; //100% //14 max modulation level
                //    CMD:100e6400    RSP:500e6400    14 max modulation level = 100
        case 3: message=0x00000200|(heat_on?0x100:0x000); break; //0 enable CH and DHW
                //    CMD:00000200    RSP:40000200     0 enable CH and DHW   (CH=off and DHW=on)
        case 4: message=0x10100000|FLOAT2OT(room_sp); break; //TODO: introduce gradual tracking
                //    CMD:10101300    RSP:50101300    16 Room Setpoint = 19
        case 5: message=0x10180000|FLOAT2OT(room_temp); break; //24 Room temperature
                //    CMD:101815a8    RSP:501815a8    24 Room temperature = 21.65625
        case 6: message=0x001b4600; break; //27 outside temp
                //    CMD:001b4600    RSP:401b1152    27 outside temp 70 => 17.32
        case 7: message=0x00394600; break; //57 max CH water setpoint (RW)
                //    CMD:00394600    RSP:40394600    57 max CH water setpoint (RW) = 70
        case 8: message=0x001c4600; break; //28 return water temp
                //    CMD:001c4600    RSP:401c126d    28 return water temp 70 => 18.425
        case 9: message=0x00110000; break; //17 rel mod level
                //    CMD:00110000    RSP:40110000    17 rel mod level
        default: break;
    }
    if (seconds>50) send_OT_frame(HEATPUMP, message); //send message to HEATPUMP OT receiver
    //since we want to run two OT channels every second, we only wait for response for 400ms
    if (xQueueReceive(gpio_evt_queue[HEATPUMP], &(message), (TickType_t)400/portTICK_PERIOD_MS) == pdTRUE) {
        UDPLUS("CH%dRSP:%08lx ",HEATPUMP,message);
        switch (timeIndex) { //check answers from HEATPUMP
            case 0: temp[PB]=(float)(message&0x0000ffff)/256; break;
                //   CMD:00194600    RSP:40191752    25 read boiler water temperature 70 => 23.32
            case 3: pumpstateflg=(message&0x0000007f); break; //0x0a is CH on
                //   CMD:00000200    RSP:40000200     0 enable CH and DHW   (CH=off and DHW=on)
            case 6: temp[PO]=(float)(message&0x0000ffff)/256; break;
                //   CMD:001b4600    RSP:401b1152    27 outside temp 70 => 17.32
            case 7: temp[PM]=(float)(message&0x0000ffff)/256; break;
                //   CMD:00394600    RSP:40394600    57 max CH water setpoint (RW) = 70
            case 8: temp[PR]=(float)(message&0x0000ffff)/256; break;
                //   CMD:001c4600    RSP:401c126d    28 return water temp 70 => 18.425
            case 9: pump_mod=(float)(message&0x0000ffff)/256; break;
                //   CMD:00110000    RSP:40110000    17 rel mod level
            default: break;
        }
    } else {
        NO_RSP(HEATPUMP);
    } //END of HEATPUMP CONTROL
    
    UDPLUS("\n"); //close detailed report line
    
    //errorflg=(seconds/600)%2; //test trick to change outcome every 10 minutes
    if (seconds%60==5) {
        if (errorflg) { //publish a ORANGE (3) ALERT on domoticz
            if (push>0) {
                int n=mqtt_client_publish("{\"idx\":%d,\"nvalue\":3,\"svalue\":\"Heater ERR: 0x%02X\"}", idx, errorflg);
                if (n<0) UDPLUS("MQTT publish of ALERT failed because %s\n",MQTT_CLIENT_ERROR(n)); else push--;
                if (push==0) push=-2;
            }
        } else { //publish a GREY (0) clean ALERT on domoticz
            if (push<0) {
                int n=mqtt_client_publish("{\"idx\":%d,\"nvalue\":0,\"svalue\":\"Heater OK (again) %d\"}", idx, seconds/60);
                if (n<0) UDPLUS("MQTT publish of ALERT failed because %s\n",MQTT_CLIENT_ERROR(n)); else push++;
                if (push==0) push=3;            
            }
        }
    }
//  if (seconds%3600==3599) { //force a alive report every hour
//      int n=mqtt_client_publish("{\"idx\":%d,\"nvalue\":0,\"svalue\":\"Heater alive %d\"}", idx, seconds/60);    
//  }

    if (seconds%60==50) { //allow 6 temperature measurments to make sure all info is loaded
        heat_on=0;
        heat_result=heater(seconds); //sets heat_sp and returns heater result, 0, 1 or 2
        if (heat_result) heat_on=1;
        cur_heat2.value.int_value=heat_result;
        if (heat_result!=1 && pump_off_time<100) cur_heat2.value.int_value=2; //retrigger rules so pump off
        homekit_characteristic_notify(&cur_heat2,HOMEKIT_UINT8(cur_heat2.value.int_value));
    }

    if (timeIndex==3) {
        UDPLUS("S1=%7.4f S2=%7.4f S3=%7.4f ERR=%02x PR=%4.2f DW=%4.1f RW=%4.1f BW=%4.1f SW=%d MOD=%02.0f ST=%02x POT=%3d ON=%d PB=%4.1f PR=%4.1f PST=%02x\n", \
           temp[S1],temp[S2],temp[S3],errorflg,pressure,temp[DW],temp[RW],temp[BW],switch_on,curr_mod,stateflg,pump_off_time,heat_on,temp[PB],temp[PR],pumpstateflg);
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

void switch_init() { //reads a NormallyOpen contact from Quatt CiC to switch on the boiler
    gpio_config_t io_conf = {}; //zero-initialize the config structure.
    io_conf.pin_bit_mask = (1ULL<<SWITCH_PIN); //bit mask of the pins
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1; //enable pull-up mode
    gpio_config(&io_conf); //configure GPIO with the given settings
}

int ping_count=120,ping_delay=1; //seconds
static void ping_success(esp_ping_handle_t hdl, void *args) {
    ping_count+=20; ping_delay+=5;
    if (ping_count>300) ping_count=300;
    if (ping_delay>60)  ping_delay=60;
    uint32_t elapsed_time;
    ip_addr_t response_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR,  &response_addr,  sizeof(response_addr));
    UDPLUS("good ping from %s %lu ms -> count: %d s\n", inet_ntoa(response_addr.u_addr.ip4), elapsed_time, ping_count);
}
static void ping_timeout(esp_ping_handle_t hdl, void *args) {
    ping_count--; ping_delay=1;
    UDPLUS("failed ping -> count: %d s\n", ping_count);
}
void ping_task(void *argv) {
    ip_addr_t target_addr;
    ipaddr_aton(pinger_target,&target_addr);
    esp_ping_handle_t ping;
    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
    ping_config.target_addr = target_addr;
    ping_config.timeout_ms = 900; //should time-out before our 1s delay
    ping_config.count = 1; //one ping at a time means we can regulate the interval at will
    esp_ping_callbacks_t cbs = {.on_ping_success=ping_success, .on_ping_timeout=ping_timeout, .on_ping_end=NULL, .cb_args=NULL}; //set callback functions
    esp_ping_new_session(&ping_config, &cbs, &ping);
    
    UDPLUS("Pinging IP %s\n", ipaddr_ntoa(&target_addr));
    while(ping_count){
        vTaskDelay((ping_delay-1)*(1000/portTICK_PERIOD_MS)); //already waited 1 second...
        esp_ping_start(ping);
        vTaskDelay(1000/portTICK_PERIOD_MS); //waiting for answer or timeout to update ping_delay value
    }
    mqtt_client_publish("{\"idx\":%d,\"nvalue\":2,\"svalue\":\"Heater No Ping\"}", idx);
    UDPLUS("restarting because can't ping home-hub\n");
    vTaskDelay(3000/portTICK_PERIOD_MS); //allow MQTT and UDPlog to flush output
    esp_restart(); //TODO: disable GPIO outputs
}

mqtt_config_t mqttconf=MQTT_DEFAULT_CONFIG;
char error[]="error";
static void ota_string() {
    char *dmtczbaseidx1=NULL;
    esp_err_t status;
    nvs_handle_t lcm_handle;
    char *otas=NULL;
    size_t  size;
    status = nvs_open("LCM", NVS_READONLY, &lcm_handle);
    
    if (!status && nvs_get_str(lcm_handle, "ota_string", NULL, &size) == ESP_OK) {
        otas = malloc(size);
        nvs_get_str(lcm_handle, "ota_string", otas, &size);
        mqttconf.host=strtok(otas,";");
        mqttconf.user=strtok(NULL,";");
        mqttconf.pass=strtok(NULL,";");
        dmtczbaseidx1=strtok(NULL,";");
        //pinger_target=strtok(NULL,";");
    }
    if (mqttconf.host==NULL) mqttconf.host=error;
    if (mqttconf.user==NULL) mqttconf.user=error;
    if (mqttconf.pass==NULL) mqttconf.pass=error;
    if (dmtczbaseidx1==NULL) idx=1000; else idx=atoi(dmtczbaseidx1);
    //if (pinger_target==NULL) pinger_target=error;
    pinger_target="192.168.178.100";
    //DO NOT free the otas since it carries the config pieces
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverride-init"
homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(
        .id=1,
        .category=homekit_accessory_category_thermostats,
        .services=(homekit_service_t*[]){
            HOMEKIT_SERVICE(ACCESSORY_INFORMATION,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "OutdoorT"),
                    &manufacturer,
                    &serial,
                    &model,
                    &revision,
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
                    NULL
                }),
            HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .primary=true,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "OutdoorT"),
                    &cur_temp3,
                    &ota_trigger,
                    &factor,
                    &chsetpoint,
                    NULL
                }),
            NULL
        }),
    HOMEKIT_ACCESSORY(
        .id=2,
        .category=homekit_accessory_category_thermostats,
        .services=(homekit_service_t*[]){
            HOMEKIT_SERVICE(ACCESSORY_INFORMATION,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Thermostat"),
                    &manufacturer,
                    &serial,
                    &model,
                    &revision,
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
                    NULL
                }),
            HOMEKIT_SERVICE(THERMOSTAT, .primary=true,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Thermostat"),
                    &tgt_heat1,
                    &cur_heat1,
                    &tgt_temp1,
                    &cur_temp1,
                    &dis_temp1,
                    NULL
                }),
            NULL
        }),
    HOMEKIT_ACCESSORY(
        .id=3,
        .category=homekit_accessory_category_thermostats,
        .services=(homekit_service_t*[]){
            HOMEKIT_SERVICE(ACCESSORY_INFORMATION,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Thermo2"),
                    &manufacturer,
                    &serial,
                    &model,
                    &revision,
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
                    NULL
                }),
            HOMEKIT_SERVICE(THERMOSTAT, .primary=true,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Thermo2"),
                    &tgt_heat2,
                    &cur_heat2,
                    &tgt_temp2,
                    &cur_temp2,
                    &dis_temp2,
                    NULL
                }),
            NULL
        }),
    NULL
};
#pragma GCC diagnostic pop

homekit_server_config_t config = {
    .accessories = accessories,
//     .on_event=device_init,
    .password = "111-11-111"
};

void main_task(void *arg) {
    udplog_init(3);
    vTaskDelay(300); //Allow Wi-Fi to connect
    UDPLUS("\n\nQuatt-control %s\n",esp_app_get_description()->version);
    
    //nvs_handle_t lcm_handle;nvs_open("LCM", NVS_READWRITE, &lcm_handle);nvs_set_str(lcm_handle,"ota_string", "192.168.178.5;test;test;68");
    //nvs_commit(lcm_handle); //can be used if not using LCM
    ota_string();
    mqtt_client_init(&mqttconf);
    switch_init();
    OT_init();
    xTaskCreatePinnedToCore(temp_task,"Temp", 4096, NULL, 1, &tempTask,1); //TODO: check if really needed to survive stuck sensor
    xTaskCreate(init_task,"Time", 2048, NULL, 6, NULL);
    xTaskCreate(ping_task,"PingT",2048, NULL, 1, NULL);
    xTimer=xTimerCreate( "Timer", 1000/portTICK_PERIOD_MS, pdTRUE, (void*)0, vTimerCallback);
    xTimerStart(xTimer, 0);
    //vTaskDelay(1000/portTICK_PERIOD_MS); //Allow inits to settle
    //esp_intr_dump(NULL);
    
    int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value,
                                      &model.value.string_value,&revision.value.string_value);
//     c_hash=1; revision.value.string_value="0.0.1"; //cheat line
    config.accessories[0]->config_number=c_hash;
    
    homekit_server_init(&config);

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
