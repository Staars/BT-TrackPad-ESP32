/* This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this software is 
   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR  
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "Goodix.h"

extern "C" {
 #include "ble_hidd_start.h"
 #include "hid_dev.h"
}

#define INT_PIN                12
#define RST_PIN                14
#define GOODIX_SDA             21
#define GOODIX_SCL             22
#define GOODIX_SPEED           200000

#define HID_MAIN_TAG "HID_MAIN"

void GT911_task(void *pvParameter);
void GT911_handleTouch(int8_t contacts, GTPoint *points);

Goodix touch = Goodix();

typedef struct {
  uint16_t x;
  uint16_t y;
} last_position_t;

last_position_t last_position[5];
int64_t lastTimestamp = esp_timer_get_time();

QueueHandle_t tap_q;
TaskHandle_t tap_handle;
int64_t tapTimestamp = esp_timer_get_time();
bool restartTap = true;



void GT911_handleTouch(int8_t contacts, GTPoint *points) {
  // ESP_LOGD(HID_MAIN_TAG,"Contacts: %d", contacts);
  // for (uint8_t i = 0; i < contacts; i++) {
  //   // ESP_LOGI(HID_MAIN_TAG,"C%d: #%d %d,%d s:%d", i, points[i].trackId, points[i].x, points[i].y, points[i].area);
  //  printf("C%d: #%d %d,%d s:%d\n", i, points[i].trackId, points[i].x, points[i].y, points[i].area);
  // }
 
   if(esp_timer_get_time()-lastTimestamp > 100000){
    last_position[0].x = points[0].x; // retouch 
    last_position[0].y = points[0].y; // we want to start at 0;0
    tapTimestamp = esp_timer_get_time();
    vTaskResume(tap_handle);
  }
  if(esp_timer_get_time()-tapTimestamp > 100000){
    vTaskSuspend(tap_handle); // after some contact time, there is no tap possible anymore
  }
  uint8_t a[4] = {0,0,0,0}; //x,y,wheel,button

  a[0] = -(points[0].x - last_position[0].x);
  a[1] = -(points[0].y - last_position[0].y);
  
  // a[3] = 0; //button

  last_position[0].x = points[0].x;
  last_position[0].y = points[0].y;
  if((uint32_t)a !=0){ // do not send zeros
    esp_hidd_send_mouse_value(hid_conn_id, 0, a[0], a[1]);
  }
  lastTimestamp = esp_timer_get_time();
}

void GT911_tapTask(void *pvParameter){
    for( ;; )
    {
        if(esp_timer_get_time()-tapTimestamp > 150000){
          esp_hidd_send_mouse_value(hid_conn_id, 1, 0, 0);
          vTaskSuspend(tap_handle);
        }
    }


}

bool GT911_init(void) {
  ESP_LOGI(HID_MAIN_TAG,": Goodix GT911x touch driver");
  touch.i2cSetup(GOODIX_SDA, GOODIX_SCL, GOODIX_SPEED); 
  ESP_LOGI(HID_MAIN_TAG,": Goodix I2C Setup complete");
  tap_q = xQueueCreate(32,sizeof(bool));
  xTaskCreate(&GT911_tapTask, "GT911_tapTask", 8192, NULL, 5, &tap_handle);
  vTaskSuspend(tap_handle); // we need it later
  xTaskCreate(&GT911_task, "GT911_task", 8192, NULL, 5, NULL);
  return true;
}

void GT911_task(void *pvParameter)
{
    touch.setHandler(GT911_handleTouch);
    if (touch.begin(INT_PIN, RST_PIN)!=true) {
    ESP_LOGE(HID_MAIN_TAG,"Module reset failed");
    } 
    else {
      ESP_LOGI(HID_MAIN_TAG,"Module reset OK");
    }
    ESP_LOGI(HID_MAIN_TAG,"Check ACK on addr request on 0x%x",touch.i2cAddr);
    ESP_LOGI(HID_MAIN_TAG,": touch started"); 
    while(1) { 
		vTaskDelay(10 / portTICK_RATE_MS);	
      touch.loop();
    }
}



void app_main()
{
    ESP_LOGI(HID_MAIN_TAG, "BT startup");
    hidd_init();
    
    ESP_LOGI(HID_MAIN_TAG, "launching GT911 task");
    GT911_init();
}
}






