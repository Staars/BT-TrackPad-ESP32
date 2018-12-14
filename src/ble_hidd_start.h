#ifndef __BLE_HIDD_START__
#define __BLE_HIDD_START__

#include "esp_hidd_prf_api.h"
#include "esp_gap_ble_api.h"

extern void hidd_init(void);

bool sec_conn;
uint16_t hid_conn_id;


#endif  ///__BLE_HIDD_START__
