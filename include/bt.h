#ifndef __BT_H__
#define __BT_H__

#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"


#define DEVICE_NAME "SOFT_HALL_1"
#define SERVER_NAME "geon-pavilion"

#define SPP_DATA_LEN 20

extern int bt_fd;



#define BT_TASK_SIG_DISPATCH (0x01)


typedef void (* bt_task_cb_t) (uint16_t event, void *param);

typedef struct {
    uint16_t             sig;      /*!< signal to spp_task_task */
    uint16_t             event;    /*!< message event id */
    bt_task_cb_t         cb;       /*!< context switch callback */
    void                 *param;   /*!< parameter area needs to be last */
} bt_task_msg_t;


esp_err_t bt_init(void);
void bt_task(void *pvParameter);
bool bt_task_work_dispatch(bt_task_cb_t p_cback, uint16_t event, void *p_params, int param_len);

void bt_wr_task_start_up(void* p_cback, int fd);
void bt_wr_task_shut_down(void);

#endif