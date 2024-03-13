
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include <arpa/inet.h>

#include "sys/unistd.h"
#include "time.h"
#include "sys/time.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_timer.h"

#include "bt.h"
#include "led.h"

static const char *TAG = "bt_cfg";
static const char *TASK_TAG = "bt_task";

static QueueHandle_t bt_task_queue = NULL;

int bt_fd = -1;

static uint64_t time_ofs = -1; // TODO: change...

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_master = ESP_SPP_ROLE_MASTER;

esp_bd_addr_t peer_bd_addr = {0};
static uint8_t peer_bdname_len;
static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];

static const esp_bt_inq_mode_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY;
static const uint8_t inq_len = 30;
static const uint8_t inq_num_rsps = 0;

static void bt_reconnect(void);
static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void esp_spp_cb(uint16_t e, void *p);
static bool get_name_from_eir(uint8_t *eir, char *bdname, uint8_t *bdname_len);
static char *bda2str(uint8_t * bda, char *str, size_t size);
static bool bt_task_send_msg(bt_task_msg_t *msg);
static void bt_task_work_dispatched(bt_task_msg_t *msg);
static void esp_spp_stack_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
static void recv_server_time(void *param);


esp_err_t bt_init(void) {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(esp_bt_gap_cb));
    ESP_ERROR_CHECK(esp_spp_register_callback(esp_spp_stack_cb));
    return ESP_OK;
}

void bt_task(void *pvParameter) {
    bt_task_queue = *((QueueHandle_t*)pvParameter);
    bt_task_msg_t msg;
    esp_spp_cfg_t bt_spp_cfg = BT_SPP_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&bt_spp_cfg));
    ESP_LOGI(TAG, "esp_spp_init finished");

    for (;;) {
        if (pdTRUE == xQueueReceive(bt_task_queue, &msg, (TickType_t)portMAX_DELAY)) {
            ESP_LOGD(TASK_TAG, "%s, sig 0x%x, 0x%x", __func__, msg.sig, msg.event);
            switch (msg.sig) {
            case BT_TASK_SIG_DISPATCH:
                bt_task_work_dispatched(&msg);
                break;
            default:
                ESP_LOGW(TASK_TAG, "%s, unhandled sig: %d", __func__, msg.sig);
                break;
            }

            if (msg.param) {
                free(msg.param);
            }
        }
    }
}

bool bt_task_work_dispatch(bt_task_cb_t p_cback, uint16_t event, void *p_params, int param_len)
{
    ESP_LOGD(TASK_TAG, "%s event 0x%x, param len %d", __func__, event, param_len);

    bt_task_msg_t msg;
    memset(&msg, 0, sizeof(bt_task_msg_t));

    msg.sig = BT_TASK_SIG_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    if (param_len == 0) {
        return bt_task_send_msg(&msg);
    } else if (p_params && param_len > 0) {
        if ((msg.param = malloc(param_len)) != NULL) {
            memcpy(msg.param, p_params, param_len);
            return bt_task_send_msg(&msg);
        }
    }

    return false;
}

esp_err_t write_time_data(char* buf, uint8_t* remain_size) {
    if (time_ofs < 0) {
        return ESP_OK;
    }
    int64_t time_u = esp_timer_get_time();
    time_u += time_ofs;
    int64_t time_m = time_u / 1000;
    uint8_t buf_size = *remain_size;
    int res = snprintf(buf, buf_size, "t:%"PRId64"/", time_m);
    if (res >= buf_size || res < 0) {
        return ESP_FAIL;
    }
    *remain_size -= res;
    return ESP_OK;
}

static bool bt_task_send_msg(bt_task_msg_t *msg)
{
    if (msg == NULL) {
        return false;
    }
    if(bt_task_queue == NULL) {
        ESP_LOGE(TASK_TAG, "%s bt_task_queue is NULL", __func__);
        return false;
    }

    if (xQueueSend(bt_task_queue, msg, 10 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TASK_TAG, "%s xQueue send failed", __func__);
        return false;
    }
    return true;
}

static void bt_task_work_dispatched(bt_task_msg_t *msg)
{
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}

static char *bda2str(uint8_t * bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static bool get_name_from_eir(uint8_t *eir, char *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

static void esp_spp_cb(uint16_t e, void *p)
{
    esp_spp_cb_event_t event = e;
    esp_spp_cb_param_t *param = p;
    uint8_t i = 0;
    char bda_str[18] = {0};

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
            esp_spp_vfs_register();
        } else {
            ESP_LOGE(TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        if (param->disc_comp.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "ESP_SPP_DISCOVERY_COMP_EVT scn_num:%d", param->disc_comp.scn_num);
            for (i = 0; i < param->disc_comp.scn_num; i++) {
                ESP_LOGI(TAG, "-- [%d] scn:%d service_name:%s", i, param->disc_comp.scn[i],
                         param->disc_comp.service_name[i]);
            }
            /* We only connect to the first found server on the remote SPP acceptor here */
            esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], peer_bd_addr);
        } else {
            ESP_LOGE(TAG, "ESP_SPP_DISCOVERY_COMP_EVT failed status=%d", param->disc_comp.status);
            bt_task_work_dispatch((void *)bt_reconnect, 0, NULL, 0);
        }
        break;
    case ESP_SPP_OPEN_EVT:
        if (param->open.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "ESP_SPP_OPEN_EVT handle:%"PRIu32" fd:%d rem_bda:[%s]", param->open.handle, param->open.fd,
                     bda2str(param->open.rem_bda, bda_str, sizeof(bda_str)));
            bt_fd = param->open.fd;
            xTaskCreate(recv_server_time, "recv_server_time", 4096, &bt_fd, 1, NULL);
        } else {
            ESP_LOGE(TAG, "ESP_SPP_OPEN_EVT status:%d", param->open.status);
        }
        break;
    case ESP_SPP_CLOSE_EVT:
        bt_fd = -1;
        ESP_LOGW(TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%"PRIu32" close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        bt_task_work_dispatch((void *)bt_reconnect, 0, NULL, 0);
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        if (param->cl_init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT handle:%"PRIu32" sec_id:%d", param->cl_init.handle, param->cl_init.sec_id);
        } else {
            ESP_LOGE(TAG, "ESP_SPP_CL_INIT_EVT status:%d", param->cl_init.status);
        }
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT");
        break;
    case ESP_SPP_VFS_REGISTER_EVT:
        if (param->vfs_register.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(TAG, "ESP_SPP_VFS_REGISTER_EVT");
            esp_bt_dev_set_device_name(DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
        } else {
            ESP_LOGE(TAG, "ESP_SPP_VFS_REGISTER_EVT status:%d", param->vfs_register.status);
        }
        break;
    default:
        break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch(event){
    case ESP_BT_GAP_DISC_RES_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_DISC_RES_EVT");
        /* Find the target peer device name in the EIR data */
        for (int i = 0; i < param->disc_res.num_prop; i++){
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR
                && get_name_from_eir(param->disc_res.prop[i].val, peer_bdname, &peer_bdname_len)){
                ESP_LOGI(TAG, "device name: %s", peer_bdname);
                if (strlen(SERVER_NAME) == peer_bdname_len
                    && strncmp(peer_bdname, SERVER_NAME, peer_bdname_len) == 0) {
                    memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
                    /* Have found the target peer device, cancel the previous GAP discover procedure. And go on
                     * dsicovering the SPP service on the peer device */
                    esp_log_buffer_char(TAG, "Found target", 12);
                    esp_bt_gap_cancel_discovery();
                    esp_spp_start_discovery(peer_bd_addr);
                }
            }
        }
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        ESP_LOGD(TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVCS_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT - PIN code not required");
        ESP_LOGI("esp_bt_gap_cb", "Input pin code: XXXXXX");
esp_bt_pin_code_t pin_code;
pin_code[0] = 'X';
pin_code[1] = 'X';
pin_code[2] = 'X';
pin_code[3] = 'X';
pin_code[4] = 'X';
pin_code[5] = 'X';
esp_bt_gap_pin_reply(param->pin_req.bda, true, 6, pin_code);
        // esp_bt_gap_pin_reply(param->pin_req.bda, false, 0, NULL);
        break;
    }
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;

    default:
        break;
    }
}

static void esp_spp_stack_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    /* To avoid stucking Bluetooth stack, we dispatch the SPP callback event to the other lower priority task */
    bt_task_work_dispatch(esp_spp_cb, event, param, sizeof(esp_spp_cb_param_t));
}

static void bt_reconnect() {
    bt_fd = -1;
    time_ofs = -1;
    esp_log_buffer_char(TAG, "reconnecting...", 15);
    esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
}

static void recv_server_time(void *param) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "recv_server_time");
    int fd = *((int*)param);
    char buf[20] = {};
    uint8_t size = read(fd, buf, 19);
    ESP_LOGI(TAG, "size = %d", size);
    if (size <= 0 || size == 19) {
        ESP_LOGE(TAG, "receiving server time failed");
        bt_task_work_dispatch((void *)bt_reconnect, 0, NULL, 0);
    } else {
        buf[size] = '\0';
        int64_t time_u = esp_timer_get_time();
        ESP_LOGI(TAG, "time_u = %"PRId64, time_u);
        uint64_t server_time_u = strtoull(buf, NULL, 10);
        ESP_LOGI(TAG, "server_time_u = %"PRId64, server_time_u);
        time_ofs = server_time_u - time_u;
        ESP_LOGI(TAG, "time_ofs = %"PRId64, time_ofs);
    }
    vTaskDelete(NULL);
}