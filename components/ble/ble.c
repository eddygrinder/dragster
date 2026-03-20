#include "ble.h"
#include "tuning.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "os/os_mbuf.h"
#include <string.h>

#define TAG "BLE"
static uint8_t own_addr_type;

static void host_task(void *param);
static void ble_on_sync(void);
static int kp_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int base_speed_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_svr_init(void);

static int kp_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR && ctxt->om->om_len >= 2)
    {
        uint16_t raw = (ctxt->om->om_data[0] << 8) | ctxt->om->om_data[1];
        tuning.KP = raw;
        ESP_LOGI(TAG, "KP updated: %d", tuning.KP);
        tuning_save();
        tuning_print_saved();
    }
    else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", tuning.KP);
        os_mbuf_append(ctxt->om, buf, strlen(buf));
    }
    return 0;
}

static int base_speed_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR && ctxt->om->om_len >= 2)
    {
        uint16_t raw = (ctxt->om->om_data[0] << 8) | ctxt->om->om_data[1];
        tuning.BASE_SPEED = raw;
        ESP_LOGI(TAG, "BASE_SPEED updated: %d", tuning.BASE_SPEED);
        tuning_save();
        tuning_print_saved();
    }
    else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", tuning.BASE_SPEED);
        os_mbuf_append(ctxt->om, buf, strlen(buf));
    }
    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(0x04, 0xa4, 0xc3, 0x5f, 0xef, 0xba, 0x6f, 0xae, 0xa7, 0x43, 0xff, 0x43, 0x92, 0x9e, 0x68, 0x07),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid      = BLE_UUID128_DECLARE(0x4f, 0x46, 0x90, 0x8f, 0x77, 0x46, 0x42, 0x9f, 0xa7, 0x5c, 0xcc, 0x71, 0x2e, 0x14, 0x59, 0xc9),
                .access_cb = kp_chr_access_cb,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
            },
            {
                .uuid      = BLE_UUID128_DECLARE(0x6a, 0x7b, 0x90, 0x8f, 0x77, 0x46, 0x42, 0x9f, 0xa7, 0x5c, 0xcc, 0x71, 0x2e, 0x14, 0x59, 0xca),
                .access_cb = base_speed_chr_access_cb,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
            },
            {0}
        },
    },
    {0}
};

static int gatt_svr_init(void)
{
    int rc;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "count_cfg erro: %d", rc); return rc; }

    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "add_svcs erro: %d", rc); return rc; }

    ESP_LOGI(TAG, "GATT services registados OK!");
    return 0;
}

static void ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) { ESP_LOGE(TAG, "Address infer failed; rc=%d", rc); return; }

    rc = gatt_svr_init();
    if (rc != 0) { ESP_LOGE(TAG, "gatt_svr_init() falhou: %d", rc); return; }

    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags           = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char *name       = "DRAGSTER";
    fields.name            = (uint8_t *)name;
    fields.name_len        = strlen(name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ESP_LOGI(TAG, "Starting advertising...");
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
}

void ble_stop_advertising(void)
{
    ble_gap_adv_stop();
}

void ble_restart_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
}

static void host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nimble_port_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svcs);
    assert(rc == 0);

    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_hs_cfg.reset_cb = NULL;

    nimble_port_freertos_init(host_task);
}