#include "comms/ble_nus.h"

#include <atomic>
#include <cstring>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace comms {
namespace {

constexpr const char* kTag = "ble";

// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E (little-endian byte order)
const ble_uuid128_t kSvcUuid = BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3,
                                                0xB5, 0x01, 0x00, 0x40, 0x6E);
const ble_uuid128_t kRxUuid = BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3,
                                               0xB5, 0x02, 0x00, 0x40, 0x6E);
const ble_uuid128_t kTxUuid = BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3,
                                               0xB5, 0x03, 0x00, 0x40, 0x6E);

uint16_t g_tx_handle = 0;
std::atomic<uint16_t> g_conn{BLE_HS_CONN_HANDLE_NONE};
std::atomic<bool> g_notify{false};
std::atomic<uint16_t> g_mtu{23};
uint8_t g_own_addr_type = 0;
QueueHandle_t g_lines = nullptr;
char g_rx_line[kBleLineMax];
size_t g_rx_len = 0;
char g_name[24] = "CubliV2";

struct Line {
    char text[kBleLineMax];
};

void advertise();

int rx_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;
    uint8_t buf[256];
    uint16_t len = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof buf, &len) != 0) return BLE_ATT_ERR_UNLIKELY;
    for (uint16_t i = 0; i < len; ++i) {
        const char c = static_cast<char>(buf[i]);
        if (c == '\r' || c == '\n') {
            if (g_rx_len > 0) {
                Line l;
                std::memcpy(l.text, g_rx_line, g_rx_len);
                l.text[g_rx_len] = '\0';
                xQueueSend(g_lines, &l, 0);
                g_rx_len = 0;
            }
        } else if (g_rx_len < kBleLineMax - 1) {
            g_rx_line[g_rx_len++] = c;
        }
    }
    // No flush without a line ending: with the default 23-byte MTU a command
    // arrives in several writes. Terminal apps must send CR and/or LF.
    return 0;
}

int tx_access(uint16_t, uint16_t, struct ble_gatt_access_ctxt*, void*) { return 0; }

const struct ble_gatt_chr_def kChars[] = {
    {.uuid = &kRxUuid.u, .access_cb = rx_access, .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP},
    {.uuid = &kTxUuid.u, .access_cb = tx_access, .flags = BLE_GATT_CHR_F_NOTIFY, .val_handle = &g_tx_handle},
    {},
};

const struct ble_gatt_svc_def kServices[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &kSvcUuid.u, .characteristics = kChars},
    {},
};

int gap_event(struct ble_gap_event* event, void*) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                g_conn.store(event->connect.conn_handle);
            } else {
                advertise();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            g_conn.store(BLE_HS_CONN_HANDLE_NONE);
            g_notify.store(false);
            g_mtu.store(23);
            advertise();
            break;
        case BLE_GAP_EVENT_ADV_COMPLETE: advertise(); break;
        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == g_tx_handle) g_notify.store(event->subscribe.cur_notify != 0);
            break;
        case BLE_GAP_EVENT_MTU: g_mtu.store(event->mtu.value); break;
        default: break;
    }
    return 0;
}

void advertise() {
    struct ble_hs_adv_fields f = {};
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    f.name = reinterpret_cast<const uint8_t*>(g_name);
    f.name_len = static_cast<uint8_t>(std::strlen(g_name));
    f.name_is_complete = 1;
    f.uuids128 = &kSvcUuid;
    f.num_uuids128 = 1;
    f.uuids128_is_complete = 1;
    if (ble_gap_adv_set_fields(&f) != 0) {
        // Name too long for one packet: advertise the UUID only.
        f.name = nullptr;
        f.name_len = 0;
        ble_gap_adv_set_fields(&f);
    }
    struct ble_gap_adv_params p = {};
    p.conn_mode = BLE_GAP_CONN_MODE_UND;
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(g_own_addr_type, nullptr, BLE_HS_FOREVER, &p, gap_event, nullptr);
}

void on_sync() {
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &g_own_addr_type);
    advertise();
}

void on_reset(int reason) { ESP_LOGW(kTag, "reset %d", reason); }

void host_task(void*) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

}  // namespace

bool ble_nus_start(const char* device_name) {
    if (device_name) {
        std::strncpy(g_name, device_name, sizeof(g_name) - 1);
        g_name[sizeof(g_name) - 1] = '\0';
    }
    g_lines = xQueueCreate(8, sizeof(Line));
    if (nimble_port_init() != ESP_OK) return false;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    if (ble_gatts_count_cfg(kServices) != 0 || ble_gatts_add_svcs(kServices) != 0) return false;
    ble_svc_gap_device_name_set(g_name);
    nimble_port_freertos_init(host_task);
    return true;
}

bool ble_nus_connected() { return g_conn.load() != BLE_HS_CONN_HANDLE_NONE && g_notify.load(); }

void ble_nus_send(const char* data, size_t len) {
    if (!ble_nus_connected()) return;
    const size_t chunk_max = g_mtu.load() > 3 ? g_mtu.load() - 3u : 20u;
    while (len > 0) {
        const size_t n = len < chunk_max ? len : chunk_max;
        int rc = -1;
        for (int attempt = 0; attempt < 50 && ble_nus_connected(); ++attempt) {
            struct os_mbuf* om = ble_hs_mbuf_from_flat(data, static_cast<uint16_t>(n));
            if (!om) {
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            rc = ble_gatts_notify_custom(g_conn.load(), g_tx_handle, om);
            if (rc == 0) break;
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        if (rc != 0) return;  // give up on this message
        data += n;
        len -= n;
    }
}

bool ble_nus_read_line(char* out, size_t cap) {
    if (!g_lines) return false;
    Line l;
    if (xQueueReceive(g_lines, &l, 0) != pdTRUE) return false;
    std::strncpy(out, l.text, cap - 1);
    out[cap - 1] = '\0';
    return true;
}

}  // namespace comms
