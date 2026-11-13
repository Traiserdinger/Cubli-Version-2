#include "hal/nvs_store.h"

#include "nvs.h"
#include "nvs_flash.h"

namespace hal {
namespace {
constexpr const char* kNamespace = "cubli";
constexpr const char* kKeyParams = "params";
constexpr const char* kKeyCalib = "calib";
uint8_t g_blob[2048];

bool save_blob(const char* key, const uint8_t* data, size_t len) {
    nvs_handle_t h;
    if (nvs_open(kNamespace, NVS_READWRITE, &h) != ESP_OK) return false;
    bool ok = nvs_set_blob(h, key, data, len) == ESP_OK && nvs_commit(h) == ESP_OK;
    nvs_close(h);
    return ok;
}

size_t load_blob(const char* key, uint8_t* data, size_t cap) {
    nvs_handle_t h;
    if (nvs_open(kNamespace, NVS_READONLY, &h) != ESP_OK) return 0;
    size_t len = cap;
    const esp_err_t e = nvs_get_blob(h, key, data, &len);
    nvs_close(h);
    return e == ESP_OK ? len : 0;
}
}  // namespace

bool nvs_init() {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        e = nvs_flash_init();
    }
    return e == ESP_OK;
}

int nvs_load_params(cubli::Params& p) {
    const size_t n = load_blob(kKeyParams, g_blob, sizeof g_blob);
    if (n == 0) {
        cubli::params_set_defaults(p);
        return 0;
    }
    return cubli::params_deserialize(p, g_blob, n);
}

bool nvs_save_params(const cubli::Params& p) {
    const size_t n = cubli::params_serialize(p, g_blob, sizeof g_blob);
    return n > 0 && save_blob(kKeyParams, g_blob, n);
}

bool nvs_load_calib(cubli::CalibrationData& c) {
    const size_t n = load_blob(kKeyCalib, g_blob, sizeof g_blob);
    return n > 0 && cubli::calib_deserialize(c, g_blob, n);
}

bool nvs_save_calib(const cubli::CalibrationData& c) {
    const size_t n = cubli::calib_serialize(c, g_blob, sizeof g_blob);
    return n > 0 && save_blob(kKeyCalib, g_blob, n);
}

bool nvs_erase_all() {
    nvs_handle_t h;
    if (nvs_open(kNamespace, NVS_READWRITE, &h) != ESP_OK) return false;
    const bool ok = ::nvs_erase_all(h) == ESP_OK && nvs_commit(h) == ESP_OK;
    nvs_close(h);
    return ok;
}

}  // namespace hal
