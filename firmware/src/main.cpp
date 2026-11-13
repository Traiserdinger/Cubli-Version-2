// CubliV2 firmware entry point: hardware init in a safe order, then tasks.
//
//   core 1: control (1 kHz, paced by IMU frames)          priority 23
//   core 0: console 4, power 5, ui 3, telemetry 2, NimBLE host, esp_timer (deadman)
#include "app/console.h"
#include "app/control_task.h"
#include "app/power_task.h"
#include "app/shared.h"
#include "app/ui_task.h"
#include "comms/ble_nus.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/encoders.h"
#include "hal/imu_link.h"
#include "hal/ina219_dev.h"
#include "hal/motor_output.h"
#include "hal/nvs_store.h"

namespace {
hal::MotorOutput g_motors;
hal::Encoders g_encoders;
hal::ImuLink g_imu;
hal::Ina219 g_ina;
}  // namespace

extern "C" void app_main(void) {
    // 1. Motors safe FIRST: BRAKE low, PWM high (= stop), then LEDC at 100 % duty.
    g_motors.init();

    app::g_reset_reason.store(static_cast<uint32_t>(esp_reset_reason()));

    // 2. Parameters and calibration.
    hal::nvs_init();
    cubli::Params params;
    hal::nvs_load_params(params);
    app::g_params.init(params);
    cubli::CalibrationData calib;
    if (!hal::nvs_load_calib(calib)) calib = cubli::CalibrationData{};
    app::g_calib.write(calib);

    // 3. Peripherals.
    g_encoders.init();
    g_ina.init();
    g_imu.init();
    app::g_cmd_queue = xQueueCreate(16, sizeof(app::ControlCommand));

    // 4. Tasks.
    app::start_console_task();
    app::start_power_task(&g_ina);
    app::start_ui_task();
    app::start_telemetry_task();
    app::start_control_task(&g_motors, &g_encoders, &g_imu, calib);
    comms::ble_nus_start("CubliV2");
}
