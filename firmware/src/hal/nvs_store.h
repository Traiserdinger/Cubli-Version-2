// Parameter and calibration persistence in NVS (namespace "cubli").
// NOTE: flash writes stall both CPU cores; never save while the motors run.
#pragma once

#include "calibration.h"
#include "params.h"

namespace hal {

bool nvs_init();
/// Returns the number of stored values applied (0: nothing stored, -1: corrupt).
int nvs_load_params(cubli::Params& p);
bool nvs_save_params(const cubli::Params& p);
bool nvs_load_calib(cubli::CalibrationData& c);
bool nvs_save_calib(const cubli::CalibrationData& c);
bool nvs_erase_all();

}  // namespace hal
