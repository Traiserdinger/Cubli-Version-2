// The 1 kHz control task (core 1, highest application priority). It is paced by
// the IMU PCB frames (frame v2 at 1 kHz); without frames it still runs every
// 2 ms so that safety logic and bench commands keep working.
#pragma once

#include "calibration.h"

namespace hal {
class MotorOutput;
class Encoders;
class ImuLink;
}  // namespace hal

namespace app {

void start_control_task(hal::MotorOutput* motors, hal::Encoders* enc, hal::ImuLink* imu,
                        const cubli::CalibrationData& calib);

}  // namespace app
