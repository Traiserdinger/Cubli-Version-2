#pragma once

namespace hal {
class Ina219;
}

namespace app {
void start_power_task(hal::Ina219* ina);
}
