# Single application component: ESP-IDF code in src/ plus the
# hardware-independent libraries in lib/ (also unit-tested on the PC).
file(GLOB_RECURSE APP_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/*.cpp
    ${CMAKE_CURRENT_LIST_DIR}/../lib/*/*.cpp)

idf_component_register(
    SRCS ${APP_SOURCES}
    INCLUDE_DIRS
        "."
        "../include"
        "../lib/core"
        "../lib/estimation"
        "../lib/control"
        "../lib/logic"
    REQUIRES
        driver esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_driver_uart
        esp_driver_i2c esp_timer nvs_flash bt esp_system)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall -Wextra -Werror -Wshadow -Wdouble-promotion -Wno-missing-field-initializers)
