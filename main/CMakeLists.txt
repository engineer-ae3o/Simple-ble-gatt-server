idf_component_register(
                       SRCS "main.cpp" "ble.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES freertos bt nvs_flash
)
