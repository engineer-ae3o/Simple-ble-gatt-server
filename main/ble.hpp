#ifndef _BLE_HPP_
#define _BLE_HPP_


#include "esp_err.h"


namespace ble {

    /**
     * @brief Initializes the ble interface
     * 
     * @return ESP_OK on success, error code otherwise
     * 
     * @note This also initializes nvs flash
     */
    esp_err_t init(void);
    
    /**
     * @brief Deinitializes the ble interface
     * 
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t deinit(void);
    
    /**
     * @brief Start BLE advertising
     * 
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t start(void);

    /**
     * @brief Stops BLE advertising
     * 
     * @return ESP_OK on success, error code otherwise
     */
    esp_err_t stop(void);

} // namespace ble


#endif // _BLE_HPP_