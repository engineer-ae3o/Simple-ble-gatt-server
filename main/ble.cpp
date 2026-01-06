#include "ble.hpp"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "os/os_mbuf.h"

#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"

#include <cstdint>
#include <array>


#define BLE_DEBUG 1

#if BLE_DEBUG == 1
static const char* TAG = "BLE";
#define BLE_LOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#define BLE_LOGW(...) ESP_LOGW(TAG, __VA_ARGS__)
#define BLE_LOGE(...) ESP_LOGE(TAG, __VA_ARGS__)
#else 
#define BLE_LOGI(...)
#define BLE_LOGW(...)
#define BLE_LOGE(...)
#endif


// Set to 0 if nvs flash hasn't been initialized at the time of calling `ble::init()`
#define NVS_ALREADY_INITIALIZED                 0

// Set to 1 if you want `ble::deinit()` to deinitialize nvs flash
#define DEINIT_NVS_FROM_BLE_DEINIT              0

// We declare this function because the devs thought it was a good idea to not
// declare it in a header file. Maybe I didn't search hard enough
extern "C"{ void ble_store_config_init(void); }

// Pardon my type aliases, but i prefer suffixing my struct names with `_t`
// It's easier to work with. If you're wondering why i didn't do for all,
// it's because they're too many
using ble_gatt_svc_def_t = struct ble_gatt_svc_def;
using ble_gatt_chr_def_t = struct ble_gatt_chr_def;
using ble_gatt_access_ctxt_t = struct ble_gatt_access_ctxt;
using ble_gap_event_t = struct ble_gap_event;
using os_mbuf_t = struct os_mbuf;


namespace ble {
    
    // Connection context
    struct connection_context_t {
        bool is_advertising;
        bool is_connected;
        uint8_t address_type;
        uint16_t connection_handle;
        // Handles for all characteristics. Needed for notifications
        uint16_t temp_chr_handle;
        uint16_t hmdt_chr_handle;
        uint16_t voltage_chr_handle;
        uint16_t current_chr_handle;
        uint16_t power_chr_handle;
        uint16_t battery_soc_chr_handle;
        uint16_t runtime_chr_handle;

        void clear_all() {
            is_advertising = false;
            is_connected = false;
            address_type = 0;
            connection_handle = 0;
            temp_chr_handle = 0;
            hmdt_chr_handle = 0;
            voltage_chr_handle = 0;
            current_chr_handle = 0;
            power_chr_handle = 0;
            battery_soc_chr_handle = 0;
            runtime_chr_handle = 0;
        }
    };

    static connection_context_t connection_context{};

    class chr_notify_t {
    private:
        bool chr_notify_state[7];

    public:
        enum class chr_t : uint8_t {
            TEMPERATURE = 0,
            HUMIDITY,
            VOLTAGE,
            CURRENT,
            POWER,
            BATT_SoC,
            RUNTIME_S
        };

        chr_notify_t() {
            for (auto ele : chr_notify_state) {
                chr_notify_state[ele] = false;
            }
        }

        void set_chr_notify_state(chr_t chr, bool state = true) {
            chr_notify_state[static_cast<size_t>(chr)] = state;
        }

        [[nodiscard]] bool get_chr_notify_state(chr_t chr) const {
            return chr_notify_state[static_cast<size_t>(chr)];
        }

        void set_all_chr_notify_state(bool state = true) {
            for (auto ele : chr_notify_state) {
                chr_notify_state[ele] = state;
            }
        }

        // If you are using a generic BLE app like the nRF Connect or some other app, it
        // will expect data in the format int16_t and an exponent of -2, so we multiply
        // by 100 and cast to int16_t. The app receives the bytes, divides by 100 and gets
        // its data and does whatever with it. Also, if you are sending to a big endian system,
        // swap the bytes before sending using `__builtin_bswap16()` if you are on gcc
        template <typename T>
        esp_err_t send_notification(T val, uint16_t chr_handle, const char* name) {
            int16_t data = static_cast<int16_t>(val * 100);
            os_mbuf_t* om = ble_hs_mbuf_from_flat(&data, sizeof(data));
            if (om && (chr_handle != 0)) {
                int rc = ble_gatts_notify_custom(connection_context.connection_handle, chr_handle, om);
                if (rc == 0) {
                    BLE_LOGI("%s notification sent successfully", name);
                    return ESP_OK;
                } else {
                    BLE_LOGE("Failed to send %s notification: %d", name, rc);
                    return ESP_FAIL;
                }
            } else {
                BLE_LOGW("Invalid %s handle or mbuf allocation failed", name);
                return ESP_FAIL;
            }
        }
    };

    static chr_notify_t chr_notify{};

    constexpr float get_temperature() {
        return 35.0;
    }
    constexpr float get_humidity() {
        return 74.0;
    }
    constexpr float get_voltage() {
        return 11.3;
    }
    constexpr float get_current() {
        return 13.2;
    }
    constexpr float get_power() {
        return (get_voltage() * get_current());
    }
    constexpr float get_battery_soc() {
        return 56.2;
    }
    constexpr uint64_t get_runtime() {
        return static_cast<uint64_t>(2.4 * 3600);
    }


    // Device name
    static constexpr const char BLE_GAP_NAME[]                       = "Test";

    // 16 bit UUIDs for all services and characteristics
    static constexpr ble_uuid16_t AHT_SERVICE_UUID                   = { .u = { .type = BLE_UUID_TYPE_16 }, .value = 0x181A };
    static constexpr ble_uuid16_t TEMPERATURE_CHAR_UUID              = { .u = { .type = BLE_UUID_TYPE_16 }, .value = 0x2A6E };
    static constexpr ble_uuid16_t HUMIDITY_CHAR_UUID                 = { .u = { .type = BLE_UUID_TYPE_16 }, .value = 0x2A6F };

    static constexpr ble_uuid16_t ADC_SERVICE_UUID                   = { .u = { .type = BLE_UUID_TYPE_16 }, .value = 0x181F };
    static constexpr ble_uuid16_t VOLTAGE_CHAR_UUID                  = { .u = { .type = BLE_UUID_TYPE_16 }, .value = 0x2B18 };
    static constexpr ble_uuid16_t CURRENT_CHAR_UUID                  = { .u = { .type = BLE_UUID_TYPE_16 }, .value = 0x2AEE };
    static constexpr ble_uuid16_t POWER_CHAR_UUID                    = { .u = { .type = BLE_UUID_TYPE_16 }, .value = 0x2B05 };

    static constexpr ble_uuid16_t BATTERY_SERVICE_UUID               = { .u = { .type = BLE_UUID_TYPE_16 }, .value = 0x180F };
    static constexpr ble_uuid16_t SoC_CHAR_UUID                      = { .u = { .type = BLE_UUID_TYPE_16 }, .value = 0x2A19 };
    static constexpr ble_uuid16_t RUNTIME_CHAR_UUID                  = { .u = { .type = BLE_UUID_TYPE_16 }, .value = 0x2B2E };


    // Forward declarations
    static int gatt_svr_init();
    static void ble_advertise();
    static int ble_event_handler(ble_gap_event_t* event, void* arg);
    

    static constexpr struct ble_gatt_chr_def aht_svc_chrs[] = {
        {
            .uuid = &TEMPERATURE_CHAR_UUID.u,
            .access_cb = [](uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt_t* ctxt, void* arg) {
                switch (ctxt->op) {
                case BLE_GATT_ACCESS_OP_READ_CHR: {
                    constexpr int16_t temperature = static_cast<int16_t>(get_temperature() * 100);
                    return os_mbuf_append(ctxt->om, &temperature, sizeof(temperature));
                }
                // Characteristics is read only
                case BLE_GATT_ACCESS_OP_WRITE_CHR:
                    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
                default:
                    return BLE_ATT_ERR_UNLIKELY;
                }
                return BLE_ATT_ERR_UNLIKELY;
            },
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &connection_context.temp_chr_handle
        },
        {
            .uuid = &HUMIDITY_CHAR_UUID.u,
            .access_cb = [](uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt_t* ctxt, void* arg) {
                switch (ctxt->op) {
                case BLE_GATT_ACCESS_OP_READ_CHR: {
                    constexpr int16_t humidity = static_cast<int16_t>(get_humidity() * 100);
                    return os_mbuf_append(ctxt->om, &humidity, sizeof(humidity));
                }
                // Characteristics is read only
                case BLE_GATT_ACCESS_OP_WRITE_CHR:
                    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
                default:
                    return BLE_ATT_ERR_UNLIKELY;
                }
                return BLE_ATT_ERR_UNLIKELY;
            },
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &connection_context.hmdt_chr_handle
        },
        // AHT characteristics array termination
        {}
    };

    static constexpr struct ble_gatt_chr_def adc_svc_chrs[] = {
        {
            .uuid = &VOLTAGE_CHAR_UUID.u,
            .access_cb = [](uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt_t* ctxt, void* arg) {
                switch (ctxt->op) {
                case BLE_GATT_ACCESS_OP_READ_CHR: {
                    constexpr int16_t voltage = static_cast<int16_t>(get_voltage() * 100);
                    return os_mbuf_append(ctxt->om, &voltage, sizeof(voltage));
                }
                // Characteristics is read only
                case BLE_GATT_ACCESS_OP_WRITE_CHR:
                    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
                default:
                    return BLE_ATT_ERR_UNLIKELY;
                }
                return BLE_ATT_ERR_UNLIKELY;
            },
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &connection_context.voltage_chr_handle
        },
        {
            .uuid = &CURRENT_CHAR_UUID.u,
            .access_cb = [](uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt_t* ctxt, void* arg) {
                switch (ctxt->op) {
                case BLE_GATT_ACCESS_OP_READ_CHR: {
                    constexpr int16_t current = static_cast<int16_t>(get_current() * 100);
                    return os_mbuf_append(ctxt->om, &current, sizeof(current));
                }
                // Characteristics is read only
                case BLE_GATT_ACCESS_OP_WRITE_CHR:
                    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
                default:
                    return BLE_ATT_ERR_UNLIKELY;
                }
                return BLE_ATT_ERR_UNLIKELY;
            },
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &connection_context.current_chr_handle
        },
        {
            .uuid = &POWER_CHAR_UUID.u,
            .access_cb = [](uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt_t* ctxt, void* arg) {
                switch (ctxt->op) {
                case BLE_GATT_ACCESS_OP_READ_CHR: {
                    constexpr int16_t power = static_cast<int16_t>(get_power() * 100);
                    return os_mbuf_append(ctxt->om, &power, sizeof(power));
                }
                // Characteristics is read only
                case BLE_GATT_ACCESS_OP_WRITE_CHR:
                    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
                default:
                    return BLE_ATT_ERR_UNLIKELY;
                }
                return BLE_ATT_ERR_UNLIKELY;
            },
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &connection_context.power_chr_handle
        },
        // AHT characteristics array termination
        {}
    };

    static constexpr struct ble_gatt_chr_def batt_svc_chrs[] = {
        {
            .uuid = &SoC_CHAR_UUID.u,
            .access_cb = [](uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt_t* ctxt, void* arg) {
                switch (ctxt->op) {
                case BLE_GATT_ACCESS_OP_READ_CHR: {
                    constexpr int16_t battery_soc = static_cast<int16_t>(get_battery_soc() * 100);
                    return os_mbuf_append(ctxt->om, &battery_soc, sizeof(battery_soc));
                }
                // Characteristics is read only
                case BLE_GATT_ACCESS_OP_WRITE_CHR:
                    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
                default:
                    return BLE_ATT_ERR_UNLIKELY;
                }
                return BLE_ATT_ERR_UNLIKELY;
            },
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &connection_context.battery_soc_chr_handle
        },
        {
            .uuid = &RUNTIME_CHAR_UUID.u,
            .access_cb = [](uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt_t* ctxt, void* arg) {
                switch (ctxt->op) {
                case BLE_GATT_ACCESS_OP_READ_CHR: {
                    constexpr int16_t runtime_s = static_cast<int16_t>(get_runtime() * 100);
                    return os_mbuf_append(ctxt->om, &runtime_s, sizeof(runtime_s));
                }
                // Characteristics is read only
                case BLE_GATT_ACCESS_OP_WRITE_CHR:
                    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
                default:
                    return BLE_ATT_ERR_UNLIKELY;
                }
                return BLE_ATT_ERR_UNLIKELY;
            },
            .arg = nullptr,
            .descriptors = nullptr,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &connection_context.runtime_chr_handle
        },
        // Battery characteristics array termination
        {}
    };

    static constexpr ble_uuid16_t uuids[] = { AHT_SERVICE_UUID, ADC_SERVICE_UUID, BATTERY_SERVICE_UUID };

    static constexpr struct ble_gatt_svc_def gatt_svc[] = {
        // AHT service
        {
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = &AHT_SERVICE_UUID.u,
            .includes = nullptr,
            .characteristics = aht_svc_chrs
        },
        // ADC service
        {
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = &ADC_SERVICE_UUID.u,
            .includes = nullptr,
            .characteristics = adc_svc_chrs
        },
        // Battery service
        {
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = &BATTERY_SERVICE_UUID.u,
            .includes = nullptr,
            .characteristics = batt_svc_chrs
        },
        // GATT services termination
        {}
    };


    // Public APIs
    esp_err_t init() {

        esp_err_t ret = ESP_OK;

#if NVS_ALREADY_INITIALIZED == 0
    // Initialize the NVS flash partition
        ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        if (ret != ESP_OK) {
            BLE_LOGE("Failed to initialize nvs flash: %s", esp_err_to_name(ret));
            return ret;
        }
#endif

        // Initialize controller and nimble host stack
        ret = nimble_port_init();
        if (ret != ESP_OK) {
            BLE_LOGE("Failed to initialize controller and nimble host stack: %s", esp_err_to_name(ret));
            return ret;
        }

        // BLE Host settings
        // @brief This is called when the the host and controller get synced
        // Determines the best address_type type to use for automatic address_type type resolution
        // @note We don't start advertising immediately as that is up to the user
        ble_hs_cfg.sync_cb = []() {

            int ret = ble_hs_id_infer_auto(0, &connection_context.address_type);
            if (ret != 0) {
                BLE_LOGE("Error getting address type");
            }
        
            uint8_t addr_val[6]{};
            ret = ble_hs_id_copy_addr(connection_context.address_type, addr_val, nullptr);
            if (ret != 0) {
                BLE_LOGE("Error getting device address value");
            }
        
            BLE_LOGI("Device Address: %X:%X:%X:%X:%X:%X",
                     addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);
            
            // Begin advertising
            ble_advertise();
            
            BLE_LOGI("Host and controller now synced");
        };

        // This is called when the host and controller get reset due to a fatal error
        ble_hs_cfg.reset_cb = [](int reason) {
            BLE_LOGE("Fatal error: Host and controller reset. Reason = %d", reason);
        };

        // Security settings
        // Sets flag for generating keys for bonding/pairing
        ble_hs_cfg.sm_bonding = 1;

        // Local key distribution mask: long term key and identity resolving key
        ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

        // User key distribution mask: same as above
        ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

        // Secure connections
        ble_hs_cfg.sm_sc = 1;

        // Man in the middle
        ble_hs_cfg.sm_mitm = 1;

        // Auto pairing
        ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;

        // This gets called when a persistence operation cannot be performed
        ble_hs_cfg.store_status_cb = [](struct ble_store_status_event* event, void* arg) {

            switch (event->event_code) {
            // Event for overflows
            case BLE_STORE_EVENT_OVERFLOW:
                switch (event->overflow.obj_type) {
            case BLE_STORE_OBJ_TYPE_OUR_SEC:
            case BLE_STORE_OBJ_TYPE_PEER_SEC:
            case BLE_STORE_OBJ_TYPE_PEER_ADDR:
                return ble_gap_unpair_oldest_peer();
            case BLE_STORE_OBJ_TYPE_CCCD:
                // Try unpairing oldest peer except current peer
                return ble_gap_unpair_oldest_except(&event->overflow.value->cccd.peer_addr);
            default:
            return BLE_HS_EUNKNOWN;
        }
            // Event in the case of an overflow
            case BLE_STORE_EVENT_FULL:
                    BLE_LOGW("BLE store event likely to end in failure. Connection handle = %d, Object type = %d",
                              event->full.conn_handle, event->full.obj_type);
                    return 0;
                    
            default:
                BLE_LOGW("Unknown BLE store status event occurred. Event code = %d", event->event_code);
                return BLE_HS_EUNKNOWN;
            }
        };

        // Called when a GATTS descriptor, service or characteristics is registered
        ble_hs_cfg.gatts_register_cb = [](struct ble_gatt_register_ctxt* ctxt, void* arg) {

            char buf[64]{};

            switch (ctxt->op) {
            case BLE_GATT_REGISTER_OP_SVC:
                BLE_LOGI("Registered GATTS service %s with handle = %d",
                         ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf), ctxt->svc.handle);
                break;
            case BLE_GATT_REGISTER_OP_CHR:
                BLE_LOGI("Registering GATTS characteristic %s with def_handle = %d and val_handle = %d",
                         ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf), ctxt->chr.def_handle, ctxt->chr.val_handle);
                break;
            case BLE_GATT_REGISTER_OP_DSC:
                BLE_LOGI("Registering GATTS descriptor %s with handle = %d",
                         ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
                break;
            default:
                BLE_LOGW("Invalid ble gatts registration event");
                break;
            }
        };

        // Initialize NVS backed persistent storage for bonds
        ble_store_config_init();

        // Initialize GATT server and GAP services
        int rc = gatt_svr_init();
        if (rc != 0) {
            BLE_LOGE("Failed to set initialize gatt server");
            return ESP_FAIL;
        }

        // Set GAP device name
        rc = ble_svc_gap_device_name_set(BLE_GAP_NAME);
        if (rc != 0) {
            BLE_LOGE("Failed to set device GAP name");
            return ESP_FAIL;
        }

        // Start nimble freertos task
        nimble_port_freertos_init([](void* arg) {
            BLE_LOGI("NimBLE task started");
            nimble_port_run();
            nimble_port_freertos_deinit();
        });

        return ret;
    }
    
    esp_err_t deinit() {

        connection_context.clear_all();

        // Stop nimble freertos task
        int rc = nimble_port_stop();
        if (rc != 0) {
            BLE_LOGE("Failed to stop nimble port task");
            return ESP_FAIL;
        }

        // Deinitialize nimble freertos port
        nimble_port_freertos_deinit();

        // Deinitialize controller and nimble's host stack
        esp_err_t ret = nimble_port_deinit();
        if (ret != ESP_OK) {
            BLE_LOGE("Failed to deinitialize controller and nimble host stack: %s", esp_err_to_name(ret));
            return ret;
        }

#if DEINIT_NVS_FROM_BLE_DEINIT == 1
        // Deinitialize NVS storage partition
        ret = nvs_flash_deinit();
        if (ret != ESP_OK) {
            BLE_LOGE("Failed to deinitialize nvs flash: %s", esp_err_to_name(ret));
            return ret;
        }
#endif
        
        return ret;
    }

    esp_err_t start() {

        if (connection_context.is_advertising) {
            BLE_LOGW("Device already advertising");
            return ESP_ERR_INVALID_STATE;
        }

        ble_advertise();
        if (!connection_context.is_advertising) {
            BLE_LOGE("Failed to start BLE advertising");
            return ESP_FAIL;
        }

        BLE_LOGI("Advertising started");

        return ESP_OK;
    }

    esp_err_t stop() {

        if (!connection_context.is_advertising) {
            BLE_LOGW("Device not advertising");
            return ESP_ERR_INVALID_STATE;
        }

        int ret = ble_gap_adv_stop();
        if (ret == BLE_HS_EALREADY) {
            BLE_LOGW("Device not advertising");
        } else if (ret != 0) {
            BLE_LOGE("Failed to stop advertising: %d", ret);
            return ESP_FAIL;
        }
        connection_context.is_advertising = false;

        BLE_LOGI("Advertising stopped");

        return ESP_OK;
    }

    // Static helpers
    static int gatt_svr_init() {

        ble_svc_gap_init();
        ble_svc_gatt_init();

        // Update GATT services counter
        int ret = ble_gatts_count_cfg(gatt_svc);
        if (ret != 0) {
            BLE_LOGE("Failed to update gatt services counter");
            return ret;
        }

        // Queues service definitions for registration
        ret = ble_gatts_add_svcs(gatt_svc);
        if (ret != 0) {
            BLE_LOGE("Failed to queue service definitions for registration");
            return ret;
        }

        return 0;
    }

    static void ble_advertise() {

        struct ble_gap_adv_params adv_params{};
        struct ble_hs_adv_fields fields{};
        int rc = 0;

        fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

        fields.tx_pwr_lvl_is_present = 1;
        fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

        fields.name = (uint8_t*)BLE_GAP_NAME;
        fields.name_len = strlen(BLE_GAP_NAME);
        fields.name_is_complete = 1;

        fields.uuids16 = uuids;
        fields.num_uuids16 = 3;
        fields.uuids16_is_complete = 1;

        // Configure advertisement field
        rc = ble_gap_adv_set_fields(&fields);
        if (rc != 0) {
            BLE_LOGE("Error setting advertisement data: %d", rc);
            return;
        }

        // Begin advertising
        adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
        adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
        rc = ble_gap_adv_start(connection_context.address_type, nullptr, BLE_HS_FOREVER, &adv_params, ble_event_handler, nullptr);
        if (rc != 0) {
            BLE_LOGE("Error enabling advertisement: %d", rc);
            return;
        }

        connection_context.is_advertising = true;
    }

    static int ble_event_handler(ble_gap_event_t* event, void* arg) {

        switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                connection_context.is_connected = true;
                // Store connection handle to use for notifications
                connection_context.connection_handle = event->connect.conn_handle;
                BLE_LOGI("Connection established. Connection handle = %u", event->connect.conn_handle);
            } else {
                BLE_LOGE("Connection failed. Resuming advertising");
                ble_advertise();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            connection_context.is_connected = false;
            chr_notify.set_all_chr_notify_state(false);
            connection_context.connection_handle = BLE_HS_CONN_HANDLE_NONE;
            BLE_LOGI("Device disonnected");
            // Resume advertising
            ble_advertise();
            break;
        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == connection_context.temp_chr_handle) {
                chr_notify.set_chr_notify_state(chr_notify_t::chr_t::TEMPERATURE);
                BLE_LOGI("Client subscribed to temperature characteristic");
            } else if (event->subscribe.attr_handle == connection_context.hmdt_chr_handle) {
                chr_notify.set_chr_notify_state(chr_notify_t::chr_t::HUMIDITY);
                BLE_LOGI("Client subscribed to humidity characteristic");
            } else if (event->subscribe.attr_handle == connection_context.voltage_chr_handle) {
                chr_notify.set_chr_notify_state(chr_notify_t::chr_t::VOLTAGE);
                BLE_LOGI("Client subscribed to voltage characteristic");
            } else if (event->subscribe.attr_handle == connection_context.current_chr_handle) {
                chr_notify.set_chr_notify_state(chr_notify_t::chr_t::CURRENT);
                BLE_LOGI("Client subscribed to current characteristic");
            } else if (event->subscribe.attr_handle == connection_context.power_chr_handle) {
                chr_notify.set_chr_notify_state(chr_notify_t::chr_t::POWER);
                BLE_LOGI("Client subscribed to power characteristic");
            } else if (event->subscribe.attr_handle == connection_context.battery_soc_chr_handle) {
                chr_notify.set_chr_notify_state(chr_notify_t::chr_t::BATT_SoC);
                BLE_LOGI("Client subscribed to battery soc characteristic");
            } else if (event->subscribe.attr_handle == connection_context.runtime_chr_handle) {
                chr_notify.set_chr_notify_state(chr_notify_t::chr_t::RUNTIME_S);
                BLE_LOGI("Client subscribed to runtime characteristic");
            } else {
                BLE_LOGW("Client subsribed to unknown characteristic");
            }
            break;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            connection_context.is_advertising = false;
            connection_context.is_connected = false;
            chr_notify.set_all_chr_notify_state(false);
            connection_context.connection_handle = BLE_HS_CONN_HANDLE_NONE;
            BLE_LOGI("Advertising complete. Reason: %d. Restarting advertising", event->adv_complete.reason);
            ble_advertise();
            break;
        case BLE_GAP_EVENT_CONN_UPDATE:
            BLE_LOGI("Connection parameters updated: Status = %d, Connection handle = %u",
                      event->conn_update.status, event->conn_update.conn_handle);
            break;
        case BLE_GAP_EVENT_CONN_UPDATE_REQ:
            BLE_LOGI("Connection parameters update requested. Accepting. Connection handle = %u, Minimum interval = %u, Maximum interval = %u, Latency = %us, Supervision timeout = %u",
                      event->conn_update_req.conn_handle, event->conn_update_req.peer_params->itvl_min,
                      event->conn_update_req.peer_params->itvl_max, event->conn_update_req.peer_params->latency,
                      event->conn_update_req.peer_params->supervision_timeout);
            break;
        case BLE_GAP_EVENT_PASSKEY_ACTION:
            BLE_LOGI("BLE passkey action. Connection handle = %d. Action = %d",
                      event->passkey.conn_handle, event->passkey.params.action);
            if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
                BLE_LOGI("Passkey to compare = %lu", event->passkey.params.numcmp);
            }
            break;
        case BLE_GAP_EVENT_REPEAT_PAIRING:
            BLE_LOGI("BLE client already with a bond requesting pairing again. Connection handle = %d", event->repeat_pairing.conn_handle);
            event->repeat_pairing.new_key_size = event->repeat_pairing.cur_key_size;
            event->repeat_pairing.new_authenticated = 1;
            event->repeat_pairing.new_sc = 1;
            event->repeat_pairing.new_bonding = 1;
            return BLE_GAP_REPEAT_PAIRING_RETRY;
        case BLE_GAP_EVENT_MTU:
            BLE_LOGI("MTU update event. Connection handle = %d, MTU = %d", event->mtu.conn_handle, event->mtu.value);
            break;
        case BLE_GAP_EVENT_NOTIFY_RX:
            BLE_LOGI("");
            break;
        default:
            BLE_LOGW("Unknown event occured: %d", event->type);
            break;
        }
        return 0;
    }

} // namespace ble
