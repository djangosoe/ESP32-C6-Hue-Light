#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <stdbool.h>

#include "esp_zigbee.h"
#include "ezbee/zha.h"


#define LED_GPIO        GPIO_NUM_15
#define LIGHT_ENDPOINT  1

static const char *TAG = "ZigbeeFlame";

static volatile bool pairing_mode = false;


/*
 * Zigbee strings are ZCL strings.
 *
 * The first byte contains the length:
 *
 * "ZigbeeFlame" = 11 characters = 0x0B
 * "ZF-ESP32C6"  = 10 characters = 0x0A
 */
#define ZIGBEE_MANUFACTURER_NAME  "\x0B" "ZigbeeFlame"
#define ZIGBEE_MODEL_IDENTIFIER   "\x0A" "ZF-ESP32C6"


/* ---------------------------------------------------------
 * Zigbee configuration
 * --------------------------------------------------------- */

#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "zb_storage"

#define ESP_ZIGBEE_ROUTER_CONFIG()                  \
{                                                 \
    .device_type = EZB_NWK_DEVICE_TYPE_ROUTER, \
    .install_code_policy = false,                 \
    .zed_config = {                               \
        .ed_timeout = EZB_NWK_ED_TIMEOUT_64MIN,   \
        .keep_alive = 3000,                       \
    },                                            \
}

#define ESP_ZIGBEE_PLATFORM_CONFIG()              \
{                                                 \
    .storage_partition_name = ESP_ZIGBEE_STORAGE_PARTITION_NAME, \
    .radio_config = {                             \
        .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE, \
    },                                            \
}

#define ESP_ZIGBEE_DEFAULT_CONFIG()               \
{                                                 \
    .device_config = ESP_ZIGBEE_ROUTER_CONFIG(),     \
    .platform_config = ESP_ZIGBEE_PLATFORM_CONFIG(), \
}


/* ---------------------------------------------------------
 * LED
 * --------------------------------------------------------- */

static void led_set(bool on)
{
    /* XIAO ESP32-C6 onboard LED is active-low */
    gpio_set_level(LED_GPIO, on ? 0 : 1);
}


static void led_init(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    led_set(false);
}


static void pairing_led_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Pairing LED task started");

    while (1)
    {
        if (pairing_mode)
        {
            led_set(true);
            vTaskDelay(pdMS_TO_TICKS(250));

            if (pairing_mode)
            {
                led_set(false);
                vTaskDelay(pdMS_TO_TICKS(250));
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}


static void log_simple_descriptor(uint8_t ep_id)
{
    const ezb_af_simple_desc_t *desc =
        ezb_af_get_simple_desc(ep_id);

    if (desc == NULL)
    {
        ESP_LOGW(TAG, "No simple descriptor found for endpoint %u", ep_id);
        return;
    }

    ESP_LOGI(TAG, "---- Simple Descriptor ----");
    ESP_LOGI(TAG, "Endpoint: %u", desc->ep_id);
    ESP_LOGI(TAG, "Profile ID: 0x%04x", desc->app_profile_id);
    ESP_LOGI(TAG, "Device ID: 0x%04x", desc->app_device_id);
    ESP_LOGI(TAG, "Device version: %u", desc->app_device_version);
    ESP_LOGI(TAG, "Input clusters: %u", desc->app_input_cluster_count);
    ESP_LOGI(TAG, "Output clusters: %u", desc->app_output_cluster_count);

    for (int i = 0; i < desc->app_input_cluster_count; i++)
    {
        ESP_LOGI(
            TAG,
            "  IN  cluster[%d] = 0x%04x",
            i,
            desc->app_cluster_list[i]
        );
    }

    for (int i = 0; i < desc->app_output_cluster_count; i++)
    {
        int index = desc->app_input_cluster_count + i;

        ESP_LOGI(
            TAG,
            "  OUT cluster[%d] = 0x%04x",
            i,
            desc->app_cluster_list[index]
        );
    }

    ESP_LOGI(TAG, "---------------------------");
}

static void zigbee_retry_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Retrying network steering");

    esp_zigbee_lock_acquire(portMAX_DELAY);

    ezb_bdb_start_top_level_commissioning(
        EZB_BDB_MODE_NETWORK_STEERING
    );

    esp_zigbee_lock_release();

    vTaskDelete(NULL);
}

/* ---------------------------------------------------------
 * Zigbee ZCL handling
 * --------------------------------------------------------- */

static void zigbee_set_attr_handler(
    ezb_zcl_set_attr_value_message_t *message)
{
    if (message == NULL)
    {
        ESP_LOGW(TAG, "SET_ATTR callback with NULL message");
        return;
    }

    ESP_LOGI(
        TAG,
        "ZCL attribute change: endpoint=%d cluster=0x%04x attr=0x%04x",
        message->info.dst_ep,
        message->info.cluster_id,
        message->in.attribute.id
    );


    /*
     * Standard On/Off cluster
     */
    if (message->info.cluster_id == EZB_ZCL_CLUSTER_ID_ON_OFF)
    {
        if (message->in.attribute.data.value != NULL)
        {
            bool on =
                *(uint8_t *)message->in.attribute.data.value;

            ESP_LOGI(
                TAG,
                "Received Zigbee On/Off: %s",
                on ? "ON" : "OFF"
            );

            led_set(on);
        }
    }
}


static void zigbee_action_handler(
    ezb_zcl_core_action_callback_id_t callback_id,
    void *message)
{
    ESP_LOGI(
        TAG,
        "ZCL action callback: 0x%04lx",
        callback_id
    );


    switch (callback_id)
    {
        case EZB_ZCL_CORE_SET_ATTR_VALUE_CB_ID:
        {
            zigbee_set_attr_handler(
                (ezb_zcl_set_attr_value_message_t *)message
            );

            break;
        }


        case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID:
        {
            if (message != NULL)
            {
                ezb_zcl_cmd_default_rsp_message_t *response =
                    (ezb_zcl_cmd_default_rsp_message_t *)message;

                ESP_LOGI(
                    TAG,
                    "ZCL default response status=0x%02x",
                    response->in.status_code
                );
            }

            break;
        }


        default:
        {
            ESP_LOGI(
                TAG,
                "Unhandled ZCL action callback: 0x%04lx",
                callback_id
            );

            break;
        }
    }
}


/* ---------------------------------------------------------
 * Print current network information
 * --------------------------------------------------------- */

static void log_network_info(void)
{
    ESP_LOGI(
        TAG,
        "Joined: %s",
        ezb_bdb_dev_joined() ? "YES" : "NO"
    );

    ESP_LOGI(
        TAG,
        "Factory new: %s",
        ezb_bdb_is_factory_new() ? "YES" : "NO"
    );

    ESP_LOGI(
        TAG,
        "RX on when idle: %s",
        ezb_nwk_get_rx_on_when_idle() ? "YES" : "NO"
    );


    if (ezb_bdb_dev_joined())
    {
        ezb_extpanid_t extended_pan_id;

        ezb_nwk_get_extended_panid(
            &extended_pan_id
        );

        ESP_LOGI(
            TAG,
            "PAN ID: 0x%04hx",
            ezb_nwk_get_panid()
        );

        ESP_LOGI(
            TAG,
            "Extended PAN ID: 0x%llx",
            extended_pan_id.u64
        );

        ESP_LOGI(
            TAG,
            "Channel: %d",
            ezb_nwk_get_current_channel()
        );

        ESP_LOGI(
            TAG,
            "Short address: 0x%04hx",
            ezb_nwk_get_short_address()
        );
    }
}


/* ---------------------------------------------------------
 * Zigbee commissioning / network events
 * --------------------------------------------------------- */

static bool zigbee_app_signal_handler(
    const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t signal_type =
        ezb_app_signal_get_type(app_signal);


    ESP_LOGI(
        TAG,
        "Zigbee APP signal: %s (0x%02x)",
        ezb_app_signal_to_string(signal_type),
        signal_type
    );


    switch (signal_type)
    {
        case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        {
            ESP_LOGI(
                TAG,
                "Initializing Zigbee commissioning"
            );

            ezb_bdb_start_top_level_commissioning(
                EZB_BDB_MODE_INITIALIZATION
            );

            break;
        }


        case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case EZB_BDB_SIGNAL_DEVICE_REBOOT:
        {
            ezb_bdb_comm_status_t status =
                *((ezb_bdb_comm_status_t *)
                    ezb_app_signal_get_params(app_signal));


            ESP_LOGI(
                TAG,
                "Zigbee startup status = 0x%02x",
                status
            );


            if (status == EZB_BDB_STATUS_SUCCESS)
            {
                log_network_info();


                if (ezb_bdb_is_factory_new())
                {
                    pairing_mode = true;

                    ESP_LOGI(
                        TAG,
                        "Factory-new device; starting network steering"
                    );

                    ezb_bdb_start_top_level_commissioning(
                        EZB_BDB_MODE_NETWORK_STEERING
                    );
                }
                else
                {
                    pairing_mode = false;
                    led_set(false);

                    ESP_LOGI(
                        TAG,
                        "Device rebooted with stored Zigbee network"
                    );

                    ESP_LOGI(
                        TAG,
                        "Waiting for coordinator traffic / device interview..."
                    );
                }
            }
            else
            {
                ESP_LOGW(
                    TAG,
                    "Zigbee initialization failed, status=0x%02x",
                    status
                );
            }

            break;
        }


        case EZB_BDB_SIGNAL_STEERING:
        {
            ezb_bdb_comm_status_t status =
                *((ezb_bdb_comm_status_t *)
                    ezb_app_signal_get_params(app_signal));


            if (status == EZB_BDB_STATUS_SUCCESS)
            {
                pairing_mode = false;
                led_set(false);

                ESP_LOGI(
                    TAG,
                    "================================================"
                );

                ESP_LOGI(
                    TAG,
                    "JOINED ZIGBEE NETWORK SUCCESSFULLY"
                );

                ESP_LOGI(
                    TAG,
                    "================================================"
                );

                log_network_info();

                ESP_LOGI(
                    TAG,
                    "Waiting for Hue Bridge to interview endpoint %d",
                    LIGHT_ENDPOINT
                );
            }
            else
            {
                /*
                 * Visual indication before retry.
                 */
               ESP_LOGW(
                    TAG,
                    "Failed to join Zigbee network, status=0x%02x. Retrying...",
                    status
                );

                pairing_mode = true;

                xTaskCreate(
                        zigbee_retry_task,
                        "zigbee_retry_task",
                        2048,
                        NULL,
                        4,
                        NULL
                    );
            }

            break;
        }


        /*
         * Useful while Hue has "Add lights" open.
         */
        case EZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        {
            uint8_t duration =
                *(uint8_t *)
                    ezb_app_signal_get_params(app_signal);

            if (duration > 0)
            {
                ESP_LOGI(
                    TAG,
                    "Network 0x%04hx permits joining for %u seconds",
                    ezb_nwk_get_panid(),
                    duration
                );
            }
            else
            {
                ESP_LOGI(
                    TAG,
                    "Network 0x%04hx permit-join window closed",
                    ezb_nwk_get_panid()
                );
            }

            break;
        }


        case EZB_ZDO_SIGNAL_LEAVE:
        {
            const ezb_zdo_signal_leave_params_t *leave_params =
                ezb_app_signal_get_params(app_signal);

            pairing_mode = false;

            ESP_LOGW(
                TAG,
                "Left Zigbee network, leave type=0x%02x",
                leave_params->leave_type
            );

            break;
        }


        case EZB_ZDO_SIGNAL_DEVICE_ANNCE:
        {
            const ezb_zdo_signal_device_annce_params_t *params =
                ezb_app_signal_get_params(app_signal);

            ESP_LOGI(
                TAG,
                "Device announce received: short address=0x%04hx",
                params->short_addr
            );

            break;
        }


        default:
        {
            /*
             * Already logged above so we don't lose potentially
             * useful Hue interview / network activity.
             */
            break;
        }
    }


    return true;
}


/* ---------------------------------------------------------
 * Create Zigbee On/Off Light
 * --------------------------------------------------------- */

static esp_err_t create_zigbee_light(void)
{
    ezb_af_device_desc_t device =
        ezb_af_create_device_desc();

//OnOf Light
    ezb_zha_on_off_light_config_t light_config =
       EZB_ZHA_ON_OFF_LIGHT_CONFIG();


    ezb_af_ep_desc_t endpoint =
       ezb_zha_create_on_off_light(
           LIGHT_ENDPOINT,
           &light_config
       );

//Dimable light
//    ezb_zha_color_dimmable_light_config_t light_config =
//        EZB_ZHA_COLOR_DIMMABLE_LIGHT_CONFIG();

    // ezb_af_ep_desc_t endpoint =
    //     ezb_zha_create_color_dimmable_light(
    //         LIGHT_ENDPOINT,
    //         &light_config
    //     );

    ESP_ERROR_CHECK(
        ezb_af_ep_desc_set_app_version(
            endpoint,
            1
        )
    );

    ESP_LOGI(
        TAG,
        "Endpoint %d device version: %d",
        LIGHT_ENDPOINT,
        ezb_af_ep_desc_get_app_version(endpoint)
    );
    /*
     * Get the Basic cluster already created by
     * ezb_zha_create_on_off_light().
     */
    ezb_zcl_cluster_desc_t basic_desc = {0};

    basic_desc =
        ezb_af_endpoint_get_cluster_desc(
            endpoint,
            EZB_ZCL_CLUSTER_ID_BASIC,
            EZB_ZCL_CLUSTER_SERVER
        );


    /*
     * Give the bridge useful identification data.
     */
    ezb_zcl_basic_cluster_desc_add_attr(
        basic_desc,
        EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
        (void *)ZIGBEE_MANUFACTURER_NAME
    );

    ezb_zcl_basic_cluster_desc_add_attr(
        basic_desc,
        EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
        (void *)ZIGBEE_MODEL_IDENTIFIER
    );


    ESP_LOGI(
        TAG,
        "Zigbee manufacturer: ZigbeeFlame"
    );

    ESP_LOGI(
        TAG,
        "Zigbee model: ZF-ESP32C6"
    );

    ESP_LOGI(
        TAG,
        "Endpoint: %d",
        LIGHT_ENDPOINT
    );

    ESP_LOGI(
        TAG,
        "Device type: ZHA On/Off Light"
    );

    ESP_LOGI(
        TAG,
        "Basic cluster: 0x%04x",
        EZB_ZCL_CLUSTER_ID_BASIC
    );

    ESP_LOGI(
        TAG,
        "On/Off cluster: 0x%04x",
        EZB_ZCL_CLUSTER_ID_ON_OFF
    );


    ESP_ERROR_CHECK(
        ezb_af_device_add_endpoint_desc(
            device,
            endpoint
        )
    );


    ESP_ERROR_CHECK(
        ezb_af_device_desc_register(
            device
        )
    );

    log_simple_descriptor(LIGHT_ENDPOINT);

    ezb_zcl_core_action_handler_register(
        zigbee_action_handler
    );


    ESP_LOGI(
        TAG,
        "Zigbee On/Off Light registered"
    );


    return ESP_OK;
}


/* ---------------------------------------------------------
 * Zigbee setup
 * --------------------------------------------------------- */

static esp_err_t setup_zigbee_commissioning(void)
{
    /*
     * Hue is the central Trust Center/coordinator.
     */
    ezb_aps_secur_enable_distributed_security(false);


    /*
     * VERY IMPORTANT for this test:
     *
     * This is an always-powered light, so keep the Zigbee
     * receiver active when idle. Hue can then immediately
     * query/interview it after it joins.
     *
     * Must be configured before joining.
     */
    ezb_nwk_set_rx_on_when_idle(true);


    ESP_LOGI(
        TAG,
        "RX-on-when-idle enabled"
    );


    ESP_ERROR_CHECK(
        ezb_app_signal_add_handler(
            zigbee_app_signal_handler
        )
    );


    return ESP_OK;
}


/* ---------------------------------------------------------
 * Zigbee task
 * --------------------------------------------------------- */

static void zigbee_task(void *pvParameters)
{
    ESP_LOGI(
        TAG,
        "Initializing Zigbee"
    );


    esp_zigbee_config_t config =
        ESP_ZIGBEE_DEFAULT_CONFIG();


    ESP_ERROR_CHECK(
        esp_zigbee_init(
            &config
        )
    );


    ESP_ERROR_CHECK(
        setup_zigbee_commissioning()
    );


    ESP_ERROR_CHECK(
        create_zigbee_light()
    );


    ESP_LOGI(
        TAG,
        "Starting Zigbee stack"
    );


    ESP_ERROR_CHECK(
        esp_zigbee_start(false)
    );


    esp_zigbee_launch_mainloop();


    vTaskDelete(NULL);
}


/* ---------------------------------------------------------
 * Main
 * --------------------------------------------------------- */

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(5000));


    ESP_LOGI(
        TAG,
        "Starting ZigbeeFlame"
    );


    led_init();


    ESP_ERROR_CHECK(
        nvs_flash_init()
    );


    ESP_ERROR_CHECK(
        nvs_flash_init_partition(
            ESP_ZIGBEE_STORAGE_PARTITION_NAME
        )
    );


    BaseType_t result =
        xTaskCreate(
            pairing_led_task,
            "pairing_led_task",
            2048,
            NULL,
            6,
            NULL
        );


    ESP_LOGI(
        TAG,
        "Pairing LED task create result: %s",
        result == pdPASS ? "OK" : "FAILED"
    );


    xTaskCreate(
        zigbee_task,
        "zigbee_task",
        4096,
        NULL,
        5,
        NULL
    );
}