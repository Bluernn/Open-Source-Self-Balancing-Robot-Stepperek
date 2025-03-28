#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <esp_wifi.h>
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <esp_http_server.h>
#include <esp_spiffs.h>

#define WIFI_SSID		CONFIG_ESP_WIFI_SSID
#define WIFI_PASSWORD	CONFIG_ESP_WIFI_PASSWORD
#define WIFI_CHANNEL	CONFIG_ESP_WIFI_CHANNEL
#define WIFI_MAX_STA	CONFIG_ESP_MAX_STA_CONN

volatile bool isStation = false;

static const char *WIFI_TAG = "WIFI";

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        isStation = true;
        ESP_LOGI(WIFI_TAG, "New station joined");
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        isStation = false;
        ESP_LOGI(WIFI_TAG, "A station left");
    }
}

void setup_wifi(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
   
	ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t * p_netif = esp_netif_create_default_wifi_ap();

	esp_netif_ip_info_t ipInfo;
	esp_netif_set_ip4_addr(&ipInfo.ip, 192,168,1,1);
	esp_netif_set_ip4_addr(&ipInfo.gw, 192,168,1,1);
	esp_netif_set_ip4_addr(&ipInfo.netmask, 255,255,255,0);
	esp_netif_dhcps_stop(p_netif);
	esp_netif_set_ip_info(p_netif, &ipInfo);
	esp_netif_dhcps_start(p_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = WIFI_MAX_STA
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(WIFI_TAG, "WiFi init done.");

	esp_netif_ip_info_t if_info;
	ESP_ERROR_CHECK(esp_netif_get_ip_info(p_netif, &if_info));
	ESP_LOGI(WIFI_TAG, "ESP32 IP:" IPSTR, IP2STR(&if_info.ip));
}