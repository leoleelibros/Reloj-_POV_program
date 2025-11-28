#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

// --- CONFIGURACIÓN ---
#define SENSOR_HALL_GPIO 15
#define MCPWM_CLK_SRC_HZ 80000000 // 80 MHz

static const char *TAG = "POV_SENDER";

typedef struct {
    float rpm;
    float freq;
} message_data_t;

// Dirección Broadcast
uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Variables Globales
uint32_t CapturaActual = 0;
uint32_t CapturaAnterior = 0;
uint32_t TicksDiferencia = 0;
static float g_rpm = 0.0;
static float g_freq = 0.0;

// --- CALLBACK CON DEBOUNCING ---
static bool Capture_Callback_Function(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t current_capture = edata->cap_value;
    uint32_t diff = current_capture - CapturaAnterior;
    CapturaActual = current_capture;
    CapturaAnterior = CapturaActual;
    TicksDiferencia = diff; 
    
    // 4. Despertar Tarea
    TaskHandle_t Tarea_a_Renaudar = (TaskHandle_t)user_data;
    vTaskNotifyGiveFromISR(Tarea_a_Renaudar, &xHigherPriorityTaskWoken);
    
    return xHigherPriorityTaskWoken == pdTRUE;
}

static void Tarea_Proceso(void *parameter){
    message_data_t data_to_send;
    float rpm_medido;
    while (true) {
        // Esperamos pulso (Timeout 500ms)
        BaseType_t notificacion_recibida = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(300));
        
        if (notificacion_recibida == pdTRUE) {
            // Calcular
            g_freq = (float)MCPWM_CLK_SRC_HZ / (float)TicksDiferencia;
            rpm_medido = g_freq * 60.0f;
            if (rpm_medido < 800){
                g_rpm=rpm_medido;
            }
        } else {
            g_rpm = 0.0f; 
            g_freq = 0.0f;
        }

        // Enviar
        data_to_send.rpm = g_rpm;
        data_to_send.freq = g_freq;
        esp_now_send(broadcast_mac, (uint8_t *) &data_to_send, sizeof(data_to_send));
        
        // Log para verificar que el ruido se fue
        if(notificacion_recibida == pdTRUE) {
             ESP_LOGI(TAG, "RPM: %.2f (Hz: %.2f)", g_rpm, g_freq);
        }
    }
}
static void wifi_espnow_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Iniciar ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    
    // Registrar el Peer de Broadcast
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcast_mac, 6);
    peerInfo.channel = 0;  // 0 = usa el canal actual del wifi
    peerInfo.encrypt = false;
    
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
    ESP_LOGI(TAG, "ESP-NOW Iniciado en modo Broadcast");
}
void app_main(void)
{
    // 1. NVS Flash (Necesario para WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Iniciar WiFi y ESP-NOW
    wifi_espnow_init();

    // 3. Crear Tarea
    TaskHandle_t Handle_Tarea;
    xTaskCreate(Tarea_Proceso, "ProcesoPOV", 4096, NULL, 5, &Handle_Tarea);

    // 4. Configurar MCPWM (Lectura RPM) en PIN 15
    mcpwm_cap_timer_handle_t Handle_CaptureTimer = NULL;
    mcpwm_capture_timer_config_t Configuracion_CaptureTimer = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT, .group_id = 0
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&Configuracion_CaptureTimer, &Handle_CaptureTimer));

    mcpwm_cap_channel_handle_t Handler_CaptureChannel = NULL;
    mcpwm_capture_channel_config_t Configuracion_CaptureChannel = {
        .gpio_num = SENSOR_HALL_GPIO, // PIN 15
        .prescale = 1,
        .flags.neg_edge = true, 
        .flags.pos_edge = false,
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(Handle_CaptureTimer, &Configuracion_CaptureChannel, &Handler_CaptureChannel));

    mcpwm_capture_event_callbacks_t Configuracion_Capture_Callback = { .on_cap = Capture_Callback_Function };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(Handler_CaptureChannel, &Configuracion_Capture_Callback, Handle_Tarea));

    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(Handler_CaptureChannel));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(Handle_CaptureTimer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(Handle_CaptureTimer));
    vTaskDelete(NULL);
    // No borramos app_main para que FreeRTOS no se queje, o usamos un loop infinito.
    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}