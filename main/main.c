#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_event.h"
#include "esp_intr_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_intr_alloc.h"
#include "portmacro.h"
#include "http_parser.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "esp_http_server.h"
#include "driver/adc_types_legacy.h"
#include "freertos/idf_additions.h"
#include "hal/adc_types.h"
#include "portmacro.h"
#include <driver/adc.h>

// --- CONFIGURACIÓN ---
#define SENSOR_HALL_GPIO 15
#define MCPWM_CLK_SRC_HZ 80000000 // 80 MHz
#define FILTER_WINDOW_SIZE 10 //para el filtro moving average
#define LED_POV_GPIO 2

static const char *TAG = "POV_SENDER";
//----------------Estructura para el envio de datos al esp-nowx 
typedef struct {
    float rpm;
    float freq;
} message_data_t;

float freq_history[FILTER_WINDOW_SIZE] = {0}; // Buffer
int history_idx = 0;                         // Puntero del buffer
float freq_sum = 0.0;                         // Suma acumulada
float freq_average = 0.0;

volatile float g_freq_average = 0.0;
TaskHandle_t Handle_Tarea_Calculo = NULL;
TaskHandle_t Handle_Tarea_Dibujo = NULL;

// Dirección Broadcast
uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Variables Globales
uint32_t CapturaActual = 0;
uint32_t CapturaAnterior = 0;
uint32_t TicksDiferencia = 0;
static float g_rpm = 0.0;
static float g_freq = 0.0;
static float g_freq_last = 0.0;
float freq_history[FILTER_WINDOW_SIZE] = {0}; // Buffer
int history_idx = 0;                         // Puntero del buffer
float freq_sum = 0.0;                         // Suma acumulada
float freq_average = 0.0;
volatile float g_freq_average = 0.0;
static float dir = -.4; //direccion del motor
//handlers tasks
TaskHandle_t Handle_Tarea_Actualizacion_Matriz = NULL;
TaskHandle_t Handle_Tarea_Calculo = NULL;
TaskHandle_t Handle_Tarea_AumHora = NULL;
gptimer_handle_t gptimer_grado = NULL;
gptimer_handle_t gptimer_AumentoHora = NULL;

//manejadores de API
static esp_err_t set_date_handler(httpd_req_t *req);
static esp_err_t set_dir_handler(httpd_req_t *req);
static esp_err_t set_temp_handler(httpd_req_t *req);

//manejadoree de archivios SPIFFS
static esp_err_t root_get_handler(httpd_req_t *req);
static esp_err_t mainhtml_get_handler(httpd_req_t *req);
static esp_err_t favicon_get_handler(httpd_req_t *req);

static esp_err_t EnviarArchivo(httpd_req_t *req, const char *path);
static const char *get_mime_type(const char *path);
esp_err_t FileSystemInit(void);
static void WiFi_STA_Initialization(void);
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

//matriz principal
uint8_t M[180][7];
uint8_t M1[180][7];
//Hora
uint8_t Time = 20;
uint8_t TimeMin = 49;
uint8_t Segs = 0;
uint8_t Temp = 30;
uint8_t Mes = 10;
uint8_t Dia = 23;
uint16_t year = 25;
//Matrices para los numeros
uint8_t Pun[10][7]={
	{0,0,0,0,0,0,0}, 
	{0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0},
	{0,1,0,0,0,1,0},
	{1,1,1,0,1,1,1},
	{0,1,0,0,0,1,0},
	{0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0}
};
uint8_t C[10][7]={
	{0,0,0,0,0,0,0}, 
	{1,1,0,0,0,1,1},
	{1,1,0,0,0,1,1},
	{1,1,0,0,0,1,1},
	{1,1,0,0,0,1,1},
	{0,1,1,1,1,1,0},
	{0,0,0,0,0,1,0},
	{0,0,0,0,1,0,1},
	{0,0,0,0,0,1,0},
	{0,0,0,0,0,0,0}
};

uint8_t Diag[10][7]={
	{0,0,0,0,0,0,0}, 
	{0,0,0,0,0,1,1},
	{0,0,0,0,1,1,1},
	{0,0,0,1,1,1,0},
	{0,0,1,1,1,0,0},
	{0,1,1,1,0,0,0},
	{1,1,1,0,0,0,0},
	{1,1,0,0,0,0,0},
	{1,0,0,0,0,0,0},
	{0,0,0,0,0,0,0}
};
uint8_t N0[10][7]={
	{0,0,0,0,0,0,0}, 
	{0,0,1,1,1,0,0},
	{0,1,1,0,1,1,0},
	{1,1,0,0,0,1,1},
	{1,0,0,0,0,0,1},
	{1,0,0,0,0,0,1},
	{1,1,0,0,0,1,1},
	{0,1,1,0,1,1,0},
	{0,0,1,1,1,0,0},
	{0,0,0,0,0,0,0}
};
uint8_t N1[10][7]={
	{0,0,0,0,0,0,0}, 
	{0,0,0,0,0,0,0},
	{1,0,0,0,0,0,0},
	{1,0,0,0,0,0,0},
	{1,1,1,1,1,1,1},
	{1,1,1,1,1,1,1},
	{1,0,0,0,1,1,0},
	{1,0,0,0,1,0,0},
	{1,0,0,0,0,0,0},
	{0,0,0,0,0,0,0}
};
uint8_t N2[10][7]={
	{0,0,0,0,0,0,0}, 
	{0,0,0,0,1,1,0},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,1,0,0,0,1},
	{1,1,0,0,0,0,1},
	{0,0,0,0,0,0,0}
};
uint8_t N3[10][7]={
	{0,0,0,0,0,0,0}, 
	{0,0,0,0,0,0,0},
	{0,1,1,1,1,1,0},
	{1,1,1,1,1,1,1},
	{1,1,0,1,0,1,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,1,0,0,0,1,1},
	{0,1,0,0,0,1,0},

};
uint8_t N4[10][7]={
	{0,0,0,0,0,0,0}, 
	{1,1,1,1,1,1,1},
	{1,1,1,1,1,1,1},
	{0,0,1,1,0,0,0},
	{0,0,1,1,0,0,0},
	{0,0,1,1,0,0,0},
	{0,0,1,1,0,0,0},
	{0,0,1,1,0,1,1},
	{0,0,0,1,1,1,1},
	{0,0,0,0,0,0,0}
};
uint8_t N5[10][7]={
	{0,0,0,0,0,0,0}, 
	{0,1,1,0,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,1,1,1},
	{0,0,0,0,0,0,0}
};
uint8_t N6[10][7]={
	{0,0,0,0,0,0,0}, 
	{0,1,1,0,0,0,0},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{0,1,1,1,1,1,0},
	{0,0,0,0,0,0,0}
};
uint8_t N7[10][7]={
	{0,0,0,0,0,0,0}, 
	{0,0,0,0,0,1,1},
	{0,0,0,0,1,1,1},
	{0,0,0,1,1,1,1},
	{0,0,1,1,0,1,1},
	{0,1,1,0,0,1,1},
	{1,1,0,0,0,1,1},
	{1,0,0,0,0,1,1},
	{0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0}
};
uint8_t N8[10][7]={
	{0,0,0,0,0,0,0}, 
	{0,1,1,0,1,1,0},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{0,1,1,0,1,1,0},
	{0,0,0,0,0,0,0}
};
uint8_t N9[10][7]={
	{0,0,0,0,0,0,0}, 
	{0,1,1,1,1,1,0},
	{1,1,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,1,0,0,1},
	{1,0,0,0,1,1,0},
	{0,0,0,0,0,0,0}
};
static bool Capture_Callback_Function(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t current_capture = edata->cap_value;
    uint32_t diff = current_capture - CapturaAnterior;
    CapturaActual = current_capture;
    CapturaAnterior = CapturaActual;
    TicksDiferencia = diff; 
    
    vTaskNotifyGiveFromISR(Handle_Tarea_Calculo, &xHigherPriorityTaskWoken);
    if(Handle_Tarea_Dibujo != NULL) {
        vTaskNotifyGiveFromISR(Handle_Tarea_Dibujo, &xHigherPriorityTaskWoken);
    }
    return xHigherPriorityTaskWoken == pdTRUE;
}
static void Tarea_Proceso(void *parameter){
    
    // Variables para el Moving Average
    float freq_medido;
    while (true) {
        // Esperamos pulso (Timeout 500ms)
        BaseType_t notificacion_recibida = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(300));
        
        if (notificacion_recibida == pdTRUE) {
            // Calcular
            freq_medido = (float)MCPWM_CLK_SRC_HZ / (float)TicksDiferencia;
            if(freq_medido < 40.0){
                g_freq =freq_medido;
            }
            g_rpm = g_freq * 60.0f;
        } else {
            g_rpm = 0.0f; 
            g_freq = 0.0f;
        }
        // 2. ALGORITMO MOVING AVERAGE (Buffer Circular)
        freq_sum -= freq_history[history_idx];       // Restar viejo
        freq_history[history_idx] = g_freq;    // Meter nuevo
        freq_sum += g_freq;           // Sumar nuevo
        
        history_idx = (history_idx + 1) % FILTER_WINDOW_SIZE; // Avanzar índice
        
        // ACTUALIZAR VARIABLE GLOBAL COMPARTIDA
        g_freq_average = freq_sum / FILTER_WINDOW_SIZE;
    }
}
static void Tarea_mandar_datos(void *parameter){
    message_data_t data_to_send;
    while(true){
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Preparar datos filtrados
        data_to_send.rpm = g_freq_average*60;
        data_to_send.freq = g_freq_average; // Mandamos freq cruda o filtrada, tú decides

        // Enviar ESP-NOW
        esp_now_send(broadcast_mac, (uint8_t *) &data_to_send, sizeof(data_to_send));
        
        
    }
    return ESP_OK;
}
// Funciones basicas del servidor web
// --------------------------------------------------------------------------------------------------
esp_err_t FileSystemInit(void){
	ESP_LOGI(TAG, "Iniciando montaje del Sistema de Archivos SPIFFS.");
	esp_vfs_spiffs_conf_t  DiskConf = {
		.base_path = "/Datos",
		.partition_label = "Datos",
		.max_files = 5,
		.format_if_mount_failed = false
	};
	esp_err_t result = esp_vfs_spiffs_register(&DiskConf);
	if (result != ESP_OK){
		ESP_LOGE(TAG, "Error al montar SPIFFS (%s)", esp_err_to_name(result));;
		return result;
	}else{
		ESP_LOGI(TAG, "Sistema de Archivos SPIFFS listo.");
	}
	size_t EspacioTotal, EspacioOcupado;
	if(esp_spiffs_info(DiskConf.partition_label, &EspacioTotal, &EspacioOcupado) == ESP_OK){
		printf("Espacio total: %d, Espacio Utilizado, %d, Espacio Libre: %d\n", EspacioTotal, EspacioOcupado, EspacioTotal-EspacioOcupado);
	}else{
		ESP_LOGE(TAG, "Error al tratar de obtener informacion de la particion.");
	}
    return ESP_OK;
}

static void WiFi_STA_Initialization(void){
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);
    wifi_config_t wifi_config = {
    	.sta = {
        	.ssid = "POCOleo",
        	.password = "manitochido"
    	}
	};
	esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Desconectado, reintentando...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("Direccion IP:" IPSTR "\n", IP2STR(&event->ip_info.ip));
    }
}

static void Tarea_Dibujar_POV(void *parameter) {
    
    // Configurar LED
    gpio_reset_pin(LED_POV_GPIO);
    gpio_set_direction(LED_POV_GPIO, GPIO_MODE_OUTPUT);

    while(1) {
        // La tarea se queda dormida hasta que el sensor Hall detecta el imán
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        float freq_actual = g_freq_average;
        
        // Si gira muy lento no dibujamos 
        if (freq_actual < 5.0) continue; 

        // Periodo de una vuelta en microsegundos (us)
        float vuelta_us = 1000000.0f / freq_actual;
        
        // Tiempo de 1 pixel 
        float pixel_us = vuelta_us / 90.0f;
        
        // Calcular ON y OFF
        uint32_t time_on = (uint32_t)(pixel_us * 0.20f);  // 1.5 grados
        uint32_t time_off = (uint32_t)(pixel_us * 0.80f); // 0.5 grados

        //------Bucle de dibujo
        for (int i = 0; i < 90; i++) {
            // LED ON
            gpio_set_level(LED_POV_GPIO, 1);
            esp_rom_delay_us(time_on); 

            // LED OFF (Espacio negro)
            gpio_set_level(LED_POV_GPIO, 0);
            esp_rom_delay_us(time_off);
        }
        
        // Al terminar los pixeles, vuelve arriba y espera el siguiente imán.
        // El LED se queda apagado en el tiempo "muerto" si la vuelta no fue perfecta.
    }
}

void app_main(void)
{
    // 1. NVS Flash (Necesario para WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    char buffer[1024];
    size_t bytes;
    while ((bytes = fread(buffer, 1, sizeof(buffer), f)) > 0) {
        httpd_resp_send_chunk(req, buffer, bytes);
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

    // 2. Iniciar WiFi y ESP-NOW
    wifi_espnow_init();
    // CREACIÓN DE TAREAS
    // Tareas lógicas en Core 0
    xTaskCreatePinnedToCore(Tarea_Proceso, "Calculo", 4096, NULL, 5, &Handle_Tarea_Calculo, 0);
    xTaskCreatePinnedToCore(Tarea_mandar_datos, "Wifi", 4096, NULL, 3, NULL, 0);
    
    // TAREA DE DIBUJO EN CORE 1 (Máxima prioridad para que nada la interrumpa)
    xTaskCreatePinnedToCore(Tarea_Dibujar_POV, "POV_LED", 4096, NULL, 20, &Handle_Tarea_Dibujo, 1);
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
// TODO
    mcpwm_capture_event_callbacks_t Configuracion_Capture_Callback = { .on_cap = Capture_Callback_Function };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(Handler_CaptureChannel, &Configuracion_Capture_Callback, NULL));

    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(Handler_CaptureChannel));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(Handle_CaptureTimer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(Handle_CaptureTimer));

	
	gptimer_enable(gptimer_grado);
	gptimer_start(gptimer_grado);
	gptimer_enable(gptimer_AumentoHora);
	gptimer_start(gptimer_AumentoHora);
	
 
}