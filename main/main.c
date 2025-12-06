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

static const char *TAG = "Web Server";
//ADC

//velocidad provisional rps
uint64_t Vel = 13;
uint64_t alarm_count_us;
//contador
uint8_t i=0;
uint8_t x=0;
//leds
//18 NO
#define LED1 GPIO_NUM_13
#define LED2 GPIO_NUM_12
#define LED3 GPIO_NUM_14
#define LED4 GPIO_NUM_27
#define LED5 GPIO_NUM_26
#define LED6 GPIO_NUM_25
#define LED7 GPIO_NUM_33
//leds del 2do palo
#define LED1b GPIO_NUM_32
#define LED2b GPIO_NUM_23
#define LED3b GPIO_NUM_22
#define LED4b GPIO_NUM_21
#define LED5b GPIO_NUM_19
#define LED6b GPIO_NUM_5
#define LED7b GPIO_NUM_4

//para el sensor efecto hall
#define SENSOR_HALL_GPIO 15
#define MCPWM_CLK_SRC_HZ 80000000 // 80 MHz
#define FILTER_WINDOW_SIZE 2 //para el filtro moving average
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
                g_freq_last = g_freq;
            }
            else{
                g_freq = g_freq_last;
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
        // 3. ACTUALIZAR TIEMPO POR GRADO (Mover la lógica del temporizador aquí)
        if (g_freq_average > 0.1) {
            // Calcular el nuevo tiempo en microsegundos (ticks)
            alarm_count_us = (uint64_t)(1000000.0 / ((g_freq + dir)* 180.0));
            // Aplicar un límite inferior para evitar tiempos irrazonablemente cortos.
            if (alarm_count_us < 300) { // Límite inferior de 100 us (10kHz)
                alarm_count_us = 300;
            }
        } else {
            alarm_count_us = 400;
        }
        gptimer_alarm_config_t CambioGrado = {
            .alarm_count = alarm_count_us,
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true
        };
        gptimer_set_alarm_action(gptimer_grado, &CambioGrado);
    }
}
static void Tarea_AumHora(void *parameter){
    while (true) {
        BaseType_t notificacion_recibida = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        if (notificacion_recibida == pdTRUE) {
            Segs++;
        
            if (Segs>=60) {
				Segs=0;
				TimeMin++;
				}
				if(TimeMin>=60){
					TimeMin=0;
					Time++;
					}
					if(Time>=24){
						Time=0;
						Dia++;
					}
			xTaskNotifyGive(Handle_Tarea_Actualizacion_Matriz);
		
            }
    }
}
static bool IRAM_ATTR CambioGradoCallback(gptimer_handle_t CambioGrado, const gptimer_alarm_event_data_t *edata, void *user_data){
	BaseType_t high_task_awoken = pdFALSE;
		gpio_set_level(LED1, M[i][0]);
        gpio_set_level(LED2, M[i][1]);
        gpio_set_level(LED3, M[i][2]);
        gpio_set_level(LED4, M[i][3]);
        gpio_set_level(LED5, M[i][4]);
        gpio_set_level(LED6, M[i][5]);
        gpio_set_level(LED7, M[i][6]);
        //segunda tira de leds
        gpio_set_level(LED1b, M1[i][0]);
        gpio_set_level(LED2b, M1[i][1]);
        gpio_set_level(LED3b, M1[i][2]);
        gpio_set_level(LED4b, M1[i][3]);
        gpio_set_level(LED5b, M1[i][4]);
        gpio_set_level(LED6b, M1[i][5]);
        gpio_set_level(LED7b, M1[i][6]);
        
        // 2. Incrementar y resetear el contador
        i++;
        if (i >= 180) { 
            i=0;
        }
	return (high_task_awoken == pdTRUE); 						
}
static bool IRAM_ATTR CambioHoraCallback(gptimer_handle_t CambioGrado, const gptimer_alarm_event_data_t *edata, void *user_data){
	BaseType_t high_task_awoken = pdFALSE;
	vTaskNotifyGiveFromISR(Handle_Tarea_AumHora, &high_task_awoken);
	return (high_task_awoken == pdTRUE); 						
}
static void Tarea_Actualizar_Matriz(void *parameter){
    while (true) {
        // Esperar la notificación de la ISR (AumentoHoraCallback)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Esperar indefinidamente
        

        uint8_t Hora = Time/10;
        uint8_t HoraRes = Time%10;
        uint8_t Min = TimeMin/10;
        uint8_t MinRes = TimeMin%10;
        uint8_t TempDec = Temp/10;
        uint8_t TempRes = Temp%10;
        uint8_t DiaDec = Dia/10;
        uint8_t DiaRes = Dia%10;
        uint8_t MesDec = Mes/10;
        uint8_t MesRes = Mes%10;
        uint8_t yearDec = (year)/10;
        uint8_t yearRes = (year)%10;
        // Ejecutar el trabajo pesado: Copiar la matriz
        for (int y = 0; y<10; y++) {
            for (int j = 0; j<7; j++) {
                switch (Hora) {
                    case 0:
                    M[y+10][j]=N0[y][j];
                    M1[y+100][j]=N0[y][j]; //para cuando los leds que llevan el sensor estan en 0 estos estan en 180(90) y viceversa
                    break;
                    case 1:
                    M[y+10][j]=N1[y][j];
                    M1[y+100][j]=N1[y][j];
                    break;
                    case 2:
                    M[y+10][j]=N2[y][j];
                    M1[y+100][j]=N2[y][j];
                    break;
                    case 3:
                    M[y+10][j]=N3[y][j];
                    M1[y+100][j]=N3[y][j];
                    break;
                    case 4:
                    M[y+10][j]=N4[y][j];
                    M1[y+100][j]=N4[y][j];
                    break;
                    case 5:
                    M[y+10][j]=N5[y][j];
                    M1[y+100][j]=N5[y][j];
                    break;
                    case 6:
                    M[y+10][j]=N6[y][j];
                    M1[y+100][j]=N6[y][j];
                    break;
                    case 7:
                    M[y+10][j]=N7[y][j];
                    M1[y+100][j]=N7[y][j];
                    break;
                    case 8:
                    M[y+10][j]=N8[y][j];
                    M1[y+100][j]=N8[y][j];
                    break;
                    case 9:
                    M[y+10][j]=N9[y][j];
                    M1[y+100][j]=N9[y][j];
                    break;
                }
                switch (HoraRes) {
                    case 0:
                    M[y][j]=N0[y][j];
                    M1[y+90][j]=N0[y][j]; //para cuando los leds que llevan el sensor estan en 0 estos estan en 180(90) y viceversa
                    break;
                    case 1:
                    M[y][j]=N1[y][j];
                    M1[y+90][j]=N1[y][j];
                    break;
                    case 2:
                    M[y][j]=N2[y][j];
                    M1[y+90][j]=N2[y][j];
                    break;
                    case 3:
                    M[y][j]=N3[y][j];
                    M1[y+90][j]=N3[y][j];
                    break;
                    case 4:
                    M[y][j]=N4[y][j];
                    M1[y+90][j]=N4[y][j];
                    break;
                    case 5:
                    M[y][j]=N5[y][j];
                    M1[y+90][j]=N5[y][j];
                    break;
                    case 6:
                    M[y][j]=N6[y][j];
                    M1[y+90][j]=N6[y][j];
                    break;
                    case 7:
                    M[y][j]=N7[y][j];
                    M1[y+90][j]=N7[y][j];
                    break;
                    case 8:
                    M[y][j]=N8[y][j];
                    M1[y+90][j]=N8[y][j];
                    break;
                    case 9:
                    M[y][j]=N9[y][j];
                    M1[y+90][j]=N9[y][j];
                    break;
                }
                M[y+170][j]=Pun[y][j];
                M1[y+80][j]=Pun[y][j];
                switch (Min) {
					case 0:
					M[y+160][j]=N0[y][j];
                    M1[y+70][j]=N0[y][j];
                    break;
                    case 1:
					M[y+160][j]=N1[y][j];
                    M1[y+70][j]=N1[y][j];
                    break;
                    case 2:
					M[y+160][j]=N2[y][j];
                    M1[y+70][j]=N2[y][j];
                    break;
                    case 3:
					M[y+160][j]=N3[y][j];
                    M1[y+70][j]=N3[y][j];
                    break;
                    case 4:
					M[y+160][j]=N4[y][j];
                    M1[y+70][j]=N4[y][j];
                    break;
                    case 5:
					M[y+160][j]=N5[y][j];
                    M1[y+70][j]=N5[y][j];
                    break;
                    case 6:
					M[y+160][j]=N6[y][j];
                    M1[y+70][j]=N6[y][j];
                    break;
                    case 7:
					M[y+160][j]=N7[y][j];
                    M1[y+70][j]=N7[y][j];
                    break;
                    case 8:
					M[y+160][j]=N8[y][j];
                    M1[y+70][j]=N8[y][j];
                    break;
                    case 9:
					M[y+160][j]=N9[y][j];
                    M1[y+70][j]=N9[y][j];
                    break;
				}
				switch (MinRes) {
					case 0:
					M[y+150][j]=N0[y][j];
                    M1[y+60][j]=N0[y][j];
                    break;
                    case 1:
					M[y+150][j]=N1[y][j];
                    M1[y+60][j]=N1[y][j];
                    break;
                    case 2:
					M[y+150][j]=N2[y][j];
                    M1[y+60][j]=N2[y][j];
                    break;
                    case 3:
					M[y+150][j]=N3[y][j];
                    M1[y+60][j]=N3[y][j];
                    break;
                    case 4:
					M[y+150][j]=N4[y][j];
                    M1[y+60][j]=N4[y][j];
                    break;
                    case 5:
					M[y+150][j]=N5[y][j];
                    M1[y+60][j]=N5[y][j];
                    break;
                    case 6:
					M[y+150][j]=N6[y][j];
                    M1[y+60][j]=N6[y][j];
                    break;
                    case 7:
					M[y+150][j]=N7[y][j];
                    M1[y+60][j]=N7[y][j];
                    break;
                    case 8:
					M[y+150][j]=N8[y][j];
                    M1[y+60][j]=N8[y][j];
                    break;
                    case 9:
					M[y+150][j]=N9[y][j];
                    M1[y+60][j]=N9[y][j];
                    break;
                    }
                    //para representar la temperatura
                    
                    switch (TempDec) {
					case 0:
					M[y+135][j]=N0[y][j];
                    M1[y+45][j]=N0[y][j];
                    break;
                    case 1:
					M[y+135][j]=N1[y][j];
                    M1[y+45][j]=N1[y][j];
                    break;
                    case 2:
					M[y+135][j]=N2[y][j];
                    M1[y+45][j]=N2[y][j];
                    break;
                    case 3:
					M[y+135][j]=N3[y][j];
                    M1[y+45][j]=N3[y][j];
                    break;
                    case 4:
					M[y+135][j]=N4[y][j];
                    M1[y+45][j]=N4[y][j];
                    break;
                    case 5:
					M[y+135][j]=N5[y][j];
                    M1[y+45][j]=N5[y][j];
                    break;
                    case 6:
					M[y+135][j]=N6[y][j];
                    M1[y+45][j]=N6[y][j];
                    break;
                    case 7:
					M[y+135][j]=N7[y][j];
                    M1[y+45][j]=N7[y][j];
                    break;
                    case 8:
					M[y+135][j]=N8[y][j];
                    M1[y+45][j]=N8[y][j];
                    break;
                    case 9:
					M[y+135][j]=N9[y][j];
                    M1[y+45][j]=N9[y][j];
                    break;
                    }
                    switch (TempRes) {
					case 0:
					M[y+125][j]=N0[y][j];
                    M1[y+35][j]=N0[y][j];
                    break;
                    case 1:
					M[y+125][j]=N1[y][j];
                    M1[y+35][j]=N1[y][j];
                    break;
                    case 2:
					M[y+125][j]=N2[y][j];
                    M1[y+35][j]=N2[y][j];
                    break;
                    case 3:
					M[y+125][j]=N3[y][j];
                    M1[y+35][j]=N3[y][j];
                    break;
                    case 4:
					M[y+125][j]=N4[y][j];
                    M1[y+35][j]=N4[y][j];
                    break;
                    case 5:
					M[y+125][j]=N5[y][j];
                    M1[y+35][j]=N5[y][j];
                    break;
                    case 6:
					M[y+125][j]=N6[y][j];
                    M1[y+35][j]=N6[y][j];
                    break;
                    case 7:
					M[y+125][j]=N7[y][j];
                    M1[y+35][j]=N7[y][j];
                    break;
                    case 8:
					M[y+125][j]=N8[y][j];
                    M1[y+35][j]=N8[y][j];
                    break;
                    case 9:
					M[y+125][j]=N9[y][j];
                    M1[y+35][j]=N9[y][j];
                    break;
                    }
                    M[y+115][j]=C[y][j];
                    M1[y+25][j]=C[y][j];
                    //para la fecha
                    switch (DiaDec) {
					case 0:
					M[y+100][j]=N0[y][j];
                    M1[y+10][j]=N0[y][j];
                    break;
                    case 1:
					M[y+100][j]=N1[y][j];
                    M1[y+10][j]=N1[y][j];
                    break;
                    case 2:
					M[y+100][j]=N2[y][j];
                    M1[y+10][j]=N2[y][j];
                    break;
                    case 3:
					M[y+100][j]=N3[y][j];
                    M1[y+10][j]=N3[y][j];
                    break;
                    case 4:
					M[y+100][j]=N4[y][j];
                    M1[y+10][j]=N4[y][j];
                    break;
                    case 5:
					M[y+100][j]=N5[y][j];
                    M1[y+10][j]=N5[y][j];
                    break;
                    case 6:
					M[y+100][j]=N6[y][j];
                    M1[y+10][j]=N6[y][j];
                    break;
                    case 7:
					M[y+100][j]=N7[y][j];
                    M1[y+10][j]=N7[y][j];
                    break;
                    case 8:
					M[y+100][j]=N8[y][j];
                    M1[y+10][j]=N8[y][j];
                    break;
                    case 9:
					M[y+100][j]=N9[y][j];
                    M1[y+10][j]=N9[y][j];
                    break;
                    }                 
                    switch (DiaRes) {
					case 0:
					M[y+90][j]=N0[y][j];
                    M1[y][j]=N0[y][j];
                    break;
                    case 1:
					M[y+90][j]=N1[y][j];
                    M1[y][j]=N1[y][j];
                    break;
                    case 2:
					M[y+90][j]=N2[y][j];
                    M1[y][j]=N2[y][j];
                    break;
                    case 3:
					M[y+90][j]=N3[y][j];
                    M1[y][j]=N3[y][j];
                    break;
                    case 4:
					M[y+90][j]=N4[y][j];
                    M1[y][j]=N4[y][j];
                    break;
                    case 5:
					M[y+90][j]=N5[y][j];
                    M1[y][j]=N5[y][j];
                    break;
                    case 6:
					M[y+90][j]=N6[y][j];
                    M1[y][j]=N6[y][j];
                    break;
                    case 7:
					M[y+90][j]=N7[y][j];
                    M1[y][j]=N7[y][j];
                    break;
                    case 8:
					M[y+90][j]=N8[y][j];
                    M1[y][j]=N8[y][j];
                    break;
                    case 9:
					M[y+90][j]=N9[y][j];
                    M1[y][j]=N9[y][j];
                    break;
                    }
                    
                    M[y+80][j]=Diag[y][j];
                    M1[y+170][j]=Diag[y][j];
                    
                    switch (MesDec) {
					case 0:
					M[y+70][j]=N0[y][j];
                    M1[y+160][j]=N0[y][j];
                    break;
                    case 1:
					M[y+70][j]=N1[y][j];
                    M1[y+160][j]=N1[y][j];
                    break;
                    }
                    switch (MesRes) {
					case 0:
					M[y+60][j]=N0[y][j];
                    M1[y+150][j]=N0[y][j];
                    break;
                    case 1:
					M[y+60][j]=N1[y][j];
                    M1[y+150][j]=N1[y][j];
                    break;
                    case 2:
					M[y+60][j]=N2[y][j];
                    M1[y+150][j]=N2[y][j];
                    break;
                    case 3:
					M[y+60][j]=N3[y][j];
                    M1[y+150][j]=N3[y][j];
                    break;
                    case 4:
					M[y+60][j]=N4[y][j];
                    M1[y+150][j]=N4[y][j];
                    break;
                    case 5:
					M[y+60][j]=N5[y][j];
                    M1[y+150][j]=N5[y][j];
                    break;
                    case 6:
					M[y+60][j]=N6[y][j];
                    M1[y+150][j]=N6[y][j];
                    break;
                    case 7:
					M[y+60][j]=N7[y][j];
                    M1[y+150][j]=N7[y][j];
                    break;
                    case 8:
					M[y+60][j]=N8[y][j];
                    M1[y+150][j]=N8[y][j];
                    break;
                    case 9:
					M[y+60][j]=N9[y][j];
                    M1[y+150][j]=N9[y][j];
                    break;
                    }
                    M[y+50][j]=Diag[y][j];
                    M1[y+140][j]=Diag[y][j];
                    
                    switch (yearDec) {
					case 0:
					M[y+40][j]=N0[y][j];
                    M1[y+130][j]=N0[y][j];
                    break;
                    case 1:
					M[y+40][j]=N1[y][j];
                    M1[y+130][j]=N1[y][j];
                    break;
                    case 2:
					M[y+40][j]=N2[y][j];
                    M1[y+130][j]=N2[y][j];
                    break;
                    case 3:
					M[y+40][j]=N3[y][j];
                    M1[y+130][j]=N3[y][j];
                    break;
                    case 4:
					M[y+40][j]=N4[y][j];
                    M1[y+130][j]=N4[y][j];
                    break;
                    case 5:
					M[y+40][j]=N5[y][j];
                    M1[y+130][j]=N5[y][j];
                    break;
                    case 6:
					M[y+40][j]=N6[y][j];
                    M1[y+130][j]=N6[y][j];
                    break;
                    case 7:
					M[y+40][j]=N7[y][j];
                    M1[y+130][j]=N7[y][j];
                    break;
                    case 8:
					M[y+40][j]=N8[y][j];
                    M1[y+130][j]=N8[y][j];
                    break;
                    case 9:
					M[y+40][j]=N9[y][j];
                    M1[y+130][j]=N9[y][j];
                    break;
                    }
                    switch (yearRes) {
					case 0:
					M[y+30][j]=N0[y][j];
                    M1[y+120][j]=N0[y][j];
                    break;
                    case 1:
					M[y+30][j]=N1[y][j];
                    M1[y+120][j]=N1[y][j];
                    break;
                    case 2:
					M[y+30][j]=N2[y][j];
                    M1[y+120][j]=N2[y][j];
                    break;
                    case 3:
					M[y+30][j]=N3[y][j];
                    M1[y+120][j]=N3[y][j];
                    break;
                    case 4:
					M[y+30][j]=N4[y][j];
                    M1[y+120][j]=N4[y][j];
                    break;
                    case 5:
					M[y+30][j]=N5[y][j];
                    M1[y+120][j]=N5[y][j];
                    break;
                    case 6:
					M[y+30][j]=N6[y][j];
                    M1[y+120][j]=N6[y][j];
                    break;
                    case 7:
					M[y+30][j]=N7[y][j];
                    M1[y+120][j]=N7[y][j];
                    break;
                    case 8:
					M[y+30][j]=N8[y][j];
                    M1[y+120][j]=N8[y][j];
                    break;
                    case 9:
					M[y+30][j]=N9[y][j];
                    M1[y+120][j]=N9[y][j];
                    break;
                    }
                    
                    
            }
        }
      

    }
}

// INICIAR SERVIDOR WEB (MODIFICADO)
// --------------------------------------------------------------------------------------------------
static httpd_handle_t Iniciar_Servidor_Web(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id = 1;
    config.max_uri_handlers =6;
	config.stack_size          = 8192;
	config.lru_purge_enable    = true;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) return NULL;
		
    // --- Manejadores GET para los archivos (del ejemplo original) ---
    httpd_uri_t root_get = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
    httpd_register_uri_handler(server, &root_get);
    httpd_uri_t mainhtml_get = { .uri = "/Reloj_POV.html", .method = HTTP_GET, .handler = mainhtml_get_handler };
    httpd_register_uri_handler(server, &mainhtml_get);
    httpd_uri_t favicon_get = { .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_get_handler };
    httpd_register_uri_handler(server, &favicon_get);
    httpd_uri_t temp_uri = {
        .uri = "/set-temp", 
        .method = HTTP_GET, 
        .handler = set_temp_handler
    };
    httpd_register_uri_handler(server, &temp_uri);
    // --- ¡NUEVO! Manejadores GET para nuestra API de comandos ---

    httpd_uri_t date_uri = {
        .uri = "/set-time", .method = HTTP_GET, .handler = set_date_handler
    };
    httpd_register_uri_handler(server, &date_uri);

    httpd_uri_t motor_dir_uri = {
        .uri = "/set-motor", .method = HTTP_GET, .handler = set_dir_handler
    };
    httpd_register_uri_handler(server, &motor_dir_uri);

    return server;
}
// MANEJADORES DE LA API DE COMANDOS
// --------------------------------------------------------------------------------------------------
static esp_err_t set_date_handler(httpd_req_t *req) {
    char buf[150];
    char value[10]; // Buffer temporal para cada valor

    // 1. Obtener la cadena de consulta (query string) de la URL
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        
        ESP_LOGI(TAG, "Recibido comando de hora: %s", buf);

        // 2. Buscar y extraer cada variable
        // atoi() convierte de ASCII (texto) a Integer (número entero)
        
        if (httpd_query_key_value(buf, "h", value, sizeof(value)) == ESP_OK) {
            Time = atoi(value); 
        }
        if (httpd_query_key_value(buf, "m", value, sizeof(value)) == ESP_OK) {
            TimeMin = atoi(value);
        }
        if (httpd_query_key_value(buf, "s", value, sizeof(value)) == ESP_OK) {
            Segs = atoi(value);
        }
        if (httpd_query_key_value(buf, "D", value, sizeof(value)) == ESP_OK) {
            Dia = atoi(value);
        }
        if (httpd_query_key_value(buf, "MM", value, sizeof(value)) == ESP_OK) {
            Mes = atoi(value);
        }
        if (httpd_query_key_value(buf, "Y", value, sizeof(value)) == ESP_OK) {
            year = atoi(value) % 100; // Aseguramos que sea formato de 2 dígitos (ej. 2025 -> 25)
        }
        
        // Forzamos una actualización inmediata de la matriz para que se vea el cambio
        xTaskNotifyGive(Handle_Tarea_Actualizacion_Matriz);
        
        ESP_LOGI(TAG, "Hora actualizada a: %02d:%02d:%02d Fecha: %02d/%02d/%02d", 
                 Time, TimeMin, Segs, Dia, Mes, year);
                 
        httpd_resp_send(req, "Hora Sincronizada OK", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send_404(req);
    }
    return ESP_OK;
}
static esp_err_t set_dir_handler(httpd_req_t *req) {
    char buf[100];
    char value[20];

    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        
        if (httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
            // atof() convierte de ASCII a Float
            float nueva_dir = atof(value);
            
            // Actualizamos la variable global 'dir'
            dir = nueva_dir;
            
            ESP_LOGI(TAG, "Dirección/Offset motor actualizado a: %.2f", dir);
            
            // Opcional: Si el motor está en modo estático (dir = -0.5 o similar),
            // podrías querer forzar el timer aquí, pero tu Tarea_Proceso ya lo hace
            // dinámicamente en el siguiente ciclo.
        }
        
        httpd_resp_send(req, "Motor Set OK", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send_404(req);
    }
    return ESP_OK;
}

static esp_err_t set_temp_handler(httpd_req_t *req) {
    char buf[100];
    char value[10];

    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        // Buscamos el parametro "val"
        if (httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
            // Convertimos texto a entero y actualizamos la variable global
            Temp = atoi(value); 
            
            // Forzamos actualización de la matriz para ver el cambio inmediato
            if(Handle_Tarea_Actualizacion_Matriz != NULL) {
                xTaskNotifyGive(Handle_Tarea_Actualizacion_Matriz);
            }
            
            ESP_LOGI(TAG, "Temperatura actualizada via Web: %d C", Temp);
        }
        httpd_resp_send(req, "Temp OK", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send_404(req);
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

// GET para "/"
static esp_err_t root_get_handler(httpd_req_t *req) {
    const char *page = "/Datos/Reloj_POV.html"; // Asume que tu HTML se llama 'main.html'
    httpd_resp_set_type(req, "text/html");
    return EnviarArchivo(req, page);
}

// GET para "main.html"
static esp_err_t mainhtml_get_handler(httpd_req_t *req) {
	char filepath[128];
	strlcpy(filepath, "/Datos", sizeof(filepath));
    strlcat(filepath, req->uri, sizeof(filepath));
    ESP_LOGI(TAG, "Peticion: %s --> %s", req->uri, filepath);
    httpd_resp_set_type(req, get_mime_type(filepath));
    return EnviarArchivo(req, filepath);
}

// GET para "favicon.ico"
static esp_err_t favicon_get_handler(httpd_req_t *req) {
	char filepath[128];
	strlcpy(filepath, "/Datos", sizeof(filepath));
    strlcat(filepath, req->uri, sizeof(filepath));
    ESP_LOGI(TAG, "Peticion: %s --> %s", req->uri, filepath);
    httpd_resp_set_type(req, get_mime_type(filepath));
    return EnviarArchivo(req, filepath);
}

// Función para enviar archivos desde SPIFFS
static esp_err_t EnviarArchivo(httpd_req_t *req, const char *path){
    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "No se pudo abrir archivo: %s", path);
        // Si no se encuentra, envía el HTML principal
        // (esto es útil si el navegador pide algo como /index.html)
        f = fopen("/Datos/main.html", "rb");
        if (!f) {
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
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

// Función para obtener el tipo MIME
static const char *get_mime_type(const char *path){
    const char *extencion = strrchr(path, '.');
    if (!extencion) return "text/plain";
    extencion++;
	if (strcasecmp(extencion, "html") == 0 || strcasecmp(extencion, "htm") == 0) return "text/html";
	if (strcasecmp(extencion, "css") == 0) return "text/css";
	if (strcasecmp(extencion, "js") == 0 || strcasecmp(extencion, "mjs") == 0) return "application/javascript";
    if (strcasecmp(extencion, "jpg") == 0 || strcasecmp(extencion, "jpeg") == 0) return "image/jpeg";
    if (strcasecmp(extencion, "png") == 0)  return "image/png";
    if (strcasecmp(extencion, "gif") == 0)  return "image/gif";
    if (strcasecmp(extencion, "ico") == 0)  return "image/x-icon";
    return "text/plain";
}

void app_main(void)
{

    //inicialirzar pagina
    nvs_flash_init();
    FileSystemInit();
    WiFi_STA_Initialization();
    
    Iniciar_Servidor_Web();
    ESP_LOGI(TAG, "Servidor listo");
    
	//gpios
	gpio_config_t io_conf;
	io_conf.pin_bit_mask = (1ULL<<LED1b | 1ULL<<LED2b | 1ULL<<LED3b | 1ULL<<LED4b | 1ULL<<LED5b | 1ULL<<LED6b | 1ULL<<LED7b | 1ULL<<LED7 | 1ULL<<LED6 | 1ULL<<LED5 | 1ULL<<LED4 | 1ULL<<LED3 | 1ULL<<LED1 | 1ULL<<LED2);
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;       // GPIO_PULLUP_ENABLE
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;   // GPIO_PULLDOWN_ENABLE
	io_conf.intr_type = GPIO_INTR_DISABLE;      // GPIO_INTR_POSEDGE, GPIO_INTR_NEGEDGE, GPIO_INTR_ANYEDG, GPIO_INTR_LOW_LEVEL, GPIO_INTR_HIGH_LEVEL
	gpio_config(&io_conf);
	//timers
	gptimer_config_t Timer_Config = {
		.clk_src = GPTIMER_CLK_SRC_APB,			
		.direction = GPTIMER_COUNT_UP, 			
		.resolution_hz = 1000000,				// Frecuencia de reloj del GPTimer.																						
	};
	//gptimer_handle_t gptimer_grado = NULL; esta se volvio global para poder usarlo en Tarea_proceso
	gptimer_new_timer(&Timer_Config, &gptimer_grado);
	//calculo de duracion del timer
	float alarm_duration_s = 1.0f / ( (float)Vel * 180.0f );
    alarm_count_us = (uint64_t)(alarm_duration_s * 1000000.0f);//esta parte es nesesaria para que sea un entero
    
		gptimer_alarm_config_t CambioGrado = {
		.alarm_count = alarm_count_us,  //micro segundos
		.reload_count = 0,
		.flags.auto_reload_on_alarm = true
	};
	gptimer_set_alarm_action(gptimer_grado, &CambioGrado);
	gptimer_event_callbacks_t EventCambioGrado = {
		.on_alarm = CambioGradoCallback,
	};
	gptimer_register_event_callbacks(gptimer_grado, &EventCambioGrado, NULL);
	
	//timer de aumento de hora
	//gptimer_handle_t gptimer_AumentoHora = NULL;
	gptimer_new_timer(&Timer_Config, &gptimer_AumentoHora);
	gptimer_alarm_config_t CambioHora = {
		.alarm_count = 1000000,  //micro segundos
		.reload_count = 0,
		.flags.auto_reload_on_alarm = true
	};
	gptimer_set_alarm_action(gptimer_AumentoHora, &CambioHora);
	gptimer_event_callbacks_t EventCambioHora = {
		.on_alarm = CambioHoraCallback,
	};
	gptimer_register_event_callbacks(gptimer_AumentoHora, &EventCambioHora, NULL);
	//tareas
	xTaskCreatePinnedToCore(Tarea_Actualizar_Matriz, "ActualizarMatriz", 2048, NULL, 4, &Handle_Tarea_Actualizacion_Matriz, 0);
	xTaskNotifyGive(Handle_Tarea_Actualizacion_Matriz);
	xTaskCreatePinnedToCore(Tarea_Proceso, "Calculo", 4096, NULL, 5, &Handle_Tarea_Calculo,1);
	xTaskCreatePinnedToCore(Tarea_AumHora, "AumHora", 4096, NULL, 5, &Handle_Tarea_AumHora, 1);
	
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
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(Handler_CaptureChannel, &Configuracion_Capture_Callback, NULL));

    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(Handler_CaptureChannel));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(Handle_CaptureTimer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(Handle_CaptureTimer));

	
	gptimer_enable(gptimer_grado);
	gptimer_start(gptimer_grado);
	gptimer_enable(gptimer_AumentoHora);
	gptimer_start(gptimer_AumentoHora);
	
 
}