/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "soc/uart_reg.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_log.h"

#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "hal/timer_hal.h"
#include "driver/ledc.h"

/**
 * This is an example which echos any data it receives on UART1 back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: UART1
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below
 */

#define ECHO_TEST_TXD  (GPIO_NUM_16)
#define ECHO_TEST_RXD  (GPIO_NUM_19)
#define ECHO_TEST_RTS  (2)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define ECHO_TEST_TXD2  (GPIO_NUM_21)
#define ECHO_TEST_RXD2  (GPIO_NUM_22)
#define ECHO_TEST_RTS2  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS2  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)

static void echo_task(void *arg)
{
	int uart_num = (int)arg;
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    esp_err_t err = uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);
    ESP_LOGI(__func__, "uart_driver_install(%d,) 0x%x", uart_num, err);
    err = uart_param_config(uart_num, &uart_config);
    ESP_LOGI(__func__, "uart_param_config(%d,) 0x%x", uart_num, err);
    if (uart_num == 1)
    	err = uart_set_pin(uart_num, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    else
    	err = uart_set_pin(uart_num, ECHO_TEST_TXD2, ECHO_TEST_RXD2, ECHO_TEST_RTS2, ECHO_TEST_CTS2);
    ESP_LOGI(__func__, "uart_set_pin(%d,) 0x%x", uart_num, err);

    uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);
    uart_set_rts(uart_num, 1); // receive mode

    uart_intr_config_t uart_intr = {
        .intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M
                            | UART_RXFIFO_TOUT_INT_ENA_M
                            | UART_FRM_ERR_INT_ENA_M
                            | UART_RXFIFO_OVF_INT_ENA_M
                            | UART_BRK_DET_INT_ENA_M
                            | UART_PARITY_ERR_INT_ENA_M,
        .rxfifo_full_thresh = 32, // размер данных после которого генерируется UART_DATA
        .rx_timeout_thresh = 2, // таймаут после которого генерируется UART_DATA (в символах)
        .txfifo_empty_intr_thresh = 10
    };
    err = uart_intr_config(uart_num, &uart_intr);
	ESP_LOGI(__func__, "uart_intr_config(%d,) 0x%x.", uart_num, err);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(uart_num, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        uart_write_bytes(uart_num, (const char *) data, len);
    }
}

volatile uint32_t timer_counters[2][2];
static void IRAM_ATTR timer_isr_handler(void *param)
{
	uint32_t tgroup = (uint32_t)param;
	int tidx = 0;
    timer_intr_t timer_intr = timer_group_get_intr_status_in_isr(tgroup);
    if ((timer_intr & TIMER_INTR_T0) != 0)
    {
    	tidx = 0;
    } else if ((timer_intr & TIMER_INTR_T1) != 0)
    {
    	tidx = 1;
    }
	timer_counters[tgroup][tidx]++;
	timer_group_clr_intr_status_in_isr(tgroup, tidx);
	if (tgroup != 0) {
		timer_ll_set_counter_enable(TIMER_LL_GET_HW(tgroup), tidx, false); // timer_pause()
		timer_ll_set_counter_value(TIMER_LL_GET_HW(tgroup), tidx, 0); //timer_set_counter_value()
	}
    timer_ll_set_alarm_enable(TIMER_LL_GET_HW(tgroup), tidx, true); //timer_group_enable_alarm_in_isr()
}
void init_timer_(uint32_t tgroup, uint32_t tidx, uint32_t counter, int reload)
{
	esp_err_t xErr;
	timer_config_t config;
	config.alarm_en = TIMER_ALARM_EN;
	config.auto_reload = reload;
	config.counter_dir = TIMER_COUNT_UP;
	config.divider = 80; // 80MHz / 80 = 1 MHz
	config.intr_type = TIMER_INTR_LEVEL;
	config.counter_en = TIMER_PAUSE;
	// Configure timer
	xErr = timer_init(tgroup, tidx, &config);
	ESP_LOGI(__func__, "timer_init(%d, %d) 0x%x", tgroup, tidx, xErr);
	// Stop timer counter
	xErr = timer_pause(tgroup, tidx);
	ESP_LOGI(__func__, "timer_pause(%d, %d) 0x%x", tgroup, tidx, xErr);
	// Reset counter value
	xErr = timer_set_counter_value(tgroup, tidx, 0);
	ESP_LOGI(__func__, "timer_set_counter_value(%d, %d) 0x%x", tgroup, tidx, xErr);
	xErr = timer_set_alarm_value(tgroup, tidx, (uint32_t)counter);
	ESP_LOGI(__func__, "timer_set_alarm_value(%d, %d) 0x%x", tgroup, tidx, xErr);
	xErr = timer_isr_register(tgroup, tidx, timer_isr_handler, (void*)tgroup, ESP_INTR_FLAG_IRAM, NULL);
	ESP_LOGI(__func__, "timer_isr_register(%d, %d) 0x%x", tgroup, tidx, xErr);
	xErr = timer_enable_intr(tgroup, tidx);
    ESP_LOGI(__func__, "timer_enable_intr(%d, %d) 0x%x", tgroup, tidx, xErr);
    xErr = timer_start(tgroup, tidx);
    ESP_LOGI(__func__, "timer_start(%d, %d) 0x%x", tgroup, tidx, xErr);
}

#define PIN_INT (34)
IRAM_ATTR static void gpio_isr_handler(void* arg) {}
bool init_data_ready_int(void) {
	esp_err_t ret = ESP_OK;
	gpio_config_t io_conf;
	//interrupt of rising edge
	io_conf.intr_type = GPIO_INTR_POSEDGE;//GPIO_INTR_POSEDGE;//GPIO_INTR_NEGEDGE;//GPIO_INTR_ANYEDGE;//
	io_conf.pin_bit_mask = ((uint64_t)1 << PIN_INT);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_down_en = 0;//1;//0;
	io_conf.pull_up_en = 0;//1;
	ret |= gpio_config(&io_conf);
#if 0
	ret |= gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
	ret |= gpio_isr_handler_add(PIN_INT, gpio_isr_handler, (void*)PIN_INT);
#else
	//gpio_isr_register(void (*fn)(void *), void *arg, int intr_alloc_flags, gpio_isr_handle_t *handle)
	ret |= gpio_isr_register(gpio_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);
#endif

	ESP_LOGI(__func__, "0x%x", ret);
	return (ret == ESP_OK);
}

bool ledc_init(void) {
	esp_err_t res = ESP_OK;
	ledc_timer_config_t ledc_timer = {
		.duty_resolution = LEDC_TIMER_5_BIT,// больше 5 бит вызывает ошибку инициализации модуля (зависит от значения .freq_hz) // resolution of PWM duty
        .freq_hz = 2048000,
		.speed_mode = LEDC_HIGH_SPEED_MODE,           // timer mode
		.timer_num = LEDC_TIMER_0,            // timer index
		.clk_cfg = LEDC_AUTO_CLK,//LEDC_USE_APB_CLK,              // Auto select the source clock
	};
	res |= ledc_timer_config(&ledc_timer);

	ledc_channel_config_t ledc_channel = {
			.channel    = LEDC_CHANNEL_1,
			//.duty       = 8, // скважность 50% для разрешения LEDC_TIMER_4_BIT
			.duty       = 32, // скважность % от 2^LEDC_TIMER_x_BIT
			.gpio_num   = 4,
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_TIMER_0
	};
	res |= ledc_channel_config(&ledc_channel);
	ESP_LOGI(__func__, "ledc_channel_config() 0x%x", res);
	return (res == ESP_OK);
}
void app_main(void)
{
	vTaskDelay(pdMS_TO_TICKS(1000));
	ESP_LOGI(__func__, "start\r\n");
	//menu
	ledc_init();
    init_timer_(TIMER_GROUP_0, TIMER_0, 1000,   TIMER_AUTORELOAD_EN);
    //uart
    xTaskCreate(echo_task, "uart_echo1", 4096, (void*)UART_NUM_1, 10, NULL);
    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreate(echo_task, "uart_echo2", 4096, (void*)UART_NUM_2, 10, NULL);
    vTaskDelay(pdMS_TO_TICKS(1000));
    //modbus
    init_timer_(TIMER_GROUP_1, TIMER_0, 10000,  TIMER_AUTORELOAD_DIS);
    init_timer_(TIMER_GROUP_1, TIMER_1, 100000, TIMER_AUTORELOAD_DIS);
    //adc
	init_data_ready_int();
	while(1) {
		ESP_LOGI(__func__, "%u %u %u %u", timer_counters[0][0], timer_counters[0][1], timer_counters[1][0], timer_counters[1][1]);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
