/*
Range-Angle Function
- RX    : 2
- CHIRP : 8
- SAMPLE: 256
*/

#include "esp_bgt60.h"
#include "esp_err.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define IRQ             GPIO_NUM_17
#define RST             GPIO_NUM_4

#define MISO            GPIO_NUM_19
#define MOSI            GPIO_NUM_23
#define SCLK            GPIO_NUM_18
#define CSN             GPIO_NUM_5
#define SPI_CLK_SPEED   40*1000*1000
#define SPI_ESP_HOST    VSPI_HOST 
#define MAX_BYTE        24576       

#define UART_PORT       UART_NUM_0
#define TX0             GPIO_NUM_1
#define RX0             GPIO_NUM_3
#define BAUDRATE        921600


extern gpio_esp_t gpio_cfg;
extern spi_esp_t spi_cfg;
extern uart_esp_t uart_cfg;

static uint8_t fifo_buf[6144] = {0};
static uint16_t ext_buf[4096] = {0};
static uint16_t rx1_buf[2048] = {0};
static uint16_t rx2_buf[2048] = {0};

const static size_t fifo_size = sizeof(fifo_buf)/sizeof(fifo_buf[0]);
const static size_t rx_size = sizeof(rx1_buf)/sizeof(rx1_buf[0]);

static volatile bool isr_state = false;
static TaskHandle_t isr_task_handle = NULL;

void IRAM_ATTR isr_handler(void* arg) {
    BaseType_t xHighPriorityTaskWoken = pdFALSE;
    if(!isr_state && isr_task_handle != NULL) {
        isr_state = true;
        vTaskNotifyGiveFromISR(isr_task_handle, &xHighPriorityTaskWoken);   
    }
    portYIELD_FROM_ISR(xHighPriorityTaskWoken);
}

void spi_acquire_task(void* arg) {
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        toggle_flag(MAIN, FSM_RST);
        read_fifo(fifo_buf);

        toggle_flag(MAIN, FIFO_RST);
        extract_fifo(fifo_buf, fifo_size, ext_buf, 2, rx1_buf, rx2_buf, NULL);

        send_uart(2, rx1_buf, rx2_buf, NULL, rx_size);

        isr_state = false;
        toggle_flag(MAIN, START);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
}

void app_main() {
        gpio_cfg = (gpio_esp_t) {
        .csn = CSN,
        .irq = IRQ,
        .rst = RST,
    };
    ESP_ERROR_CHECK(init_gpio(&gpio_cfg));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(gpio_cfg.irq, isr_handler, NULL);

    spi_cfg = (spi_esp_t) {
        .spi_clk_speed = SPI_CLK_SPEED,
        .spi_host = SPI_ESP_HOST,
        .miso = MISO,
        .mosi = MOSI,
        .sclk = SCLK,
        .csn = CSN,
        .max_byte = MAX_BYTE,
    };
    ESP_ERROR_CHECK(init_spi(&spi_cfg));

    uart_cfg = (uart_esp_t) {
        .baudrate = BAUDRATE,
        .uart_port = UART_PORT,
        .tx = TX0,
        .rx = RX0,
    };
    ESP_ERROR_CHECK(init_uart(&uart_cfg));

    hard_reset();
    soft_reset();
    init_sensor();

    set_cref(2047);
    rx_enable(1, 1, 0, 1);
    mixer_enable(1, 1, 0, 1);
    lo_enable(1, 1, 0, 1);
    set_chirp(1, 3);
    set_rsu(1, 661);
    set_rtu(1, 1340);
    set_tstart(299);
    
    xTaskCreate(spi_acquire_task, "SPI", 4096, NULL, 1, &isr_task_handle);
}