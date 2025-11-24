#include <inttypes.h>
#include <math.h>
#include "esp_bgt60.h"
#include "esp_log.h"
#include "esp_err.h"

//#define dbg_msg

static const char* T = "ESP_BGT60";
static esp_err_t ret;

gpio_esp_t gpio_cfg;
spi_esp_t spi_cfg;
uart_esp_t uart_cfg;

static spi_device_handle_t spi;

typedef struct {
    uint8_t address;
    uint32_t data;
} reg_init_t; 

const reg_init_t init_register[] = {
    {MAIN       , 0x1E8780},
    {ADC0       , 0x0A0B60},    // DIV = 40
    {PARC1      , 0xE861ED},
    {PARC2      , 0x0400F4},
    {SFCTL      , 0x713FFF},    // CREF = 8192 
    {SADC       , 0x000110}, 
    {CSI0       , 0x000000},
    {CSI1       , 0x000000},
    {CSI2       , 0x000000},
    {CSIC       , 0x000BE0},
    {CSDS0      , 0x000000},
    {CSDS1      , 0x000000},
    {CSDS2      , 0x000000},
    {CSDSC      , 0x000B60},
    {CSU10      , 0x103053},    // LO:RX1, MIXER:RX1, LOD:Disable
    {CSU11      , 0x1A141F},    // Transmission_power:31, Baseband:RX1, Temp_sensor:Enable
    {CSU12      , 0x706739},    // Gain_start:18dB, Gain_sat:30dB, HPF:70kHz
    {CSC1       , 0x000496},    // REPC 64 Chirp
    {CSC2       , 0x000B60},
    {CSC3       , 0x000B60},
    {CSC4       , 0x000B60},
    {CCR0       , 0xDF521D},    // TR_INIT1 
    {CCR1       , 0x112B16},    // TR_START 279 
    {CCR2       , 0x000001},
    {CCR3       , 0xDF7D2C},    // TR_INIT0, TR_SSTART 30 , TR_PAEN 300 
    {PLL10      , 0xAA0000},    // FSU 58 GHz
    {PLL11      , 0x000252},    // RSU 594 
    {PLL12      , 0x0A0564},    // RTU 1380 
    {PLL13      , 0x000100},    // APU 256 
    {PLL14      , 0x000000},
    {PLL15      , 0x000000},
    {PLL16      , 0x0A0000},
    {PLL17      , 0x187B16},    // REPS 64 Chirp
    {PLL27      , 0x000000},
    {PLL37      , 0x000000},
    {PLL47      , 0x000000}
};

/* =============== DRIVER SETTING FUNCTION =============== */


esp_err_t init_gpio(const gpio_esp_t* config) {
    /*
    Initialize the GPIO pin.
    - IRQ: INPUT, PULLDOWN, INTR(RISING_EDGE)
    - RST: OUTPUT, PULLUP
    - CSN: OUTPUT, PULLUP
    */

    gpio_cfg = *config;

    ESP_LOGI(T, "Start GPIO initialize");

    gpio_config_t irq_conf = {
        .pin_bit_mask = (1ULL << gpio_cfg.irq),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ret = gpio_config(&irq_conf);
    if(ret != ESP_OK) { 
        ESP_LOGE(T, "Can't Initial IRQ");
        return ret; 
    }

    gpio_config_t csn_conf = {
        .pin_bit_mask = (1ULL << gpio_cfg.csn),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ret = gpio_config(&csn_conf);
    if(ret != ESP_OK) { 
        ESP_LOGE(T, "Can't Initial CSN");
        return ret; 
    }

    gpio_set_level(gpio_cfg.csn, 1);

    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << gpio_cfg.rst),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ret = gpio_config(&rst_conf);
    if(ret != ESP_OK) { 
        ESP_LOGE(T, "Can't Initial RST");
        return ret; 
    }

    gpio_set_level(gpio_cfg.rst, 1);

    ESP_LOGI(T, "Success GPIO initialize");
    return ESP_OK;
}

esp_err_t init_spi(const spi_esp_t* config) {
    /*
    Initialize the SPI pin and settings.
    - CSN: manual control
    - mode: SPI_MODE0
    - Max_transfer_size: 24576 Byte (8192 WORD of BGT60xx FIFO)
    */

    spi_cfg = *config;

    spi_bus_config_t buscfg = {
        .miso_io_num = spi_cfg.miso,
        .mosi_io_num = spi_cfg.mosi,
        .sclk_io_num = spi_cfg.sclk,
        .max_transfer_sz = 24576,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .flags = 0,
    };
    ret = spi_bus_initialize(spi_cfg.spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if(ret != ESP_OK) { return ret; }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = spi_cfg.spi_clk_speed,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 7,
        .input_delay_ns = 20,
        .flags = 0,
    };
    ret = spi_bus_add_device(spi_cfg.spi_host, &devcfg, &spi);
    if(ret != ESP_OK) { return ret; }

    if(spi == NULL) { 
        ESP_LOGE(T, "SPI NOT INITIALIZATION ...");
        return ret; 
    }

    return ESP_OK;
}

esp_err_t init_uart(const uart_esp_t* config) {
    /*
    Initialize the UART pin.
    - PORT: 0 (for serial debugging)
    - Parity: Disable
    - Flowcontrol: Disable
    */

    uart_cfg = *config;

    uart_config_t uart_conf = {
        .baud_rate = uart_cfg.baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ret = uart_param_config(uart_cfg.uart_port, &uart_conf);
    if(ret != ESP_OK) { return ret; }

    ret = uart_set_pin(uart_cfg.uart_port, uart_cfg.tx, uart_cfg.rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if(ret != ESP_OK) { return ret; }

    ret = uart_driver_install(uart_cfg.uart_port, 2048, 0, 0, NULL, 0);
    if(ret != ESP_OK) { return ret; }

    return ESP_OK;
}


/* ==================== MAIN FUNCTION ==================== */


uint32_t read_reg(uint8_t address) {
    /*
    Parameters:
    - address: Register address

    Workflow:
    - Send data 32 bit: 
        - [31:25]: ADDRESS
        - [24:24]: R/W (Read 0)
        - [23:00]: DUMMY
    - Receive data 32 bit:
        - [31:24]: ERROR STATUS
        - [23:00]: DATA 
    */

    uint8_t tx_buff[4] = {(address << 1 & 0xfe), 0, 0, 0};
    uint8_t rx_buff[4];

    spi_transaction_t read_t = {
        .tx_buffer = tx_buff,
        .rx_buffer = rx_buff,
        .rxlength = 32,
        .length = 32,
        .flags = 0,
    };

    gpio_set_level(gpio_cfg.csn, 0);
    ret = spi_device_polling_transmit(spi, &read_t);
    gpio_set_level(gpio_cfg.csn, 1);

    if(ret != ESP_OK) {
        ESP_LOGE(T, "READ FAIL : %s", esp_err_to_name(ret));
        return 0x00;
    }

    uint32_t rx = {
        ((uint32_t)rx_buff[0] << 24) |
        ((uint32_t)rx_buff[1] << 16) |
        ((uint32_t)rx_buff[2] << 8 ) |
        ((uint32_t)rx_buff[3])
    };

    return rx;
}

void write_reg(uint8_t address, uint32_t data) {
    /*
    Parameters:
    - address: Register address
    - data: configuration data to register
    Workflow:
    - Send data 32 bit: 
        - [31:25]: ADDRESS
        - [24:24]: R/W (Write 1)
        - [23:00]: Data
    */

    uint8_t tx_buff[4] = {
        (uint8_t)(address << 1 | 0x01),
        (uint8_t)(data >> 16 & 0xff),
        (uint8_t)(data >> 8 & 0xff),
        (uint8_t)(data & 0xff)
    };

    spi_transaction_t write_t = {
        .tx_buffer = tx_buff,
        .length = 32,
        .flags = 0,
    };

    gpio_set_level(gpio_cfg.csn, 0);
    ret = spi_device_polling_transmit(spi, &write_t);
    gpio_set_level(gpio_cfg.csn, 1);

    if(ret != ESP_OK) {
        ESP_LOGE(T, "WRITE FAIL : %s", esp_err_to_name(ret));
    }
}

void toggle_flag(uint8_t address, uint32_t data) {
    /*
    For toggle flag bit without change any configuration.
    */
    uint32_t send_flag = read_reg(address) | (data & 0xffffff);
    write_reg(address, send_flag);
}

void init_sensor() {
    /*
    For initialize starter register configuration.
    */
    size_t count = sizeof(init_register)/sizeof(init_register[0]);
    for(size_t i = 0; i < count; i++) {
        write_reg(init_register[i].address, init_register[i].data);

        #ifdef dbg_msg
        uint32_t readback = read_reg(init_register[i].address);
        ESP_LOGI(T, "[ Addr : 0x%"PRIx8" | Register : 0x%08"PRIX32"]",init_register[i].address, readback);
        #endif
    }
}

void read_fifo(uint8_t* buffer) {
    /*
    Parameters:
    - buffer: buffer for contain Raw ADC from FIFO

    Workflow:
    - send burst command (32 Bits): trigger Register 0x70 to acquire ADC in FIFO
    - send dummy (0x00): amount to receive depend on MAX_BYTE
    */

    static const uint8_t burst_cmd[4] = {0xff, 0xc0, 0x00, 0x00};

    spi_transaction_t burst_t = {
        .tx_buffer = burst_cmd,
        .length = 32,
        .flags = 0,
    };

    uint8_t rx_buff;

    spi_transaction_t read_burst_t = {
        .tx_buffer = 0x00,
        .rx_buffer = &rx_buff,
        .length = 8,
        .rxlength = 8,
        .flags = 0,
    };

    gpio_set_level(gpio_cfg.csn, 0);
    spi_device_polling_transmit(spi, &burst_t);

    for(size_t i = 0; i < spi_cfg.max_byte; i++) {
        spi_device_polling_transmit(spi, &read_burst_t);
        buffer[i] = rx_buff;
    }

    gpio_set_level(gpio_cfg.csn, 1);

    #ifdef dbg_msg
    for(size_t i = 0; i < 22; i+=3) {
        ESP_LOGI(T, "[%"PRIu8" - %"PRIu8"] 0x%02"PRIx8" | 0x%02"PRIx8" | 0x%02"PRIx8, i, i+2, buffer[i], buffer[i+1], buffer[i+2]);
    }
    #endif
}

void extract_fifo(uint8_t* fifo_buff, size_t fifo_size, uint16_t* extract_buff, uint8_t mode, uint16_t* rx1_buff, uint16_t* rx2_buff, uint16_t* rx3_buff) {
    /*
    Parameter:
    - fifo_buff: buffer that contain raw FIFO data
    - fifo_size: amount fifo_buff size to extract
    - extract_buff: buffer for contain extract data from fifo_buff
    - mode: 
        - 1RX (you can use extract_buff same as rx1_buff)
        - 2RX
        - 3RX
    - rx1_buff: rx1 buffer
    - rx2_buff: rx2 buffer
    - rx3_buff: rx3 buffer
    */
    
    uint16_t bidx = 0;
    uint16_t aidx = 0;

    while(bidx + 2 < fifo_size) {
        extract_buff[aidx] = (fifo_buff[bidx] & 0xff) << 4 | (fifo_buff[bidx+1] & 0xf0) >> 4 ;
        extract_buff[aidx+1] = (fifo_buff[bidx+1] & 0x0f) << 8 | (fifo_buff[bidx+2] & 0xff);

        bidx += 3;
        aidx += 2;
    }

    uint16_t cidx = 0;
    uint16_t didx = 0;

    switch (mode)
    {
    case 1: //in case use 1 RX
        while(cidx < fifo_size) {
            rx1_buff[didx] = extract_buff[cidx];
            cidx += 1;
            didx += 1;
        }
        #ifdef dbg_msg 
        for(size_t i = 0; i < 20; i++) {
            ESP_LOGI(T, "RX [%u] 0x%04x", i, rx1_buff[i]);
        }
        #endif 
        break;

    case 2: //in case use 2 RX
        while(cidx + 1 < fifo_size) {
            rx1_buff[didx] = extract_buff[cidx];
            rx2_buff[didx] = extract_buff[cidx+1];
            cidx += 2;
            didx += 1;
        }
        #ifdef dbg_msg 
        for(size_t i = 0; i < 20; i++) {
            ESP_LOGI(T, "RX [%u] 0x%04x | 0x%04x", i, rx1_buff[i], rx2_buff[i]);
        }
        #endif 
        break;

    case 3: //in case use 3 RX
        while(cidx + 2 < fifo_size) {
            rx1_buff[didx] = extract_buff[cidx];
            rx2_buff[didx] = extract_buff[cidx+1];
            rx3_buff[didx] = extract_buff[cidx+2];
            cidx += 3;
            didx += 1;
        }
        #ifdef dbg_msg 
        for(size_t i = 0; i < 20; i++) {
            ESP_LOGI(T, "[%u] 0x%04x | 0x%04x | 0x%04x", i, rx1_buff[i], rx2_buff[i], rx3_buff[i]);
        }
        #endif 
        break;

    default:
        ESP_LOGE(T, "Undefine Extract Mode ...");
        break;
    }
}

void init_value(uint8_t address, uint32_t data, uint32_t masked) {
    /*
    Parameters:
    - address: Register address
    - data: new configuration data
    - masked: bit position of configuration

    Workflow:
    - contain the previous configuration
    - use masked set bit position to 0
    - merge with new configuration data
    - write to register
    */
    uint32_t prev_val = read_reg(address);
    uint32_t curr_val = (prev_val & ~masked) | data;
    write_reg(address, curr_val);
}

void hard_reset() {
    /*
    Reset Pin (RST)
    - logic low: reset
    - logic high: default state
    */
    gpio_set_level(gpio_cfg.rst, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(gpio_cfg.rst, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
}

void soft_reset() {
    /*
    Workflow:
    - SW_RST: software reset (will reset all configuration to default)
    - FSM_RST: Finite state machine reset (will stop working flow on sensor, and going to Deepsleep state)
    - FIFO_RST: FIFO reset (remove data in FIFO) 
    */
    toggle_flag(MAIN, SW_RST);
    vTaskDelay(pdMS_TO_TICKS(1));
    toggle_flag(MAIN, FSM_RST);
    vTaskDelay(pdMS_TO_TICKS(1));
    toggle_flag(MAIN, FIFO_RST);
    vTaskDelay(pdMS_TO_TICKS(1));
}


/* ================ CONFIGURATION FUNCTION ================ */

void rx_enable(uint8_t ch, bool ch1, bool ch2, bool ch3) { // BBCHSEL
    /*
    Use for enable/disable Baseband channel
    Parameters:
    - ch: channel to be configuration
    - ch1: RX1
    - ch2: RX2
    - ch3: RX3
    */
    uint32_t new_value = (uint32_t)ch1 << 20 | (uint32_t)ch2 << 21 | (uint32_t)ch3 << 22;
    
    switch (ch)
    {
    case 1:
        init_value(CSU11, new_value, 0x700000);
        break;
    case 2:
        init_value(CSU21, new_value, 0x700000);
        break;
    case 3:
        init_value(CSU31, new_value, 0x700000);
        break;
    case 4:
        init_value(CSU41, new_value, 0x700000);
        break;
    default:
        ESP_LOGE(T, "Undefine CSUx1 address");
        break;
    }
}

void mixer_enable(uint8_t ch, bool ch1, bool ch2, bool ch3) {
    /*
    Use for enable/disable mixer channel
    Parameters:
    - ch: channel to be configuration
    - ch1: RX1
    - ch2: RX2
    - ch3: RX3
    */
    uint32_t new_value = (uint32_t)ch1 << 13 | (uint32_t)ch2 << 15 | (uint32_t)ch3 << 17;
    
    switch (ch)
    {
    case 1:
        init_value(CSU10, new_value, 0x02a000);
        break;
    case 2:
        init_value(CSU20, new_value, 0x02a000);
        break;
    case 3:
        init_value(CSU30, new_value, 0x02a000);
        break;
    case 4:
        init_value(CSU40, new_value, 0x02a000);
        break;
    default:
        ESP_LOGE(T, "Undefine CSUx0 address");
        break;
    }
}

void lo_enable(uint8_t ch, bool ch1, bool ch2, bool ch3) {
    /*
    Use for enable/disable local ocsillator channel
    Parameters:
    - ch: channel to be configuration
    - ch1: RX1
    - ch2: RX2
    - ch3: RX3
    */
    uint32_t new_value = (uint32_t)ch1 << 12 | (uint32_t)ch2 << 14 | (uint32_t)ch3 << 16;
    
    switch (ch)
    {
    case 1:
        init_value(CSU10, new_value, 0x015000);
        break;
    case 2:
        init_value(CSU20, new_value, 0x015000);
        break;
    case 3:
        init_value(CSU30, new_value, 0x015000);
        break;
    case 4:
        init_value(CSU40, new_value, 0x015000);
        break;
    default:
        ESP_LOGE(T, "Undefine CSUx0 address");
        break;
    }
}

void lod_enable(uint8_t ch, bool lod1, bool lod2) {
    /*
    Use for enable/disable Local oscillator distribution channel
    Parameters:
    - ch: channel to be configuration
    - lod1: RX1-RX3
    - lod2: RX2-TX
    */
    uint32_t new_value = (uint32_t)lod1 << 11 | (uint32_t)lod2 << 10;
    
    switch (ch)
    {
    case 1:
        init_value(CSU10, new_value, 0x000c00);
        break;
    case 2:
        init_value(CSU20, new_value, 0x000c00);
        break;
    case 3:
        init_value(CSU30, new_value, 0x000c00);
        break;
    case 4:
        init_value(CSU40, new_value, 0x000c00);
        break;
    default:
        ESP_LOGE(T, "Undefine CSUx0 address");
        break;
    }
    
}

void set_gain_hpf(uint8_t ch, bool vga1, bool vga2, bool vga3) {
    /*
    Use for set number of starter gain 
    - 0: 18 dB
    - 1: 30 dB
    Parameters:
    - ch: channel to be configuration
    - vga1: RX1
    - vga2: RX2
    - vga3: RX3
    */
    uint32_t new_value = (uint32_t)vga1 << 20 | (uint32_t)vga2 << 21 | (uint32_t)vga3 << 22;
    
    switch (ch)
    {
    case 1:
        init_value(CSU12, new_value, 0x700000);
        break;
    case 2:
        init_value(CSU22, new_value, 0x700000);
        break;
    case 3:
        init_value(CSU32, new_value, 0x700000);
        break;
    case 4:
        init_value(CSU42, new_value, 0x700000);
        break;
    default:
        ESP_LOGE(T, "Undefine CSUx2 address");
        break;
    }
} 

void set_gain_vga(uint8_t ch, uint32_t vga1, uint32_t vga2, uint32_t vga3) {
    /*
    Use for set number of gain
    - use range 0-6
        - 0: 0  dB
        - 1: 5  dB
        - 2: 10 dB
        - 3: 15 dB
        - 4: 20 dB
        - 5: 25 dB
        - 6: 30 dB
    Parameters:
    - ch: channel to be configuration
    - vga1: RX1
    - vga2: RX2
    - vga3: RX3
    */
    if((vga1 <= 6) && (vga2 <= 6) && (vga3 <= 6)) {

        switch (ch)
        {
        case 1:
            init_value(CSU12, (vga1 << 2 | vga2 << 7 | vga3 << 12), 0x00739c);
            break;
        case 2:
            init_value(CSU22, (vga1 << 2 | vga2 << 7 | vga3 << 12), 0x00739c);
            break;
        case 3:
            init_value(CSU32, (vga1 << 2 | vga2 << 7 | vga3 << 12), 0x00739c);
            break;
        case 4:
            init_value(CSU42, (vga1 << 2 | vga2 << 7 | vga3 << 12), 0x00739c);
            break;
        default:
            ESP_LOGE(T, "Undefine CSUx2 address");
            break;
        }
    }
    else{ ESP_LOGE(T, "VGA gain out of limit ! (gain only available 0-6)"); }
}

void set_hpf(uint8_t ch, uint32_t hpf1, uint32_t hpf2, uint32_t hpf3) {
    /*
    Use for set number of cutoff frequency
    - use range 0-3
        - 0: 20 kHz
        - 1: 45 kHz
        - 2: 70 kHz
        - 3: 80 kHz
    Parameters:
    - ch: channel to be configuration
    - hpf1: RX1
    - hpf2: RX2
    - hpf3: RX3
    */
    if((hpf1 <= 3) && (hpf2 <= 3) && (hpf3 <= 3)) {
        
        switch (ch)
        {
        case 1:
            init_value(CSU12, (hpf1 | hpf2 << 5 | hpf3 << 10), 0x000c63);
            break;
        case 2:
            init_value(CSU22, (hpf1 | hpf2 << 5 | hpf3 << 10), 0x000c63);
            break;
        case 3:
            init_value(CSU32, (hpf1 | hpf2 << 5 | hpf3 << 10), 0x000c63);
            break;
        case 4:
            init_value(CSU42, (hpf1 | hpf2 << 5 | hpf3 << 10), 0x000c63);
            break;
        default:
            ESP_LOGE(T, "Undefine CSUx2 address");
            break;
        }
    }
    else{ ESP_LOGE(T, "HPF cutoff out of limit ! (cutoff only available mode 0-3)"); }
}

void set_tx_power(uint8_t ch, uint32_t tx_pwr) {
    /*
    Use for set number of gain
    - use range 0-31
    Parameters:
    - ch: channel to be configuration
    - tx_pwr: Power of transmit
    */
    if(tx_pwr <= 31) {
        
        switch (ch)
        {
        case 1:
            init_value(CSU11, tx_pwr, 0x00001f);
            break;
        case 2:
            init_value(CSU21, tx_pwr, 0x00001f);
            break;
        case 3:
            init_value(CSU31, tx_pwr, 0x00001f);
            break;
        case 4:
            init_value(CSU41, tx_pwr, 0x00001f);
            break;
        default:
            ESP_LOGE(T, "Undefine CSUx1 address");
            break;
        }
    }
    else{ ESP_LOGE(T, "TX power out of limit ! (TX power only available 0-31)"); }
}

void set_adc_div(uint32_t adc_div) {
    /*
    Parameters:
    - adc_div: ADC divider
    */
    if((20 <= adc_div) && (adc_div <= 1023)) {
        init_value(ADC0, adc_div << 14, 0xffc000);
    }
    else{ ESP_LOGE(T, "ADC DIV out of limit ! (ADC DIV only available 20-1023)"); }
}

void set_chirp(uint8_t ch, uint32_t coeff) {
    /*
    Use for set number of gain
    - use range 0-15
    Parameters:
    - ch: channel to be configuration
    - coeff: number of n, when 2^n
    */
    if(coeff <= 15) {
        switch (ch)
        {
        case 1:
            init_value(CSC1, coeff, 0x00000f);
            init_value(PLL17, coeff, 0x00000f);
            break;
        case 2:
            init_value(CSC2, coeff, 0x00000f);
            init_value(PLL27, coeff, 0x00000f);
            break;
        case 3:
            init_value(CSC3, coeff, 0x00000f);
            init_value(PLL37, coeff, 0x00000f);
            break;
        case 4:
            init_value(CSC4, coeff, 0x00000f);
            init_value(PLL47, coeff, 0x00000f);
            break;
        default:
            ESP_LOGE(T, "Undefine CSCx and PLLx7 address");
            break;
        }
    }
    else{ ESP_LOGE(T, "Chirp amount setting is not available"); }
}

void set_apu(uint8_t ch, uint32_t coeff) {
    /*
    Use for set number of Sample
    Parameters:
    - ch: channel to be configuration
    - coeff: number on Sample
    */
    if(coeff <= 4095) {
        switch (ch)
        {
        case 1:
            init_value(PLL13, coeff, 0x000fff);
            break;
        case 2:
            init_value(PLL23, coeff, 0x000fff);
            break;
        case 3:
            init_value(PLL33, coeff, 0x000fff);
            break;
        case 4:
            init_value(PLL43, coeff, 0x000fff);
            break;
        default:
            ESP_LOGE(T, "Undefine PLLx3 Address");
            break;
        }
    }
    else{ ESP_LOGE(T, "APU configuration is not available"); }
}

void set_fsu(uint8_t ch, uint32_t coeff) {
    /*
    Use for set starter frequency
    Parameters:
    - ch: channel to be configuration
    - coeff: start frequency point
    */

    switch (ch)
    {
    case 1:
        init_value(PLL10, coeff, 0xffffff);
        break;
    case 2:
        init_value(PLL20, coeff, 0xffffff);
        break;
    case 3:
        init_value(PLL30, coeff, 0xffffff);
        break;
    case 4:
        init_value(PLL40, coeff, 0xffffff);
        break;
    default:
        ESP_LOGE(T, "Undefine PLLx0 address");
        break;
    }
}

void set_rsu(uint8_t ch, uint32_t coeff) {
    /*
    Use for set number of Ramp step up
    Parameters:
    - ch: channel to be configuration
    - coeff: number on ramp step up
    */
    switch (ch)
    {
    case 1:
        init_value(PLL11, coeff, 0xffffff);
        break;
    case 2:
        init_value(PLL21, coeff, 0xffffff);
        break;
    case 3:
        init_value(PLL31, coeff, 0xffffff);
        break;
    case 4:
        init_value(PLL41, coeff, 0xffffff);
        break;
    default:
        ESP_LOGE(T, "Undefine PLLx1 address");
        break;
    }
}

void set_rtu(uint8_t ch, uint32_t coeff) {
    /*
    Use for set number of ramp time up
    Parameters:
    - ch: channel to be configuration
    - coeff: number on ramp time
    */
    if(coeff <= 16383) {
        switch (ch)
        {
        case 1:
            init_value(PLL12, coeff, 0x003fff);
            break;
        case 2:
            init_value(PLL22, coeff, 0x003fff);
            break;
        case 3:
            init_value(PLL32, coeff, 0x003fff);
            break;
        case 4:
            init_value(PLL42, coeff, 0x003fff);
            break;
        default:
            ESP_LOGE(T, "Undefine PLLx2 address");
            break;
        }
    }
    else{ ESP_LOGE(T, "RTU configuration is not available"); }
}

void set_spi_spd(bool mode) {
    /*
    Parameters:
    - mode: select speed (0: <25MHz, 1: >25MHz)
    */
    init_value(SFCTL, (uint32_t)mode << 16, 0x010000);
}

void set_cref(uint32_t cref) {
    /*
    Parameters:
    - cref: set FIFO interrupt threshold
    */
    if(cref <= 8191) {
        init_value(SFCTL, cref, 0x001fff);
    }
    else{ ESP_LOGE(T, "CREF out of limit ! (CREF only available 0-8191)"); }
}

void set_twkup(uint32_t coeff, uint32_t mul) {
    if((coeff <= 255) && (mul <= 15)) {
        init_value(MAIN, (coeff << 4 | mul << 12), 0x00FFF0);
    }
    else{ ESP_LOGE(T, "T_WU configuration is not available"); }
}

void set_tsed(uint32_t coeff, uint32_t mul) {
    if((coeff <= 255) && (mul <= 31)) {
        init_value(PLL17, (coeff << 11 | mul << 19), 0xFFF800);
    }
    else{ ESP_LOGE(T, "T_SED configuration is not available"); }
}

void set_tfed(uint32_t coeff, uint32_t mul) {
    if((coeff <= 255) && (mul <= 31)) {
        init_value(CCR1, (coeff << 11 | mul << 19), 0xFFF800);
    }
    else{ ESP_LOGE(T, "T_FED configuration is not available"); }    
}

void set_tend(uint32_t coeff) {
    if(coeff <= 511) {
        init_value(CCR0, coeff, 0x0001ff);
    }
    else{ ESP_LOGE(T, "T_END configuration is not available"); }
}

void set_tedu(uint32_t coeff) {
    if(coeff <= 255) {
        init_value(PLL12, coeff << 16, 0xff0000);
    }
    else{ ESP_LOGE(T, "T_EDU configuration is not available"); }
}

void set_tinit0(uint32_t coeff, uint32_t mul) {
    if((coeff <= 255) && (mul <= 3)) {
        init_value(CCR3, (coeff << 14 | mul << 22), 0xffc000);
    }
    else{ ESP_LOGE(T, "T_INIT0 configuration is not available"); }
}

void set_tinit1(uint32_t coeff, uint32_t mul) {
    if((coeff <= 255) && (mul <= 3)) {
        init_value(CCR0, (coeff << 14 | mul << 22), 0xffc000);
    }
    else{ ESP_LOGE(T, "T_INIT0 configuration is not available"); }
}

void set_tpaen(uint32_t coeff) {
    if(coeff <= 511) {
        init_value(CCR3, coeff, 0x0001ff);
    }
    else{ ESP_LOGE(T, "T_PAEN configuration is not available"); }
}

void set_tsstart(uint32_t coeff) {
    if(coeff <= 31) {
        init_value(CCR3, coeff << 9, 0x003e00);
    }
    else{ ESP_LOGE(T, "T_SSTART configuration is not available"); } 
}   

void set_tstart(uint32_t coeff) {
    if(coeff <= 511) {
        init_value(CCR1, coeff, 0x0001ff);
    }
    else{ ESP_LOGE(T, "T_START configuration is not available"); }
}


/* ==================== DEBUG FUNCTION ==================== */

void params_check() {
    const uint8_t RF_CLK = 80; //MHz     

    // ---- Timing ----
    uint16_t TR_WU = (read_reg(MAIN) & 0xFF0) >> 4;
    uint16_t TR_WU_M = (read_reg(MAIN) & 0xF000) >> 12;
    uint16_t T_WU = ((TR_WU * pow(2, TR_WU_M) * 8) + TR_WU_M + 3) / RF_CLK;
    ESP_LOGI(T, "TR_WU (%"PRIu16") - TR_WU_M (%"PRIu16") - T_WU (%"PRIu16")", TR_WU, TR_WU_M, T_WU);

    uint16_t TR_INIT0 = (read_reg(CCR3) & 0x3fc000) >> 14;
    uint16_t TR_INIT0_M = (read_reg(CCR3) & 0xc00000) >> 22;
    uint16_t T_INIT0 = ((TR_INIT0 * pow(2, TR_INIT0_M) * 8) + TR_INIT0_M + 3) / RF_CLK;
    ESP_LOGI(T, "TR_INIT0 (%"PRIu16") - TR_INIT0_M (%"PRIu16") - T_INIT0 (%"PRIu16")", TR_INIT0, TR_INIT0_M, T_INIT0);

    uint16_t TR_INIT1 = (read_reg(CCR0) & 0x3fc000) >> 14;
    uint16_t TR_INIT1_M = (read_reg(CCR0) & 0xc00000) >> 22;
    uint16_t T_INIT1 = ((TR_INIT1 * pow(2, TR_INIT1_M) * 8) + TR_INIT1_M + 3) / RF_CLK;
    ESP_LOGI(T, "TR_INIT1 (%"PRIu16") - TR_INIT1_M (%"PRIu16") - T_INIT1 (%"PRIu16")", TR_INIT1, TR_INIT1_M, T_INIT1);

    uint16_t TR_PAEN = (read_reg(CCR3) & 0x1ff);
    uint16_t T_PAEN = (TR_PAEN * 8) / RF_CLK;
    ESP_LOGI(T, "TR_PAEN (%"PRIu16") - T_PAEN (%"PRIu16")", TR_PAEN, T_PAEN);

    uint16_t TR_SSTART = (read_reg(CCR3) & 0x3e00) >> 9;
    uint16_t T_SSTART = ((TR_SSTART * 8) + 1) / RF_CLK;
    ESP_LOGI(T, "TR_SSTART (%"PRIu16") - T_SSTART (%"PRIu16")", TR_SSTART, T_SSTART);

    uint16_t TR_START = (read_reg(CCR1) & 0x1ff);
    uint16_t T_START = ((TR_START * 8) + 10) / RF_CLK;
    ESP_LOGI(T, "TR_START (%"PRIu16") - T_START (%"PRIu16")", TR_START, T_START);

    uint16_t TR_END = (read_reg(CCR0) & 0x1ff);
    uint16_t T_END = ((TR_END * 8) + 5) / RF_CLK;
    ESP_LOGI(T, "TR_END (%"PRIu16") - T_END (%"PRIu16")", TR_END, T_END);

    uint16_t TR_SED = (read_reg(PLL17) & 0x7f800) >> 11;
    uint16_t TR_SED_M = (read_reg(PLL17) & 0xf80000) >> 19;
    uint16_t T_SED = ((TR_SED * pow(2, TR_SED_M) * 8) + TR_SED_M + 3) / RF_CLK;
    ESP_LOGI(T, "TR_SED (%"PRIu16") - TR_SED_M (%"PRIu16") - T_SED (%"PRIu16")", TR_SED, TR_SED_M, T_SED);

    uint16_t TR_EDU = (read_reg(PLL12) & 0xff0000) >> 16;
    uint16_t T_EDU = (8 * TR_EDU + 5) / RF_CLK;
    ESP_LOGI(T, "TR_EDU (%"PRIu16") - T_EDU (%"PRIu16")", TR_EDU, T_EDU);

    uint16_t TR_EDD = (read_reg(PLL16) & 0xff0000) >> 16;
    uint16_t T_EDD = (8 * TR_EDD + 5) / RF_CLK;
    ESP_LOGI(T, "TR_EDD (%"PRIu16") - T_EDD (%"PRIu16")", TR_EDD, T_EDD);
    
    uint16_t T_RAMP = ((read_reg(PLL12) & 0x3fff) * 8) / RF_CLK;
    ESP_LOGI(T, "T_RAMP (%"PRIu16")", T_RAMP);

    uint16_t T_TOTAL = T_WU + T_INIT0 + T_INIT1 + T_PAEN + T_RAMP;
    ESP_LOGI(T, "T_TOTAL (%"PRIu16")", T_TOTAL);
    
    // ---- ADC ----
    uint16_t APU = (read_reg(PLL13) & 0xfff);
    uint16_t ADC_DIV = (read_reg(ADC0) & 0xffc000) >> 14;
    uint16_t T_ACQ = APU / (RF_CLK / ADC_DIV);
    ESP_LOGI(T, "T_ACQ (%"PRIu16")", T_ACQ);

    // ---- PLL ----
    int32_t FSU = (read_reg(PLL10) & 0xffffff) | 0xff000000;
    int32_t RSU = read_reg(PLL11) & 0xffffff;
    int32_t RTU = read_reg(PLL12) & 0x3fff;
    ESP_LOGI(T, "APU (%"PRIu16") - ADC_DIV (%"PRIu16")", APU, ADC_DIV);
    ESP_LOGI(T, "FSU (%"PRId32") - RSU (%"PRId32") - RTU (%"PRId32")", FSU, RSU, RTU);

}

bool read_error() {
    uint8_t gsr0 = (uint8_t)((read_reg(CHIP_ID) & 0x0f000000) >> 24);

    if((gsr0 & 0x08) == 0x08) {
        ESP_LOGE(T, "FIFO overflow/underflow ERROR");
        return false;
    }
    else if((gsr0 & 0x02) == 0x02) {
        ESP_LOGE(T, "SPI burst mode ERROR");
        return false;
    }
    else if((gsr0 & 0x01) == 0x01) {
        ESP_LOGE(T, "SPI clock speed ERROR");
        return false;
    }
    else {
        ESP_LOGI(T, "NO ERROR Header report");
        return true;
    }
}

void send_uart(uint8_t mode, uint16_t* rx1_buff, uint16_t* rx2_buff, uint16_t* rx3_buff, size_t rx_size) {
    const uint8_t start_mark[] = {0xAA, 0x55, 0xAA, 0x55};
    const uint8_t stop_mark[] = {0x55, 0xAA, 0x55, 0xAA};

    uart_write_bytes(uart_cfg.uart_port, start_mark, sizeof(start_mark));

    switch (mode)
    {
    case 1:
        for(size_t i = 0; i < rx_size; i++) {
            uint8_t buffer[2];
            buffer[0] = (rx1_buff[i] & 0xff);
            buffer[1] = (rx1_buff[i] >> 8) & 0xff; 
            uart_write_bytes(uart_cfg.uart_port, buffer, sizeof(buffer));
        }
        break;
    
    case 2:
        for(size_t i = 0; i < rx_size; i++) {
            uint8_t buffer[4];
            buffer[0] = (rx1_buff[i] & 0xff);
            buffer[1] = (rx1_buff[i] >> 8) & 0xff; 
            buffer[2] = (rx2_buff[i] & 0xff);
            buffer[3] = (rx3_buff[i] >> 8) & 0xff;
            uart_write_bytes(uart_cfg.uart_port, buffer, sizeof(buffer));
        }
        break;

    case 3:
        for(size_t i = 0; i < rx_size; i++) {
            uint8_t buffer[6];
            buffer[0] = (rx1_buff[i] & 0xff);
            buffer[1] = (rx1_buff[i] >> 8) & 0xff; 
            buffer[2] = (rx2_buff[i] & 0xff);
            buffer[3] = (rx2_buff[i] >> 8) & 0xff; 
            buffer[4] = (rx3_buff[i] & 0xff);
            buffer[5] = (rx3_buff[i] >> 8) & 0xff; 
            uart_write_bytes(uart_cfg.uart_port, buffer, sizeof(buffer));
        }
        break;

    default:
        ESP_LOGE(T, "Undefine Send UART Mode ...");
        break;
    }

    uart_write_bytes(uart_cfg.uart_port, stop_mark, sizeof(stop_mark));
}