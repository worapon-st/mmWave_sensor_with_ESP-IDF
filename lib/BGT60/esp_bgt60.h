/*
    @ ESP_BGT60 is the library that use for communicate with BGT60TR13C (Infineon)
        - ESP-IDF 
        - SPI

    ! This code still running on "Prototype"
        - It might have error or mistake, So you should recheck together with Datasheet.

    > Driver setting Function
        - init_gpio     : initialize GPIO
        - init_spi      : initialize SPI
        - init_uart     : initialize UART

    > Main Work Function
        - read_reg      : read register from slave
        - write_reg     : write register to slave
        - read_fifo     : burst read register from slave
        - extract_fifo  : extract fifo data to each RX buffer
        - toggle_flag   : toggle register flag
        - init_sensor   : initialize the starter configuration
        - init_value    : initialize new configuration 
        - hard_reset    : hardware reset
        - soft_reset    : software reset
        - start_frame   : generate frame to acquire

    > Configuration Function
        # RF
        - rx_enable     : enable/disable RX Baseband
        - mixer_enable  : enable/disable Mixer 
        - lo_enable     : enable/disable Local oscillator buffer
        - lod_enable    : enable/disable Local oscillator distribution buffer
        - set_gain_hpf  : set gain of first stage
        - set_gain_vga  : set gain of VGA
        - set_hpf       : set cutoff frequency
        - set_tx_power  : set TX sending power 
        
        # ADC 
        - set_adc_div   : set ADC divider
        
        # PLL
        - set_chirp     : set amount of chirp
        - set_apu       : set sample number
        - set_fsu       : set starter frequency
        - set_rsu       : set ramp step
        - set_rtu       : set ramp time
        
        # SPI
        - set_spi_spd   : change SPI clock mode
        - set_cref      : set threshold of interrupt sender

        # Timing
        - set_twkup     : set WKUP time
        - set_tsed      : set SED time
        - set_tfed      : set FED time
        - set_tend      : set END time
        - set_tedu      : set EDU time
        - set_tinit0    : set INIT0 time
        - set_tinit1    : set INIT1 time
        - set_tpaen     : set PAEN time
        - set_tsstart   : set SSTART time
        - set_tstart    : set START time

    > Debugging Function
        - params_check  : timing and PLL parameter checking
        - time_cond     : timing condition for start frame generate
        - read_error    : read the GSR0 for status
        - send_uart     : send data to PC for visualize
*/

#ifndef ESP_BGT60_H
#define ESP_BGT60_H

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"

/* =================== REGISTER ADDRESS ================== */

const typedef enum {
    MAIN        = 0x00,
    ADC0        = 0x01,
    CHIP_ID     = 0x02,
    STAT1       = 0x03,
    PARC1       = 0x04,
    PARC2       = 0x05,
    SFCTL       = 0x06,
    SADC        = 0x07,
    CSI0        = 0x08,
    CSI1        = 0x09,
    CSI2        = 0x0A,
    CSIC        = 0x0B,
    CSDS0       = 0x0C,
    CSDS1       = 0x0D,
    CSDS2       = 0x0E,
    CSDSC       = 0x0F,
    CSU10       = 0x10,
    CSU11       = 0x11,
    CSU12       = 0x12,
    CSD10       = 0x13,
    CSD11       = 0x14,
    CSD12       = 0x15,
    CSC1        = 0x16,
    CSU20       = 0x17,
    CSU21       = 0x18,
    CSU22       = 0x19,
    CSD20       = 0x1A,
    CSD21       = 0x1B,
    CSD22       = 0x1C,
    CSC2        = 0x1D,
    CSU30       = 0x1E,
    CSU31       = 0x1F,
    CSU32       = 0x20,
    CSD30       = 0x21,
    CSD31       = 0x22,
    CSD32       = 0x23,
    CSC3        = 0x24,
    CSU40       = 0x25,
    CSU41       = 0x26,
    CSU42       = 0x27,
    CSD40       = 0x28,
    CSD41       = 0x29,
    CSD42       = 0x2A,
    CSC4        = 0x2B,
    CCR0        = 0x2C,
    CCR1        = 0x2D,
    CCR2        = 0x2E,
    CCR3        = 0x2F,
    PLL10       = 0x30,
    PLL11       = 0x31,
    PLL12       = 0x32,
    PLL13       = 0x33,
    PLL14       = 0x34,
    PLL15       = 0x35,
    PLL16       = 0x36,
    PLL17       = 0x37,
    PLL20       = 0x38,
    PLL21       = 0x39,
    PLL22       = 0x3A,
    PLL23       = 0x3B,
    PLL24       = 0x3C,
    PLL25       = 0x3D,
    PLL26       = 0x3E,
    PLL27       = 0x3F,
    PLL30       = 0x40,
    PLL31       = 0x41,
    PLL32       = 0x42,
    PLL33       = 0x43,
    PLL34       = 0x44,
    PLL35       = 0x45,
    PLL36       = 0x46,
    PLL37       = 0x47,
    PLL40       = 0x48,
    PLL41       = 0x49,
    PLL42       = 0x4A,
    PLL43       = 0x4B,
    PLL44       = 0x4C,
    PLL45       = 0x4D,
    PLL46       = 0x4E,
    PLL47       = 0x4F,
    RFT0        = 0x55,
    RFT1        = 0x56,
    RLL_DFT0    = 0x59,
    STAT0       = 0x5D,
    SADC_RESULT = 0x5E,
    FSTAT       = 0x5F,
    FIFO        = 0x60
} reg_address;

/* ==================== REGISTER FLAGS =================== */

typedef enum {
    START       = 1 << 0,
    SW_RST      = 1 << 1,
    FSM_RST     = 1 << 2,
    FIFO_RST    = 1 << 3,
} flags;

/* =============== DRIVER SETTING FUNCTION =============== */

typedef struct {
    gpio_num_t irq;
    gpio_num_t rst;
    gpio_num_t csn;
} gpio_esp_t;

typedef struct {
    uint32_t spi_clk_speed;
    spi_host_device_t spi_host;
    gpio_num_t miso;
    gpio_num_t mosi;
    gpio_num_t sclk;
    gpio_num_t csn;
    uint32_t max_byte;
} spi_esp_t;

typedef struct {
    uint32_t baudrate;
    int uart_port;
    gpio_num_t tx;
    gpio_num_t rx;
} uart_esp_t; 

esp_err_t init_gpio(const gpio_esp_t* config);
esp_err_t init_spi(const spi_esp_t* config);
esp_err_t init_uart(const uart_esp_t* config);

/* ==================== MAIN FUNCTION ==================== */

uint32_t read_reg(uint8_t address);    
void write_reg(uint8_t address, uint32_t data);   

void read_fifo(uint8_t* buffer);
void extract_fifo(uint8_t* fifo_buff, size_t fifo_size, uint16_t* extract_buff, uint8_t mode, uint16_t* rx1_buff, uint16_t* rx2_buff, uint16_t* rx3_buff);

void init_sensor();
void toggle_flag(uint8_t address, uint32_t flag);
void init_value(uint8_t address, uint32_t data, uint32_t masked);

void hard_reset();
void soft_reset();

/* ================ CONFIGURATION FUNCTION ================ */

// RF configuration section
void rx_enable(uint8_t ch, bool ch1, bool ch2, bool ch3);   
void mixer_enable(uint8_t ch, bool ch1, bool ch2, bool ch3);
void lo_enable(uint8_t ch, bool ch1, bool ch2, bool ch3);
void lod_enable(uint8_t ch, bool lod1, bool lod2);
void set_gain_hpf(uint8_t ch, bool vga1, bool vga2, bool vga3);
void set_gain_vga(uint8_t ch, uint32_t vga1, uint32_t vga2, uint32_t vga3);
void set_hpf(uint8_t ch, uint32_t hpf1, uint32_t hpf2, uint32_t hpf3);
void set_tx_power(uint8_t ch, uint32_t tx_pwr);

// ADC configuration section
void set_adc_div(uint32_t adc_div);

// PLL configuration section
void set_chirp(uint8_t ch, uint32_t coeff);
void set_apu(uint8_t ch, uint32_t coeff);  
void set_fsu(uint8_t ch, uint32_t coeff);  
void set_rsu(uint8_t ch, uint32_t coeff);  
void set_rtu(uint8_t ch, uint32_t coeff);  

// SPI configuration section
void set_spi_spd(bool mode);
void set_cref(uint32_t cref);   
 
// TIMING configuration section
void set_twkup(uint32_t coeff, uint32_t mul);
void set_tsed(uint32_t coeff, uint32_t mul);
void set_tfed(uint32_t coeff, uint32_t mul);
void set_tend(uint32_t coeff);
void set_tedu(uint32_t coeff);
void set_tinit0(uint32_t coeff, uint32_t mul); 
void set_tinit1(uint32_t coeff, uint32_t mul); 
void set_tpaen(uint32_t coeff);
void set_tsstart(uint32_t coeff);
void set_tstart(uint32_t coeff);

/* ==================== DEBUG FUNCTION ==================== */

void params_check();
bool read_error();
void send_uart(uint8_t mode, uint16_t* rx1_buff, uint16_t* rx2_buff, uint16_t* rx3_buff, size_t rx_size);  

#endif


