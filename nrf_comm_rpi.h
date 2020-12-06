
/*
        V1.0
        Dylan Kane
        7/17/2020
*/

// Assumes SPI1 is used for all nrf communicatio
#ifndef NRF_COMM_H
#define NRF_COMM_H

#include <linux/spi/spidev.h>
#include <stdint.h>
#include <wiringPi.h>

#define SPEED_HZ 500000

#define MAX_PIPES 6

#define ENABLE 1
#define DISABLE 0
#define NRF_ERROR -1
#define NRF_SUCCESS 0

/* NRF Register Address */
#define CONFIG_REG_ADDR 0x00
#define STATUS_REG_ADDR 0x07
#define RX_P0_ADDR_REG_ADDR 0x0A // address of rx pipe 0 address register
#define TX_PIPE_ADDR_REG_ADDR 0x10 // address of tx pipe address register
#define EN_RX_PIPE_REG_ADDR 0x02 // address of register that controls enabling if rx pipes
#define RX_P0_PW_REG_ADDR 0x11 // address of register with payload width of rx pipe 0
#define SETUP_RETR_ADDR 0x04
#define FEATURE_REG_ADDR 0x1D
#define DYNPD_REG_ADDR 0x1C
#define FIFO_STATUS_REG_ADDR 0x17
#define RF_CH_REG_ADDR 0x05

/* NRF SPI Commands */
#define READ_RX_PAYLOAD_CMD 0x61
#define WRITE_TX_PAYLOAD_CMD 0xA0
#define FLUSH_TX_CMD 0xE1
#define FLUSH_RX_CMD 0xE2
#define R_RX_PL_WID_CMD 0x60

#define RX_DR_BIT 0x40
#define TX_DS_BIT 0x20
#define MAX_RT_BIT 0x10

#define SETBITS(mem, bits)  (mem) |= (bits)
#define CLEARBITS(mem, bits)    (mem) &= ~(bits)

typedef void (* gpio_set) (int);

typedef struct interface_t {
    gpio_set ce_set;
} interface_t;

extern uint8_t pipe_addr[MAX_PIPES][5];

uint8_t send_dumby_byte(struct spi_ioc_transfer *tr, int fd);
void NRF_pulse_ce(interface_t interface);

uint8_t NRF_read_status_reg(struct spi_ioc_transfer *tr, int fd);
uint8_t NRF_read_register(struct spi_ioc_transfer *tr, int fd, uint8_t address);
void NRF_write_register(struct spi_ioc_transfer *tr, int fd, uint8_t address, uint8_t data);
void NRF_read_payload(struct spi_ioc_transfer *tr, int fd, uint8_t *rx_payload, uint8_t pw);
void NRF_write_payload(struct spi_ioc_transfer *tr, int fd, uint8_t *tx_payload, uint8_t pw);
void NRF_read_long_addr(struct spi_ioc_transfer *tr, int fd, uint8_t reg_addr, uint8_t pipe_addr[5]);

void NRF_set_prim_rx(struct spi_ioc_transfer *tr, int fd);
void NRF_clear_prim_rx(struct spi_ioc_transfer *tr, int fd);
void NRF_clear_max_rt(struct spi_ioc_transfer *tr, int fd);
void NRF_clear_rx_dr(struct spi_ioc_transfer *tr, int fd);
void NRF_clear_tx_ds(struct spi_ioc_transfer *tr, int fd);

void NRF_flush_tx(struct spi_ioc_transfer *tr, int fd);
void NRF_flush_rx(struct spi_ioc_transfer *tr, int fd);

void NRF_reset(struct spi_ioc_transfer *tr, int fd, interface_t interface);

void NRF_set_tx_addr(struct spi_ioc_transfer *tr, int fd, uint8_t tx_addr[5]);
void NRF_set_rx_addr(struct spi_ioc_transfer *tr, int fd, uint8_t rx_addr[5], uint8_t pipe_num);

void NRF_config_dpl(struct spi_ioc_transfer *tr, int fd, uint8_t state);
uint8_t NRF_read_top_rx_pw(struct spi_ioc_transfer *tr, int fd);

void NRF_en_rx_pipes(struct spi_ioc_transfer *tr, int fd, uint8_t pipes_to_enable);
void NRF_config_pw(struct spi_ioc_transfer *tr, int fd, uint8_t pipe_num, uint8_t pw);
void NRF_config_auto_retr(struct spi_ioc_transfer *tr, int fd, uint16_t retr_delay_us, uint8_t retr_count);

void NRF_config_rx_dr_int(struct spi_ioc_transfer *tr, int fd, uint8_t state);
void NRF_config_tx_ds_int(struct spi_ioc_transfer *tr, int fd, uint8_t state);
void NRF_config_max_rt_int(struct spi_ioc_transfer *tr, int fd, uint8_t state);
void NRF_set_rf_ch(struct spi_ioc_transfer *tr, int fd, uint8_t rf_ch);

#endif

