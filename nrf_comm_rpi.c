/*
        V1.0
        Dylan Kane
        7/21/2020
*/

#include <stdlib.h>
#include <stdio.h>
#include "nrf_comm_rpi.h"
//#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <sys/ioctl.h>
//#include <wiringPi.h>

uint8_t pipe_addr[MAX_PIPES][5] = {
    {0xE7,0xE7,0xE7,0xE7,0xE7},
    {0xC2,0xC2,0xC2,0xC2,0xC2},
    {0xC3,0xC2,0xC2,0xC2,0xC2},
    {0xC4,0xC2,0xC2,0xC2,0xC2},
    {0xC5,0xC2,0xC2,0xC2,0xC2},
    {0xC6,0xC2,0xC2,0xC2,0xC2}
};

uint8_t send_dumby_byte(struct spi_ioc_transfer *tr, int fd) {
	uint8_t rx[1] = {0x00};
	uint8_t tx[1] = {0xff};

	tr->rx_buf = (unsigned long)rx;
	tr->tx_buf = (unsigned long)tx;
	tr->len = 1;
	int ret = ioctl(fd, SPI_IOC_MESSAGE(1), tr);
	if (ret < 1)
		printf("can't send spi message\n");

	return rx[0];
}

void NRF_pulse_ce(interface_t interface) {
	interface.ce_set(1);
	delayMicroseconds(200);
	interface.ce_set(0);
	return;
}

uint8_t NRF_read_status_reg(struct spi_ioc_transfer *tr, int fd) {
	uint8_t status_reg_data;
	
	status_reg_data = send_dumby_byte(tr, fd);
	return status_reg_data;		
}

uint8_t NRF_read_register(struct spi_ioc_transfer *tr, int fd, uint8_t address) {
	CLEARBITS(address, 0xE0);
	uint8_t rx[2] = {0x00, 0x00};
	uint8_t tx[2] = {0x00, 0xff};
	tx[0] = address;	

	tr->tx_buf = (unsigned long)tx;
	tr->rx_buf = (unsigned long)rx;
	tr->len = 2;
	ioctl(fd, SPI_IOC_MESSAGE(1), tr);	
	return rx[1];
}

void NRF_write_register(struct spi_ioc_transfer *tr, int fd, uint8_t address, uint8_t data) {
	CLEARBITS(address, 0xC0);
	SETBITS(address, 0x20);
	uint8_t rx[2] = {0x00, 0x00};
	uint8_t tx_msg[2] = {0x00, 0x00};
	tx_msg[0] = address;
	tx_msg[1] = data;

	tr->tx_buf = (unsigned long)tx_msg;
	tr->rx_buf = (unsigned long)rx;
	tr->len = 2;
	ioctl(fd, SPI_IOC_MESSAGE(1), tr);

}

void NRF_read_payload(struct spi_ioc_transfer *tr, int fd, uint8_t *rx_payload, uint8_t pw) {
	//uint8_t *tx_payload = (uint8_t *) malloc(pw + 1);
	//uint8_t *rx_temp_pl = (uint8_t *) malloc(pw + 1);
	uint8_t tx_payload[32] = {0};
	uint8_t rx_temp_pl[32] = {0};
	int i;	

	tx_payload[0] = READ_RX_PAYLOAD_CMD;
	for (i = 0; i < pw; i++) {
		tx_payload[i+1] = 0xff;
	}
	tr->tx_buf = (unsigned long)tx_payload;
	tr->rx_buf = (unsigned long)rx_temp_pl;
	tr->len = pw + 1;
	ioctl(fd, SPI_IOC_MESSAGE(1), tr);		
	for (i = 0; i < pw; i++) {
		rx_payload[i] = rx_temp_pl[i + 1];
	}
	//free(tx_payload);
	//free(rx_temp_pl);
}

void NRF_write_payload(struct spi_ioc_transfer *tr, int fd, uint8_t *tx_payload, uint8_t pw) {
	//printf("@\n");
	//uint8_t *tx_full_msg = (uint8_t *) malloc(pw + 1);
	//uint8_t *rx_payload = (uint8_t *) malloc(pw + 1);
	uint8_t tx_full_msg[32] = {0};
	uint8_t rx_payload[32] = {0};
	int i;

	tx_full_msg[0] = WRITE_TX_PAYLOAD_CMD;
	for (i = 0; i < pw; i++) {
		tx_full_msg[i + 1] = tx_payload[i];
	}
	tr->tx_buf = (unsigned long)tx_full_msg;
	tr->rx_buf = (unsigned long)rx_payload;
	tr->len = pw + 1;
	ioctl(fd, SPI_IOC_MESSAGE(1), tr);

	//free(tx_full_msg);
	//free(rx_payload);	
}

void NRF_read_long_addr(struct spi_ioc_transfer *tr, int fd, uint8_t reg_addr, uint8_t pipe_addr[5]) {
	uint8_t rx[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t tx[6] = {0x00, 0xff, 0xff, 0xff, 0xff, 0xff};
	int i;

	tx[0] = reg_addr;
	tr->tx_buf = (unsigned long)tx;
	tr->rx_buf = (unsigned long)rx;
	tr->len = 6;
	ioctl(fd, SPI_IOC_MESSAGE(1), tr);
	/*debug
	printf("full rx:");
	for (i=0;i<6;i++){
		printf(" %.2X,",rx[i]);
	}
	printf("\n");*/
	for (i = 0; i < 5; i++) {
		pipe_addr[i] = rx[i + 1];
	}	
}

void NRF_set_prim_rx(struct spi_ioc_transfer *tr, int fd) {
	uint8_t config_reg_data;

	config_reg_data = NRF_read_register(tr, fd, CONFIG_REG_ADDR);
	SETBITS(config_reg_data, 0x01);
	NRF_write_register(tr, fd, CONFIG_REG_ADDR, config_reg_data);
}

void NRF_clear_prim_rx(struct spi_ioc_transfer *tr, int fd) {
        uint8_t config_reg_data;

        config_reg_data = NRF_read_register(tr, fd, CONFIG_REG_ADDR);
        CLEARBITS(config_reg_data, 0x01);
        NRF_write_register(tr, fd, CONFIG_REG_ADDR, config_reg_data);

}

void NRF_clear_max_rt(struct spi_ioc_transfer *tr, int fd) {
	uint8_t status_byte;

	status_byte = NRF_read_status_reg(tr, fd);
	SETBITS(status_byte, 0x10);
	NRF_write_register(tr, fd, STATUS_REG_ADDR, status_byte);	
}

void NRF_flush_tx(struct spi_ioc_transfer *tr, int fd) {
	uint8_t tx[1] = {FLUSH_TX_CMD};

	tr->tx_buf = (unsigned long)tx;
	tr->len = 1;
	ioctl(fd, SPI_IOC_MESSAGE(1), tr); 	
}

void NRF_flush_rx(struct spi_ioc_transfer *tr, int fd) {
	uint8_t tx[1] = {FLUSH_RX_CMD};

	tr->tx_buf = (unsigned long)tx;
	tr->len = 1;
	ioctl(fd, SPI_IOC_MESSAGE(1), tr); 	
}

void NRF_clear_rx_dr(struct spi_ioc_transfer *tr, int fd) {
	uint8_t status_byte;

	status_byte = NRF_read_status_reg(tr, fd);
	SETBITS(status_byte, 0x40); // write 1 to clear rx data ready bit	
	NRF_write_register(tr, fd, STATUS_REG_ADDR, status_byte);
}

void NRF_clear_tx_ds(struct spi_ioc_transfer *tr, int fd) {
        uint8_t status_byte;

        status_byte = NRF_read_status_reg(tr, fd);
        SETBITS(status_byte, 0x20); // write 1 to clear tx data sent bit
        NRF_write_register(tr, fd, STATUS_REG_ADDR, status_byte);
}

void NRF_reset(struct spi_ioc_transfer *tr, int fd, interface_t interface) {
	uint8_t status_reg_data, config_reg_data;
	
	NRF_clear_prim_rx(tr, fd);
	interface.ce_set(0);
	config_reg_data = NRF_read_register(tr, fd, CONFIG_REG_ADDR);
	CLEARBITS(config_reg_data, 0x02); // clear power up bit
	NRF_write_register(tr, fd, CONFIG_REG_ADDR, config_reg_data);

	delay(100);

	config_reg_data = NRF_read_register(tr, fd, CONFIG_REG_ADDR);
	SETBITS(config_reg_data, 0x02); // set power up bit
	SETBITS(config_reg_data, 0x70); // disable all interrupt sources
	NRF_write_register(tr, fd, CONFIG_REG_ADDR, config_reg_data);

	status_reg_data = NRF_read_status_reg(tr, fd);
	if (status_reg_data & 0x10)
		NRF_write_register(tr, fd, STATUS_REG_ADDR, status_reg_data); // clear max rt bit	

	NRF_write_register(tr, fd, FEATURE_REG_ADDR, 0x00); // disable features

	NRF_clear_rx_dr(tr, fd);
	NRF_clear_tx_ds(tr, fd);

	NRF_flush_tx(tr, fd);
	NRF_flush_rx(tr, fd);
}

void NRF_set_tx_addr(struct spi_ioc_transfer *tr, int fd, uint8_t tx_addr[5]) {
	int i;
	uint8_t tx[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t rx[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	tx[0] = TX_PIPE_ADDR_REG_ADDR;

	SETBITS(tx[0], 0x20); // set msb 3 for write operation
	for (i = 0; i < 5; i++) {
		tx[i + 1] = tx_addr[i];
	}
	tr->tx_buf = (unsigned long)tx;
	tr->rx_buf = (unsigned long)rx;
	tr->len = 6;
	ioctl(fd, SPI_IOC_MESSAGE(1), tr);
}

void NRF_set_rx_addr(struct spi_ioc_transfer *tr, int fd, uint8_t rx_addr[5], uint8_t pipe_num) {
	int i;
	uint8_t tx[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t rx[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};	

	tx[0] = RX_P0_ADDR_REG_ADDR; 
	SETBITS(tx[0], 0x20); // set bit for write operation
	tx[0] += pipe_num; // other pipe addresses are offset by pipe#
	if (pipe_num < 2) {
		for (i = 0; i < 5; i++) {
			tx[i + 1] = rx_addr[i];	
		}
		tr->tx_buf = (unsigned long)tx;
		tr->rx_buf = (unsigned long)rx;	
		tr->len = 6;
		ioctl(fd, SPI_IOC_MESSAGE(1), tr);
	} else {
		tx[1] = rx_addr[0];		
		tr->tx_buf = (unsigned long)tx;
		tr->rx_buf = (unsigned long)rx;	
		tr->len = 2;
		ioctl(fd, SPI_IOC_MESSAGE(1), tr);

	}
}

void NRF_config_dpl(struct spi_ioc_transfer *tr, int fd, uint8_t state) {
	uint8_t feature_reg_data = NRF_read_register(tr, fd, FEATURE_REG_ADDR);
	if (state == ENABLE) {
		SETBITS(feature_reg_data, 0x04);
		NRF_write_register(tr, fd, FEATURE_REG_ADDR, feature_reg_data);
		NRF_write_register(tr, fd, DYNPD_REG_ADDR, 0x3F); // enable dynamic payload length for6 pipes
	} else if (state == DISABLE) {
		CLEARBITS(feature_reg_data, 0x04);
		NRF_write_register(tr, fd, FEATURE_REG_ADDR, feature_reg_data);
		NRF_write_register(tr, fd, DYNPD_REG_ADDR, 0x00); // enable dynamic payload length for6 pipes	
	}	
}

uint8_t NRF_read_top_rx_pw(struct spi_ioc_transfer *tr, int fd) {
	uint8_t rx[2] = {0x00, 0x00};
	uint8_t tx[2] = {0x00, 0xff};

	tx[0] = R_RX_PL_WID_CMD;
	tr->tx_buf = (unsigned long)tx;
	tr->rx_buf = (unsigned long)rx;
	tr->len = 2;
	ioctl(fd, SPI_IOC_MESSAGE(1), tr);
	return rx[1];
}

void NRF_en_rx_pipes(struct spi_ioc_transfer *tr, int fd, uint8_t pipes_to_enable) {
	NRF_write_register(tr, fd, EN_RX_PIPE_REG_ADDR, pipes_to_enable);	
}

void NRF_config_pw(struct spi_ioc_transfer *tr, int fd, uint8_t pipe_num, uint8_t pw) {
	NRF_write_register(tr, fd, RX_P0_PW_REG_ADDR+pipe_num, pw);
}

void NRF_config_auto_retr(struct spi_ioc_transfer *tr, int fd, uint16_t retr_delay_us, uint8_t retr_count) {
	uint8_t retr_delay_4bits, retr_config_byte;
    	
	retr_delay_4bits = retr_delay_us / 250; 
	if (retr_delay_4bits > 15) {
		retr_delay_4bits = 15;
	}
	retr_delay_4bits <<= 4;
    
	if (retr_count > 15) {
		retr_count = 15;
	}
	retr_config_byte = retr_delay_4bits + retr_count;
	NRF_write_register(tr, fd, SETUP_RETR_ADDR, retr_config_byte);
}

void NRF_config_rx_dr_int(struct spi_ioc_transfer *tr, int fd, uint8_t state) {
    uint8_t config_reg_data;
    
    config_reg_data = NRF_read_register(tr, fd, CONFIG_REG_ADDR);
    if (state == ENABLE) {
        CLEARBITS(config_reg_data, 0x40); // Reflect RX_DR active low interrupt on the IRQ pin
    } else if (state == DISABLE) {
        SETBITS(config_reg_data, 0x40);
    }
    NRF_write_register(tr, fd, CONFIG_REG_ADDR, config_reg_data);
    return;
}

void NRF_config_tx_ds_int(struct spi_ioc_transfer *tr, int fd, uint8_t state) {
    uint8_t config_reg_data;

    config_reg_data = NRF_read_register(tr, fd, CONFIG_REG_ADDR);
    if (state == ENABLE) {
        CLEARBITS(config_reg_data, 0x20); // Reflect TX_DS active low interrupt on the IRQ pin
    } else if (state == DISABLE) {
        SETBITS(config_reg_data, 0x20);
    }
    NRF_write_register(tr, fd, CONFIG_REG_ADDR, config_reg_data);
    return;
}

void NRF_config_max_rt_int(struct spi_ioc_transfer *tr, int fd, uint8_t state) {
    uint8_t config_reg_data;

    config_reg_data = NRF_read_register(tr, fd, CONFIG_REG_ADDR);
    if (state == ENABLE) {
        CLEARBITS(config_reg_data, 0x10); // Reflect MAX_RT active low interrupt on the IRQ pin
    } else if (state == DISABLE) {
        SETBITS(config_reg_data, 0x10);
    }
    NRF_write_register(tr, fd, CONFIG_REG_ADDR, config_reg_data);
    return;
}

void NRF_set_rf_ch(struct spi_ioc_transfer *tr, int fd, uint8_t rf_ch) {
	if (rf_ch > 127) {
		rf_ch = 127;
	}
	NRF_write_register(tr, fd, RF_CH_REG_ADDR, rf_ch);
}







