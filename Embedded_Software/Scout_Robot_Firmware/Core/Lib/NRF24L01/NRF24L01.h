#ifndef NRF24L01_H
#define NRF24L01_H

#include "stm32l4xx_hal.h"

/* REGISTER MAP */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF


void NRF_CS_Select();
void NRF_CS_Unselect();
void NRF_CE_Enable();
void NRF_CE_Disable();
void NRF_WriteRegister(uint8_t reg, uint8_t data);
void NRF_WriteRegisterMultiple(uint8_t reg, uint8_t *data, uint16_t size);
uint8_t NRF_ReadRegister(uint8_t reg);
void NRF_ReadRegisterMultiple(uint8_t reg, uint8_t *data, uint16_t size);
void NRF_SendCommand(uint8_t cmd);
void NRF_Init(SPI_HandleTypeDef* spi, GPIO_TypeDef* irqport, uint32_t irqpin, GPIO_TypeDef* csport, uint32_t cspin, GPIO_TypeDef* ceport, uint32_t cepin );
void NRF_SetTxMode(uint8_t *RXPipeAddr, uint8_t channel, uint8_t irqEnable);
uint8_t NRF_Transmit(uint8_t* data);
void NRF_SetRxMode(uint8_t *TXPipeAddr, uint8_t channel, uint8_t irqEnable);
uint8_t NRF_dataAvailable(uint8_t pipeNum);
void NRF_Receive(uint8_t* data);
void NRF_ResetReg(uint8_t REG);


#endif
