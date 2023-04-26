
#include "NRF24L01.h"



SPI_HandleTypeDef* nrf_spi;

GPIO_TypeDef* nrfCsPort;
uint32_t nrfCsPin;

GPIO_TypeDef* nrfCePort;
uint32_t nrfCePin;

GPIO_TypeDef* nrfIrqPort;
uint32_t nrfIrqPin;

uint8_t RX_DR_flag;
uint8_t TX_DS_flag;




void NRF_CS_Select(){
	HAL_GPIO_WritePin(nrfCsPort, nrfCsPin, GPIO_PIN_RESET);
}

void NRF_CS_Unselect(){
	HAL_GPIO_WritePin(nrfCsPort, nrfCsPin, GPIO_PIN_SET);
}

void NRF_CE_Enable(){
	HAL_GPIO_WritePin(nrfCePort, nrfCePin, GPIO_PIN_SET);
}

void NRF_CE_Disable(){
	HAL_GPIO_WritePin(nrfCePort, nrfCePin, GPIO_PIN_RESET);
}

/* write a single byte to the specified register */
void NRF_WriteRegister(uint8_t reg, uint8_t data){
	uint8_t buffer[2];
	/* according to datasheet - we must write 1 to a 5th bit writing data to a register*/
	buffer[0] = reg | (1<<5);
	buffer[1] = data;
	//select device
	NRF_CS_Select();

	//send data
	HAL_SPI_Transmit(nrf_spi, buffer, 2, 100);

	//unselect NRF device
	NRF_CS_Unselect();
}

/* write multiple bytes of data starting from a specified register */
void NRF_WriteRegisterMultiple(uint8_t reg, uint8_t *data, uint16_t size){
	uint8_t buffer[2];
	/* according to datasheet - we must write 1 to a 5th bit writing data to a register*/
	buffer[0] = reg | (1<<5);
	//buffer[1] = data;
	//select device
	NRF_CS_Select();

	//send register address
	HAL_SPI_Transmit(nrf_spi, buffer, 1, 100);
	//send the data all at once
	HAL_SPI_Transmit(nrf_spi, data, size, 100);

	//unselect NRF device
	NRF_CS_Unselect();
}

/* read a byte from a specified register */
uint8_t NRF_ReadRegister(uint8_t reg){
	uint8_t data;
	//select device
	NRF_CS_Select();

	//send register address to read from
	HAL_SPI_Transmit(nrf_spi, &reg, 1, 100);

	//receive data
	HAL_SPI_Receive(nrf_spi, &data, 1, 100);

	//unselect NRF device
	NRF_CS_Unselect();
	return data;
}


/* read multiple bytes starting from a specified register */
void NRF_ReadRegisterMultiple(uint8_t reg, uint8_t *data, uint16_t size){
	//select device
	NRF_CS_Select();

	//send register address to read from
	HAL_SPI_Transmit(nrf_spi, &reg, 1, 100);

	//receive data
	HAL_SPI_Receive(nrf_spi, data, size, 100);

	//unselect NRF device
	NRF_CS_Unselect();
}


/* send special command to the NRF module */
void NRF_SendCommand(uint8_t cmd){
	//select device
	NRF_CS_Select();

	//send data
	HAL_SPI_Transmit(nrf_spi, &cmd, 1, 100);

	//unselect NRF device
	NRF_CS_Unselect();
}

void NRF_Init(SPI_HandleTypeDef* spi, GPIO_TypeDef* irqport, uint32_t irqpin, GPIO_TypeDef* csport, uint32_t cspin, GPIO_TypeDef* ceport, uint32_t cepin ){

	/* substitute gpios */
	nrfCsPort = csport;
	nrfCsPin = cspin;
	nrfCePort = ceport;
	nrfCePin = cepin;
	nrfIrqPort = irqport;
	nrfIrqPin = irqpin;

	//substitute used SPI handler
	nrf_spi = spi;

	//disable device for configuration
	NRF_CE_Disable();

	/* reset all the registers*/

	NRF_ResetReg(0);

	/* configure interrupt masking, power mode and working mode to transmit */
	uint8_t config = 0;
	config |= (1<<6); //disable rx data ready interrupt request
	config |= (1<<5); // disable tx data sent interrupt request
	config |= (1<<4); //disable max retransmision interrupt request
	NRF_WriteRegister(CONFIG, config);

	/* disable autoacknowledgement */
	NRF_WriteRegister(EN_AA, 0);

	/* enable pipe x - tbi */
	NRF_WriteRegister(EN_RXADDR, 0);

	/* configure 5 byte TX/RX address*/
	NRF_WriteRegister(SETUP_AW, 0x03);

	/* configure retransmission - disable */
	NRF_WriteRegister(SETUP_RETR, 0);

	/* select transmission channel */
	NRF_WriteRegister(RF_CH, 0);

	/* RF settings */
	NRF_WriteRegister(RF_SETUP, 0x0E);

	/* enable back the device */
	NRF_CE_Enable();

}

/* set up transmit mode */
void NRF_SetTxMode(uint8_t *RXPipeAddr, uint8_t channel, uint8_t irqEnable){

	TX_DS_flag = irqEnable;

	/* disable device for configuration */
	NRF_CE_Disable();

	/* Select the channel */
	NRF_WriteRegister(RF_CH, channel);

	/* set up the TX address */
	NRF_WriteRegisterMultiple(TX_ADDR, RXPipeAddr, 5);

	uint8_t config = NRF_ReadRegister(CONFIG);

	/* enable tx data sent interrupt */
	if(irqEnable)
		config &= !(1<<5);


	/* power up the device */
	config = config | (1<<1);
	config = config & (0xF2);



	NRF_WriteRegister(CONFIG, config);
	config = NRF_ReadRegister(CONFIG);
	/* enable the device back */
	NRF_CE_Enable();
}

/* transmit data over SPI to the NRF module */

uint8_t NRF_Transmit(uint8_t* data){
	uint8_t command;
	command = W_TX_PAYLOAD;

	/* select device for transmission*/
	NRF_CS_Select();


	/* send transmit payload command */
	HAL_SPI_Transmit(nrf_spi, &command, 1, 100);

	/* transmit 32 bit long data stream */
	HAL_SPI_Transmit(nrf_spi, data, 32, 100);

	/* unselect device after transmission */
	NRF_CS_Unselect();

	/* wait for the pin to settle */
	HAL_Delay(1);

	/* check for data in fifo - transmission successful */
	uint8_t fifoStatus = NRF_ReadRegister(FIFO_STATUS);
	if((fifoStatus & (1<<4)) && (!(fifoStatus & (1<<3)))){
		/* if the data was transfered to the fifo flush it for sending */
		command = FLUSH_TX;
		NRF_SendCommand(command);

		/*reset fifo status after flushing */
		NRF_ResetReg(FIFO_STATUS);
		return 1;
	}
		return 0;
}


/* set up receive mode */
void NRF_SetRxMode(uint8_t *TXPipeAddr, uint8_t channel, uint8_t irqEnable){

	RX_DR_flag = irqEnable;

	/* disable device for configuration */
	NRF_CE_Disable();

	/*reset registers */
	NRF_ResetReg(STATUS);

	/* Select the channel */
	NRF_WriteRegister(RF_CH, channel);

	/* enable pipe 1 without disabling other pipes */
	uint8_t en_rxAddr = NRF_ReadRegister(EN_RXADDR);
	en_rxAddr |= (1<<1);
	NRF_WriteRegister(EN_RXADDR, en_rxAddr);

	/* set up the RX pipe address */
	NRF_WriteRegisterMultiple(RX_ADDR_P1, TXPipeAddr, 5);


	/* set up the payload length to 32 bytes */
	NRF_WriteRegister(RX_PW_P1, 32);

	/* power up the device and set it into Rx mode */
	uint8_t config = NRF_ReadRegister(CONFIG);

	/* enable rx data ready interrupt */
	if(irqEnable){
		config &=!(1<<6);
	}
	config = config | ((1<<1) |(1<<0));

	NRF_WriteRegister(CONFIG, config);

	/* enable the device back */
	NRF_CE_Enable();
}


/* check if data is in the fifo */
uint8_t NRF_dataAvailable(uint8_t pipeNum){
	uint8_t statusReg = NRF_ReadRegister(STATUS);

	/* check for data ready rx fifo flag */
	/* also check pipe number in the payload available register */
	if((statusReg&(1<<6))&&(statusReg&(pipeNum<<1))){
		/* clear the data ready flag */
		NRF_WriteRegister(STATUS, (1<<6));
		return 1;
	}
	return 0;
}

/* receive data from fifo */

void NRF_Receive(uint8_t* data){
	uint8_t command;
	command = R_RX_PAYLOAD;

	/* select device */
	NRF_CS_Select();

	/* send receive payload command */
	HAL_SPI_Transmit(nrf_spi, &command, 1, 100);

	/* receive 32 bit long data stream */
	HAL_SPI_Receive(nrf_spi, data, 32, 100);

	/* unselect device after transmission */
	NRF_CS_Unselect();

	/* wait for the pin to settle */
	HAL_Delay(1);

	/* flush the rx fifo after reading */
	command = FLUSH_RX;
	NRF_SendCommand(FLUSH_RX);

	if(RX_DR_flag)
		NRF_WriteRegister(STATUS, (1<<6)); //clear interrupt

}


/* reset register settings */
void NRF_ResetReg(uint8_t REG){
	if (REG == STATUS)
	{
		NRF_WriteRegister(STATUS, 0x00);
	}

	else if (REG == FIFO_STATUS)
	{
		NRF_WriteRegister(FIFO_STATUS, 0x11);
	}

	else {
		NRF_WriteRegister(CONFIG, 0x08);
		NRF_WriteRegister(EN_AA, 0x3F);
		NRF_WriteRegister(EN_RXADDR, 0x03);
		NRF_WriteRegister(SETUP_AW, 0x03);
		NRF_WriteRegister(SETUP_RETR, 0x03);
		NRF_WriteRegister(RF_CH, 0x02);
		NRF_WriteRegister(RF_SETUP, 0x0E);
		NRF_WriteRegister(STATUS, 0x00);
		NRF_WriteRegister(OBSERVE_TX, 0x00);
		NRF_WriteRegister(CD, 0x00);
		uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
		NRF_WriteRegisterMultiple(RX_ADDR_P0, rx_addr_p0_def, 5);
		uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
		NRF_WriteRegisterMultiple(RX_ADDR_P1, rx_addr_p1_def, 5);
		NRF_WriteRegister(RX_ADDR_P2, 0xC3);
		NRF_WriteRegister(RX_ADDR_P3, 0xC4);
		NRF_WriteRegister(RX_ADDR_P4, 0xC5);
		NRF_WriteRegister(RX_ADDR_P5, 0xC6);
		uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
		NRF_WriteRegisterMultiple(TX_ADDR, tx_addr_def, 5);
		NRF_WriteRegister(RX_PW_P0, 0);
		NRF_WriteRegister(RX_PW_P1, 0);
		NRF_WriteRegister(RX_PW_P2, 0);
		NRF_WriteRegister(RX_PW_P3, 0);
		NRF_WriteRegister(RX_PW_P4, 0);
		NRF_WriteRegister(RX_PW_P5, 0);
		NRF_WriteRegister(FIFO_STATUS, 0x11);
		NRF_WriteRegister(DYNPD, 0);
		NRF_WriteRegister(FEATURE, 0);
	}
}


