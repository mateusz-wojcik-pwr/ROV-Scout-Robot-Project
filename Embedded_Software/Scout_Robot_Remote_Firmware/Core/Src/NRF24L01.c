
#include <NRF24L01.h>

#define NRF_CSN_Pin GPIO_PIN_12
#define NRF_CSN_GPIO_Port GPIOA
#define NRF_IRQ_Pin GPIO_PIN_6
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_7
#define NRF_CE_GPIO_Port GPIOB


SPI_HandleTypeDef* nrf_spi;


void NRF_CS_Select(){
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
}

void NRF_CS_Unselect(){
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

void NRF_CE_Enable(){
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
}

void NRF_CE_Disable(){
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
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
	HAL_SPI_Transmit(nrf_spi, &buffer, 1, 100);
	//send the data all at once
	HAL_SPI_Transmit(nrf_spi, &data, size, 100);

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

void NRF_Init(SPI_HandleTypeDef* spi){
	//substitute used SPI handler
	nrf_spi = spi;

	//disable device for configuration
	NRF_CE_Disable();

	/* configure interrupt masking, power mode and working mode to transmit */
	NRF_WriteRegister(CONFIG, 0);

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
void NRF_SetTxMode(uint8_t *RXPipeAddr, uint8_t channel){

	/* disable device for configuration */
	NRF_CE_Disable();

	/* Select the channel */
	NRF_WriteRegister(RF_CH, channel);

	/* set up the TX address */
	NRF_WriteRegisterMultiple(TX_ADDR, RXPipeAddr, 5);


	/* power up the device */
	uint8_t config = NRF_ReadRegister(CONFIG);
	config = config | (1<<1);
	NRF_WriteRegister(CONFIG, config);

	/* enable the device back */
	NRF_CE_Enable();
}

/* transmit data over SPI to the NRF module */

uint8_t NRF_Transmit(uint8_t* data){
	uint8_t command;
	command = W_TX_PAYLOAD;


	/* transmit payload command */
	NRF_SendCommand(command);

	/* transmit 32 bit long data stream */
	HAL_SPI_Transmit(nrf_spi, data, 32, 100);

	/* wait for the pin to settle */
	HAL_Delay(1);

	/* check for data in fifo - transmission successful */
	uint8_t fifoStatus = NRF_ReadRegister(FIFO_STATUS);
	if((fifoStatus & (1<<4)) && (!(fifoStatus & (1<<3)))){
		/* if the data was transfered to the fifo flush it for sending */
		command = FLUSH_TX;
		NRF_SendCommand(command);
		return 1;
	}
		return 0;
}








