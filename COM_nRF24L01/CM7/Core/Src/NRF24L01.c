

#include "NRF24L01.h"

extern SPI_HandleTypeDef hspi1;
#define NRF24_SPI &hspi1

#define NRF24_CE_PORT GPIOD
#define NRF24_CE_PIN GPIO_PIN_15

#define NRF24_CSN_PORT GPIOD
#define NRF24_CSN_PIN GPIO_PIN_14


void CS_Select(void) {
    HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET);
}


void CS_UnSelect(void) {
    HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);
}


void CE_Enable(void) {
    HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}


void CS_Disable(void) {
    HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}


/* Write multiple bytes starting from the given register */
void  nrf24_WriteReg_Multi(uint8_t Reg, uint8_t *Data, int size) {
    uint8_t buf[2];
    buf[0] = Reg|1<<5;  // 5th bit must be 1 for writing
    // buf[1] = Data;

    // Pull the CS Pin LOW to select the device
    CS_Select();

    HAL_SPI_Transmit(NRF24_SPI, buf, 1, 100);
    HAL_SPI_Transmit(NRF24_SPI, Data, size, 1000);

    // Pull the CS Pin HIGH to de-select the device
    CS_UnSelect();
}


/* Write a single byte to the given register */
void nrf24_WriteReg(uint8_t Reg, uint8_t *Data) {
    nrf24_WriteReg_Multi(Reg, Data, 1);
}


/* Read multiple bytes starting from the given register */
void nrf24_ReadReg_Multi(uint8_t Reg, uint8_t *Data, int size) {
    // Pull the CS Pin LOW to select the device
    CS_Select();

    HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
    HAL_SPI_Receive(NRF24_SPI, Data, size, 1000);

    // Pull the CS Pin HIGH to de-select the device
    CS_UnSelect();
}


/* Read a single byte from the given register */
void nrf24_ReadReg(uint8_t Reg, uint8_t *Data) {
    nrf24_ReadReg_Multi(Reg, Data, 1);
}


/* Send a command to the NRF */
void nrf24_SendCmd(uint8_t cmd) {
    // Pull the CS Pin LOW to select the device
    CS_Select();

    HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

    // Pull the CS Pin HIGH to de-select the device
    CS_UnSelect();
}


void NRF24_Init(void) {
    // Disable the chip before configuring the device
    CE_Disable();

    nrf24_WriteReg(CONFIG, 0);      // Will be configured later
    nrf24_WriteReg(EN_AA, 0);       // No Auto ACK
    nrf24_WriteReg(EN_RXADDR, 0);   // No enabling any data pipe right now
    nrf24_WriteReg(SETUP_AW, 0x03); // 5 bytes for the TX/RX address
    nrf24_WriteReg(SETUP_RETR, 0);  // No retransmissions
    nrf24_WriteReg(RF_CH, 0);       // Will be setup during TX or RX
    nrf24_WriteReg(RF_SETUP, 0x0E); // Power = 0db, data rate = 2Mbps

    // Enable the chip after configuring the device
    CE_Enable();
}


/* Setup the TX mode */
void NRF24_TXMode(uint8_t *Address, uint8_t channel) {
    // Disable the chip before configuring the device
    CE_Disable();

    nrf24_WriteReg(RF_CH, channel);  // Select the channel
    nrf24_WriteReg_Multi(TX_ADDR, Address, 5); // Set the TX address

    // Power up the device
    uint8_t config;
    nrf24_ReadReg(CONFIG, config);
    config |= (1<<1); // Set the PWR_UP bit
    nrf24_WriteReg(CONFIG, config);

    // Enable the chip after configuring the device
    CE_Enable();
}


/* Transmit the data */
uint8_t NRF24_Transmit(uint8_t *Data) {
    uint8_t cmdToSend = 0;

    // Payload command
    cmdToSend = W_TX_PAYLOAD;
    nrf24_SendCmd(cmdToSend);

    // Select the device
    CS_Select();

    // Send the payload
    HAL_SPI_Transmit(NRF24_SPI, Data, 32, 1000);

    // Unselect the device
    CS_UnSelect();

    HAL_Delay(1);
    uint8_t fifoStatus;
    nrf24_ReadReg(FIFO_STATUS, fifoStatus);
    
    /* Check the 4th bit of FIFO_STATUS to know if the TX fifo is empty */
    if ((fifoStatus & (1<<4)) && (!(fifoStatus & (1<<3)))) {
        cmdToSend = FLUSH_TX;
        nrf24_SendCmd(cmdToSend);
        return 1;   // Success
    }
    return 0;   // Failure
}
