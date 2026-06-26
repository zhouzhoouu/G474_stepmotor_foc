#include "ma600a.h"
#include "spi.h"
#include "main.h"

// MA600A write: 3 separate SPI frames with 1ms delay between each
void ma600a_write_register(uint8_t addr, uint8_t val) {
    uint8_t tx[6], rx[6];
    tx[0]=0xEA; tx[1]=0x54;           // Frame 1: write command
    tx[2]=addr; tx[3]=val;            // Frame 2: address + value
    tx[4]=0x00; tx[5]=0x00;           // Frame 3: dummy readback

    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx+2, rx+2, 2, 100);
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx+4, rx+4, 2, 100);
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

// MA600A read: 2 separate SPI frames with 1ms delay
void ma600a_read_register(uint8_t *rx, uint8_t addr) {
    uint8_t tx[4], rx_buf[4]={0};
    tx[0]=0xD2; tx[1]=addr;            // Frame 1: read command
    tx[2]=0x00; tx[3]=0x00;            // Frame 2: dummy + readback

    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx, rx_buf, 2, 100);
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx+2, rx_buf+2, 2, 100);
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    *rx = rx_buf[3];
}

// Clear error flags: send 0xD700 as one 16-bit SPI frame
void ma600a_clear_errors(void) {
    uint8_t tx[2] = {0xD7, 0x00};
    uint8_t rx[2];
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

// Read STATUS register (0x1A) to check error flags
// Returns: status byte (bit 7=NVMB, bit 6=ERRCRC, bit 5=ERRMEM, bit 4=ERRPAR)
void ma600a_read_status(uint8_t *status) {
    ma600a_read_register(status, STATUS_ADDR);
}

// Init: clear errors, set MTSP + filter bandwidth
void ma600a_init(void) {
    // 1. Clear any power-up error flags
    ma600a_clear_errors();

    // 2. Set MTSP (multi-turn zero position) bit 7 in register 0x1C
    uint8_t temp;
    ma600a_read_register(&temp, MTSP_ADDR);
    temp |= 0x80;
    ma600a_write_register(MTSP_ADDR, temp);
    ma600a_read_register(&temp, MTSP_ADDR);

    // 3. Set filter for maximum resolution in register 0x0D
    ma600a_read_register(&temp, 0x0D);
    temp = (temp & 0xF0) | FILTER14;
    ma600a_write_register(0x0D, temp);
    ma600a_read_register(&temp, 0x0D);
}

