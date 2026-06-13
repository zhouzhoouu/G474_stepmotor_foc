#include "ma600a.h"

#include "spi.h"
uint8_t rxbuf_angle[4];
uint8_t spi3_free = 1;

//写ma600a的寄存器，只能写1位
void ma600a_write_register(uint8_t reg_addr, uint8_t value) {
    uint8_t tx_buf[6];  // 3帧 × 2字节
    uint8_t rx_buf[6];

    // 第1帧：写寄存器命令 0xEA54
    tx_buf[0] = 0xEA;
    tx_buf[1] = 0x54;
    // 第2帧：地址 + 值
    tx_buf[2] = reg_addr;
    tx_buf[3] = value;
    // 第3帧：0x0000（读回验证）
    tx_buf[4] = 0x00;
    tx_buf[5] = 0x00;

    // 第1帧
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 2, 100);
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);
    for (volatile int i = 0; i < 25; i++) { __NOP(); }

    // 第2帧
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx_buf+ 2, rx_buf + 2, 2, 100);
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);
    for (volatile int i = 0; i < 25; i++) { __NOP(); }

    // 第3帧
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx_buf + 4, rx_buf + 4, 2, 100);
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);
    for (volatile int i = 0; i < 25; i++) { __NOP(); }
    // 可选：验证 rx_buf[5] == value
}

//读ma600a的寄存器，只能读1位
void ma600a_read_register(uint8_t *rxbuf, uint8_t reg_addr) {

    uint8_t tx_buf[4];
    uint8_t rx_buf[4];

    tx_buf[0] = 0xD2;
    tx_buf[1] = reg_addr;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00;

    // 第1帧
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx_buf, rx_buf, 2, 100);
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);

    // 帧间延时：至少 120 ns
    // 在 168 MHz 下，120 ns ≈ 21 个时钟周期
    for (volatile int i = 0; i < 40; i++) {  // 多给几个余量
        __NOP();
    }

    // 第2帧
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx_buf + 2, rx_buf + 2, 2, 100);
    HAL_GPIO_WritePin(CSn_GPIO_Port, CSn_Pin, GPIO_PIN_SET);
    for (volatile int i = 0; i < 25; i++) { __NOP(); }
    *rxbuf = rx_buf[3];
}
//读取角度的中断函数，非阻塞

void ma600a_set_MTSP() {
    static uint8_t temp_MTSP;
    ma600a_read_register(&temp_MTSP, MTSP_ADDR);
    temp_MTSP |= 0x80; // 设置第7位为1
    ma600a_write_register(MTSP_ADDR,temp_MTSP);
    temp_MTSP = 0;
    ma600a_read_register(&temp_MTSP, MTSP_ADDR);
}

void ma600a_set_filter() {
    static uint8_t temp_filter;
    ma600a_read_register(&temp_filter,0x0D);
    temp_filter &= 0b11110000; // 清除低4位
    temp_filter |= FILTER14; // 设置低4位为0x08
    ma600a_write_register(0x0D,temp_filter);
    temp_filter = 0;
    ma600a_read_register(&temp_filter,0x0D);
}

void ma600a_init() {
    ma600a_set_MTSP();
    ma600a_set_filter();
}
