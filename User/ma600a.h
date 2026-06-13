//
// Created by DJ on 26-6-9.
//

#ifndef MA600A_H
#define MA600A_H

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

// 寄存器地址与滤波选项
#define MTSP_ADDR     0x1C
#define FILTER12_5    0x05
#define FILTER14      0x08
#define TABLE_SIZE    100
// 以下宏在 ma600a.c 中也可能被定义，为了头文件可用性，在未定义时提供默认值
#define POLE_PAIRS 50


// 函数原型（供外部调用）
void ma600a_write_register(uint8_t reg_addr, uint8_t value);
void ma600a_read_register(uint8_t *rxbuf, uint8_t reg_addr);
void ma600a_set_MTSP(void);
void ma600a_set_filter(void);
void ma600a_init(void);

#ifdef __cplusplus
}
#endif

#endif // MA600A_H
