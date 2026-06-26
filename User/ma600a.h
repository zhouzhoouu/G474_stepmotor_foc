#ifndef MA600A_PORT_H_
#define MA600A_PORT_H_
#include <stdint.h>

// Registers
#define MTSP_ADDR       0x1C
#define STATUS_ADDR     0x1A
#define FILTER14      0x08  // 14-bit mode
#define POLE_PAIRS      50

void ma600a_init(void);
void ma600a_write_register(uint8_t addr, uint8_t val);
void ma600a_read_register(uint8_t *rx, uint8_t addr);
void ma600a_clear_errors(void);
void ma600a_read_status(uint8_t *status);
#endif


