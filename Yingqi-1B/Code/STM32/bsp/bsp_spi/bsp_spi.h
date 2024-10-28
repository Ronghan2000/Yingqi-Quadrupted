#ifndef BSP_SPI_H
#define BSP_SPI_H
#include "struct_typedef.h"

#include <stdint.h>
extern void SPI1_DMA_init(unsigned long tx_buf, unsigned long rx_buf, uint16_t num);
extern void SPI1_DMA_enable(unsigned long tx_buf, unsigned long rx_buf, uint16_t ndtr);

#endif
