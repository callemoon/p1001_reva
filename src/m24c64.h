/*
 * m24c64.h
 *
 *  Created on: May 5, 2016
 *      Author: calle
 */

#ifndef M24C64_H_
#define M24C64_H_

void m24c64_init(void);
void m24c64_write(uint16_t address, uint8_t data);
uint8_t m24c64_read(uint16_t address);


#endif /* M24C64_H_ */
