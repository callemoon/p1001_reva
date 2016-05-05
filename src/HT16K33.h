/*
 * HT16K33.h
 *
 *  Created on: May 5, 2016
 *      Author: calle
 */

#ifndef HT16K33_H_
#define HT16K33_H_

void ht16k33_init(void);
void ht16k33_i2cwrite(uint8_t i2cAdress, uint8_t command, uint8_t *data, uint8_t len);
void ht16k33_writepixeldata(void);

#endif /* HT16K33_H_ */
