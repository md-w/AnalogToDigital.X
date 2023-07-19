/* 
 * File:   eeprom_wrapper.h
 * Author: user
 *
 * Created on March 21, 2021, 10:18 PM
 */

#ifndef EEPROM_WRAPPER_H
#define	EEPROM_WRAPPER_H


void readByte(unsigned char* buffer, unsigned char addr, unsigned char nBytes);
void writeByte(unsigned char* buffer, unsigned char addr, unsigned char nBytes);

#endif	/* EEPROM_WRAPPER_H */

