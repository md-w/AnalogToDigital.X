
#include "eeprom_wrapper.h"
#include "memory.h"

void readByte(unsigned char* buffer, unsigned char addr, unsigned char nBytes) {
    int i;
    for (i = 0; i < nBytes; i++) {
        *(buffer++) = DATAEE_ReadByte(addr++);
    }
}

void writeByte(unsigned char* buffer, unsigned char addr, unsigned char nBytes) {
    int i;
    for (i = 0; i < nBytes; i++) {
        DATAEE_WriteByte(addr++, *(buffer++));
    }
}