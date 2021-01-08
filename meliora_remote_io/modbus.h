/*
 * modbus.h
 *
 *  Created on: Dec 4, 2020
 *      Author: Irving
 */

#ifndef MODBUS_H_
#define MODBUS_H_

extern int* readBits(int* bits, int address, int amount);
extern int* readRegisters(int* registers, int address, int amount);
extern void writeBit(int* bits, int address, int value);
extern void writeRegister(int* registers, int address, int value);
extern void writeMultipleBits(int* bits, int address, int amount, int* values);
extern void writeMultipleRegisters(int* registers, int address, int amount, int* values);
extern char* clientHandler(char* buffer);

#endif /* MODBUS_H_ */
