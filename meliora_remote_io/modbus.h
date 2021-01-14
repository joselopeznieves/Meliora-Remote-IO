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
extern int instant_readDigital(int start, int end);
extern float scale(float n1, float n2, float m1, float m2, float value);
extern float slope(float M, float D, float value);
extern void instant_readAnalog();
extern char* clientHandler(char* buffer);
extern void readMask(int* discrete, int* coils, int* input, int* holding);
extern void save_autoscaling(char* message);
extern void save_slopeintercept(char* message, int state);
extern void set_flag();
extern void clear_flag();
extern void save_udma(char* message);
extern unsigned char* sendRegisterValues(int state);

#endif /* MODBUS_H_ */
