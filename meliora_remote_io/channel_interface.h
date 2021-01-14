/*
 * channel_interface.h
 *
 *  Created on: Jan 13, 2021
 *      Author: josel
 */
#ifndef CHANNEL_INTERFACE_H_
#define CHANNEL_INTERFACE_H_

void ADCInit(void);
void InitPWMModules();
int ReadDigitalInput(int channel);
void WriteDigitalOutput(int channel, int state);
float ReadAnalogInput(int channel);
void WriteAnalogOutput(int channel, float voltage);



#endif /* CHANNEL_INTERFACE_H_ */
