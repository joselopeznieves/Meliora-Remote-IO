/*
 * channel_interface.h
 *
 *  Created on: Jan 13, 2021
 *      Author: josel
 */
#ifndef CHANNEL_INTERFACE_H_
#define CHANNEL_INTERFACE_H_

extern void ADCInit(void);
extern void InitPWMModules();
extern int ReadDigitalInput(int channel);
extern void WriteDigitalOutput(int channel, int state);
extern float ReadAnalogInput(int channel);
extern void WriteAnalogOutput(int channel, float voltage);



#endif /* CHANNEL_INTERFACE_H_ */
