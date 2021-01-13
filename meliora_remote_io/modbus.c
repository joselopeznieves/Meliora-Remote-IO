/*
 * modbus.c
 *
 *  Created on: Dec 4, 2020
 *      Author: Irving
 */
#include "stdlib.h"
#include "string.h"
#include "modbus.h"
#include "channel_interface.h"

#define COILS                     4
#define DISCRETE_INPUTS           4
#define HOLDING_REGISTERS         8
#define INPUT_REGISTERS           8
#define ANALOG_CHANNELS           4
#define SCALING_PARAMETERS        4

/*
 * MODBUS Data Types
 * coils -> Digital Outputs
 * discrete inputs -> Digital Inputs
 * holding registers -> Analog Outputs
 * input registers -> Analog Inputs
 */
int coils[1];
int discrete_inputs[1];
int holding_registers[2*HOLDING_REGISTERS] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int input_registers[2*INPUT_REGISTERS] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*
 * State of channels
 * 0 -> channel is off
 * 1 -> channel is on
 */
int coilmask[COILS] = {0,0,0,0};
int discretemask[DISCRETE_INPUTS] = {0,0,0,0};
int holdingmask[HOLDING_REGISTERS] = {0,0,0,0,0,0,0,0};
int inputmask[INPUT_REGISTERS] = {0,0,0,0,0,0,0,0};

/*
 * Initialize auto-scaling with default values
 * Analog Inputs: [0,1.5] -> [-24,24]
 *
 * Note: Reading Analog Inputs scales the value down to a number in [0,1.5]
 */
float autoScaling[ANALOG_CHANNELS][SCALING_PARAMETERS] = {{0.0,-24.0,1.5,24.0},
                                                          {0.0,-24.0,1.5,24.0},
                                                          {0.0,-24.0,1.5,24.0},
                                                          {0.0,-24.0,1.5,24.0}};
/*
Function: readBits

Description: Reads multiple coils or discrete inputs starting from a given
            address specified in the client request

Parameters:
    data -> Coil or Discrete Input Array to be read
    address -> Starting address for Coils or Discrete Inputs to be read
    amount -> Amount of Coils or Discrete Inputs to be read
 */
int* readBits(int* bits, int address, int amount) {
    int index = (address-1) >> 3;
    int last = index+(amount >> 3);
    int shift = (address-1)%8;
    int size = amount >> 3;
    if(amount%8 > 0)
        size++;
    int* response = (int*) malloc((size+1)*sizeof(int));
    response[0] = size;
    int count = 1;
    int i;
    for(i = index; i <= last; i++) {
        if(i < last)
            response[count] = ((bits[i+1] << (8-shift)) | (bits[i] >> shift)) & 0xFF;
        else
            response[count] = (bits[i] >> shift) & (0xFF >> (8-amount%8));
        count++;
    }
    return response;
}

/*
Function: readRegisters

Description: Reads multiple Holding Registers or Output Registers starting
            from a given address in the client request

Parameters:
    data -> Array of Holding Registers or Output Registers to be read
    address -> Starting address for Holding Registers or Output Registers to be read
    amount -> Amount of Holding Registers or Output Registers to be read
 */
int* readRegisters(int* registers, int start, int amount) {
    int index = (start-1) << 1;
    int last = index+(amount << 1);
    int size = amount << 1;
    int* response = (int*) malloc((size+1)*sizeof(int));
    response[0] = size;
    int count = 1, i;
    for(i = index; i < last; i++) {
        response[count] = registers[i];
        count++;
    }
    return response;
}

/*
Function: writeBit

Description: Ovewrites the status of the Coil in the given address

Parameters:
    data -> Coil Array to be written
    address -> Coil address to be modified
    value -> Value to be stored

Note:
    The value must be 0xFF00 to store a 1 and 0x0000 to store a 0 according
    to Modbus documentation.
 */
void writeBit(int* bits, int address, int value) {
    int index = (address-1) >> 3;
    int shift = (address-1)%8;
    bits[index] = bits[index] & (0xFF ^ (1 << shift));
    if(value == 0xFF00)
        bits[index] = bits[index] | (0x01 << shift);
    int channel = (index << 3)+shift;
    int state = (value >> 8) & 0x01;
    WriteDigitalOutput(channel, state);
}

/*
Function: writeRegister

Description: Overwrites content of the Output Register in the given address

Parameters:
    data -> Holding Register Array to be written
    address -> Address of Holding Register to be modified
    value -> Value to be stored in the Holding Register

Note:
    Value in this case stores a 2-byte number instead of 1-byte like
    the data array
 */
void writeRegister(int* registers , int address, int value) {
    int index = (address-1) << 1;
    registers[index+1] = value & 0x00FF;
    registers[index] = (value & 0xFF00) >> 8;

}

/*
Function: writeMultipleBits

Description: Overwrites values of multiple coils

Parameters:
    bits -> Coil Array to be written
    address -> Starting address for Coils to be written
    amount -> Amount of Coils to be modified
    values -> Array of values to be stored
 */
void writeMultipleBits(int* bits, int start, int amount, int* values) {
    int end = start+amount;
    int count = amount/8;
    int val = 0;
    int j = 0;
    int i;
    int shift = (start-1)%8;
    for(i = start; i < end; i++) {
        val = (values[j] >> shift) & 0x01;
        if(val == 0x01)
            writeBit(bits, i, 0xFF00);
        else
            writeBit(bits, i, 0x0000);
        shift++;
        if(shift >= 8 && j < count) {
            shift = 0;
            j++;
        }
    }
}

/*
Function: writeMultipleRegisters

Description: Overwrites values of multiple registers

Parameters:
    data -> Output Register Array to be written
    address -> Starting address for Output Registers to be written
    amount -> Amount of Output Registers to be modified
    values -> Array of values to be stored
 */
void writeMultipleRegisters(int* registers, int address, int amount, int* values) {
    int start = (address-1) << 1;
    int end = start+(amount << 1);
    int index = 0, i;
    for(i = start; i < end; i++) {
        registers[i] = values[index];
        index++;
    }
    short upper, lower;
    float voltage = 0.0;
    for(i = 0; i < 4; i++) {
        upper = (registers[4*i] << 8) | registers[4*i+1];
        lower = (registers[4*i+2] << 8) | registers[4*i+3];
        short bits[2] = {upper, lower};
        memcpy(&voltage, bits, sizeof(voltage));
        WriteAnalogOutput(i, voltage);
    }
}

/********************
 * HELPER FUNCTIONS *
 ********************/

/*
Function: instant_read

Description: Read the values at the Digital input channels of the Microcontroller

Parameters:
    start -> starting address for channels to be read
    end -> last address for channels to be read
 */
int instant_readDigital(int start, int end) {
    int value = 0;
    int i;
    for(i = start; i < end; i++) {
        int bit = ReadDigitalInput(i);
        value = value | (bit << i);
    }
    return value;
}

float scale(float n1, float n2, float m1, float m2, float value) {
    float result = n2+((value-n1)*((m2-n2)/(m1-n1)));
    return result;
}

void instant_readAnalog() {
    int i;
    float read, value;
    short float2int[2] = {0};
    for(i = 0; i < 4; i++) {
        value = 0.0;
        if(inputmask[2*i]) {
            read = ReadAnalogInput(i); // [0,1.5]
            value = scale(autoScaling[i][0], autoScaling[i][1], autoScaling[i][2], autoScaling[i][3], read); // [-24,24]
        }
        memcpy(float2int, &value, sizeof(float2int));
        input_registers[4*i] = (float2int[0] & 0xFF00) >> 8;
        input_registers[4*i+1] = (float2int[0] & 0x00FF);
        input_registers[4*i+2] = (float2int[1] & 0xFF00) >> 8;
        input_registers[4*i+3] = (float2int[1] & 0x00FF);
    }
}

/*
Function: clientHandler

Description: Executes a function according to the Modbus function codes.
            Ignores request if function code is not supprted (not implemented)
            or if at least one parameter is invalid

Parameters:
    fc -> Modbus function code
    data -> Array to be read or written depending on Modbus function
    address -> Starting address for Modbus data to be read or written
    amount -> Amount of Modbus data to be read or written
    value -> Value to be stored in a single register or coil
    values -> Array of values to be stored in multiple register or coils
 */
char* clientHandler(char buffer[]) {
    char* response = (char*) malloc(300*sizeof(char));
    int len = 7;
    int i, address, amount, value, maskFlag, start, last;
    int* values = (int*) malloc(20*sizeof(int));

    // Copy MBAP Header
    // NOTE: length field must be changed before sending message, unless it is a write
    for(i = 0; i < len; i++)
        response[i+1] = buffer[i];
    int fc = buffer[7];

    // Exception Code in case command is invalid
    int ec = (fc+0x80) << 8;

    switch(fc) {
    // Read values of coils
    // Each coil contains 1 bit of information
    // Each space in the data array contains 8 coils
    case 0x01: {
        response[8] = fc;
        address = (buffer[8] << 8) | buffer[9];
        amount = (buffer[10] << 8) | buffer[11];
        if(address < 1 || address > COILS) {
            ec = ec | 0x02;
            goto exception01;
        }
        else if(amount < 1 || amount > COILS || address+amount-1 > COILS) {
            ec = ec | 0x03;
            goto exception01;
        }
        maskFlag = 0;
        start = address-1;
        last = start+amount;
        for(i = start; i < last; i++)
            if(!coilmask[i]) maskFlag = 1;

        if(maskFlag) {
            ec = ec | 0x04;
            goto exception01;
        }
        if((ec & 0x00FF) == 0) {
            int* bits = readBits(coils, address, amount);
            for(i = 0; i <= bits[0]; i++)
                response[9+i] = bits[i];
            response[6] = bits[0]+3;
            len += response[6]-1;
            response[0] = len;
            return response;
        }

        else {
exception01:
            response[0] = 9;
            response[6] = 0x03;
            response[8] = (ec & 0xFF00) >> 8;
            response[9] = ec & 0x00FF;
            return response;
        }
    }
    // Read values of discrete inputs
    // Each discrete input contains 1 bit of information
    // Each space in the data array contains 8 discrete inputs
    case 0x02: {
        response[8] = fc;
        address = (buffer[8] << 8) | buffer[9];
        amount = (buffer[10] << 8) | buffer[11];
        if(address < 1 || address > DISCRETE_INPUTS) {
            ec = ec | 0x02;
            goto exception02;
        }
        else if(amount < 1 || amount > DISCRETE_INPUTS || address+amount-1 > DISCRETE_INPUTS) {
            ec = ec | 0x03;
            goto exception02;
        }
        maskFlag = 0;
        start = address-1;
        last = start+amount;
        for(i = start; i < last; i++)
            if(!discretemask[i]) maskFlag = 1;

        if(maskFlag) {
            ec = ec | 0x04;
            goto exception02;
        }
        if((ec & 0x00FF) == 0) {
            discrete_inputs[0] = instant_readDigital(start, last);
            int* bits = readBits(discrete_inputs, address, amount);
            for(i = 0; i <= bits[0]; i++)
                response[9+i] = bits[i];
            response[6] = bits[0]+3;
            len += response[6]-1;
            response[0] = len;
            return response;
        }
        else {
exception02:
            response[0] = 9;
            response[6] = 0x03;
            response[8] = (ec & 0xFF00) >> 8;
            response[9] = ec & 0x00FF;
            return response;
        }
    }
    // Read values of holding registers
    // Each register contains 2 bytes of information
    // Each space in the data array contains 1 byte of a register
    // stored in a Big-endian format
    case 0x03: {
        response[8] = fc;
        address = (buffer[8] << 8) | buffer[9];
        amount = (buffer[10] << 8) | buffer[11];
        if(address < 1 || address > HOLDING_REGISTERS) {
            ec = ec | 0x02;
            goto exception03;
        }
        else if(amount < 1 || amount > HOLDING_REGISTERS || address+amount-1 > HOLDING_REGISTERS) {
            ec = ec | 0x03;
            goto exception03;
        }
        maskFlag = 0;
        start = address-1;
        last = start+amount;
        for(i = start; i < last; i++)
            if(!holdingmask[i]) maskFlag = 1;

        if(maskFlag) {
            ec = ec | 0x04;
            goto exception03;
        }
        if((ec & 0x00FF) == 0) {
            int* bits = readRegisters(holding_registers, address, amount);
            for(i = 0; i <= bits[0]; i++)
                response[9+i] = bits[i];
            response[6] = bits[0]+3;
            len += response[6]-1;
            response[0] = len;
            return response;
        }
        else {
exception03:
            response[0] = 9;
            response[6] = 0x03;
            response[8] = (ec & 0xFF00) >> 8;
            response[9] = ec & 0x00FF;
            return response;
        }
    }
    // Read values of input registers
    // Each register contains 2 bytes of information
    // Each space in the data array contains 1 byte of a register
    // stored in a Big-endian format
    case 0x04: {
        response[8] = fc;
        address = (buffer[8] << 8) | buffer[9];
        amount = (buffer[10] << 8) | buffer[11];
        if(address < 1 || address > INPUT_REGISTERS) {
            ec = ec | 0x02;
            goto exception04;
        }
        else if(amount < 1 || amount > INPUT_REGISTERS || address+amount-1 > INPUT_REGISTERS) {
            ec = ec | 0x03;
            goto exception04;
        }
        maskFlag = 0;
        start = address-1;
        last = start+amount;
        for(i = start; i < last; i++)
            if(!inputmask[i]) maskFlag = 1;

        if(maskFlag) {
            ec = ec | 0x04;
            goto exception04;
        }
        if((ec & 0x00FF) == 0) {
            instant_readAnalog();
            int* bits = readRegisters(input_registers, address, amount);
            for(i = 0; i <= bits[0]; i++)
                response[9+i] = bits[i];
            response[6] = bits[0]+3;
            len += response[6]-1;
            response[0] = len;
            return response;
        }
        else {
exception04:
            response[0] = 9;
            response[6] = 0x03;
            response[8] = (ec & 0xFF00) >> 8;
            response[9] = ec & 0x00FF;
            return response;
        }
    }
    // Write to a single coil
    // Each coil contains 1 bit of information
    // Each space in the data array contains 8 coils
    case 0x05: {
        response[8] = fc;
        address = (buffer[8] << 8) | buffer[9];
        value = (buffer[10] << 8) | buffer[11];
        if(address < 1 || address > COILS) {
            ec = ec | 0x02;
            goto exception05;
        }
        else if(value != 0xFF00 && value != 0x0000) {
            ec = ec | 0x03;
            goto exception05;
        }
        else if(!coilmask[address-1]) {
            ec = ec | 0x04;
            goto exception05;
        }
        if((ec & 0x00FF) == 0) {
            writeBit(coils, address, value);
            len = 12;
            for(i = 0; i < len; i++)
                response[i+1] = buffer[i];
            response[0] = len;
            response[6] = buffer[5]-2;
            return response;
        }
        else {
exception05:
            response[0] = 9;
            response[6] = 0x03;
            response[8] = (ec & 0xFF00) >> 8;
            response[9] = ec & 0x00FF;
            return response;
        }
    }
    // Write to a single holding registers
    // Each register contains 2 bytes of information
    // Each space in the data array contains 1 byte of a register
    // stored in a Big-endian format
    case 0x06: {
        response[8] = fc;
        address = (buffer[8] << 8) | buffer[9];
        value = (buffer[10] << 8) | buffer[11];
        if(address < 1 || address > COILS) {
            ec = ec | 0x02;
            goto exception06;
        }
        else if(value < 0x0000 || value > 0xFFFF) {
            ec = ec | 0x03;
            goto exception06;
        }
        else if(!holdingmask[address-1]) {
            ec = ec | 0x04;
            goto exception06;
        }
        if((ec & 0x00FF) == 0) {
            writeRegister(holding_registers, address, value);
            len = 12;
            for(i = 0; i < len; i++)
                response[i+1] = buffer[i];
            response[0] = len;
            response[6] = buffer[5]-2;
            return response;
        }
        else {
exception06:
            response[0] = 9;
            response[6] = 0x03;
            response[8] = (ec & 0xFF00) >> 8;
            response[9] = ec & 0x00FF;
            return response;
        }
    }
    // Write to multiple coil
    // Each coil contains 1 bit of information
    // Each space in the data array contains 8 coils
    case 0x0F: {
        response[8] = fc;
        address = (buffer[8] << 8) | buffer[9];
        amount = (buffer[10] << 8) | buffer[11];
        if(address < 1 || address > COILS) {
            ec = ec | 0x02;
            goto exception0F;
        }
        else if(amount < 1 || address+amount-1 > COILS) {
            ec = ec | 0x03;
            goto exception0F;
        }
        maskFlag = 0;
        start = address-1;
        last = start+amount;
        for(i = start; i < last; i++)
            if(!coilmask[i]) maskFlag = 1;

        if(maskFlag) {
            ec = ec | 0x04;
            goto exception0F;
        }
        if((ec & 0x00FF) == 0) {
            for(i = 0; i < buffer[12]; i++)
                values[i] = buffer[i+13];
            writeMultipleBits(coils, address, amount, values);
            response[0] = 12;
            for(i = 0; i < response[0]; i++)
                response[i+1] = buffer[i];
            response[6] = 6;
            return response;
        }
        else {
exception0F:
            response[0] = 9;
            response[6] = 0x03;
            response[8] = (ec & 0xFF00) >> 8;
            response[9] = ec & 0x00FF;
            return response;
        }
    }
    // Write to multiple output registers
    // Each register contains 2 bytes of information
    // Each space in the data array contains 1 byte of a register
    // stored in a Big-endian format
    case 0x10: {
        response[8] = fc;
        address = (buffer[8] << 8) | buffer[9];
        amount = (buffer[10] << 8) | buffer[11];
        if(address < 1 || address > HOLDING_REGISTERS || (address+1)%2) {
            ec = ec | 0x02;
            goto exception10;
        }
        else if(amount < 1 || address+amount-1 > HOLDING_REGISTERS) {
            ec = ec | 0x03;
            goto exception10;
        }
        maskFlag = 0;
        start = address-1;
        last = start+amount;
        for(i = start; i < last; i++)
            if(!holdingmask[i]) maskFlag = 1;

        if(maskFlag) {
            ec = ec | 0x04;
            goto exception10;
        }
        if((ec & 0x00FF) == 0) {
            for(i = 0; i < buffer[12]; i++)
                values[i] = buffer[i+13];
            writeMultipleRegisters(holding_registers, address, amount, values);
            response[0] = 12;
            for(i = 0; i < response[0]; i++)
                response[i+1] = buffer[i];
            response[6] = 6;
            return response;
        }
        else {
exception10:
            response[0] = 9;
            response[6] = 0x03;
            response[8] = (ec & 0xFF00) >> 8;
            response[9] = ec & 0x00FF;
            return response;
        }
    }
    default:
    {
        return 0;
    }
    }
}

void readMask(int* coils, int* discrete, int* holding, int* input) {
    int i;
    for(i = 0; i < 4; i++) {
        coilmask[i] = coils[i];
        discretemask[i] = discrete[i];
        holdingmask[2*i] = holding[i];
        holdingmask[2*i+1] = holding[i];
        inputmask[2*i] = input[i];
        inputmask[2*i+1] = input[i];
    }

}

void saveAutoScaling(char* message) {
       char * token = strtok(message, ",");
       int channel = atoi(token);
       token = strtok(NULL, ",");
       int i;
       for(i = 0; i < 4; i++) {
          autoScaling[channel][i] = atof(token);
          token = strtok(NULL, ",");
       }
}

unsigned char* sendRegisterValues(int state) {
    int i;
    int Registers[4];
    unsigned char* values = (unsigned char*) malloc(30*sizeof(unsigned char));
    switch(state) {
    // Analog Inputs
    case 0: {

        for(i = 0; i < 4; i++) {
            short high = (short) ((input_registers[4*i] << 8) | input_registers[4*i+1]);
            short low = (short) ((input_registers[4*i+2] << 8) | input_registers[4*i+3]);
            int bits = (high << 16) | low;
            Registers[i] = bits;
        }
        sprintf(values, "%0.1f,%0.1f,%0.1f,%0.1f", Registers[0], Registers[1], Registers[2], Registers[3]);
        break;
    }
    case 1: {
        for(i = 0; i < 4; i++) {
            short high = (short) ((holding_registers[4*i] << 8) | holding_registers[4*i+1]);
            short low = (short) ((holding_registers[4*i+2] << 8) | holding_registers[4*i+3]);
            int bits = (high << 16) | low;
            Registers[i] = bits;
        }
        sprintf(values, "%0.1f,%0.1f,%0.1f,%0.1f", Registers[0], Registers[1], Registers[2], Registers[3]);
        break;
    }
    default: {
        break;
    }
    }
    return values;
}
