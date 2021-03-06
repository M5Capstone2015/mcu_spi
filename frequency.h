/*
 * frequency.h
 *
 *  Created on: Jan 24, 2015
 *      Author: Dan Thompson
 */

#ifndef FREQUENCY_H_
#define FREQUENCY_H_

#include "iocontrols.h"

#define BYTE_SIZE = 8

uint32_t mappedByte = 0;
int byte_array[8];

/**************************************************************************//**
 * @brief Toggles io with a set delay time a number of times
 * @param delayTime - delay times in Milliseconds (period will be twice this number), repetitions
 *****************************************************************************/
void toggle_with_delay(uint32_t delayTime, uint32_t repetitions)
{
	uint32_t i;
	for ( i=0; i<repetitions; i++ )
	  {
		  set_pin(0);
		  delay(delayTime);
		  clear_pin(0);
		  delay(delayTime);
	  }
}

/**************************************************************************//**
 * @brief Defines frequencies and lengths for 0,1, and header.
 * @param Bit: 0 = 0 bit, 1 = 1 bit, else = Header bit
 *****************************************************************************/
void send_bit(uint32_t bit)
{
	if((bit == 0))
	{
		toggle_with_delay(100,10);
	}
	else if((bit == 1))
	{
		toggle_with_delay(200,7);
	}
	else
	{
		toggle_with_delay(150,14);
	}
}


void toBitArray(int a,int* output){
	if (a >= 255){
		a = 255;
	}
	int i=0;
	int tmp;
	for (i=0;i<8;i++){
		tmp = a % 2;
		output[7-i] = tmp;
		a = a/2;
	}
}

int mappingFunction(int a){
	int mappedOutput;
	if (a <= 750 || a >= 1000)
		mappedOutput = 0;
    else
    	mappedOutput = a - 750;
	return mappedOutput;
}

int entered_send_byte = 0;
void send_byte(uint32_t byte)
{
	entered_send_byte++;
	int i, temp;
	send_bit(2);
	mappedByte = mappingFunction(byte);
	for(i = 7; i > -1; i--) {
		temp = mappedByte >> i;
//		temp = byte >> i;
		temp &= 0x1;
		byte_array[i] = temp;
		send_bit(temp);
	}

}

#endif /* FREQUENCY_H_ */
