/*
 * ArduinoHardware
 *
 *  Created on: Aug 3, 2011
 *      Author: Adam Stambler
 */

#ifndef ARDUINOHARDWARE_H_
#define ARDUINOHARDWARE_H_

#include "WProgram.h"
#include <HardwareSerial.h>

class ArduinoHardware {
public:
	ArduinoHardware(HardwareSerial* io , long baud= 57600){
		iostream = io;
		baud_ = baud;
	}
	ArduinoHardware()
	{
		iostream = &Serial;
		baud_ = 57600;
	}
	ArduinoHardware(ArduinoHardware& h){
		this->baud_ = h.baud_;
		this->iostream = iostream;
	}
	
	void setBaud(long baud){
		this->baud_= baud;
	}
	
	int getBaud(){return baud_;}

	void init(){
		iostream->begin(baud_);
	}

	int read(){return iostream->read();};
	void write(uint8_t* data, int length){
		for(int i=0; i<length; i++) iostream->write(data[i]);
	}

	unsigned long time(){return millis();}

protected:
	long baud_;
	HardwareSerial* iostream;
};


#endif /* ARDUINOHARDWARE_H_ */
