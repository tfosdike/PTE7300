/*
  PTE7300_I2C.h - Public library for PTE7300 I2C interfacing.
  Created by M.H.W. Stopel, 02 September 2019.
  Last update: 10 Nov 2020, Updates with start() command for single mode.

  Copyright to Sensata Technologies
*/
#ifndef PTE7300_I2C_h
#define PTE7300_I2C_h

#include "Arduino.h"
#include "Wire.h"

class PTE7300_I2C
{
  public:
    // constructor
	PTE7300_I2C();
  PTE7300_I2C(TwoWire& bus);

	// low-level functions
	bool		  isConnected(); // check connectivity of device to the I2C-bus
	void		  CRC(bool tf);	// use CRC checking on datatransmission on/off (default true)

	// high-level functions
	uint32_t      readSERIAL();
	int16_t       readDSP_T();
	int16_t       readDSP_S();
	uint16_t	  readSTATUS();
	int           readADC_TC();
	void		  start();
	void 	      sleep();
	void 		  idle();
	void 		  reset();

  private:
    // class properties
	int _nodeAddress;
	bool _bUseCRC;
	TwoWire* _bus;

	// static functions
	unsigned int  readRegister(uint8_t address, unsigned int number, uint16_t *buffer);
	unsigned int  readRegisterNoCRC(uint8_t address, unsigned int number, uint16_t *buffer);
	unsigned int  readRegisterCRC(uint8_t address, unsigned int number, uint16_t *buffer);
	void          writeRegisterNoCRC(uint8_t address, unsigned int number, uint16_t *data);
	void 		  writeRegisterCRC(uint8_t address, unsigned int number, uint16_t *data);
	void          writeRegister(uint8_t address, unsigned int number, uint16_t *data);
	static char   calc_crc4(unsigned char polynom, unsigned char init, unsigned char* data, unsigned int len);
	static char   calc_crc8(unsigned char polynom, unsigned char init, unsigned char* data, unsigned int len);
};

#endif
