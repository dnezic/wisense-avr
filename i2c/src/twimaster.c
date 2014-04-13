/*****************************************************************************
 *
 * Atmel Corporation
 *
 * File              : USI_TWI_Master.c
 * Compiler          : AVRGCC Toolchain version 3.4.2
 * Revision          : $Revision: 992 $
 * Date              : $Date: 2013-11-07 $
 * Updated by        : $Author: Atmel $
 *
 * Support mail      : avr@atmel.com
 *
 * Supported devices : All device with USI module can be used.
 *                     The example is written for the ATmega169, ATtiny26 and ATtiny2313
 *
 * AppNote           : AVR310 - Using the USI module as a TWI Master
 *
 * Description       : This is an implementation of an TWI master using
 *                     the USI module as basis. The implementation assumes the AVR to
 *                     be the only TWI master in the system and can therefore not be
 *                     used in a multi-master system.
 * Usage             : Initialize the USI module by calling the USI_TWI_Master_Initialise()
 *                     function. Hence messages/data are transceived on the bus using
 *                     the USI_TWI_Transceive() function. The transceive function
 *                     returns a status byte, which can be used to evaluate the
 *                     success of the transmission.
 *
 ****************************************************************************/
#include <avr/io.h>
#include "twimaster.h"
#include <util/delay.h>
#include <avr/eeprom.h>


unsigned char USI_TWI_Master_Transfer(unsigned char);
unsigned char USI_TWI_Master_Stop(void);

//#define EEPROM_READ_ERROR 90
//#define EEPROM_WRITE_ERROR 91

unsigned int write_errors = 0;
unsigned int read_errors = 0;
unsigned int write_ok = 0;
unsigned int read_ok = 0;

union USI_TWI_state {
	unsigned char errorState; // Can reuse the TWI_state for error states due to that it will not be need if there exists an error.
	struct {
		unsigned char addressMode :1;
		unsigned char masterWriteDataMode :1;
		unsigned char unused :6;
	};
} USI_TWI_state;

/*---------------------------------------------------------------
 USI TWI single master initialization function
 ---------------------------------------------------------------*/
void USI_TWI_Master_Initialise(void) {

#if defined( __AVR_ATtiny261A__ ) | \
     defined( __AVR_ATtiny461A__ ) | \
     defined( __AVR_ATtiny861A__ )
		USIPP = (1 << USIPOS);
#endif

	PORT_USI |= (1 << PIN_USI_SDA); // Enable pullup on SDA, to set high as released state.
	PORT_USI |= (1 << PIN_USI_SCL); // Enable pullup on SCL, to set high as released state.

	DDR_USI |= (1 << PIN_USI_SCL);           // Enable SCL as output.
	DDR_USI |= (1 << PIN_USI_SDA);           // Enable SDA as output.

	USIDR = 0xFF;            // Preload dataregister with "released level" data.
	USICR = (0 << USISIE) | (0 << USIOIE) |              // Disable Interrupts.
			(1 << USIWM1) | (0 << USIWM0) |        // Set USI in Two-wire mode.
			(1 << USICS1) | (0 << USICS0) | (1 << USICLK) | // Software stobe as counter clock source
			(0 << USITC);
	USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | // Clear flags,
			(0x0 << USICNT0);                              // and reset counter.

	USI_TWI_state.errorState = 0;
}

/*---------------------------------------------------------------
 Use this function to get hold of the error message from the last transmission
 ---------------------------------------------------------------*/
unsigned char USI_TWI_Get_State_Info(void) {
	return (USI_TWI_state.errorState);                    // Return error state.
}

/*---------------------------------------------------------------
 Function for generating a TWI Start Condition.
 ---------------------------------------------------------------*/
unsigned char USI_TWI_Master_Start(void) {
	/* Release SCL to ensure that (repeated) Start can be performed */
	PORT_USI |= (1 << PIN_USI_SCL);                     // Release SCL.
	while (!(PORT_USI & (1 << PIN_USI_SCL)))
		;          // Verify that SCL becomes high.
	_delay_us(T2_TWI);

	/* Generate Start Condition */
	PORT_USI &= ~(1 << PIN_USI_SDA);                    // Force SDA LOW.
	_delay_us(T4_TWI);
	PORT_USI &= ~(1 << PIN_USI_SCL);                    // Pull SCL LOW.
	PORT_USI |= (1 << PIN_USI_SDA);                     // Release SDA.

#ifdef SIGNAL_VERIFY
	if (!(USISR & (1 << USISIF))) {
		USI_TWI_state.errorState = USI_TWI_MISSING_START_CON;
		return (FALSE);
	}
#endif
	return (TRUE);
}

/*---------------------------------------------------------------
 USI Transmit and receive function. LSB of first byte in data
 indicates if a read or write cycles is performed. If set a read
 operation is performed.

 Function generates (Repeated) Start Condition, sends address and
 R/W, Reads/Writes Data, and verifies/sends ACK.

 Success or error code is returned. Error codes are defined in
 USI_TWI_Master.h
 ---------------------------------------------------------------*/
unsigned char USI_TWI_Start_Transceiver_With_Data(unsigned char *msg,
		unsigned char msgSize) {
	unsigned char tempUSISR_8bit = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF)
			| (1 << USIDC) |     // Prepare register value to: Clear flags, and
			(0x0 << USICNT0); // set USI to shift 8 bits i.e. count 16 clock edges.
	unsigned char tempUSISR_1bit = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF)
			| (1 << USIDC) |     // Prepare register value to: Clear flags, and
			(0xE << USICNT0); // set USI to shift 1 bit i.e. count 2 clock edges.

	USI_TWI_state.errorState = 0;
	USI_TWI_state.addressMode = TRUE;

#ifdef PARAM_VERIFICATION
	if(msg > (unsigned char*)RAMEND) // Test if address is outside SRAM space
	{
		USI_TWI_state.errorState = USI_TWI_DATA_OUT_OF_BOUND;
		return (FALSE);
	}
	if(msgSize <= 1)            // Test if the transmission buffer is empty
	{
		USI_TWI_state.errorState = USI_TWI_NO_DATA;
		return (FALSE);
	}
#endif


#ifdef NOISE_TESTING                                // Test if any unexpected conditions have arrived prior to this execution.
	if( USISR & (1<<USISIF) )
	{
		USI_TWI_state.errorState = USI_TWI_UE_START_CON;
		return (FALSE);
	}
	if( USISR & (1<<USIPF) )
	{
		USI_TWI_state.errorState = USI_TWI_UE_STOP_CON;
		return (FALSE);
	}

	if( USISR & (1<<USIDC) )
	{
		USI_TWI_state.errorState = USI_TWI_UE_DATA_COL;
		return (FALSE);
	}


#endif

	if (!(*msg & (1 << TWI_READ_BIT))) // The LSB in the address byte determines if is a masterRead or masterWrite operation.
	{
		USI_TWI_state.masterWriteDataMode = TRUE;
	}

	/* added */
	if (!USI_TWI_Master_Start()) {
		return (FALSE);                // Send a START condition on the TWI bus.
	}

	/*Write address and Read/Write data */


	do {
		/* If masterWrite cycle (or inital address tranmission)*/
		if (USI_TWI_state.addressMode || USI_TWI_state.masterWriteDataMode) {
			/* Write a byte */
			PORT_USI &= ~(1 << PIN_USI_SCL);                // Pull SCL LOW.
			USIDR = *(msg++);                        // Setup data.
			USI_TWI_Master_Transfer(tempUSISR_8bit);    // Send 8 bits on bus.

			/* Clock and verify (N)ACK from slave */
			DDR_USI &= ~(1 << PIN_USI_SDA);              // Enable SDA as input.
			if (USI_TWI_Master_Transfer(tempUSISR_1bit) & (1 << TWI_NACK_BIT)) {
				if (USI_TWI_state.addressMode)
					USI_TWI_state.errorState = USI_TWI_NO_ACK_ON_ADDRESS;
				else
					USI_TWI_state.errorState = USI_TWI_NO_ACK_ON_DATA;
				return (FALSE);
			}
			USI_TWI_state.addressMode = FALSE; // Only perform address transmission once.
		}
		/* Else masterRead cycle*/
		else {
			/* Read a data byte */
			DDR_USI &= ~(1 << PIN_USI_SDA);              // Enable SDA as input.
			*(msg++) = USI_TWI_Master_Transfer(tempUSISR_8bit);

			/* Prepare to generate ACK (or NACK in case of End Of Transmission) */
			if (msgSize == 1)     // If transmission of last byte was performed.
					{
				USIDR = 0xFF;       // Load NACK to confirm End Of Transmission.
			} else {
				USIDR = 0x00; // Load ACK. Set data register bit 7 (output for SDA) low.
			}
			USI_TWI_Master_Transfer(tempUSISR_1bit);   // Generate ACK/NACK.
		}
	} while (--msgSize);                        // Until all data sent/received.

	USI_TWI_Master_Stop();              // Send a STOP condition on the TWI bus.

	if(USI_TWI_state.masterWriteDataMode == TRUE) {
		write_ok = write_ok + 1;
	}
	else
	{
		read_ok = read_ok + 1;
	}

	/* Transmission successfully completed*/
	return (TRUE);
}

/*---------------------------------------------------------------
 Core function for shifting data in and out from the USI.
 Data to be sent has to be placed into the USIDR prior to calling
 this function. Data read, will be return'ed from the function.
 ---------------------------------------------------------------*/
unsigned char USI_TWI_Master_Transfer(unsigned char temp) {
	USISR = temp;                                // Set USISR according to temp.
												 // Prepare clocking.
	temp = (0 << USISIE) | (0 << USIOIE) |               // Interrupts disabled
			(1 << USIWM1) | (0 << USIWM0) |        // Set USI in Two-wire mode.
			(1 << USICS1) | (0 << USICS0) | (1 << USICLK) | // Software clock strobe as source.
			(1 << USITC);                              // Toggle Clock Port.
	do {
		_delay_us( T2_TWI);
		USICR = temp;                          // Generate positve SCL edge.
		while (!(PIN_USI & (1 << PIN_USI_SCL)))
			;                          // Wait for SCL to go high.
		_delay_us( T4_TWI);
		USICR = temp;                          // Generate negative SCL edge.
	} while (!(USISR & (1 << USIOIF)));        // Check for transfer complete.


	_delay_us( T2_TWI);
	temp = USIDR;                           // Read out data.
	USIDR = 0xFF;                            // Release SDA.
	DDR_USI |= (1 << PIN_USI_SDA);             // Enable SDA as output.

	return temp;                             // Return the data from the USIDR
}

/*---------------------------------------------------------------
 Function for generating a TWI Stop Condition. Used to release
 the TWI bus.
 ---------------------------------------------------------------*/
unsigned char USI_TWI_Master_Stop(void) {
	PORT_USI &= ~(1 << PIN_USI_SDA);           // Pull SDA low.
	PORT_USI |= (1 << PIN_USI_SCL);            // Release SCL.
	while (!(PIN_USI & (1 << PIN_USI_SCL)))
		;  // Wait for SCL to go high.
	_delay_us( T4_TWI);
	PORT_USI |= (1 << PIN_USI_SDA);            // Release SDA.
	_delay_us( T2_TWI);

#ifdef SIGNAL_VERIFY
	if (!(USISR & (1 << USIPF))) {
		USI_TWI_state.errorState = USI_TWI_MISSING_STOP_CON;
		return (FALSE);
	}
#endif

	return (TRUE);
}

//unsigned char send_general_call(unsigned char* messageBuf) {
//	messageBuf[0] = TWI_GEN_CALL; // The first byte must always consit of General Call code or the TWI slave address.
//	messageBuf[1] = 0xAA; // The command or data to be included in the general call.
//	return USI_TWI_Start_Transceiver_With_Data(messageBuf, 2);
//}

unsigned char send_data(unsigned char SLAVE_ADDR, unsigned char data) {
	unsigned char messageBuf[MESSAGEBUF_SIZE];
	messageBuf[0] = (SLAVE_ADDR << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT); // The first byte must always consit of General Call code or the TWI slave address.
	messageBuf[1] = data; // The first byte is used for commands.
	//messageBuf[2] = data; // The second byte is used for the data.

	unsigned char temp;

	temp = USI_TWI_Start_Transceiver_With_Data(messageBuf, 2);

	if (temp == FALSE) {
//			uint8_t error_counter = 0;
//			error_counter = eeprom_read_byte((uint8_t *) EEPROM_WRITE_ERROR);
//			error_counter = error_counter + 1;
			//eeprom_update_byte((uint8_t *) EEPROM_WRITE_ERROR, USI_TWI_state.errorState);
		}

	return temp;
}

unsigned char send_data_pair(unsigned char SLAVE_ADDR, unsigned char reg, unsigned char value) {
	unsigned char messageBuf[MESSAGEBUF_SIZE];
		messageBuf[0] = (SLAVE_ADDR << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT); // The first byte must always consit of General Call code or the TWI slave address.
		messageBuf[1] = reg; // The first byte is used for commands.
		messageBuf[2] = value; // The second byte is used for the data.

		unsigned char temp;

		temp = USI_TWI_Start_Transceiver_With_Data(messageBuf, 3);

		if (temp == FALSE) {
	//			uint8_t error_counter = 0;
	//			error_counter = eeprom_read_byte((uint8_t *) EEPROM_WRITE_ERROR);
	//			error_counter = error_counter + 1;
				//eeprom_update_byte((uint8_t *) EEPROM_WRITE_ERROR, USI_TWI_state.errorState);
				write_errors = write_errors + 1;
			}

		return temp;
}

unsigned char send_data_no(unsigned char SLAVE_ADDR) {
	unsigned char messageBuf[MESSAGEBUF_SIZE];
	messageBuf[0] = (SLAVE_ADDR << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT); // The first byte must always consit of General Call code or the TWI slave address.
	//messageBuf[1] = data; // The first byte is used for commands.
	//messageBuf[2] = data; // The second byte is used for the data.
	return USI_TWI_Start_Transceiver_With_Data(messageBuf, 1);
}

/*
 unsigned char send_for_receive(unsigned char SLAVE_ADDR) {
 unsigned char messageBuf[MESSAGEBUF_SIZE];
 // Send a Address Call, sending a request, followed by a receive
 // Send the request-for-data command to the Slave
 unsigned char temp;
 messageBuf[0] = (SLAVE_ADDR << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT); // The first byte must always consit of General Call code or the TWI slave address.
 //messageBuf[1] = TWI_CMD_MASTER_READ; // The first byte is used for commands.
 temp = USI_TWI_Start_Transceiver_With_Data(messageBuf, 1);
 return temp;
 }*/

unsigned char receive_byte(unsigned char SLAVE_ADDR) {
	unsigned char messageBuf[MESSAGEBUF_SIZE];
	unsigned char temp;
	unsigned char byte;
	// Transmit request and get the received data from the transceiver buffer
	messageBuf[0] = (SLAVE_ADDR << TWI_ADR_BITS) | (TRUE << TWI_READ_BIT); // The first byte must always consit of General Call code or the TWI slave address.
	messageBuf[1] = 0; // The first byte is used for commands.

	do
		temp = USI_TWI_Start_Transceiver_With_Data(messageBuf, 2);
	while (USI_TWI_Get_State_Info() == USI_TWI_NO_ACK_ON_ADDRESS);

	if (temp == FALSE) {
//		uint8_t error_counter = 0;
//		error_counter = eeprom_read_byte((uint8_t *) EEPROM_READ_ERROR);
//		error_counter = error_counter + 1;
		//eeprom_update_byte((uint8_t *) EEPROM_READ_ERROR, USI_TWI_state.errorState);
	}

	byte = messageBuf[1];        // Store data on PORTB.
	return byte;
}


unsigned char receive_bytes(unsigned char SLAVE_ADDR, uint8_t bytes, uint8_t buff[]) {
	unsigned char messageBuf[MESSAGEBUF_SIZE];
	unsigned char temp;
	unsigned char byte;
	// Transmit request and get the received data from the transceiver buffer
	messageBuf[0] = (SLAVE_ADDR << TWI_ADR_BITS) | (TRUE << TWI_READ_BIT); // The first byte must always consit of General Call code or the TWI slave address.

	do
		temp = USI_TWI_Start_Transceiver_With_Data(messageBuf, bytes+1);
	while (USI_TWI_Get_State_Info() == USI_TWI_NO_ACK_ON_ADDRESS);

	if (temp == FALSE) {
//		uint8_t error_counter = 0;
//		error_counter = eeprom_read_byte((uint8_t *) EEPROM_READ_ERROR);
//		error_counter = error_counter + 1;
		read_errors = read_errors  + 1;
		//eeprom_update_byte((uint8_t *) EEPROM_READ_ERROR, USI_TWI_state.errorState);
	}

	for(uint8_t i = 0; i < bytes; i++) {
		buff[i] = messageBuf[i + 1];
	}


	return temp;
}


unsigned char receive_bytes_pair(unsigned char SLAVE_ADDR, uint8_t reg, uint8_t bytes, uint8_t buff[]) {
	unsigned char messageBuf[MESSAGEBUF_SIZE];
	unsigned char temp;
	unsigned char byte;

	messageBuf[0] = (SLAVE_ADDR << TWI_ADR_BITS) | (FALSE << TWI_READ_BIT); // The first byte must always consit of General Call code or the TWI slave address.
	messageBuf[1] = reg; // The first byte is used for commands.

	temp = USI_TWI_Start_Transceiver_With_Data(messageBuf, 2);

	//_delay_ms(15);

	// Transmit request and get the received data from the transceiver buffer
	messageBuf[0] = (SLAVE_ADDR << TWI_ADR_BITS) | (TRUE << TWI_READ_BIT); // The first byte must always consit of General Call code or the TWI slave address.

	do
		temp = USI_TWI_Start_Transceiver_With_Data(messageBuf, bytes+1);
	while (USI_TWI_Get_State_Info() == USI_TWI_NO_ACK_ON_ADDRESS);

	if (temp == FALSE) {
//		uint8_t error_counter = 0;
//		error_counter = eeprom_read_byte((uint8_t *) EEPROM_READ_ERROR);
//		error_counter = error_counter + 1;
		read_errors = read_errors  + 1;
		//eeprom_update_byte((uint8_t *) EEPROM_READ_ERROR, USI_TWI_state.errorState);
	}

	for(uint8_t i = 0; i < bytes; i++) {
		buff[i] = messageBuf[i + 1];
	}


	return temp;
}

unsigned char USI_TWI_Get_Write_Errors(void) {
	return write_errors;
}
unsigned char USI_TWI_Get_Read_Errors(void) {
	return read_errors;
}

unsigned char USI_TWI_Get_Write_Oks(void) {
	return write_ok;
}
unsigned char USI_TWI_Get_Read_Oks(void) {
	return read_ok;
}

void USI_TWI_ResetStats(void) {
	write_errors = 0;
	write_ok = 0;
	read_errors = 0;
	read_ok = 0;
}
