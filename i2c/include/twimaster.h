/*****************************************************************************
 *
 * Atmel Corporation
 *
 * File              : USI_TWI_Master.h
 * Compiler          : IAR EWAAVR 2.28a/3.10a
 * Revision          : $Revision: 1.11 $
 * Date              : $Date: Tuesday, September 13, 2005 09:09:36 UTC $
 * Updated by        : $Author: jtyssoe $
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
 *                     the USI_TWI_Start_Transceiver_With_Data() function. If the transceiver
 *                     returns with a fail, then use USI_TWI_Get_Status_Info to evaluate the
 *                     cause of the failure.
 *
 ****************************************************************************/
#include <avr/io.h>
#include <util/twi.h>

//********** Defines **********//

// TWI STANDARD mode timing limits. SCL <= 100kHz, F_CPU = 1000000
#define T2_TWI    5 // >4,7us
#define T4_TWI    4 // >4,0us


/****************************************************************************
 Bit and byte definitions
 ****************************************************************************/
#define TWI_READ_BIT  0       // Bit position for R/W bit in "address byte".
#define TWI_ADR_BITS  1       // Bit position for LSB of the slave address bits in the init byte.
#define TWI_NACK_BIT  0       // Bit position for (N)ACK bit.

#define USI_TWI_NO_DATA             0x00  // Transmission buffer is empty
#define USI_TWI_DATA_OUT_OF_BOUND   0x01  // Transmission buffer is outside SRAM space
#define USI_TWI_UE_START_CON        0x02  // Unexpected Start Condition
#define USI_TWI_UE_STOP_CON         0x03  // Unexpected Stop Condition
#define USI_TWI_UE_DATA_COL         0x04  // Unexpected Data Collision (arbitration)
#define USI_TWI_NO_ACK_ON_DATA      0x05  // The slave did not acknowledge  all data
#define USI_TWI_NO_ACK_ON_ADDRESS   0x06  // The slave did not acknowledge  the address
#define USI_TWI_MISSING_START_CON   0x07  // Generated Start Condition not detected on bus
#define USI_TWI_MISSING_STOP_CON    0x08  // Generated Stop Condition not detected on bus

//#define SIGNAL_VERIFY 1
//#define PARAM_VERIFICATION 1
//#define NOISE_TESTING 1

#define MESSAGEBUF_SIZE       64

#if defined( __AVR_ATtiny261A__ ) | \
     defined( __AVR_ATtiny461A__ ) | \
     defined( __AVR_ATtiny861A__ )
#define DDR_USI             DDRA
#define PORT_USI            PORTA
#define PIN_USI             PINA
#define PORT_USI_SDA        PA0
#define PORT_USI_SCL        PA2
#define PIN_USI_SDA         PINA0
#define PIN_USI_SCL         PINA2
#define USI_START_COND_INT  USISIF
#define USI_START_VECTOR    USI_START_vect
#define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATtiny24__ ) | defined( __AVR_ATtiny44__ ) | defined( __AVR_ATtiny84__ ) | defined( __AVR_ATtiny84A__ )
#define DDR_USI             DDRA
#define PORT_USI            PORTA
#define PIN_USI             PINA
#define PORT_USI_SDA        PORTA6
#define PORT_USI_SCL        PORTA4
#define PIN_USI_SDA         PINA6
#define PIN_USI_SCL         PINA4
#define USI_START_COND_INT  USISIF
#define USI_START_VECTOR    USI_START_vect
#define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATtiny85__ )
#define DDR_USI             DDRB
#define PORT_USI            PORTB
#define PIN_USI             PINB
#define PORT_USI_SDA        PORTB0
#define PORT_USI_SCL        PORTB2
#define PIN_USI_SDA         PINB0
#define PIN_USI_SCL         PINB2
#define USI_START_COND_INT  USISIF
#define USI_START_VECTOR    USI_START_vect
#define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

// General defines
#define TRUE  1
#define FALSE 0

//********** Prototypes **********//

extern void USI_TWI_Master_Initialise(void);
extern unsigned char USI_TWI_Start_Transceiver_With_Data(unsigned char *,
		unsigned char);
extern unsigned char USI_TWI_Get_State_Info(void);
extern unsigned char send_data_reg(unsigned char SLAVE_ADDR, unsigned char reg, unsigned char value);
extern unsigned char receive_data_reg(unsigned char SLAVE_ADDR, unsigned char reg, uint8_t bytes, uint8_t buff[]);


extern unsigned char USI_TWI_Get_Write_Errors(void);
extern unsigned char USI_TWI_Get_Read_Errors(void);
extern unsigned char USI_TWI_Get_Write_Oks(void);
extern unsigned char USI_TWI_Get_Read_Oks(void);
extern void USI_TWI_ResetStats(void);
extern void *memset(void *s, int c, uint8_t n);


