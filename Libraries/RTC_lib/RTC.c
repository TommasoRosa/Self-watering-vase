
#ifndef __AVR_ATmega328P__ 
	#define __AVR_ATmega328P__
#endif
#include <avr/io.h>
#include <stdio.h>
//#include <util/delay.h>
//#include <avr/iom328p.h>
#include <avr/pgmspace.h>


#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <inttypes.h>
#define bool int8_t
#define false 0
#define true 1

//#define ATMEGA8

#ifndef CPU_FREQ
#define CPU_FREQ 8000000L
#endif

#ifndef TWI_FREQ
#define TWI_FREQ 100000L
#endif

#ifndef TWI_BUFFER_LENGTH
#define TWI_BUFFER_LENGTH 32
#endif

#define TWI_READY 0
#define TWI_MRX   1
#define TWI_MTX   2
#define TWI_SRX   3
#define TWI_STX   4

void twi_init(void);
void twi_setAddress(uint8_t);
uint8_t twi_readFrom(uint8_t, uint8_t*, uint8_t);
uint8_t twi_writeTo(uint8_t, uint8_t*, uint8_t, uint8_t);
uint8_t twi_transmit(uint8_t*, uint8_t);
void twi_attachSlaveRxEvent( void (*)(uint8_t*, int) );
void twi_attachSlaveTxEvent( void (*)(void) );
void twi_reply(uint8_t);
void twi_stop(void);
void twi_releaseBus(void);

#define BUFFER_LENGTH 32

void twi_init_master(void);
void twi_init_slave(uint8_t);
void twi_begin_transmission(uint8_t);
uint8_t twi_end_transmission(void);
uint8_t twi_request_from(uint8_t, uint8_t);
void twi_send_byte(uint8_t);
void twi_send(uint8_t*, uint8_t);
void twi_send_char(char*);
uint8_t twi_available(void);
uint8_t twi_receive(void);
void twi_set_on_receive( void (*)(int) );
void twi_set_on_request( void (*)(void) );

struct tm {
	int sec;      // 0 to 59
	int min;      // 0 to 59
	int hour;     // 0 to 23
	int mday;     // 1 to 31
	int mon;      // 1 to 12
	int year;     // year-99
	int wday;     // 1-7

    // 12-hour clock data
    bool am; // true for AM, false for PM
    int twelveHour; // 12 hour clock time
}tm_type;

// statically allocated
extern struct tm _tm;

// Initialize the RTC and autodetect type (DS1307 or DS3231)
void rtc_init(void);

// Autodetection
bool rtc_is_ds1307(void);
bool rtc_is_ds3231(void);

void rtc_set_ds1307(void);
void rtc_set_ds3231(void);

// Get/set time
// Gets the time: Supports both 24-hour and 12-hour mode
struct tm* rtc_get_time(void);
// Gets the time: 24-hour mode only
void rtc_get_time_s(uint8_t* hour, uint8_t* min, uint8_t* sec);
// Sets the time: Supports both 24-hour and 12-hour mode
void rtc_set_time(struct tm* tm_);
// Sets the time: Supports 12-hour mode only
void rtc_set_time_s(uint8_t hour, uint8_t min, uint8_t sec);

// start/stop clock running (DS1307 only)
void rtc_run_clock(bool run);
bool rtc_is_clock_running(void);

// Read Temperature (DS3231 only)
void  ds3231_get_temp_int(int8_t* i, uint8_t* f);
void rtc_force_temp_conversion(uint8_t block);

// SRAM read/write DS1307 only
void rtc_get_sram(uint8_t* data);
void rtc_set_sram(uint8_t *data);
uint8_t rtc_get_sram_byte(uint8_t offset);
void rtc_set_sram_byte(uint8_t b, uint8_t offset);

  // Auxillary functions
enum RTC_SQW_FREQ { FREQ_1 = 0, FREQ_1024, FREQ_4096, FREQ_8192 };

void rtc_SQW_enable(bool enable);
void rtc_SQW_set_freq(enum RTC_SQW_FREQ freq);
void rtc_osc32kHz_enable(bool enable);

// Alarm functionality
void rtc_reset_alarm(void);
void rtc_set_alarm(struct tm* tm_);
void rtc_set_alarm_s(uint8_t hour, uint8_t min, uint8_t sec);
struct tm* rtc_get_alarm(void);
void rtc_get_alarm_s(uint8_t* hour, uint8_t* min, uint8_t* sec);
bool rtc_check_alarm(void);


#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include <compat/twi.h>

static volatile uint8_t twi_state;
static uint8_t twi_slarw;

static void (*twi_onSlaveTransmit)(void);
static void (*twi_onSlaveReceive)(uint8_t*, int);

static uint8_t twi_masterBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_masterBufferIndex;
static uint8_t twi_masterBufferLength;

static uint8_t twi_txBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_txBufferIndex;
static volatile uint8_t twi_txBufferLength;

static uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_rxBufferIndex;

static volatile uint8_t twi_error;

/*
 * Function twi_init
 * Desc     readys twi pins and sets twi bitrate
 * Input    none
 * Output   none
 */
void twi_init(void)
{
  // initialize state
  twi_state = TWI_READY;


    // activate internal pull-ups for twi
    // as per note from atmega8 manual pg167
    sbi(PORTC, 4);
    sbi(PORTC, 5);


  // initialize twi prescaler and bit rate
  cbi(TWSR, TWPS0);
  cbi(TWSR, TWPS1);
  TWBR = ((CPU_FREQ / TWI_FREQ) - 16) / 2;

  /* twi bit rate formula from atmega128 manual pg 204
  SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
  note: TWBR should be 10 or higher for master mode
  It is 72 for a 16mhz Wiring board with 100kHz TWI */

  // enable twi module, acks, and twi interrupt
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}

/*
 * Function twi_slaveInit
 * Desc     sets slave address and enables interrupt
 * Input    none
 * Output   none
 */
void twi_setAddress(uint8_t address)
{
  // set twi slave address (skip over TWGCE bit)
  TWAR = address << 1;
}

/*
 * Function twi_readFrom
 * Desc     attempts to become twi bus master and read a
 *          series of bytes from a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes to read into array
 * Output   number of bytes read
 */
uint8_t twi_readFrom(uint8_t address, uint8_t* data, uint8_t length)
{
  uint8_t i;

  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length){
    return 0;
  }

  // wait until twi is ready, become master receiver
  while(TWI_READY != twi_state){
    continue;
  }
  twi_state = TWI_MRX;
  // reset error state (0xFF.. no error occured)
  twi_error = 0xFF;

  // initialize buffer iteration vars
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length-1;  // This is not intuitive, read on...
  // On receive, the previously configured ACK/NACK setting is transmitted in
  // response to the received byte before the interrupt is signalled.
  // Therefor we must actually set NACK when the _next_ to last byte is
  // received, causing that NACK to be sent in response to receiving the last
  // expected byte of data.

  // build sla+w, slave device address + w bit
  twi_slarw = TW_READ;
  twi_slarw |= address << 1;

  // send start condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);

  // wait for read operation to complete
  while(TWI_MRX == twi_state){
    continue;
  }

  if (twi_masterBufferIndex < length)
    length = twi_masterBufferIndex;

  // copy twi buffer to data
  for(i = 0; i < length; ++i){
    data[i] = twi_masterBuffer[i];
  }

  return length;
}

/*
 * Function twi_writeTo
 * Desc     attempts to become twi bus master and write a
 *          series of bytes to a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes in array
 *          wait: boolean indicating to wait for write or not
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
uint8_t twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait)
{
  uint8_t i;

  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length){
    return 1;
  }

  // wait until twi is ready, become master transmitter
  while(TWI_READY != twi_state){
    continue;
  }
  twi_state = TWI_MTX;
  // reset error state (0xFF.. no error occured)
  twi_error = 0xFF;

  // initialize buffer iteration vars
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length;

  // copy data to twi buffer
  for(i = 0; i < length; ++i){
    twi_masterBuffer[i] = data[i];
  }

  // build sla+w, slave device address + w bit
  twi_slarw = TW_WRITE;
  twi_slarw |= address << 1;

  // send start condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);

  // wait for write operation to complete
  while(wait && (TWI_MTX == twi_state)){
    continue;
  }

  if (twi_error == 0xFF)
    return 0;	// success
  else if (twi_error == TW_MT_SLA_NACK)
    return 2;	// error: address send, nack received
  else if (twi_error == TW_MT_DATA_NACK)
    return 3;	// error: data send, nack received
  else
    return 4;	// other twi error
}

/*
 * Function twi_transmit
 * Desc     fills slave tx buffer with data
 *          must be called in slave tx event callback
 * Input    data: pointer to byte array
 *          length: number of bytes in array
 * Output   1 length too long for buffer
 *          2 not slave transmitter
 *          0 ok
 */
uint8_t twi_transmit(uint8_t* data, uint8_t length)
{
  uint8_t i;

  // ensure data will fit into buffer
  if(TWI_BUFFER_LENGTH < length){
    return 1;
  }

  // ensure we are currently a slave transmitter
  if(TWI_STX != twi_state){
    return 2;
  }

  // set length and copy data into tx buffer
  twi_txBufferLength = length;
  for(i = 0; i < length; ++i){
    twi_txBuffer[i] = data[i];
  }

  return 0;
}

/*
 * Function twi_attachSlaveRxEvent
 * Desc     sets function called before a slave read operation
 * Input    function: callback function to use
 * Output   none
 */
void twi_attachSlaveRxEvent( void (*function)(uint8_t*, int) )
{
  twi_onSlaveReceive = function;
}

/*
 * Function twi_attachSlaveTxEvent
 * Desc     sets function called before a slave write operation
 * Input    function: callback function to use
 * Output   none
 */
void twi_attachSlaveTxEvent( void (*function)(void) )
{
  twi_onSlaveTransmit = function;
}

/*
 * Function twi_reply
 * Desc     sends byte or readys receive line
 * Input    ack: byte indicating to ack or to nack
 * Output   none
 */
void twi_reply(uint8_t ack)
{
  // transmit master read ready signal, with or without ack
  if(ack){
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
  }else{
	  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
  }
}

/*
 * Function twi_stop
 * Desc     relinquishes bus master status
 * Input    none
 * Output   none
 */
void twi_stop(void)
{
  // send stop condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

  // wait for stop condition to be exectued on bus
  // TWINT is not set after a stop condition!
  while(TWCR & _BV(TWSTO)){
    continue;
  }

  // update twi state
  twi_state = TWI_READY;
}

/*
 * Function twi_releaseBus
 * Desc     releases bus control
 * Input    none
 * Output   none
 */
void twi_releaseBus(void)
{
  // release bus
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);

  // update twi state
  twi_state = TWI_READY;
}

SIGNAL(TWI_vect)
{
  switch(TW_STATUS){
    // All Master
    case TW_START:     // sent start condition
    case TW_REP_START: // sent repeated start condition
      // copy device address and r/w bit to output register and ack
      TWDR = twi_slarw;
      twi_reply(1);
      break;

    // Master Transmitter
    case TW_MT_SLA_ACK:  // slave receiver acked address
    case TW_MT_DATA_ACK: // slave receiver acked data
      // if there is data to send, send it, otherwise stop
      if(twi_masterBufferIndex < twi_masterBufferLength){
        // copy data to output register and ack
        TWDR = twi_masterBuffer[twi_masterBufferIndex++];
        twi_reply(1);
      }else{
        twi_stop();
      }
      break;
    case TW_MT_SLA_NACK:  // address sent, nack received
      twi_error = TW_MT_SLA_NACK;
      twi_stop();
      break;
    case TW_MT_DATA_NACK: // data sent, nack received
      twi_error = TW_MT_DATA_NACK;
      twi_stop();
      break;
    case TW_MT_ARB_LOST: // lost bus arbitration
      twi_error = TW_MT_ARB_LOST;
      twi_releaseBus();
      break;

    // Master Receiver
    case TW_MR_DATA_ACK: // data received, ack sent
      // put byte into buffer
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR;
    case TW_MR_SLA_ACK:  // address sent, ack received
      // ack if more bytes are expected, otherwise nack
      if(twi_masterBufferIndex < twi_masterBufferLength){
        twi_reply(1);
      }else{
        twi_reply(0);
      }
      break;
    case TW_MR_DATA_NACK: // data received, nack sent
      // put final byte into buffer
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR;
    case TW_MR_SLA_NACK: // address sent, nack received
      twi_stop();
      break;
    // TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case

    // Slave Receiver
    case TW_SR_SLA_ACK:   // addressed, returned ack
    case TW_SR_GCALL_ACK: // addressed generally, returned ack
    case TW_SR_ARB_LOST_SLA_ACK:   // lost arbitration, returned ack
    case TW_SR_ARB_LOST_GCALL_ACK: // lost arbitration, returned ack
      // enter slave receiver mode
      twi_state = TWI_SRX;
      // indicate that rx buffer can be overwritten and ack
      twi_rxBufferIndex = 0;
      twi_reply(1);
      break;
    case TW_SR_DATA_ACK:       // data received, returned ack
    case TW_SR_GCALL_DATA_ACK: // data received generally, returned ack
      // if there is still room in the rx buffer
      if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
        // put byte in buffer and ack
        twi_rxBuffer[twi_rxBufferIndex++] = TWDR;
        twi_reply(1);
      }else{
        // otherwise nack
        twi_reply(0);
      }
      break;
    case TW_SR_STOP: // stop or repeated start condition received
      // put a null char after data if there's room
      if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
        twi_rxBuffer[twi_rxBufferIndex] = '\0';
      }
      // sends ack and stops interface for clock stretching
      twi_stop();
      // callback to user defined callback
      twi_onSlaveReceive(twi_rxBuffer, twi_rxBufferIndex);
      // since we submit rx buffer to "wire" library, we can reset it
      twi_rxBufferIndex = 0;
      // ack future responses and leave slave receiver state
      twi_releaseBus();
      break;
    case TW_SR_DATA_NACK:       // data received, returned nack
    case TW_SR_GCALL_DATA_NACK: // data received generally, returned nack
      // nack back at master
      twi_reply(0);
      break;

    // Slave Transmitter
    case TW_ST_SLA_ACK:          // addressed, returned ack
    case TW_ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
      // enter slave transmitter mode
      twi_state = TWI_STX;
      // ready the tx buffer index for iteration
      twi_txBufferIndex = 0;
      // set tx buffer length to be zero, to verify if user changes it
      twi_txBufferLength = 0;
      // request for txBuffer to be filled and length to be set
      // note: user must call twi_transmit(bytes, length) to do this
      twi_onSlaveTransmit();
      // if they didn't change buffer & length, initialize it
      if(0 == twi_txBufferLength){
        twi_txBufferLength = 1;
        twi_txBuffer[0] = 0x00;
      }
      // transmit first byte from buffer, fall
    case TW_ST_DATA_ACK: // byte sent, ack returned
      // copy data to output register
      TWDR = twi_txBuffer[twi_txBufferIndex++];
      // if there is more to send, ack, otherwise nack
      if(twi_txBufferIndex < twi_txBufferLength){
        twi_reply(1);
      }else{
        twi_reply(0);
      }
      break;
    case TW_ST_DATA_NACK: // received nack, we are done
    case TW_ST_LAST_DATA: // received ack, but we are done already!
      // ack future responses
      twi_reply(1);
      // leave slave receiver state
      twi_state = TWI_READY;
      break;

    // All
    case TW_NO_INFO:   // no state information
      break;
    case TW_BUS_ERROR: // bus error, illegal stop/start
      twi_error = TW_BUS_ERROR;
      twi_stop();
      break;
  }
}

uint8_t rxBuffer[BUFFER_LENGTH];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

uint8_t txAddress = 0;
uint8_t txBuffer[BUFFER_LENGTH];
uint8_t txBufferIndex = 0;
uint8_t txBufferLength = 0;

uint8_t transmitting = 0;
void (*user_onRequest)(void);
void (*user_onReceive)(int);

void onRequestService(void);
void onReceiveService(uint8_t*, int);

void twi_init_master(void)
{
  rxBufferIndex = 0;
  rxBufferLength = 0;

  txBufferIndex = 0;
  txBufferLength = 0;

  twi_init();
}

void twi_init_slave(uint8_t address)
{
  twi_setAddress(address);
  twi_attachSlaveTxEvent(onRequestService);
  twi_attachSlaveRxEvent(onReceiveService);
  twi_init_master();
}

uint8_t twi_request_from(uint8_t address, uint8_t quantity)
{
  // clamp to buffer length
  if(quantity > BUFFER_LENGTH){
    quantity = BUFFER_LENGTH;
  }
  // perform blocking read into buffer
  uint8_t read = twi_readFrom(address, rxBuffer, quantity);
  // set rx buffer iterator vars
  rxBufferIndex = 0;
  rxBufferLength = read;

  return read;
}

void twi_begin_transmission(uint8_t address)
{
  // indicate that we are transmitting
  transmitting = 1;
  // set address of targeted slave
  txAddress = address;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
}

uint8_t twi_end_transmission(void)
{
  // transmit buffer (blocking)
  int8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 1);
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
  // indicate that we are done transmitting
  transmitting = 0;
  return ret;
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
void twi_send_byte(uint8_t data)
{
  if(transmitting){
  // in master transmitter mode
    // don't bother if buffer is full
    if(txBufferLength >= BUFFER_LENGTH){
      return;
    }
    // put byte in tx buffer
    txBuffer[txBufferIndex] = data;
    ++txBufferIndex;
    // update amount in buffer
    txBufferLength = txBufferIndex;
  }else{
  // in slave send mode
    // reply to master
    twi_transmit(&data, 1);
  }
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
void twi_send(uint8_t* data, uint8_t quantity)
{
  if(transmitting){
  // in master transmitter mode
    for(uint8_t i = 0; i < quantity; ++i){
     twi_send_byte(data[i]);
    }
  }else{
  // in slave send mode
    // reply to master
    twi_transmit(data, quantity);
  }
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
void twi_send_char(char* data)
{
  twi_send((uint8_t*)data, strlen(data));
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
uint8_t twi_available(void)
{
  return rxBufferLength - rxBufferIndex;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
uint8_t twi_receive(void)
{
  // default to returning null char
  // for people using with char strings
  uint8_t value = '\0';

  // get each successive byte on each call
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }

  return value;
}

// behind the scenes function that is called when data is received
void onReceiveService(uint8_t* inBytes, int numBytes)
{
  // don't bother if user hasn't registered a callback
  if(!user_onReceive){
    return;
  }
  // don't bother if rx buffer is in use by a master requestFrom() op
  // i know this drops data, but it allows for slight stupidity
  // meaning, they may not have read all the master requestFrom() data yet
  if(rxBufferIndex < rxBufferLength){
    return;
  }
  // copy twi rx buffer into local read buffer
  // this enables new reads to happen in parallel
  for(uint8_t i = 0; i < numBytes; ++i){
    rxBuffer[i] = inBytes[i];
  }
  // set rx iterator vars
  rxBufferIndex = 0;
  rxBufferLength = numBytes;
  // alert user program
  user_onReceive(numBytes);
}

// behind the scenes function that is called when data is requested
void onRequestService(void)
{
  // don't bother if user hasn't registered a callback
  if(!user_onRequest){
    return;
  }
  // reset tx buffer iterator vars
  // !!! this will kill any pending pre-master sendTo() activity
  txBufferIndex = 0;
  txBufferLength = 0;
  // alert user program
  user_onRequest();
}

// sets function called on slave write
void twi_set_on_receive( void (*function)(int) )
{
  user_onReceive = function;
}

// sets function called on slave read
void twi_set_on_request( void (*function)(void) )
{
  user_onRequest = function;
}


#define RTC_ADDR 0x68 // I2C address
#define CH_BIT 7 // clock halt bit

// statically allocated structure for time value
struct tm _tm;

uint8_t dec2bcd(uint8_t d)
{
  return ((d/10 * 16) + (d % 10));
}

uint8_t bcd2dec(uint8_t b)
{
  return ((b/16 * 10) + (b % 16));
}

uint8_t rtc_read_byte(uint8_t offset)
{
	twi_begin_transmission(RTC_ADDR);
	twi_send_byte(offset);
	twi_end_transmission();

	twi_request_from(RTC_ADDR, 1);
	return twi_receive();
}

void rtc_write_byte(uint8_t b, uint8_t offset)
{
	twi_begin_transmission(RTC_ADDR);
	twi_send_byte(offset);
	twi_send_byte(b);
	twi_end_transmission();
}

static bool s_is_ds1307 = false;
static bool s_is_ds3231 = false;

void rtc_init(void)
{
	// Attempt autodetection:
	// 1) Read and save temperature register
	// 2) Write a value to temperature register
	// 3) Read back the value
	//   equal to the one written: DS1307, write back saved value and return
	//   different from written:   DS3231

	uint8_t temp1 = rtc_read_byte(0x11);
	uint8_t temp2 = rtc_read_byte(0x12);

	rtc_write_byte(0xee, 0x11);
	rtc_write_byte(0xdd, 0x12);

	if (rtc_read_byte(0x11) == 0xee && rtc_read_byte(0x12) == 0xdd) {
		s_is_ds1307 = true;
		// restore values
		rtc_write_byte(temp1, 0x11);
		rtc_write_byte(temp2, 0x12);
	}
	else {
		s_is_ds3231 = true;
	}
}

// Autodetection
bool rtc_is_ds1307(void) { return s_is_ds1307; }
bool rtc_is_ds3231(void) { return s_is_ds3231; }

// Autodetection override
void rtc_set_ds1307(void) { s_is_ds1307 = true;   s_is_ds3231 = false; }
void rtc_set_ds3231(void) { s_is_ds1307 = false;  s_is_ds3231 = true;  }

struct tm* rtc_get_time(void)
{
	uint8_t rtc[9];
	uint8_t century = 0;

	// read 7 bytes starting from register 0
	// sec, min, hour, day-of-week, date, month, year
	twi_begin_transmission(RTC_ADDR);
	twi_send_byte(0x0);
	twi_end_transmission();

	twi_request_from(RTC_ADDR, 7);

	for (uint8_t i = 0; i < 7; i++) {
		rtc[i] = twi_receive();
	}

	twi_end_transmission();

	// Clear clock halt bit from read data
	// This starts the clock for a DS1307, and has no effect for a DS3231
	rtc[0] &= ~(_BV(CH_BIT)); // clear bit

	_tm.sec = bcd2dec(rtc[0]);
	_tm.min = bcd2dec(rtc[1]);
	_tm.hour = bcd2dec(rtc[2]);
	_tm.mday = bcd2dec(rtc[4]);
	_tm.mon = bcd2dec(rtc[5] & 0x1F); // returns 1-12
	century = (rtc[5] & 0x80) >> 7;
	_tm.year = century == 1 ? 2000 + bcd2dec(rtc[6]) : 1900 + bcd2dec(rtc[6]); // year 0-99
	_tm.wday = bcd2dec(rtc[3]); // returns 1-7

	if (_tm.hour == 0) {
		_tm.twelveHour = 0;
		_tm.am = 1;
	} else if (_tm.hour < 12) {
		_tm.twelveHour = _tm.hour;
		_tm.am = 1;
	} else {
		_tm.twelveHour = _tm.hour - 12;
		_tm.am = 0;
	}

	return &_tm;
}

void rtc_get_time_s(uint8_t* hour, uint8_t* min, uint8_t* sec)
{
	uint8_t rtc[9];

	// read 7 bytes starting from register 0
	// sec, min, hour, day-of-week, date, month, year
	twi_begin_transmission(RTC_ADDR);
	twi_send_byte(0x0);
	twi_end_transmission();

	twi_request_from(RTC_ADDR, 7);

	for(uint8_t i=0; i<7; i++) {
		rtc[i] = twi_receive();
	}

	twi_end_transmission();

	if (sec)  *sec =  bcd2dec(rtc[0]);
	if (min)  *min =  bcd2dec(rtc[1]);
	if (hour) *hour = bcd2dec(rtc[2]);
}

// fixme: support 12-hour mode for setting time
void rtc_set_time(struct tm* tm_)
{
	twi_begin_transmission(RTC_ADDR);
	twi_send_byte(0x0);

	uint8_t century;
	if (tm_->year > 2000) {
		century = 0x80;
		tm_->year = tm_->year - 2000;
	} else {
		century = 0;
		tm_->year = tm_->year - 1900;
	}

	// clock halt bit is 7th bit of seconds: this is always cleared to start the clock
	twi_send_byte(dec2bcd(tm_->sec)); // seconds
	twi_send_byte(dec2bcd(tm_->min)); // minutes
	twi_send_byte(dec2bcd(tm_->hour)); // hours
	twi_send_byte(dec2bcd(tm_->wday)); // day of week
	twi_send_byte(dec2bcd(tm_->mday)); // day
	twi_send_byte(dec2bcd(tm_->mon) + century); // month
	twi_send_byte(dec2bcd(tm_->year)); // year

	twi_end_transmission();
}

void rtc_set_time_s(uint8_t hour, uint8_t min, uint8_t sec)
{
	twi_begin_transmission(RTC_ADDR);
	twi_send_byte(0x0);

	// clock halt bit is 7th bit of seconds: this is always cleared to start the clock
	twi_send_byte(dec2bcd(sec)); // seconds
	twi_send_byte(dec2bcd(min)); // minutes
	twi_send_byte(dec2bcd(hour)); // hours

	twi_end_transmission();
}

// DS1307 only (has no effect when run on DS3231)
// halt/start the clock
// 7th bit of register 0 (second register)
// 0 = clock is running
// 1 = clock is not running
void rtc_run_clock(bool run)
{
  if (s_is_ds3231) return;

  uint8_t b = rtc_read_byte(0x0);

  if (run)
    b &= ~(_BV(CH_BIT)); // clear bit
  else
    b |= _BV(CH_BIT); // set bit

    rtc_write_byte(b, 0x0);
}

// DS1307 only
// Returns true if the clock is running, false otherwise
// For DS3231, it always returns true
bool rtc_is_clock_running(void)
{
  if (s_is_ds3231) return true;

  uint8_t b = rtc_read_byte(0x0);

  if (b & _BV(CH_BIT)) return false;
  return true;
}

void ds3231_get_temp_int(int8_t* i, uint8_t* f)
{
	uint8_t msb, lsb;

	*i = 0;
	*f = 0;

	if (s_is_ds1307) return; // only valid on DS3231

	twi_begin_transmission(RTC_ADDR);
	// temp registers 0x11 and 0x12
	twi_send_byte(0x11);
	twi_end_transmission();

	twi_request_from(RTC_ADDR, 2);

	if (twi_available()) {
		msb = twi_receive(); // integer part (in twos complement)
		lsb = twi_receive(); // fraction part

		// integer part in entire byte
		*i = msb;
		// fractional part in top two bits (increments of 0.25)
		*f = (lsb >> 6) * 25;

		// float value can be read like so:
		// float temp = ((((short)msb << 8) | (short)lsb) >> 6) / 4.0f;
	}
}

void rtc_force_temp_conversion(uint8_t block)
{
	if (s_is_ds1307) return; // only valid on DS3231

	// read control register (0x0E)
	twi_begin_transmission(RTC_ADDR);
	twi_send_byte(0x0E);
	twi_end_transmission();

	twi_request_from(RTC_ADDR, 1);
	uint8_t ctrl = twi_receive();

	ctrl |= 0b00100000; // Set CONV bit

	// write new control register value
	twi_begin_transmission(RTC_ADDR);
	twi_send_byte(0x0E);
	twi_send_byte(ctrl);
	twi_end_transmission();

	if (!block) return;

	// Temp conversion is ready when control register becomes 0
	do {
		// Block until CONV is 0
		twi_begin_transmission(RTC_ADDR);
		twi_send_byte(0x0E);
		twi_end_transmission();
		twi_request_from(RTC_ADDR, 1);
	} while ((twi_receive() & 0b00100000) != 0);
}


#define DS1307_SRAM_ADDR 0x08

// SRAM: 56 bytes from address 0x08 to 0x3f (DS1307-only)
void rtc_get_sram(uint8_t* data)
{
	// cannot receive 56 bytes in one go, because of the TWI library buffer limit
	// so just receive one at a time for simplicity
  	for(int i=0;i<56;i++)
		data[i] = rtc_get_sram_byte(i);
}

void rtc_set_sram(uint8_t *data)
{
	// cannot send 56 bytes in one go, because of the TWI library buffer limit
	// so just send one at a time for simplicity
  	for(int i=0;i<56;i++)
		rtc_set_sram_byte(data[i], i);
}

uint8_t rtc_get_sram_byte(uint8_t offset)
{
	twi_begin_transmission(RTC_ADDR);
	twi_send_byte(DS1307_SRAM_ADDR + offset);
	twi_end_transmission();

	twi_request_from(RTC_ADDR, 1);
	return twi_receive();
}

void rtc_set_sram_byte(uint8_t b, uint8_t offset)
{
	twi_begin_transmission(RTC_ADDR);
	twi_send_byte(DS1307_SRAM_ADDR + offset);
	twi_send_byte(b);
	twi_end_transmission();
}

void rtc_SQW_enable(bool enable)
{
	if (s_is_ds1307) {
		twi_begin_transmission(RTC_ADDR);
		twi_send_byte(0x07);
		twi_end_transmission();

		// read control
   		twi_request_from(RTC_ADDR, 1);
		uint8_t control = twi_receive();

		if (enable)
			control |=  0b00010000; // set SQWE to 1
		else
			control &= ~0b00010000; // set SQWE to 0

		// write control back
		twi_begin_transmission(RTC_ADDR);
		twi_send_byte(0x07);
		twi_send_byte(control);
		twi_end_transmission();

	}
	else { // DS3231
		twi_begin_transmission(RTC_ADDR);
		twi_send_byte(0x0E);
		twi_end_transmission();

		// read control
   		twi_request_from(RTC_ADDR, 1);
		uint8_t control = twi_receive();

		if (enable) {
			control |=  0b01000000; // set BBSQW to 1
			control &= ~0b00000100; // set INTCN to 0
		}
		else {
			control &= ~0b01000000; // set BBSQW to 0
		}

		// write control back
		twi_begin_transmission(RTC_ADDR);
		twi_send_byte(0x0E);
		twi_send_byte(control);
		twi_end_transmission();
	}
}

void rtc_SQW_set_freq(enum RTC_SQW_FREQ freq)
{
	if (s_is_ds1307) {
		twi_begin_transmission(RTC_ADDR);
		twi_send_byte(0x07);
		twi_end_transmission();

		// read control (uses bits 0 and 1)
   		twi_request_from(RTC_ADDR, 1);
		uint8_t control = twi_receive();

		control &= ~0b00000011; // Set to 0
		control |= freq; // Set freq bitmask

		// write control back
		twi_begin_transmission(RTC_ADDR);
		twi_send_byte(0x07);
		twi_send_byte(control);
		twi_end_transmission();

	}
	else { // DS3231
		twi_begin_transmission(RTC_ADDR);
		twi_send_byte(0x0E);
		twi_end_transmission();

		// read control (uses bits 3 and 4)
   		twi_request_from(RTC_ADDR, 1);
		uint8_t control = twi_receive();

		control &= ~0b00011000; // Set to 0
		control |= (freq << 4); // Set freq bitmask

		// write control back
		twi_begin_transmission(RTC_ADDR);
		twi_send_byte(0x0E);
		twi_send_byte(control);
		twi_end_transmission();
	}
}

void rtc_osc32kHz_enable(bool enable)
{
	if (!s_is_ds3231) return;

	twi_begin_transmission(RTC_ADDR);
	twi_send_byte(0x0F);
	twi_end_transmission();

	// read status
	twi_request_from(RTC_ADDR, 1);
	uint8_t status = twi_receive();

	if (enable)
		status |= 0b00001000; // set to 1
	else
		status &= ~0b00001000; // Set to 0

	// write status back
	twi_begin_transmission(RTC_ADDR);
	twi_send_byte(0x0F);
	twi_send_byte(status);
	twi_end_transmission();
}

// Alarm functionality
// fixme: should decide if "alarm disabled" mode should be available, or if alarm should always be enabled
// at 00:00:00. Currently, "alarm disabled" only works for ds3231
void rtc_reset_alarm(void)
{
	if (s_is_ds1307) {
		rtc_set_sram_byte(0, 0); // hour
		rtc_set_sram_byte(0, 1); // minute
		rtc_set_sram_byte(0, 2); // second
	}
	else {
		// writing 0 to bit 7 of all four alarm 1 registers disables alarm
		rtc_write_byte(0, 0x07); // second
		rtc_write_byte(0, 0x08); // minute
		rtc_write_byte(0, 0x09); // hour
		rtc_write_byte(0, 0x0a); // day
	}
}

// fixme: add an option to set whether or not the INTCN and Interrupt Enable flag is set when setting the alarm
void rtc_set_alarm_s(uint8_t hour, uint8_t min, uint8_t sec)
{
	if (hour > 23) return;
	if (min > 59) return;
	if (sec > 59) return;

	if (s_is_ds1307) {
		rtc_set_sram_byte(hour, 0); // hour
		rtc_set_sram_byte(min,  1); // minute
		rtc_set_sram_byte(sec,  2); // second
	}
	else {
		/*
		 *  07h: A1M1:0  Alarm 1 seconds
		 *  08h: A1M2:0  Alarm 1 minutes
		 *  09h: A1M3:0  Alarm 1 hour (bit6 is am/pm flag in 12h mode)
		 *  0ah: A1M4:1  Alarm 1 day/date (bit6: 1 for day, 0 for date)
		 *  Sets alarm to fire when hour, minute and second matches
		 */
		rtc_write_byte(dec2bcd(sec),  0x07); // second
		rtc_write_byte(dec2bcd(min),  0x08); // minute
		rtc_write_byte(dec2bcd(hour), 0x09); // hour
		rtc_write_byte(0b10000001,         0x0a); // day (upper bit must be set)

		// clear alarm flag
		uint8_t val = rtc_read_byte(0x0f);
		rtc_write_byte(val & ~0b00000001, 0x0f);
	}
}

void rtc_set_alarm(struct tm* tm_)
{
	if (!tm_) return;
	rtc_set_alarm_s(tm_->hour, tm_->min, tm_->sec);
}

void rtc_get_alarm_s(uint8_t* hour, uint8_t* min, uint8_t* sec)
{
	if (s_is_ds1307) {
		if (hour) *hour = rtc_get_sram_byte(0);
		if (min)  *min  = rtc_get_sram_byte(1);
		if (sec)  *sec  = rtc_get_sram_byte(2);
	}
	else {
		*sec  = bcd2dec(rtc_read_byte(0x07) & ~0b10000000);
		*min  = bcd2dec(rtc_read_byte(0x08) & ~0b10000000);
		*hour = bcd2dec(rtc_read_byte(0x09) & ~0b10000000);
	}
}

struct tm* rtc_get_alarm(void)
{
	uint8_t hour, min, sec;

	rtc_get_alarm_s(&hour, &min, &sec);
	_tm.hour = hour;
	_tm.min = min;
	_tm.sec = sec;
	return &_tm;
}

bool rtc_check_alarm(void)
{
	if (s_is_ds1307) {
		uint8_t hour = rtc_get_sram_byte(0);
		uint8_t min  = rtc_get_sram_byte(1);
		uint8_t sec  = rtc_get_sram_byte(2);

		uint8_t cur_hour, cur_min, cur_sec;
		rtc_get_time_s(&cur_hour, &cur_min, &cur_sec);

		if (cur_hour == hour && cur_min == min && cur_sec == sec)
			return true;
		return false;
	}
	else {
		// Alarm 1 flag (A1F) in bit 0
		uint8_t val = rtc_read_byte(0x0f);

		// clear flag when set
		if (val & 1)
			rtc_write_byte(val & ~0b00000001, 0x0f);

		return val & 1 ? 1 : 0;
	}
}


