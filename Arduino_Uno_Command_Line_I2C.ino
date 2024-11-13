// Arduino_Uno_Command_Line_I2C.ino
// Development board: Arduino Uno
// Connections:
//   I2C:SCL - Arduino A5 (PC5)   
//   I2C:SDA - Arduino A4 (PC4)
// Use Arduino's "Serial Monitor" or Teraterm with local echo
//
// Notes:
//
#include <Wire.h>
#include "command_line.h"

void init_ds3231(void);

// Create a file stream with name, f_out
FILE f_out;
int sput(char c, __attribute__((unused)) FILE* f) {return !Serial.write(c);}

void setup() {
  Wire.begin();
  // Using Serial1, open serial communications and wait for port to open:
  Serial.begin(115200);
  //Serial.setTimeout(100); // try 100ms timeout

  // Assign f_out for _FDEV_SETUP_WRITE
  fdev_setup_stream(&f_out, sput, nullptr, _FDEV_SETUP_WRITE); // cf https://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html#gaf41f158c022cbb6203ccd87d27301226
  stdout = &f_out;

  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  init_ds3231(); // init DS3231 - allow it to begin counting
  cl_setup();    // init command line
}

// lame dump routine
void lame_dump(uint8_t *p, int len)
{
  int i=0;
  while (len > 0) {
    printf("%02X ",*p);
    p++; i++; len--;
    if(i>=16) {
      printf("\n");
      i=0;
    }
  }
  printf("\n"); // new line
}

// Given a null terminated character array, walk the array.
// If carrage return or linefeed are found, terminate the array, removing these characters
void remove_crlf(char * sz)
{
  while(*sz) {
    if(*sz == '\r' || *sz == '\n')
    {
      *sz = '\0'; // null terminate the string
      return;
    }
    sz++; // increment index
  }
}

String ser_data; // Uninitialized String object

void loop() {
  // Wait for a string from the Arduino "Serial Monitor"
  while(!Serial.available()){}; // spin until more serial data is available

  ser_data = Serial.readString(); // read in an entire string (or timeout) into String object..
  char * buffer = ser_data.c_str();

  // The following two lines write the buffer to the Arduino Terminal, displaying the text entered
  // For Teraterm - local echo, it produces duplicates of the text entered
  remove_crlf(buffer);
  printf("%s\n",buffer);  // display string entered w/o CR/LF on end
  
  cl_process_buffer(buffer);
  printf(">"); // command prompt for next line

}

#define I2C_ADDRESS_MIN	0x03
#define I2C_ADDRESS_MAX 0x77

int cl_i2c_scan(void)
{
  printf("I2C Scan - scanning I2C addresses 0x%02X - 0x%02X\n",I2C_ADDRESS_MIN,I2C_ADDRESS_MAX);
  // Display Hex Header
  printf("    "); for(int i=0;i<=0x0F;i++) printf(" %0X ",i);
  // Walk through address range 0x00 - 0x77, but only test 0x03 - 0x77
  for(uint8_t addr=0;addr<=I2C_ADDRESS_MAX;addr++) {
    // If address defines the beginning of a row, start a new row and display row text
    if(!(addr%16)) printf("\n%02X: ",addr);
    // Check I2C addresses in the range 0x03-0x7F
    if(addr < I2C_ADDRESS_MIN || addr > I2C_ADDRESS_MAX) {
      printf("   "); // out of range
      continue;
    }
    // Perform I2C device detection
    // The i2c_scanner uses the return value of the Wire.endTransmission to see if
    // a device acknowledged the address.
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();

    if(0 == error)
      printf("%02X ",addr);
    else
      printf("-- ");
  } // for-loop
  printf("\n");
  return 0;
}

// I2C helper function that validates I2C address is within range
// If I2C address is within range, return 0, else display error and return -1.
int i2c_validate_address(uint16_t i2c_address)
{
	if(i2c_address < I2C_ADDRESS_MIN || i2c_address > I2C_ADDRESS_MAX) {
		printf("Address out of range. Expect 0x%02X to 0x%02X\n",I2C_ADDRESS_MIN,I2C_ADDRESS_MAX); // out of range
		return -1;
	}
	return 0; // success
}

// I2C helper function that begins by writing zero or more bytes, followed by reading zero or more bytes.
// Return 0 for success
int i2c_write_read(uint8_t i2c_address, uint8_t * pwrite, uint8_t wr_count, uint8_t * pread, uint8_t rd_count)
{
	// Validate I2C address
	int rc=i2c_validate_address(i2c_address);
	if(rc) return rc;

	// If there are bytes to write, write them
	if(pwrite && wr_count) {
    //printf("%s Writing\n",__func__);
		Wire.beginTransmission(i2c_address);
    Wire.write(pwrite, wr_count);
    Wire.endTransmission(); // send stop condition
	} // if bytes to write

	// If there are bytes to read, read them
	if(pread && rd_count) {
    //printf("%s Reading\n",__func__);
		Wire.requestFrom(i2c_address, rd_count, (uint8_t)1); // send stop condition
    while(rd_count) {
      if(Wire.available()) {
        *pread = Wire.read(); // read I2C byte to pointer accress
        pread++;    // update pointer
        rd_count--; // decrement counter
      }
    } // while(rd_count)
	} // if bytes to read
	return 0;
}

//=================================================================================================
// DS3231 routines
//=================================================================================================
/** Constants */
#define I2C_ADDRESS_DS3231  0x68

#define SECONDS_PER_DAY 86400L ///< 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000                                              \
  946684800 ///< Unixtime for 2000-01-01 00:00:00, useful for initialization

// DS3231 registers use BCD encoding for time/date storage.
// This requires BCD to BIN and BIN to BCD functions to convert back and forth
static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }
static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }

// Values to write to DS3231 for a "reset" condition
const DATE_TIME dt_reset = {
	0,  //< Year offset from 2000
	1,  //< Month 1-12
	1,	//< Day 1-31
	0,  //< Hours 0-23
	0,  //< Minutes 0-59
	0   //< Seconds 0-59
};

// The Time/Date registers are located at index 00h - 06h
// Use a "generic I2C API" to write index registers 000h - 06h
// Write the DS3231 time/date registers given a DATE_TIME structure
void write_ds3231(const DATE_TIME * dt)
{
	uint8_t reg_data[8]; // [0] index, [1] - [7] time and date registers
	// Convert DATE_TIME format into DS3231 register values
	reg_data[0] = 0x00; // begin writing at index 0
	reg_data[1] = bin2bcd(dt->ss);
	reg_data[2] = bin2bcd(dt->mm);
	reg_data[3] = bin2bcd(dt->hh); // bit 6 should be low (We can force it...)
	reg_data[4] = 0; // day of the week (don't care)
	reg_data[5] = bin2bcd(dt->d);
	reg_data[6] = bin2bcd(dt->m); // remove century bit
	reg_data[7] = bin2bcd(dt->yOff);

	i2c_write_read(I2C_ADDRESS_DS3231, reg_data, sizeof(reg_data), NULL, 0);

	// Clear the OSF bit
	uint8_t index_status[2] = {0x0F, 0x00}; // index of status register and value to write to it
	i2c_write_read(I2C_ADDRESS_DS3231, index_status, sizeof(index_status), NULL, 0);
}

// The Time/Date registers are located at index 00h - 06h
// Use a "generic I2C API" to read index registers 000h - 06h
// Read the DS3231 time/date registers into a DATE_TIME structure
void read_ds3231(DATE_TIME * dt)
{
	uint8_t index = 0;
	uint8_t reg_data[7];
	i2c_write_read(I2C_ADDRESS_DS3231, &index, sizeof(index), reg_data, sizeof(reg_data));
	// Convert DS3231 register data into DATE_TIME format
	dt->ss = bcd2bin(reg_data[0]);
	dt->mm = bcd2bin(reg_data[1]);
	dt->hh = bcd2bin(reg_data[2]); // bit 6 should be low (We can force it...)
	dt->d  = bcd2bin(reg_data[4]);
	dt->m  = bcd2bin(reg_data[5] & 0x1F); // remove century bit
	dt->yOff = bcd2bin(reg_data[6]);
}


// Write the DS3231 Control register, get RTC counting
// Does NOT clear Oscillator Stop Flag (OSF)
// Set time to reset value if clock was stopped
void init_ds3231(void)
{
	uint8_t indx_control[2] = {0x0E, 0b00000000};  // Index, Control
  i2c_write_read(I2C_ADDRESS_DS3231, indx_control, sizeof(indx_control), NULL, 0);
	// Read status register - is OSF set?
	uint8_t index = 0x0F;
	uint8_t reg_data;
	i2c_write_read(I2C_ADDRESS_DS3231, &index, sizeof(index), &reg_data, sizeof(reg_data));
	// If OSF (BIT 7) set, write initial values to RTC registers
	if(reg_data & 0x80) {
    printf("Writing reset value toDS3231\n");
		write_ds3231(&dt_reset);
	}
}

// Command line method to read / set the time
int cl_time(void)
{
	DATE_TIME dt;
	if(4 == argc) {
	  read_ds3231(&dt); // read in time and date values into DATE_TIME structure
		// Set the time using arguments at index 1 <hours>, 2 <minutes>, 3 <seconds>
		dt.hh = strtol(argv[1], NULL, 10); // user will use decimal
		dt.mm = strtol(argv[2], NULL, 10);
		dt.ss = strtol(argv[3], NULL, 10);
		// Write new time values to DS3231
		write_ds3231(&dt);
	}

	// Always read the DS3231 and display the time
	read_ds3231(&dt);
	printf("%02u:%02u:%02u\n",dt.hh,dt.mm,dt.ss);
	return 0;
}

// Command line method to read / set the date
int cl_date(void)
{
	DATE_TIME dt;
	read_ds3231(&dt); // read in time and date values into DATE_TIME structure

	if(4 == argc) {
		// Set the date using arguments at index 1 <day>, 2 <month>, 3 <year>
		dt.d = strtol(argv[1], NULL, 10); // user will use decimal
		dt.m = strtol(argv[2], NULL, 10);
		uint16_t year = strtol(argv[3], NULL, 10);
		if(year >= 2000) year -= 2000; // convert to offset
		dt.yOff = (uint8_t)year;
		// Write new time values to DS3231
		write_ds3231(&dt);
	}

	// Always read the DS3231 and display the date
	read_ds3231(&dt);
	printf("%02u/%02u/%04u\r\n",dt.d,dt.m,dt.yOff + 2000);
	return 0;
}

