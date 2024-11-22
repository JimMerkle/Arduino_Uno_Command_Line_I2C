// Copyright Jim Merkle, 11/19/2024
// File: i2c_ds3231.h
// I2C and DS3231 routines
//
// Notes:
//
#ifndef _i2c_ds3231_h_
#define _i2c_ds3231_h_


#ifdef __cplusplus
extern "C" {
#endif

#define I2C_ADDRESS_MIN	0x03
#define I2C_ADDRESS_MAX 0x77


#define SECONDS_PER_DAY 86400L ///< 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000                                              \
  946684800 ///< Unixtime for 2000-01-01 00:00:00, useful for initialization

//=================================================================================================
// I2C helper functions
//=================================================================================================
int i2c_validate_address(uint16_t i2c_address);
int i2c_write_read(uint8_t i2c_address, uint8_t * pwrite, uint8_t wr_count, uint8_t * pread, uint8_t rd_count);

//=================================================================================================
// DS3231 constants, functions
//=================================================================================================
/** Constants */
#define I2C_ADDRESS_DS3231  0x68

// Structure to hold/record time read from to written to an external RTC module (DS3231)
typedef struct {
	uint8_t yOff; ///< Year offset from 2000
	uint8_t m;    ///< Month 1-12
	uint8_t d;    ///< Day 1-31
	uint8_t hh;   ///< Hours 0-23
	uint8_t mm;   ///< Minutes 0-59
	uint8_t ss;   ///< Seconds 0-59
}DATE_TIME;

void init_ds3231(void);
void write_ds3231(const DATE_TIME * dt);
void read_ds3231(DATE_TIME * dt);

//=================================================================================================
// Command line functions
//=================================================================================================
int cl_i2c_scan(void);
int cl_time(void);
int cl_date(void);

#ifdef __cplusplus
}
#endif

#endif // _i2c_ds3231_h_
