// Arduino_Uno_Command_Line_I2C_DS3231_TM1637.ino
// Development board: Arduino Uno
// DS3231 Connections:
//   DS3231_I2C:SCL - Arduino A5 (PC5)   
//   DS3231_I2C:SDA - Arduino A4 (PC4)
//
// Using the "TM1637" library by Avishay Orpaz, vesion 1.2.0, "TM1637Display.h" / "TM1637Display.cpp"
// TM1637 Connections:
//   TM1637_CLK: A0
//   TM1637_IO:  A1
//
// Use Arduino's "Serial Monitor" or Teraterm with local echo
//
// Notes:
//
#include <Wire.h>           // I2C library class
#include "command_line.h"
#include "i2c_ds3231.h"
#include <TM1637Display.h> // TM1637 library class

//=============================================================================
// printf() support - Create an "f_out" file stream
// - use f_out for Serial.write() - single character output
//=============================================================================
FILE f_out;
int sput(char c, __attribute__((unused)) FILE* f) {return !Serial.write(c);}

void setup() {
  Wire.begin();
  // Using Serial1, open serial communications and wait for port to open:
  Serial.begin(115200);
  //Serial.setTimeout(100); // try 100ms timeout

  //=============================================================================
  // printf() support - Assign f_out for _FDEV_SETUP_WRITE
  fdev_setup_stream(&f_out, sput, nullptr, _FDEV_SETUP_WRITE); // cf https://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html#gaf41f158c022cbb6203ccd87d27301226
  stdout = &f_out;
  //=============================================================================

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

// Implement a non-blocking function to look for an input character (used by command_line.c)
// If character available, return chracter, else return EOF (-1)
int __io_getchar(void)
{
  int c = EOF;
  if(Serial.available()) {
    c = Serial.read();
  }
  return c;
}

void loop() {
  cl_loop(); // check for input character available (command_line.c)

}
