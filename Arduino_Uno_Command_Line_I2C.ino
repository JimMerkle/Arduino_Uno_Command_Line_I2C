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
#include "i2c_ds3231.h"

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
