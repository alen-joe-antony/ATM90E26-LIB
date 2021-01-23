/* 
 *  TEST PROGRAM
*/

#include "ATM90E26SPI.h"

ATM90E26SPI meter;

void setup() {
  /* Initialize the serial port to host */
  Serial.begin(115200);
  /*Initialise the ATM90E26 + SPI port */
  meter.InitASIC();
}

void loop() {
  /*Repeatedly fetch some values from the ATM90E26 */
  EnergyRegisters eReg;
  eReg = meter.GetEnergyRegisters();
  Serial.print("Meter Status: ");
  Serial.println(eReg.meteringStatus,HEX);
  yield();
  delay(2000);
}
