/* 
 Description
*/

#include "ATM90E26SPI.h"
#include <SPI.h>

ATM90E26SPI::ATM90E26SPI(int pin) {
  pinCS = pin;
}

unsigned short ATM90E26SPI::RegisterRW(byte RW, byte address, unsigned short val) {
  unsigned short output = 0x0000;
  SPISettings settings(200000, MSBFIRST, SPI_MODE3);

  /* Set read write flag */
  address |= RW << 7;

  /* Transmit and receive data */
  SPI.beginTransaction(settings);
  digitalWrite(pinCS, LOW);
  delayMicroseconds(10);
  SPI.transfer(address);

  /* Must wait 4 us for data to become valid */
  delayMicroseconds(4);

  if(RW) {
    output = SPI.transfer16(0x0000);
  }
  else {
    SPI.transfer16(val);
  }

  digitalWrite(pinCS, HIGH);
  delayMicroseconds(10);
  SPI.endTransaction();

  return output;
}

void ATM90E26SPI::SoftResetASIC() {
  RegisterRW(0, SOFT_RESET, 0x789A);
}

void ATM90E26SPI::InitASIC() {

  pinMode(pinCS, OUTPUT);
  digitalWrite(pinCS, HIGH);
  delay(10);
  
  /* Enable SPI */
  SPI.begin();
  
  /* Perform soft reset */
  SoftResetASIC();
  
  /* Calibrate ASIC with Default Values */
  /* Metering Calbration */
  MeteringCalibrationRegisters meteringCalibrationRegisters;
  meteringCalibrationRegisters.plConstH                 =   _PL_CONST_H;
  meteringCalibrationRegisters.plConstL                 =   _PL_CONST_L;
  meteringCalibrationRegisters.calibrationGainL         =   _L_GAIN;
  meteringCalibrationRegisters.calibrationAngleL        =   _L_PHI;   
  meteringCalibrationRegisters.calibrationGainN         =   _N_GAIN;    
  meteringCalibrationRegisters.calibrationAngleN        =   _N_PHI;   
  meteringCalibrationRegisters.activeStartupPowerTh     =   _P_START_TH;
  meteringCalibrationRegisters.activeNoLoadPowerTh      =   _P_NOL_TH;
  meteringCalibrationRegisters.reactiveStartupPowerTh   =   _Q_START_TH;
  meteringCalibrationRegisters.reactiveNoLoadPowerTh    =   _Q_NOL_TH;
  meteringCalibrationRegisters.meteringModeConfig       =   _M_MODE;

  /* Measurement Calbration */
  MeasurementCalibrationRegisters measurementCalibrationRegisters;
  measurementCalibrationRegisters.rmsVoltageGain         =   _U_GAIN;
  measurementCalibrationRegisters.rmsCurrentGainL        =   _I_GAIN_L;  
  measurementCalibrationRegisters.rmsCurrentGainN        =   _I_GAIN_N; 
  measurementCalibrationRegisters.voltageOffset          =   _U_OFFSET; 
  measurementCalibrationRegisters.currentOffsetL         =   _I_OFFSET_L;
  measurementCalibrationRegisters.currentOffsetN         =   _I_OFFSET_N;
  measurementCalibrationRegisters.activePowerOffsetL     =   _P_OFFSET_L;
  measurementCalibrationRegisters.reactivePowerOffsetL   =   _Q_OFFSET_L;
  measurementCalibrationRegisters.activePowerOffsetN     =   _P_OFFSET_N;
  measurementCalibrationRegisters.reactivePowerOffsetN   =   _Q_OFFSET_N;
  
  /* Calibrate ASIC */
  CalibrateASIC(meteringCalibrationRegisters, measurementCalibrationRegisters);
  
}

SpecialRegisters ATM90E26SPI::GetSpecialRegisters() {
  SpecialRegisters specialRegisters;

  specialRegisters.systemStatus     =   RegisterRW(1, SYS_STATUS , 0xFFFF);
  specialRegisters.functionEnable   =   RegisterRW(1, FUNC_EN    , 0xFFFF);
  specialRegisters.voltageSagTh     =   RegisterRW(1, SAG_TH     , 0xFFFF);
  specialRegisters.smallPowerMode   =   RegisterRW(1, SMALL_P_MOD, 0xFFFF);
  specialRegisters.lastDataRW       =   RegisterRW(1, LAST_DATA  , 0xFFFF);

  return specialRegisters;
}

MeteringCalibrationRegisters ATM90E26SPI::GetMeteringCalibrationRegisters() {
  MeteringCalibrationRegisters meteringCalibrationRegisters;

  meteringCalibrationRegisters.lsb                      =   RegisterRW(1, LSB       , 0xFFFF);
  meteringCalibrationRegisters.calibrationStart         =   RegisterRW(1, CAL_START , 0xFFFF);
  meteringCalibrationRegisters.plConstH                 =   RegisterRW(1, PL_CONST_H, 0xFFFF);
  meteringCalibrationRegisters.plConstL                 =   RegisterRW(1, PL_CONST_L, 0xFFFF);
  meteringCalibrationRegisters.calibrationGainL         =   RegisterRW(1, L_GAIN    , 0xFFFF);
  meteringCalibrationRegisters.calibrationAngleL        =   RegisterRW(1, L_PHI     , 0xFFFF);
  meteringCalibrationRegisters.calibrationGainN         =   RegisterRW(1, N_GAIN    , 0xFFFF);
  meteringCalibrationRegisters.calibrationAngleN        =   RegisterRW(1, N_PHI     , 0xFFFF);
  meteringCalibrationRegisters.activeStartupPowerTh     =   RegisterRW(1, P_START_TH, 0xFFFF);
  meteringCalibrationRegisters.activeNoLoadPowerTh      =   RegisterRW(1, P_NOL_TH  , 0xFFFF);
  meteringCalibrationRegisters.reactiveStartupPowerTh   =   RegisterRW(1, Q_START_TH, 0xFFFF);
  meteringCalibrationRegisters.reactiveNoLoadPowerTh    =   RegisterRW(1, Q_NOL_TH  , 0xFFFF);
  meteringCalibrationRegisters.meteringModeConfig       =   RegisterRW(1, M_MODE    , 0xFFFF);
  meteringCalibrationRegisters.checksum                 =   RegisterRW(1, CS1       , 0xFFFF);

  return meteringCalibrationRegisters;
}

MeasurementCalibrationRegisters ATM90E26SPI::GetMeasurementCalibrationRegisters() {
  MeasurementCalibrationRegisters measurementCalibrationRegisters;

  measurementCalibrationRegisters.calibrationStart       =   RegisterRW(1, ADJ_START , 0xFFFF);
  measurementCalibrationRegisters.rmsVoltageGain         =   RegisterRW(1, U_GAIN    , 0xFFFF);
  measurementCalibrationRegisters.rmsCurrentGainL        =   RegisterRW(1, I_GAIN_L  , 0xFFFF);
  measurementCalibrationRegisters.rmsCurrentGainN        =   RegisterRW(1, I_GAIN_N  , 0xFFFF);
  measurementCalibrationRegisters.voltageOffset          =   RegisterRW(1, U_OFFSET  , 0xFFFF);
  measurementCalibrationRegisters.currentOffsetL         =   RegisterRW(1, I_OFFSET_L, 0xFFFF);
  measurementCalibrationRegisters.currentOffsetN         =   RegisterRW(1, I_OFFSET_N, 0xFFFF);
  measurementCalibrationRegisters.activePowerOffsetL     =   RegisterRW(1, P_OFFSET_L, 0xFFFF);
  measurementCalibrationRegisters.reactivePowerOffsetL   =   RegisterRW(1, Q_OFFSET_L, 0xFFFF);
  measurementCalibrationRegisters.activePowerOffsetN     =   RegisterRW(1, P_OFFSET_N, 0xFFFF);
  measurementCalibrationRegisters.reactivePowerOffsetN   =   RegisterRW(1, Q_OFFSET_N, 0xFFFF);
  measurementCalibrationRegisters.checksum               =   RegisterRW(1, CS2       , 0xFFFF);

  return measurementCalibrationRegisters;
}

EnergyRegisters ATM90E26SPI::GetEnergyRegisters() {
  EnergyRegisters energyRegisters;
  
  energyRegisters.forwardActiveEnergy      =   RegisterRW(1, AP_ENERGY, 0xFFFF);
  energyRegisters.reverseActiveEnergy      =   RegisterRW(1, AN_ENERGY, 0xFFFF);
  energyRegisters.absoluteActiveEnergy     =   RegisterRW(1, AT_ENERGY, 0xFFFF);
  energyRegisters.forwardReactiveEnergy    =   RegisterRW(1, RP_ENERGY, 0xFFFF);
  energyRegisters.reverseReactiveEnergy    =   RegisterRW(1, RN_ENERGY, 0xFFFF);
  energyRegisters.absoluteReactiveEnergy   =   RegisterRW(1, RT_ENERGY, 0xFFFF);
  energyRegisters.meteringStatus           =   RegisterRW(1, EN_STATUS, 0xFFFF);
  
  return energyRegisters;
}

MeasurementRegisters ATM90E26SPI::GetMeasurementRegisters() {
  MeasurementRegisters measurementRegisters;

  measurementRegisters.rmsCurrentL          =   RegisterRW(1, I_RMS_L  , 0xFFFF);
  measurementRegisters.rmsVoltage           =   RegisterRW(1, U_RMS    , 0xFFFF);
  measurementRegisters.meanActivePowerL     =   RegisterRW(1, P_MEAN_L , 0xFFFF);
  measurementRegisters.meanReactivePowerL   =   RegisterRW(1, Q_MEAN_L , 0xFFFF);
  measurementRegisters.voltageFrequency     =   RegisterRW(1, FREQ     , 0xFFFF);
  measurementRegisters.powerFactorL         =   RegisterRW(1, POWER_F_L, 0xFFFF);
  measurementRegisters.phaseAngleL          =   RegisterRW(1, P_ANGLE_L, 0xFFFF);
  measurementRegisters.meanApparentPowerL   =   RegisterRW(1, S_MEAN_L , 0xFFFF);
  measurementRegisters.rmsCurrentN          =   RegisterRW(1, I_RMS_N  , 0xFFFF);
  measurementRegisters.meanActivePowerN     =   RegisterRW(1, P_MEAN_N , 0xFFFF);
  measurementRegisters.meanReactivePowerN   =   RegisterRW(1, Q_MEAN_N , 0xFFFF);
  measurementRegisters.powerFactorN         =   RegisterRW(1, POWER_F_N, 0xFFFF);
  measurementRegisters.phaseAngleN          =   RegisterRW(1, P_ANGLE_N, 0xFFFF);
  measurementRegisters.meanApparentPowerN   =   RegisterRW(1, S_MEAN_N , 0xFFFF);

  return measurementRegisters;
}

unsigned short ATM90E26SPI::CalcMeteringChecksum(MeteringCalibrationRegisters meteringCalibrationRegisters) {
  unsigned char checksumLowByte = 0;
  unsigned char checksumHighByte = 0;

  unsigned short meteringValues[11];
  meteringValues[0] = meteringCalibrationRegisters.plConstH;
  meteringValues[1] = meteringCalibrationRegisters.plConstL;
  meteringValues[2] = meteringCalibrationRegisters.calibrationGainL;
  meteringValues[3] = meteringCalibrationRegisters.calibrationAngleL;
  meteringValues[4] = meteringCalibrationRegisters.calibrationGainN;
  meteringValues[5] = meteringCalibrationRegisters.calibrationAngleN;
  meteringValues[6] = meteringCalibrationRegisters.activeStartupPowerTh;
  meteringValues[7] = meteringCalibrationRegisters.activeNoLoadPowerTh;
  meteringValues[8] = meteringCalibrationRegisters.reactiveStartupPowerTh;
  meteringValues[9] = meteringCalibrationRegisters.reactiveNoLoadPowerTh;
  meteringValues[10] = meteringCalibrationRegisters.meteringModeConfig;

  for(int i=0; i<11; i++) {
    checksumLowByte += meteringValues[i];
    checksumLowByte += meteringValues[i] >> 8;

    checksumHighByte ^= meteringValues[i];
    checksumHighByte ^= meteringValues[i] >> 8;
  }
  return ((unsigned short)checksumHighByte << 8) | checksumLowByte;
}

unsigned short ATM90E26SPI::CalcMeasurementChecksum(MeasurementCalibrationRegisters measurementCalibrationRegisters) {
  unsigned char checksumLowByte = 0;
  unsigned char checksumHighByte = 0;

  unsigned short measurementValues[10];
  measurementValues[0] = measurementCalibrationRegisters.rmsVoltageGain;
  measurementValues[1] = measurementCalibrationRegisters.rmsCurrentGainL;
  measurementValues[2] = measurementCalibrationRegisters.rmsCurrentGainN;
  measurementValues[3] = measurementCalibrationRegisters.voltageOffset;
  measurementValues[4] = measurementCalibrationRegisters.currentOffsetL;
  measurementValues[5] = measurementCalibrationRegisters.currentOffsetN;
  measurementValues[6] = measurementCalibrationRegisters.activePowerOffsetL;
  measurementValues[7] = measurementCalibrationRegisters.reactivePowerOffsetL;
  measurementValues[8] = measurementCalibrationRegisters.activePowerOffsetN;
  measurementValues[9] = measurementCalibrationRegisters.reactivePowerOffsetN;

  for(int i=0; i<10; i++) {
    checksumLowByte += measurementValues[i];
    checksumLowByte += measurementValues[i] >> 8;

    checksumHighByte ^= measurementValues[i];
    checksumHighByte ^= measurementValues[i] >> 8;
  }
  return ((unsigned short)checksumHighByte << 8) | checksumLowByte;
}

void ATM90E26SPI::CalibrateASIC(MeteringCalibrationRegisters meteringCalibrationRegisters, MeasurementCalibrationRegisters measurementCalibrationRegisters) {
  unsigned short meteringChecksum = CalcMeteringChecksum(meteringCalibrationRegisters);
  unsigned short measurementChecksum = CalcMeasurementChecksum(measurementCalibrationRegisters);

  /* Set Metering Calibration Values*/
  RegisterRW(0, CAL_START , 0x5678);
  
  RegisterRW(0, PL_CONST_H, meteringCalibrationRegisters.plConstH);
  RegisterRW(0, PL_CONST_L, meteringCalibrationRegisters.plConstL);
  RegisterRW(0, L_GAIN    , meteringCalibrationRegisters.calibrationGainL);
  RegisterRW(0, L_PHI     , meteringCalibrationRegisters.calibrationAngleL);
  RegisterRW(0, N_GAIN    , meteringCalibrationRegisters.calibrationGainN);
  RegisterRW(0, N_PHI     , meteringCalibrationRegisters.calibrationAngleN);
  RegisterRW(0, P_START_TH, meteringCalibrationRegisters.activeStartupPowerTh);
  RegisterRW(0, P_NOL_TH  , meteringCalibrationRegisters.activeNoLoadPowerTh);
  RegisterRW(0, Q_START_TH, meteringCalibrationRegisters.reactiveStartupPowerTh);
  RegisterRW(0, Q_NOL_TH  , meteringCalibrationRegisters.reactiveNoLoadPowerTh);
  RegisterRW(0, M_MODE    , meteringCalibrationRegisters.meteringModeConfig);

  RegisterRW(0, CS1       , meteringChecksum);

  /* Set Measurement Calibration Values */ 
  RegisterRW(0, ADJ_START , 0x5678);

  RegisterRW(0, U_GAIN    , measurementCalibrationRegisters.rmsVoltageGain);
  RegisterRW(0, I_GAIN_L  , measurementCalibrationRegisters.rmsCurrentGainL);
  RegisterRW(0, I_GAIN_N  , measurementCalibrationRegisters.rmsCurrentGainN);
  RegisterRW(0, U_OFFSET  , measurementCalibrationRegisters.voltageOffset);
  RegisterRW(0, I_OFFSET_L, measurementCalibrationRegisters.currentOffsetL);
  RegisterRW(0, I_OFFSET_N, measurementCalibrationRegisters.currentOffsetN);
  RegisterRW(0, P_OFFSET_L, measurementCalibrationRegisters.activePowerOffsetL);
  RegisterRW(0, Q_OFFSET_L, measurementCalibrationRegisters.reactivePowerOffsetL);
  RegisterRW(0, P_OFFSET_N, measurementCalibrationRegisters.activePowerOffsetN);
  RegisterRW(0, Q_OFFSET_N, measurementCalibrationRegisters.reactivePowerOffsetN);

  RegisterRW(0, CS1       , measurementChecksum);

  /* Verify Checksum */
  RegisterRW(0, CAL_START, 0x8765);
  RegisterRW(0, ADJ_START, 0x8765);
}
