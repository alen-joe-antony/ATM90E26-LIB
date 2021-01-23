/* 
  Description
*/

#ifndef __ATM90E26SPI_H__
#define __ATM90E26SPI_H__

#include <Arduino.h>

/* Status and Special Registers */
#define SOFT_RESET    0x00    // Software Reset Register
#define SYS_STATUS    0x01    // System Status Register
#define FUNC_EN       0x02    // Function Enable Register
#define SAG_TH        0x03    // Voltage Sag Threshold Register
#define SMALL_P_MOD   0x04    // Small Power Mode Register
#define LAST_DATA     0x06    // Last SPI/UART Read/Write Data Register
#define EN_STATUS     0x46    // Metering Status Register

/* Metering Calibration and Configuration Register */
#define LSB          0x08    // RMS/Power 16-Bit LSB Register
#define CAL_START    0x20    // Calibration Start Register
#define PL_CONST_H   0x21    // High Word of PL_Constant Register
#define PL_CONST_L   0x22    // Low Word of PL_Constant Register
#define L_GAIN       0x23    // L Line Calibration Gain Register
#define L_PHI        0x24    // L Line Calibration Angle Register
#define N_GAIN       0x25    // N Line Calibration Gain Register
#define N_PHI        0x26    // N Line Calibration Angle Register
#define P_START_TH   0x27    // Active Startup Power Threshold Register
#define P_NOL_TH     0x28    // Active No-Load Power Threshold Register
#define Q_START_TH   0x29    // Reactive Startup Power Threshold Register
#define Q_NOL_TH     0x2A    // Reactive No-Load Power Threshold Register
#define M_MODE       0x2B    // Metering Mode Configuration Register
#define CS1          0x2C    // Checksum 1 Register

/* Metering Calibration and Configuration Register Default Values */
#define _PL_CONST_H   0x0015
#define _PL_CONST_L   0xD174
#define _L_GAIN       0x0000
#define _L_PHI        0x0000
#define _N_GAIN       0x0000
#define _N_PHI        0x0000
#define _P_START_TH   0x08BD
#define _P_NOL_TH     0x0000
#define _Q_START_TH   0x0AEC
#define _Q_NOL_TH     0x0000
#define _M_MODE       0x9422

/* Measurement Calibration Register */
#define ADJ_START    0x30    // Measurement Calibration Start Register
#define U_GAIN       0x31    // Voltage RMS Gain Register
#define I_GAIN_L     0x32    // L Line Current RMS Gain Register
#define I_GAIN_N     0x33    // N Line Current RMS Gain Register
#define U_OFFSET     0x34    // Voltage Offset Register
#define I_OFFSET_L   0x35    // L Line Current Offset Register
#define I_OFFSET_N   0x36    // N Line Current Offset Register
#define P_OFFSET_L   0x37    // L Line Active Power Offset Register
#define Q_OFFSET_L   0x38    // L Line Reactive Power Offset Register
#define P_OFFSET_N   0x39    // N Line Active Power Offset Register
#define Q_OFFSET_N   0x3A    // N Line Reactive Power Offset Register
#define CS2          0x3B    // Checksum 2 Register

/* Measurement Calibration Register Default Values */
#define _U_GAIN       0x6720
#define _I_GAIN_L     0x7A13
#define _I_GAIN_N     0x7530
#define _U_OFFSET     0x0000
#define _I_OFFSET_L   0x0000
#define _I_OFFSET_N   0x0000
#define _P_OFFSET_L   0x0000
#define _Q_OFFSET_L   0x0000
#define _P_OFFSET_N   0x0000
#define _Q_OFFSET_N   0x0000


/* Energy Registers */
#define AP_ENERGY   0x40     // Forward Active Energy Register
#define AN_ENERGY   0x41     // Reverse Active Energy Register
#define AT_ENERGY   0x42     // Absolute Active Energy Register
#define RP_ENERGY   0x43     // Forward (Inductive) Reactive Energy Register 
#define RN_ENERGY   0x44     // Reverse (Capacitive) Reactive Energy Register
#define RT_ENERGY   0x45     // Absolute Rective Energy Register

/* Measurement Register */
#define I_RMS_L     0x48    // L Line Current RMS
#define U_RMS       0x49    // Voltage RMS
#define P_MEAN_L    0x4A    // L Line Mean Active Power
#define Q_MEAN_L    0x4B    // L Line Mean Reactive Power
#define FREQ        0x4C    // Voltage Frequency
#define POWER_F_L   0x4D    // L Line Power Factor
#define P_ANGLE_L   0x4E    // Phase Angle between Voltage and L Line Current
#define S_MEAN_L    0x4F    // L Line Mean Apparent Power 
#define I_RMS_N     0x68    // N Line Current RMS
#define P_MEAN_N    0x6A    // N Line Mean Active Power
#define Q_MEAN_N    0x6B    // N Line Mean Reactive Power
#define POWER_F_N   0x6D    // N Line Power Factor
#define P_ANGLE_N   0x6E    // Phase Angle between Voltage and N Line Current
#define S_MEAN_N    0x6F    // N Line Mean Apparent Power 


/* Use default SS pin for CS */
const int _CS = SS;

/* Struct to store Status and Special Register values */
struct SpecialRegisters {
  unsigned short systemStatus;
  unsigned short functionEnable;
  unsigned short voltageSagTh;
  unsigned short smallPowerMode;
  unsigned short lastDataRW;
};

/* Struct to store Metering Calibration and Configuration Register values */
struct MeteringCalibrationRegisters {
  unsigned short lsb;
  unsigned short calibrationStart;
  unsigned short plConstH;
  unsigned short plConstL;
  unsigned short calibrationGainL;
  unsigned short calibrationAngleL;
  unsigned short calibrationGainN;
  unsigned short calibrationAngleN;
  unsigned short activeStartupPowerTh;
  unsigned short activeNoLoadPowerTh;
  unsigned short reactiveStartupPowerTh;
  unsigned short reactiveNoLoadPowerTh;
  unsigned short meteringModeConfig;
  unsigned short checksum;
};

/* Struct to store Measurement Calibration Register values */
struct MeasurementCalibrationRegisters {
  unsigned short calibrationStart;
  unsigned short rmsVoltageGain;
  unsigned short rmsCurrentGainL;
  unsigned short rmsCurrentGainN;
  unsigned short voltageOffset;
  unsigned short currentOffsetL;
  unsigned short currentOffsetN;
  unsigned short activePowerOffsetL;
  unsigned short reactivePowerOffsetL;
  unsigned short activePowerOffsetN;
  unsigned short reactivePowerOffsetN;
  unsigned short checksum;
};

/* Struct to store Energy Register values */
struct EnergyRegisters {
  unsigned short forwardActiveEnergy;
  unsigned short reverseActiveEnergy;
  unsigned short absoluteActiveEnergy;
  unsigned short forwardReactiveEnergy;
  unsigned short reverseReactiveEnergy;
  unsigned short absoluteReactiveEnergy;
  unsigned short meteringStatus;
};

/* Structure to store Measurement Register values */
struct MeasurementRegisters {
  unsigned short rmsCurrentL;
  unsigned short rmsVoltage;
  unsigned short meanActivePowerL;
  unsigned short meanReactivePowerL;
  unsigned short voltageFrequency;
  unsigned short powerFactorL;
  unsigned short phaseAngleL;
  unsigned short meanApparentPowerL;
  unsigned short rmsCurrentN;
  unsigned short meanActivePowerN;
  unsigned short meanReactivePowerN;
  unsigned short powerFactorN;
  unsigned short phaseAngleN;
  unsigned short meanApparentPowerN;
};

class ATM90E26SPI {
  public:
    ATM90E26SPI(int pin = _CS);
    
    /* Soft Reset the ASIC */
    void SoftResetASIC();

    /* Initialize the ASIC */
    void InitASIC();

    /* Get values from Status and Special Registers */
    SpecialRegisters GetSpecialRegisters();

    /* Get values from Metering Calibration and Configuration Registers */
    MeteringCalibrationRegisters GetMeteringCalibrationRegisters();

    /* Get values from Measurement Calibration Registers */
    MeasurementCalibrationRegisters GetMeasurementCalibrationRegisters();

    /* Get values from Energy Registers */
    EnergyRegisters GetEnergyRegisters();

    /* Get values from Measurement Registers */
    MeasurementRegisters GetMeasurementRegisters();

  private:
    /* Read from / Write to ASIC Register Using SPI */
    unsigned short RegisterRW(byte, byte, unsigned short);

    /* Calculate Checksum */
    unsigned short CalcMeteringChecksum(MeteringCalibrationRegisters);
    unsigned short CalcMeasurementChecksum(MeasurementCalibrationRegisters);

    /* Calibrate ASIC */
    void CalibrateASIC(MeteringCalibrationRegisters, MeasurementCalibrationRegisters);

    int pinCS;
};

#endif
