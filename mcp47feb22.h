#ifndef __MCP47FEB22_H
#define __MCP47FEB22_H

#include "stm32l0xx.h"
#include "i2c.h"

// REG ADDRESSES ALREADY << 3
#define defaultVDD 5000u
#define BASE_ADDR 0x60
#define RESET_REG 0B00000110
#define WAKE_REG 0B00001010
#define UPDATE_REG 0B00001000
#define GENERALCALL 0B0000000
#define READ 0B00000110
#define WRITE 0B00000000
#define DAC0_REG 0B00000000
#define DAC1_REG 0B00001000
#define VREF_REG 0B01000000
#define PD_REG 0B01001000
#define GAIN_REG 0B01010000
#define WL_REG 0B000001011
#define DAC0_EP_REG 0B10000000
#define DAC1_EP_REG 0B10001000
#define VREF_EP_REG 0B11000000
#define PD_EP_REG 0B11001000
#define GAIN_EP_REG 0B11010000

#define UNLOCK_SALCK 0B11010010
#define LOCK_SALCK 0b11010100


unsigned char 	readBuffer[5];

unsigned char _buffer[5];
uint8_t      _dev_address;
uint8_t      _deviceID;
uint8_t      _intVref[2];
uint8_t      _gain[2];
uint8_t      _powerDown[2];
uint16_t     _values[2];
uint16_t     _valuesEp[2];
uint8_t      _intVrefEp[2];
uint8_t      _gainEp[2];
uint8_t      _powerDownEp[2];
uint8_t      _wiperLock[2];
uint16_t     _vdd;

 typedef struct {
	uint8_t deviceID;
	uint8_t devAddr;
	uint16_t vdd;
	I2C_HandleTypeDef hi2c;
} DAC_TypeDef;




void MCP47FEB22_Init(DAC_TypeDef* dac, uint8_t deviceID, I2C_HandleTypeDef hi2c);
void  MCP47FEB22_UnlockSALCK(DAC_TypeDef dac);
void  MCP47FEB22_LockSALCK(DAC_TypeDef dac, uint8_t addr);
void MCP47FEB22_ChangeAddr(DAC_TypeDef dac, uint8_t addr);
uint8_t MCP47FEB22_GetPowerDown(DAC_TypeDef dac, int channel);
void MCP47FEB22_SetPowerDown(DAC_TypeDef dac, int val0, int val1);
uint8_t MCP47FEB22_GetPowerDownEp(DAC_TypeDef dac, int channel);
uint8_t MCP47FEB22_GetGain(DAC_TypeDef dac, int channel);
void MCP47FEB22_SetGain(DAC_TypeDef dac, int val0, int val1);
uint8_t MCP47FEB22_GetGainEp(DAC_TypeDef dac, int channel);
uint8_t MCP47FEB22_GetVref(DAC_TypeDef dac, uint8_t channel);
void MCP47FEB22_SetVref(DAC_TypeDef dac, uint8_t val0, uint8_t val1);
uint8_t MCP47FEB22_GetVrefEp(DAC_TypeDef dac, uint8_t channel);
uint16_t MCP47FEB22_GetValue(DAC_TypeDef dac, uint8_t channel);
void MCP47FEB22_AnalogWrite(DAC_TypeDef dac, uint16_t val0, uint16_t val1);
void MCP47FEB22_EEPROMWrite(DAC_TypeDef dac);



#endif
