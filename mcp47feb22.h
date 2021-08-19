#ifndef __MCP47FEB22_H
#define __MCP47FEB22_H

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



typedef struct {
	uint8_t deviceID;
	uint8_t devAddr;
	uint16_t vdd;
	I2C_HandleTypeDef hi2c;
} MCP47FEB22_TypeDef;




void MCP47FEB22_Init(MCP47FEB22_TypeDef *dac, uint8_t deviceID, I2C_HandleTypeDef hi2c);
void  MCP47FEB22_UnlockSALCK(MCP47FEB22_TypeDef *dac);
void  MCP47FEB22_LockSALCK(MCP47FEB22_TypeDef *dac, uint8_t addr);
void MCP47FEB22_ChangeAddr(MCP47FEB22_TypeDef *dac, uint8_t addr);
uint8_t MCP47FEB22_GetPowerDown(MCP47FEB22_TypeDef *dac, int channel);
void MCP47FEB22_SetPowerDown(MCP47FEB22_TypeDef *dac, int val0, int val1);
uint8_t MCP47FEB22_GetPowerDownEp(MCP47FEB22_TypeDef *dac, int channel);
uint8_t MCP47FEB22_GetGain(MCP47FEB22_TypeDef *dac, int channel);
void MCP47FEB22_SetGain(MCP47FEB22_TypeDef *dac, int val0, int val1);
uint8_t MCP47FEB22_GetGainEp(MCP47FEB22_TypeDef *dac, int channel);
uint8_t MCP47FEB22_GetVref(MCP47FEB22_TypeDef *dac, uint8_t channel);
void MCP47FEB22_SetVref(MCP47FEB22_TypeDef *dac, uint8_t val0, uint8_t val1);
uint8_t MCP47FEB22_GetVrefEp(MCP47FEB22_TypeDef *dac, uint8_t channel);
uint16_t MCP47FEB22_GetValue(MCP47FEB22_TypeDef *dac, uint8_t channel);
void MCP47FEB22_AnalogWrite(MCP47FEB22_TypeDef *dac, uint16_t val0, uint16_t val1);
void MCP47FEB22_EEPROMWrite(MCP47FEB22_TypeDef *dac);



#endif
