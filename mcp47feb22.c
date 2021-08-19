#include "mcp47feb22.h"
#include "stdio.h"
#include "i2c.h"


#define word(x,y) (((x) << 8) | (y))
#define lowByte(x) ((x) & 0b00001111)


static unsigned char 	readBuffer[5];
static unsigned char _buffer[5];
static uint8_t      _dev_address;
static uint8_t      _deviceID;
static uint8_t      _intVref[2];
static uint8_t      _gain[2];
static uint8_t      _powerDown[2];
static uint16_t     _values[2];
static uint16_t     _valuesEp[2];
static uint8_t      _intVrefEp[2];
static uint8_t      _gainEp[2];
static uint8_t      _powerDownEp[2];
static uint8_t      _wiperLock[2];
static uint16_t     _vdd;


static void _ReadEpAddr(MCP47FEB22_TypeDef *dac, uint8_t REG, unsigned char buffer[5]);
static void _ReadAddr(MCP47FEB22_TypeDef *dac, uint8_t REG, unsigned char buffer[5]);
static void _FastWrite(MCP47FEB22_TypeDef *dac, uint8_t REG, uint16_t DATA);
static void _WriteAddr(MCP47FEB22_TypeDef *dac, uint8_t REG, uint8_t data);

//DEVICE ID DEFAULT = 0
void MCP47FEB22_Init(MCP47FEB22_TypeDef *dac, uint8_t deviceID, I2C_HandleTypeDef hi2c){//uint8_t deviceID = 0x00) {
	dac->deviceID = deviceID;
	dac->devAddr = (BASE_ADDR | (dac->deviceID));
	dac->vdd = defaultVDD;
	dac->hi2c = hi2c;
}

static void _ReadEpAddr(MCP47FEB22_TypeDef *dac, uint8_t REG, unsigned char buffer[5]) {
	uint8_t readReg = 0x80 | (READ | REG);// << 3);
	
	HAL_I2C_Master_Transmit(&dac->hi2c, dac->devAddr<<1, &readReg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&dac->hi2c, dac->devAddr<<1, buffer, 2, HAL_MAX_DELAY);
}

static void _ReadAddr(MCP47FEB22_TypeDef *dac, uint8_t REG, unsigned char buffer[5]) {
	uint8_t readReg = (READ | REG);// << 3);
	HAL_I2C_Master_Transmit(&dac->hi2c, dac->devAddr<<1, &readReg, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&dac->hi2c, dac->devAddr<<1, buffer, 2, HAL_MAX_DELAY);
}


static void _FastWrite(MCP47FEB22_TypeDef *dac, uint8_t REG, uint16_t DATA) {
	uint8_t payload[8];
	payload[0] = REG | WRITE;
	payload[1] = (DATA >> 8) & 0xFF;
	payload[2] = (DATA & 0xFF);
	HAL_I2C_Master_Transmit(&dac->hi2c, dac->devAddr<<1, (uint8_t*)&payload, 3, HAL_MAX_DELAY);
}


static void _WriteAddr(MCP47FEB22_TypeDef *dac, uint8_t REG, uint8_t data) {
	uint8_t payload[3];
	payload[0] = REG | WRITE;
	if (REG == GAIN_REG) {
		payload[2] = 0;
		payload[1] = data;
	} else {
		payload[1] = 0;
		payload[2] = data;
	}
	HAL_I2C_Master_Transmit(&dac->hi2c, dac->devAddr<<1, (uint8_t*)&payload, 3, HAL_MAX_DELAY);

}


void MCP47FEB22_UnlockSALCK(MCP47FEB22_TypeDef *dac) {
	//SET HVC PIN LOW
	_WriteAddr(dac, UNLOCK_SALCK, 0);
	// SET HVC PIN LOW
}

void MCP47FEB22_LockSALCK(MCP47FEB22_TypeDef *dac, uint8_t addr) {
	//SET HVC PIN HIGH
	MCP47FEB22_TypeDef BIAS_DAC = {
		addr,
		BASE_ADDR | addr,
		dac->vdd,
		dac->hi2c,
	};

	_WriteAddr(&BIAS_DAC, LOCK_SALCK, 0);
	//SET HVC PIN LOW
}

void MCP47FEB22_ChangeAddr(MCP47FEB22_TypeDef *dac, uint8_t addr) {
	//BSP_LCD_DisplayString((uint8_t*)"HVC ON... Unlocking SALCK", 0, 45, SMALL_FONT);
	//HAL_Delay(10000);
	
	//MCP47FEB22_UnlockSALCK(dac);

	//BSP_LCD_DisplayString((uint8_t*)"HVC OFF...send addr change", 0, 60, SMALL_FONT);
	//HAL_Delay(10000);

	//_WriteAddr(dac, 0b11010000, BASE_ADDR|addr);

	//BSP_LCD_DisplayString((uint8_t*)"HVC ON... Locking SALCK", 0, 75, SMALL_FONT);
	//HAL_Delay(10000);

	//MCP47FEB22_LockSALCK(dac, addr);

	//BSP_LCD_DisplayString((uint8_t*)"HVC OFF...", 0, 90, SMALL_FONT);
	//HAL_Delay(10000);

}

uint8_t MCP47FEB22_GetPowerDown(MCP47FEB22_TypeDef *dac, int channel) {
	_ReadAddr(dac, PD_REG, readBuffer);
	_powerDown[0] = (readBuffer[1] & 0B00000011);
	_powerDown[1] = (readBuffer[1] & 0B00001100) >> 2;
	return (channel == 0) ? _powerDown[0] : _powerDown[1];
}

void MCP47FEB22_SetPowerDown(MCP47FEB22_TypeDef *dac, int val0, int val1) {
	_WriteAddr(dac, PD_REG, (val0 | val1<<2));
}


uint8_t MCP47FEB22_GetPowerDownEp(MCP47FEB22_TypeDef *dac, int channel) {
	_ReadEpAddr(dac, PD_REG, readBuffer);
	_powerDownEp[0] = (readBuffer[1] & 0B00000011);
	_powerDownEp[1] = (readBuffer[1] & 0B00001100) >> 2;
	return (channel == 0) ? _powerDownEp[0] : _powerDownEp[1];
}


uint8_t MCP47FEB22_GetGain(MCP47FEB22_TypeDef *dac, int channel) {
	uint8_t buff[5] = {0};
	_ReadAddr(dac, GAIN_REG, buff);
	_gain[0] = (buff[0] & 0B00000001);
	_gain[1] = (buff[0] & 0B00000010)>>1;
	return (channel == 0) ? _gain[0] : _gain[1];
}


void MCP47FEB22_SetGain(MCP47FEB22_TypeDef *dac, int val0, int val1) {
	_WriteAddr(dac, GAIN_REG, (val0 | (val1<<1)));
}

uint8_t MCP47FEB22_GetGainEp(MCP47FEB22_TypeDef *dac, int channel) {
	_ReadEpAddr(dac, GAIN_REG, readBuffer);
	_gainEp[0] = (readBuffer[0] & 0B00000001);
	_gainEp[1] = (readBuffer[0] & 0B00000010)>>1;
	return (channel == 0) ? _gainEp[0] : _gainEp[1];
}

uint8_t MCP47FEB22_GetVref(MCP47FEB22_TypeDef *dac, uint8_t channel) {//uint8_t channel) {
	_ReadAddr(dac,VREF_REG, readBuffer);
	_intVref[0] = (readBuffer[1] & 0b00000011);
	_intVref[1] = (readBuffer[1] & 0b00001100) >> 2;
	return (channel == 0) ? _intVref[0] : _intVref[1];
}

void MCP47FEB22_SetVref(MCP47FEB22_TypeDef *dac, uint8_t val0, uint8_t val1) {
	_WriteAddr(dac, VREF_REG, (val0 | (val1<<2)));
}

uint8_t MCP47FEB22_GetVrefEp(MCP47FEB22_TypeDef *dac, uint8_t channel) {//uint8_t channel) {
	_ReadEpAddr(dac, VREF_REG, readBuffer);
	_intVrefEp[0] = (readBuffer[1] & 0b00000011);
	_intVrefEp[1] = (readBuffer[1] & 0b00001100) >> 2;
	return (channel == 0) ? _intVrefEp[0] : _intVrefEp[1];
}

uint16_t MCP47FEB22_GetValue(MCP47FEB22_TypeDef *dac, uint8_t channel) {
	_ReadAddr(dac, (channel << 3), readBuffer);
	return word((readBuffer[0] & 0B00001111), readBuffer[1]);
}

void MCP47FEB22_AnalogWrite(MCP47FEB22_TypeDef *dac, uint16_t val0, uint16_t val1) {
	val0 &= 0xFFF;
	val1 &= 0xFFF; //Prevent going over 4095
	_FastWrite(dac, DAC0_REG, val0);
	_FastWrite(dac, DAC1_REG, val1);
}

void MCP47FEB22_EEPROMWrite(MCP47FEB22_TypeDef *dac) {
	_FastWrite(dac, DAC0_EP_REG, MCP47FEB22_GetValue(dac, 0));
	_FastWrite(dac, DAC1_EP_REG, MCP47FEB22_GetValue(dac, 1));

	volatile uint16_t vref0 = MCP47FEB22_GetVref(dac, 0);

	volatile uint16_t vref1 = MCP47FEB22_GetVref(dac, 1);
	_FastWrite(dac, VREF_EP_REG, (vref0|vref1<<2));//(MCP47FEB22_GetVref(dac,0) | MCP47FEB22_GetVref(dac,1)<<2));
	_FastWrite(dac, GAIN_EP_REG, (MCP47FEB22_GetGain(dac, 0) | MCP47FEB22_GetGain(dac, 1)<<1)<<8);
	_FastWrite(dac, PD_EP_REG, (MCP47FEB22_GetPowerDown(dac, 0) | MCP47FEB22_GetPowerDown(dac, 1)<<2));
}
