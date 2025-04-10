/**
  ********************************************************************************
  * @brief   INA226 class
  * @author  Xilin Li
  ********************************************************************************
  * @details
			This library contains the necessary functions to initialize, read and
			write data to the TI INA226 Current/Power Monitor using the I2C
			protocol.
	******************************************************************************
	*/

#ifndef _INA226_H_
#define _INA226_H_

#ifdef	__cplusplus
extern "C" {
#endif


#define STM32F4

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#elif defined(STM32L4)
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#elif defined(STM32F3)
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_i2c.h"
#endif



#ifndef INA226_ADDRESS
#define INA226_ADDRESS	0x80
#endif

/*
#define INA226_CALIB_VAL		512
#define INA226_CURRENTLSB		0.0001F // A/bit
#define INA226_I2CTIMEOUT		10*/

#define INA226_CONFIG		0x00 // Configuration Register (R/W)
#define INA226_SHUNTV		0x01 // Shunt Voltage (R)
#define INA226_BUSV			0x02 // Bus Voltage (R)i
#define INA226_POWER		0x03 // Power (R)
#define INA226_CURRENT		0x04 // Current (R)
#define INA226_CALIB		0x05 // Calibration (R/W)
#define INA226_MASK			0x06 // Mask/Enable (R/W)
#define INA226_ALERTL		0x07 // Alert Limit (R/W)
#define INA226_MANUF_ID		0xFE // Manufacturer ID (R)
#define INA226_DIE_ID		0xFF // Die ID (R)

#define INA226_MODE_POWER_DOWN			(0<<0) // Power-Down
#define INA226_MODE_TRIG_SHUNT_VOLTAGE	(1<<0) // Shunt Voltage, Triggered
#define INA226_MODE_TRIG_BUS_VOLTAGE	(2<<0) // Bus Voltage, Triggered
#define INA226_MODE_TRIG_SHUNT_AND_BUS	(3<<0) // Shunt and Bus, Triggered
#define INA226_MODE_POWER_DOWN2			(4<<0) // Power-Down
#define INA226_MODE_CONT_SHUNT_VOLTAGE	(5<<0) // Shunt Voltage, Continuous
#define INA226_MODE_CONT_BUS_VOLTAGE	(6<<0) // Bus Voltage, Continuous
#define INA226_MODE_CONT_SHUNT_AND_BUS	(7<<0) // Shunt and Bus, Continuous

// Shunt Voltage Conversion Time
#define INA226_VSH_140uS			(0<<3)
#define INA226_VSH_204uS			(1<<3)
#define INA226_VSH_332uS			(2<<3)
#define INA226_VSH_588uS			(3<<3)
#define INA226_VSH_1100uS			(4<<3)
#define INA226_VSH_2116uS			(5<<3)
#define INA226_VSH_4156uS			(6<<3)
#define INA226_VSH_8244uS			(7<<3)

// Bus Voltage Conversion Time (VBUS CT Bit Settings[6-8])
#define INA226_VBUS_140uS			(0<<6)
#define INA226_VBUS_204uS			(1<<6)
#define INA226_VBUS_332uS			(2<<6)
#define INA226_VBUS_588uS			(3<<6)
#define INA226_VBUS_1100uS			(4<<6)
#define INA226_VBUS_2116uS			(5<<6)
#define INA226_VBUS_4156uS			(6<<6)
#define INA226_VBUS_8244uS			(7<<6)

// Averaging Mode (AVG Bit Settings[9-11])
#define INA226_AVG_1				(0<<9)
#define INA226_AVG_4				(1<<9)
#define INA226_AVG_16				(2<<9)
#define INA226_AVG_64				(3<<9)
#define INA226_AVG_128				(4<<9)
#define INA226_AVG_256				(5<<9)
#define INA226_AVG_512				(6<<9)
#define INA226_AVG_1024				(7<<9) 

// Reset Bit (RST bit [15])
#define INA226_RESET_ACTIVE			(1<<15)
#define INA226_RESET_INACTIVE		(0<<15)

// Mask/Enable Register
#define INA226_MER_SOL				(1<<15) // Shunt Voltage Over-Voltage
#define INA226_MER_SUL				(1<<14) // Shunt Voltage Under-Voltage
#define INA226_MER_BOL				(1<<13) // Bus Voltagee Over-Voltage
#define INA226_MER_BUL				(1<<12) // Bus Voltage Under-Voltage
#define INA226_MER_POL				(1<<11) // Power Over-Limit
#define INA226_MER_CNVR				(1<<10) // Conversion Ready
#define INA226_MER_AFF				(1<<4)  // Alert Function Flag
#define INA226_MER_CVRF				(1<<3)  // Conversion Ready Flag
#define INA226_MER_OVF				(1<<2)  // Math Overflow Flag
#define INA226_MER_APOL				(1<<1)  // Alert Polarity Bit
#define INA226_MER_LEN				(1<<0)  // Alert Latch Enable

//#define INA226_MANUF_ID_DEFAULT	0x5449
//#define INA226_DIE_ID_DEFAULT		0x2260

class INA226{
public:
	//INA226();
	INA226(I2C_HandleTypeDef* i2c_handle,float current_lsb,float shunt_resistant, uint8_t address);
	INA226(I2C_HandleTypeDef* i2c_handle,uint8_t address);

	float Get_Bus_Voltage();
	float Get_Current();
	float Get_Power();
	float Get_Shunt_Voltage();

	uint16_t Get_Bus_Voltage_Raw();
	uint16_t Get_Current_Raw();
	uint16_t Get_Power_Raw();
	uint16_t Get_Shunt_Voltage_Raw();

	uint8_t Set_Config(uint16_t config_word);
	uint16_t Get_Config();
	//uint16_t Get_Shunt_Voltage_Reg();
	//uint16_t Get_Bus_Voltage_Reg();
	//uint16_t Get_Power_Reg();
	uint8_t Set_Calibration_Reg(uint16_t config_word);
	uint16_t Get_Calibration_Reg();
	uint16_t Get_Current_Reg();
	uint16_t Get_Manufacture_ID();
	uint16_t Get_Die_ID();
	uint8_t Set_Mask_Enable(uint16_t config_word);
	uint16_t Get_Mask_Enable();
	uint8_t Set_Alert_Limit(uint16_t config_word);
	uint16_t Get_Alert_Limit();

	void Set_R_Shunt_And_A_Max(float shunt_resistant,float a_max);


private:
	const uint32_t time_out_ = 10;
	const float voltage_lsb_ = 0.00125;
	const float shunt_voltage_lsb_ = 0.0000025;
	float shunt_resistant_ = 0.1f;
	uint16_t calibration_;
	uint8_t address_ = 0x80;
	float current_lsb_;
	I2C_HandleTypeDef* i2c_handle_ = nullptr;
};

/*
uint16_t INA226_getBusVRaw(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getCurrentRaw(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getPowerRaw(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getShuntVRaw(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);

uint8_t INA226_setConfig(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord);
uint16_t INA226_getConfig(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getShuntVReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getBusVReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getPowerReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint8_t INA226_setCalibrationReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord);
uint16_t INA226_getCalibrationReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getCurrentReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getManufID(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint16_t INA226_getDieID(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint8_t INA226_setMaskEnable(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord);
uint16_t INA226_getMaskEnable(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);
uint8_t INA226_setAlertLimit(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord);
uint16_t INA226_getAlertLimit(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress);*/

#ifdef __cplusplus
} // extern "C"
#endif

#endif
