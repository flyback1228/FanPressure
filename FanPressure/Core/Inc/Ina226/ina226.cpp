	
#include <Ina226/ina226.h>
	


INA226::INA226(I2C_HandleTypeDef* i2c_handle,float current_lsb,float shunt_resistant, uint8_t address = 0x80):i2c_handle_(i2c_handle),address_(address),current_lsb_(current_lsb),shunt_resistant_(shunt_resistant){
	calibration_ = 0.00512/(shunt_resistant * current_lsb);

	Set_Config(INA226_MODE_CONT_SHUNT_AND_BUS | INA226_VSH_140uS | INA226_VBUS_140uS | INA226_AVG_1024);

	Set_Calibration_Reg(calibration_);

}

INA226::INA226(I2C_HandleTypeDef* i2c_handle,uint8_t address = 0x80):i2c_handle_(i2c_handle),address_(address){
	Set_Config(INA226_MODE_CONT_SHUNT_AND_BUS | INA226_VSH_140uS | INA226_VBUS_140uS | INA226_AVG_1024);
}

float INA226::Get_Bus_Voltage(){
	return voltage_lsb_ * Get_Bus_Voltage_Raw();
}


float INA226::Get_Current(){
	return current_lsb_* Get_Current_Raw();
}


float INA226::Get_Power(){
	return 25*current_lsb_*Get_Power_Raw();
}


float INA226::Get_Shunt_Voltage(){
	return shunt_voltage_lsb_*Get_Shunt_Voltage_Raw();
}


uint8_t INA226::Set_Config(uint16_t config_word){
	uint8_t send_data[]={INA226_CONFIG,(config_word & 0xFF00) >> 8,(config_word & 0x00FF)};
	return HAL_I2C_Master_Transmit(i2c_handle_, address_, send_data, 3, time_out_);
}

uint16_t INA226::Get_Config(){
	uint8_t send_data[] = {INA226_CONFIG};
	uint8_t receive_data[2];
	HAL_I2C_Master_Transmit(i2c_handle_,address_, send_data, 1, time_out_);
	if (HAL_I2C_Master_Receive(i2c_handle_,address_, receive_data, 2, time_out_) != HAL_OK) return 0xFF;
	else return ((uint16_t)receive_data[0]<<8 | receive_data[1]);
}

uint16_t INA226::Get_Shunt_Voltage_Raw(){
	uint8_t send_data[] = {INA226_SHUNTV};
	uint8_t receive_data[2];
	HAL_I2C_Master_Transmit(i2c_handle_,address_, send_data, 1, time_out_);
	if (HAL_I2C_Master_Receive(i2c_handle_,address_, receive_data, 2, time_out_) != HAL_OK) return 0xFF;
	else return ((uint16_t)receive_data[0]<<8 | receive_data[1]);
}

uint16_t INA226::Get_Bus_Voltage_Raw(){
	uint8_t send_data[] = {INA226_BUSV};
	uint8_t receive_data[2];
	HAL_I2C_Master_Transmit(i2c_handle_,address_, send_data, 1, time_out_);
	if (HAL_I2C_Master_Receive(i2c_handle_,address_, receive_data, 2, time_out_) != HAL_OK) return 0xFF;
	else return ((uint16_t)receive_data[0]<<8 | receive_data[1]);
}

uint16_t INA226::Get_Power_Raw(){
	uint8_t send_data[] = {INA226_POWER};
	uint8_t receive_data[2];
	HAL_I2C_Master_Transmit(i2c_handle_,address_, send_data, 1, time_out_);
	if (HAL_I2C_Master_Receive(i2c_handle_,address_, receive_data, 2, time_out_) != HAL_OK) return 0xFF;
	else return ((uint16_t)receive_data[0]<<8 | receive_data[1]);
}

uint8_t INA226::Set_Calibration_Reg(uint16_t config_word){
	uint8_t send_data[]={INA226_CALIB,(config_word & 0xFF00) >> 8,(config_word & 0x00FF)};
	return HAL_I2C_Master_Transmit(i2c_handle_, address_, send_data, 3, time_out_);
}

uint16_t INA226::Get_Calibration_Reg(){
	uint8_t send_data[] = {INA226_CALIB};
	uint8_t receive_data[2];
	HAL_I2C_Master_Transmit(i2c_handle_,address_, send_data, 1, time_out_);
	if (HAL_I2C_Master_Receive(i2c_handle_,address_, receive_data, 2, time_out_) != HAL_OK) return 0xFF;
	else return ((uint16_t)receive_data[0]<<8 | receive_data[1]);
}

uint16_t INA226::Get_Current_Raw(){
	uint8_t send_data[] = {INA226_CURRENT};
	uint8_t receive_data[2];
	HAL_I2C_Master_Transmit(i2c_handle_,address_, send_data, 1, time_out_);
	if (HAL_I2C_Master_Receive(i2c_handle_,address_, receive_data, 2, time_out_) != HAL_OK) return 0xFF;
	else return ((uint16_t)receive_data[0]<<8 | receive_data[1]);
}

uint16_t INA226::Get_Manufacture_ID(){
	uint8_t send_data[] = {INA226_MANUF_ID};
	uint8_t receive_data[2];
	HAL_I2C_Master_Transmit(i2c_handle_,address_, send_data, 1, time_out_);
	if (HAL_I2C_Master_Receive(i2c_handle_,address_, receive_data, 2, time_out_) != HAL_OK) return 0xFF;
	else return ((uint16_t)receive_data[0]<<8 | receive_data[1]);
}

uint16_t INA226::Get_Die_ID(){
	uint8_t send_data[] = {INA226_DIE_ID};
	uint8_t receive_data[2];
	HAL_I2C_Master_Transmit(i2c_handle_,address_, send_data, 1, time_out_);
	if (HAL_I2C_Master_Receive(i2c_handle_,address_, receive_data, 2, time_out_) != HAL_OK) return 0xFF;
	else return ((uint16_t)receive_data[0]<<8 | receive_data[1]);
}

uint8_t INA226::Set_Mask_Enable(uint16_t config_word){
	uint8_t send_data[]={INA226_MASK,(config_word & 0xFF00) >> 8,(config_word & 0x00FF)};
	return HAL_I2C_Master_Transmit(i2c_handle_, address_, send_data, 3, time_out_);
}

uint16_t INA226::Get_Mask_Enable(){
	uint8_t send_data[] = {INA226_MASK};
	uint8_t receive_data[2];
	HAL_I2C_Master_Transmit(i2c_handle_,address_, send_data, 1, time_out_);
	if (HAL_I2C_Master_Receive(i2c_handle_,address_, receive_data, 2, time_out_) != HAL_OK) return 0xFF;
	else return ((uint16_t)receive_data[0]<<8 | receive_data[1]);
}

uint8_t INA226::Set_Alert_Limit(uint16_t config_word){
	uint8_t send_data[]={INA226_ALERTL,(config_word & 0xFF00) >> 8,(config_word & 0x00FF)};
	return HAL_I2C_Master_Transmit(i2c_handle_, address_, send_data, 3, time_out_);
}

uint16_t INA226::Get_Alert_Limit(){
	uint8_t send_data[1] = {INA226_ALERTL};
	uint8_t receive_data[2];
	HAL_I2C_Master_Transmit(i2c_handle_,address_, send_data, 1, time_out_);
	if (HAL_I2C_Master_Receive(i2c_handle_,address_, receive_data, 2, time_out_) != HAL_OK) return 0xFF;
	else return ((uint16_t)receive_data[0]<<8 | receive_data[1]);
}

void INA226::Set_R_Shunt_And_A_Max(float shunt_resistant,float a_max){
	shunt_resistant_=shunt_resistant;

	float current_lsb = a_max/32768;
	uint16_t cal = 0.00512/(shunt_resistant * current_lsb);
	uint16_t temp_cal = 1;

	while(cal>1){
		cal=cal/2;
		temp_cal*=2;
	}

	calibration_ = temp_cal;
	Set_Calibration_Reg(calibration_);
}

/*
uint16_t INA226_getBusVRaw(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
	return INA226_getBusVReg(I2CHandler, DevAddress);
}

uint16_t INA226_getCurrentRaw(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
	//return (INA226_getCurrentReg(I2CHandler, DevAddress)*INA226_CURRENTLSB_INV);
	return INA226_getCurrentReg(I2CHandler, DevAddress);
}

uint16_t INA226_getPowerRaw(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
	//return (INA226_getPowerReg(I2CHandler, DevAddress)*INA226_POWERLSB_INV);
	return INA226_getPowerReg(I2CHandler, DevAddress);
}

uint16_t INA226_getShuntVRaw(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
	//return (INA226_getPowerReg(I2CHandler, DevAddress)*INA226_POWERLSB_INV);
	return INA226_getShuntVReg(I2CHandler,DevAddress);
}
	
uint8_t INA226_setConfig(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord) {
	uint8_t SentTable[3];
	SentTable[0] = INA226_CONFIG;
	SentTable[1] = (ConfigWord & 0xFF00) >> 8;
	SentTable[2] = (ConfigWord & 0x00FF);
	return HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 3, INA226_I2CTIMEOUT);
}

uint16_t INA226_getConfig(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
	uint8_t SentTable[1] = {INA226_CONFIG};
	uint8_t ReceivedTable[2];
	HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
	if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
	else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint16_t INA226_getShuntVReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
	uint8_t SentTable[1] = {INA226_SHUNTV};
	uint8_t ReceivedTable[2];
	HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
	if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
	else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint16_t INA226_getBusVReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
	uint8_t SentTable[1] = {INA226_BUSV};
	uint8_t ReceivedTable[2];
	HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
	if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
	else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint8_t INA226_setCalibrationReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord) {
	uint8_t SentTable[3];
	SentTable[0] = INA226_CALIB;
	SentTable[1] = (ConfigWord & 0xFF00) >> 8;
	SentTable[2] = (ConfigWord & 0x00FF);
	return HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 3, INA226_I2CTIMEOUT);
}

uint16_t INA226_getCalibrationReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
	uint8_t SentTable[1] = {INA226_CALIB};
	uint8_t ReceivedTable[2];
	HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
	if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
	else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint16_t INA226_getPowerReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
	uint8_t SentTable[1] = {INA226_POWER};
	uint8_t ReceivedTable[2];
	HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
	if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
	else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint16_t INA226_getCurrentReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
	uint8_t SentTable[1] = {INA226_CURRENT};
	uint8_t ReceivedTable[2];
	HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
	if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
	else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint16_t INA226_getManufID(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
	uint8_t SentTable[1] = {INA226_MANUF_ID};
	uint8_t ReceivedTable[2];
	HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
	if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
	else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint16_t INA226_getDieID(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
	uint8_t SentTable[1] = {INA226_DIE_ID};
	uint8_t ReceivedTable[2];
	HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
	if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
	else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint8_t INA226_setMaskEnable(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord) {
	uint8_t SentTable[3];
	SentTable[0] = INA226_MASK;
	SentTable[1] = (ConfigWord & 0xFF00) >> 8;
	SentTable[2] = (ConfigWord & 0x00FF);
	return HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 3, INA226_I2CTIMEOUT);
}

uint16_t INA226_getMaskEnable(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
	uint8_t SentTable[1] = {INA226_MASK};
	uint8_t ReceivedTable[2];
	HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
	if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
	else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}

uint8_t INA226_setAlertLimit(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord) {
	uint8_t SentTable[3];
	SentTable[0] = INA226_ALERTL;
	SentTable[1] = (ConfigWord & 0xFF00) >> 8;
	SentTable[2] = (ConfigWord & 0x00FF);
	return HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 3, INA226_I2CTIMEOUT);
}

uint16_t INA226_getAlertLimit(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
	uint8_t SentTable[1] = {INA226_ALERTL};
	uint8_t ReceivedTable[2];
	HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
	if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
	else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}*/
