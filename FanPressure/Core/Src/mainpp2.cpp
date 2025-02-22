//#include "mainpp.h"
//#include "Ina219.h"
//#include "stdio.h"
//#include "string.h"
//#include "math.h"
////#include "Ads1256/ads1256.h"
//#include "Ina226/ina226.h"
//#include "utility.h"
//#include "Pid/pid.h"
//
//const int SPEED_BUFFER_SIZE = 10;
//const int RX_BUF_SIZE = 100;
//const int SPEED_MAX_INCREMENT = 4;
////INA219 *hina219;
//extern I2C_HandleTypeDef hi2c1;
//extern I2C_HandleTypeDef hi2c2;
//
//extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
//
//extern SPI_HandleTypeDef hspi2;
//
//extern TIM_HandleTypeDef htim4;
//extern TIM_HandleTypeDef htim3;
//extern TIM_HandleTypeDef htim2;
//
//uint32_t pulse[SPEED_BUFFER_SIZE];
//
//uint8_t rx1_buf[RX_BUF_SIZE];
//uint8_t rx2_buf[RX_BUF_SIZE];
//
//uint32_t loop_index=0;
//int pulse_index = 0;
//uint8_t speed_increment_count = 0;
//
////uint16_t pwm_value = 0;
//
//uint8_t error_code;
//
////ADS125X_t ads1256;
//
//uint8_t run_state = 0;
//uint8_t control_mode = 0;
//uint8_t read_ads1256 = 0;
//
////bk1697 upload
//uint8_t bk1697_connected = 0;
//uint8_t bk1697_connected_count = 0;
////uint8_t bk1697_screen_locked;
////float bk1697_voltage = 0.0;
////float bk1697_current = 0.0;
//
////bk1697 command
//uint8_t register_changed;
////uint8_t bk1697_setpoint3_changed;
//
//
//Command_t pre_cmd_ = DEFAULT_COMMAND;
//Command_t current_cmd_ = DEFAULT_COMMAND;
//
//Parameter_t current_parameter_ = DEFAULT_PARAMETER;
//
//Sensor_t sensor = DEFAULT_SENSOR;
//
//PID<float>* pid;
//
//float speed_frequency = 0.0;
//float speed_pwm = 0.0;
//float speed_setpoint = 0.0;
///*
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	if(huart->Instance == USART1){
//
//	}else if(huart->Instance == USART2){
//
//	}
//
//}*/
//
//
////write to the 7th sector, starting address is 0x08060000
//uint32_t Flash_Write_Data (uint32_t *Data, uint16_t numberofwords)
//{
//
//	static FLASH_EraseInitTypeDef EraseInitStruct;
//	uint32_t SECTORError;
//	int sofar=0;
//
//
//	 /* Unlock the Flash to enable the flash control register access *************/
//	  HAL_FLASH_Unlock();
//
//	  /* Erase the user Flash area */
//
//	  /* Get the number of sector to erase from 1st sector */
//
//	  uint32_t StartSector = 7;
//	  //uint32_t EndSectorAddress = StartSectorAddress + numberofwords*4;
//	  uint32_t EndSector = 7;
//
//	  /* Fill EraseInit structure*/
//	  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
//	  EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
//	  EraseInitStruct.Sector        = StartSector;
//	  EraseInitStruct.NbSectors     = (EndSector - StartSector) + 1;
//
//	  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
//	     you have to make sure that these data are rewritten before they are accessed during code
//	     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
//	     DCRST and ICRST bits in the FLASH_CR register. */
//	  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
//	  {
//		  return HAL_FLASH_GetError ();
//	  }
//
//	  /* Program the user Flash area word by word
//	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
//
//	  uint32_t StartSectorAddress = 0x08060000;
//	   while (sofar<numberofwords)
//	   {
//	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress, Data[sofar]) == HAL_OK)
//	     {
//	    	 StartSectorAddress += 4;  // use StartPageAddress += 2 for half word and 8 for double word
//	    	 sofar++;
//	     }
//	     else
//	     {
//	       /* Error occurred while writing data in Flash memory*/
//	    	 return HAL_FLASH_GetError ();
//	     }
//	   }
//
//	  /* Lock the Flash to disable the flash control register access (recommended
//	     to protect the FLASH memory against possible unwanted operation) *********/
//	  HAL_FLASH_Lock();
//
//	   return 0;
//}
//
//void Flash_Read_Data (uint32_t *RxBuf, uint16_t numberofwords)
//{
//	uint32_t StartSectorAddress = 0x08060000;
//	for(int i=0;i<numberofwords;++i)
//	{
//		*RxBuf = *(__IO uint32_t *)StartSectorAddress;
//		StartSectorAddress += 4;
//		RxBuf++;
//	}
//}
//
//float gpalToFloat(uint8_t*data ){
//	uint8_t temp[4];
//
//	for(uint8_t i=0;i<4;++i){
//		switch(data[i] & 0b01111111){
//		case 0b0111111:
//			temp[i]=0;break;
//		case 0b0000110:
//			temp[i]=1;break;
//		case 0b1011011:
//			temp[i]=2;break;
//		case 0b1001111:
//			temp[i]=3;break;
//		case 0b1100110:
//			temp[i]=4;break;
//		case 0b1101101:
//			temp[i]=5;break;
//		case 0b1111101:
//			temp[i]=6;break;
//		case 0b0000111:
//		case 0b0010001:
//			temp[i]=7;break;
//		case 0b1111111:
//			temp[i]=8;break;
//		case 0b1101111:
//			temp[i]=9;break;
//		default:
//			temp[i]=0;break;
//		}
//
//	}
//
//	float multi = 1.0;
//	if(data[0]>>7)multi = 0.001;
//	else if(data[1]>>7)multi = 0.01;
//	else if(data[2]>>7)multi = 0.1;
//
//	return (temp[0]*1000+ temp[1]*100+ temp[2]*10+ temp[3])*multi;
//
//
//
//}
//
//void currentVoltCurr(uint8_t* data, float* voltaget, float* current){
//
//}
//
//void rx485SendData (uint8_t *data, uint16_t size)
//{
//	// Pull DE high to enable TX operation
//	HAL_GPIO_WritePin(RS485_TX_EN_GPIO_Port, RS485_TX_EN_Pin, GPIO_PIN_SET);
//	//HAL_Delay(2);
//	HAL_UART_Transmit(&huart2, data, size , 10);
//	// Pull RE Low to enable RX operation
//	HAL_GPIO_WritePin(RS485_TX_EN_GPIO_Port, RS485_TX_EN_Pin, GPIO_PIN_RESET);
//	HAL_Delay(100);
//}
//
//
//void calcVelocity(){
//
//	//for(uint8_t i=0;i<SPEED_PIN_COUNT;++i){
//		if(speed_increment_count >= SPEED_MAX_INCREMENT){
//			speed_increment_count = 10;
//			memset(pulse,0,SPEED_BUFFER_SIZE);
//			sensor.fan_speed = 0;
//			return;
//		}
//
//		uint32_t s=pulse[(pulse_index+SPEED_BUFFER_SIZE-1)%SPEED_BUFFER_SIZE]-pulse[pulse_index];
//
//		if(s==0){
//			sensor.fan_speed = 0;
//		}
//		else{
//			speed_frequency = (SPEED_BUFFER_SIZE-1)*1000000.0/s;
//			//uint8_t speed_int = (uint8_t)speed;
//			//uint8_t speed_decimal = (uint8_t)((speed-speed_int)*100.0);
//
//			sensor.fan_speed = (uint16_t)speed_frequency*100;
//
//			//send_data[2*i+8]=speed_int;
//			//send_data[2*i+9]=speed_decimal;
//		}
//
//}
//
//
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
//	if(huart->Instance == USART1){
//		if(Size==sizeof(Command_t)){
//			memcpy( &current_cmd_, rx1_buf, Size);
//			if(strncmp(current_cmd_.header,"SYR",3)!=0){
//				HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buf, RX_BUF_SIZE);
//				return;
//			}
//			if(strncmp(current_cmd_.tailer,"COE\n",4)!=0){
//				HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buf, RX_BUF_SIZE);
//				return;
//			}
//			register_changed = current_cmd_.bool_register^pre_cmd_.bool_register;
//			//pre_cmd_.bool_register = current_cmd_.bool_register;
//
//			/*
//			if(current_cmd_.bk1697_set_voltage != pre_cmd_.bk1697_set_voltage ){
//
//				bk1697_setpoint_changed = 1;
//			}*/
//
//			memcpy(&pre_cmd_,&current_cmd_,sizeof(Command_t));
//			speed_setpoint = current_cmd_.fan_speed_setpoint/30.0f;
//
//		}else if(Size == sizeof(Parameter_t)){
//			memcpy( &current_parameter_, rx1_buf, Size);
//			if(strncmp(current_parameter_.header,"COE\n",4)!=0){
//				HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buf, RX_BUF_SIZE);
//				return;
//			}
//			if(strncmp(current_parameter_.tailer,"SYR\n",4)!=0){
//				HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buf, RX_BUF_SIZE);
//				return;
//			}
//			pid->set_tunings(current_parameter_.kp, current_parameter_.ki, current_parameter_.kd);
//			Flash_Write_Data((uint32_t*) (&current_parameter_), sizeof(Parameter_t)/4);
//		}
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buf, RX_BUF_SIZE);
//	}
//	else if(huart->Instance == USART2){
//		if(rx2_buf[Size-1] == '\r'){
//			bk1697_connected_count = 0;
//		}
//		if(Size==72){
//			if(rx2_buf[69]=='O' && rx2_buf[70]=='K'){
//				//bk1697_connected = 1;
//				//sensor.bool_register |= 0x01;
//				uint8_t temp[4];
//				for(int i=0;i<4;++i){
//					temp[i] = (rx2_buf[2*i]-'0')<<4 | (rx2_buf[2*i+1]-'0');
//				}
//
//				sensor.bk1697_voltage = gpalToFloat(temp) * 100;
//
//				for(int i=0;i<4;++i){
//					temp[i] = (rx2_buf[2*i+9]-'0')<<4 | (rx2_buf[2*i+10]-'0');
//				}
//				sensor.bk1697_current = gpalToFloat(temp) * 1000;
//
//
//				/*
//				if(rx2_buf[67]=='1'){
//					sensor.bool_register |= 0b00000010;
//				}else if(rx2_buf[67]=='0'){
//					sensor.bool_register &= 0b11111101;
//				}*/
//			}else{
//				sensor.bk1697_voltage = 0;
//				sensor.bk1697_current = 0;
//			}
//		}
//		memset(rx2_buf,0,RX_BUF_SIZE);
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx2_buf, 200);
//	}
//}
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	if(htim->Instance == TIM3){
//		//manual control
//		if(control_mode){
//			speed_pwm = current_cmd_.pwm_setpoint;
//			TIM2->CCR1 = (uint32_t)(current_cmd_.pwm_setpoint)*TIM2->ARR/1000;
//
//			/*
//			if(TIM2->CCR1 != current_cmd_.pwm_setpoint)
//				TIM2->CCR1 = current_cmd_.pwm_setpoint;*/
//			return;
//		}
//		if(!pid)return;
//		pid->compute();
//		TIM2->CCR1 = speed_pwm*TIM2->ARR;
//
//
//		//pid control
//
//
//
//		/*
//		if(bk1697_screen_lock_changed){
//			if(current_cmd_.bk1697_screen_lock == 1){
//				bk1697_lable_ = LOCK_SCREEN;
//				rx485SendData((uint8_t*)"SESS00\r",7);
//		  	}else{
//				bk1697_lable_ = UNLOCK_SCREEN;
//				rx485SendData((uint8_t*)"ENDS00\r",7);
//			}
//			bk1697_screen_lock_changed = 0;
//		}else if(bk1697_setpoint_changed){
//			bk1697_lable_ = SET_VOLTAGE;
//
//			char data[10];
//			data[9]='\r';
//			sprintf(data,"VOLT00%03d",int(current_cmd_.bk1697_setpoint*10));
//			rx485SendData((uint8_t*)data,10);
//			bk1697_setpoint_changed = 0;
//		}else{
//
//			rx485SendData((uint8_t*)"GETS00\r",7);
//		};*/
//
//	}else if(htim->Instance == TIM4){
//		calcVelocity();
//		sensor.pwm = speed_frequency*1000;
//		HAL_UART_Transmit(&huart1, (uint8_t*)(&sensor), sizeof(Sensor_t), 20);
//	}
//}
//
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//
//	if(GPIO_Pin == SPEED_Pin){
//		pulse[pulse_index]=micros();
//		++pulse_index;
//		pulse_index%=SPEED_BUFFER_SIZE;
//		speed_increment_count=0;
//	}
//}
//
//void setup(){
//
//
//	/*
//	hina219 = new INA219(0x40,&hi2c1);
//	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//	if (!hina219->begin())
//	{
//	  printf("Could not connect. Fix and Reboot");
//	}
//
//	printf("Initialize Ina219 Successfully\n");
//	hina219->setBusVoltageRange(16);
//	hina219->setMaxCurrentShunt(0.4, 0.001);*/
//
//
//	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//
//	//relay
//	HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);
//
//
//
//
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buf, RX_BUF_SIZE);
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx2_buf, RX_BUF_SIZE);
//
//	DWT_Init();
//
//
///*
//	ads1256.csPort = ADS1256_CS_GPIO_Port;
//	ads1256.csPin = ADS1256_CS_Pin;
//
//	ads1256.drdyPort = ADS1256_DROY_GPIO_Port;
//	ads1256.drdyPin = ADS1256_DROY_Pin;
//
//	ads1256.resetPort = ADS1256_RST_GPIO_Port;
//	ads1256.resetPin = ADS1256_RST_Pin;
//
//	ads1256.vref = ADS125X_VREF;
//	ads1256.oscFreq = ADS125X_OSC_FREQ;
//
//	ADS125X_Init(&ads1256, &hspi2, ADS125X_DRATE_10SPS, ADS125X_PGA1, 0);
//	*/
//
//	INA226_setConfig(&hi2c1, INA226_ADDRESS, INA226_MODE_CONT_SHUNT_AND_BUS | INA226_VBUS_140uS | INA226_VBUS_140uS | INA226_AVG_1024);
//
//	INA226_setCalibrationReg(&hi2c1, INA226_ADDRESS,INA226_CALIB_VAL);
//	HAL_Delay(10);
//
//	HAL_TIM_Base_Start_IT(&htim3);
//	HAL_TIM_Base_Start_IT(&htim4);
//	//pwm signal
//	TIM2->CCR1 = 0;
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//
//	Flash_Read_Data( (uint32_t*) (&current_parameter_), sizeof(Parameter_t)/4);
//
//	pid = new PID<float>(&speed_frequency,&speed_pwm,&speed_setpoint,current_parameter_.kp,current_parameter_.ki,current_parameter_.kd);
//	pid->set_output_limits(0.0f,1.0f);
//}
//
//void loop(){
//
//	auto start_time = HAL_GetTick();
//	speed_increment_count++;
//
//
//	if(bk1697_connected_count++>5){
//		bk1697_connected=0;
//		sensor.bool_register &= 0b11111110;
//		bk1697_connected_count = 10;
//		sensor.bk1697_voltage = 0;
//		sensor.bk1697_current = 0;
//	}else{
//		bk1697_connected=1;
//		sensor.bool_register |= 0x01;
//
//	}
//
//	if(loop_index & 0x0001){
//		rx485SendData((uint8_t*)"GPAL01\r",7);
//	}
//	loop_index++;
//
//	//check bk1697 is connected
//
//	/*
//	rx485SendData((uint8_t*)"GETS01\r",7);
//	if(HAL_UART_Receive(&huart2, rx2_buf, 10, 500)==HAL_OK){
//		if(rx2_buf[7]=='O' && rx2_buf[8]=='K'){
//			sensor.bool_register |= 0x01;
//			sensor.bk1697_voltage = (rx2_buf[0]-'0')*100 + (rx2_buf[1]-'0')*10 + (rx2_buf[2]-'0');
//			sensor.bk1697_current = (rx2_buf[3]-'0')*100 + (rx2_buf[4]-'0')*10 + (rx2_buf[5]-'0') ;
//		}
//	}else{ //bk not connected
//		sensor.bk1697_voltage = 0;
//		sensor.bk1697_current = 0;
//		//set last two bits ZERO
//		sensor.bool_register &= 0b11111100;
//	}*/
//
//	//check bked panel is locked
//
//	//memset(rx2_buf,0,100);
//
//
////	if(HAL_UART_Receive(&huart2, rx2_buf, 72, 400)==HAL_OK){
////		if(rx2_buf[69]=='O' && rx2_buf[70]=='K'){
////			bk1697_connected = 1;
////			sensor.bool_register |= 0x01;
////			uint8_t temp[4];
////			for(int i=0;i<4;++i){
////				temp[i] = (rx2_buf[2*i]-'0')<<4 | (rx2_buf[2*i+1]-'0');
////			}
////
////			sensor.bk1697_voltage = gpalToFloat(temp) * 100;
////
////			for(int i=0;i<4;++i){
////				temp[i] = (rx2_buf[2*i+9]-'0')<<4 | (rx2_buf[2*i+10]-'0');
////			}
////			sensor.bk1697_current = gpalToFloat(temp) * 100;
////
////
////			/*
////			if(rx2_buf[67]=='1'){
////				sensor.bool_register |= 0b00000010;
////			}else if(rx2_buf[67]=='0'){
////				sensor.bool_register &= 0b11111101;
////			}*/
////		}else{
////			bk1697_connected = 0;
////			sensor.bool_register &= 0b11111100;
////		}
////	}
//
//	//bit 0: run state; bit 1: bk front panel lock; bit 2: read ads1256; bit 3: control mode;
//
//	//run state changes
//	if((register_changed & 0x01) ){
//		run_state = current_cmd_.bool_register & 0x01;
//		HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, run_state?GPIO_PIN_SET:GPIO_PIN_RESET);
//		if(bk1697_connected ){
//			char data[8];
//			sprintf(data,"SOUT01%d\r",int(!run_state));
//			rx485SendData((uint8_t*)data,8);
//			HAL_Delay(200);
//
//			//bk1697_lable_ = SET_VOLTAGE;
//			if(run_state){
//				char data_power[10];
//				//data[9]='\r';
//				sprintf(data_power,"VOLT01%03d\r",int(current_cmd_.bk1697_set_voltage));
//				rx485SendData((uint8_t*)data_power,10);
//				HAL_Delay(200);
//			}
//			//bk1697_setpoint_changed = 0;
//
//
//			//HAL_UART_Receive(&huart2, rx2_buf, 3, 100);
//		}
//		register_changed &= 0b11111110;
//	}
//
//	/*
//	if((register_changed & 0x02) && (sensor.bool_register & 0x01)){
//		if(current_cmd_.bool_register & 0x02){
//			rx485SendData((uint8_t*)"SESS00\r",7);
//		}else{
//			rx485SendData((uint8_t*)"ENDS00\r",7);
//		}
//		HAL_UART_Receive(&huart2, rx2_buf, 3, 50);
//
//		if(rx2_buf[0]=='O' && rx2_buf[1]=='K'){
//			sensor.bool_register &= 0b11111101;
//			sensor.bool_register |= (current_cmd_.bool_register>>1 & 1) <<1;
//		}
//		register_changed &= 0b11111101;
//	}*/
//
//	if(register_changed&0x04){
//		read_ads1256 = (current_cmd_.bool_register & 0x04)>>2;
//		register_changed &= 0b11111011;
//	}
//
//	if(register_changed&0x08){
//		control_mode = (current_cmd_.bool_register & 0x08)>>3;
//
//		register_changed &= 0b11110111;
//	}
//
//
//
//	/*
//
//	if(bk1697_connected & bk1697_setpoint_changed){
//		//bk1697_lable_ = SET_VOLTAGE;
//		char data[10];
//		//data[9]='\r';
//		sprintf(data,"VOLT01%03d\r",int(current_cmd_.bk1697_set_voltage));
//		rx485SendData((uint8_t*)data,10);
//		bk1697_setpoint_changed = 0;
//		//HAL_UART_Receive(&huart2, rx2_buf, 3, 500);
//	}*/
//
//
//
//	//printf("hello\n");
//
//
//	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//
//	if(read_ads1256){
//		/*
//		ADS125X_ChannelDiff_Set(&ads1256, ADS125X_MUXP_AIN0, ADS125X_MUXN_AIN1);
//		HAL_Delay(7);
//		sensor.ads1256_channel_4 = (int16_t)ADS125X_ADC_ReadVolt(&ads1256)*10000;
//
//		ADS125X_ChannelDiff_Set(&ads1256, ADS125X_MUXP_AIN2, ADS125X_MUXN_AIN3);
//		HAL_Delay(7);
//		sensor.ads1256_channel_1 = (int16_t)ADS125X_ADC_ReadVolt(&ads1256)*10000;
//
//		ADS125X_ChannelDiff_Set(&ads1256, ADS125X_MUXP_AIN4, ADS125X_MUXN_AIN5);
//		HAL_Delay(7);
//		sensor.ads1256_channel_2 = (int16_t)ADS125X_ADC_ReadVolt(&ads1256)*10000;
//
//		ADS125X_ChannelDiff_Set(&ads1256, ADS125X_MUXP_AIN6, ADS125X_MUXN_AIN7);
//		sensor.ads1256_channel_3 = (int16_t)ADS125X_ADC_ReadVolt(&ads1256)*10000;*/
//	}
//
//	uint16_t bus_v = INA226_getBusVRaw(&hi2c1, INA226_ADDRESS);
//	uint16_t bus_c = INA226_getCurrentRaw(&hi2c1, INA226_ADDRESS);
//	uint16_t bus_p = INA226_getPowerRaw(&hi2c1, INA226_ADDRESS);
//	uint16_t shunt_v = INA226_getShuntVRaw(&hi2c1, INA226_ADDRESS);
//
//	sensor.fan_current = bus_c!=0xFF?bus_c:0;
//	sensor.fan_power = bus_p!=0xFF?bus_p:0;
//	sensor.fan_voltage = bus_v!=0xFF?bus_v:0;
//	sensor.fan_shunt_voltage = shunt_v!=0xFF?shunt_v:0;
//
//
//
//
//
//
//	//volt[0] = ADS125X_ADC_ReadVolt(&ads1256);
//
//
//	//HAL_UART_Transmit(&huart2, (uint8_t*)data, 5, 100);
//	//auto vbus = hina219->getBusVoltage();
//	//auto vshunt = hina219->getCurrent();
//	//auto current = hina219->getShuntVoltage();
//
//	//printf("voltage: %f\t Shunt voltage: %f\t current: %f\n",vbus,vshunt,current);
//
//	while(HAL_GetTick()-start_time<500);
//}
