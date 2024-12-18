#include "mainpp.h"
#include "Ina219.h"
#include "stdio.h"
#include "Ads1256/ads1256.h"
#include "string.h"
#include "math.h"

INA219 *hina219;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

uint8_t rx1_buf[100];
uint8_t rx2_buf[100];

ADS125X_t ads1256;

uint8_t control_start = 0;
uint8_t control_mode = 0;
uint8_t read_ads1256 = 0;

//bk1697 upload
//uint8_t bk1697_connected_ = 0;
//uint8_t bk1697_screen_locked;
//float bk1697_voltage = 0.0;
//float bk1697_current = 0.0;

//bk1697 command
uint8_t register_changed;
uint8_t bk1697_setpoint_changed;
uint16_t fan_speed_setpoint;

Command_t pre_cmd_ = DEFAULT_COMMAND;
Command_t current_cmd_ = DEFAULT_COMMAND;

Sensor_t sensor = DEFAULT_SENSOR;

/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){

	}else if(huart->Instance == USART2){

	}

}*/

void rx485SendData (uint8_t *data, uint16_t size)
{
	// Pull DE high to enable TX operation
	HAL_GPIO_WritePin(RS485_TX_EN_GPIO_Port, RS485_TX_EN_Pin, GPIO_PIN_SET);
	//HAL_Delay(2);
	HAL_UART_Transmit(&huart2, data, size , 10);
	// Pull RE Low to enable RX operation
	HAL_GPIO_WritePin(RS485_TX_EN_GPIO_Port, RS485_TX_EN_Pin, GPIO_PIN_RESET);
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == USART1){
		if(Size!=sizeof(Command_t)){
			return;
		}

		memcpy( &current_cmd_, rx1_buf, Size);
		if(strcmp(current_cmd_.header,"SYR")!=0)
			return;
		if(strcmp(current_cmd_.tailer,"COE\n")!=0)
			return;


		register_changed = current_cmd_.bool_register^pre_cmd_.bool_register;
		pre_cmd_.bool_register = current_cmd_.bool_register;


		if(current_cmd_.bk1697_set_voltage != pre_cmd_.bk1697_set_voltage ){
			bk1697_setpoint_changed = 1;
		}

		fan_speed_setpoint = current_cmd_.fan_speed_setpoint;
		/*
		if(current_cmd_.fan_speed_setpoint != pre_cmd_.fan_speed_setpoint ){
			//fan_speed_setpoint_changed = 1;
		}*/

		//rx485SendData(rx1_buf,Size);


		//HAL_UART_Transmit(&huart1, rx1_buf, Size , 100);
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buf, 100);
	}else if(huart->Instance == USART2){
		//HAL_UART_Transmit(&huart1, rx2_buf, Size, 100);
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx2_buf, 100);

		/*
		switch(bk1697_lable_){
		case LOCK_SCREEN:
			if(Size == 3 && rx2_buf[0]=='O' && rx2_buf[1]=='K'){
				bk1697_count_ = 0;
				bk1697_screen_locked = 1;
			}
			break;
		case UNLOCK_SCREEN:
			if(Size == 3 && rx2_buf[0]=='O' && rx2_buf[1]=='K'){
				bk1697_count_ = 0;
				bk1697_screen_locked = 0;
			}
			break;
		case SET_VOLTAGE:
			if(Size == 3 && rx2_buf[0]=='O' && rx2_buf[1]=='K'){
				bk1697_count_ = 0;
			}
			break;
		case INQUIRE_DATA:
			if(Size == 13 && rx2_buf[10] && rx2_buf[11]=='K'){
				bk1697_count_ = 0;
				bk1697_voltage = (rx2_buf[0]-'0')*10 + (rx2_buf[1]-'0') + ((rx2_buf[2]-'0')*10 + (rx2_buf[3]-'0'))*0.01;
				bk1697_current = (rx2_buf[4]-'0') + ((rx2_buf[5]-'0')*100 + (rx2_buf[6]-'0')*10 + (rx2_buf[7]-'0'))*0.001;
			}
			break;
		default:
			break;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx2_buf, 100);*/
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1){
		/*
		if(bk1697_screen_lock_changed){
			if(current_cmd_.bk1697_screen_lock == 1){
				bk1697_lable_ = LOCK_SCREEN;
				rx485SendData((uint8_t*)"SESS00\r",7);
			}else{
				bk1697_lable_ = UNLOCK_SCREEN;
				rx485SendData((uint8_t*)"ENDS00\r",7);
			}
			bk1697_screen_lock_changed = 0;
		}else if(bk1697_setpoint_changed){
			bk1697_lable_ = SET_VOLTAGE;
			char data[10];
			data[9]='\r';
			sprintf(data,"VOLT00%03d",int(current_cmd_.bk1697_setpoint*10));
			rx485SendData((uint8_t*)data,10);
			bk1697_setpoint_changed = 0;
		}else{

			rx485SendData((uint8_t*)"GETS00\r",7);
		};*/

	}
}

void setup(){

	hina219 = new INA219(0x40,&hi2c1);
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	if (!hina219->begin())
	{
	  printf("Could not connect. Fix and Reboot");
	}

	printf("Initialize Ina219 Successfully\n");
	hina219->setBusVoltageRange(16);
	hina219->setMaxCurrentShunt(0.4, 0.001);



	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	//relay
	HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);

	//pwm signal
	TIM2->CCR1 = 0;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);


	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buf, 100);
	//HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx2_buf, 100);



	ads1256.csPort = ADS1256_CS_GPIO_Port;
	ads1256.csPin = ADS1256_CS_Pin;

	ads1256.drdyPort = ADS1256_DROY_GPIO_Port;
	ads1256.drdyPin = ADS1256_DROY_Pin;

	ads1256.resetPort = ADS1256_RST_GPIO_Port;
	ads1256.resetPin = ADS1256_RST_Pin;

	ads1256.vref = ADS125X_VREF;
	ads1256.oscFreq = ADS125X_OSC_FREQ;

	ADS125X_Init(&ads1256, &hspi2, ADS125X_DRATE_10SPS, ADS125X_PGA1, 0);

	HAL_TIM_Base_Start_IT(&htim1);

}

void loop(){

	auto start_time = HAL_GetTick();

	//check bk1697 is connected
	rx485SendData((uint8_t*)"GETS00\r",7);
	if(HAL_UART_Receive(&huart2, rx2_buf, 13, 50)==HAL_OK){
		if(rx2_buf[10]=='O' && rx2_buf[11]=='K'){
			sensor.bool_register |= 0x01;
			sensor.bk1697_voltage = (rx2_buf[0]-'0')*1000 + (rx2_buf[1]-'0')*100 + (rx2_buf[2]-'0')*10 + (rx2_buf[3]-'0');
			sensor.bk1697_current = (rx2_buf[4]-'0')*1000 + (rx2_buf[5]-'0')*100 + (rx2_buf[6]-'0')*10 + (rx2_buf[7]-'0');
		}
	}else{ //bk not connected
		sensor.bk1697_voltage = 0;
		sensor.bk1697_current = 0;
		//set last two bits ZERO
		sensor.bool_register &= 0b11111100;
	}

	//check bked panel is locked
	if(sensor.bool_register & 0x01){
		rx485SendData((uint8_t*)"CPAL00\r",7);
		if(HAL_UART_Receive(&huart2, rx2_buf, 72, 50)==HAL_OK){
			if(rx2_buf[69]=='O' && rx2_buf[70]=='K'){
				if(rx2_buf[67]=='1'){
					sensor.bool_register |= 0b00000010;
				}else if(rx2_buf[67]=='0'){
					sensor.bool_register &= 0b11111101;
				}
			}
		}
	}

	//bit 0: run state; bit 1: bk front panel lock; bit 2: read ads1256; bit 3: control mode;

	//run state changes
	if(register_changed & 0x01){
		control_start = current_cmd_.bool_register & 0x01;
		register_changed &= 0b11111110;

	}

	if((register_changed & 0x02) && (sensor.bool_register & 0x01)){
		if(current_cmd_.bool_register & 0x02){
			rx485SendData((uint8_t*)"SESS00\r",7);
		}else{
			rx485SendData((uint8_t*)"ENDS00\r",7);
		}
		HAL_UART_Receive(&huart2, rx2_buf, 3, 50);

		if(rx2_buf[0]=='O' && rx2_buf[1]=='K'){
			sensor.bool_register &= 0b11111101;
			sensor.bool_register |= (current_cmd_.bool_register>>1 & 1) <<1;
		}
		register_changed &= 0b11111101;
	}

	if(register_changed&0x04){
		read_ads1256 = current_cmd_.bool_register & 0x04;
		register_changed &= 0b11111011;
	}

	if(register_changed&0x08){
		control_mode = current_cmd_.bool_register & 0x08;
		register_changed &= 0b11110111;
	}




	if((sensor.bool_register & 0x01) && bk1697_setpoint_changed){
		//bk1697_lable_ = SET_VOLTAGE;
		char data[10];
		data[9]='\r';
		sprintf(data,"VOLT00%03d",int(current_cmd_.bk1697_set_voltage));
		rx485SendData((uint8_t*)data,10);
		bk1697_setpoint_changed = 0;
		HAL_UART_Receive(&huart2, rx2_buf, 3, 50);
	}



	//printf("hello\n");


	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	if(read_ads1256){
		ADS125X_ChannelDiff_Set(&ads1256, ADS125X_MUXP_AIN0, ADS125X_MUXN_AIN1);
		HAL_Delay(7);
		sensor.ads1256_channel_4 = (int16_t)ADS125X_ADC_ReadVolt(&ads1256)*10000;

		ADS125X_ChannelDiff_Set(&ads1256, ADS125X_MUXP_AIN2, ADS125X_MUXN_AIN3);
		HAL_Delay(7);
		sensor.ads1256_channel_1 = (int16_t)ADS125X_ADC_ReadVolt(&ads1256)*10000;

		ADS125X_ChannelDiff_Set(&ads1256, ADS125X_MUXP_AIN4, ADS125X_MUXN_AIN5);
		HAL_Delay(7);
		sensor.ads1256_channel_2 = (int16_t)ADS125X_ADC_ReadVolt(&ads1256)*10000;

		ADS125X_ChannelDiff_Set(&ads1256, ADS125X_MUXP_AIN6, ADS125X_MUXN_AIN7);
		sensor.ads1256_channel_3 = (int16_t)ADS125X_ADC_ReadVolt(&ads1256)*10000;
	}

	HAL_UART_Transmit(&huart1, (uint8_t*)(&sensor), sizeof(Sensor_t), 10);

		//volt[0] = ADS125X_ADC_ReadVolt(&ads1256);


	//HAL_UART_Transmit(&huart2, (uint8_t*)data, 5, 100);
	//auto vbus = hina219->getBusVoltage();
	//auto vshunt = hina219->getCurrent();
	//auto current = hina219->getShuntVoltage();

	//printf("voltage: %f\t Shunt voltage: %f\t current: %f\n",vbus,vshunt,current);

	while(HAL_GetTick()-start_time<500);
}
