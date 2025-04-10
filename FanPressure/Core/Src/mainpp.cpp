#include "mainpp.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "Ina226/ina226.h"
#include "utility.h"
#include "Pid/pid.h"

// uart receive buf size for uart1 and uart2
const int RX_BUF_SIZE = 100;

//uart receive bufs
uint8_t rx1_buf[RX_BUF_SIZE];
uint8_t rx2_buf[RX_BUF_SIZE];

//variables to store speed data. pulse is to store the time when the external interupt happen. pulse_index is the index of pulse to write the current value.
const int SPEED_BUFFER_SIZE = 10;
uint32_t pulse[SPEED_BUFFER_SIZE];
int pulse_index = 0;

// speed_increment_count is increased by 1 in every loop, if it exceeds SPEED_MAX_INCREMENT, all elements in pulse are reset to zero
const int SPEED_MAX_INCREMENT = 8;
uint8_t speed_increment_count = 0;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

//extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;

//extern IWDG_HandleTypeDef hiwdg;

//voltage setpoint limit
uint8_t min_voltage = 0;
uint8_t max_voltage = 30;

// to indicate the run state, controlled by computer
uint8_t run_state = 0;
// to indicate the control mode (auto(value 0) or manual(value 1)), controlled by compute 1r
uint8_t control_mode = 0;



uint16_t bk1697_previous_volt = 0;
uint8_t bk1697_connected = 0;
uint8_t bk1697_increment_count = 10;
uint8_t bk1697_same_count = 0;

// to indicate the some state from command changed
uint8_t register_changed;


//default structures
Command_t pre_cmd_ = DEFAULT_COMMAND;
Command_t current_cmd_ = DEFAULT_COMMAND;
Parameter_t current_parameter_ = DEFAULT_PARAMETER;
Sensor_t sensor = DEFAULT_SENSOR;

// pid
PID<float>* pid;

//ina226
INA226* ina226;

// pid variables
float speed_frequency = 0.0;
float pid_bk_output = 0.0;
float speed_setpoint = 0.0;




/***
 * write to the 7th sector, starting address is 0x08060000
 * param data: data to write
 * param numberofwords: number of data in word to write, usually equals to Sizeof(data)/4
 */
uint32_t Flash_Write_Data (uint32_t *data, uint16_t numberofwords)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	int sofar=0;


	 /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Erase the user Flash area */

	  /* Get the number of sector to erase from 1st sector */

	  uint32_t StartSector = 7;
	  //uint32_t EndSectorAddress = StartSectorAddress + numberofwords*4;
	  uint32_t EndSector = 7;

	  /* Fill EraseInit structure*/
	  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	  EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	  EraseInitStruct.Sector        = StartSector;
	  EraseInitStruct.NbSectors     = (EndSector - StartSector) + 1;

	  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	     you have to make sure that these data are rewritten before they are accessed during code
	     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	     DCRST and ICRST bits in the FLASH_CR register. */
	  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	  {
		  return HAL_FLASH_GetError ();
	  }

	  /* Program the user Flash area word by word
	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	  uint32_t StartSectorAddress = 0x08060000;
	   while (sofar<numberofwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress, data[sofar]) == HAL_OK)
	     {
	    	 StartSectorAddress += 4;  // use StartPageAddress += 2 for half word and 8 for double word
	    	 sofar++;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  HAL_FLASH_Lock();

	   return 0;
}


/***
 * read data from 7th sector, starting address is 0x08060000
 * param data: data to read
 * param numberofwords: number of data in word to write, usually equals to Sizeof(data)/4
 */
void Flash_Read_Data (uint32_t *data, uint16_t numberofwords)
{
	uint32_t StartSectorAddress = 0x08060000;
	for(int i=0;i<numberofwords;++i)
	{
		*data = *(__IO uint32_t *)StartSectorAddress;
		StartSectorAddress += 4;
		data++;
	}
}


/***
 * convert a LCD show dada to a float for bk1697b
 */
float Gpal_To_Float(uint8_t*data ){
	uint8_t temp[4];

	for(uint8_t i=0;i<4;++i){
		switch(data[i] & 0b01111111){
		case 0b0111111:
			temp[i]=0;break;
		case 0b0000110:
			temp[i]=1;break;
		case 0b1011011:
			temp[i]=2;break;
		case 0b1001111:
			temp[i]=3;break;
		case 0b1100110:
			temp[i]=4;break;
		case 0b1101101:
			temp[i]=5;break;
		case 0b1111101:
			temp[i]=6;break;
		case 0b0000111:
		case 0b0010001:
			temp[i]=7;break;
		case 0b1111111:
			temp[i]=8;break;
		case 0b1101111:
			temp[i]=9;break;
		default:
			temp[i]=0;break;
		}

	}

	float multi = 1.0;
	if(data[0]>>7)multi = 0.001;
	else if(data[1]>>7)multi = 0.01;
	else if(data[2]>>7)multi = 0.1;

	return (temp[0]*1000+ temp[1]*100+ temp[2]*10+ temp[3])*multi;
}

/***
 * send data via rx485, the corresponding uart is uart2
 */
void Rx485_Send_Data (uint8_t *data, uint16_t size)
{
	// Pull DE high to enable TX operation
	HAL_GPIO_WritePin(RS485_TX_EN_GPIO_Port, RS485_TX_EN_Pin, GPIO_PIN_SET);
	//HAL_Delay(2);
	HAL_UART_Transmit(&huart2, data, size , 10);
	// Pull RE Low to enable RX operation
	HAL_GPIO_WritePin(RS485_TX_EN_GPIO_Port, RS485_TX_EN_Pin, GPIO_PIN_RESET);
	//HAL_Delay(100);
}

/***
 * calculate fan speed
 */
void Calc_Velocity(){
	if(speed_increment_count >= SPEED_MAX_INCREMENT){
		speed_increment_count = SPEED_MAX_INCREMENT;
		memset(pulse,0,SPEED_BUFFER_SIZE*sizeof(uint32_t));
		speed_frequency = 0;
		sensor.fan_speed = 0;
		return;
	}

	__disable_irq();
	uint32_t s1 = pulse[(pulse_index+SPEED_BUFFER_SIZE-1)%SPEED_BUFFER_SIZE];
	uint32_t s2 = pulse[pulse_index];
	__enable_irq();

	uint32_t s=s1-s2;

	if(s==0)
		speed_frequency = 0;
	else
		speed_frequency = (SPEED_BUFFER_SIZE-1)*1000000.0/s;

	sensor.fan_speed = (uint16_t)(speed_frequency*100);
}

/***
 * uart receive interrupt callback, override
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

	//uart1, from computer
	if(huart->Instance == USART1){
		if(Size==sizeof(Command_t)){
			memcpy( &current_cmd_, rx1_buf, Size);
			if(strncmp(current_cmd_.header,"SYR",3)==0 && strncmp(current_cmd_.tailer,"COE\n",4)==0){
				register_changed = current_cmd_.bool_register^pre_cmd_.bool_register;

				memcpy(&pre_cmd_,&current_cmd_,sizeof(Command_t));
				speed_setpoint = current_cmd_.fan_speed_setpoint/30.0f;

				bool vol_changed = false;
				if(((current_cmd_.voltage_range >> 8) & 0xFF) != max_voltage){
					max_voltage = (current_cmd_.voltage_range >> 8) & 0xFF;
					vol_changed = true;
				}
				if((current_cmd_.voltage_range & 0xFF) != min_voltage){
					min_voltage = current_cmd_.voltage_range & 0xFF;
					vol_changed = true;
				}
				if(vol_changed && pid){
					pid->set_output_limits(min_voltage, max_voltage);
				}
			}
		}
		if(Size == sizeof(Parameter_t)){
			memcpy( &current_parameter_, rx1_buf, Size);
			if(strncmp(current_parameter_.header,"COE\n",4)==0 && strncmp(current_parameter_.tailer,"SYR\n",4)==0){
				pid->set_tunings(current_parameter_.kp, current_parameter_.ki, current_parameter_.kd);

				Flash_Write_Data((uint32_t*) (&current_parameter_), sizeof(Parameter_t)/4);
				sensor.kp = current_parameter_.kp;
				sensor.ki = current_parameter_.ki;
				sensor.kd = current_parameter_.kd;
				sensor.speed_50 = current_parameter_.fan_speed_half;
				sensor.speed_100 = current_parameter_.fan_speed_full;
			}
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buf, RX_BUF_SIZE);
	}

	//uart2, from bk1697
	else if(huart->Instance == USART2){
		if(rx2_buf[Size-1] == '\r'){
			bk1697_increment_count = 0;
		}
		if(Size==72){
			if(rx2_buf[69]=='O' && rx2_buf[70]=='K'){
				uint8_t temp[4];
				for(int i=0;i<4;++i){
					temp[i] = (rx2_buf[2*i]-'0')<<4 | (rx2_buf[2*i+1]-'0');
				}

				sensor.bk1697_voltage = Gpal_To_Float(temp) * 100;

				for(int i=0;i<4;++i){
					temp[i] = (rx2_buf[2*i+9]-'0')<<4 | (rx2_buf[2*i+10]-'0');
				}
				sensor.bk1697_current = Gpal_To_Float(temp) * 1000;
			}else{
				sensor.bk1697_voltage = 0;
				sensor.bk1697_current = 0;
			}
		}
		memset(rx2_buf,0,Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx2_buf, RX_BUF_SIZE);
	}
}

/***
 * timeout interrupt callback, override
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	// TIM3 timeout, for control output calculate
	if(htim->Instance == TIM3){
		if(!bk1697_connected)
			return;

		bk1697_same_count++;

		if(!run_state){
			//bk1697_previous_volt = pid_bk_output*10;
			//set bk1697 with 1 because the command with 0 not work, may be due to the hardware set of bk1697
			if(bk1697_same_count > 4 || bk1697_previous_volt >0){
				if(bk1697_previous_volt>0){
					bk1697_previous_volt=0;
				}else
					bk1697_previous_volt=1;

				pid_bk_output = 0.0;
				char data[10];
				sprintf(data,"VOLT01%03d\r",bk1697_previous_volt);
				Rx485_Send_Data((uint8_t*)data,10);
				bk1697_same_count = 0;
			}
			return;
		}

		//manual control
		if(control_mode){
			if(current_cmd_.voltage!=bk1697_previous_volt){
				auto v = current_cmd_.voltage;
				if(v==0){
					if(bk1697_previous_volt==0){
						bk1697_previous_volt=1;
					}else{
						bk1697_previous_volt=0;
					}
				}else{
					bk1697_previous_volt = current_cmd_.voltage;
				}

				pid_bk_output = bk1697_previous_volt/10.0f;
				char data[10];
				sprintf(data,"VOLT01%03d\r",bk1697_previous_volt);
				Rx485_Send_Data((uint8_t*)data,10);
				bk1697_same_count = 0;
			}
			return;
		}
		if(!pid)
			return;
		pid->compute();

		if(bk1697_same_count > 4 || uint16_t(pid_bk_output*10)!=bk1697_previous_volt){
			bk1697_previous_volt = uint16_t(pid_bk_output*10);
			char data[10];
			sprintf(data,"VOLT01%03d\r",bk1697_previous_volt);
			Rx485_Send_Data((uint8_t*)data,10);
			bk1697_same_count = 0;
		}

	}
	//TIM4 timeout, speed calculation and upload the sensor data to host
	else if(htim->Instance == TIM4){
		Calc_Velocity();
		HAL_UART_Transmit(&huart1, (uint8_t*)(&sensor), sizeof(Sensor_t), 20);
		if(sensor.bool_register & 0x02)
			sensor.bool_register &= 0b11111101;
	}
}

/***
 * external gpio pin interrupt. record the time when the interrupt triggers. used for speed calculation
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == SPEED_Pin){
		static uint32_t last_interrupt_time = 0;
		uint32_t interrupt_time = micros();

		if ((interrupt_time - last_interrupt_time) > 2000) { // ignore pulses < 1 ms
			pulse[pulse_index++] = interrupt_time;
			pulse_index %= SPEED_BUFFER_SIZE;
			speed_increment_count=0;
			last_interrupt_time = interrupt_time;
		}

	}
}

/***
 * setup function, call once after all peripheral initializes
 */
void setup(){
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);



	DWT_Init();

	//INA226_setConfig(&hi2c1, INA226_ADDRESS, INA226_MODE_CONT_SHUNT_AND_BUS | INA226_VBUS_140uS | INA226_VBUS_140uS | INA226_AVG_1024);
	//INA226_setCalibrationReg(&hi2c1, INA226_ADDRESS,INA226_CALIB_VAL);

	ina226 = new INA226(&hi2c1,0.0002f,0.024,0x80);

	HAL_Delay(10);

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);

	Flash_Read_Data( (uint32_t*) (&current_parameter_), sizeof(Parameter_t)/4);

	pid = new PID<float>(&speed_frequency,&pid_bk_output,&speed_setpoint,current_parameter_.kp,current_parameter_.ki,current_parameter_.kd);
	pid->set_output_limits(0.0f,15.0f);

	sensor.kp = current_parameter_.kp;
	sensor.ki = current_parameter_.ki;
	sensor.kd = current_parameter_.kd;
	sensor.speed_50 = current_parameter_.fan_speed_half;
	sensor.speed_100 = current_parameter_.fan_speed_full;

	// notify compute to resend the command
	sensor.bool_register |= 0x02;

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buf, RX_BUF_SIZE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx2_buf, RX_BUF_SIZE);

}

/***
 * called in while function.
 */
void loop(){
	auto start_time = HAL_GetTick();

	//switch LED status
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	//refresh watchdog
	//HAL_IWDG_Refresh(&hiwdg);

	//this variable is set to ZERO in the gpio interrupt callback. if reachs SPEED_MAX_INCREMENT, the speed buffer will set to ZEROs
	speed_increment_count++;

	//bk1697_increment_count is same as speed_increment_count. reaching to 8 means that bk1697 is not connected.
	if(bk1697_increment_count++>8){
		bk1697_connected=0;
		sensor.bool_register &= 0b11111110;
		bk1697_increment_count = 10;
		sensor.bk1697_voltage = 0;
		sensor.bk1697_current = 0;
	}else{
		bk1697_connected=1;
		sensor.bool_register |= 0x01;
	}

	//if bk1697 is not connected, try to send this command to bk1697 to re-check its status. if response, the handle is in the uart receive callback function
	if(!bk1697_connected ){
		char data[8];
		sprintf(data,"SOUT01%d\r",int(!run_state));
		Rx485_Send_Data((uint8_t*)data,8);
		HAL_Delay(200);
	}

	//bit 0: run state; bit 3: control mode;

	//run state changes
	if((register_changed & 0x01) ){
		run_state = current_cmd_.bool_register & 0x01;
		HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, run_state?GPIO_PIN_SET:GPIO_PIN_RESET);
		register_changed &= 0b11111110;
	}

	if(register_changed&0x08){
		control_mode = (current_cmd_.bool_register & 0x08)>>3;
		register_changed &= 0b11110111;
	}

	sensor.bk1697_voltage = pid_bk_output*10;

	//read ina226 state
	uint16_t bus_v = ina226->Get_Bus_Voltage_Raw();
	uint16_t bus_c = ina226->Get_Current_Raw();
	uint16_t bus_p = ina226->Get_Power_Raw();
	uint16_t shunt_v = ina226->Get_Shunt_Voltage_Raw();

	sensor.fan_current = bus_c!=0xFF?bus_c:0;
	sensor.fan_power = bus_p!=0xFF?bus_p:0;
	sensor.fan_voltage = bus_v!=0xFF?bus_v:0;
	sensor.fan_shunt_voltage = shunt_v!=0xFF?shunt_v:0;

	//the loop time 500ms
	while(HAL_GetTick()-start_time<500);
}
