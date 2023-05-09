/******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h" //Accelerometer
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h" //Gyroscope
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h" //Humidity Sensor
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h" //Magnetometer
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h" //Temperature Sensor
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h" //Pressure Sensor
#include "stdio.h"
#include "math.h"

extern void initialise_monitor_handles(void);  // for semi-hosting support (printf)
static void MX_GPIO_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
static void UART1_Init(void);
UART_HandleTypeDef huart1;
static void EXTI_PD11_Init(void);
static void LSM6DSL_Init(void);

volatile int mode = 0; //Mode == 0 (Healthy Mode), Mode == 1 (ICU Mode)

//Indicators for when threshold is crossed
volatile int fever_detect = 0;
volatile int fall_detect = 0;
volatile int pain_detect = 0;
volatile int abnormal_detect = 0;
volatile int breath_detect = 0;

//Detect PB intervals
volatile int current_press = 0;
volatile int previous_press = 0;

int main(void){
	initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* UART initialization  */
	UART1_Init();

	//Configure PC13 to EXTI mode
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_HSENSOR_Init();
	BSP_TSENSOR_Init();
	BSP_MAGNETO_Init();
  	BSP_PSENSOR_Init();

  	//Configure LD2
  	MX_GPIO_Init();

  	//Configure PD11 as External Interrupt
  	EXTI_PD11_Init();
  	LSM6DSL_Init();

  	//Variables declaration
  	float temp_data; //Body Temperature Monitoring

  	float accel_data[3]; //Fall Detection
  	int16_t accel_data_i16[3] = { 0 };      // array to store the x, y and z readings.

  	float gyro_data[3]; //Pain Detection
  	float gyroscope_data;

  	float magneto_data[3]; //Lying Monitoring
  	int16_t magneto_data_i16[3] = { 0 };  // array to store the x, y and z readings.

  	float humid_data; //Respiratory Monitoring
  	float pressure_data; //Respiratory Monitoring

  	//Threshold
  	int TEMP_THRESHOLD = 33.5;
  	int ACC_THRESHOLD[3];
  	ACC_THRESHOLD[2] = 8;
  	int GYRO_THRESHOLD = 10000;
  	int MAG_THRESHOLD[3];
  	MAG_THRESHOLD[0] = -0.6;
  	MAG_THRESHOLD[1] = -13;
  	MAG_THRESHOLD[2] = 4;
  	int HUMID_THRESHOLD = 75;
  	int PRESS_THRESHOLD = 1020;

  	//Timer
  	int T01;
  	int T02;

  	int num = 0; //transmission number

  	//messages for UART
  	char healthy_message[] = "Entering Healthy Mode.\r\n";
  	char healthy_print[32];
  	sprintf(healthy_print,"%s", healthy_message);

	char icu_message[] = "Entering Intensive Care Mode.\r\n";
	char icu_print[32];
	sprintf(icu_print,"%s", icu_message);

	char fever_message[] = "Fever is detected\r\n";
	char fever_print[32];
	sprintf(fever_print,"%s", fever_message);

	char fall_message[] = "Falling is detected\r\n";
	char fall_print[32];
	sprintf(fall_print,"%s", fall_message);

	char pain_message[] = "Patient in pain!\r\n";
	char pain_print[32];
	sprintf(pain_print,"%s", pain_message);

	char abnormal_message[] = "Check patient's abnormal orientation!\r\n";
	char abnormal_print[32];
	sprintf(abnormal_print,"%s", abnormal_message);

	char breath_message[] = "Check patient's breath!\r\n";
	char breath_print[32];
	sprintf(breath_print,"%s", breath_message);

	while (1){

		//HEALTHY MODE ACTIONS
		if (mode == 0){
			HAL_UART_Transmit(&huart1, (uint8_t*)healthy_print, strlen(healthy_print),0xFFFF); //Sends message to each time the system is set to Healthy Mode
			HAL_GPIO_WritePin(GPIOB, LED2_Pin, RESET); //Ensures that LD2 is OFF each time the mode is changed
			int counter = 0;

			while (mode == 0){
				T01 = HAL_GetTick();
				T02 = HAL_GetTick();
				while (T02 - T01 < 250) //250ms
					T02 = HAL_GetTick();
				counter++;
				temp_data = BSP_TSENSOR_ReadTemp();      // read temperature sensor


				//Body Temperature Monitoring
				if (temp_data > TEMP_THRESHOLD){ //Print statement immediately when fever is detected
					fever_detect = 1; //Change indicator to 1 once threshold is crossed
					int temp_counter = 0;
					HAL_UART_Transmit(&huart1, (uint8_t*)fever_print, strlen(fever_print),0xFFFF);

					while (fever_detect == 1){
						temp_data = BSP_TSENSOR_ReadTemp();      // read temperature sensor
						HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);		//toggle LD2
						T01 = HAL_GetTick();
						T02 = HAL_GetTick();
						while (T02 - T01 < 250) //LD2 to blink at 2Hz
							T02 = HAL_GetTick();
						temp_counter++;

						if (temp_counter % 40 == 0){ //Print statement every 10s
							HAL_UART_Transmit(&huart1, (uint8_t*)fever_print, strlen(fever_print),0xFFFF);
						}//if loop
					}//while loop
				}//if loop

				//Fall Detection
				BSP_ACCELERO_AccGetXYZ(accel_data_i16);    // read accelerometer
				//the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
				accel_data[0] = (float)accel_data_i16[0] / 100.0f;
				accel_data[1] = (float)accel_data_i16[1] / 100.0f;
				accel_data[2] = (float)accel_data_i16[2] / 100.0f;

				if (accel_data[2] < ACC_THRESHOLD[2]){
					fall_detect = 1; //Set indicator to 1 when threshold is passed
					int fall_counter = 0;
					HAL_UART_Transmit(&huart1, (uint8_t*)fall_print, strlen(fall_print),0xFFFF);

					while (fall_detect ==1){
						BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
						accel_data[0] = (float)accel_data_i16[0] / 100.0f;
						accel_data[1] = (float)accel_data_i16[1] / 100.0f;
						accel_data[2] = (float)accel_data_i16[2] / 100.0f;

						HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
						T01 = HAL_GetTick();
						T02 = HAL_GetTick();
						while (T02 - T01 < 250) //LD2 to blink at 2Hz
							T02 = HAL_GetTick();
						fall_counter++;
						if (fall_counter % 40 == 0){ //Print statement every 10s
							HAL_UART_Transmit(&huart1, (uint8_t*)fall_print, strlen(fall_print),0xFFFF);
						}//if loop
					}//while loop
				}//if loop

			}//while mode == 0 loop

		}///if mode == 0 loop

		//INTENSIVE CARE MODE ACTIONS
		if (mode == 1){
			HAL_UART_Transmit(&huart1, (uint8_t*)icu_print, strlen(icu_print),0xFFFF); //Sends message to each time the system is set to ICU Mode
			HAL_GPIO_WritePin(GPIOB, LED2_Pin, RESET); //Ensures LD2 is OFF each time the mode is changed
			int seconds_counter = 0;

			while (mode == 1){
				int T1, T2;
				T1 = HAL_GetTick();
				T2 = HAL_GetTick();
				while (T2 - T1 < 250) //LD2 to blink at 2Hz
					T2 = HAL_GetTick();
				seconds_counter++;

				//LD2 will start blinking once any of the thresholds are crossed
				if (fever_detect == 1 || fall_detect == 1 || pain_detect == 1 || abnormal_detect == 1 || breath_detect == 1)
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

				if (seconds_counter % 4 == 0){ //sample every second
					//Fever Detection
					temp_data = BSP_TSENSOR_ReadTemp();		//read temperature sensor
					if (temp_data > TEMP_THRESHOLD)
						fever_detect = 1;

					//Fall Detection
					BSP_ACCELERO_AccGetXYZ(accel_data_i16);    // read accelerometer
					accel_data[0] = (float)accel_data_i16[0] / 100.0f;
					accel_data[1] = (float)accel_data_i16[1] / 100.0f;
					accel_data[2] = (float)accel_data_i16[2] / 100.0f;
					if(accel_data[2] < ACC_THRESHOLD[2])
						fall_detect = 1;

					//Pain Detection
					BSP_GYRO_GetXYZ(gyro_data);      //read gyroscope
					gyroscope_data = sqrt(pow(gyro_data[0], 2) + pow(gyro_data[1], 2) + pow(gyro_data[2], 2));
					if (gyroscope_data > GYRO_THRESHOLD)
						pain_detect = 1;

					//Abnormal Orientation Detection
					BSP_MAGNETO_GetXYZ(magneto_data_i16); 	//read magnetometer
					// the function above returns 16 bit integers which are 100 * magneto. Converting to float to print the actual magneto reading.
					magneto_data[0] = (float)magneto_data_i16[0] / 100.0f; //changing to float number
					magneto_data[1] = (float)magneto_data_i16[1] / 100.0f;
					magneto_data[2] = (float)magneto_data_i16[2] / 100.0f;
					if ((magneto_data[0] >= MAG_THRESHOLD[0]) || (magneto_data[1] >= MAG_THRESHOLD[1]) || (magneto_data[2] >= MAG_THRESHOLD[2]) )
						abnormal_detect = 1;

					//Breath Detection
					humid_data = BSP_HSENSOR_ReadHumidity();
					pressure_data = BSP_PSENSOR_ReadPressure();
					if (humid_data < HUMID_THRESHOLD || pressure_data > PRESS_THRESHOLD)
						breath_detect = 1;
				}//sample every 1s if loop

				if (seconds_counter % 40 == 0){ //Print statement every 10s
					num++;//transmission increases by 1
					char data_1_print[32];
					char data_2_print[32];
					char data_3_print[32];

					if (temp_data > TEMP_THRESHOLD) //Print statement when fever is detected
						HAL_UART_Transmit(&huart1, (uint8_t*)fever_print, strlen(fever_print),0xFFFF);

					if(accel_data[2] < ACC_THRESHOLD[2]) ///Print statement when falling is detected
						HAL_UART_Transmit(&huart1, (uint8_t*)fall_print, strlen(fall_print),0xFFFF);

					sprintf(data_1_print,"%03d_TEMP_%.2f_ACC_%.2f_%.2f_%.2f \r\n", num, temp_data, accel_data[0], accel_data[1], accel_data[2]);
					HAL_UART_Transmit(&huart1, (uint8_t*)data_1_print, strlen(data_1_print),0xFFFF);

					if (gyroscope_data > GYRO_THRESHOLD) //Prints statement if gyroscope reading is larger than threshold
						HAL_UART_Transmit(&huart1, (uint8_t*)pain_print, strlen(pain_print),0xFFFF);

					//Prints statement if magnetometer reading hits threshold
					if ((magneto_data[0] >= MAG_THRESHOLD[0]) || (magneto_data[1] >= MAG_THRESHOLD[1]) || (magneto_data[2] >= MAG_THRESHOLD[2]) ) //If magneto reading hits the threshold
						HAL_UART_Transmit(&huart1, (uint8_t*)abnormal_print, strlen(abnormal_print),0xFFFF);

					sprintf(data_2_print,"%03d GYRO %.1f MAGNETO %.2f %.2f %.2f\r\n", num, gyroscope_data, magneto_data[0], magneto_data[1], magneto_data[2]);
					HAL_UART_Transmit(&huart1, (uint8_t*)data_2_print, strlen(data_2_print),0xFFFF);

					if (humid_data < HUMID_THRESHOLD || pressure_data > PRESS_THRESHOLD) //Prints statement if humidity or pressure threshold is passed
						HAL_UART_Transmit(&huart1, (uint8_t*)breath_print, strlen(breath_print),0xFFFF);

					sprintf(data_3_print,"%03d HUMIDITY %.2f and BARO %.2f \r\n\n",num, humid_data, pressure_data);
					HAL_UART_Transmit(&huart1, (uint8_t*)data_3_print, strlen(data_3_print),0xFFFF);
				}//UART transmission loop


			} //while mode == 1 loop

		} //if mode == 1

	}//end of while (1) loop

}//main loop

//Configure LD2
static void MX_GPIO_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin LED2_Pin */
    GPIO_InitStruct.Pin = LED2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

//EXTI Handler for Pushbutton & LSM6DSL Interrupts
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//Pushbutton Interrupt
	if (GPIO_Pin == GPIO_PIN_13){
		int time1, time2;

		time1 = HAL_GetTick();
		time2 = HAL_GetTick();
		while (time2 - time1 < 50) //Read the PB every 50ms
			time2 = HAL_GetTick();

		current_press = HAL_GetTick(); //Get the current_press timing for comparison later

		//If 2 presses are within 1sec, set to "ICU Mode"
		if (current_press - previous_press < 1000){
			mode = 1; //Set mode to "ICU Mode"
			//Reset all indicators every time mode is changed
			fever_detect = 0;
			fall_detect = 0;
			pain_detect = 0;
			abnormal_detect = 0;
			breath_detect = 0;
		}

		//If 2 presses are outside 1sec, set to "Healthy Mode"
		else {
			mode = 0; //Set mode to "Healthy Mode"
			//Reset all indicators every time mode is changed
			fever_detect = 0;
			fall_detect = 0;
			pain_detect = 0;
			abnormal_detect = 0;
			breath_detect = 0;
		}

		previous_press = current_press; //assign previous_press to current_press for comparison when the PB is pressed again
	}//if GPIO_PIN_13 loop

	//LSM6DSL Interrupt
	if (GPIO_Pin == LSM6DSL_INT1_EXTI11_Pin){
		char tilt_message[] = "ATTENTION REQUIRED! PATIENT REQUIRES ASSISTANCE!\r\n";
		char tilt_print[32];
		sprintf(tilt_print,"%s", tilt_message);
		HAL_UART_Transmit(&huart1, (uint8_t*)tilt_print, strlen(tilt_print),0xFFFF);
	}

}//HAL loop

//UART Initialisation
static void UART1_Init(void)
{
	/* Pin configuration for UART. BSP_COM_Init() can do this automatically */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Configuring UART1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		while(1);
	}
}

//Configure LSM6DSL pin (PD11) as External interrupt
static void EXTI_PD11_Init(void){
	GPIO_InitTypeDef TILT_InitStruct;

	/* Enable the GPIO D's clock */
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* Configure LSM6DSL pin (PD11) as input with External interrupt */
	TILT_InitStruct.Pin = LSM6DSL_INT1_EXTI11_Pin;
	TILT_InitStruct.Pull = GPIO_PULLUP;
	TILT_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	TILT_InitStruct.Mode = GPIO_MODE_IT_RISING;

	HAL_GPIO_Init(LSM6DSL_INT1_EXTI11_GPIO_Port, &TILT_InitStruct);

	/* Enable and set LSM6DSL Interrupt to the 01 sub priority */
	HAL_NVIC_SetPriority((IRQn_Type)(LSM6DSL_INT1_EXTI11_EXTI_IRQn), 0x0F, 0x01);
	HAL_NVIC_EnableIRQ((IRQn_Type)(LSM6DSL_INT1_EXTI11_EXTI_IRQn));
}

//Configure LSM6DSL to TILT Detection
static void LSM6DSL_Init(void){
	//Configuring Registers For Tilt Detection
	//Slave address for I2C = 1101 010x
	//For Write: x = 0, Slave address = 0xD4
	//For Read: x = 1, Slave address = 0xD5

	SENSOR_IO_Write(0xD4, LSM6DSL_ACC_GYRO_CTRL1_XL,0x20);

	//1 both the FUNC_EN and the TILT_EN bits of the CTRL10_C register: 0000 1100 (0x0C)
	SENSOR_IO_Write(0xD4, LSM6DSL_ACC_GYRO_CTRL10_C, 0x0C);

	//1 the INT1_TILT bit of the MD1_CFG register: 0000 0010 (0x02)
	SENSOR_IO_Write(0xD4, LSM6DSL_ACC_GYRO_MD1_CFG, 0x02);
}
