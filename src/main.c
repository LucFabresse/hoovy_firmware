#include "stm32f1xx_hal.h"
#include "constants.h"
#include "config.h"
#include "power.h"
#include "debug.h"
#include "uart.h"
#include "adc.h"
#include "motor.h"

extern volatile struct UART uart;

// ----------------------PRIVATE----------------------
static void SystemClock_Config(void);
static void MX_IWDG_Init(void);
static void receive_data();
static void transmit_data();
static void check_power();

// ----------------------PUBLIC----------------------
void error_handler(void);

uint32_t time, last_tx_time, last_rx_time, last_pwr_time;
volatile int8_t status;
int16_t speeds[2];
IWDG_HandleTypeDef hiwdg;

#ifdef DEBUG
extern struct ADC adc_L;
extern struct ADC adc_R;
#endif


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ROS SERIAL
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include "string.h"


extern DMA_HandleTypeDef hdma_usart2_rx;
extern volatile struct UART uart;

const static uint16_t rbuflen = 128; // BUFFER_LENGTH (uart.h)

extern UART_HandleTypeDef huart2;
UART_HandleTypeDef *huart = &huart2;
uint32_t rind=0;

int rosserial_read(){
	int rx_value = -1;
	int dma_count = -1;
	
	if (uart.RX_available) {
		// __HAL_DMA_GET_COUNTER(__HANDLE__) Returns the number of remaining data units in the current DMAy Streamx transfer.
		dma_count =  rbuflen - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
					
		if( rind != dma_count){
		    rx_value = uart.RX_buffer[rind++];
		    rind &= rbuflen - 1;
		}
		last_rx_time = HAL_GetTick();
	}
	return rx_value;
}

const static uint16_t tbuflen = 256;
uint8_t tbuf[256];
uint32_t twind=0, tfind=0;

void rosserial_flush(void){
  if (Uart_is_TX_free()) {
    uart.TX_free = 0;    //busy
    if(twind != tfind){
      uint16_t len = tfind < twind ? twind - tfind : tbuflen - tfind;
      HAL_UART_Transmit_DMA(huart, &(tbuf[tfind]), len);
      tfind = (tfind + len) & (tbuflen - 1);
    }
    last_tx_time = HAL_GetTick();
  }
}

void rosserial_write(uint8_t* data, int length){
  int n = length;
  n = n <= tbuflen ? n : tbuflen;

  int n_tail = n <= tbuflen - twind ? n : tbuflen - twind;
  memcpy(&(tbuf[twind]), data, n_tail);
  twind = (twind + n) & (tbuflen - 1);

  if(n != n_tail){
    memcpy(tbuf, &(data[n_tail]), n - n_tail);
  }

  rosserial_flush();
}


void rosserial_write_int(int i){
	char buf[10];
	bzero(buf,10);
	sprintf(buf,"%d\n\r",i);
	rosserial_write(buf,strlen(buf));
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




/* MAIN
 * Setup the clock and watchdog, and initialize all the peripherals.
 * Check the RX data, TX data, and power statuses at different intervals.
 */
int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	/* Watchdog setup -- force restart if something unexpected happens*/
	MX_IWDG_Init();

	// power everything
	button_init();
	buzzer_init();
	#ifdef BUZZER_START_DEBUG
	buzzer_one_beep();
	#endif
	charging_init();
	led_init();
	led_set(1);

	MX_USART2_UART_Init();
	adcs_setup_and_init();
	motors_setup_and_init();

#ifdef CALIBRATION

	while (1) {
		motors_calibrate();
	}
#else

	last_rx_time = HAL_GetTick();
	last_tx_time = HAL_GetTick();
	last_pwr_time = HAL_GetTick();

	int shouldWrite = 0;
	int readChar = -1;

	
	while (1) {
		time = HAL_GetTick();
					
		if( !shouldWrite ) {
			readChar=rosserial_read();
			shouldWrite = (readChar != -1);
		} else {		
			rosserial_write(&readChar, 1);
			shouldWrite = !shouldWrite;
		}				

		HAL_IWDG_Refresh(&hiwdg);   //819mS
	}

#endif

}


// ----------------------PRIVATE----------------------
/* RECEIVE DATA
 * Process the newly received data. If a proper frame was processed, update the last_rx_time.
 * Update the motors to the new speeds.
 */
static void receive_data() {
	int uart_rx_status = Uart_RX_process();
	if (uart_rx_status == 1) {
		last_rx_time = HAL_GetTick();
		motors_speeds(speeds[0], speeds[1]);
	}
}

/* TRANSMIT DATA
 * Send the status byte, as well as the battery voltage if the TX line is free.
 * In DEBUG mode, additional readings are outputted - current readings from the wheel.
 */
static void transmit_data() {
	float data_v;
	data_v = get_battery_volt();

#if defined(DEBUG) && !defined(DEBUG_NO_ADC)
	//TODO: These readings are not in amps, needs work.
	float data_i_L, data_i_R;
	data_i_L = get_motor_current(&adc_L);
	data_i_R = get_motor_current(&adc_R);
	sprintf((char *)&uart.TX_buffer[0],"[%d, %d, %d, %d]\n", status, (int)data_v, (int)data_i_L, (int)data_i_R);
#else
	sprintf((char *)&uart.TX_buffer[0],"[%d, %d]\n", status, (int)data_v);
#endif

	if (Uart_is_TX_free()) {
		Uart_TX((char *)&uart.TX_buffer[0]);
		last_tx_time = HAL_GetTick();
	}
}

/* Check two power related things:
 * - the battery level is too low (limit set at 32V)
 * - if it's charging
 *
 * If either of them is true, the robot should not move,
 * and the corresponding error bits should be set in the status byte.
 */
static void check_power() {
	/* based off of power button at PA1
	 * PA1 is detected high at ~2V in 3.3V system
	 * voltage detected is 1/16 of battery voltage
	 */
	if (get_battery_volt() < 32) {
		SET_ERROR_BIT(status, STATUS_LOW_BATTERY);
		motors_stop();
		buzzer_short_beep();
	} else {
		CLR_ERROR_BIT(status, STATUS_LOW_BATTERY);
	}
	HAL_IWDG_Refresh(&hiwdg);   //819mS


	/* don't move if we are charging
	 */
	if (is_charging()) {
		SET_ERROR_BIT(status, STATUS_IS_CHARGING);
		motors_stop();
	} else {
		CLR_ERROR_BIT(status, STATUS_IS_CHARGING);
	}

	last_pwr_time = HAL_GetTick();
}


/** System Clock Configuration
 */
static void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		error_handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		error_handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		error_handler();
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{
	__HAL_RCC_WWDG_CLK_ENABLE();
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		error_handler();
	}
	HAL_IWDG_Start(&hiwdg);
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void error_handler(void)
{
	/* USER CODE BEGIN error_handler */
	/* User can add his own implementation to report the HAL error return state */
	motors_stop();

	while(1)
	{
	}
	/* USER CODE END error_handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
