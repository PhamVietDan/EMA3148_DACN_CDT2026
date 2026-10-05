/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Chuong trinh chinh
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

#include "rc_sbus.h"
#include "sensor_imu.h"
#include "sensor_baro.h"
#include "sensor_gps_mag.h"
#include "sensor_flow.h"
#include "flight_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Mag_Calibrate(uint16_t duration_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char tx_buffer[100];
uint8_t telemetry_cnt = 0;

extern float pitch;
extern float roll;
extern float yaw;

extern uint16_t rc_channel[16];
extern uint16_t throttle_pwm;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // 1. QUAN TRỌNG: Khởi động giao tiếp UART SBUS TRƯỚC TIÊN.
  // Phải có SBUS đọc tay cầm thì mới biết là người dùng muốn Calib hay Khởi động.
  SBUS_Init();

  // 2. Gọi hàm Calib / Arm bằng tay cầm (Code sẽ bị giam ở đây nếu chưa bật TX)
  // Nhớ sửa lại tên hàm trong file flight_control.h của bạn cho đồng nhất
  Motor_Init_And_Calibrate_By_RC();

  // 3. Khởi động các thiết bị khác
  GPS_Init();
  Flow_DMA_Init();

  // 4. Khởi tạo & Calib các cảm biến
  IMU_Init_And_Calibrate();
  Mag_Init();
  Baro_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      Flow_Process_Data();
      GPS_Process_Sentence();
      Baro_Read_Altitude();
      Mag_Read_Heading();

      IMU_Read_And_Calculate_Angles();
      FlightControl_Update_PID();

      telemetry_cnt++;
      if (telemetry_cnt >= 50)
      {
          telemetry_cnt = 0;
          if (huart6.gState == HAL_UART_STATE_READY)
          {
              sprintf(tx_buffer, "Roll: %.1f | Pitch: %.1f | Yaw: %.1f\r\n", roll, pitch, yaw);
              HAL_UART_Transmit_DMA(&huart6, (uint8_t*)tx_buffer, strlen(tx_buffer));
          }
          char usb_buffer[100];
                        sprintf(usb_buffer, "CH1_Roll: %4d | CH2_Pitch: %4d | CH3_Thr: %4d | CH4_Yaw: %4d | PWM: %4d\r\n",
                                rc_channel[0], rc_channel[1], rc_channel[2], rc_channel[3], throttle_pwm);

                        // Hàm này sẽ đẩy dữ liệu qua dây Type-C lên máy tính
                        CDC_Transmit_FS((uint8_t*)usb_buffer, strlen(usb_buffer));
      }

      HAL_Delay(2);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Xử lý ngắt nhận UART chung
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Ngắt SBUS (Tay cầm)
    if (huart->Instance == USART1) {
        SBUS_Parse();
    }
    // Ngắt GPS (UART3)
    if (huart->Instance == USART3) {
        GPS_Parse_Interrupt();
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
