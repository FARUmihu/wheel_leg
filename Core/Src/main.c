/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
#include "dm_motor.h"
#include "feetech_servo.h"
#include "leg_kinematics.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KIN_LENGTH_TEST_FIXED_THETA1_DEG  (-140.0f)
#define KIN_LENGTH_TEST_FT_SPEED          80U
#define KIN_LENGTH_TEST_FT_ACC            10U
#define KIN_LENGTH_TEST_DM_KP             10.0f
#define KIN_LENGTH_TEST_DM_KD             0.20f
#define KIN_LENGTH_TEST_PERIOD_MS         2000U
#define KIN_LENGTH_TEST_DM_SEND_MS        10U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const float kin_length_test_targets_mm[] = {
  120.0f,
  150.0f,
  180.0f,
};

static uint8_t kin_length_test_index = 0U;

volatile float g_kin_length_test_fixed_theta1_deg = KIN_LENGTH_TEST_FIXED_THETA1_DEG;
volatile uint16_t g_kin_length_test_fixed_ft_raw;
volatile float g_kin_length_test_target_mm;
volatile float g_kin_length_test_preferred_dm_rad = 1.10f;
volatile float g_kin_length_test_target_dm_rad;
volatile float g_kin_length_test_feedback_dm_rad;
volatile float g_kin_length_test_target_length_mm;
volatile float g_kin_length_test_feedback_length_mm;
volatile float g_kin_length_test_length_error_mm;
volatile float g_kin_length_test_dm_error_rad;
volatile uint8_t g_kin_length_test_dm_send_status;
volatile int g_kin_length_test_ft_update_status;
volatile uint32_t g_kin_length_test_rx_count;
volatile uint32_t g_kin_length_test_cycle_count;
volatile leg_kinematics_status_t g_kin_length_test_status;
volatile leg_kinematics_status_t g_kin_length_test_target_forward_status;
volatile leg_kinematics_status_t g_kin_length_test_feedback_forward_status;
volatile leg_kinematics_result_t g_kin_length_test_inverse_result;
volatile leg_kinematics_result_t g_kin_length_test_target_forward_result;
volatile leg_kinematics_result_t g_kin_length_test_feedback_forward_result;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float point_length_mm(leg_point_t p)
{
  return sqrtf((p.x_mm * p.x_mm) + (p.y_mm * p.y_mm));
}

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  app_init();
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_Delay(3000);

  uint16_t fixed_ft_raw =
      leg_kinematics_theta1_deg_to_ft_raw(LEG_SIDE_LEFT,
                                          KIN_LENGTH_TEST_FIXED_THETA1_DEG);
  g_kin_length_test_fixed_ft_raw = fixed_ft_raw;

  dm_motor_enable(&g_dm_motors[DM_MOTOR_LEFT_JOINT_IDX]);
  feetech_servo_enable(FEETECH_ID_LEFT);
  feetech_servo_set_pos(FEETECH_ID_LEFT,
                        fixed_ft_raw,
                        KIN_LENGTH_TEST_FT_SPEED,
                        KIN_LENGTH_TEST_FT_ACC);
  HAL_Delay(1500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    float target_length_mm = kin_length_test_targets_mm[kin_length_test_index];
    float target_dm_rad = g_kin_length_test_preferred_dm_rad;
    leg_kinematics_result_t inverse_result = {0};
    leg_kinematics_result_t target_forward_result = {0};

    leg_kinematics_status_t status =
        leg_kinematics_fixed_ft_inverse_length(LEG_SIDE_LEFT,
                                               fixed_ft_raw,
                                               target_length_mm,
                                               g_kin_length_test_preferred_dm_rad,
                                               &target_dm_rad,
                                               &inverse_result);
    leg_kinematics_status_t target_forward_status =
        leg_kinematics_fixed_ft_forward(LEG_SIDE_LEFT,
                                        fixed_ft_raw,
                                        target_dm_rad,
                                        &target_forward_result);

    uint32_t start_ms = HAL_GetTick();
    while ((HAL_GetTick() - start_ms) < KIN_LENGTH_TEST_PERIOD_MS) {
      leg_kinematics_result_t feedback_forward_result = {0};
      float feedback_dm_rad = g_dm_motors[DM_MOTOR_LEFT_JOINT_IDX].pos;

      uint8_t dm_send_status =
          dm_motor_send_mit(&g_dm_motors[DM_MOTOR_LEFT_JOINT_IDX],
                            target_dm_rad,
                            0.0f,
                            KIN_LENGTH_TEST_DM_KP,
                            KIN_LENGTH_TEST_DM_KD,
                            0.0f);

      leg_kinematics_status_t feedback_forward_status =
          leg_kinematics_fixed_ft_forward(LEG_SIDE_LEFT,
                                          fixed_ft_raw,
                                          feedback_dm_rad,
                                          &feedback_forward_result);

      float target_result_length_mm = point_length_mm(target_forward_result.foot_d);
      float feedback_length_mm = point_length_mm(feedback_forward_result.foot_d);

      g_kin_length_test_target_mm = target_length_mm;
      g_kin_length_test_preferred_dm_rad = target_dm_rad;
      g_kin_length_test_target_dm_rad = target_dm_rad;
      g_kin_length_test_feedback_dm_rad = feedback_dm_rad;
      g_kin_length_test_target_length_mm = target_result_length_mm;
      g_kin_length_test_feedback_length_mm = feedback_length_mm;
      g_kin_length_test_length_error_mm = target_length_mm - feedback_length_mm;
      g_kin_length_test_dm_error_rad = target_dm_rad - feedback_dm_rad;
      g_kin_length_test_dm_send_status = dm_send_status;
      g_kin_length_test_rx_count = g_dm_rx_count;
      g_kin_length_test_status = status;
      g_kin_length_test_target_forward_status = target_forward_status;
      g_kin_length_test_feedback_forward_status = feedback_forward_status;
      g_kin_length_test_inverse_result = inverse_result;
      g_kin_length_test_target_forward_result = target_forward_result;
      g_kin_length_test_feedback_forward_result = feedback_forward_result;

      app_background();
      HAL_Delay(KIN_LENGTH_TEST_DM_SEND_MS);
    }

    g_kin_length_test_ft_update_status = feetech_servo_update(APP_FT_LEFT);
    g_kin_length_test_cycle_count++;

    kin_length_test_index++;
    if (kin_length_test_index >=
        (sizeof(kin_length_test_targets_mm) / sizeof(kin_length_test_targets_mm[0]))) {
      kin_length_test_index = 0U;
    }

    /* USER CODE END WHILE */
    app_background();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
