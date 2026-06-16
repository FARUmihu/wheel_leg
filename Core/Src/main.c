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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IK_TEST_DM_KP          6.0f
#define IK_TEST_DM_KD          0.15f
#define IK_TEST_FT_SPEED       130U
#define IK_TEST_FT_ACC         10U
#define IK_TEST_PERIOD_MS      1800U
#define IK_TEST_DM_SEND_MS     10U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const leg_point_t left_ik_test_points[] = {
  { -75.0f, -165.0f },
  { -80.0f, -180.0f },
  { -75.0f, -195.0f },
  { -65.0f, -205.0f },
  { -45.0f, -210.0f },
  { -30.0f, -200.0f },
  { -25.0f, -185.0f },
  { -35.0f, -170.0f },
  { -55.0f, -160.0f },
  { -70.0f, -155.0f },
};

static const leg_point_t right_ik_test_points[] = {
  { 75.0f, -165.0f },
  { 80.0f, -180.0f },
  { 75.0f, -195.0f },
  { 65.0f, -205.0f },
  { 45.0f, -210.0f },
  { 30.0f, -200.0f },
  { 25.0f, -185.0f },
  { 35.0f, -170.0f },
  { 55.0f, -160.0f },
  { 70.0f, -155.0f },
};

static uint8_t ik_test_index = 0U;

volatile leg_point_t g_left_ik_test_target;
volatile leg_point_t g_right_ik_test_target;
volatile leg_actuator_state_t g_left_ik_test_actuator;
volatile leg_actuator_state_t g_right_ik_test_actuator;
volatile leg_kinematics_result_t g_left_ik_test_result;
volatile leg_kinematics_result_t g_right_ik_test_result;
volatile leg_kinematics_status_t g_left_ik_test_status;
volatile leg_kinematics_status_t g_right_ik_test_status;
volatile uint8_t g_left_ik_test_dm_send_status;
volatile uint8_t g_right_ik_test_dm_send_status;
volatile int g_left_ik_test_ft_update_status;
volatile int g_right_ik_test_ft_update_status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  dm_motor_enable(&g_dm_motors[DM_MOTOR_LEFT_JOINT_IDX]);
  dm_motor_enable(&g_dm_motors[DM_MOTOR_RIGHT_JOINT_IDX]);
  feetech_servo_enable(FEETECH_ID_LEFT);
  feetech_servo_enable(FEETECH_ID_RIGHT);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    leg_point_t left_target = left_ik_test_points[ik_test_index];
    leg_point_t right_target = right_ik_test_points[ik_test_index];
    leg_actuator_state_t left_actuator = {0};
    leg_actuator_state_t right_actuator = {0};
    leg_kinematics_result_t left_result = {0};
    leg_kinematics_result_t right_result = {0};
    leg_kinematics_status_t left_status =
        leg_kinematics_inverse_to_actuator(LEG_SIDE_LEFT,
                                           &left_target,
                                           &left_actuator,
                                           &left_result);
    leg_kinematics_status_t right_status =
        leg_kinematics_inverse_to_actuator(LEG_SIDE_RIGHT,
                                           &right_target,
                                           &right_actuator,
                                           &right_result);

    g_left_ik_test_target = left_target;
    g_right_ik_test_target = right_target;
    g_left_ik_test_actuator = left_actuator;
    g_right_ik_test_actuator = right_actuator;
    g_left_ik_test_result = left_result;
    g_right_ik_test_result = right_result;
    g_left_ik_test_status = left_status;
    g_right_ik_test_status = right_status;

    if ((left_status == LEG_KINEMATICS_OK) &&
        (right_status == LEG_KINEMATICS_OK)) {
      feetech_servo_sync_set_pos(left_actuator.ft_raw,
                                 right_actuator.ft_raw,
                                 IK_TEST_FT_SPEED,
                                 IK_TEST_FT_ACC);
      g_left_ik_test_ft_update_status = feetech_servo_update(APP_FT_LEFT);
      g_right_ik_test_ft_update_status = feetech_servo_update(APP_FT_RIGHT);

      uint32_t start_ms = HAL_GetTick();
      while ((HAL_GetTick() - start_ms) < IK_TEST_PERIOD_MS) {
        g_left_ik_test_dm_send_status =
            dm_motor_send_mit(&g_dm_motors[DM_MOTOR_LEFT_JOINT_IDX],
                              left_actuator.dm_rad,
                              0.0f,
                              IK_TEST_DM_KP,
                              IK_TEST_DM_KD,
                              0.0f);
        g_right_ik_test_dm_send_status =
            dm_motor_send_mit(&g_dm_motors[DM_MOTOR_RIGHT_JOINT_IDX],
                              right_actuator.dm_rad,
                              0.0f,
                              IK_TEST_DM_KP,
                              IK_TEST_DM_KD,
                              0.0f);
        app_background();
        HAL_Delay(IK_TEST_DM_SEND_MS);
      }
    } else {
      HAL_Delay(IK_TEST_PERIOD_MS);
    }

    ik_test_index++;
    if (ik_test_index >= (sizeof(right_ik_test_points) / sizeof(right_ik_test_points[0]))) {
      ik_test_index = 0U;
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
