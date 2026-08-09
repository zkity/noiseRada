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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ===== 噪声检测参数(按现场调) ===== */
#define MIC_WINDOW_MS        40U     /* 每次采样窗口,捕捉若干个语音周期 */
#define NOISE_THRESHOLD      600U    /* ADC 峰峰值阈值(0~4095),越大越不灵敏,需现场标定 X 分贝 */
#define NOISE_SUSTAIN_MS     300U   /* Y: 持续吵闹多久后报警;2 分钟则填 (2U*60U*1000U) */
#define QUIET_RESET_MS       1500U   /* 允许的短暂安静间隙,超过则重新计时 */
#define ALARM_COOLDOWN_MS    8000U   /* 一次报警后的冷却时间,避免连环触发 */
#define LED_BLINK_DURATION_MS 6000U  /* 外接 LED 闪烁总时长 */
#define LED_BLINK_INTERVAL_MS 150U   /* 亮/灭各持续多久 */

/* ===== 舵机脉宽(微秒),SG90 约 500=0° 1500=90° 2500=180° ===== */
#define SERVO_RETRACT_US     600U    /* 平时停靠位置 */
#define SERVO_SWEEP_MIN_US   500U    /* 搜索扫动左端(舵机极限约 0°) */
#define SERVO_SWEEP_MAX_US   2500U   /* 搜索扫动右端(舵机极限约 180°) */
#define SERVO_SWEEP_STEP_US  50U     /* 每步转动量 */
#define SERVO_SWEEP_STEP_MS  20U     /* 每步间隔,越小越快 */
#define SERVO_SWEEP_ROUNDS   4U      /* 来回搜索圈数(一来一回算 1 圈) */

/* ===== DFPlayer ===== */
#define DFPLAYER_VOLUME      25U     /* 音量 0~30 */
#define DFPLAYER_TRACK_MIN   1U      /* 0001.mp3 */
#define DFPLAYER_TRACK_MAX   9U      /* 0009.mp3 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t s_loudAccumMs = 0;   /* 累计的连续吵闹时间 */
static uint32_t s_quietStreakMs = 0; /* 当前连续安静时间 */
static uint8_t  s_lastTrack = 1U;    /* 上次播放曲目;首次触发前视为 0001.mp3 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 向 DFPlayer Mini 发送一条 10 字节命令帧 */
static void DFPlayer_SendCmd(uint8_t cmd, uint16_t param)
{
  uint8_t dh = (uint8_t)((param >> 8) & 0xFF);
  uint8_t dl = (uint8_t)(param & 0xFF);
  int16_t sum = (int16_t)-(0xFF + 0x06 + cmd + 0x00 + dh + dl);
  uint8_t frame[10];
  frame[0] = 0x7E;            /* 起始 */
  frame[1] = 0xFF;            /* 版本 */
  frame[2] = 0x06;            /* 数据长度 */
  frame[3] = cmd;             /* 命令 */
  frame[4] = 0x00;            /* 是否需要应答,0=不需要 */
  frame[5] = dh;              /* 参数高字节 */
  frame[6] = dl;              /* 参数低字节 */
  frame[7] = (uint8_t)((sum >> 8) & 0xFF); /* 校验高 */
  frame[8] = (uint8_t)(sum & 0xFF);        /* 校验低 */
  frame[9] = 0xEF;            /* 结束 */
  HAL_UART_Transmit(&huart1, frame, sizeof(frame), 100);
}

/* 设置舵机脉宽(微秒);TIM2 已配成 1MHz 计数,1 计数 = 1µs */
static void Servo_SetMicroseconds(uint16_t us)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, us);
}

/* 统一控制 4 个红灯(PB12~PB15,高电平亮) */
static void Leds_All(GPIO_PinState st)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, st);
}

/* 板载灯 PC13 低电平亮 */
static void BoardLed(uint8_t on)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* 采样一段时间内麦克风的峰峰值,用来表征"响度" */
static uint16_t Mic_ReadPeakToPeak(uint32_t window_ms)
{
  uint16_t vmin = 4095, vmax = 0;
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < window_ms)
  {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
      uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
      if (v > vmax) vmax = v;
      if (v < vmin) vmin = v;
    }
  }
  HAL_ADC_Stop(&hadc1);
  return (uint16_t)(vmax - vmin);
}

/* 简易随机数(无标准库 rand 时用) */
static uint32_t SimpleRandom(void)
{
  return HAL_GetTick() ^ (uint32_t)SysTick->VAL ^ ((uint32_t)s_lastTrack << 8);
}

/* 从除上一首外的曲目中随机选一首,并更新 s_lastTrack */
static uint8_t PickNextTrack(void)
{
  uint8_t pool[DFPLAYER_TRACK_MAX - 1U];
  uint8_t count = 0U;

  for (uint8_t t = DFPLAYER_TRACK_MIN; t <= DFPLAYER_TRACK_MAX; t++)
  {
    if (t != s_lastTrack)
    {
      pool[count++] = t;
    }
  }

  uint8_t idx = (uint8_t)(SimpleRandom() % count);
  s_lastTrack = pool[idx];
  return s_lastTrack;
}

/* 舵机从 from 平滑扫到 to,同时按间隔闪灯 */
static void Servo_SweepWhileBlink(uint16_t from, uint16_t to,
                                  uint32_t *ledNextToggle, uint8_t *ledOn)
{
  int16_t dir = (to >= from) ? (int16_t)SERVO_SWEEP_STEP_US
                             : -(int16_t)SERVO_SWEEP_STEP_US;
  uint16_t pos = from;

  while (1)
  {
    Servo_SetMicroseconds(pos);
    HAL_Delay(SERVO_SWEEP_STEP_MS);

    /* 与舵机扫动并行闪灯 */
    if ((HAL_GetTick() - *ledNextToggle) >= LED_BLINK_INTERVAL_MS)
    {
      *ledOn = !*ledOn;
      Leds_All(*ledOn ? GPIO_PIN_SET : GPIO_PIN_RESET);
      *ledNextToggle = HAL_GetTick();
    }

    if (pos == to)
    {
      break;
    }

    int16_t next = (int16_t)pos + dir;
    if ((dir > 0 && next >= (int16_t)to) || (dir < 0 && next <= (int16_t)to))
    {
      pos = to;
    }
    else
    {
      pos = (uint16_t)next;
    }
  }
}

/* 触发报警:播报语音 + 雷达式来回搜索 + 闪红灯 */
static void TriggerAlarm(void)
{
  uint8_t track = PickNextTrack();
  DFPlayer_SendCmd(0x03, track);            /* 播放随机语音提醒 */

  uint32_t ledNextToggle = HAL_GetTick();
  uint8_t ledOn = 0U;
  uint32_t alarmStart = HAL_GetTick();

  Leds_All(GPIO_PIN_SET);
  ledOn = 1U;

  /* 多圈来回扫描,像雷达在搜索 */
  for (uint8_t r = 0U; r < SERVO_SWEEP_ROUNDS; r++)
  {
    Servo_SweepWhileBlink(SERVO_SWEEP_MIN_US, SERVO_SWEEP_MAX_US,
                          &ledNextToggle, &ledOn);
    Servo_SweepWhileBlink(SERVO_SWEEP_MAX_US, SERVO_SWEEP_MIN_US,
                          &ledNextToggle, &ledOn);
  }

  /* 若扫描结束得太早,把灯继续闪满设定时长 */
  while ((HAL_GetTick() - alarmStart) < LED_BLINK_DURATION_MS)
  {
    Leds_All(GPIO_PIN_SET);
    HAL_Delay(LED_BLINK_INTERVAL_MS);
    Leds_All(GPIO_PIN_RESET);
    HAL_Delay(LED_BLINK_INTERVAL_MS);
  }

  Leds_All(GPIO_PIN_RESET);
  Servo_SetMicroseconds(SERVO_RETRACT_US);  /* 回到停靠位 */
  HAL_Delay(300);
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* --- 按实际时钟把 TIM2 配成 1MHz 计数、50Hz 周期(20ms) --- */
  /* CubeMX 里 PSC=71 是为 72MHz 准备的;当前若是 HSI 8MHz 会变成 ~5.5Hz,这里运行时纠正 */
  {
    uint32_t timclk = HAL_RCC_GetPCLK1Freq();
    /* APB1 分频 !=1 时,定时器时钟为 PCLK1 的 2 倍 */
    if (((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos) >= 4U)
    {
      timclk *= 2U;
    }
    __HAL_TIM_SET_PRESCALER(&htim2, (timclk / 1000000U) - 1U); /* → 1MHz,1 计数=1µs */
    __HAL_TIM_SET_AUTORELOAD(&htim2, 19999U);                  /* 20000µs=20ms=50Hz */
  }
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  Servo_SetMicroseconds(SERVO_RETRACT_US);

  /* ADC 自校准,提高读数精度 */
  HAL_ADCEx_Calibration_Start(&hadc1);

  /* 初始状态:灯全灭 */
  Leds_All(GPIO_PIN_RESET);
  BoardLed(0);

  /* DFPlayer 上电初始化:等模块启动 → 选 TF 卡 → 设音量 */
  HAL_Delay(500);
  DFPlayer_SendCmd(0x09, 0x0002);   /* 指定播放设备:TF 卡 */
  HAL_Delay(200);
  DFPlayer_SendCmd(0x06, DFPLAYER_VOLUME);
  HAL_Delay(200);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint16_t amp = Mic_ReadPeakToPeak(MIC_WINDOW_MS);

    if (amp >= NOISE_THRESHOLD)
    {
      s_loudAccumMs += MIC_WINDOW_MS;   /* 吵闹累计 */
      s_quietStreakMs = 0;
    }
    else
    {
      s_quietStreakMs += MIC_WINDOW_MS;
      if (s_quietStreakMs >= QUIET_RESET_MS)
      {
        s_loudAccumMs = 0;              /* 安静够久,重新计时 */
      }
    }

    if (s_loudAccumMs >= NOISE_SUSTAIN_MS)
    {
      TriggerAlarm();
      s_loudAccumMs = 0;
      s_quietStreakMs = 0;
      HAL_Delay(ALARM_COOLDOWN_MS);     /* 冷却,避免连环触发 */
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
