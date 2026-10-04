/* USER CODE BEGIN Header */

/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */

typedef enum
{
  WAVEFORM_LFM = 0,
  WAVEFORM_LOGFM,
  WAVEFORM_HFM,
  WAVEFORM_PHASECODED,
  WAVEFORM_COUNT
} WaveformType_t;

typedef enum
{
  WIN_TUKEY = 0,
  WIN_HANN,
  WIN_HAMMING,
  WIN_BLACKMAN,
  WIN_COUNT
} WindowType_t;

typedef struct
{
  float turbidity_ntu;
  float temperature_c;
  float depth_m;
  float salinity_psu;
  float range_m;
} EnvSensors_t;

typedef struct
{
  WaveformType_t type;
  WindowType_t   window;
  float f_start_hz;
  float f_end_hz;
  float duration_ms;
  float amplitude_scale;

  float fc_model_hz;
  float bw_model_hz;
  float range_m;
  float sound_speed;
  float alpha_dbkm;
  float res_mm;
  uint32_t pri_ms;
} SonarParams_t;

typedef struct
{
  uint8_t  type;
  uint8_t  window;
  uint8_t  amp_pct;
  uint16_t dur_10us;
  uint32_t f0_100hz;
  uint32_t f1_100hz;
} PingKey_t;

typedef struct
{
  SonarParams_t p;
  PingKey_t     key;
  uint32_t      n;
  uint8_t       valid;
  uint8_t       hit;
} Ping_t;

/* USER CODE END PTD */

/* USER CODE BEGIN PD */

#define PROFILE_MVP

#if defined(PROFILE_MVP)
  #define DAC_SAMPLE_RATE   1000000UL
  #define BAND_F_MIN_HZ     20000.0f
  #define BAND_F_MAX_HZ     50000.0f
  #define BAND_F_MAX_HZ_INT 50000UL
  #define MODEL_F_MIN_HZ    100000.0f
  #define MODEL_F_MAX_HZ    500000.0f
  #define TIM_CLK_HZ        84000000UL
  _Static_assert((TIM_CLK_HZ % DAC_SAMPLE_RATE) == 0, "TIM6 divider must be integer");
#else

  #define DAC_SAMPLE_RATE   10000000UL
  #define BAND_F_MIN_HZ     100000.0f
  #define BAND_F_MAX_HZ     500000.0f
  #define BAND_F_MAX_HZ_INT 500000UL
  #define MODEL_F_MIN_HZ    100000.0f
  #define MODEL_F_MAX_HZ    500000.0f
  #error "PROFILE_PAYLOAD is an architectural target; use the ported payload driver rather than this F446RE MSP configuration."
#endif

#if defined(PROFILE_PAYLOAD) && (DAC_SAMPLE_RATE < 10000000UL)
  #error "Payload profile requires a DAC/sample clock of at least 10 MS/s"
#endif

_Static_assert((DAC_SAMPLE_RATE * 2UL) >= (BAND_F_MAX_HZ_INT * 5UL),
               "Sample rate too low for band");

#define MAX_PULSE_MS        7.0f
#define MAX_PULSE_SAMPLES   ((DAC_SAMPLE_RATE / 1000UL) * 7UL)
#define TAIL_PAD            2U

#if defined(PROFILE_MVP)
  _Static_assert((2UL * (MAX_PULSE_SAMPLES + TAIL_PAD) * sizeof(uint16_t) +
                  MAX_PULSE_SAMPLES * sizeof(float)) < (96UL * 1024UL),
                 "MVP waveform buffers exceed the intended F446 SRAM budget");
#endif
#define BARKER13_LEN        13U
#define MIN_PULSE_SAMPLES   (BARKER13_LEN * 8U)

#define BARKER_RC_PER_B     1.0f
#define BARKER_RAMP_SAMPLES 8U
#define WIN_BARKER_GUARD    ((WindowType_t)(WIN_COUNT + 1))

#define MATCH_PAYLOAD_TBP   0

#define CLOCK_USE_HSE_BYPASS 1

#define NTU_MAX             200.0f
#define TEMP_MIN_C          (-2.0f)
#define TEMP_MAX_C          35.0f
#define DEPTH_MAX_M         2000.0f
#define SAL_MAX_PSU         40.0f
#define EMA_ALPHA           0.30f
#define ADC_OVERSAMPLE      16U

#define BW_FRACTION         0.20f
#define ABS_BUDGET_DB       20.0f
#define FC_STEP_HZ          5000.0f
#define TURB_FREQ_EXP       1.0f
#define TURB_MODEL_CALIBRATED 0
#define WATER_PH            8.0f

#define USE_RANGE_POT       1
#define MISSION_RANGE_FIXED_M 50.0f
#define RANGE_MIN_M         10.0f
#define RANGE_MAX_M         300.0f
#define TAU_PER_M_MS        0.02f
#define TAU_MIN_MS          0.5f
#define AMP_MIN             0.20f

#define AMP_MAX             0.85f

#define DAC_PARK_ENABLED    1

#define LOSS_REF_DB         85.0f
#define DRIVE_DB_DIV        20.0f
#define PRI_MIN_MS          60U
#define PRI_GUARD_MS        10.0f
#define TUKEY_ALPHA         0.25f
#define HFM_SEGMENTS        64U

#define STOP_MIN_MS         12U
#define STOP_MARGIN_MS      3U
#define KEEP_DEBUG_IN_STOP  0
#define LSI16_MIN_HZ        1000U
#define LSI16_MAX_HZ        3200U

extern volatile uint32_t uwTick;

#define BUTTON_DEBOUNCE_MS  200U

#define LUT_BITS            10U
#define LUT_SIZE            (1UL << LUT_BITS)
#define TWO_PI_F            6.28318530718f

#define TELEMETRY_ENABLE    1
#define TELEMETRY_EVERY     8U
#define TELEMETRY_BUDGET_MS 12U
#define TELEMETRY_ANSI      0
#define TELEMETRY_BUF_SIZE  896U

#define DAC_STREAM_ENABLE   1
#define DAC_STREAM_EVERY    16U

#define PIPELINE_GUARD_US   50U
#define ADC_CONV_TIMEOUT_MS 2U

/* USER CODE END PD */

ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static float sine_lut[LUT_SIZE + 1U];

static const int8_t barker13[BARKER13_LEN] = { 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1 };

static const char *const WAVE_NAME[WAVEFORM_COUNT] = { "LFM", "GeoFM", "HFM", "Barker13" };
static const char *const WIN_NAME[WIN_COUNT]       = { "Tukey", "Hann", "Hamming", "Blackman" };

static uint16_t pulse_buf[2][MAX_PULSE_SAMPLES + TAIL_PAD];
static Ping_t   ping[2];

static volatile uint8_t tx_done  = 0;
static volatile uint8_t tx_error = 0;

static volatile uint8_t  wave_override = 0;
static volatile uint32_t btn_last_ms   = 0;

static uint32_t sleep_cycles  = 0;
static uint32_t gen_us        = 0;
static uint32_t duty_pm       = 0;
static uint32_t cache_hits    = 0;
static volatile uint8_t rtc_woke = 0;
static uint32_t lsi16_hz      = 2000;
static uint8_t  stop_ok       = 0;

static float        win_cache[MAX_PULSE_SAMPLES];
static WindowType_t win_cache_type = WIN_COUNT;
static uint32_t     win_cache_n    = 0;
static uint32_t fault_count   = 0;
static int32_t  i_active_uA   = -1;
static int32_t  i_sleep_uA    = -1;
static int32_t  v_bus_mV      = 0;
static volatile uint32_t tx_t0     = 0;
static volatile uint32_t tx_cycles = 0;
static volatile uint32_t tx_us_meas = 0;
static uint32_t tx_us_expected = 0;
static uint32_t pipeline_miss_count = 0;
static uint32_t tx_timeout_count = 0;
static uint32_t adc_error_count = 0;
static uint32_t synthesis_count = 0;
static uint32_t synthesis_overrun_count = 0;
static uint32_t max_gen_us = 0;
static uint8_t pipeline_safe = 1;
static uint32_t pipeline_ready_us = 0;
static uint32_t pipeline_slack_us = 0;
static uint32_t telemetry_overrun_count = 0;
static uint32_t telemetry_max_us = 0;
static uint8_t  clk_hse = 0;

/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

static void     BuildSineLut(void);
static inline float SinLut(uint32_t phase);
static inline float CosLut(uint32_t phase);
static float    WindowValue(WindowType_t w, uint32_t i, uint32_t n);
static uint16_t SampleToDac(float s);

static uint32_t ADC_ReadChannel(uint32_t channel);
static float    ReadNorm(uint32_t channel);
static void     ReadEnvironmentalSensors(EnvSensors_t *env);

static float    SoundSpeed(float T, float S, float z_m);
static float    AbsorptionDbPerKm(float f_khz, float T, float S, float z_m);
static float    TotalAlphaDbPerKm(float f_hz, const EnvSensors_t *e);
static float    ModelToHw(float f_model_hz);
static uint8_t  ZoneHyst(uint8_t prev, float v, const float *thr, uint8_t n, float margin);
static void     DecideSonarParameters(const EnvSensors_t *env, SonarParams_t *p);

static uint32_t SynthesizePulse(const SonarParams_t *p, uint16_t *buf);
static void     BarkerPlan(float f_lo, float f_hi, float *fc_out, uint32_t *spc_out);
static uint32_t TxOverheadNs(uint32_t n);
static uint32_t PhaseStepFromHz(float f_hz);
static uint8_t PingKeyEqual(const PingKey_t *a, const PingKey_t *b);
static void     PreparePing(Ping_t *pg, uint16_t *buf);
static void     StartTx(const uint16_t *buf, uint32_t n);
static uint8_t  WaitTxDone(uint32_t timeout_ms);
static void     StopTx(void);

static void     DwtInit(void);
static void     ParkDacAtMid(void);
static void     DelayUs(uint32_t us);
static void     SleepWfi(void);
static void     SleepUntil(uint32_t deadline_tick);
static void     RtcWakeInit(void);
static void     RtcArm(uint32_t counts);
static void     RtcCalibrate(void);
static void     StopFor(uint32_t ms);
static void     EnsureWindow(WindowType_t w, uint32_t n);
static void     WatchdogInit(void);
static inline void WatchdogKick(void);
static void     Tpf(const char *fmt, ...);
static void     Telemetry(uint32_t seq, const Ping_t *pg);
#if DAC_STREAM_ENABLE
static void     DacStream(uint32_t seq, const Ping_t *pg, const uint16_t *buf);
#endif

#ifdef USE_INA226
extern I2C_HandleTypeDef hi2c1;
static void    Ina226_Init(void);
static int32_t Ina226_ReadCurrent_uA(void);
static int32_t Ina226_ReadBus_mV(void);
#endif

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t  cur = 0;
  uint32_t seq = 0;
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  BuildSineLut();
  DwtInit();
#if DAC_PARK_ENABLED
  ParkDacAtMid();
#endif
#ifdef USE_INA226
  Ina226_Init();
#endif
  WatchdogInit();
  RtcWakeInit();
  RtcCalibrate();

  PreparePing(&ping[cur], pulse_buf[cur]);
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    const uint32_t t_start = HAL_GetTick();
    const uint32_t c_start = DWT->CYCCNT;
    const uint8_t  nxt     = cur ^ 1U;
    sleep_cycles = 0;
    WatchdogKick();

    StartTx(pulse_buf[cur], ping[cur].n);
#ifdef USE_INA226
    i_active_uA = Ina226_ReadCurrent_uA();
    v_bus_mV    = Ina226_ReadBus_mV();
#endif

    {
      const uint32_t g0 = DWT->CYCCNT;
      PreparePing(&ping[nxt], pulse_buf[nxt]);
      gen_us = (DWT->CYCCNT - g0) / (SystemCoreClock / 1000000UL);
      if (gen_us > max_gen_us) max_gen_us = gen_us;
      tx_us_expected = (uint32_t)(((uint64_t)ping[cur].n * 1000000ULL) / DAC_SAMPLE_RATE);

      pipeline_ready_us = (DWT->CYCCNT - c_start) / (SystemCoreClock / 1000000UL);
      {
        const uint32_t pri_us = ping[cur].p.pri_ms * 1000UL;
        if (pri_us > pipeline_ready_us + PIPELINE_GUARD_US)
        {
          pipeline_safe = 1U;
          pipeline_slack_us = pri_us - pipeline_ready_us;
        }
        else
        {
          pipeline_safe = 0U;
          pipeline_slack_us = 0U;
          pipeline_miss_count++;
          synthesis_overrun_count++;

        }
      }
    }

    if (!WaitTxDone((uint32_t)ping[cur].p.duration_ms + 25U) || tx_error)
    {
      fault_count++;
      if (!tx_done && !tx_error) tx_timeout_count++;
    }
    StopTx();

    const uint32_t deadline = t_start + ping[cur].p.pri_ms;
    const uint8_t telemetry_due =
#if TELEMETRY_ENABLE
      ((seq % TELEMETRY_EVERY) == 0U);
#else
      0U;
#endif

    if (telemetry_due)
    {
      const int32_t rem = (int32_t)(deadline - HAL_GetTick());
      if (rem > (int32_t)TELEMETRY_BUDGET_MS)
        SleepUntil(deadline - TELEMETRY_BUDGET_MS);
      else
        SleepUntil(deadline);
    }
    else
    {
      SleepUntil(deadline);
    }

    {

      const uint32_t run = (DWT->CYCCNT - c_start) - sleep_cycles;
      const uint64_t period = (uint64_t)ping[cur].p.pri_ms * (SystemCoreClock / 1000UL);
      duty_pm = (period > 0U) ? (uint32_t)(((uint64_t)run * 1000ULL) / period) : 1000U;
    }

#if TELEMETRY_ENABLE
    if (telemetry_due)
    {
      const uint32_t telem_t0 = DWT->CYCCNT;
      Telemetry(seq, &ping[cur]);
      const uint32_t telem_us = (DWT->CYCCNT - telem_t0) / (SystemCoreClock / 1000000UL);
      if (telem_us > telemetry_max_us) telemetry_max_us = telem_us;
      if (telem_us > (TELEMETRY_BUDGET_MS * 1000UL)) telemetry_overrun_count++;

      SleepUntil(deadline);
    }
#endif

#if DAC_STREAM_ENABLE

    if ((seq % DAC_STREAM_EVERY) == 0U) DacStream(seq, &ping[cur], pulse_buf[cur]);
#endif

    cur = nxt;
    seq++;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  static int8_t hse_state = -1;

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

#if CLOCK_USE_HSE_BYPASS
  if (hse_state != 0)
  {
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    RCC_OscInitStruct.PLL.PLLR = 2;
    hse_state = (HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK) ? 1 : 0;
  }
#else
  hse_state = 0;
#endif

  if (hse_state != 1)
  {
    memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
  }
  clk_hse = (hse_state == 1) ? 1U : 0U;

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_DAC_Init(void)
{
  DAC_ChannelConfTypeDef sConfig = {0};

  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = (TIM_CLK_HZ / DAC_SAMPLE_RATE) - 1UL;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_1;

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin  = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

static void BuildSineLut(void)
{
  for (uint32_t i = 0; i <= LUT_SIZE; i++)
  {
    sine_lut[i] = sinf(TWO_PI_F * (float)i / (float)LUT_SIZE);
  }
}

static inline float SinLut(uint32_t phase)
{
  const uint32_t idx  = phase >> (32U - LUT_BITS);
  const float    frac = (float)(phase & ((1UL << (32U - LUT_BITS)) - 1UL))
                        * (1.0f / (float)(1UL << (32U - LUT_BITS)));
  const float a = sine_lut[idx];
  const float b = sine_lut[idx + 1U];
  return a + (b - a) * frac;
}

static inline float CosLut(uint32_t phase)
{
  return SinLut(phase + 0x40000000UL);
}

static inline uint32_t TurnsToPhase(float x)
{
  return ((uint32_t)(x * 16777216.0f)) << 8;
}

static inline float RaisedCosineRamp(float u)
{
  if (u >= 1.0f) return 1.0f;
  return 0.5f - 0.5f * CosLut(TurnsToPhase(0.5f * u));
}

static float WindowValue(WindowType_t w, uint32_t i, uint32_t n)
{

  if (w == WIN_BARKER_GUARD)
  {
    const float r = (float)BARKER_RAMP_SAMPLES;
    if ((float)i < r)            return RaisedCosineRamp((float)i / r);
    if ((float)(n - 1U - i) < r) return RaisedCosineRamp((float)(n - 1U - i) / r);
    return 1.0f;
  }

  const float x = (float)i / (float)(n - 1U);
  const uint32_t ph = TurnsToPhase(x);

  switch (w)
  {
    case WIN_HANN:
      return 0.5f - 0.5f * CosLut(ph);

    case WIN_HAMMING:
    {

      float v = 0.54f - 0.46f * CosLut(ph);
      float guard = (float)(n / 50U);
      if (guard < 16.0f) guard = 16.0f;
      if ((float)i < guard)             v *= RaisedCosineRamp((float)i / guard);
      else if ((float)(n - 1U - i) < guard) v *= RaisedCosineRamp((float)(n - 1U - i) / guard);
      return v;
    }

    case WIN_BLACKMAN:
      return 0.42f - 0.5f * CosLut(ph) + 0.08f * CosLut(ph << 1);

    case WIN_TUKEY:
    default:
    {
      float taper = TUKEY_ALPHA * (float)(n - 1U) * 0.5f;
      if (taper < 1.0f) taper = 1.0f;
      if ((float)i < taper)                  return RaisedCosineRamp((float)i / taper);
      if ((float)(n - 1U - i) < taper)       return RaisedCosineRamp((float)(n - 1U - i) / taper);
      return 1.0f;
    }
  }
}

static uint16_t SampleToDac(float s)
{
  if (s > 1.0f)  s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  return (uint16_t)(2048.0f + 2047.0f * s + 0.5f);
}

static uint32_t PhaseStepFromHz(float f_hz)
{

  if (f_hz <= 0.0f) return 0U;
  if (f_hz >= ((float)DAC_SAMPLE_RATE * 0.49f))
    f_hz = (float)DAC_SAMPLE_RATE * 0.49f;
  const uint64_t f_hz_i = (uint64_t)(f_hz + 0.5f);
  return (uint32_t)((f_hz_i * 4294967296ULL) / (uint64_t)DAC_SAMPLE_RATE);
}

static uint8_t PingKeyEqual(const PingKey_t *a, const PingKey_t *b)
{
  return (a->type == b->type && a->window == b->window && a->amp_pct == b->amp_pct &&
          a->dur_10us == b->dur_10us && a->f0_100hz == b->f0_100hz && a->f1_100hz == b->f1_100hz);
}

static void EnsureWindow(WindowType_t w, uint32_t n)
{
  if (w == win_cache_type && n == win_cache_n) return;
  for (uint32_t i = 0; i < n; i++) win_cache[i] = WindowValue(w, i, n);
  win_cache_type = w;
  win_cache_n = n;
}

static void BarkerPlan(float f_lo, float f_hi, float *fc_out, uint32_t *spc_out)
{
  const float fs = (float)DAC_SAMPLE_RATE;
  const uint32_t spc_max = MAX_PULSE_SAMPLES / BARKER13_LEN;
  const float rc_min = fs / (float)spc_max;
  const float rc_max = 0.5f * (BAND_F_MAX_HZ - BAND_F_MIN_HZ);

  float rc = BARKER_RC_PER_B * (f_hi - f_lo);
  if (rc < rc_min) rc = rc_min;
  if (rc > rc_max) rc = rc_max;

  float fc = 0.5f * (f_lo + f_hi);
  if (fc > BAND_F_MAX_HZ - rc) fc = BAND_F_MAX_HZ - rc;
  if (fc < BAND_F_MIN_HZ + rc) fc = BAND_F_MIN_HZ + rc;

  uint32_t k = (uint32_t)(fc / rc + 0.5f);
  if (k < 2U) k = 2U;

  for (;; k++)
  {

    const uint32_t spc = (uint32_t)ceilf((float)k * fs / fc);
    if (spc > spc_max)
    {

      uint32_t kk = (uint32_t)(fc * (float)spc_max / fs);
      if (kk < 1U) kk = 1U;
      if (((float)kk - 1.0f) * fs / (float)spc_max < BAND_F_MIN_HZ) kk++;
      *fc_out  = (float)kk * fs / (float)spc_max;
      *spc_out = spc_max;
      return;
    }
    const float fcq = (float)k * fs / (float)spc;
    const float rcq = fs / (float)spc;
    if ((fcq + rcq) <= BAND_F_MAX_HZ && (fcq - rcq) >= BAND_F_MIN_HZ)
    {
      *fc_out  = fcq;
      *spc_out = spc;
      return;
    }
  }
}

static uint32_t SynthesizePulse(const SonarParams_t *p, uint16_t *buf)
{
  uint32_t n = (uint32_t)(p->duration_ms * (float)DAC_SAMPLE_RATE * 0.001f + 0.5f);

  if (n > MAX_PULSE_SAMPLES) n = MAX_PULSE_SAMPLES;
  if (n < MIN_PULSE_SAMPLES) n = MIN_PULSE_SAMPLES;

  if (p->type == WAVEFORM_PHASECODED)
  {
    n = (n / BARKER13_LEN) * BARKER13_LEN;
    if (n < BARKER13_LEN) n = BARKER13_LEN;
  }

  float f0 = p->f_start_hz;
  float f1 = p->f_end_hz;
  if (f0 < BAND_F_MIN_HZ) f0 = BAND_F_MIN_HZ;
  if (f1 > BAND_F_MAX_HZ) f1 = BAND_F_MAX_HZ;
  if (f1 < f0) { const float t = f0; f0 = f1; f1 = t; }

  uint32_t phase_step = PhaseStepFromHz(f0);
  const uint32_t step0 = phase_step;
  const uint32_t step1 = PhaseStepFromHz(f1);

  int64_t step_q16 = ((int64_t)step0) << 16;
  const int64_t step_delta_q16 = (n > 1U)
      ? ((((int64_t)step1 - (int64_t)step0) << 16) / (int64_t)(n - 1U))
      : 0;

  uint64_t log_ratio_q32 = 0x100000000ULL;
  if (p->type == WAVEFORM_LOGFM && f0 > 0.0f && f1 > 0.0f && n > 1U)
  {
    const double ratio = pow((double)f1 / (double)f0, 1.0 / (double)(n - 1U));
    if (ratio > 0.0 && ratio < 2.0)
      log_ratio_q32 = (uint64_t)(ratio * 4294967296.0 + 0.5);
  }

  static uint32_t hfm_seg_step[HFM_SEGMENTS + 1U];
  uint32_t hfm_seg = 0U;
  uint32_t hfm_seg_end = n;
  int64_t hfm_step_q16 = 0;
  int64_t hfm_delta_q16 = 0;

  if (p->type == WAVEFORM_HFM)
  {
    for (uint32_t s = 0U; s <= HFM_SEGMENTS; s++)
    {
      const float u = (float)s / (float)HFM_SEGMENTS;
      const float inv_f = ((1.0f - u) / f0) + (u / f1);
      const float f = (inv_f > 0.0f) ? (1.0f / inv_f) : f0;
      hfm_seg_step[s] = PhaseStepFromHz(f);
    }

    hfm_step_q16 = ((int64_t)hfm_seg_step[0]) << 16;
    hfm_seg_end = (n + HFM_SEGMENTS - 1U) / HFM_SEGMENTS;
    if (hfm_seg_end == 0U || hfm_seg_end > n) hfm_seg_end = n;
    hfm_delta_q16 = (hfm_seg_end > 0U)
        ? ((((int64_t)hfm_seg_step[1] - (int64_t)hfm_seg_step[0]) << 16) /
           (int64_t)hfm_seg_end)
        : 0;
  }

  uint32_t spc = 1U, chip = 0U, chip_cnt = 0U;
  if (p->type == WAVEFORM_PHASECODED)
  {
    spc = n / BARKER13_LEN;
    if (spc == 0U) spc = 1U;

    uint32_t kc = (uint32_t)(0.5f * (f0 + f1) * (float)spc / (float)DAC_SAMPLE_RATE + 0.5f);
    if (kc < 1U) kc = 1U;
    phase_step = (uint32_t)((((uint64_t)kc << 32) + (uint64_t)(spc / 2U)) / (uint64_t)spc);
  }

  const WindowType_t active_window =
      (p->type == WAVEFORM_PHASECODED) ? WIN_BARKER_GUARD : p->window;
  EnsureWindow(active_window, n);

  uint32_t phase = 0U;
  const float amp = (p->amplitude_scale < AMP_MIN) ? AMP_MIN :
                    ((p->amplitude_scale > AMP_MAX) ? AMP_MAX : p->amplitude_scale);

  for (uint32_t i = 0; i < n; i++)
  {
    float sample = amp * win_cache[i] * SinLut(phase);

    if (p->type == WAVEFORM_PHASECODED)
    {
      sample *= (float)barker13[chip];
      if (++chip_cnt >= spc)
      {
        chip_cnt = 0U;
        if (chip < (BARKER13_LEN - 1U)) chip++;
      }
    }

    buf[i] = SampleToDac(sample);
    phase += phase_step;

    switch (p->type)
    {
      case WAVEFORM_LFM:
        step_q16 += step_delta_q16;
        if (step_q16 < 0) step_q16 = 0;
        phase_step = (uint32_t)(step_q16 >> 16);
        break;

      case WAVEFORM_LOGFM:
        phase_step = (uint32_t)(((uint64_t)phase_step * log_ratio_q32) >> 32);
        break;

      case WAVEFORM_HFM:
      {
        hfm_step_q16 += hfm_delta_q16;
        phase_step = (uint32_t)(hfm_step_q16 >> 16);

        if ((i + 1U) >= hfm_seg_end && hfm_seg < (HFM_SEGMENTS - 1U))
        {
          hfm_seg++;
          const uint32_t seg_start = (n * hfm_seg) / HFM_SEGMENTS;
          hfm_seg_end = (n * (hfm_seg + 1U)) / HFM_SEGMENTS;
          if (hfm_seg_end <= seg_start) hfm_seg_end = seg_start + 1U;
          if (hfm_seg_end > n) hfm_seg_end = n;

          hfm_step_q16 = ((int64_t)hfm_seg_step[hfm_seg]) << 16;
          const uint32_t seg_len = hfm_seg_end - seg_start;
          hfm_delta_q16 = (seg_len > 0U)
              ? ((((int64_t)hfm_seg_step[hfm_seg + 1U] -
                   (int64_t)hfm_seg_step[hfm_seg]) << 16) / (int64_t)seg_len)
              : 0;
        }
        break;
      }

      default:
        break;
    }
  }

  for (uint32_t k = 0; k < TAIL_PAD; k++) buf[n + k] = 2048U;

  return n + TAIL_PAD;
}
static void PreparePing(Ping_t *pg, uint16_t *buf)
{
  EnvSensors_t env;
  SonarParams_t t;
  PingKey_t k;

  ReadEnvironmentalSensors(&env);
  DecideSonarParameters(&env, &t);

  memset(&k, 0, sizeof(k));
  k.type     = (uint8_t)t.type;
  k.window   = (uint8_t)t.window;
  k.f0_100hz = (uint32_t)(t.f_start_hz / 100.0f + 0.5f);
  k.f1_100hz = (uint32_t)(t.f_end_hz   / 100.0f + 0.5f);
  k.dur_10us = (uint16_t)(t.duration_ms * 100.0f + 0.5f);
  k.amp_pct  = (uint8_t)(t.amplitude_scale * 100.0f + 0.5f);

  t.f_start_hz      = (float)k.f0_100hz * 100.0f;
  t.f_end_hz        = (float)k.f1_100hz * 100.0f;
  t.duration_ms     = (float)k.dur_10us * 0.01f;
  t.amplitude_scale = (float)k.amp_pct * 0.01f;

  pg->p = t;

  if (pg->valid && PingKeyEqual(&pg->key, &k))
  {
    pg->hit = 1;
    cache_hits++;
    return;
  }

  pg->key   = k;
  pg->n     = SynthesizePulse(&pg->p, buf);
  synthesis_count++;
  pg->valid = 1;
  pg->hit   = 0;
}

static uint32_t ADC_ReadChannel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint32_t value = 0;

  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    adc_error_count++;
    return 0U;
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    adc_error_count++;
    return 0U;
  }
  if (HAL_ADC_PollForConversion(&hadc1, ADC_CONV_TIMEOUT_MS) == HAL_OK)
    value = HAL_ADC_GetValue(&hadc1);
  else
    adc_error_count++;
  HAL_ADC_Stop(&hadc1);
  return value;
}

static float ReadNorm(uint32_t channel)
{
  uint32_t acc = 0;
  for (uint32_t k = 0; k < ADC_OVERSAMPLE; k++) acc += ADC_ReadChannel(channel);
  return (float)acc / (float)(ADC_OVERSAMPLE * 4095U);
}

static void ReadEnvironmentalSensors(EnvSensors_t *env)
{
  static uint8_t primed = 0;
  static EnvSensors_t f;

  EnvSensors_t raw;
  raw.turbidity_ntu = ReadNorm(ADC_CHANNEL_0)  * NTU_MAX;
  raw.temperature_c = TEMP_MIN_C + ReadNorm(ADC_CHANNEL_1) * (TEMP_MAX_C - TEMP_MIN_C);
  raw.depth_m       = ReadNorm(ADC_CHANNEL_10) * DEPTH_MAX_M;
  raw.salinity_psu  = ReadNorm(ADC_CHANNEL_11) * SAL_MAX_PSU;
#if USE_RANGE_POT
  raw.range_m       = RANGE_MIN_M * powf(RANGE_MAX_M / RANGE_MIN_M, ReadNorm(ADC_CHANNEL_8));
#else
  raw.range_m       = MISSION_RANGE_FIXED_M;
#endif

  if (raw.turbidity_ntu < 0.0f) raw.turbidity_ntu = 0.0f;
  if (raw.turbidity_ntu > NTU_MAX) raw.turbidity_ntu = NTU_MAX;
  if (raw.temperature_c < TEMP_MIN_C) raw.temperature_c = TEMP_MIN_C;
  if (raw.temperature_c > TEMP_MAX_C) raw.temperature_c = TEMP_MAX_C;
  if (raw.depth_m < 0.0f) raw.depth_m = 0.0f;
  if (raw.depth_m > DEPTH_MAX_M) raw.depth_m = DEPTH_MAX_M;
  if (raw.salinity_psu < 0.0f) raw.salinity_psu = 0.0f;
  if (raw.salinity_psu > SAL_MAX_PSU) raw.salinity_psu = SAL_MAX_PSU;
  if (raw.range_m < RANGE_MIN_M) raw.range_m = RANGE_MIN_M;
  if (raw.range_m > RANGE_MAX_M) raw.range_m = RANGE_MAX_M;

  if (!primed) { f = raw; primed = 1; }
  else
  {
    f.turbidity_ntu += EMA_ALPHA * (raw.turbidity_ntu - f.turbidity_ntu);
    f.temperature_c += EMA_ALPHA * (raw.temperature_c - f.temperature_c);
    f.depth_m       += EMA_ALPHA * (raw.depth_m       - f.depth_m);
    f.salinity_psu  += EMA_ALPHA * (raw.salinity_psu  - f.salinity_psu);
    f.range_m       += EMA_ALPHA * (raw.range_m       - f.range_m);
  }
  *env = f;
}

static float SoundSpeed(float T, float S, float z_m)
{
  if (T < TEMP_MIN_C) T = TEMP_MIN_C;
  if (T > TEMP_MAX_C) T = TEMP_MAX_C;
  if (S < 0.0f) S = 0.0f;
  if (S > SAL_MAX_PSU) S = SAL_MAX_PSU;
  if (z_m < 0.0f) z_m = 0.0f;
  if (z_m > DEPTH_MAX_M) z_m = DEPTH_MAX_M;

  const float c = 1449.2f + 4.6f * T - 0.055f * T * T + 0.00029f * T * T * T
                + (1.34f - 0.010f * T) * (S - 35.0f) + 0.016f * z_m;
  return (c > 1300.0f && c < 1700.0f) ? c : 1500.0f;
}

static float AbsorptionDbPerKm(float f_khz, float T, float S, float z_m)
{
  if (f_khz < 1.0f) f_khz = 1.0f;
  if (S < 0.1f) S = 0.1f;
  if (S > SAL_MAX_PSU) S = SAL_MAX_PSU;
  const float z_km = fmaxf(0.0f, z_m * 0.001f);
  const float f2s  = f_khz * f_khz;
  const float f1   = 0.78f * sqrtf(S / 35.0f) * expf(T / 26.0f);
  const float f2   = 42.0f * expf(T / 17.0f);

  const float boric = 0.106f * (f1 * f2s / (f1 * f1 + f2s)) * expf((WATER_PH - 8.0f) / 0.56f);
  const float mgso4 = 0.52f * (1.0f + T / 43.0f) * (S / 35.0f)
                    * (f2 * f2s / (f2 * f2 + f2s)) * expf(-z_km / 6.0f);
  const float visc  = 0.00049f * f2s * expf(-(T / 27.0f + z_km / 17.0f));
  return boric + mgso4 + visc;
}

#define TURB_PTS 5U
static const float turb_ntu_pts[TURB_PTS]  = { 0.0f, 20.0f, 60.0f, 120.0f, 200.0f };
static const float turb_a100_pts[TURB_PTS] = { 0.0f, 10.0f, 30.0f,  60.0f, 100.0f };

static float TurbAlphaDbPerKm(float f_khz, float ntu)
{
  float a100 = turb_a100_pts[TURB_PTS - 1U];
  if (ntu <= 0.0f) return 0.0f;
  for (uint32_t i = 1U; i < TURB_PTS; i++)
  {
    if (ntu <= turb_ntu_pts[i])
    {
      const float u = (ntu - turb_ntu_pts[i - 1U]) / (turb_ntu_pts[i] - turb_ntu_pts[i - 1U]);
      a100 = turb_a100_pts[i - 1U] + u * (turb_a100_pts[i] - turb_a100_pts[i - 1U]);
      break;
    }
  }
  return a100 * powf(f_khz / 100.0f, TURB_FREQ_EXP);
}

static float TotalAlphaDbPerKm(float f_hz, const EnvSensors_t *e)
{
  const float f_khz = f_hz * 0.001f;
  const float a_w = AbsorptionDbPerKm(f_khz, e->temperature_c, e->salinity_psu, e->depth_m);
  const float a_t = TurbAlphaDbPerKm(f_khz, e->turbidity_ntu);
  const float total = a_w + a_t;
  return (total > 10000.0f) ? 10000.0f : ((total < 0.0f) ? 0.0f : total);
}

static float ModelToHw(float f_model_hz)
{
  if (f_model_hz < MODEL_F_MIN_HZ) f_model_hz = MODEL_F_MIN_HZ;
  if (f_model_hz > MODEL_F_MAX_HZ) f_model_hz = MODEL_F_MAX_HZ;
  return BAND_F_MIN_HZ + (f_model_hz - MODEL_F_MIN_HZ)
         * ((BAND_F_MAX_HZ - BAND_F_MIN_HZ) / (MODEL_F_MAX_HZ - MODEL_F_MIN_HZ));
}

static uint8_t ZoneHyst(uint8_t prev, float v, const float *thr, uint8_t n, float margin)
{
  uint8_t z = prev;
  while (z < n && v > thr[z] + margin) z++;
  while (z > 0U && v < thr[z - 1U] - margin) z--;
  return z;
}

static void DecideSonarParameters(const EnvSensors_t *e, SonarParams_t *p)
{
  static uint8_t z_ntu = 0, z_rng = 0, z_deep = 0;
  static uint8_t model_valid = 0;
  static EnvSensors_t last_model_env;
  static SonarParams_t last_model_params;
  static const float ntu_thr[3]   = { 5.0f, 20.0f, 60.0f };
  static const float range_thr[2] = { 35.0f, 60.0f };
  static const float deep_thr[1]  = { 500.0f };

  const uint8_t ov = wave_override;

  if (model_valid &&
      fabsf(e->turbidity_ntu - last_model_env.turbidity_ntu) < 1.0f &&
      fabsf(e->temperature_c - last_model_env.temperature_c) < 0.25f &&
      fabsf(e->depth_m - last_model_env.depth_m) < 5.0f &&
      fabsf(e->salinity_psu - last_model_env.salinity_psu) < 0.25f &&
      fabsf(e->range_m - last_model_env.range_m) < 1.0f &&
      ov == 0U)
  {
    *p = last_model_params;
    return;
  }

  const float c = SoundSpeed(e->temperature_c, e->salinity_psu, e->depth_m);

  const float R = e->range_m;

  const float fc_hi = MODEL_F_MAX_HZ / (1.0f + 0.5f * BW_FRACTION);
  const float fc_lo = MODEL_F_MIN_HZ / (1.0f - 0.5f * BW_FRACTION);
  float fc = fc_lo;
  float alpha = 0.0f;
  uint8_t found = 0;

  const uint32_t coarse_n = 8U;
  float coarse_step = (fc_hi - fc_lo) / (float)(coarse_n - 1U);
  for (uint32_t i = 0U; i < coarse_n; i++)
  {
    const float f = fc_hi - (float)i * coarse_step;
    const float a = TotalAlphaDbPerKm(f, e);
    if ((2.0f * a * R / 1000.0f) <= ABS_BUDGET_DB)
    {
      fc = f; alpha = a; found = 1U;
      break;
    }
  }

  if (!found)
  {
    fc = fc_lo;
    alpha = TotalAlphaDbPerKm(fc_lo, e);
  }
  else if (coarse_step > FC_STEP_HZ)
  {

    const float upper = fminf(fc_hi, fc + coarse_step);
    float lo = fc;
    float hi = upper;
    for (uint32_t k = 0U; k < 5U; k++)
    {
      const float mid = 0.5f * (lo + hi);
      const float a = TotalAlphaDbPerKm(mid, e);
      if ((2.0f * a * R / 1000.0f) <= ABS_BUDGET_DB)
      {
        lo = mid;
        alpha = a;
      }
      else
      {
        hi = mid;
      }
    }
    fc = lo;
    alpha = TotalAlphaDbPerKm(fc, e);
  }

  const float B = BW_FRACTION * fc;

  z_ntu  = ZoneHyst(z_ntu,  e->turbidity_ntu, ntu_thr,   3U, 3.0f);
  z_rng  = ZoneHyst(z_rng,  R,                range_thr, 2U, 3.0f);
  z_deep = ZoneHyst(z_deep, e->depth_m,       deep_thr,  1U, 25.0f);

  if (ov != 0U)                         p->type = (WaveformType_t)(ov - 1U);
  else if (z_rng == 0U && z_ntu >= 1U)  p->type = WAVEFORM_PHASECODED;
  else if (z_ntu == 3U)                 p->type = WAVEFORM_HFM;
  else if (z_ntu == 2U)                 p->type = WAVEFORM_LOGFM;
  else                                  p->type = WAVEFORM_LFM;

  if (z_deep == 1U)       p->window = WIN_BLACKMAN;
  else if (z_ntu >= 2U)   p->window = WIN_HANN;
  else if (z_rng <= 1U)   p->window = WIN_HAMMING;
  else                    p->window = WIN_TUKEY;

  if (p->type == WAVEFORM_PHASECODED) p->window = WIN_TUKEY;

  float tau = TAU_PER_M_MS * R;
  if (tau < TAU_MIN_MS) tau = TAU_MIN_MS;
  if (tau > MAX_PULSE_MS) tau = MAX_PULSE_MS;

  float f_start = ModelToHw(fc - 0.5f * B);
  float f_end   = ModelToHw(fc + 0.5f * B);

  if (p->type == WAVEFORM_PHASECODED)
  {

    float fcb;
    uint32_t spc;
    BarkerPlan(f_start, f_end, &fcb, &spc);
    const float rc = (float)DAC_SAMPLE_RATE / (float)spc;
    f_start = fcb - rc;
    f_end   = fcb + rc;
    tau = (float)((BARKER13_LEN * spc + 9U) / 10U) * 0.01f;
  }
#if MATCH_PAYLOAD_TBP
  else if ((f_end - f_start) > 1.0f)
  {

    tau *= B / (f_end - f_start);
    if (tau > MAX_PULSE_MS) tau = MAX_PULSE_MS;
  }
#endif

  const float loss_db = 40.0f * log10f(R) + 2.0f * alpha * R / 1000.0f - 10.0f * log10f(tau);
  float A = AMP_MIN * powf(10.0f, (loss_db - LOSS_REF_DB) / DRIVE_DB_DIV);
  if (A < AMP_MIN) A = AMP_MIN;
  if (A > AMP_MAX) A = AMP_MAX;

  p->f_start_hz      = f_start;
  p->f_end_hz        = f_end;
  p->duration_ms     = tau;
  p->amplitude_scale = A;
  p->fc_model_hz     = fc;
  p->bw_model_hz     = B;
  p->range_m         = R;
  p->sound_speed     = c;
  p->alpha_dbkm      = alpha;
  p->res_mm          = 1000.0f * c / (2.0f * B);

  float pri = 2000.0f * R / c + tau + PRI_GUARD_MS;
  if (pri < (float)PRI_MIN_MS) pri = (float)PRI_MIN_MS;
  if (pri > 60000.0f) pri = 60000.0f;
  p->pri_ms = (uint32_t)(pri + 0.5f);

  if (ov == 0U)
  {
    last_model_env = *e;
    last_model_params = *p;
    model_valid = 1U;
  }
}

static void ParkDacAtMid(void)
{
  if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048U) != HAL_OK) Error_Handler();
  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK) Error_Handler();
  DelayUs(20);
  TIM6->EGR = TIM_EGR_UG;
  DelayUs(5);
  hdac.State = HAL_DAC_STATE_READY;
  __HAL_UNLOCK(&hdac);
}

static void StartTx(const uint16_t *buf, uint32_t n)
{
  tx_done = 0;
  tx_error = 0;

  __HAL_TIM_SET_COUNTER(&htim6, 0);

  if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)buf, n, DAC_ALIGN_12B_R) != HAL_OK)
  {
    Error_Handler();
  }

  DelayUs(20);
  tx_t0 = DWT->CYCCNT;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  htim6.State = HAL_TIM_STATE_READY;
  __HAL_TIM_ENABLE(&htim6);
}

static uint8_t WaitTxDone(uint32_t timeout_ms)
{
  const uint32_t t0 = HAL_GetTick();
  while (!tx_done && !tx_error)
  {
    if ((HAL_GetTick() - t0) > timeout_ms) return 0;
    WatchdogKick();
    SleepWfi();
  }
  return 1;
}

static void StopTx(void)
{
  __HAL_TIM_DISABLE(&htim6);
#if DAC_PARK_ENABLED

  (void)HAL_DMA_Abort(&hdma_dac1);
  hdma_dac1.State = HAL_DMA_STATE_READY;
  __HAL_UNLOCK(&hdma_dac1);
  DAC->CR &= ~DAC_CR_DMAEN1;
  DAC->SR  = DAC_SR_DMAUDR1;
  hdac.State = HAL_DAC_STATE_READY;
  __HAL_UNLOCK(&hdac);
#else
  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
#endif
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac_)
{
  (void)hdac_;
  __HAL_TIM_DISABLE(&htim6);
  tx_cycles = DWT->CYCCNT - tx_t0;
  tx_us_meas = tx_cycles / (SystemCoreClock / 1000000UL);
  tx_done = 1;
}

void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac_)
{
  (void)hdac_;
  __HAL_TIM_DISABLE(&htim6);
  tx_error = 1;
}

static void DwtInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void DelayUs(uint32_t us)
{
  const uint32_t t0 = DWT->CYCCNT;
  const uint32_t cycles = us * (SystemCoreClock / 1000000UL);
  while ((DWT->CYCCNT - t0) < cycles) { }
}

static void SleepWfi(void)
{
  const uint32_t t0 = DWT->CYCCNT;
  __WFI();
  sleep_cycles += (DWT->CYCCNT - t0);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin != GPIO_PIN_13) return;

  const uint32_t now = HAL_GetTick();
  if ((now - btn_last_ms) < BUTTON_DEBOUNCE_MS) return;
  btn_last_ms = now;

  wave_override = (uint8_t)((wave_override + 1U) % (WAVEFORM_COUNT + 1U));
}

void RTC_WKUP_IRQHandler(void)
{
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->CR &= ~RTC_CR_WUTE;
  RTC->ISR &= ~RTC_ISR_WUTF;
  RTC->WPR = 0xFF;
  EXTI->PR = (1UL << 22);
  rtc_woke = 1;
}

static void RtcWakeInit(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  PWR->CR |= PWR_CR_DBP;
  while (!(PWR->CR & PWR_CR_DBP)) { }

  RCC->CSR |= RCC_CSR_LSION;
  while (!(RCC->CSR & RCC_CSR_LSIRDY)) { }

  if ((RCC->BDCR & RCC_BDCR_RTCSEL) != RCC_BDCR_RTCSEL_1)
  {
    RCC->BDCR |= RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
    RCC->BDCR |= RCC_BDCR_RTCSEL_1;
  }
  RCC->BDCR |= RCC_BDCR_RTCEN;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->CR &= ~RTC_CR_WUTE;
  while (!(RTC->ISR & RTC_ISR_WUTWF)) { }
  RTC->CR &= ~RTC_CR_WUCKSEL;
  RTC->CR |= RTC_CR_WUTIE;
  RTC->WPR = 0xFF;

  EXTI->IMR  |= (1UL << 22);
  EXTI->RTSR |= (1UL << 22);
  HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

#if KEEP_DEBUG_IN_STOP
  HAL_DBGMCU_EnableDBGStopMode();
#endif
}

static void RtcArm(uint32_t counts)
{
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->CR &= ~RTC_CR_WUTE;
  while (!(RTC->ISR & RTC_ISR_WUTWF)) { }
  RTC->WUTR = counts - 1U;
  RTC->ISR &= ~RTC_ISR_WUTF;
  rtc_woke = 0;
  RTC->CR |= RTC_CR_WUTE;
  RTC->WPR = 0xFF;
}

static void RtcCalibrate(void)
{
  const uint32_t counts = 400U;
  const uint32_t t0 = HAL_GetTick();

  RtcArm(counts);
  while (!rtc_woke && (HAL_GetTick() - t0) < 1000U)
  {
    WatchdogKick();
    __WFI();
  }

  uint32_t ms = HAL_GetTick() - t0;
  if (ms == 0U) ms = 1U;
  lsi16_hz = (counts * 1000U) / ms;
  stop_ok = (rtc_woke && lsi16_hz >= LSI16_MIN_HZ && lsi16_hz <= LSI16_MAX_HZ) ? 1U : 0U;

}

static void StopFor(uint32_t ms)
{
  uint32_t counts = (ms * lsi16_hz) / 1000U;
  if (counts < 2U) return;
  if (counts > 65535U) counts = 65535U;

  WatchdogKick();
  RtcArm(counts);

  HAL_SuspendTick();
  PWR->CR |= PWR_CR_FPDS;
  do
  {
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  } while (!rtc_woke);

  SystemClock_Config();
  HAL_ResumeTick();
  uwTick += (counts * 1000U) / lsi16_hz;
}

static void SleepUntil(uint32_t deadline_tick)
{
  for (;;)
  {
    const int32_t rem = (int32_t)(deadline_tick - HAL_GetTick());
    if (rem <= 0) break;

    if (stop_ok && rem > (int32_t)STOP_MIN_MS)
    {
      const uint32_t stop_ms = (uint32_t)rem - ((uint32_t)rem >> 4) - STOP_MARGIN_MS;
      StopFor(stop_ms);
#ifdef USE_INA226
      i_sleep_uA = Ina226_ReadCurrent_uA();
#endif
      continue;
    }

    WatchdogKick();
    SleepWfi();
  }
}

static void WatchdogInit(void)
{
  __HAL_DBGMCU_FREEZE_IWDG();
  IWDG->KR  = 0xCCCC;
  IWDG->KR  = 0x5555;
  IWDG->PR  = 4;
  IWDG->RLR = 1000;
  uint32_t guard = 100000U;
  while (IWDG->SR && guard--) { }
  IWDG->KR  = 0xAAAA;
}

static inline void WatchdogKick(void)
{
  IWDG->KR = 0xAAAA;
}

static char     tel_buf[TELEMETRY_BUF_SIZE];
static uint32_t tel_len;

#if TELEMETRY_ANSI
  #define T_EOL "\x1b[K\r\n"
#else
  #define T_EOL "\r\n"
#endif

#define UL(x)    ((unsigned long)(x))
#define KHZ_I(h) UL((h) / 1000UL)
#define KHZ_F(h) UL(((h) % 1000UL) / 100UL)

static void Tpf(const char *fmt, ...)
{
  if (tel_len >= (sizeof(tel_buf) - 1U)) return;
  va_list ap;
  va_start(ap, fmt);
  const int r = vsnprintf(tel_buf + tel_len, sizeof(tel_buf) - tel_len, fmt, ap);
  va_end(ap);
  if (r > 0)
  {
    tel_len += (uint32_t)r;
    if (tel_len > (sizeof(tel_buf) - 1U)) tel_len = sizeof(tel_buf) - 1U;
  }
}

static void Telemetry(uint32_t seq, const Ping_t *pg)
{
  static uint8_t screen_init = 0;
  const SonarParams_t *p = &pg->p;
  const uint8_t ov = wave_override;

  const uint32_t fc     = (uint32_t)p->fc_model_hz;
  const uint32_t bw     = (uint32_t)p->bw_model_hz;
  const uint32_t hw0    = (uint32_t)p->f_start_hz;
  const uint32_t hw1    = (uint32_t)p->f_end_hz;
  const uint32_t tau_us = (uint32_t)(p->duration_ms * 1000.0f + 0.5f);
  const uint32_t amp    = (uint32_t)(p->amplitude_scale * 100.0f + 0.5f);
  const uint32_t lat_ns = TxOverheadNs(pg->n);
  const uint8_t healthy = (fault_count == 0U && pipeline_miss_count == 0U &&
                           adc_error_count == 0U && telemetry_overrun_count == 0U);

  tel_len = 0;
#if TELEMETRY_ANSI
  if (!screen_init) { Tpf("\x1b[2J"); screen_init = 1; }
  Tpf("\x1b[H");
#else
  (void)screen_init;
#endif

  Tpf("==[ PING #%06lu ]============================================" T_EOL, UL(seq));
  Tpf(" Waveform : %-8s / %-8s     Mode : %s" T_EOL,
      WAVE_NAME[p->type],
      (p->type == WAVEFORM_PHASECODED) ? "ramp" : WIN_NAME[p->window],
      ov ? "MANUAL" : "AUTO");
  Tpf(" Band     : fc %lu.%lu kHz   B %lu.%lu kHz   HW %lu.%lu-%lu.%lu kHz%s" T_EOL,
      KHZ_I(fc), KHZ_F(fc), KHZ_I(bw), KHZ_F(bw),
      KHZ_I(hw0), KHZ_F(hw0), KHZ_I(hw1), KHZ_F(hw1),
      (p->type == WAVEFORM_PHASECODED) ? " (main-lobe nulls)" : "");
  Tpf(" Pulse    : tau %lu us   amp %lu %%   PRI %lu ms   dR %lu mm" T_EOL,
      UL(tau_us), UL(amp), UL(p->pri_ms), UL(p->res_mm));
  Tpf(" Link     : R %lu m   c %lu m/s   alpha %lu dB/km" T_EOL,
      UL(p->range_m), UL(p->sound_speed), UL(p->alpha_dbkm));
  Tpf(" Timing   : prep %lu us   ready %lu us   slack %lu us" T_EOL,
      UL(gen_us), UL(pipeline_ready_us), UL(pipeline_slack_us));
  Tpf(" TX       : %lu / %lu us (meas/exp)   IRQ lat %lu.%lu us   clk %s" T_EOL,
      UL(tx_us_meas), UL(tx_us_expected),
      UL(lat_ns / 1000UL), UL((lat_ns % 1000UL) / 100UL),
      clk_hse ? "HSE 8M" : "HSI RC (+-1%)");
  Tpf(" Power    : CPU %lu.%lu %%   Stop %s   cache %s (%lu hit / %lu synth)" T_EOL,
      UL(duty_pm / 10U), UL(duty_pm % 10U), stop_ok ? "ON " : "OFF",
      pg->hit ? "HIT " : "MISS", UL(cache_hits), UL(synthesis_count));

  if (i_active_uA >= 0 && i_sleep_uA >= 0)
  {

    const uint32_t d = (duty_pm > 1000U) ? 1000U : duty_pm;
    const uint64_t avg_uA = ((uint64_t)i_active_uA * d + (uint64_t)i_sleep_uA * (1000U - d)) / 1000U;

    const uint64_t e_uJ   = ((uint64_t)(v_bus_mV > 0 ? v_bus_mV : 0) * avg_uA * p->pri_ms) / 1000000ULL;
    Tpf(" Current  : act %ld mA   stop %ld mA   Vbus %ld mV   avg %lu uA   E %lu uJ" T_EOL,
        (long)(i_active_uA / 1000), (long)(i_sleep_uA / 1000), (long)v_bus_mV,
        UL(avg_uA), UL(e_uJ));
  }

  Tpf(" Health   : %s   flt %lu  tmo %lu  miss %lu  adc %lu  ovr %lu  telem-ovr %lu (max %lu us)%s" T_EOL,
      healthy ? "OK  " : "WARN",
      UL(fault_count), UL(tx_timeout_count), UL(pipeline_miss_count),
      UL(adc_error_count), UL(synthesis_overrun_count),
      UL(telemetry_overrun_count), UL(telemetry_max_us),
      TURB_MODEL_CALIBRATED ? "" : "   [turb UNCAL]");
  Tpf("------------------------------------------------------------" T_EOL);
#if TELEMETRY_ANSI
  Tpf("\x1b[J");
#endif

  HAL_UART_Transmit(&huart2, (uint8_t *)tel_buf, (uint16_t)tel_len, 50);
}

static uint32_t TxOverheadNs(uint32_t n)
{
  const uint32_t mhz = SystemCoreClock / 1000000UL;
  const uint32_t cps = SystemCoreClock / DAC_SAMPLE_RATE;
  if (tx_cycles == 0U || mhz == 0U) return 0U;
  const int64_t ovh = (int64_t)tx_cycles - (int64_t)n * (int64_t)cps;
  if (ovh <= 0) return 0U;
  const uint64_t ns = ((uint64_t)ovh * 1000ULL) / mhz;
  return (ns > 65535ULL) ? 65535U : (uint32_t)ns;
}

#if DAC_STREAM_ENABLE

#define DS_HDR_LEN  46U
#define DS_CHUNK    256U

static void Put16(uint8_t *d, uint32_t v) { d[0] = (uint8_t)v; d[1] = (uint8_t)(v >> 8); }
static void Put32(uint8_t *d, uint32_t v)
{
  d[0] = (uint8_t)v; d[1] = (uint8_t)(v >> 8); d[2] = (uint8_t)(v >> 16); d[3] = (uint8_t)(v >> 24);
}

static uint32_t ClampU16(float v)
{
  if (v < 0.0f) return 0U;
  if (v > 65535.0f) return 65535U;
  return (uint32_t)(v + 0.5f);
}

static uint16_t Crc16(uint16_t crc, const uint8_t *d, uint32_t len)
{
  while (len--)
  {
    crc ^= (uint16_t)((uint16_t)(*d++) << 8);
    for (uint8_t b = 0U; b < 8U; b++)
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
  }
  return crc;
}

static void DacStream(uint32_t seq, const Ping_t *pg, const uint16_t *buf)
{
  static const uint8_t magic[4] = { 0xA5U, 0x5AU, 0xA5U, 0x5AU };
  uint8_t h[DS_HDR_LEN];
  uint8_t tail[2];
  const SonarParams_t *p = &pg->p;
  const uint32_t clk_info = (clk_hse ? 0x80000000UL : 0UL) | TxOverheadNs(pg->n);

  h[0] = 2U;
  h[1] = (uint8_t)p->type;
  h[2] = (uint8_t)p->window;
  h[3] = (uint8_t)ClampU16(p->amplitude_scale * 100.0f);
  Put32(&h[4],  seq);
  Put16(&h[8],  pg->n);
  Put32(&h[10], DAC_SAMPLE_RATE);
  Put32(&h[14], clk_info);
  Put32(&h[18], (uint32_t)(p->f_start_hz + 0.5f));
  Put32(&h[22], (uint32_t)(p->f_end_hz + 0.5f));
  Put32(&h[26], (uint32_t)(p->fc_model_hz + 0.5f));
  Put32(&h[30], (uint32_t)(p->bw_model_hz + 0.5f));
  Put16(&h[34], ClampU16(p->duration_ms * 100.0f));
  Put16(&h[36], p->pri_ms > 65535U ? 65535U : p->pri_ms);
  Put16(&h[38], ClampU16(p->range_m * 10.0f));
  Put16(&h[40], ClampU16(p->sound_speed * 10.0f));
  Put16(&h[42], ClampU16(p->alpha_dbkm));
  Put16(&h[44], ClampU16(p->res_mm));

  uint16_t crc = Crc16(0xFFFFU, h, DS_HDR_LEN);

  WatchdogKick();
  HAL_UART_Transmit(&huart2, (uint8_t *)magic, 4U, 20U);
  HAL_UART_Transmit(&huart2, h, (uint16_t)DS_HDR_LEN, 20U);

  const uint8_t *s = (const uint8_t *)buf;
  uint32_t left = pg->n * 2U;
  while (left > 0U)
  {
    const uint32_t c = (left > DS_CHUNK) ? DS_CHUNK : left;
    crc = Crc16(crc, s, c);
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)c, 50U);
    s += c;
    left -= c;
    WatchdogKick();
  }

  Put16(tail, crc);
  HAL_UART_Transmit(&huart2, tail, 2U, 20U);
}
#endif

#ifdef USE_INA226
#define INA226_ADDR (0x40U << 1)
static void Ina226_Init(void)
{
  uint8_t d[2] = { 0x45, 0x27 };
  HAL_I2C_Mem_Write(&hi2c1, INA226_ADDR, 0x00, 1, d, 2, 10);
  d[0] = 0x02; d[1] = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, INA226_ADDR, 0x05, 1, d, 2, 10);
}
static int32_t Ina226_ReadCurrent_uA(void)
{
  uint8_t d[2];
  if (HAL_I2C_Mem_Read(&hi2c1, INA226_ADDR, 0x04, 1, d, 2, 10) != HAL_OK) return -1;
  return (int32_t)((int16_t)((d[0] << 8) | d[1])) * 100;
}
static int32_t Ina226_ReadBus_mV(void)
{
  uint8_t d[2];
  if (HAL_I2C_Mem_Read(&hi2c1, INA226_ADDR, 0x02, 1, d, 2, 10) != HAL_OK) return 0;
  return (int32_t)(((uint32_t)((d[0] << 8) | d[1]) * 125U) / 100U);
}
#endif

/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  TIM6->CR1 &= ~TIM_CR1_CEN;
  DAC->CR   &= ~DAC_CR_EN1;
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
