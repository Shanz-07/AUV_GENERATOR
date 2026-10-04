/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Adaptive Software-Defined Sonar Transmitter (SIH26058)
  *                   NUCLEO-F446RE MVP (scaled twin of the 100-500 kHz payload)
  *                   NOTE: turbidity coefficients are an uncalibrated engineering
  *                         prior until replaced by tank measurements.
  *
  * ARCHITECTURE
  *   Sense   : 5 ADC channels (16x oversampled + EMA) -> physical units
  *   Estimate: Medwin sound speed + Ainslie-McColm absorption + turbidity loss
  *   Decide  : fc, B, tau, A, waveform, window  (zone hysteresis, no chatter)
  *   Generate: 32-bit phase-accumulator DDS, 1024-pt interpolated sine LUT,
  *             Tukey/Hann/Hamming/Blackman windows
  *   Transmit: TIM6 TRGO -> DAC -> DMA1 Stream5 (zero CPU per sample)
  *   Power   : ping-pong buffers (next ping computed WHILE DMA streams),
  *             CPU in WFI Sleep during the DMA burst, then true STOP MODE
  *             (RTC-wakeup timer, LSI auto-calibrated, flash power-down, low-power
  *             regulator) for ~90% of every ping interval, clocks restored on wake,
  *             ADC powered only during sensing, DAC + TIM6 off between pings,
  *             DWT-measured CPU run-duty, optional INA226 current logging
  *   Speed   : windows cached in RAM, whole ping reused when quantized
  *             parameters are unchanged (zero synthesis cost in steady state),
  *             UART telemetry at 921600 baud
  *   Safety  : IWDG watchdog, DMA error trap, transmitter parked on fault
  *   UI      : PC13 button = EXTI falling-edge interrupt (pull-up), cycles
  *             waveform override AUTO -> LFM -> GeoFM -> HFM -> Barker13 -> AUTO
  *   Debug   : plain-text telemetry on USART2 (ST-LINK VCP) for PuTTY / Tera Term:
  *             921600 8N1, one block every TELEMETRY_EVERY pings.
  *             TELEMETRY_ANSI 1 = in-place dashboard, 0 = scrolling log.
  *   Rev 2   : - Barker-13 chip rate tied to the allocated bandwidth, main lobe kept
  *               inside the hardware band, integer carrier cycles per chip (every
  *               phase flip on a zero crossing), short edge ramp instead of the
  *               25 % Tukey taper (PSLR back to ~-21 dB instead of -13 dB)
  *             - sample clock from HSE bypass (ST-LINK 8 MHz MCO) with automatic
  *               HSI fallback; telemetry reports the clock source
  *             - "fs measured" replaced by TX IRQ latency (DWT and TIM6 share one
  *               clock, so DWT can never measure a sample-clock error)
  *             - optional MATCH_PAYLOAD_TBP: stretch tau so the MVP keeps the
  *               payload's time-bandwidth product (true scaled pulse compression)
  *
  * CUBEMX REQUIREMENTS (.ioc)
  *   DAC1_OUT1 (PA4) trigger TIM6 TRGO, DMA1 Stream5 Ch7, Mem->Periph,
  *   Normal mode, Halfword/Halfword, Priority High, DMA1_Stream5 IRQ enabled.
  *   TIM6: PSC=0, ARR=83 (84 MHz/84 = 1 MS/s), TRGO = Update.
  *   ADC1: PA0 (IN0 turbidity) PA1 (IN1 temp) PC0 (IN10 depth) PC1 (IN11 salinity)
  *         PB0 (IN8 MISSION TARGET RANGE, log 10..300 m).  USART2 921600 8N1.
  *   PC13: GPIO_EXTI13, falling edge, pull-up, EXTI15_10 IRQ enabled (0,0).
  *   stm32f4xx_it.c must call HAL_DMA_IRQHandler(&hdma_dac1) in
  *   DMA1_Stream5_IRQHandler and HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13) in
  *   EXTI15_10_IRQHandler (CubeMX generates both).
  ******************************************************************************
  */
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
  WAVEFORM_LOGFM,          /* geometric (exponential) sweep */
  WAVEFORM_HFM,            /* hyperbolic FM, Doppler tolerant */
  WAVEFORM_PHASECODED,     /* Barker-13 BPSK */
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
  float range_m;           /* MISSION target range (operator input, not derived from depth) */
} EnvSensors_t;

typedef struct
{
  WaveformType_t type;
  WindowType_t   window;
  float f_start_hz;        /* hardware-band sweep start */
  float f_end_hz;          /* hardware-band sweep end   */
  float duration_ms;
  float amplitude_scale;   /* 0.2 .. 0.95 of DAC full scale */
  /* physical (payload-domain) quantities, kept for telemetry */
  float fc_model_hz;
  float bw_model_hz;
  float range_m;
  float sound_speed;
  float alpha_dbkm;
  float res_mm;            /* range resolution c/2B in the payload domain */
  uint32_t pri_ms;         /* ping repetition interval */
} SonarParams_t;

/* Quantized parameter set: identical key => identical waveform => reuse buffer */
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
  uint32_t      n;         /* samples incl. tail pad */
  uint8_t       valid;     /* buffer holds a synthesized pulse for 'key' */
  uint8_t       hit;       /* last PreparePing reused the buffer */
} Ping_t;

/* USER CODE END PTD */

/* USER CODE BEGIN PD */

/* ---- Profile: MVP is a scaled twin of the final payload ----------------- */
#define PROFILE_MVP
/* #define PROFILE_PAYLOAD */   /* external >=10 MS/s DAC on STM32H7 / FPGA */

#if defined(PROFILE_MVP)
  #define DAC_SAMPLE_RATE   1000000UL      /* internal 12-bit DAC          */
  #define BAND_F_MIN_HZ     20000.0f       /* hardware band                */
  #define BAND_F_MAX_HZ     50000.0f
  #define BAND_F_MAX_HZ_INT 50000UL
  #define MODEL_F_MIN_HZ    100000.0f      /* physical band the MVP stands */
  #define MODEL_F_MAX_HZ    500000.0f      /* in for (maps linearly)       */
  #define TIM_CLK_HZ        84000000UL
  _Static_assert((TIM_CLK_HZ % DAC_SAMPLE_RATE) == 0, "TIM6 divider must be integer");
#else
  /* Deployment target only: port the clock tree, timer, DAC/DMA and MSP
     layer to the selected STM32H7/FPGA + external DAC before enabling this. */
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

/* Nyquist guard: at least 2.5 samples per cycle at the top of the band */
_Static_assert((DAC_SAMPLE_RATE * 2UL) >= (BAND_F_MAX_HZ_INT * 5UL),
               "Sample rate too low for band");

/* ---- Pulse buffers ------------------------------------------------------ */
#define MAX_PULSE_MS        7.0f             /* 7 ms leaves room for MATCH_PAYLOAD_TBP */
#define MAX_PULSE_SAMPLES   ((DAC_SAMPLE_RATE / 1000UL) * 7UL)
#define TAIL_PAD            2U               /* mid-scale samples after pulse */

/* The F446RE SRAM cannot hold the full 10-MS/s payload buffers. Keep the
   deployable profile as an architectural target unless a larger-memory MCU
   or external RAM is used. */
#if defined(PROFILE_MVP)
  _Static_assert((2UL * (MAX_PULSE_SAMPLES + TAIL_PAD) * sizeof(uint16_t) +
                  MAX_PULSE_SAMPLES * sizeof(float)) < (96UL * 1024UL),
                 "MVP waveform buffers exceed the intended F446 SRAM budget");
#endif
#define BARKER13_LEN        13U
#define MIN_PULSE_SAMPLES   (BARKER13_LEN * 8U)

/* ---- Barker-13 planning --------------------------------------------------
   Chip rate Rc = BARKER_RC_PER_B * (allocated HW bandwidth), so the coded pulse
   uses the same spectrum allocation as the FM pulses. The carrier is pulled
   inward so the sinc main lobe (fc +- Rc) stays inside BAND_F_MIN..MAX, and
   every chip holds an INTEGER number of carrier cycles, so each phase flip
   lands on a zero crossing (no voltage step, no DC). tau follows: 13 chips.
   Edges get a short raised-cosine ramp; a 25 % Tukey taper attenuates ~1.6
   chips per side and destroys the Barker sidelobe cancellation (-22 -> -13 dB). */
#define BARKER_RC_PER_B     1.0f
#define BARKER_RAMP_SAMPLES 8U
#define WIN_BARKER_GUARD    ((WindowType_t)(WIN_COUNT + 1))   /* internal only, never sent */

/* ---- Scaled-twin option ---------------------------------------------------
   The MVP maps frequency by 0.075x but keeps tau, so B*T drops from ~45
   (payload) to ~3.4: essentially no pulse compression. With 1, tau of the FM
   pulses is stretched by B_model/B_hw so B*T matches the payload (capped at
   MAX_PULSE_MS). Longer bursts = more TX-awake time and bigger DAC frames. */
#define MATCH_PAYLOAD_TBP   0

/* ---- Clock source ----------------------------------------------------------
   1 = HSE bypass from the ST-LINK 8 MHz MCO (NUCLEO default wiring, see UM1724
   "OSC clock supply"). Falls back to HSI automatically if no HSE clock is seen.
   HSI is +-1 % at 25 C and drifts with temperature, which shifts every
   transmitted frequency by the same percentage. */
#define CLOCK_USE_HSE_BYPASS 1

/* ---- Sensor scaling (pot 0..1 -> physical units) ------------------------ */
#define NTU_MAX             200.0f
#define TEMP_MIN_C          (-2.0f)
#define TEMP_MAX_C          35.0f
#define DEPTH_MAX_M         2000.0f
#define SAL_MAX_PSU         40.0f
#define EMA_ALPHA           0.30f
#define ADC_OVERSAMPLE      16U

/* ---- Adaptation model constants ----------------------------------------- */
#define BW_FRACTION         0.20f            /* B = 0.2 fc                   */
#define ABS_BUDGET_DB       20.0f            /* 2*alpha*R <= 20 dB           */
#define FC_STEP_HZ          5000.0f
#define TURB_FREQ_EXP       1.0f             /* engineering prior; calibrate from tank data */
#define TURB_MODEL_CALIBRATED 0             /* set to 1 only after measured tank calibration */
#define WATER_PH            8.0f
/* Mission target range is an INPUT (5th pot on PB0 / ADC IN8, log scale 10..300 m).
   Depth/temperature/salinity only affect sound speed + absorption. */
#define USE_RANGE_POT       1
#define MISSION_RANGE_FIXED_M 50.0f          /* used when USE_RANGE_POT == 0 */
#define RANGE_MIN_M         10.0f
#define RANGE_MAX_M         300.0f
#define TAU_PER_M_MS        0.02f
#define TAU_MIN_MS          0.5f
#define AMP_MIN             0.20f
/* STM32F4 DAC with output buffer ON only guarantees 0.2 V .. VDDA-0.2 V
   (codes ~248..3847 at 3.3 V). Symmetric swing around code 2048 must stay
   inside that, so cap at 0.85 to avoid flat-topping and harmonics. */
#define AMP_MAX             0.85f
/* Keep DAC enabled and parked at mid-scale between pings (no DC step at
   ping start/end). Costs some DAC supply current in Stop - measure it.
   Set 0 only if the analog front end is DC-biased to mid-rail externally. */
#define DAC_PARK_ENABLED    1
/* Amplitude law: A = AMP_MIN * 10^((loss - LOSS_REF_DB)/20)  (true voltage dB law).
   The DAC drive window AMP_MIN..AMP_MAX is only ~12.6 dB wide, so LOSS_REF_DB
   places that window over the mid-range missions: loss <= 85 dB -> 20 %,
   loss >= ~97.6 dB -> AMP_MAX (saturated: raise the power-amplifier gain/rail
   for those missions). Re-calibrate on the real transducer + amplifier. */
#define LOSS_REF_DB         85.0f
#define DRIVE_DB_DIV        20.0f
#define PRI_MIN_MS          60U
#define PRI_GUARD_MS        10.0f
#define TUKEY_ALPHA         0.25f
#define HFM_SEGMENTS        64U              /* piecewise-linear HFM phase-step approximation */

/* ---- Low-power (Stop mode) ---------------------------------------------- */
#define STOP_MIN_MS         12U              /* don't bother stopping for less */
#define STOP_MARGIN_MS      3U               /* wake early, finish in Sleep    */
#define KEEP_DEBUG_IN_STOP  0                /* production/power-measurement default */
#define LSI16_MIN_HZ        1000U            /* LSI/16 sanity window (17-47 kHz LSI) */
#define LSI16_MAX_HZ        3200U

extern volatile uint32_t uwTick;             /* HAL tick, advanced manually after Stop */

/* ---- Button (PC13, EXTI falling edge) ----------------------------------- */
#define BUTTON_DEBOUNCE_MS  200U

/* ---- Misc --------------------------------------------------------------- */
#define LUT_BITS            10U
#define LUT_SIZE            (1UL << LUT_BITS)
#define TWO_PI_F            6.28318530718f

/* ---- Telemetry ---------------------------------------------------------- */
#define TELEMETRY_ENABLE    1                /* set 0 for power runs          */
#define TELEMETRY_EVERY     8U               /* print one block every N pings */
#define TELEMETRY_BUDGET_MS 12U              /* block is ~700 B = ~8 ms @921600 */
#define TELEMETRY_ANSI      0                /* 1 = live in-place dashboard (PuTTY/Tera Term/minicom) */
#define TELEMETRY_BUF_SIZE  896U

#define PIPELINE_GUARD_US   50U
#define ADC_CONV_TIMEOUT_MS 2U
/* #define USE_INA226 */                     /* needs I2C1 in CubeMX */

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/

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

/* Ping-pong buffers: one streams via DMA while the other is being computed */
static uint16_t pulse_buf[2][MAX_PULSE_SAMPLES + TAIL_PAD];
static Ping_t   ping[2];

static volatile uint8_t tx_done  = 0;
static volatile uint8_t tx_error = 0;

/* Written from the EXTI ISR, read in the main loop */
static volatile uint8_t  wave_override = 0;   /* 0 = AUTO, 1..4 = forced waveform */
static volatile uint32_t btn_last_ms   = 0;

static uint32_t sleep_cycles  = 0;
static uint32_t gen_us        = 0;
static uint32_t duty_pm       = 0;        /* CPU run-duty, permille of the ping period */
static uint32_t cache_hits    = 0;
static volatile uint8_t rtc_woke = 0;
static uint32_t lsi16_hz      = 2000;     /* calibrated LSI/16 */
static uint8_t  stop_ok       = 0;

static float        win_cache[MAX_PULSE_SAMPLES];
static WindowType_t win_cache_type = WIN_COUNT;
static uint32_t     win_cache_n    = 0;
static uint32_t fault_count   = 0;
static int32_t  i_active_uA   = -1;
static int32_t  i_sleep_uA    = -1;
static int32_t  v_bus_mV      = 0;
static volatile uint32_t tx_t0     = 0;   /* DWT stamp when sample clock starts   */
static volatile uint32_t tx_cycles = 0;   /* DWT cycles from start to DMA TC      */
static volatile uint32_t tx_us_meas = 0;
static uint32_t tx_us_expected = 0;
static uint32_t pipeline_miss_count = 0;
static uint32_t tx_timeout_count = 0;
static uint32_t adc_error_count = 0;
static uint32_t synthesis_count = 0;
static uint32_t synthesis_overrun_count = 0;
static uint32_t max_gen_us = 0;
static uint8_t pipeline_safe = 1;
static uint32_t pipeline_ready_us = 0;  /* elapsed control time until next buffer is ready */
static uint32_t pipeline_slack_us = 0;  /* remaining PRI budget after next buffer preparation */
static uint32_t telemetry_overrun_count = 0;
static uint32_t telemetry_max_us = 0;
static uint8_t  clk_hse = 0;              /* 1 = PLL locked to HSE, 0 = HSI fallback */

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
  ParkDacAtMid();                 /* output sits at 2048 (~1.65 V) from boot onward */
#endif
#ifdef USE_INA226
  Ina226_Init();
#endif
  WatchdogInit();
  RtcWakeInit();
  RtcCalibrate();

  /* Prime first ping, then enter the ping-pong loop */
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

    /* 1) Launch DMA stream of the ready ping: no CPU involvement from here */
    StartTx(pulse_buf[cur], ping[cur].n);
#ifdef USE_INA226
    i_active_uA = Ina226_ReadCurrent_uA();
    v_bus_mV    = Ina226_ReadBus_mV();
#endif

    /* 2) While DMA streams, sense + decide + synthesize the NEXT ping */
    {
      const uint32_t g0 = DWT->CYCCNT;
      PreparePing(&ping[nxt], pulse_buf[nxt]);
      gen_us = (DWT->CYCCNT - g0) / (SystemCoreClock / 1000000UL);
      if (gen_us > max_gen_us) max_gen_us = gen_us;
      tx_us_expected = (uint32_t)(((uint64_t)ping[cur].n * 1000000ULL) / DAC_SAMPLE_RATE);

      /* Measure the real elapsed control time from ping start until the next
         waveform becomes ready. This is the quantity that matters for the
         schedule; comparing only gen_us to the acoustic burst duration was
         misleading because the next ping is launched after the full PRI. */
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
          /* The next waveform is complete and remains intact. If the complete
             control schedule has already consumed the PRI, the following ping
             necessarily starts late; stale data is never substituted silently. */
        }
      }
    }

    /* 3) Sleep until DMA transfer-complete interrupt */
    if (!WaitTxDone((uint32_t)ping[cur].p.duration_ms + 25U) || tx_error)
    {
      fault_count++;
      if (!tx_done && !tx_error) tx_timeout_count++;
    }
    StopTx();

    /* 4) Preserve the nominal PRI even when blocking UART telemetry is enabled.
          Reserve a bounded telemetry budget at the tail of the interval. */
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
      /* DWT only counts while the core clock runs. Run time excludes WFI/STOP.
         UART telemetry is intentionally excluded from this control-duty metric. */
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
      /* Finish the reserved tail if UART transmission was shorter than budget. */
      SleepUntil(deadline);
    }
#endif

    cur = nxt;
    seq++;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock -> 84 MHz SYSCLK, TIM6 clock = 84 MHz.
  *        Preferred: HSE bypass 8 MHz (ST-LINK MCO) -> PLL (M=8).
  *        Fallback : HSI 16 MHz RC -> PLL (M=16), used if HSE never becomes ready.
  *        Called again after every Stop-mode wake; the HSE probe result is
  *        remembered so a missing HSE costs the 100 ms timeout only once.
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  static int8_t hse_state = -1;            /* -1 unknown, 0 absent, 1 working */

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

#if CLOCK_USE_HSE_BYPASS
  if (hse_state != 0)
  {
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;          /* 8 MHz / 8 = 1 MHz VCO input */
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* TIM6 = 2 x 42 = 84 MHz */
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
  htim6.Init.Period = (TIM_CLK_HZ / DAC_SAMPLE_RATE) - 1UL;   /* 83 -> 1 MS/s */
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

  /* Analog sensor inputs: PA0 (turbidity) PA1 (temperature) */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PC0 (depth) PC1 (salinity) */
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PB0 (mission target range) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_1;      /* restore for any later reuse */

  /* LD2 on PA5: ON while transmitting */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /* PC13 button: falling-edge EXTI, internal pull-up (button pulls to GND).
     Cycles the waveform override from HAL_GPIO_EXTI_Callback(). */
  GPIO_InitStruct.Pin  = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

/* ======================= DDS / LUT / WINDOWS ============================== */

static void BuildSineLut(void)
{
  for (uint32_t i = 0; i <= LUT_SIZE; i++)       /* +1 guard entry for interpolation */
  {
    sine_lut[i] = sinf(TWO_PI_F * (float)i / (float)LUT_SIZE);
  }
}

/* 32-bit phase -> sine with linear interpolation (error ~ -106 dB, below 12-bit DAC floor) */
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

/* x in [0,1] turns -> 32-bit phase (24-bit resolution is plenty) */
static inline uint32_t TurnsToPhase(float x)
{
  return ((uint32_t)(x * 16777216.0f)) << 8;
}

/* Half-cosine ramp 0 -> 1 for u in [0,1] */
static inline float RaisedCosineRamp(float u)
{
  if (u >= 1.0f) return 1.0f;
  return 0.5f - 0.5f * CosLut(TurnsToPhase(0.5f * u));
}

static float WindowValue(WindowType_t w, uint32_t i, uint32_t n)
{
  /* Barker-13: flat chips, short raised-cosine edge ramp only */
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
      /* Hamming has a 0.08 pedestal: add a short guard ramp so the pulse
         never starts/ends with a voltage step (transducer protection). */
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

/* ======================= WAVEFORM SYNTHESIS =============================== */

static uint32_t PhaseStepFromHz(float f_hz)
{
  /* 1-Hz quantization is below the firmware's 100-Hz parameter resolution.
     The 64-bit product is safe at the full 500-kHz payload target. */
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


/* Window is rebuilt only when its type or length changes */
static void EnsureWindow(WindowType_t w, uint32_t n)
{
  if (w == win_cache_type && n == win_cache_n) return;
  for (uint32_t i = 0; i < n; i++) win_cache[i] = WindowValue(w, i, n);
  win_cache_type = w;
  win_cache_n = n;
}

/* Barker-13 plan for an allocated hardware band f_lo..f_hi.
   Returns the carrier and the samples per chip such that
     - chip rate Rc ~= BARKER_RC_PER_B * (f_hi - f_lo)
     - main lobe fc +- Rc lies inside BAND_F_MIN_HZ .. BAND_F_MAX_HZ
     - fc * Tc is an integer (k carrier cycles per chip -> flips on zero crossings)
   The carrier only ever moves DOWN from the allocated centre (less absorption,
   so the link budget computed for the allocated band still holds). */
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
    /* ceil -> realised carrier k*fs/spc is <= fc */
    const uint32_t spc = (uint32_t)ceilf((float)k * fs / fc);
    if (spc > spc_max)
    {
      /* Longest allowed chips: integer-cycle carrier at or just below fc
         (fc <= BAND_MAX - rc and rc >= rc_min keep the upper null in band);
         step up one cycle only if that would push the lower null out. */
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

  /* Guard against an invalid parameter set ever reaching the DDS. */
  float f0 = p->f_start_hz;
  float f1 = p->f_end_hz;
  if (f0 < BAND_F_MIN_HZ) f0 = BAND_F_MIN_HZ;
  if (f1 > BAND_F_MAX_HZ) f1 = BAND_F_MAX_HZ;
  if (f1 < f0) { const float t = f0; f0 = f1; f1 = t; }

  uint32_t phase_step = PhaseStepFromHz(f0);
  const uint32_t step0 = phase_step;
  const uint32_t step1 = PhaseStepFromHz(f1);

  /* LFM: fixed-point interpolation of the DDS increment. This removes
     float-to-integer conversion from the inner sample loop. */
  int64_t step_q16 = ((int64_t)step0) << 16;
  const int64_t step_delta_q16 = (n > 1U)
      ? ((((int64_t)step1 - (int64_t)step0) << 16) / (int64_t)(n - 1U))
      : 0;

  /* LogFM: one Q32 multiplicative ratio is calculated once, then applied with
     a 64-bit multiply per sample. */
  uint64_t log_ratio_q32 = 0x100000000ULL;
  if (p->type == WAVEFORM_LOGFM && f0 > 0.0f && f1 > 0.0f && n > 1U)
  {
    const double ratio = pow((double)f1 / (double)f0, 1.0 / (double)(n - 1U));
    if (ratio > 0.0 && ratio < 2.0)
      log_ratio_q32 = (uint64_t)(ratio * 4294967296.0 + 0.5);
  }

  /* HFM follows the inverse-frequency law. Instead of performing an expensive
     reciprocal/division for every sample, approximate the HFM phase increment
     with HFM_SEGMENTS piecewise-linear segments. Segment endpoints use the
     exact reciprocal law; the inner sample loop then uses only fixed-point adds. */
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
    /* BarkerPlan() put f0/f1 at fc -+ Rc with fc = k*fs/spc. The 100-Hz
       quantization moves fc by <= 50 Hz, far less than half a cycle per chip,
       so k is recovered exactly and the DDS step is built from k/spc directly:
       every chip is exactly k carrier cycles and starts at phase 0. */
    uint32_t kc = (uint32_t)(0.5f * (f0 + f1) * (float)spc / (float)DAC_SAMPLE_RATE + 0.5f);
    if (kc < 1U) kc = 1U;
    phase_step = (uint32_t)((((uint64_t)kc << 32) + (uint64_t)(spc / 2U)) / (uint64_t)spc);
  }

  /* Barker-13 relies on equal-amplitude chips for its correlation properties:
     flat chips with only a short edge ramp (see BARKER_RAMP_SAMPLES). */
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

  /* Park output at mid-scale after the pulse. */
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

  /* Quantize to what the hardware can actually resolve (100 Hz, 10 us, 1 %) */
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
    pg->hit = 1;                     /* identical waveform already in buffer */
    cache_hits++;
    return;
  }

  pg->key   = k;
  pg->n     = SynthesizePulse(&pg->p, buf);
  synthesis_count++;
  pg->valid = 1;
  pg->hit   = 0;
}

/* ======================= SENSING ========================================== */

static uint32_t ADC_ReadChannel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint32_t value = 0;

  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;   /* high-impedance pots */
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
  HAL_ADC_Stop(&hadc1);                              /* ADC off between reads */
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

  /* Keep the physical model inside its documented input envelope. */
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

/* ======================= PHYSICS + ADAPTATION ============================= */

/* Medwin (1975) sound speed, T in C, S in psu, z in m */
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

/* Ainslie & McColm (1998), f in kHz, result in dB/km */
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

/* Suspended-particle attenuation at 100 kHz vs turbidity (dB/km).
   These coefficients are an engineering placeholder, NOT measured ocean data.
   Replace with tank measurements before claiming calibrated environmental adaptation. */
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

/* Seawater absorption + suspended-particle loss */
static float TotalAlphaDbPerKm(float f_hz, const EnvSensors_t *e)
{
  const float f_khz = f_hz * 0.001f;
  const float a_w = AbsorptionDbPerKm(f_khz, e->temperature_c, e->salinity_psu, e->depth_m);
  const float a_t = TurbAlphaDbPerKm(f_khz, e->turbidity_ntu);
  const float total = a_w + a_t;
  return (total > 10000.0f) ? 10000.0f : ((total < 0.0f) ? 0.0f : total);
}

/* Payload-domain frequency -> hardware-band frequency (identity on payload) */
static float ModelToHw(float f_model_hz)
{
  if (f_model_hz < MODEL_F_MIN_HZ) f_model_hz = MODEL_F_MIN_HZ;
  if (f_model_hz > MODEL_F_MAX_HZ) f_model_hz = MODEL_F_MAX_HZ;
  return BAND_F_MIN_HZ + (f_model_hz - MODEL_F_MIN_HZ)
         * ((BAND_F_MAX_HZ - BAND_F_MIN_HZ) / (MODEL_F_MAX_HZ - MODEL_F_MIN_HZ));
}

/* Zone detector with symmetric hysteresis (prevents parameter chatter) */
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

  /* wave_override is changed from the EXTI ISR: take one consistent snapshot
     so the decision below cannot see two different values. */
  const uint8_t ov = wave_override;

  /* Environmental sensors are deliberately low-pass filtered. If they have
     not moved enough to change a waveform decision, reuse the previous model
     result and avoid repeating the expensive acoustic equations. */
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

  /* Mission target range: operator/mission input. Depth is NOT used as a proxy. */
  const float R = e->range_m;

  /* Highest centre frequency whose two-way absorption fits the budget */
  const float fc_hi = MODEL_F_MAX_HZ / (1.0f + 0.5f * BW_FRACTION);
  const float fc_lo = MODEL_F_MIN_HZ / (1.0f - 0.5f * BW_FRACTION);
  float fc = fc_lo;
  float alpha = 0.0f;
  uint8_t found = 0;

  /* Find the highest usable centre frequency without evaluating dozens of
     expensive acoustic-model points on every ping. First scan a coarse grid,
     then refine the boundary locally. */
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
    /* Refine only the last accepted interval. */
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

  /* Zones with hysteresis */
  z_ntu  = ZoneHyst(z_ntu,  e->turbidity_ntu, ntu_thr,   3U, 3.0f);
  z_rng  = ZoneHyst(z_rng,  R,                range_thr, 2U, 3.0f);
  z_deep = ZoneHyst(z_deep, e->depth_m,       deep_thr,  1U, 25.0f);

  /* Waveform (button override wins) */
  if (ov != 0U)                         p->type = (WaveformType_t)(ov - 1U);
  else if (z_rng == 0U && z_ntu >= 1U)  p->type = WAVEFORM_PHASECODED; /* near-field, turbid: harbour inspection */
  else if (z_ntu == 3U)                 p->type = WAVEFORM_HFM;        /* muddy: Doppler-tolerant        */
  else if (z_ntu == 2U)                 p->type = WAVEFORM_LOGFM;      /* moderate turbidity             */
  else                                  p->type = WAVEFORM_LFM;        /* clear / deep                   */

  /* Window */
  if (z_deep == 1U)       p->window = WIN_BLACKMAN;   /* max dynamic range            */
  else if (z_ntu >= 2U)   p->window = WIN_HANN;       /* cluttered / turbid           */
  else if (z_rng <= 1U)   p->window = WIN_HAMMING;    /* strong + weak targets nearby */
  else                    p->window = WIN_TUKEY;      /* max energy                   */

  /* Barker-13 ignores the window choice (flat chips + short edge ramp in
     SynthesizePulse). A fixed value keeps the cache key stable; telemetry
     prints it as "ramp" for phase-coded pulses. */
  if (p->type == WAVEFORM_PHASECODED) p->window = WIN_TUKEY;

  /* Pulse duration proportional to range */
  float tau = TAU_PER_M_MS * R;
  if (tau < TAU_MIN_MS) tau = TAU_MIN_MS;
  if (tau > MAX_PULSE_MS) tau = MAX_PULSE_MS;

  /* Allocated hardware band */
  float f_start = ModelToHw(fc - 0.5f * B);
  float f_end   = ModelToHw(fc + 0.5f * B);

  if (p->type == WAVEFORM_PHASECODED)
  {
    /* Chip rate from the allocated bandwidth; tau = 13 chips, rounded UP to the
       10-us parameter grid so n/13 in SynthesizePulse gives back the same spc.
       f_start/f_end now carry the main-lobe nulls fc -+ Rc. */
    float fcb;
    uint32_t spc;
    BarkerPlan(f_start, f_end, &fcb, &spc);
    const float rc = (float)DAC_SAMPLE_RATE / (float)spc;
    f_start = fcb - rc;
    f_end   = fcb + rc;
    tau = (float)((BARKER13_LEN * spc + 9U) / 10U) * 0.01f;      /* ms, exact integer ceil */
  }
#if MATCH_PAYLOAD_TBP
  else if ((f_end - f_start) > 1.0f)
  {
    /* Keep the payload's B*T: same compression ratio and sidelobe behaviour */
    tau *= B / (f_end - f_start);
    if (tau > MAX_PULSE_MS) tau = MAX_PULSE_MS;
  }
#endif

  /* Minimum drive for the link: spreading + absorption - matched-filter gain */
  const float loss_db = 40.0f * log10f(R) + 2.0f * alpha * R / 1000.0f - 10.0f * log10f(tau);
  float A = AMP_MIN * powf(10.0f, (loss_db - LOSS_REF_DB) / DRIVE_DB_DIV);
  if (A < AMP_MIN) A = AMP_MIN;
  if (A > AMP_MAX) A = AMP_MAX;

  /* Publish */
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

  /* PRI must cover the round trip + pulse + guard */
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

/* ======================= TRANSMIT PATH (TIM6 + DAC + DMA) ================= */

/* Enable the DAC and force code 2048 onto the pin before the first ping.
   The DAC trigger is TIM6 TRGO, so a software UG event on TIM6 is used to
   load DHR -> DOR once (the timer itself stays stopped). */
static void ParkDacAtMid(void)
{
  if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048U) != HAL_OK) Error_Handler();
  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK) Error_Handler();
  DelayUs(20);                                   /* DAC wake-up (tWAKEUP <= 10 us) */
  TIM6->EGR = TIM_EGR_UG;                        /* one TRGO pulse -> DOR = 2048    */
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

  DelayUs(20);                                   /* DAC wake-up margin */
  tx_t0 = DWT->CYCCNT;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  /* Start the sample clock directly. HAL_TIM_Base_Start() leaves htim6.State
     at BUSY (the timer is stopped with __HAL_TIM_DISABLE, not
     HAL_TIM_Base_Stop), and newer HAL versions then return HAL_ERROR on the
     2nd ping -> Error_Handler -> watchdog reset. */
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
    SleepWfi();                                  /* CPU sleeps, DMA keeps streaming */
  }
  return 1;
}

static void StopTx(void)
{
  __HAL_TIM_DISABLE(&htim6);
#if DAC_PARK_ENABLED
  /* Stop only the DMA stream; leave the DAC channel enabled so the pin keeps
     holding the last code (mid-scale tail pad = 2048). */
  (void)HAL_DMA_Abort(&hdma_dac1);
  hdma_dac1.State = HAL_DMA_STATE_READY;         /* always reusable for the next ping */
  __HAL_UNLOCK(&hdma_dac1);
  DAC->CR &= ~DAC_CR_DMAEN1;
  DAC->SR  = DAC_SR_DMAUDR1;                     /* clear any underrun flag */
  hdac.State = HAL_DAC_STATE_READY;
  __HAL_UNLOCK(&hdac);
#else
  HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);        /* DAC off between pings */
#endif
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

/* DMA transfer-complete: stop the sample clock immediately (no DAC underrun) */
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

/* ======================= POWER / TIMING / SAFETY ========================== */

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

/* Sleep mode: core clock gated, DMA/TIM6/DAC keep running. Wakes on any IRQ (SysTick 1 kHz). */
static void SleepWfi(void)
{
  const uint32_t t0 = DWT->CYCCNT;
  __WFI();
  sleep_cycles += (DWT->CYCCNT - t0);
}

/* ---- Button: PC13 EXTI falling edge --------------------------------------
   Runs in interrupt context (also wakes the MCU from Stop; StopFor() ignores
   non-RTC wakes and goes back to sleep, so ping timing is unaffected).
   The new waveform takes effect on the next PreparePing().
   NOTE: uwTick is frozen in Stop, so the debounce is measured in awake time,
   which is fine for contact bounce (milliseconds). */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin != GPIO_PIN_13) return;

  const uint32_t now = HAL_GetTick();
  if ((now - btn_last_ms) < BUTTON_DEBOUNCE_MS) return;   /* contact bounce */
  btn_last_ms = now;

  wave_override = (uint8_t)((wave_override + 1U) % (WAVEFORM_COUNT + 1U));
}

/* ---- RTC wake-up timer (register level, LSI clock, EXTI line 22) --------- */

void RTC_WKUP_IRQHandler(void)
{
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->CR &= ~RTC_CR_WUTE;                                   /* one-shot */
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

  if ((RCC->BDCR & RCC_BDCR_RTCSEL) != RCC_BDCR_RTCSEL_1)   /* not LSI yet */
  {
    RCC->BDCR |= RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
    RCC->BDCR |= RCC_BDCR_RTCSEL_1;                          /* LSI */
  }
  RCC->BDCR |= RCC_BDCR_RTCEN;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->CR &= ~RTC_CR_WUTE;
  while (!(RTC->ISR & RTC_ISR_WUTWF)) { }
  RTC->CR &= ~RTC_CR_WUCKSEL;                                /* RTCCLK / 16 */
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
  RTC->WUTR = counts - 1U;                                   /* fires after 'counts' ticks */
  RTC->ISR &= ~RTC_ISR_WUTF;
  rtc_woke = 0;
  RTC->CR |= RTC_CR_WUTE;
  RTC->WPR = 0xFF;
}

/* LSI is +-40 % part-to-part: measure it once against SysTick (Run-mode Sleep) */
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
  /* If calibration looks wrong we silently fall back to Sleep-mode idling */
}

/* Enter Stop for ~ms. Restores PLL clocks and the HAL tick on wake. */
static void StopFor(uint32_t ms)
{
  uint32_t counts = (ms * lsi16_hz) / 1000U;
  if (counts < 2U) return;
  if (counts > 65535U) counts = 65535U;

  WatchdogKick();
  RtcArm(counts);

  HAL_SuspendTick();
  PWR->CR |= PWR_CR_FPDS;                                    /* flash power-down in Stop */
  do
  {
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  } while (!rtc_woke);                                       /* ignore spurious wakes (incl. button) */

  SystemClock_Config();                                      /* back to 84 MHz PLL */
  HAL_ResumeTick();
  uwTick += (counts * 1000U) / lsi16_hz;                     /* tick was frozen in Stop */
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
      i_sleep_uA = Ina226_ReadCurrent_uA();                  /* mostly-Stop average */
#endif
      continue;
    }

    WatchdogKick();
    SleepWfi();                                              /* final few ms in Sleep */
  }
}

/* Independent watchdog (~2 s). Frozen while debugging. */
static void WatchdogInit(void)
{
  __HAL_DBGMCU_FREEZE_IWDG();
  IWDG->KR  = 0xCCCC;
  IWDG->KR  = 0x5555;
  IWDG->PR  = 4;          /* LSI/64 ~ 500 Hz */
  IWDG->RLR = 1000;       /* ~2 s            */
  uint32_t guard = 100000U;
  while (IWDG->SR && guard--) { }
  IWDG->KR  = 0xAAAA;
}

static inline void WatchdogKick(void)
{
  IWDG->KR = 0xAAAA;
}

/* ======================= TELEMETRY / OPTIONAL INA226 ====================== */

/* Block telemetry: the whole dashboard is formatted into one RAM buffer and
   sent with a single HAL_UART_Transmit(). Only integer formatting is used,
   so no "-u _printf_float" linker option is needed. */

static char     tel_buf[TELEMETRY_BUF_SIZE];
static uint32_t tel_len;

#if TELEMETRY_ANSI
  #define T_EOL "\x1b[K\r\n"                 /* clear to end of line, then newline */
#else
  #define T_EOL "\r\n"
#endif

#define UL(x)    ((unsigned long)(x))
#define KHZ_I(h) UL((h) / 1000UL)            /* Hz -> kHz, integer part    */
#define KHZ_F(h) UL(((h) % 1000UL) / 100UL)  /* Hz -> kHz, 1 decimal digit */

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
  if (!screen_init) { Tpf("\x1b[2J"); screen_init = 1; }   /* clear once */
  Tpf("\x1b[H");                                           /* cursor home every block */
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
  Tpf(" Pulse    : tau %lu us   amp %lu %%   PRI %lu ms   dR(c/2B) %lu mm" T_EOL,
      UL(tau_us), UL(amp), UL(p->pri_ms), UL(p->res_mm));
  if (p->type == WAVEFORM_PHASECODED && hw1 > hw0)
  {
    /* f_start/f_end carry the main-lobe nulls fc -+ Rc for Barker-13 */
    const uint32_t fcb = (hw0 + hw1) / 2U;
    const uint32_t rc  = (hw1 - hw0) / 2U;
    const uint32_t spc = (rc > 0U) ? (DAC_SAMPLE_RATE + rc / 2U) / rc : 0U;
    const uint32_t cyc = (uint32_t)(((uint64_t)fcb * spc + DAC_SAMPLE_RATE / 2U) / DAC_SAMPLE_RATE);
    Tpf(" Barker   : carrier %lu.%lu kHz   chips 13 x %lu us   %lu cycles/chip" T_EOL,
        KHZ_I(fcb), KHZ_F(fcb), UL(spc), UL(cyc));
  }
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
    /* Average current = duty-weighted mix of active and Stop readings.
       Energy is an estimate from INA226 samples, not a calibrated power analyser. */
    const uint32_t d = (duty_pm > 1000U) ? 1000U : duty_pm;
    const uint64_t avg_uA = ((uint64_t)i_active_uA * d + (uint64_t)i_sleep_uA * (1000U - d)) / 1000U;
    /* mV * uA * ms = 1e-12 J; convert to microjoules by dividing by 1e6. */
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
  Tpf("\x1b[J");                                           /* wipe leftovers below */
#endif

  HAL_UART_Transmit(&huart2, (uint8_t *)tel_buf, (uint16_t)tel_len, 50);
}

/* Time from the last sample trigger to the DMA transfer-complete callback.
   DWT and TIM6 run from the same PLL, so this is pure IRQ latency, never a
   sample-clock error (that can only be checked against an external reference). */
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


#ifdef USE_INA226
#define INA226_ADDR (0x40U << 1)
static void Ina226_Init(void)
{
  uint8_t d[2] = { 0x45, 0x27 };                     /* AVG16, 1.1 ms, continuous */
  HAL_I2C_Mem_Write(&hi2c1, INA226_ADDR, 0x00, 1, d, 2, 10);
  d[0] = 0x02; d[1] = 0x00;                          /* CAL=512: 0.1 ohm, 100 uA/LSB */
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
  return (int32_t)(((uint32_t)((d[0] << 8) | d[1]) * 125U) / 100U);   /* 1.25 mV/LSB */
}
#endif

/* USER CODE END 4 */

/**
  * @brief  Fault: transmitter parked (sample clock + DAC off), watchdog resets MCU.
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  TIM6->CR1 &= ~TIM_CR1_CEN;
  DAC->CR   &= ~DAC_CR_EN1;
  while (1)
  {
    /* IWDG (if running) resets the device in ~2 s */
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
