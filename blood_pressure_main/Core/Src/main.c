

#include "main.h"
#include "lcd.h"
#include "images.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>

/* ===========================================================================
   PIN / PORT DEFINITIONS
   =========================================================================== */
#define BUTTON_PORT         GPIOA
#define BUTTON_PIN          GPIO_PIN_1

#define BUTTON_RST_PORT     GPIOA
#define BUTTON_RST_PIN      GPIO_PIN_3

#define HX710B_SCK_PORT     GPIOA
#define HX710B_SCK_PIN      GPIO_PIN_2
#define HX710B_OUT_PORT     GPIOA
#define HX710B_OUT_PIN      GPIO_PIN_0

#define VALVE_PWM_PORT       GPIOB
#define VALVE_PWM_PIN        GPIO_PIN_6
#define VALVE_TIM_CHANNEL    TIM_CHANNEL_1

/* ===========================================================================
   HX710B CALIBRATION
   =========================================================================== */
#define RAW_READING_1       1176000.0f
#define PRESSURE_CAL_1      0.00f
#define RAW_READING_2       1600000.0f
#define PRESSURE_CAL_2      10.0f

/* ===========================================================================
   SYSTEM THRESHOLDS
   =========================================================================== */
#define PRESSURE_TARGET      160.0f
#define PRESSURE_MAX         180.0f
#define PRESSURE_SPIKE_DELTA  50.0f

/* ===========================================================================
   OSCILLOMETRIC ENVELOPE PARAMETERS
   =========================================================================== */
#define ENVELOPE_STEP            2.0f
#define MAX_ENVELOPE_POINTS     80
#define MIN_ENVELOPE_FOR_CALC   10
#define SYS_RATIO   0.55f
#define DIA_RATIO   0.70f

/* ===========================================================================
   MAP-BAYESIAN PARAMETERS
   =========================================================================== */
#define BAYES_LM_ITERS          30
#define BAYES_LM_LAMBDA_INIT     0.01f
#define PRIOR_MU_MEAN           90.0f
#define PRIOR_MU_STD            50.0f
#define PRIOR_SIGMA_MEAN        20.0f
#define PRIOR_SIGMA_STD         16.0f

/* ===========================================================================
   VALVE PWM (TIM4, PB6) & PI CONTROLLER MACROS
   =========================================================================== */
#define VALVE_TIM_PRESCALER       9
#define VALVE_TIM_PERIOD        999
#define VALVE_DUTY_MIN             0
#define VALVE_DUTY_MAX  (VALVE_TIM_PERIOD + 1)

#define DEFLATE_TARGET_RATE       2.0f   /* mmHg/s - giảm từ 3.0 để xả chậm/êm hơn (2-3 mmHg/s là khoảng khuyến cáo lâm sàng) */
#define DEFLATE_CONTROL_PERIOD_MS 100
#define DEFLATE_KP                15.0f
#define DEFLATE_KI                8.0f
#define DEFLATE_FF_BASE           220.0f /* hạ nhẹ theo tốc độ xả mới; có thể thấp hơn vùng chết của van -> xem bù vùng chết bên dưới */

/* ---------------------------------------------------------------------------

--------------------------------------------------------------------------- */
#define VALVE_DEADZONE_DUTY       150     /* TODO: đo lại trên van thật, đơn vị duty (0..999) */
#define VALVE_DITHER_PERIOD_MS    400     /* chu kỳ băm xung (ms) */

/* ===========================================================================
   TIMING CONSTANTS  (ms)
   =========================================================================== */
#define HX710B_READ_INTERVAL    10
#define UART_SEND_INTERVAL     500
#define BUTTON_DEBOUNCE_MS     200
#define HOLDING_TIME_MS       1000
#define DEFLATE_TIMEOUT_MS   50000
#define PURGE_TIMEOUT_MS     10000
#define PURGE_DONE_PRESSURE    2.0f
#define MAX_PUMP_TIME_MS     30000
#define LCD_UPDATE_INTERVAL    300

/* ===========================================================================
   THÊM MỚI: UART RECEIVE (Nhận lệnh từ ESP32)
   =========================================================================== */
#define UART_RX_BUF_SIZE    64          /* Kích thước vùng đệm nhận */
#define CMD_START_MEASURE   "START_MEASURE"

/* ===========================================================================
   TYPES
   =========================================================================== */
typedef enum { INIT, IDLE, PUMPING, HOLDING, DEFLATING, PURGING, DONE, ALARM } SYSTEM_STATE;

typedef struct {
    float pressure;
    float amplitude;
} OscillometricPoint;

typedef enum { BP_QUALITY_GOOD, BP_QUALITY_WARN, BP_QUALITY_BAD } BP_Quality;

typedef struct {
    float sys;
    float dia;
    float map;
    float confidence;
    uint8_t valid;
} BPResult;

/* ===========================================================================
   HAL HANDLES
   =========================================================================== */
SPI_HandleTypeDef  hspi1;
SPI_HandleTypeDef  hspi2;
DMA_HandleTypeDef  hdma_spi1_tx;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef  htim4;

/* ===========================================================================
   THÊM MỚI: BIẾN UART RECEIVE
   =========================================================================== */
/* 1 byte nhận ngắt tại một thời điểm (HAL IT mode) */
static uint8_t  uart_rx_byte    = 0;

/* Buffer tích lũy từng ký tự cho đến khi gặp '\n' */
static char     uart_rx_buf[UART_RX_BUF_SIZE];
static uint8_t  uart_rx_idx     = 0;

/* Cờ báo main loop: có lệnh START_MEASURE mới */
static volatile uint8_t uart_start_flag = 0;

/* ===========================================================================
   GLOBAL STATE
   =========================================================================== */
static SYSTEM_STATE state      = INIT;
static SYSTEM_STATE last_state = INIT;

static float   pressure        = 0.0f;
static float   prev_pressure   = 0.0f;
static float   pressure_offset = 0.0f;

static float   baseline_pressure = 0.0f;
static float   pulse_wave_amp    = 0.0f;
static float   max_oscillation   = 0.0f;
static float   peak_amp_in_bin   = 0.0f;

static float sys_pressure = 0.0f;
static float dia_pressure = 0.0f;
static float map_pressure = 0.0f;

static OscillometricPoint oscillogram[MAX_ENVELOPE_POINTS];
static uint8_t  envelope_count      = 0;
static uint8_t  first_envelopePoint = 1;
static float    last_saved_pressure = 0.0f;

static int32_t  last_raw_value = 0;

static uint8_t  bp_frozen  = 0;
static uint8_t  bp_drawn   = 0;

static uint32_t last_adc_read          = 0;
static uint32_t last_uart_send         = 0;
static uint32_t state_start_time       = 0;
static uint32_t last_button_press_time = 0;
static uint32_t last_btn_rst_time      = 0;
static uint32_t last_lcd_update        = 0;

static uint16_t valve_duty             = 0;
static uint32_t last_deflate_calc_tick = 0;
static float    pressure_at_last_calc  = 0.0f;
static float    deflate_rate_ema       = 0.0f;
static float    deflate_integral       = 0.0f;

/* THÊM MỚI: mốc thời gian của chu kỳ băm xung (dead-zone dither) */
static uint32_t dither_cycle_start_tick = 0;

static char tx_buffer[192];

/* ===========================================================================
   FORWARD DECLARATIONS
   =========================================================================== */
static void       Delay_us(uint32_t us);
static void       Pump_ON(void);
static void       Pump_OFF(void);
static void       Valve_SetDuty(uint16_t duty);
static void       Valve_SetDuty_DeadzoneAware(float duty_f, uint32_t tick);
static void       Valve_Open_Full(void);
static void       Valve_Close(void);
static void       Deflate_Control_Reset(uint32_t tick);
static void       Deflate_Control_Update(uint32_t tick);
static int32_t    HX710B_ReadRaw(void);
static float      HX710B_RawToMmHg(int32_t raw);
static void       Calibrate_Pressure(void);
static float      Read_Pressure(void);
static void       Update_Oscillometric(float p);
static void       Reset_Oscillometric_Data(void);
static BP_Quality BP_Quality_Check(void);
static void       Send_Data_UART(float p);
static void       Clear_Screen(void);
static void       Display_Welcome(void);
static void       Display_Static_UI(SYSTEM_STATE s);
static void       Display_Dynamic_UI(SYSTEM_STATE s, float p);
static BPResult   Algo_MAP_Bayesian(void);
static void       Calculate_BP(void);

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);

/* ===========================================================================
   THÊM MỚI: UART RX CALLBACK — Chạy trong ngắt, KHÔNG blocking
   ===========================================================================
   Mỗi lần nhận 1 byte, HAL gọi hàm này.
   Ta tích lũy vào buf đến khi gặp '\n' thì kiểm tra lệnh,
   sau đó kích hoạt lại ngắt cho byte kế tiếp.
   =========================================================================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    char c = (char)uart_rx_byte;

    if (c == '\n' || c == '\r')
    {
        /* Kết thúc dòng → kiểm tra lệnh */
        if (uart_rx_idx > 0)
        {
            uart_rx_buf[uart_rx_idx] = '\0';    /* Null-terminate */

            /* So sánh với lệnh START_MEASURE (không phân biệt hoa thường không cần,
               ESP32 gửi đúng chữ hoa) */
            if (strcmp(uart_rx_buf, CMD_START_MEASURE) == 0)
            {
                uart_start_flag = 1;            /* Báo main loop xử lý */
            }

            uart_rx_idx = 0;                    /* Reset buffer */
        }
    }
    else
    {
        /* Còn giữa dòng: tích lũy ký tự, tránh tràn buffer */
        if (uart_rx_idx < (UART_RX_BUF_SIZE - 1))
        {
            uart_rx_buf[uart_rx_idx++] = c;
        }
        else
        {
            /* Buffer tràn → reset để tránh rác */
            uart_rx_idx = 0;
        }
    }

    /* Kích hoạt lại ngắt nhận byte tiếp theo */
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}

/* ===========================================================================
   UTILITY
   =========================================================================== */
static void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000UL);
    while ((DWT->CYCCNT - start) < ticks);
}

static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* ===========================================================================
   ACTUATOR CONTROL
   =========================================================================== */
static void Pump_ON(void)   { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   }
static void Pump_OFF(void)  { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); }

static void Valve_SetDuty(uint16_t duty)
{
    if (duty > VALVE_DUTY_MAX) duty = VALVE_DUTY_MAX;
    valve_duty = duty;
    __HAL_TIM_SET_COMPARE(&htim4, VALVE_TIM_CHANNEL, duty);
}


static void Valve_SetDuty_DeadzoneAware(float duty_f, uint32_t tick)
{
    if (duty_f <= 0.0f)
    {
        Valve_SetDuty(0);
        return;
    }

    if (duty_f >= (float)VALVE_DEADZONE_DUTY)
    {
        Valve_SetDuty((uint16_t)duty_f);
        return;
    }

    /* duty_f nằm trong vùng chết -> chuyển sang chế độ băm xung */
    float    on_ratio   = duty_f / (float)VALVE_DEADZONE_DUTY;        /* 0..1 */
    uint32_t on_time_ms = (uint32_t)(on_ratio * VALVE_DITHER_PERIOD_MS);

    uint32_t phase = (tick - dither_cycle_start_tick);
    if (phase >= VALVE_DITHER_PERIOD_MS)
    {
        dither_cycle_start_tick = tick;
        phase = 0;
    }

    if (phase < on_time_ms) Valve_SetDuty(VALVE_DEADZONE_DUTY);
    else                    Valve_SetDuty(0);
}

static void Valve_Open_Full(void) { Valve_SetDuty(VALVE_DUTY_MAX); }
static void Valve_Close(void)     { Valve_SetDuty(VALVE_DUTY_MIN); }

/* ===========================================================================
   DEFLATE CONTROL (PI + Feedforward + Dead-zone compensation)
   =========================================================================== */
static void Deflate_Control_Reset(uint32_t tick)
{
    valve_duty               = 0;
    Valve_SetDuty(valve_duty);
    last_deflate_calc_tick   = tick;
    pressure_at_last_calc    = pressure;
    deflate_rate_ema         = DEFLATE_TARGET_RATE;
    deflate_integral         = 0.0f;
    dither_cycle_start_tick  = tick;   /* THÊM MỚI: đồng bộ lại pha băm xung */
}

static void Deflate_Control_Update(uint32_t tick)
{
    if ((tick - last_deflate_calc_tick) < DEFLATE_CONTROL_PERIOD_MS) return;

    float dt_s = (tick - last_deflate_calc_tick) / 1000.0f;
    if (dt_s < 0.001f) dt_s = 0.001f;

    float raw_rate = (pressure_at_last_calc - pressure) / dt_s;
    if (raw_rate < -1.0f) raw_rate = -1.0f;

    deflate_rate_ema = 0.6f * deflate_rate_ema + 0.4f * raw_rate;

    float error = DEFLATE_TARGET_RATE - deflate_rate_ema;

    float next_integral = deflate_integral + (error * dt_s);
    if (next_integral >  400.0f) next_integral =  400.0f;
    if (next_integral < -100.0f) next_integral = -100.0f;
    deflate_integral = next_integral;

    float ff = DEFLATE_FF_BASE + (160.0f - pressure) * 0.8f;
    if (ff < DEFLATE_FF_BASE) ff = DEFLATE_FF_BASE;

    float duty_f = ff + (DEFLATE_KP * error) + (DEFLATE_KI * deflate_integral);
    if (duty_f > 800.0f) duty_f = 800.0f;
    if (duty_f < 0.0f)   duty_f = 0.0f;

    /* THÊM MỚI: xuất duty qua lớp bù vùng chết thay vì gửi thẳng */
    Valve_SetDuty_DeadzoneAware(duty_f, tick);

    pressure_at_last_calc  = pressure;
    last_deflate_calc_tick = tick;
}

/* ===========================================================================
   HX710B LOW-LEVEL DRIVER
   =========================================================================== */
static int32_t HX710B_ReadRaw(void)
{
    int32_t  value = 0;
    uint32_t start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(HX710B_OUT_PORT, HX710B_OUT_PIN) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start) > 100) return -1;
    }

    for (uint8_t i = 0; i < 24; i++)
    {
        HAL_GPIO_WritePin(HX710B_SCK_PORT, HX710B_SCK_PIN, GPIO_PIN_SET);
        Delay_us(2);
        value <<= 1;
        if (HAL_GPIO_ReadPin(HX710B_OUT_PORT, HX710B_OUT_PIN) == GPIO_PIN_SET)
            value |= 1;
        HAL_GPIO_WritePin(HX710B_SCK_PORT, HX710B_SCK_PIN, GPIO_PIN_RESET);
        Delay_us(2);
    }

    HAL_GPIO_WritePin(HX710B_SCK_PORT, HX710B_SCK_PIN, GPIO_PIN_SET);
    Delay_us(2);
    HAL_GPIO_WritePin(HX710B_SCK_PORT, HX710B_SCK_PIN, GPIO_PIN_RESET);
    Delay_us(2);

    if (value & 0x800000) value |= (int32_t)0xFF000000;
    return value;
}

static float HX710B_RawToMmHg(int32_t raw)
{
    static const float slope = (PRESSURE_CAL_2 - PRESSURE_CAL_1) /
                               (RAW_READING_2  - RAW_READING_1);
    float raw_f = (float)raw - pressure_offset;
    return PRESSURE_CAL_1 + slope * (raw_f - (RAW_READING_1 - pressure_offset));
}

static void Calibrate_Pressure(void)
{
    int64_t sum   = 0;
    uint8_t count = 0;
    for (int i = 0; i < 20; i++)
    {
        int32_t raw = HX710B_ReadRaw();
        if (raw != -1) { sum += raw; count++; }
        HAL_Delay(20);
    }
    if (count > 0) pressure_offset = (float)(sum / count);
}

static float Read_Pressure(void)
{
    int32_t raw = HX710B_ReadRaw();
    last_raw_value = raw;
    if (raw == -1) return pressure;

    float np = HX710B_RawToMmHg(raw);
    if (np < 0.0f)   np = 0.0f;
    if (np > 300.0f) return pressure;

    if (pressure > 1.0f && fabsf(np - pressure) > PRESSURE_SPIKE_DELTA)
        return pressure;

    pressure      = 0.8f * pressure + 0.2f * np;
    prev_pressure = pressure;
    return pressure;
}

/* ===========================================================================
   OSCILLOMETRIC ENGINE
   =========================================================================== */
static void Update_Oscillometric(float p)
{
    if (state != DEFLATING)
    {
        baseline_pressure = p;
        peak_amp_in_bin   = 0.0f;
        return;
    }

    const float alpha = 0.02f;
    if (baseline_pressure < 0.5f) baseline_pressure = p;
    else baseline_pressure = alpha * p + (1.0f - alpha) * baseline_pressure;

    pulse_wave_amp = fabsf(p - baseline_pressure);
    if (pulse_wave_amp > peak_amp_in_bin) peak_amp_in_bin = pulse_wave_amp;

    if (first_envelopePoint || fabsf(p - last_saved_pressure) >= ENVELOPE_STEP)
    {
        if (envelope_count < MAX_ENVELOPE_POINTS)
        {
            oscillogram[envelope_count].pressure  = p;
            oscillogram[envelope_count].amplitude = peak_amp_in_bin;
            if (peak_amp_in_bin > max_oscillation) max_oscillation = peak_amp_in_bin;
            envelope_count++;
            last_saved_pressure = p;
            first_envelopePoint = 0;
            peak_amp_in_bin     = 0.0f;
        }
    }
}

/* ===========================================================================
   MAP-BAYESIAN ESTIMATOR
   =========================================================================== */
static BPResult Algo_MAP_Bayesian(void)
{
    BPResult r = {0};
    if (envelope_count < MIN_ENVELOPE_FOR_CALC) return r;

    float A0    = 0.0f;
    float mu    = PRIOR_MU_MEAN;
    float sigma = PRIOR_SIGMA_MEAN;

    for (uint16_t i = 0; i < envelope_count; i++)
    {
        if (oscillogram[i].amplitude > A0)
        {
            A0 = oscillogram[i].amplitude;
            mu = oscillogram[i].pressure;
        }
    }
    if (A0 < 0.001f) return r;

    const float inv_var_mu    = 1.0f / (PRIOR_MU_STD    * PRIOR_MU_STD);
    const float inv_var_sigma = 1.0f / (PRIOR_SIGMA_STD * PRIOR_SIGMA_STD);

    float lambda = BAYES_LM_LAMBDA_INIT;

    for (int iter = 0; iter < BAYES_LM_ITERS; iter++)
    {
        float JtJ[3][3] = {{0}};
        float Jtr[3]    = {0};
        float J_obj     = 0.0f;

        for (uint16_t k = 0; k < envelope_count; k++)
        {
            float P   = oscillogram[k].pressure;
            float A_k = oscillogram[k].amplitude;
            float diff  = P - mu;
            float sig2  = sigma * sigma;
            float expo  = expf(-diff * diff / (2.0f * sig2));
            float f     = A0 * expo;
            float resid = A_k - f;

            float dA0    =  expo;
            float dmu    =  A0 * expo * diff / sig2;
            float dsigma =  A0 * expo * diff * diff / (sig2 * sigma);
            float Jr[3]  = { dA0, dmu, dsigma };

            for (int a = 0; a < 3; a++)
            {
                Jtr[a] += Jr[a] * resid;
                for (int b = 0; b < 3; b++) JtJ[a][b] += Jr[a] * Jr[b];
            }
            J_obj += resid * resid;
        }

        {
            float r_mu    = (PRIOR_MU_MEAN    - mu);
            float r_sigma = (PRIOR_SIGMA_MEAN - sigma);
            JtJ[1][1] += inv_var_mu;    Jtr[1] += inv_var_mu    * r_mu;
            JtJ[2][2] += inv_var_sigma; Jtr[2] += inv_var_sigma * r_sigma;
            J_obj     += r_mu * r_mu * inv_var_mu + r_sigma * r_sigma * inv_var_sigma;
        }

        for (int a = 0; a < 3; a++) JtJ[a][a] *= (1.0f + lambda);

        float M[3][4];
        for (int a = 0; a < 3; a++)
        {
            for (int b = 0; b < 3; b++) M[a][b] = JtJ[a][b];
            M[a][3] = Jtr[a];
        }

        uint8_t singular = 0;
        for (int a = 0; a < 3; a++)
        {
            if (fabsf(M[a][a]) < FLT_EPSILON) { singular = 1; break; }
            float inv = 1.0f / M[a][a];
            for (int b = a; b < 4; b++) M[a][b] *= inv;
            for (int c = 0; c < 3; c++)
            {
                if (c == a) continue;
                float fac = M[c][a];
                for (int b = a; b < 4; b++) M[c][b] -= fac * M[a][b];
            }
        }

        if (singular) { lambda *= 10.0f; continue; }

        float A0_new    = A0    + M[0][3];
        float mu_new    = mu    + M[1][3];
        float sigma_new = fabsf(sigma + M[2][3]);

        float J_new = 0.0f;
        for (uint16_t k = 0; k < envelope_count; k++)
        {
            float P = oscillogram[k].pressure;
            float diff = P - mu_new;
            float f  = A0_new * expf(-diff * diff / (2.0f * sigma_new * sigma_new));
            float res = oscillogram[k].amplitude - f;
            J_new += res * res;
        }
        {
            float r_mu    = PRIOR_MU_MEAN    - mu_new;
            float r_sigma = PRIOR_SIGMA_MEAN - sigma_new;
            J_new += r_mu * r_mu * inv_var_mu + r_sigma * r_sigma * inv_var_sigma;
        }

        if (J_new < J_obj)
        {
            A0    = A0_new;
            mu    = mu_new;
            sigma = sigma_new;
            lambda *= 0.1f;
        }
        else
        {
            lambda *= 10.0f;
            if (lambda > 1e6f) break;
        }
    }

    sigma = clampf(fabsf(sigma), 5.0f, 50.0f);
    mu    = clampf(mu, 40.0f, 220.0f)- 25 ;

    float dynamic_sys_ratio = (mu > 110.0f) ? 0.60f : SYS_RATIO;
    float sys_offset = sigma * sqrtf(-2.0f * logf(dynamic_sys_ratio));
    float dia_offset = sigma * sqrtf(-2.0f * logf(DIA_RATIO))  ;

    r.map = mu ;
    r.sys = mu + sys_offset  ;
    r.dia = mu - dia_offset  ;

    float rms = 0.0f;
    for (uint16_t k = 0; k < envelope_count; k++)
    {
        float diff = oscillogram[k].pressure - mu;
        float f    = A0 * expf(-diff * diff / (2.0f * sigma * sigma));
        float res  = oscillogram[k].amplitude - f;
        rms += res * res;
    }
    rms = sqrtf(rms / envelope_count);
    float nrmse = (A0 > 1e-4f) ? clampf(rms / A0, 0.0f, 1.0f) : 1.0f;
    r.confidence = 1.0f - nrmse;
    r.valid = 1;
    return r;
}

static void Calculate_BP(void)
{
    if (envelope_count < MIN_ENVELOPE_FOR_CALC) return;
    BPResult r = Algo_MAP_Bayesian();
    if (r.valid) {
        sys_pressure = r.sys;
        dia_pressure = r.dia;
        map_pressure = r.map;
    }
}

static void Reset_Oscillometric_Data(void)
{
    envelope_count      = 0;
    first_envelopePoint = 1;
    max_oscillation     = 0.0f;
    baseline_pressure   = 0.0f;
    sys_pressure        = 0.0f;
    dia_pressure        = 0.0f;
    map_pressure        = 0.0f;
    peak_amp_in_bin     = 0.0f;
    last_saved_pressure = 0.0f;
    memset(oscillogram, 0, sizeof(oscillogram));

    if (pressure <= 5.0f)
    {
        int32_t cur = HX710B_ReadRaw();
        if (cur != -1) pressure_offset = (float)cur;
    }
}

static BP_Quality BP_Quality_Check(void)
{
    if (envelope_count < MIN_ENVELOPE_FOR_CALC)          return BP_QUALITY_BAD;
    if (sys_pressure < 60.0f || sys_pressure > 250.0f)  return BP_QUALITY_BAD;
    if (dia_pressure < 30.0f || dia_pressure > 150.0f)  return BP_QUALITY_BAD;
    if (map_pressure < 50.0f || map_pressure > 200.0f)  return BP_QUALITY_BAD;
    if (sys_pressure <= dia_pressure)                    return BP_QUALITY_BAD;
    if (map_pressure < dia_pressure || map_pressure > sys_pressure)
                                                         return BP_QUALITY_WARN;
    if (max_oscillation < 0.5f)                          return BP_QUALITY_WARN;
    return BP_QUALITY_GOOD;
}

static void Send_Data_UART(float p)
{
    snprintf(tx_buffer, sizeof(tx_buffer), "P=%.2f mmHg\r\n", p);
    HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, strlen(tx_buffer), 100);
}

/* ===========================================================================
   LCD
   =========================================================================== */
static void Clear_Screen(void) { UG_FillScreen(C_BLACK); }

static void Display_Welcome(void)
{
    Clear_Screen();
    LCD_PutStr(30,  10, "MAY DO HUYET AP",  DEFAULT_FONT, C_CYAN,   C_BLACK);
    UG_DrawLine(20, 35, 220, 35, C_DARK_GREEN);
    UG_DrawCircle(120, 70, 30, C_RED);
    UG_DrawCircle(120, 70, 25, C_RED);
    UG_DrawCircle(120, 70, 20, C_RED);
    LCD_PutStr(45, 120, "NHAN NUT DE",      DEFAULT_FONT, C_ORANGE, C_BLACK);
    LCD_PutStr(55, 140, "BAT DAU BOM",      DEFAULT_FONT, C_ORANGE, C_BLACK);
    LCD_PutStr(20, 170, "ALG: MAP BAYESIAN", DEFAULT_FONT, C_YELLOW, C_BLACK);
    UG_Update();
}

static void Display_Static_UI(SYSTEM_STATE s)
{
    Clear_Screen();
    LCD_PutStr(10, 10, "TRANG THAI:", DEFAULT_FONT, C_WHITE, C_BLACK);
    if (s != DONE) LCD_PutStr(10, 40, "HUYET AP:", DEFAULT_FONT, C_WHITE, C_BLACK);

    switch (s)
    {
        case IDLE:
            LCD_PutStr(100, 10, "CHO DE DO    ", DEFAULT_FONT, C_GREEN,  C_BLACK);
            UG_DrawLine(10, 30, 230, 30, C_GRAY);
            LCD_PutStr(10,  70, "DIEN THE:",    DEFAULT_FONT, C_WHITE,  C_BLACK);
            LCD_PutStr(10,  90, "ADC:",         DEFAULT_FONT, C_WHITE,  C_BLACK);
            LCD_PutStr(30, 160, "Nhan nut de bat dau", DEFAULT_FONT, C_ORANGE, C_BLACK);
            break;
        case PUMPING:
            LCD_PutStr(100, 10, "DANG BOM...  ", DEFAULT_FONT, C_RED,    C_BLACK);
            UG_DrawLine(10, 30, 230, 30, C_RED);
            LCD_PutStr(10,  70, "DIEN THE:",    DEFAULT_FONT, C_WHITE,  C_BLACK);
            LCD_PutStr(10,  90, "ADC:",         DEFAULT_FONT, C_WHITE,  C_BLACK);
            UG_DrawFrame(10, 115, 230, 140, C_WHITE);
            LCD_PutStr(10, 165, "MUC TIEU: 160 mmHg", DEFAULT_FONT, C_YELLOW, C_BLACK);
            break;
        case HOLDING:
            LCD_PutStr(100, 10, "GIU AP SUAT  ", DEFAULT_FONT, C_YELLOW, C_BLACK);
            UG_DrawLine(10, 30, 230, 30, C_YELLOW);
            UG_DrawCircle(120, 90, 30, C_GREEN);
            LCD_PutStr(105, 82, "OK",           DEFAULT_FONT, C_BLACK,  C_GREEN);
            LCD_PutStr(50, 140, "Dat muc tieu!", DEFAULT_FONT, C_GREEN,  C_BLACK);
            break;
        case DEFLATING:
            LCD_PutStr(100, 10, "DANG XA AP... ", DEFAULT_FONT, C_ORANGE, C_BLACK);
            UG_DrawLine(10, 30, 230, 30, C_ORANGE);
            LCD_PutStr(50, 130, "Doi gia tri...", DEFAULT_FONT, C_YELLOW, C_BLACK);
            UG_DrawFrame(10, 155, 230, 180, C_WHITE);
            break;
        case PURGING:
            LCD_PutStr(100, 10, "XA HET KHI... ", DEFAULT_FONT, C_ORANGE, C_BLACK);
            UG_DrawLine(10, 30, 230, 30, C_ORANGE);
            LCD_PutStr(20, 130, "Dang xa het khi cu...", DEFAULT_FONT, C_YELLOW, C_BLACK);
            UG_DrawFrame(10, 155, 230, 180, C_WHITE);
            break;
        case DONE:
            LCD_PutStr(100, 10, "HOAN THANH    ", DEFAULT_FONT, C_GREEN,  C_BLACK);
            UG_DrawLine(10, 30, 230, 30, C_GREEN);
            UG_DrawLine(10, 185, 230, 185, C_GRAY);
            LCD_PutStr(15, 195, "Nhan RESET de do lai", DEFAULT_FONT, C_ORANGE, C_BLACK);
            break;
        case ALARM:
            UG_FillFrame(0, 0, 240, 40, C_RED);
            LCD_PutStr(10,  10, "  CANH BAO!  ", DEFAULT_FONT, C_WHITE,  C_RED);
            UG_DrawLine(10, 50, 230, 50, C_RED);
            LCD_PutStr(25, 100, "AP SUAT QUA CAO!", DEFAULT_FONT, C_RED,    C_BLACK);
            LCD_PutStr(30, 130, "May dang xa ap",   DEFAULT_FONT, C_YELLOW, C_BLACK);
            break;
        default: break;
    }
    UG_Update();
}

static void Display_Dynamic_UI(SYSTEM_STATE s, float p)
{
    char buf[28];

    if (!bp_frozen && s != DONE)
    {
        UG_FillFrame(95, 40, 220, 55, C_BLACK);
        snprintf(buf, sizeof(buf), "%.1f mmHg", p);
        LCD_PutStr(95, 40, buf, DEFAULT_FONT, (s == ALARM) ? C_RED : C_GREEN, C_BLACK);
    }

    switch (s)
    {
        case IDLE:
        case PUMPING:
        {
            float voltage = (p / 100.0f) * 3.3f;
            float adc_val = (p / 330.0f) * 4095.0f;
            UG_FillFrame(95, 70, 200, 85, C_BLACK);
            snprintf(buf, sizeof(buf), "%.2f V", voltage);
            LCD_PutStr(95, 70, buf, DEFAULT_FONT, C_GREEN, C_BLACK);
            UG_FillFrame(95, 90, 200, 105, C_BLACK);
            snprintf(buf, sizeof(buf), "%.0f", adc_val);
            LCD_PutStr(95, 90, buf, DEFAULT_FONT, C_GREEN, C_BLACK);
            uint16_t bar_w = (uint16_t)((p / 200.0f) * 220.0f);
            if (bar_w > 220) bar_w = 220;
            if (bar_w > 0) UG_FillFrame(11, 116, 11 + bar_w, 139, C_GREEN);
            UG_FillFrame(11 + bar_w + 1, 116, 229, 139, C_BLACK);
            snprintf(buf, sizeof(buf), "%3.0f%%", (p / 200.0f) * 100.0f);
            LCD_PutStr(60, 145, buf, DEFAULT_FONT, C_GREEN, C_BLACK);
            break;
        }
        case DEFLATING:
        case PURGING:
        {
            uint16_t bar_w = (uint16_t)((p / 200.0f) * 220.0f);
            if (bar_w > 220) bar_w = 220;
            if (bar_w > 0) UG_FillFrame(11, 156, 11 + bar_w, 179, C_ORANGE);
            UG_FillFrame(11 + bar_w + 1, 156, 229, 179, C_BLACK);
            break;
        }
        case DONE:
        {
            if (bp_drawn) break;
            BP_Quality  q     = BP_Quality_Check();
            const char *q_str = (q == BP_QUALITY_GOOD) ? "  OK  " :
                                (q == BP_QUALITY_WARN) ? " WARN " : "  BAD ";
            uint16_t    q_col = (q == BP_QUALITY_GOOD) ? C_GREEN  :
                                (q == BP_QUALITY_WARN) ? C_YELLOW : C_RED;
            UG_FillFrame(20, 60, 220, 180, C_BLACK);
            snprintf(buf, sizeof(buf), "SYS: %3d mmHg", (int)sys_pressure);
            LCD_PutStr(30,  70, buf, DEFAULT_FONT, C_GREEN, C_BLACK);
            snprintf(buf, sizeof(buf), "DIA: %3d mmHg", (int)dia_pressure);
            LCD_PutStr(30, 105, buf, DEFAULT_FONT, C_GREEN, C_BLACK);
            snprintf(buf, sizeof(buf), "MAP: %3d mmHg", (int)map_pressure);
            LCD_PutStr(30, 140, buf, DEFAULT_FONT, C_CYAN,  C_BLACK);
            LCD_PutStr(70, 170, q_str, DEFAULT_FONT, q_col, C_BLACK);
            UG_FillFrame(10, 200, 239, 215, C_BLACK);
            bp_drawn  = 1;
            bp_frozen = 1;
            break;
        }
        default: break;
    }
    UG_Update();
}

/* ===========================================================================
   MAIN
   =========================================================================== */
int main(void)
{
    HAL_Init();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();

    LCD_init();
    UG_FontSetTransparency(1);

    Pump_OFF();
    Valve_Close();
    HAL_GPIO_WritePin(HX710B_SCK_PORT, HX710B_SCK_PIN, GPIO_PIN_RESET);

    Display_Welcome();
    HAL_Delay(2000);
    Calibrate_Pressure();

    state      = IDLE;
    last_state = INIT;


    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

    /* =====================================================================
       MAIN LOOP
       ===================================================================== */
    while (1)
    {
        uint32_t tick = HAL_GetTick();

        /* 1. READ SENSOR */
        if ((tick - last_adc_read) >= HX710B_READ_INTERVAL)
        {
            last_adc_read = tick;
            pressure = Read_Pressure();
            Update_Oscillometric(pressure);
        }

        /* 2. UART TELEMETRY (gửi áp suất định kỳ) */
        if ((tick - last_uart_send) >= UART_SEND_INTERVAL && state != IDLE)
        {
            Send_Data_UART(pressure);
            last_uart_send = tick;
        }

        /* 3. NÚT BẤM VẬT LÝ (PA1) */
        static uint8_t last_btn = GPIO_PIN_SET;
        uint8_t        curr_btn = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
        if (last_btn == GPIO_PIN_SET && curr_btn == GPIO_PIN_RESET)
        {
            if ((tick - last_button_press_time) >= BUTTON_DEBOUNCE_MS && state != INIT)
            {
                if (state == IDLE)
                {
                    Reset_Oscillometric_Data();
                    bp_frozen = 0; bp_drawn = 0;
                    state = PUMPING;
                }
                else if (state == ALARM) { Pump_OFF(); Valve_Open_Full(); }
                else                     { Pump_OFF(); Valve_Open_Full(); state = IDLE; }
                last_button_press_time = tick;
            }
        }
        last_btn = curr_btn;


        if (uart_start_flag)
        {
            uart_start_flag = 0;    /* Xóa cờ ngay để tránh xử lý lặp */

            if (state == IDLE)
            {
                /* Ghi log xác nhận ra UART trước khi TX_buffer bị ghi đè */
                const char *ack = "ACK:START_MEASURE\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)ack, strlen(ack), 100);

                Reset_Oscillometric_Data();
                bp_frozen = 0;
                bp_drawn  = 0;
                state     = PUMPING;
            }
            else
            {
                /* STM32 đang bận (đang đo) → báo lại cho ESP32 biết */
                const char *busy = "BUSY\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)busy, strlen(busy), 100);
            }
        }

        /* 4. NÚT RESET (PA3) */
        static uint8_t last_btn_rst = GPIO_PIN_SET;
        uint8_t        curr_btn_rst = HAL_GPIO_ReadPin(BUTTON_RST_PORT, BUTTON_RST_PIN);
        if (last_btn_rst == GPIO_PIN_SET && curr_btn_rst == GPIO_PIN_RESET)
        {
            if ((tick - last_btn_rst_time) >= BUTTON_DEBOUNCE_MS)
            {
                Pump_OFF();
                Valve_Open_Full();
                HAL_Delay(5);
                NVIC_SystemReset();
            }
        }
        last_btn_rst = curr_btn_rst;

        /* 5. DISPLAY */
        if (state != last_state)
        {
            Display_Static_UI(state);
            last_state       = state;
            state_start_time = tick;
            if (state == DEFLATING) Deflate_Control_Reset(tick);
        }
        if ((tick - last_lcd_update) >= LCD_UPDATE_INTERVAL)
        {
            Display_Dynamic_UI(state, pressure);
            if (!bp_frozen)
            {
                char dbg[40];
                snprintf(dbg, sizeof(dbg), "R:%ld O:%ld",
                         (long)last_raw_value, (long)(int32_t)pressure_offset);
                UG_FillFrame(10, 200, 239, 215, C_BLACK);
                LCD_PutStr(10, 200, dbg, DEFAULT_FONT, C_CYAN, C_BLACK);
                UG_Update();
            }
            last_lcd_update = tick;
        }

        /* 6. STATE MACHINE */
        switch (state)
        {
            case IDLE:
                Pump_OFF();
                Valve_Close();
                break;

            case PUMPING:
                if ((tick - state_start_time) >= MAX_PUMP_TIME_MS)
                    { Pump_OFF(); Valve_Open_Full(); state = ALARM; break; }
                if (pressure >= PRESSURE_MAX)
                    { Pump_OFF(); Valve_Open_Full(); state = ALARM; break; }
                if (pressure >= PRESSURE_TARGET)
                    { Pump_OFF(); Valve_Close(); state = HOLDING; }
                else
                    { Pump_ON();  Valve_Close(); }
                break;

            case HOLDING:
                Pump_OFF(); Valve_Close();
                if ((tick - state_start_time) >= HOLDING_TIME_MS)
                    state = DEFLATING;
                break;

            case DEFLATING:
                Pump_OFF();
                Deflate_Control_Update(tick);
                if (pressure <= 5.0f ||
                    (tick - state_start_time) >= DEFLATE_TIMEOUT_MS)
                {
                    Valve_Close();
                    Calculate_BP();
                    snprintf(tx_buffer, sizeof(tx_buffer),
                             "SYS:%d DIA:%d MAP:%d Q:%d\r\n",
                             (int)sys_pressure, (int)dia_pressure,
                             (int)map_pressure, (int)BP_Quality_Check());
                    HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer,
                                      strlen(tx_buffer), 200);
                    state = PURGING;
                }
                break;

            case PURGING:
                Pump_OFF();
                Valve_Open_Full();
                if (pressure <= PURGE_DONE_PRESSURE ||
                    (tick - state_start_time) >= PURGE_TIMEOUT_MS)
                {
                    Valve_Close();
                    state = DONE;
                }
                break;

            case DONE:
                Pump_OFF();
                Valve_Close();
                break;

            case ALARM:
                Pump_OFF(); Valve_Open_Full();
                if (pressure <= 5.0f)
                    { Valve_Close(); state = IDLE; }
                break;

            default:
                state = IDLE;
                break;
        }
    }
}

/* ===========================================================================
   CLOCK & PERIPHERAL INIT
   =========================================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 100;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
        Error_Handler();
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_SPI2_Init(void)
{
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi2.Init.NSS               = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) Error_Handler();
}

static void MX_TIM4_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = VALVE_TIM_PRESCALER;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = VALVE_TIM_PERIOD;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) Error_Handler();

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, VALVE_TIM_CHANNEL) != HAL_OK)
        Error_Handler();

    HAL_TIM_PWM_Start(&htim4, VALVE_TIM_CHANNEL);
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;     /* TX + RX đều bật */
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LCD_DC_Pin | LCD_RST_Pin | LCD_CS_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = button_Pin | button_rst_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = GPIO_PIN_2 | GPIO_PIN_4;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = LCD_DC_Pin | LCD_RST_Pin | LCD_CS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = VALVE_PWM_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(VALVE_PWM_PORT, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
