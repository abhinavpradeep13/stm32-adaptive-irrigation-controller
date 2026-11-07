/*
 * Two-zone soil irrigation controller with ILI9341 touch display
 * NUCLEO-F401RE, register-level peripherals on top of HAL startup.
 * Replace Core/Src/main.c with this file (the Cube-generated main is not used).
 *
 * Each zone is an independent controller: its own sensor, pump, calibration, thresholds,
 * state machine, drying-rate fit and prediction.
 *
 * DEMO PROGRAM: the soil ADC values come from a software soil model (hidden moisture that dries at
 * a constant speed and rises while the pump runs), with fast demo timing and a DRY button on the
 * detail screen.  Everything else is real.  The tank is the real HC-SR04 sensor.
 * For the real soil sensors use the other program (irrigation_real.c).
 *
 * ---------------------------------------------------------------- PIN MAP
 * (all board pins are #defines or in the zone table below, except SPI1 / timer pins,
 *  which are fixed by the chip)
 *
 *  Zone 1  sensor AOUT -> PA0 (A0)    sensor VCC -> PA1 (A1, switched on only while sampling)
 *          L293D EN1,2 -> PB6 (D10, TIM4_CH1 PWM)   IN1 -> PB0 (A3)   IN2 -> PB5 (D4)
 *  Zone 2  sensor AOUT -> PA4 (A2)    sensor VCC -> PC1 (A4)
 *          L293D EN3,4 -> PB10 (D6, TIM2_CH3 PWM)   IN3 -> PA9 (D8)   IN4 -> PA8 (D7)
 *          (pump 1 on OUT1/OUT2, pump 2 on OUT3/OUT4; L293D VSS 5 V, VS pump supply, common GND)
 *  TFT/touch on SPI1: SCK -> PA5 (D13)   MISO -> PA6 (D12)   MOSI -> PA7 (D11)
 *          TFT CS -> PB8 (D15)   TFT DC -> PB9 (D14)   TFT RESET -> PA10 (D2)
 *          touch CS -> PC7 (D9)  touch IRQ -> PC0 (A5)   TFT LED -> 3.3 V, VCC 3.3 V, GND
 *  Serial  USART2 (PA2) -> ST-Link virtual COM, 115200 baud
 *  Force stop  blue user button B1 -> PC13, EXTI13 interrupt: press = pumps off + red screen,
 *          press again = back to normal operation
 *  Tank    HC-SR04: TRIG -> PB3 (D3), ECHO -> PB4 (D5, TIM3_CH1 input capture), GND -> GND.
 *          Power it from the 3.3 V pin (echo is then 3.3 V), or from 5 V with a 1k/2k divider on ECHO.
 *
 * ---------------------------------------------------------------- SCREENS
 *  HOME    split in two halves, ZONE 1 left, ZONE 2 right: moisture %, band, bar, pump status.
 *  DETAIL  tap a half: moisture, ADC, state, thresholds, drying rate, water loss,
 *          predicted time to next watering, last cycle, pump totals.  BACK button top right.
 *  On power-up it asks for 3 touches (touch calibration), the axes are detected automatically.
 */
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define NZONES 2

/* ================================================================ settings */
#define PUMP_TEST_AT_BOOT 1       /* 1 = run both pumps for 3 s at power-up (hardware check) */
#define SERIALIZE_PUMPS   0       /* 1 = never run both pumps at once (zone 2 waits)   */
#define TS_CAL_AT_BOOT    1       /* 3-touch calibration at power-up                   */
#define TFT_MADCTL        0x28    /* landscape 320x240, BGR. 0xE8 = rotated 180 degrees */

#define MAX_BURSTS        25      /* safety limit per watering cycle, then FAULT       */
#define ESTOP_DEBOUNCE_MS 250U    /* ignore button bounce for this long after a press  */

/* water tank, measured with an HC-SR04 mounted above the water: CALIBRATE FOR YOUR TANK */
#define TANK_FULL_MM      24     /* sensor-to-water distance when the tank is full    */
#define TANK_EMPTY_MM     66     /* sensor-to-water distance when it is empty         */
#define TANK_CAP_ML       850     /* tank volume between those two levels              */
#define TANK_LOW_PCT      10      /* below this: TANK EMPTY, watering is blocked       */
#define TANK_SAMPLE_MS    5000UL  /* the tank is measured every 5 s                    */

  #define SOAK_MS         15000UL /* soak between bursts (demo timing)                */
  #define RATE_SAMPLE_MS  10000UL
  #define SETTLE_MS       30000UL

#define RATE_N            24        /* samples in the fit window (24 x 10 min = 4 h)   */
/* learning: gradient descent, one small step per drying period (see gd_fit) */
#define GD_EPOCHS         25        /* gradient steps per drying period                                       */
#define GD_ETA_GAIN       0.10f     /* learning rate of the rate gain                                         */
#define GD_ETA_OFFSET     0.50f     /* learning rate of the start-up offset                                   */
#define GD_DECAY          0.5f      /* learning rate = 1 / (1 + GD_DECAY x cycles learned): big steps first, then fine tuning */
#define GD_LR_MIN         0.2f      /* ... but never below this fraction of the full rate                    */
#define GD_B0             100.0f    /* the offset is counted in units of this many seconds                    */
#define GD_R0             60.0f     /* relative error is measured against (real time left + this), in s      */
#define GD_MIN_R_S        30.0f     /* examples with less real time left than this are ignored                */
#define GD_MIN_EX         8         /* a period needs at least this many examples to learn from               */
#define EX_N              64        /* examples kept per drying period (thinned automatically when full)      */
#define ML_ETA            0.5f      /* learning rate of mL per percent: each cycle closes this fraction of the gap */
#define DIST_RISE_PCT     1.0f      /* a rise bigger than this between two drying samples (no pump running) = rain / manual watering */
#define SIM_RAIN_PCT      25.0f     /* what one simulated rain adds (serial command '1' or '2', or the dashboard button) */
#define RATE_MIN_SAMPLES  4         /* samples needed before predicting                */
#define MIN_DRY_RATE      0.1f      /* %/hour below this = "not drying yet" (noise)    */
#define MAX_PREDICT_H     72.0f     /* show ">72h" beyond this                         */
#define PREDICT_PRINT_MS  30000UL   /* serial prediction line every 30 s               */
#define CSV_LOG           1         /* 1 = print a CSV data line over the serial port (see log_to_csv.py) */
#define CSV_LOG_MS        1000UL   /* time between CSV lines, per zone                */
#define SERIAL_VERBOSE    1         /* 0 = only the CSV lines (cleaner for logging)    */

#define FILTER_DIV        4         /* moving average strength (higher = smoother)     */

#define BAND_DRY_MAX_PCT  10        /* at or below: DRY (flat reading)                 */
#define BAND_WET_MIN_PCT  90        /* at or above: WET (flat, noisy reading)          */

/* per-zone configuration.  Zone 2 starts as a copy of zone 1: CALIBRATE ZONE 2 (its own
 * sensor, soil and pump) with soil_calibration.c and enter its numbers here. */
typedef struct {
    int      adc0;         /* ADC at 0 % (dry: air or bone-dry soil)     */
    int      adc100;       /* ADC at 100 % (saturated)                   */
    int      low_pct;      /* start watering below this                  */
    int      high_pct;     /* stop watering at this                      */
    int      duty;         /* pump PWM duty, percent                     */
    uint32_t burst_ms;     /* pump on-time per burst                     */
    float    burst_ml;     /* MEASURED volume of one burst               */
    float    ml_per_pct;   /* starting estimate, relearned after cycles  */
    int      settle_ms;    /* wait after powering the sensor             */
} zone_cfg_t;

static const zone_cfg_t zcfg[NZONES] = {
    /* zone 1 */ { 2400, 1400, 40, 80, 50, 2000UL, 25.0f, 1.0f, 20 },
    /* zone 2 */ { 2400, 1400, 40, 80, 50, 2000UL, 25.0f, 1.0f, 20 },
};

/* ================================================================ pins */
#define TFT_CS_PORT   GPIOB
#define TFT_CS_PIN    8
#define TFT_DC_PORT   GPIOB
#define TFT_DC_PIN    9
#define TFT_RST_PORT  GPIOA
#define TFT_RST_PIN   10
#define TS_CS_PORT    GPIOC
#define TS_CS_PIN     7
#define TS_IRQ_PORT   GPIOC
#define TS_IRQ_PIN    0

typedef struct {
    GPIO_TypeDef *pwr_port;  uint8_t pwr_pin;     /* sensor VCC (switched) */
    GPIO_TypeDef *aout_port; uint8_t aout_pin;    /* sensor AOUT           */
    uint8_t       adc_ch;                         /* ADC1 channel of AOUT  */
    GPIO_TypeDef *in1_port;  uint8_t in1_pin;     /* L293D direction       */
    GPIO_TypeDef *in2_port;  uint8_t in2_pin;
    GPIO_TypeDef *en_port;   uint8_t en_pin;      /* L293D enable (PWM)    */
    uint8_t       en_af;                          /* alternate function    */
} zone_hw_t;

static const zone_hw_t hw[NZONES] = {
    { GPIOA, 1, GPIOA, 0, 0, GPIOB, 0, GPIOB, 5, GPIOB, 6,  2 },   /* zone 1: TIM4_CH1 on PB6  */
    { GPIOC, 1, GPIOA, 4, 4, GPIOA, 9, GPIOA, 8, GPIOB, 10, 1 },   /* zone 2: TIM2_CH3 on PB10 */
};

/* ================================================================ colours (RGB565) */
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_GREY    0x8410
#define C_DGREY   0x39E7
#define C_GREEN   0x07E0
#define C_YELLOW  0xFFE0
#define C_RED     0xF800
#define C_CYAN    0x07FF
#define C_ORANGE  0xFD20

#define SCR_W 320
#define SCR_H 240

/* ================================================================ 5x7 font, ASCII 32..126 */
static const uint8_t font5x7[95 * 5] = {
    0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x5F,0x00,0x00, 0x00,0x07,0x00,0x07,0x00, 0x14,0x7F,0x14,0x7F,0x14,
    0x24,0x2A,0x7F,0x2A,0x12, 0x23,0x13,0x08,0x64,0x62, 0x36,0x49,0x56,0x20,0x50, 0x00,0x08,0x07,0x03,0x00,
    0x00,0x1C,0x22,0x41,0x00, 0x00,0x41,0x22,0x1C,0x00, 0x2A,0x1C,0x7F,0x1C,0x2A, 0x08,0x08,0x3E,0x08,0x08,
    0x00,0x80,0x70,0x30,0x00, 0x08,0x08,0x08,0x08,0x08, 0x00,0x00,0x60,0x60,0x00, 0x20,0x10,0x08,0x04,0x02,
    0x3E,0x51,0x49,0x45,0x3E, 0x00,0x42,0x7F,0x40,0x00, 0x72,0x49,0x49,0x49,0x46, 0x21,0x41,0x49,0x4D,0x33,
    0x18,0x14,0x12,0x7F,0x10, 0x27,0x45,0x45,0x45,0x39, 0x3C,0x4A,0x49,0x49,0x31, 0x41,0x21,0x11,0x09,0x07,
    0x36,0x49,0x49,0x49,0x36, 0x46,0x49,0x49,0x29,0x1E, 0x00,0x00,0x14,0x00,0x00, 0x00,0x40,0x34,0x00,0x00,
    0x00,0x08,0x14,0x22,0x41, 0x14,0x14,0x14,0x14,0x14, 0x00,0x41,0x22,0x14,0x08, 0x02,0x01,0x59,0x09,0x06,
    0x3E,0x41,0x5D,0x59,0x4E, 0x7C,0x12,0x11,0x12,0x7C, 0x7F,0x49,0x49,0x49,0x36, 0x3E,0x41,0x41,0x41,0x22,
    0x7F,0x41,0x41,0x41,0x3E, 0x7F,0x49,0x49,0x49,0x41, 0x7F,0x09,0x09,0x09,0x01, 0x3E,0x41,0x41,0x51,0x73,
    0x7F,0x08,0x08,0x08,0x7F, 0x00,0x41,0x7F,0x41,0x00, 0x20,0x40,0x41,0x3F,0x01, 0x7F,0x08,0x14,0x22,0x41,
    0x7F,0x40,0x40,0x40,0x40, 0x7F,0x02,0x1C,0x02,0x7F, 0x7F,0x04,0x08,0x10,0x7F, 0x3E,0x41,0x41,0x41,0x3E,
    0x7F,0x09,0x09,0x09,0x06, 0x3E,0x41,0x51,0x21,0x5E, 0x7F,0x09,0x19,0x29,0x46, 0x26,0x49,0x49,0x49,0x32,
    0x03,0x01,0x7F,0x01,0x03, 0x3F,0x40,0x40,0x40,0x3F, 0x1F,0x20,0x40,0x20,0x1F, 0x3F,0x40,0x38,0x40,0x3F,
    0x63,0x14,0x08,0x14,0x63, 0x03,0x04,0x78,0x04,0x03, 0x61,0x59,0x49,0x4D,0x43, 0x00,0x7F,0x41,0x41,0x41,
    0x02,0x04,0x08,0x10,0x20, 0x41,0x41,0x41,0x7F,0x00, 0x04,0x02,0x01,0x02,0x04, 0x40,0x40,0x40,0x40,0x40,
    0x00,0x03,0x07,0x08,0x00, 0x20,0x54,0x54,0x78,0x40, 0x7F,0x28,0x44,0x44,0x38, 0x38,0x44,0x44,0x44,0x28,
    0x38,0x44,0x44,0x28,0x7F, 0x38,0x54,0x54,0x54,0x18, 0x00,0x08,0x7E,0x09,0x02, 0x18,0xA4,0xA4,0x9C,0x78,
    0x7F,0x08,0x04,0x04,0x78, 0x00,0x44,0x7D,0x40,0x00, 0x20,0x40,0x40,0x3D,0x00, 0x7F,0x10,0x28,0x44,0x00,
    0x00,0x41,0x7F,0x40,0x00, 0x7C,0x04,0x78,0x04,0x78, 0x7C,0x08,0x04,0x04,0x78, 0x38,0x44,0x44,0x44,0x38,
    0xFC,0x18,0x24,0x24,0x18, 0x18,0x24,0x24,0x18,0xFC, 0x7C,0x08,0x04,0x04,0x08, 0x48,0x54,0x54,0x54,0x24,
    0x04,0x04,0x3F,0x44,0x24, 0x3C,0x40,0x40,0x20,0x7C, 0x1C,0x20,0x40,0x20,0x1C, 0x3C,0x40,0x30,0x40,0x3C,
    0x44,0x28,0x10,0x28,0x44, 0x4C,0x90,0x90,0x90,0x7C, 0x44,0x64,0x54,0x4C,0x44, 0x00,0x08,0x36,0x41,0x00,
    0x00,0x00,0x77,0x00,0x00, 0x00,0x41,0x36,0x08,0x00, 0x02,0x01,0x02,0x04,0x02,
};

/* ================================================================ GPIO helpers */
static inline void gpio_mode(GPIO_TypeDef *p, int pin, uint32_t mode)   /* 0 in, 1 out, 2 AF, 3 analog */
{
    p->MODER = (p->MODER & ~(3U << (pin * 2))) | (mode << (pin * 2));
}
static inline void gpio_af(GPIO_TypeDef *p, int pin, uint32_t af)
{
    int r = pin >> 3, s = (pin & 7) * 4;
    p->AFR[r] = (p->AFR[r] & ~(0xFU << s)) | (af << s);
}
static inline void gpio_pullup(GPIO_TypeDef *p, int pin)
{
    p->PUPDR = (p->PUPDR & ~(3U << (pin * 2))) | (1U << (pin * 2));
}
static inline void gpio_fast(GPIO_TypeDef *p, int pin) { p->OSPEEDR |= (3U << (pin * 2)); }
static inline void gpio_high(GPIO_TypeDef *p, int pin) { p->BSRR = (1U << pin); }
static inline void gpio_low(GPIO_TypeDef *p, int pin)  { p->BSRR = (1U << (pin + 16)); }

/* ================================================================ UART */
static void uart2_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;       /* the GPIOA clock must be on BEFORE PA2 is configured */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    gpio_mode(GPIOA, 2, 2);
    gpio_af(GPIOA, 2, 7);
    gpio_mode(GPIOA, 3, 2);                    /* PA3 = RX, so the dashboard can send the 'rain' command */
    gpio_af(GPIOA, 3, 7);
    USART2->BRR = 139;                         /* 16 MHz / 115200 */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void uart2_print(const char *s)
{
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE)) {}
        USART2->DR = (uint8_t)*s++;
    }
}

/* ================================================================ SPI1 + ILI9341 */
#define SPI_BR_TFT    0     /* fPCLK/2  = 8 MHz  */
#define SPI_BR_TOUCH  3     /* fPCLK/16 = 1 MHz  (XPT2046 allows up to 2 MHz) */

static void spi_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    gpio_mode(GPIOA, 5, 2);  gpio_af(GPIOA, 5, 5);  gpio_fast(GPIOA, 5);    /* SCK  */
    gpio_mode(GPIOA, 6, 2);  gpio_af(GPIOA, 6, 5);  gpio_pullup(GPIOA, 6);  /* MISO */
    gpio_mode(GPIOA, 7, 2);  gpio_af(GPIOA, 7, 5);  gpio_fast(GPIOA, 7);    /* MOSI */
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (SPI_BR_TFT << SPI_CR1_BR_Pos);
    SPI1->CR1 |= SPI_CR1_SPE;
}

static void spi_speed(uint32_t br)
{
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 = (SPI1->CR1 & ~SPI_CR1_BR) | (br << SPI_CR1_BR_Pos);
    SPI1->CR1 |= SPI_CR1_SPE;
}

static uint8_t spi_xfer(uint8_t b)
{
    while (!(SPI1->SR & SPI_SR_TXE)) {}
    SPI1->DR = b;
    while (!(SPI1->SR & SPI_SR_RXNE)) {}
    return (uint8_t)SPI1->DR;
}

static void spi_wait_idle(void)
{
    while (SPI1->SR & SPI_SR_BSY) {}
}

static void tft_cmd_args(uint8_t cmd, const uint8_t *args, int n)
{
    gpio_low(TFT_DC_PORT, TFT_DC_PIN);
    gpio_low(TFT_CS_PORT, TFT_CS_PIN);
    spi_xfer(cmd);
    gpio_high(TFT_DC_PORT, TFT_DC_PIN);
    for (int i = 0; i < n; i++) spi_xfer(args[i]);
    spi_wait_idle();
    gpio_high(TFT_CS_PORT, TFT_CS_PIN);
}

static void tft_cmd(uint8_t cmd) { tft_cmd_args(cmd, 0, 0); }

static void tft_window(int x0, int y0, int x1, int y1)
{
    uint8_t a[4];
    a[0] = (uint8_t)(x0 >> 8); a[1] = (uint8_t)x0; a[2] = (uint8_t)(x1 >> 8); a[3] = (uint8_t)x1;
    tft_cmd_args(0x2A, a, 4);
    a[0] = (uint8_t)(y0 >> 8); a[1] = (uint8_t)y0; a[2] = (uint8_t)(y1 >> 8); a[3] = (uint8_t)y1;
    tft_cmd_args(0x2B, a, 4);
    tft_cmd(0x2C);
}

static void tft_stream_begin(void)
{
    gpio_high(TFT_DC_PORT, TFT_DC_PIN);
    gpio_low(TFT_CS_PORT, TFT_CS_PIN);
}

static void tft_stream_end(void)
{
    spi_wait_idle();
    gpio_high(TFT_CS_PORT, TFT_CS_PIN);
}

static void tft_fill(int x, int y, int w, int h, uint16_t col)
{
    if (w <= 0 || h <= 0) return;
    tft_window(x, y, x + w - 1, y + h - 1);
    tft_stream_begin();
    uint8_t hi = (uint8_t)(col >> 8), lo = (uint8_t)col;
    for (uint32_t i = 0; i < (uint32_t)w * (uint32_t)h; i++) { spi_xfer(hi); spi_xfer(lo); }
    tft_stream_end();
}

static void tft_rect(int x, int y, int w, int h, uint16_t col)   /* 1 px outline */
{
    tft_fill(x, y, w, 1, col);
    tft_fill(x, y + h - 1, w, 1, col);
    tft_fill(x, y, 1, h, col);
    tft_fill(x + w - 1, y, 1, h, col);
}

/* one character cell of 6x8 pixels at the given scale, foreground on background */
static void tft_char(int x, int y, char c, int s, uint16_t fg, uint16_t bg)
{
    if (c < 32 || c > 126) c = '?';
    const uint8_t *g = &font5x7[(c - 32) * 5];
    tft_window(x, y, x + 6 * s - 1, y + 8 * s - 1);
    tft_stream_begin();
    for (int row = 0; row < 8; row++) {
        for (int rs = 0; rs < s; rs++) {
            for (int col = 0; col < 6; col++) {
                int on = (col < 5 && row < 7) ? ((g[col] >> row) & 1) : 0;
                uint16_t px = on ? fg : bg;
                uint8_t hi = (uint8_t)(px >> 8), lo = (uint8_t)px;
                for (int cs = 0; cs < s; cs++) { spi_xfer(hi); spi_xfer(lo); }
            }
        }
    }
    tft_stream_end();
}

/* draws t padded with spaces to width_chars, so older, longer text is overwritten */
static void tft_text(int x, int y, const char *t, int s, uint16_t fg, uint16_t bg, int width_chars)
{
    int n = 0;
    while (*t && n < width_chars) { tft_char(x, y, *t++, s, fg, bg); x += 6 * s; n++; }
    while (n < width_chars)       { tft_char(x, y, ' ', s, fg, bg);  x += 6 * s; n++; }
}

static void tft_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    gpio_mode(TFT_CS_PORT, TFT_CS_PIN, 1);   gpio_high(TFT_CS_PORT, TFT_CS_PIN);
    gpio_mode(TFT_DC_PORT, TFT_DC_PIN, 1);
    gpio_mode(TFT_RST_PORT, TFT_RST_PIN, 1);
    gpio_mode(TS_CS_PORT, TS_CS_PIN, 1);     gpio_high(TS_CS_PORT, TS_CS_PIN);
    gpio_mode(TS_IRQ_PORT, TS_IRQ_PIN, 0);   gpio_pullup(TS_IRQ_PORT, TS_IRQ_PIN);

    gpio_high(TFT_RST_PORT, TFT_RST_PIN); HAL_Delay(10);
    gpio_low(TFT_RST_PORT, TFT_RST_PIN);  HAL_Delay(20);
    gpio_high(TFT_RST_PORT, TFT_RST_PIN); HAL_Delay(150);

    static const uint8_t i_ef[] = {0x03, 0x80, 0x02};
    static const uint8_t i_cf[] = {0x00, 0xC1, 0x30};
    static const uint8_t i_ed[] = {0x64, 0x03, 0x12, 0x81};
    static const uint8_t i_e8[] = {0x85, 0x00, 0x78};
    static const uint8_t i_cb[] = {0x39, 0x2C, 0x00, 0x34, 0x02};
    static const uint8_t i_f7[] = {0x20};
    static const uint8_t i_ea[] = {0x00, 0x00};
    static const uint8_t i_c0[] = {0x23};
    static const uint8_t i_c1[] = {0x10};
    static const uint8_t i_c5[] = {0x3E, 0x28};
    static const uint8_t i_c7[] = {0x86};
    static const uint8_t i_36[] = {TFT_MADCTL};
    static const uint8_t i_3a[] = {0x55};
    static const uint8_t i_b1[] = {0x00, 0x18};
    static const uint8_t i_b6[] = {0x08, 0x82, 0x27};
    static const uint8_t i_f2[] = {0x00};
    static const uint8_t i_26[] = {0x01};
    static const uint8_t i_e0[] = {0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,0x37,0x07,0x10,0x03,0x0E,0x09,0x00};
    static const uint8_t i_e1[] = {0x00,0x0E,0x14,0x03,0x11,0x07,0x31,0xC1,0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F};

    tft_cmd_args(0xEF, i_ef, 3);  tft_cmd_args(0xCF, i_cf, 3);  tft_cmd_args(0xED, i_ed, 4);
    tft_cmd_args(0xE8, i_e8, 3);  tft_cmd_args(0xCB, i_cb, 5);  tft_cmd_args(0xF7, i_f7, 1);
    tft_cmd_args(0xEA, i_ea, 2);  tft_cmd_args(0xC0, i_c0, 1);  tft_cmd_args(0xC1, i_c1, 1);
    tft_cmd_args(0xC5, i_c5, 2);  tft_cmd_args(0xC7, i_c7, 1);  tft_cmd_args(0x36, i_36, 1);
    tft_cmd_args(0x3A, i_3a, 1);  tft_cmd_args(0xB1, i_b1, 2);  tft_cmd_args(0xB6, i_b6, 3);
    tft_cmd_args(0xF2, i_f2, 1);  tft_cmd_args(0x26, i_26, 1);
    tft_cmd_args(0xE0, i_e0, 15); tft_cmd_args(0xE1, i_e1, 15);
    tft_cmd(0x11);  HAL_Delay(150);
    tft_cmd(0x29);  HAL_Delay(20);
    tft_fill(0, 0, SCR_W, SCR_H, C_BLACK);
}

/* ================================================================ XPT2046 touch */
static int ts_swap = 0;
static int cal_ax_a, cal_ax_b, cal_ay_b, cal_ay_c;     /* raw values at the calibration points */
#define CAL_A_X 20
#define CAL_A_Y 20
#define CAL_B_X 300
#define CAL_B_Y 20
#define CAL_C_X 300
#define CAL_C_Y 220

static int ts_pressed(void)
{
    return !(TS_IRQ_PORT->IDR & (1U << TS_IRQ_PIN));
}

static uint16_t ts_xfer12(uint8_t cmd)
{
    spi_xfer(cmd);
    uint8_t h = spi_xfer(0);
    uint8_t l = spi_xfer(0);
    return (uint16_t)((((h << 8) | l) >> 3) & 0x0FFF);
}

/* returns 1 and the averaged raw position while the screen is touched */
static int ts_read_raw(int *rx, int *ry)
{
    if (!ts_pressed()) return 0;
    spi_speed(SPI_BR_TOUCH);
    gpio_low(TS_CS_PORT, TS_CS_PIN);
    (void)ts_xfer12(0xD0);                                   /* throw away the first conversion */
    int sx = 0, sy = 0, xmin = 4095, xmax = 0, ymin = 4095, ymax = 0;
    for (int i = 0; i < 4; i++) {
        int x = ts_xfer12(0xD0), y = ts_xfer12(0x90);
        sx += x; sy += y;
        if (x < xmin) xmin = x;
        if (x > xmax) xmax = x;
        if (y < ymin) ymin = y;
        if (y > ymax) ymax = y;
    }
    (void)ts_xfer12(0x00);                                   /* power the controller down      */
    gpio_high(TS_CS_PORT, TS_CS_PIN);
    spi_speed(SPI_BR_TFT);
    if (!ts_pressed()) return 0;
    if ((xmax - xmin) > 150 || (ymax - ymin) > 150) return 0;   /* noisy reading, ignore */
    *rx = sx / 4;  *ry = sy / 4;
    return 1;
}

static void ts_map(int rx, int ry, int *sx, int *sy)
{
    int ax = ts_swap ? ry : rx;
    int ay = ts_swap ? rx : ry;
    int dx = cal_ax_b - cal_ax_a;
    int dy = cal_ay_c - cal_ay_b;
    if (dx == 0) dx = 1;
    if (dy == 0) dy = 1;
    int x = CAL_A_X + (int)(((int32_t)(ax - cal_ax_a) * (CAL_B_X - CAL_A_X)) / dx);
    int y = CAL_B_Y + (int)(((int32_t)(ay - cal_ay_b) * (CAL_C_Y - CAL_B_Y)) / dy);
    if (x < 0) x = 0;
    if (x > SCR_W - 1) x = SCR_W - 1;
    if (y < 0) y = 0;
    if (y > SCR_H - 1) y = SCR_H - 1;
    *sx = x;  *sy = y;
}

static void ts_wait_touch(int *rx, int *ry)
{
    int stable = 0;
    while (stable < 5) {
        int x, y;
        if (ts_read_raw(&x, &y)) { *rx = x; *ry = y; stable++; }
        else stable = 0;
        HAL_Delay(10);
    }
}

static void ts_wait_release(void)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 150U) {
        if (ts_pressed()) t0 = HAL_GetTick();
        HAL_Delay(5);
    }
}

static void ts_cross(int x, int y, uint16_t col)
{
    tft_fill(x - 12, y, 25, 1, col);
    tft_fill(x, y - 12, 1, 25, col);
    tft_rect(x - 4, y - 4, 9, 9, col);
}

static void ts_calibrate(void)
{
    int rxa, rya, rxb, ryb, rxc, ryc;
    char b[96];
    tft_fill(0, 0, SCR_W, SCR_H, C_BLACK);
    tft_text(40, 100, "TOUCH CALIBRATION", 2, C_WHITE, C_BLACK, 17);
    tft_text(40, 130, "Touch each cross firmly", 1, C_GREY, C_BLACK, 24);

    ts_cross(CAL_A_X, CAL_A_Y, C_YELLOW);  ts_wait_touch(&rxa, &rya);  ts_wait_release();
    ts_cross(CAL_A_X, CAL_A_Y, C_BLACK);
    ts_cross(CAL_B_X, CAL_B_Y, C_YELLOW);  ts_wait_touch(&rxb, &ryb);  ts_wait_release();
    ts_cross(CAL_B_X, CAL_B_Y, C_BLACK);
    ts_cross(CAL_C_X, CAL_C_Y, C_YELLOW);  ts_wait_touch(&rxc, &ryc);  ts_wait_release();

    /* between A and B only the screen X changes: the raw axis that moved most is screen X */
    int dxr = rxb > rxa ? rxb - rxa : rxa - rxb;
    int dyr = ryb > rya ? ryb - rya : rya - ryb;
    ts_swap = (dyr > dxr) ? 1 : 0;
    cal_ax_a = ts_swap ? rya : rxa;
    cal_ax_b = ts_swap ? ryb : rxb;
    cal_ay_b = ts_swap ? rxb : ryb;
    cal_ay_c = ts_swap ? rxc : ryc;

    sprintf(b, "TOUCH CAL: swap=%d ax_a=%d ax_b=%d ay_b=%d ay_c=%d\r\n",
            ts_swap, cal_ax_a, cal_ax_b, cal_ay_b, cal_ay_c);
    uart2_print(b);
    tft_fill(0, 0, SCR_W, SCR_H, C_BLACK);
}

/* ================================================================ soil simulator + sensors */
static uint8_t  sim_pump[NZONES];                      /* pump running flags, used by the simulators */
static uint32_t sim_seed = 12345;


/* ---------------------------------------------------------------- simulated soil
 * Each zone has a hidden "true" moisture that dries and rises while the pump is on.  sim_read() converts it to
 * the ADC value the real sensor would give, with a little noise, so the rest of the firmware cannot tell the
 * difference.
 *
 * It copies what the real soil log showed: the soil dries at an almost constant speed (a straight line), but
 * slowly at first right after watering and a little faster later, and the speed wanders over time (room
 * temperature, air flow).  So the first prediction is systematically a bit off and has to be corrected. */
#define SIM_JITTER         0.004f   /* size of the random drift of the drying speed (0 = perfectly steady) */
#define SIM_START_FRAC     0.45f    /* speed right after watering, as a fraction of the normal speed       */
#define SIM_RAMP_S         150.0f   /* time to reach the normal speed                                      */
#define SIM_ACCEL          0.12f    /* the speed then rises by this much more ...                          */
#define SIM_ACCEL_S        480.0f   /* ... over this time (the room warms up)                              */
static const float SIM_RATE_PCT_S[NZONES] = { 0.085f, 0.075f };   /* normal drying speed, % per second (a real soil is ~0.0001) */
static const float SIM_ML_PER_PCT[NZONES] = { 3.0f,   3.6f };     /* "true" soil: mL needed for a 1 % rise
                                                                   * (set to 100 to force a FAULT)          */
static float    sim_pct[NZONES] = { 30.0f, 60.0f };    /* zone 1 starts dry, zone 2 moist */
static float    sim_speed[NZONES] = { 1.0f, 1.0f };    /* drying speed factor, drifts around 1.0 */
static float    sim_age[NZONES];                        /* seconds since the pump last ran */
static float    sim_soil[NZONES]  = { 1.0f, 1.0f };    /* how thirsty the soil is, drifts: more water per % when > 1 */
static uint32_t sim_t[NZONES];

static void sim_rain(int z)
{
    sim_pct[z] += SIM_RAIN_PCT;
    if (sim_pct[z] > 100.0f) sim_pct[z] = 100.0f;
}

static uint16_t sim_read(int z)
{
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - sim_t[z]) / 1000.0f;
    sim_t[z] = now;
    if (dt > 2.0f) dt = 2.0f;

    /* the drying speed drifts slowly around 1.0 (random walk pulled back to 1.0) */
    sim_seed = sim_seed * 1664525UL + 1013904223UL;
    float r = (float)((int)((sim_seed >> 16) & 0xFFFFU) - 32768) / 32768.0f;          /* -1 .. +1 */
    sim_speed[z] += (1.0f - sim_speed[z]) * 0.005f * dt + r * SIM_JITTER * dt;
    if (sim_speed[z] < 0.7f) sim_speed[z] = 0.7f;
    if (sim_speed[z] > 1.4f) sim_speed[z] = 1.4f;

    sim_soil[z] += (1.0f - sim_soil[z]) * 0.0005f * dt + r * 0.004f * dt;      /* much slower than the drying speed */
    if (sim_soil[z] < 0.8f)  sim_soil[z] = 0.8f;
    if (sim_soil[z] > 1.25f) sim_soil[z] = 1.25f;

    float ramp  = sim_age[z] / SIM_RAMP_S;   if (ramp  > 1.0f) ramp  = 1.0f;
    float accel = sim_age[z] / SIM_ACCEL_S;  if (accel > 1.0f) accel = 1.0f;
    float shape = (SIM_START_FRAC + (1.0f - SIM_START_FRAC) * ramp) * (1.0f + SIM_ACCEL * accel);
    sim_pct[z] -= SIM_RATE_PCT_S[z] * sim_speed[z] * shape * dt;               /* a straight line, not a curve */
    sim_age[z] = sim_pump[z] ? 0.0f : sim_age[z] + dt;
    if (sim_pump[z]) {
        float flow = (zcfg[z].burst_ml / (float)zcfg[z].burst_ms) * 1000.0f;   /* mL per second */
        sim_pct[z] += flow / (SIM_ML_PER_PCT[z] * sim_soil[z]) * dt;
    }
    if (sim_pct[z] < 0.0f)   sim_pct[z] = 0.0f;
    if (sim_pct[z] > 100.0f) sim_pct[z] = 100.0f;

    sim_seed = sim_seed * 1664525UL + 1013904223UL;
    int noise = (int)((sim_seed >> 24) % 7) - 3;                 /* +-3 counts */
    float span = (float)(zcfg[z].adc0 - zcfg[z].adc100);
    int adc = zcfg[z].adc0 - (int)(sim_pct[z] / 100.0f * span) + noise;
    if (adc < 0)    adc = 0;
    if (adc > 4095) adc = 4095;
    return (uint16_t)adc;
}

/* demo button: drop a zone to 10 % below its watering threshold so it starts watering */
static void sim_make_dry(int z)
{
    sim_pct[z] = (float)zcfg[z].low_pct - 10.0f;
}

/* demo: the "sensor" is the soil simulator (the real program powers the sensor and averages ADC readings here) */
static uint16_t sensor_read(int z)
{
    return sim_read(z);
}

/* ================================================================ pumps (L293D) */
/* force stop state, written by the button interrupt, read by the main loop */
static volatile uint8_t  estop_active = 0;     /* 1 = force stop engaged                     */
static volatile uint8_t  estop_hit    = 0;     /* set when it was engaged, main loop aborts the running cycles */
static volatile uint32_t estop_tick   = 0;     /* time of the last accepted button press     */

static void pwm_set(int z, uint32_t duty_permille)
{
    if (z == 0) TIM4->CCR1 = duty_permille;
    else        TIM2->CCR3 = duty_permille;
}

static void pump_off(int z)
{
    pwm_set(z, 0);
    gpio_low(hw[z].in1_port, hw[z].in1_pin);
    gpio_low(hw[z].in2_port, hw[z].in2_pin);
    sim_pump[z] = 0;
}

static void pump_on(int z)
{
    if (estop_active) return;                               /* never start a pump during a force stop */
    gpio_high(hw[z].in1_port, hw[z].in1_pin);
    gpio_low(hw[z].in2_port, hw[z].in2_pin);
    pwm_set(z, (uint32_t)zcfg[z].duty * 10U);
    sim_pump[z] = 1;
    if (estop_active) pump_off(z);                          /* button pressed while switching on */
}

/* ---------------------------------------------------------------- force stop button (B1 = PC13)
 * Falling edge on PC13 raises the EXTI15_10 interrupt.  The ISR toggles the force stop and, when it
 * engages, switches both pumps off immediately.  The screen and the state machines are handled by
 * the main loop (estop_service).
 * If the linker reports "multiple definition of EXTI15_10_IRQHandler", delete that handler from
 * Core/Src/stm32f4xx_it.c.                                                                      */
static void estop_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    gpio_mode(GPIOC, 13, 0);                                /* input                              */
    gpio_pullup(GPIOC, 13);                                 /* button pulls the pin low           */
    SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] & ~(0xFU << 4)) | (2U << 4);   /* EXTI13 <- port C    */
    EXTI->PR    = (1U << 13);
    EXTI->RTSR &= ~(1U << 13);
    EXTI->FTSR |= (1U << 13);                               /* falling edge = button pressed      */
    EXTI->IMR  |= (1U << 13);
    NVIC_SetPriority(EXTI15_10_IRQn, 1);                    /* above SysTick, so it always gets in */
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1U << 13)) {
        EXTI->PR = (1U << 13);                              /* clear the pending flag (write 1)   */
        uint32_t now = HAL_GetTick();
        if ((now - estop_tick) < ESTOP_DEBOUNCE_MS) return; /* contact bounce                     */
        estop_tick = now;
        estop_active = !estop_active;
        if (estop_active) {
            for (int z = 0; z < NZONES; z++) pump_off(z);   /* stop the pumps right now           */
            estop_hit = 1;
        }
    }
}

static void zones_hw_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM2EN;

    for (int z = 0; z < NZONES; z++) {
        gpio_mode(hw[z].pwr_port, hw[z].pwr_pin, 1);   gpio_low(hw[z].pwr_port, hw[z].pwr_pin);
        gpio_mode(hw[z].aout_port, hw[z].aout_pin, 3);
        gpio_mode(hw[z].in1_port, hw[z].in1_pin, 1);
        gpio_mode(hw[z].in2_port, hw[z].in2_pin, 1);
        gpio_mode(hw[z].en_port, hw[z].en_pin, 2);
        gpio_af(hw[z].en_port, hw[z].en_pin, hw[z].en_af);
    }

    TIM4->PSC = 15;  TIM4->ARR = 999;  TIM4->CCR1 = 0;               /* 1 kHz PWM, zone 1 */
    TIM4->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM4->CCER = TIM_CCER_CC1E;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->CR1 = TIM_CR1_CEN;

    TIM2->PSC = 15;  TIM2->ARR = 999;  TIM2->CCR3 = 0;               /* 1 kHz PWM, zone 2 */
    TIM2->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM2->CCER = TIM_CCER_CC3E;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_CEN;

    for (int z = 0; z < NZONES; z++) pump_off(z);
}

/* ================================================================ water tank level (HC-SR04)
 * TRIG (PB3) gets a 10 us pulse; the sensor answers with an ECHO pulse whose width is the sound's
 * round trip.  TIM3 runs at 1 MHz and captures both edges of the echo in hardware: CH1 on the rising
 * edge, CH2 on the falling edge of the same pin (PB4), so the width is CCR2 - CCR1 in microseconds. */
static int tank_ok = 0;          /* 1 = a valid reading exists, 0 = level unknown (sensor missing)  */
static int tank_mm = 0;          /* filtered sensor-to-water distance                               */
static int tank_pct = 0;
static int tank_ml = 0;
static int tank_empty = 0;       /* 1 = tank empty: no watering                                     */

static void tank_hw_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    gpio_mode(GPIOB, 3, 1);  gpio_low(GPIOB, 3);                           /* TRIG                      */
    gpio_mode(GPIOB, 4, 2);  gpio_af(GPIOB, 4, 2);                         /* ECHO = TIM3_CH1           */
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(3U << 8)) | (2U << 8);                /* pull-down: no sensor = low */

    TIM3->PSC = 15;                                                        /* 16 MHz / 16 = 1 MHz       */
    TIM3->ARR = 0xFFFF;
    TIM3->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos) | (2U << TIM_CCMR1_CC2S_Pos); /* CC1 and CC2 both on TI1   */
    TIM3->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;           /* CC1 rising, CC2 falling   */
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_CEN;
}

/* one measurement: distance in mm, 0 = no echo */
static int hcsr04_read_mm(void)
{
    TIM3->SR = 0;                                                          /* clear the capture flags   */
    gpio_high(GPIOB, 3);
    uint16_t t0 = (uint16_t)TIM3->CNT;
    while ((uint16_t)((uint16_t)TIM3->CNT - t0) < 12U) {}                  /* 10 us trigger pulse       */
    gpio_low(GPIOB, 3);

    uint32_t t = HAL_GetTick();
    while (!(TIM3->SR & TIM_SR_CC2IF)) {                                   /* falling edge = echo over  */
        if ((HAL_GetTick() - t) > 50U) return 0;                           /* nothing came back         */
    }
    if (!(TIM3->SR & TIM_SR_CC1IF)) return 0;
    uint16_t rise = (uint16_t)TIM3->CCR1;
    uint16_t fall = (uint16_t)TIM3->CCR2;
    uint32_t us = (uint16_t)(fall - rise);
    int mm = (int)(us * 343U / 2000U);                                     /* 343 m/s, there and back   */
    if (mm < 20 || mm > 4000) return 0;                                    /* outside the sensor range  */
    return mm;
}

static int tank_read_mm(void)
{
    int v[5], n = 0;
    for (int i = 0; i < 5; i++) {
        int mm = hcsr04_read_mm();
        if (mm > 0) v[n++] = mm;
        HAL_Delay(30);                                                     /* let the old echo die out  */
    }
    if (n < 3) return 0;                                                   /* too few good readings     */
    for (int i = 1; i < n; i++) {                                          /* insertion sort            */
        int k = v[i], j = i - 1;
        while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }
        v[j + 1] = k;
    }
    return v[n / 2];                                                       /* median: ignores stray echoes */
}

/* measures the tank, updates level and the empty flag (with hysteresis) */
static void tank_update(void)
{
    char buf[100];
    int mm = tank_read_mm();

    if (mm <= 0) {                           /* no valid reading: level unknown, watering is NOT blocked */
        if (tank_ok) uart2_print("TANK: no valid echo, level unknown\r\n");
        tank_ok = 0;
        tank_empty = 0;
        return;
    }
    int d = mm - tank_mm;
    if (d < 0) d = -d;
    if (!tank_ok || d > 30) tank_mm = mm;                                  /* first reading or refill: jump */
    else                    tank_mm = (3 * tank_mm + mm) / 4;              /* otherwise smooth          */
    tank_ok = 1;

    int p = (TANK_EMPTY_MM - tank_mm) * 100 / (TANK_EMPTY_MM - TANK_FULL_MM);
    if (p < 0)   p = 0;
    if (p > 100) p = 100;
    tank_pct = p;
    tank_ml = (int)((long)TANK_CAP_ML * p / 100);

    if (!tank_empty && p < TANK_LOW_PCT) {
        tank_empty = 1;
        uart2_print("TANK EMPTY: watering blocked until it is refilled\r\n");
    } else if (tank_empty && p >= TANK_LOW_PCT + 5) {
        tank_empty = 0;
        uart2_print("TANK REFILLED: watering allowed again\r\n");
    }

    sprintf(buf, "TANK: %d %% (~%d mL), distance %d mm\r\n", tank_pct, tank_ml, tank_mm);
    uart2_print(buf);
}

/* ================================================================ zone controller */
typedef enum { Z_IDLE = 0, Z_PUMPING, Z_SOAKING, Z_FAULT } zstate_t;
typedef enum { P_COLLECT = 0, P_SATURATED, P_NOT_DRYING, P_OK } pkind_t;

typedef struct {
    float    ml_per_pct;                 /* learned after each cycle */
    zstate_t st;
    uint32_t t_state, t_idle_start, t_rate_last, t_predict;
    uint32_t pump_total_ms, cycle_pump_ms;
    int      bursts;
    int32_t  filt16;                     /* filtered ADC x16, -1 = not started */
    int      learn_pending;
    int32_t  cycle_pct10_start;
    float    rx[RATE_N], ry[RATE_N];
    int      rn;
    float    ga, gb;                     /* learned by gradient descent: gain on the fitted time (starts 1.0), start-up offset (starts 0) */
    int      gd_n;                       /* drying periods learned from                                       */
    float    gd_loss;                    /* RMS error of the last period's predictions, % (before its update) */
    float    ml_err;                     /* plan error of mL per percent in the last cycle, % (signed)        */
    int      new_sample;                 /* a drying sample was just stored: record a training example        */
    int      ne, ex_stride, ex_skip;     /* examples of this drying period                                    */
    float    ex_h[EX_N], ex_w[EX_N];     /* raw prediction (s) and start-up weight of each example            */
    uint32_t ex_t[EX_N];                 /* ... and when it was made (ms)                                     */
    int      disturbed;                  /* 1 = this drying period was interrupted (rain, manual watering) */
    int      ml_n;                       /* cycles the mL-per-percent has learned from */
    /* latest measurement */
    uint16_t adc;
    int32_t  adc_f, pct10, pct;
    /* prediction results, for the screens */
    pkind_t  pkind;
    int      rate_x100;                  /* drying rate, 0.01 %/h   */
    int      model, tau_min, r2_x100;    /* fit: 0 none, 1 exp, 2 line; time constant; R^2 x100 */
    int      mlh_x10;                    /* water loss, 0.1 mL/h    */
    int      next_min;                   /* minutes to next watering */
    int      next_s;                     /* the same in seconds (finer, for the dashboard) */
    int      next_capped;
    int      last_bursts, last_ml;       /* last finished cycle     */
} zone_t;

static zone_t Z[NZONES];

static const char *band_name(int32_t pct)
{
    if (pct <= BAND_DRY_MAX_PCT) return "DRY";
    if (pct >= BAND_WET_MIN_PCT) return "WET";
    return "MOIST";
}

/* "<=" on the dry plateau and ">=" on the wet plateau, where the reading is only a limit */
static const char *limit_tag(int32_t pct)
{
    if (pct <= BAND_DRY_MAX_PCT) return "<=";
    if (pct >= BAND_WET_MIN_PCT) return ">=";
    return "";
}

static const char *state_name(zstate_t s)
{
    static const char *n[] = { "IDLE", "PUMPING", "SOAKING", "FAULT" };
    return n[s];
}

/* moisture in tenths of a percent: linear between the zone's two anchors, 0 % and 100 % */
static int32_t adc_to_pct10(int z, int32_t adc)
{
    int32_t span = (int32_t)zcfg[z].adc0 - (int32_t)zcfg[z].adc100;
    if (span == 0) span = 1;
    int32_t p = 1000 * ((int32_t)zcfg[z].adc0 - adc) / span;
    if (p < 0)    p = 0;
    if (p > 1000) p = 1000;
    return p;
}

static float ml_per_ms(int z) { return zcfg[z].burst_ml / (float)zcfg[z].burst_ms; }

static int bursts_for(int z, float pct_needed)
{
    if (pct_needed <= 0.0f) return 0;
    float ml = pct_needed * Z[z].ml_per_pct;
    int b = (int)(ml / zcfg[z].burst_ml);
    if ((float)b * zcfg[z].burst_ml < ml) b++;
    return b;
}

static void rate_clear(int z)
{
    Z[z].rn = 0;
#if CSV_LOG
    char b[24];
    sprintf(b, "SAMP_CLEAR,%d\r\n", z + 1);                /* the dashboard empties its sample list too */
    uart2_print(b);
#endif
}

static void rate_add(int z, float minutes, float pct)
{
    zone_t *q = &Z[z];
    if (q->rn == RATE_N) {
        memmove(&q->rx[0], &q->rx[1], (RATE_N - 1) * sizeof(float));
        memmove(&q->ry[0], &q->ry[1], (RATE_N - 1) * sizeof(float));
        q->rn--;
    }
    q->rx[q->rn] = minutes;  q->ry[q->rn] = pct;  q->rn++;
    q->new_sample = 1;

#if CSV_LOG
    {   /* SAMP,zone,ms,moisture_x10 : the exact sample the prediction just stored (the dashboard replots the fit) */
        char b[48];
        sprintf(b, "SAMP,%d,%lu,%d\r\n", z + 1, (unsigned long)HAL_GetTick(), (int)(pct * 10.0f + 0.5f));
        uart2_print(b);
    }
#endif
}

/* ---- drying prediction ----------------------------------------------------------------------
 * The last RATE_N samples are fitted with a straight line (least squares).  Its slope is the drying rate in
 * %/hour, and the time to the watering threshold is (moisture now - low_pct) / rate.  R^2 tells how well the
 * samples follow the line (1.00 = perfect).  The learned gain and offset (gradient descent) then correct it.   */
/* PREDICT-BEGIN */
static int linreg(const float *x, const float *y, int n, float *slope, float *icpt, float *r2)
{
    float mx = 0, my = 0;
    for (int i = 0; i < n; i++) { mx += x[i]; my += y[i]; }
    mx /= (float)n;  my /= (float)n;
    float sxx = 0, sxy = 0, syy = 0;
    for (int i = 0; i < n; i++) {
        sxx += (x[i] - mx) * (x[i] - mx);
        sxy += (x[i] - mx) * (y[i] - my);
        syy += (y[i] - my) * (y[i] - my);
    }
    if (sxx < 1e-3f) return 0;
    *slope = sxy / sxx;
    *icpt  = my - *slope * mx;
    *r2    = (syy > 1e-9f) ? (sxy * sxy) / (sxx * syy) : 1.0f;
    return 1;
}

/* model: 0 = no prediction, 1 = straight line */
static void predict_from_samples(const float *rx, const float *ry, int n, float now_min, float low_pct, float min_rate,
                                 int *model, float *rate_pct_h, float *hours, float *r2out)
{
    float xs[RATE_N], b, a, r2;
    *model = 0; *rate_pct_h = 0; *hours = 0; *r2out = 0;
    for (int i = 0; i < n; i++) xs[i] = rx[i] - rx[0];
    if (!linreg(xs, ry, n, &b, &a, &r2) || b >= 0.0f) return;
    float rate = -b * 60.0f;                                     /* %/hour */
    if (rate < min_rate) return;                                 /* not drying measurably */
    float now_fit = a + b * (now_min - rx[0]);                   /* the line's value now (less noisy than one reading) */
    *model = 1; *rate_pct_h = rate; *r2out = r2;
    *hours = (now_fit <= low_pct) ? 0.0f : (now_fit - low_pct) / rate;
}

/* ---- learning by gradient descent ------------------------------------------------------------
 * Prediction used by the controller:  T = h * (1 + (a-1) * w) + b * B0 * w      (h = the raw line-fit time)
 *   w = 1 - samples/RATE_N : how much of the window is still the unreliable start after watering (1 -> 0)
 *   a = gain, b = start-up offset: the two numbers that are learned, they start at a = 1, b = 0.
 * After every drying period the real time left R is known for every prediction that was made.
 * Loss  L = mean( ((T - R) / (R + R0))^2 )   (relative error, so early and late predictions count alike)
 * Gradient steps:  a -= eta_a * dL/da,  b -= eta_b * dL/db,  repeated GD_EPOCHS times.   */
static float gd_loss(const float *h, const float *w, const float *R, int n, float a, float b)
{
    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        float T = h[i] * (1.0f + (a - 1.0f) * w[i]) + b * GD_B0 * w[i];
        float u = (T - R[i]) / (R[i] + GD_R0);
        sum += u * u;
    }
    return sqrtf(sum / (float)n);                                        /* RMS relative error (0.10 = 10 %) */
}

static void gd_fit(const float *h, const float *w, const float *R, int n, float *a, float *b, float lr)
{
    float pa = *a, pb = *b;
    for (int e = 0; e < GD_EPOCHS; e++) {
        float ga = 0.0f, gb = 0.0f;
        for (int i = 0; i < n; i++) {
            float T = h[i] * (1.0f + (pa - 1.0f) * w[i]) + pb * GD_B0 * w[i];
            float d = R[i] + GD_R0;
            float u = (T - R[i]) / d;
            ga += 2.0f * u * (h[i] * w[i]) / d;
            gb += 2.0f * u * (GD_B0 * w[i]) / d;
        }
        pa -= lr * GD_ETA_GAIN   * ga / (float)n;
        pb -= lr * GD_ETA_OFFSET * gb / (float)n;
        if (pa < 0.4f) pa = 0.4f;   if (pa > 2.5f) pa = 2.5f;
        if (pb < -3.0f) pb = -3.0f; if (pb > 3.0f) pb = 3.0f;
    }
    *a = pa;  *b = pb;
}
/* PREDICT-END */

/* called when watering is about to start: the drying period that just ended gives one training example per
 * prediction (the real time left is now known).  One gradient-descent run improves gain and offset. */
static void gd_learn(int z)
{
    zone_t *q = &Z[z];
    char buf[200];
    float h[EX_N], w[EX_N], R[EX_N];
    int n = 0;

    if (q->disturbed) {                                                   /* rain or manual watering spoilt the period */
        q->disturbed = 0;  q->ne = 0;
        sprintf(buf, "Z%d NOT LEARNED: this drying period was disturbed\r\n", z + 1);
        uart2_print(buf);
        return;
    }
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < q->ne; i++) {
        float r = (float)(now - q->ex_t[i]) / 1000.0f;
        if (r < GD_MIN_R_S) continue;
        h[n] = q->ex_h[i];  w[n] = q->ex_w[i];  R[n] = r;  n++;
    }
    q->ne = 0;  q->ex_stride = 1;  q->ex_skip = 0;
    if (n < GD_MIN_EX) return;                                            /* too little to learn from */

    float before = gd_loss(h, w, R, n, q->ga, q->gb);                     /* how the predictions really performed */
    float lr = 1.0f / (1.0f + GD_DECAY * (float)q->gd_n);                /* learning rate decays: big steps first, then fine tuning */
    if (lr < GD_LR_MIN) lr = GD_LR_MIN;
    gd_fit(h, w, R, n, &q->ga, &q->gb, lr);
    float after  = gd_loss(h, w, R, n, q->ga, q->gb);
    q->gd_loss = before * 100.0f;
    q->gd_n++;
    sprintf(buf, "Z%d LEARNED cycle %d: prediction error %d%% -> %d%% on this period, gain x%d.%02d, offset %d s (%d examples)\r\n",
            z + 1, q->gd_n, (int)(before * 100.0f + 0.5f), (int)(after * 100.0f + 0.5f),
            (int)q->ga, (int)(q->ga * 100.0f) % 100, (int)(q->gb * GD_B0), n);
    uart2_print(buf);
}

static void zones_init(void)
{
    for (int z = 0; z < NZONES; z++) {
        memset(&Z[z], 0, sizeof(zone_t));
        Z[z].ml_per_pct = zcfg[z].ml_per_pct;
        Z[z].ga = 1.0f;
        Z[z].ex_stride = 1;
        Z[z].filt16 = -1;
        Z[z].st = Z_IDLE;
    }
}

static int other_pumping(int z)
{
#if SERIALIZE_PUMPS
    for (int o = 0; o < NZONES; o++)
        if (o != z && Z[o].st == Z_PUMPING) return 1;
#else
    (void)z;
#endif
    return 0;
}

/* reads the sensor and updates the filtered moisture */
static void zone_sample(int z)
{
    zone_t *q = &Z[z];
    q->adc = sensor_read(z);
    if (q->filt16 < 0) q->filt16 = (int32_t)q->adc * 16;
    else               q->filt16 += ((int32_t)q->adc * 16 - q->filt16) / FILTER_DIV;
    q->adc_f = q->filt16 / 16;
    q->pct10 = adc_to_pct10(z, q->adc_f);
    q->pct   = (q->pct10 + 5) / 10;
}

/* the watering state machine, called once a second */
static void zone_step(int z)
{
    zone_t *q = &Z[z];
    const zone_cfg_t *c = &zcfg[z];
    char buf[200];
    uint32_t now = HAL_GetTick();
    int to_idle = 0;

    if (tank_empty && (q->st == Z_PUMPING || q->st == Z_SOAKING)) {   /* never run the pump dry */
        if (q->st == Z_PUMPING) {
            pump_off(z);
            q->pump_total_ms += now - q->t_state;
            q->cycle_pump_ms += now - q->t_state;
        }
        sprintf(buf, "Z%d STOPPED: tank empty, watering cannot continue\r\n", z + 1);
        uart2_print(buf);
        to_idle = 1;
    }

    if (!to_idle) switch (q->st) {
    case Z_IDLE:
        if (q->pct < c->low_pct) {
            if (tank_empty || other_pumping(z)) break;                               /* wait for the other pump */
            float need_pct = (float)(c->high_pct * 10 - q->pct10) / 10.0f;
            int   nb = bursts_for(z, need_pct);
            int   ml = (int)(need_pct * q->ml_per_pct + 0.5f);
            sprintf(buf, "Z%d WATERING PLAN: now %ld %%, target %d %%, need ~%d mL = ~%d bursts of %d mL (%lu s each)%s\r\n",
                    z + 1, (long)q->pct, c->high_pct, ml, nb, (int)(c->burst_ml + 0.5f),
                    (unsigned long)(c->burst_ms / 1000UL),
                    (nb > MAX_BURSTS) ? "  WARNING: exceeds MAX_BURSTS" : "");
            uart2_print(buf);
            q->bursts = 0;
            q->cycle_pump_ms = 0;
            q->cycle_pct10_start = q->pct10;
            pump_on(z);
            q->t_state = now;
            q->st = Z_PUMPING;
            gd_learn(z);                                              /* learn from the drying period that just ended */
            rate_clear(z);
        } else if ((now - q->t_idle_start) >= SETTLE_MS) {
            if (q->learn_pending) {                                    /* learn mL per percent */
                int32_t rise10 = q->pct10 - q->cycle_pct10_start;
                if (rise10 >= 100) {
                    float water_ml = ml_per_ms(z) * (float)q->cycle_pump_ms;
                    float measured = water_ml / ((float)rise10 / 10.0f);
                    if (measured < 0.05f) measured = 0.05f;
                    if (measured > 5.0f)  measured = 5.0f;
                    /* gradient step on (measured - estimate)^2: move ML_ETA of the gap; a single far-off cycle
                     * (probably a one-off) is weighted lightly */
                    float eta = ML_ETA;
                    int odd = (q->ml_n >= 2 && (measured > 2.5f * q->ml_per_pct || measured < 0.4f * q->ml_per_pct));
                    if (odd) eta *= 0.3f;
                    q->ml_err = 100.0f * (q->ml_per_pct - measured) / measured;   /* plan error: positive = planned too much water */
                    q->ml_per_pct += eta * (measured - q->ml_per_pct);
                    q->ml_n++;
                    sprintf(buf, "Z%d LEARNED: %d.%02d mL per %%, plan was off by %d%%, estimate now %d.%02d mL per %% (from %d cycles)%s\r\n", z + 1,
                            (int)measured, (int)(measured * 100) % 100, (int)q->ml_err,
                            (int)q->ml_per_pct, (int)(q->ml_per_pct * 100) % 100, q->ml_n,
                            odd ? " [unusual, weighted lightly]" : "");
                    uart2_print(buf);
                }
                q->learn_pending = 0;
            }
            if ((now - q->t_rate_last) >= RATE_SAMPLE_MS) {            /* drying sample */
                q->t_rate_last = now;
                /* right after watering the filtered reading can still be climbing; a rising sample at the start
                 * of a window is not drying, so start the window again instead of fitting a curve through it */
                float now_pct = (float)q->pct10 / 10.0f;
                if (q->rn > 0 && now_pct > q->ry[q->rn - 1] + DIST_RISE_PCT) {         /* moisture jumped up without the pump */
                    sprintf(buf, "Z%d DISTURBED: moisture rose %d.%d %% with the pump off (rain or manual watering), measuring again\r\n",
                            z + 1, (int)(now_pct - q->ry[q->rn - 1]), (int)((now_pct - q->ry[q->rn - 1]) * 10.0f) % 10);
                    uart2_print(buf);
                    q->disturbed = 1;
                    q->ne = 0;  q->ex_stride = 1;  q->ex_skip = 0;
                    rate_clear(z);
                }
                rate_add(z, (float)now / 60000.0f, (float)q->pct10 / 10.0f);
            }
        }
        break;

    case Z_PUMPING:
        if (q->pct >= c->high_pct) {                                   /* reached target mid-burst */
            pump_off(z);
            q->pump_total_ms += now - q->t_state;
            q->cycle_pump_ms += now - q->t_state;
            to_idle = 1;
        } else if ((now - q->t_state) >= c->burst_ms) {
            pump_off(z);
            q->pump_total_ms += now - q->t_state;
            q->cycle_pump_ms += now - q->t_state;
            q->bursts++;
            q->t_state = now;
            q->st = Z_SOAKING;
        }
        break;

    case Z_SOAKING:
        if ((now - q->t_state) >= SOAK_MS) {
            if (q->pct >= c->high_pct) {
                to_idle = 1;
            } else if (q->bursts >= MAX_BURSTS) {
                pump_off(z);
                q->st = Z_FAULT;                                       /* locked until RESET */
            } else if (!other_pumping(z)) {
                pump_on(z);
                q->t_state = now;
                q->st = Z_PUMPING;
            }
        }
        break;

    case Z_FAULT:
        pump_off(z);
        break;
    }

    if (to_idle) {
        q->st = Z_IDLE;
        q->t_idle_start = now;
        q->t_rate_last = now;
        q->learn_pending = 1;
        q->disturbed = 0;                                              /* a new drying period starts clean */
        q->ne = 0;  q->ex_stride = 1;  q->ex_skip = 0;
        rate_clear(z);
        q->last_bursts = q->bursts;
        q->last_ml = (int)(ml_per_ms(z) * (float)q->cycle_pump_ms + 0.5f);
        sprintf(buf, "Z%d CYCLE DONE: %d bursts, pump on %lu s, ~%d mL delivered\r\n", z + 1,
                q->bursts, (unsigned long)(q->cycle_pump_ms / 1000U), q->last_ml);
        uart2_print(buf);
    }
}

/* drying rate and time to the next watering, for the screens and the serial log */
static void zone_predict(int z)
{
    zone_t *q = &Z[z];
    const zone_cfg_t *c = &zcfg[z];
    float rate, hours, r2;
    int model;

    q->mlh_x10 = 0;
    q->rate_x100 = 0;
    q->next_min = 0;
    q->next_s = 0;
    q->next_capped = 0;
    q->model = 0; q->tau_min = 0; q->r2_x100 = 0;
    if (q->pct >= BAND_WET_MIN_PCT)  { q->pkind = P_SATURATED; return; }
    if (q->rn < RATE_MIN_SAMPLES)    { q->pkind = P_COLLECT;   return; }

    predict_from_samples(q->rx, q->ry, q->rn, (float)HAL_GetTick() / 60000.0f,
                         (float)c->low_pct, MIN_DRY_RATE, &model, &rate, &hours, &r2);
    if (model == 0) { q->pkind = P_NOT_DRYING; return; }

    /* record this prediction as a training example (the real time left becomes known when the soil is dry) */
    float w = 1.0f - (float)q->rn / (float)RATE_N;
    if (w < 0.0f) w = 0.0f;
    if (q->new_sample) {
        q->new_sample = 0;
        if (q->ex_skip++ % q->ex_stride == 0) {
            if (q->ne == EX_N) {                                          /* full: keep every second example, record half as often */
                for (int i = 0; i < EX_N / 2; i++) { q->ex_h[i] = q->ex_h[2*i]; q->ex_w[i] = q->ex_w[2*i]; q->ex_t[i] = q->ex_t[2*i]; }
                q->ne = EX_N / 2;  q->ex_stride *= 2;
            }
            q->ex_h[q->ne] = hours * 3600.0f;  q->ex_w[q->ne] = w;  q->ex_t[q->ne] = HAL_GetTick();  q->ne++;
        }
    }
    hours = hours * (1.0f + (q->ga - 1.0f) * w) + q->gb * (GD_B0 / 3600.0f) * w;     /* the learned correction */
    if (hours < 0.0f) hours = 0.0f;

    if (hours > MAX_PREDICT_H) { hours = MAX_PREDICT_H; q->next_capped = 1; }
    q->model     = model;
    q->tau_min   = 0;
    q->r2_x100   = (int)(r2 * 100.0f + 0.5f);
    q->rate_x100 = (int)(rate * 100.0f + 0.5f);
    q->mlh_x10   = (int)(rate * q->ml_per_pct * 10.0f + 0.5f);
    q->next_min  = (int)(hours * 60.0f + 0.5f);
    q->next_s    = (int)(hours * 3600.0f + 0.5f);
    q->pkind = P_OK;
}

static void zone_log(int z)
{
    zone_t *q = &Z[z];
    char buf[200];
    uint32_t now = HAL_GetTick();

#if CSV_LOG
    static uint32_t t_csv[NZONES];
    if ((now - t_csv[z]) >= CSV_LOG_MS || t_csv[z] == 0) {
        t_csv[z] = now ? now : 1U;
        /* CSV,ms,zone,raw,filt,moisture,state,pump,tank_pct,tank_ml,pred_next_min,pred_rate,model,tau_min,r2,
         *     ml_per_h,ml_per_pct,low_pct,high_pct,bursts,pred_next_s,gain,offset_s,gd_cycles,loss_pct,ml_cycles,ml_err_pct,disturbed      (tank -1 = no reading, pred_next_min -1 = none yet) */
        sprintf(buf, "CSV,%lu,%d,%u,%ld,%ld.%ld,%s,%d,%d,%d,%d,%d.%02d,%d,%d,%d.%02d,%d.%d,%d.%02d,%d,%d,%d,%d,%d.%02d,%d,%d,%d.%d,%d,%d,%d\r\n",
                (unsigned long)now, z + 1, q->adc, (long)q->adc_f, (long)(q->pct10 / 10), (long)(q->pct10 % 10),
                state_name(q->st), (q->st == Z_PUMPING) ? 1 : 0, tank_ok ? tank_pct : -1, tank_ok ? tank_ml : -1,
                (q->pkind == P_OK) ? q->next_min : -1, q->rate_x100 / 100, q->rate_x100 % 100,
                q->model, q->tau_min, q->r2_x100 / 100, q->r2_x100 % 100,
                q->mlh_x10 / 10, q->mlh_x10 % 10, (int)q->ml_per_pct, (int)(q->ml_per_pct * 100.0f) % 100,
                zcfg[z].low_pct, zcfg[z].high_pct, q->bursts, (q->pkind == P_OK) ? q->next_s : -1,
                (int)q->ga, (int)(q->ga * 100.0f) % 100, (int)(q->gb * GD_B0), q->gd_n,
                (int)q->gd_loss, (int)(q->gd_loss * 10.0f) % 10, q->ml_n, (int)q->ml_err, q->disturbed);
        uart2_print(buf);
    }
#endif
#if SERIAL_VERBOSE
    sprintf(buf, "Z%d t %lu s  raw %4u  filt %4ld  moisture %s%ld %% (%s)  %-7s  pump_on_total %lu s\r\n",
            z + 1, (unsigned long)(now / 1000U), q->adc, (long)q->adc_f, limit_tag(q->pct), (long)q->pct,
            band_name(q->pct), state_name(q->st), (unsigned long)(q->pump_total_ms / 1000U));
    uart2_print(buf);
#endif

#if SERIAL_VERBOSE
    if (q->st == Z_IDLE && (now - q->t_predict) >= PREDICT_PRINT_MS) {
        q->t_predict = now;
        switch (q->pkind) {
        case P_SATURATED:
            sprintf(buf, "Z%d PREDICT: sensor saturated above %d %%, drying not measurable yet\r\n", z + 1, BAND_WET_MIN_PCT);
            break;
        case P_COLLECT:
            sprintf(buf, "Z%d PREDICT: collecting drying data (%d/%d samples, one every %lu min)\r\n",
                    z + 1, q->rn, RATE_MIN_SAMPLES, (unsigned long)(RATE_SAMPLE_MS / 60000UL));
            break;
        case P_NOT_DRYING:
            sprintf(buf, "Z%d PREDICT: not drying measurably yet\r\n", z + 1);
            break;
        default:
            sprintf(buf, "Z%d PREDICT: drying %d.%02d %%/h (~%d.%d mL/h). Next watering in %s%dh %02dm at %d %%  [line fit, gain x%d.%02d, R2 %d.%02d]\r\n",
                    z + 1, q->rate_x100 / 100, q->rate_x100 % 100, q->mlh_x10 / 10, q->mlh_x10 % 10,
                    q->next_capped ? ">" : "~", q->next_min / 60, q->next_min % 60, zcfg[z].low_pct,
                    (int)q->ga, (int)(q->ga * 100.0f) % 100, q->r2_x100 / 100, q->r2_x100 % 100);
            break;
        }
        uart2_print(buf);
    }
#endif
}

/* ================================================================ user interface */
typedef enum { SCR_HOME = 0, SCR_DETAIL } screen_t;
static screen_t scr = SCR_HOME;
static int      scr_zone = 0;

typedef struct {
    char home_pct[10];
    char home_band[10];
    char home_tank[26];
    int  home_state;
    int  home_bar;
    char det[9][30];
} ui_cache_t;
static ui_cache_t uic[NZONES];

static void ui_cache_clear(void)
{
    memset(uic, 0, sizeof(uic));
    for (int z = 0; z < NZONES; z++) { uic[z].home_state = -1; uic[z].home_bar = -1; }
}

/* draws text only when it differs from what is already on the screen */
static void ui_put(char *cache, size_t cache_n, const char *s, int x, int y, int sc,
                   uint16_t fg, uint16_t bg, int width_chars)
{
    if (strncmp(cache, s, cache_n - 1) == 0 && strlen(s) == strlen(cache)) return;
    tft_text(x, y, s, sc, fg, bg, width_chars);
    strncpy(cache, s, cache_n - 1);
    cache[cache_n - 1] = 0;
}

static uint16_t band_colour(int32_t pct)
{
    if (pct <= BAND_DRY_MAX_PCT) return C_ORANGE;
    if (pct >= BAND_WET_MIN_PCT) return C_CYAN;
    return C_GREEN;
}

#define HOME_BAR_X   10
#define HOME_BAR_Y   120
#define HOME_BAR_W   138
#define HOME_BAR_H   16

static void ui_home_static(void)
{
    tft_fill(0, 0, SCR_W, SCR_H, C_BLACK);
    tft_fill(159, 0, 2, SCR_H, C_GREY);
    for (int z = 0; z < NZONES; z++) {
        int ox = z ? 162 : 0;
        tft_text(ox + 25, 8, z ? "ZONE 2" : "ZONE 1", 3, C_YELLOW, C_BLACK, 6);
        tft_rect(ox + HOME_BAR_X, HOME_BAR_Y, HOME_BAR_W, HOME_BAR_H, C_WHITE);
        tft_text(ox + 14, 212, "SIMULATED SOIL SENSORS", 1, C_ORANGE, C_BLACK, 22);
    }
    ui_cache_clear();
}

static void ui_home_zone(int z)
{
    const zone_t *q = &Z[z];
    int ox = z ? 162 : 0;
    char s[32];

    snprintf(s, sizeof s, "%s%ld%%", limit_tag(q->pct), (long)q->pct);
    ui_put(uic[z].home_pct, sizeof uic[z].home_pct, s, ox + 7, 44, 4, C_WHITE, C_BLACK, 6);

    ui_put(uic[z].home_band, sizeof uic[z].home_band, band_name(q->pct), ox + 37, 90, 2,
           band_colour(q->pct), C_BLACK, 7);

    int fillw = (int)q->pct * (HOME_BAR_W - 4) / 100;
    if (fillw != uic[z].home_bar) {
        tft_fill(ox + HOME_BAR_X + 2, HOME_BAR_Y + 2, fillw, HOME_BAR_H - 4, band_colour(q->pct));
        tft_fill(ox + HOME_BAR_X + 2 + fillw, HOME_BAR_Y + 2, (HOME_BAR_W - 4) - fillw, HOME_BAR_H - 4, C_BLACK);
        uic[z].home_bar = fillw;
    }

    if ((int)q->st != uic[z].home_state) {
        uint16_t bg = C_DGREY, fg = C_WHITE;
        const char *t = "PUMP OFF";
        if (q->st == Z_PUMPING)      { bg = C_GREEN;  fg = C_BLACK; t = "PUMP ON"; }
        else if (q->st == Z_SOAKING) { bg = C_YELLOW; fg = C_BLACK; t = "SOAKING"; }
        else if (q->st == Z_FAULT)   { bg = C_RED;    fg = C_WHITE; t = "FAULT!"; }
        tft_fill(ox + 10, 148, 138, 34, bg);
        tft_text(ox + 10 + (138 - (int)strlen(t) * 12) / 2, 157, t, 2, fg, bg, (int)strlen(t));
        uic[z].home_state = (int)q->st;
    }

    uint16_t tcol = C_WHITE;
    if (!tank_ok)        snprintf(s, sizeof s, "Tank level = n/a");
    else if (tank_empty) { snprintf(s, sizeof s, "Tank level = %d%% EMPTY", tank_pct); tcol = C_RED; }
    else {
        snprintf(s, sizeof s, "Tank level = %d%%", tank_pct);
        if (tank_pct < 25) tcol = C_ORANGE;
    }
    ui_put(uic[z].home_tank, sizeof uic[z].home_tank, s, ox + 8, 192, 1, tcol, C_BLACK, 23);
}

static void ui_detail_static(int z)
{
    tft_fill(0, 0, SCR_W, SCR_H, C_BLACK);
    tft_text(8, 8, z ? "ZONE 2" : "ZONE 1", 3, C_YELLOW, C_BLACK, 6);
    tft_fill(206, 4, 108, 32, C_DGREY);
    tft_rect(206, 4, 108, 32, C_WHITE);
    tft_text(226, 12, "BACK", 2, C_WHITE, C_DGREY, 4);
    tft_fill(0, 38, SCR_W, 1, C_GREY);
    tft_fill(130, 4, 70, 32, C_DGREY);                      /* demo button: force this zone dry */
    tft_rect(130, 4, 70, 32, C_ORANGE);
    tft_text(147, 12, "DRY", 2, C_ORANGE, C_DGREY, 3);
    ui_cache_clear();
}

static void ui_detail_update(int z)
{
    const zone_t *q = &Z[z];
    const zone_cfg_t *c = &zcfg[z];
    char l[9][30];
    uint32_t now = HAL_GetTick();

    snprintf(l[0], sizeof l[0], "Moisture: %s%ld%% %s", limit_tag(q->pct), (long)q->pct, band_name(q->pct));
    snprintf(l[1], sizeof l[1], "ADC: %u  filt %ld", q->adc, (long)q->adc_f);
    if (q->st == Z_PUMPING)      snprintf(l[2], sizeof l[2], "State: PUMPING b%d", q->bursts + 1);
    else if (q->st == Z_SOAKING) {
        uint32_t el = (now - q->t_state) / 1000U, tot = SOAK_MS / 1000U;
        snprintf(l[2], sizeof l[2], "State: SOAKING %lus", (unsigned long)(el < tot ? tot - el : 0));
    }
    else if (q->st == Z_FAULT)   snprintf(l[2], sizeof l[2], "State: FAULT (reset)");
    else                         snprintf(l[2], sizeof l[2], "State: idle");
    snprintf(l[3], sizeof l[3], "Water <%d%%, stop %d%%", c->low_pct, c->high_pct);

    switch (q->pkind) {
    case P_OK:
        snprintf(l[4], sizeof l[4], "Drying: %d.%02d %%/h", q->rate_x100 / 100, q->rate_x100 % 100);
        snprintf(l[5], sizeof l[5], "Water loss: %d.%d mL/h", q->mlh_x10 / 10, q->mlh_x10 % 10);
        break;
    case P_COLLECT:
        snprintf(l[4], sizeof l[4], "Drying: data %d/%d", q->rn, RATE_MIN_SAMPLES);
        snprintf(l[5], sizeof l[5], "Water loss: n/a");
        break;
    case P_SATURATED:
        snprintf(l[4], sizeof l[4], "Drying: sensor flat");
        snprintf(l[5], sizeof l[5], "Water loss: n/a");
        break;
    default:
        snprintf(l[4], sizeof l[4], "Drying: not measurable");
        snprintf(l[5], sizeof l[5], "Water loss: n/a");
        break;
    }

    if (q->st == Z_IDLE && tank_empty && q->pct < c->low_pct) snprintf(l[6], sizeof l[6], "Next: tank empty");
    else if (q->st == Z_PUMPING || q->st == Z_SOAKING) snprintf(l[6], sizeof l[6], "Next: watering now");
    else if (q->pkind == P_OK)  snprintf(l[6], sizeof l[6], "Next: %s%dh %02dm", q->next_capped ? ">" : "~", q->next_min / 60, q->next_min % 60);
    else if (q->pkind == P_COLLECT && q->disturbed) snprintf(l[6], sizeof l[6], "Next: disturbed, measuring");
    else if (q->pkind == P_COLLECT)   snprintf(l[6], sizeof l[6], "Next: waiting for data");
    else if (q->pkind == P_SATURATED) snprintf(l[6], sizeof l[6], "Next: reading flat >%d%%", BAND_WET_MIN_PCT);
    else                               snprintf(l[6], sizeof l[6], "Next: not drying yet");

    if (q->last_bursts > 0) snprintf(l[7], sizeof l[7], "Last: %d bursts %d mL", q->last_bursts, q->last_ml);
    else                    snprintf(l[7], sizeof l[7], "Last: none yet");
    snprintf(l[8], sizeof l[8], "g%d.%02d e%d%% %d.%dmL/%%(%dx)", (int)q->ga, (int)(q->ga * 100.0f) % 100, (int)q->gd_loss,
             (int)q->ml_per_pct, (int)(q->ml_per_pct * 10.0f) % 10, q->ml_n);

    for (int i = 0; i < 9; i++) {
        uint16_t col = (i == 4 || i == 5 || i == 6) ? C_CYAN : C_WHITE;
        ui_put(uic[z].det[i], sizeof uic[z].det[i], l[i], 4, 44 + i * 21, 2, col, C_BLACK, 26);
    }
}

static void ui_show(screen_t s, int z)
{
    scr = s;
    scr_zone = z;
    if (s == SCR_HOME) ui_home_static();
    else               ui_detail_static(z);
}

static void ui_update(void)
{
    if (scr == SCR_HOME) { for (int z = 0; z < NZONES; z++) ui_home_zone(z); }
    else                 { ui_detail_update(scr_zone); }
}

static void ui_tap(int x, int y)
{
    if (estop_active) return;                               /* touch is ignored during a force stop */
    if (scr == SCR_HOME) {
        if (y >= 186) {                                         /* tank level area */
            return;
        }
        ui_show(SCR_DETAIL, (x < 160) ? 0 : 1);
        ui_update();
    } else {
        if (x >= 128 && x < 200 && y <= 40) {                  /* DRY button, only when the zone is idle */
            if (Z[scr_zone].st == Z_IDLE) sim_make_dry(scr_zone);
            return;
        }
        if (x >= 200 && y <= 40) {
            ui_show(SCR_HOME, 0);
            ui_update();
        }
    }
}

/* ================================================================ force stop, main loop side */
static int estop_shown = 0;                                 /* what the screen currently shows    */

static void ui_estop_show(void)
{
    tft_fill(0, 0, SCR_W, SCR_H, C_RED);
    tft_text(40, 50,  "FORCE STOP", 4, C_WHITE, C_RED, 10);
    tft_text(52, 95,  "ACTIVATED",  4, C_WHITE, C_RED, 9);
    tft_text(106, 150, "PUMPS OFF", 2, C_WHITE, C_RED, 9);
    tft_text(58, 190, "PRESS BLUE BUTTON", 2, C_WHITE, C_RED, 17);
    tft_text(106, 210, "TO RESUME", 2, C_WHITE, C_RED, 9);
}

/* a force stop abandons every watering cycle in progress; zones re-evaluate from IDLE on resume.
 * A FAULT stays a FAULT (it is only cleared by RESET).                                          */
static void estop_abort_cycles(void)
{
    uint32_t now = HAL_GetTick();
    for (int z = 0; z < NZONES; z++) {
        zone_t *q = &Z[z];
        if (q->st == Z_PUMPING) {
            uint32_t ran = estop_tick - q->t_state;         /* pump time before the stop           */
            if (ran > zcfg[z].burst_ms) ran = 0;
            q->pump_total_ms += ran;
        }
        if (q->st == Z_PUMPING || q->st == Z_SOAKING) {
            q->st = Z_IDLE;
            q->t_idle_start = now;
            q->t_rate_last = now;
            q->learn_pending = 0;                           /* the cycle was not finished, do not learn from it */
            q->bursts = 0;
            rate_clear(z);
        }
    }
}

/* called every pass of the main loop: reacts to what the button interrupt flagged */
static void estop_service(void)
{
    if (estop_active)                                       /* keep the pumps off whatever else happens */
        for (int z = 0; z < NZONES; z++) pump_off(z);

    if (estop_hit) {
        estop_hit = 0;
        estop_abort_cycles();
    }

    if ((int)estop_active != estop_shown) {
        estop_shown = (int)estop_active;
        if (estop_active) {
            ui_estop_show();
            uart2_print("FORCE STOP ACTIVATED: pumps off, control paused. Press the blue button to resume\r\n");
        } else {
            uint32_t now = HAL_GetTick();
            for (int z = 0; z < NZONES; z++) { rate_clear(z); Z[z].t_idle_start = now; Z[z].t_rate_last = now; }
            ui_show(SCR_HOME, 0);
            ui_update();
            uart2_print("FORCE STOP RELEASED: normal operation resumed\r\n");
        }
    }
}

/* ================================================================ main */
int main(void)
{
    HAL_Init();                          /* starts SysTick (1 ms) for HAL_GetTick, clock stays HSI 16 MHz */
    uart2_init();
    zones_init();
    zones_hw_init();
    estop_init();                        /* force stop button, interrupt on PC13 */
    tank_hw_init();                      /* HC-SR04 tank level: TRIG PB3, ECHO PB4 (TIM3 capture) */
    spi_init();
    tft_init();
    uart2_print("Two-zone soil irrigation controller started (DEMO: SIMULATED SOIL SENSORS)\r\n");

#if CSV_LOG
    uart2_print("CSV_HEADER,ms,zone,raw_adc,filtered_adc,moisture_pct,state,pump_on,tank_pct,tank_ml,pred_next_min,pred_rate_pct_h,model,tau_min,r2,ml_per_h,ml_per_pct,low_pct,high_pct,bursts,pred_next_s,gain,offset_s,gd_cycles,loss_pct,ml_cycles,ml_err_pct,disturbed\r\n");
#endif

#if PUMP_TEST_AT_BOOT
    uart2_print("Pump test: both pumps on for 3 s\r\n");
    pump_on(0);  pump_on(1);
    HAL_Delay(3000);
    pump_off(0); pump_off(1);
#endif

#if TS_CAL_AT_BOOT
    ts_calibrate();
#else
    ts_swap = 0; cal_ax_a = 300; cal_ax_b = 3800; cal_ay_b = 300; cal_ay_c = 3800;
#endif

    for (int z = 0; z < NZONES; z++) { zone_sample(z); Z[z].t_idle_start = HAL_GetTick(); Z[z].t_rate_last = HAL_GetTick(); }
    for (int z = 0; z < NZONES; z++) zone_predict(z);
    tank_update();
    ui_show(SCR_HOME, 0);
    ui_update();

    uint32_t t_ctl = HAL_GetTick(), t_touch = 0, t_last_down = 0, t_tank = HAL_GetTick();
    int was_down = 0;

    while (1) {
        estop_service();
        uint32_t now = HAL_GetTick();

        /* serial command: '1' or '2' = simulate rain on that zone (sent by the dashboard button) */
        if (USART2->SR & USART_SR_RXNE) {
            char ch = (char)USART2->DR;
            if (ch >= '1' && ch < (char)('1' + NZONES)) {
                sim_rain(ch - '1');
                char rb[48];
                sprintf(rb, "Z%d RAIN simulated: +%d %%\r\n", ch - '1' + 1, (int)SIM_RAIN_PCT);
                uart2_print(rb);
            }
        }

        /* touch, polled every 30 ms; a tap is the moment the screen is first touched */
        if ((now - t_touch) >= 30U) {
            t_touch = now;
            int rx, ry;
            if (ts_read_raw(&rx, &ry)) {
                t_last_down = now;
                if (!was_down) {
                    was_down = 1;
                    int sx, sy;
                    ts_map(rx, ry, &sx, &sy);
                    ui_tap(sx, sy);
                }
            } else if (was_down && (now - t_last_down) > 120U) {
                was_down = 0;
            }
        }

        /* control, once a second for every zone */
        if ((now - t_ctl) >= 1000U) {
            t_ctl += 1000U;
            if (!estop_active) {                                       /* control is paused during a force stop */
                if ((now - t_tank) >= TANK_SAMPLE_MS) { t_tank = now; tank_update(); }
                for (int z = 0; z < NZONES; z++) {
                    zone_sample(z);
                    zone_step(z);
                    zone_predict(z);
                    zone_log(z);
                }
                ui_update();
            }
        }
    }
}
