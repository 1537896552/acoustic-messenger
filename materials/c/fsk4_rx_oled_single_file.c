#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/*
 * STM32F103 RX single-file 4FSK demo with I2C OLED
 *
 * Wiring:
 *   PA1  <- receive analog board output (biased around 1.65V, within 0~3.3V)
 *   PB6  -> OLED SCL
 *   PB7  -> OLED SDA
 *   PC13 -> optional onboard LED
 *
 * OLED module pin order from your photo:
 *   GND  VCC  SCL  SDA
 *
 * OLED address:
 *   tries 0x3C first, then 0x3D
 *
 * Screen behavior:
 *   power on    : RX WAIT
 *   seeing tone : SYNC
 *   valid frame : RX: TEST
 */

#define SYSCLK_HZ                 72000000UL
#define FSK4_SAMPLE_RATE          16000U
#define FSK4_SYMBOL_MS               40U
#define FSK4_SYMBOL_SAMPLES        640U

#define FSK4_FREQ_0               1400U
#define FSK4_FREQ_1               1600U
#define FSK4_FREQ_2               1800U
#define FSK4_FREQ_3               2000U

#define FSK4_PREAMBLE_LEN           16U
#define FSK4_SYNC_LEN                8U
#define FSK4_MAX_PAYLOAD            32U

#define ADC_MIDPOINT              2048
#define PEAK_THRESHOLD              80
#define SILENCE_RESET_WINDOWS        4

#define OLED_WIDTH                128U
#define OLED_PAGES                  8U
#define OLED_ADDR0               0x3CU
#define OLED_ADDR1               0x3DU

#define RCC_BASE                0x40021000UL
#define FLASH_BASE              0x40022000UL
#define GPIOA_BASE              0x40010800UL
#define GPIOB_BASE              0x40010C00UL
#define GPIOC_BASE              0x40011000UL
#define AFIO_BASE               0x40010000UL
#define TIM3_BASE               0x40000400UL
#define ADC1_BASE               0x40012400UL

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
} RCC_TypeDef;

typedef struct {
    volatile uint32_t ACR;
    volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
    volatile uint32_t CR;
    volatile uint32_t AR;
    volatile uint32_t RESERVED;
    volatile uint32_t OBR;
    volatile uint32_t WRPR;
} FLASH_TypeDef;

typedef struct {
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t EVCR;
    volatile uint32_t MAPR;
    volatile uint32_t EXTICR1;
    volatile uint32_t EXTICR2;
    volatile uint32_t EXTICR3;
    volatile uint32_t EXTICR4;
    volatile uint32_t MAPR2;
} AFIO_TypeDef;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} TIM_TypeDef;

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    volatile uint32_t JOFR1;
    volatile uint32_t JOFR2;
    volatile uint32_t JOFR3;
    volatile uint32_t JOFR4;
    volatile uint32_t HTR;
    volatile uint32_t LTR;
    volatile uint32_t SQR1;
    volatile uint32_t SQR2;
    volatile uint32_t SQR3;
    volatile uint32_t JSQR;
    volatile uint32_t JDR1;
    volatile uint32_t JDR2;
    volatile uint32_t JDR3;
    volatile uint32_t JDR4;
    volatile uint32_t DR;
} ADC_TypeDef;

#define RCC                      ((RCC_TypeDef *)RCC_BASE)
#define FLASH                    ((FLASH_TypeDef *)FLASH_BASE)
#define GPIOA                    ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB                    ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC                    ((GPIO_TypeDef *)GPIOC_BASE)
#define AFIO                     ((AFIO_TypeDef *)AFIO_BASE)
#define TIM3                     ((TIM_TypeDef *)TIM3_BASE)
#define ADC1                     ((ADC_TypeDef *)ADC1_BASE)

typedef struct {
    uint8_t state;
    uint8_t preamble_count;
    uint8_t sync_index;
    uint8_t current_byte;
    uint8_t symbols_in_byte;
    uint8_t length;
    uint8_t payload[FSK4_MAX_PAYLOAD];
    uint8_t payload_pos;
    uint8_t checksum;
    uint8_t rx_checksum;
} FSK4_RxContext;

typedef struct {
    uint32_t e1400;
    uint32_t e1600;
    uint32_t e1800;
    uint32_t e2000;
} FSK4_Energy;

static const uint8_t k_sync_symbols[FSK4_SYNC_LEN] = {0, 1, 2, 3, 3, 2, 1, 0};
volatile uint8_t g_message_ready;
volatile uint8_t g_last_len;
volatile char g_last_message[FSK4_MAX_PAYLOAD + 1];
static int16_t g_symbol_samples[FSK4_SYMBOL_SAMPLES];
static uint8_t g_oled_addr;
static uint8_t g_oled_buf[OLED_WIDTH * OLED_PAGES];

static void clock_init_72mhz(void);
static void delay_cycles(uint32_t cycles);
static void delay_ms(uint32_t ms);
static void led_init_pc13(void);
static void led_on(void);
static void led_off(void);
static void led_toggle(void);
static void gpio_init(void);
static void tim3_init_16khz(void);
static void adc1_init_pa1(void);
static uint16_t adc1_read_once(void);
static void capture_symbol_window(int16_t *dst);
static int16_t peak_abs(const int16_t *samples, uint32_t count);
static void rx_init(FSK4_RxContext *ctx);
static bool feed_symbol(FSK4_RxContext *ctx, uint8_t symbol, uint8_t *out_payload, uint8_t *out_len);
static uint8_t detect_symbol_goertzel(const int16_t *samples, uint32_t count, FSK4_Energy *energy);
static uint32_t goertzel_power(const int16_t *samples, uint32_t count, uint32_t freq_hz);

static void i2c_gpio_init(void);
static void i2c_scl_high(void);
static void i2c_scl_low(void);
static void i2c_sda_high(void);
static void i2c_sda_low(void);
static uint8_t i2c_sda_read(void);
static void i2c_start(void);
static void i2c_stop(void);
static bool i2c_write_byte(uint8_t b);
static bool oled_write_cmd(uint8_t cmd);
static bool oled_write_data(const uint8_t *data, uint16_t len);
static bool oled_try_addr(uint8_t addr);
static bool oled_init_detect(void);
static void oled_clear_buf(void);
static void oled_update(void);
static void oled_set_pos(uint8_t page, uint8_t col);
static void oled_draw_char(uint8_t x, uint8_t page, char c);
static void oled_draw_text(uint8_t x, uint8_t page, const char *s);
static void oled_show_wait(void);
static void oled_show_sync(void);
static void oled_show_message(const char *s);

int main(void)
{
    FSK4_RxContext rx;
    uint8_t msg[FSK4_MAX_PAYLOAD];
    uint8_t len = 0;
    uint8_t idle_windows = 0;
    uint32_t blink_div = 0;
    uint8_t syncing = 0;

    clock_init_72mhz();
    led_init_pc13();
    gpio_init();
    tim3_init_16khz();
    adc1_init_pa1();
    i2c_gpio_init();
    oled_init_detect();
    oled_show_wait();
    rx_init(&rx);

    while (1) {
        FSK4_Energy energy;
        uint8_t symbol;
        int16_t peak;

        capture_symbol_window(g_symbol_samples);
        peak = peak_abs(g_symbol_samples, FSK4_SYMBOL_SAMPLES);

        if (peak < PEAK_THRESHOLD) {
            idle_windows++;
            if (idle_windows >= SILENCE_RESET_WINDOWS) {
                rx_init(&rx);
                idle_windows = 0;
                if (syncing) {
                    syncing = 0;
                    oled_show_wait();
                }
            }
        } else {
            idle_windows = 0;
            if (!syncing) {
                syncing = 1;
                oled_show_sync();
            }

            symbol = detect_symbol_goertzel(g_symbol_samples, FSK4_SYMBOL_SAMPLES, &energy);
            if (feed_symbol(&rx, symbol, msg, &len)) {
                uint8_t i;
                g_last_len = len;
                for (i = 0; i < len; ++i) {
                    g_last_message[i] = (char)msg[i];
                }
                g_last_message[len] = '\0';
                g_message_ready = 1;

                oled_show_message((const char *)g_last_message);
                led_on();
                delay_ms(300);
                led_off();
                syncing = 0;
            }
        }

        blink_div++;
        if ((blink_div & 0x07U) == 0U) {
            led_toggle();
        }
    }
}

static void clock_init_72mhz(void)
{
    RCC->CR |= (1UL << 16);
    while ((RCC->CR & (1UL << 17)) == 0U) {}

    FLASH->ACR = (1UL << 4) | 0x2UL;
    RCC->CFGR = 0;
    RCC->CFGR |= (0x4UL << 8);
    RCC->CFGR |= (0x7UL << 18);
    RCC->CFGR |= (1UL << 16);

    RCC->CR |= (1UL << 24);
    while ((RCC->CR & (1UL << 25)) == 0U) {}

    RCC->CFGR &= ~0x3UL;
    RCC->CFGR |= 0x2UL;
    while ((RCC->CFGR & 0xCUL) != 0x8UL) {}
}

static void delay_cycles(uint32_t cycles)
{
    while (cycles--) {
        __asm volatile ("nop");
    }
}

static void delay_ms(uint32_t ms)
{
    uint32_t i;
    while (ms--) {
        for (i = 0; i < (SYSCLK_HZ / 8000UL); ++i) {
            __asm volatile ("nop");
        }
    }
}

static void led_init_pc13(void)
{
    RCC->APB2ENR |= (1UL << 4);
    GPIOC->CRH &= ~(0xFUL << 20);
    GPIOC->CRH |= (0x2UL << 20);
    led_off();
}

static void led_on(void)  { GPIOC->BRR  = (1UL << 13); }
static void led_off(void) { GPIOC->BSRR = (1UL << 13); }
static void led_toggle(void) { GPIOC->ODR ^= (1UL << 13); }

static void gpio_init(void)
{
    RCC->APB2ENR |= (1UL << 0) | (1UL << 2) | (1UL << 3);
    AFIO->MAPR = AFIO->MAPR;

    GPIOA->CRL &= ~(0xFUL << 4); /* PA1 analog */
}

static void tim3_init_16khz(void)
{
    RCC->APB1ENR |= (1UL << 1);
    TIM3->PSC = 0;
    TIM3->ARR = (SYSCLK_HZ / FSK4_SAMPLE_RATE) - 1UL;
    TIM3->EGR = 1UL;
    TIM3->CR1 = 1UL;
}

static void adc1_init_pa1(void)
{
    RCC->APB2ENR |= (1UL << 9);

    ADC1->SMPR2 &= ~(0x7UL << 3);
    ADC1->SMPR2 |=  (0x6UL << 3);
    ADC1->SQR1 = 0;
    ADC1->SQR2 = 0;
    ADC1->SQR3 = 1UL;

    ADC1->CR2 &= ~(0x7UL << 17);
    ADC1->CR2 |=  (0x7UL << 17);
    ADC1->CR2 |=  (1UL << 20);
    ADC1->CR2 |=  (1UL << 0);
    delay_ms(2);

    ADC1->CR2 |= (1UL << 3);
    while ((ADC1->CR2 & (1UL << 3)) != 0U) {}

    ADC1->CR2 |= (1UL << 2);
    while ((ADC1->CR2 & (1UL << 2)) != 0U) {}
}

static uint16_t adc1_read_once(void)
{
    ADC1->SR = 0;
    ADC1->CR2 |= (1UL << 22);
    while ((ADC1->SR & (1UL << 1)) == 0U) {}
    return (uint16_t)ADC1->DR;
}

static void capture_symbol_window(int16_t *dst)
{
    uint32_t i;
    for (i = 0; i < FSK4_SYMBOL_SAMPLES; ++i) {
        while ((TIM3->SR & 1UL) == 0U) {}
        TIM3->SR &= ~1UL;
        dst[i] = (int16_t)((int32_t)adc1_read_once() - ADC_MIDPOINT);
    }
}

static int16_t abs16(int16_t x)
{
    return (x < 0) ? (int16_t)(-x) : x;
}

static int16_t peak_abs(const int16_t *samples, uint32_t count)
{
    uint32_t i;
    int16_t peak = 0;
    for (i = 0; i < count; ++i) {
        int16_t a = abs16(samples[i]);
        if (a > peak) peak = a;
    }
    return peak;
}

static void rx_init(FSK4_RxContext *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->state = 0;
}

static bool feed_symbol(FSK4_RxContext *ctx, uint8_t symbol, uint8_t *out_payload, uint8_t *out_len)
{
    switch (ctx->state) {
    case 0:
        if (symbol == ((ctx->preamble_count & 1U) ? 3U : 0U)) {
            ctx->preamble_count++;
            if (ctx->preamble_count >= FSK4_PREAMBLE_LEN) {
                ctx->state = 1;
                ctx->sync_index = 0;
            }
        } else {
            ctx->preamble_count = (symbol == 0U) ? 1U : 0U;
        }
        break;

    case 1:
        if (symbol == k_sync_symbols[ctx->sync_index]) {
            ctx->sync_index++;
            if (ctx->sync_index >= FSK4_SYNC_LEN) {
                ctx->state = 2;
                ctx->current_byte = 0;
                ctx->symbols_in_byte = 0;
            }
        } else {
            rx_init(ctx);
        }
        break;

    case 2:
    case 3:
    case 4:
        ctx->current_byte = (uint8_t)((ctx->current_byte << 2) | (symbol & 0x03U));
        ctx->symbols_in_byte++;
        if (ctx->symbols_in_byte >= 4U) {
            ctx->symbols_in_byte = 0U;
            if (ctx->state == 2) {
                ctx->length = ctx->current_byte;
                ctx->payload_pos = 0;
                ctx->checksum = 0;
                if (ctx->length == 0U || ctx->length > FSK4_MAX_PAYLOAD) {
                    rx_init(ctx);
                } else {
                    ctx->state = 3;
                }
            } else if (ctx->state == 3) {
                ctx->payload[ctx->payload_pos++] = ctx->current_byte;
                ctx->checksum = (uint8_t)(ctx->checksum + ctx->current_byte);
                if (ctx->payload_pos >= ctx->length) {
                    ctx->state = 4;
                }
            } else {
                ctx->rx_checksum = ctx->current_byte;
                if (ctx->rx_checksum == ctx->checksum) {
                    memcpy(out_payload, ctx->payload, ctx->length);
                    *out_len = ctx->length;
                    rx_init(ctx);
                    return true;
                }
                rx_init(ctx);
            }
            ctx->current_byte = 0U;
        }
        break;

    default:
        rx_init(ctx);
        break;
    }
    return false;
}

static uint32_t goertzel_power(const int16_t *samples, uint32_t count, uint32_t freq_hz)
{
    double omega = 2.0 * 3.14159265358979323846 * (double)freq_hz / (double)FSK4_SAMPLE_RATE;
    double coeff = 2.0 * cos(omega);
    double q0 = 0.0;
    double q1 = 0.0;
    double q2 = 0.0;
    uint32_t i;

    for (i = 0; i < count; ++i) {
        q0 = coeff * q1 - q2 + (double)samples[i];
        q2 = q1;
        q1 = q0;
    }

    {
        double power = q1 * q1 + q2 * q2 - coeff * q1 * q2;
        if (power < 0.0) power = 0.0;
        if (power > 4294967295.0) power = 4294967295.0;
        return (uint32_t)power;
    }
}

static uint8_t detect_symbol_goertzel(const int16_t *samples, uint32_t count, FSK4_Energy *energy)
{
    uint32_t best;
    uint8_t symbol = 0;

    energy->e1400 = goertzel_power(samples, count, FSK4_FREQ_0);
    energy->e1600 = goertzel_power(samples, count, FSK4_FREQ_1);
    energy->e1800 = goertzel_power(samples, count, FSK4_FREQ_2);
    energy->e2000 = goertzel_power(samples, count, FSK4_FREQ_3);

    best = energy->e1400;
    if (energy->e1600 > best) { best = energy->e1600; symbol = 1U; }
    if (energy->e1800 > best) { best = energy->e1800; symbol = 2U; }
    if (energy->e2000 > best) { symbol = 3U; }
    return symbol;
}

static void i2c_gpio_init(void)
{
    /* PB6, PB7 open-drain outputs, 2MHz */
    GPIOB->CRL &= ~((0xFUL << 24) | (0xFUL << 28));
    GPIOB->CRL |=  ((0x6UL << 24) | (0x6UL << 28));
    i2c_scl_high();
    i2c_sda_high();
}

static void i2c_scl_high(void) { GPIOB->BSRR = (1UL << 6); }
static void i2c_scl_low(void)  { GPIOB->BRR  = (1UL << 6); }
static void i2c_sda_high(void) { GPIOB->BSRR = (1UL << 7); }
static void i2c_sda_low(void)  { GPIOB->BRR  = (1UL << 7); }
static uint8_t i2c_sda_read(void) { return (uint8_t)((GPIOB->IDR >> 7) & 1U); }

static void i2c_start(void)
{
    i2c_sda_high();
    i2c_scl_high();
    delay_cycles(80);
    i2c_sda_low();
    delay_cycles(80);
    i2c_scl_low();
}

static void i2c_stop(void)
{
    i2c_sda_low();
    delay_cycles(80);
    i2c_scl_high();
    delay_cycles(80);
    i2c_sda_high();
    delay_cycles(80);
}

static bool i2c_write_byte(uint8_t b)
{
    uint8_t i;
    for (i = 0; i < 8U; ++i) {
        if (b & 0x80U) i2c_sda_high();
        else           i2c_sda_low();
        delay_cycles(40);
        i2c_scl_high();
        delay_cycles(80);
        i2c_scl_low();
        b <<= 1;
    }

    i2c_sda_high();
    delay_cycles(40);
    i2c_scl_high();
    delay_cycles(80);
    i2c_scl_low();

    /* Most modules ACK low; if floating/high, still continue. */
    return (i2c_sda_read() == 0U);
}

static bool oled_write_cmd(uint8_t cmd)
{
    i2c_start();
    if (!i2c_write_byte((uint8_t)(g_oled_addr << 1))) {
        i2c_stop();
        return false;
    }
    i2c_write_byte(0x00);
    i2c_write_byte(cmd);
    i2c_stop();
    return true;
}

static bool oled_write_data(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    i2c_start();
    if (!i2c_write_byte((uint8_t)(g_oled_addr << 1))) {
        i2c_stop();
        return false;
    }
    i2c_write_byte(0x40);
    for (i = 0; i < len; ++i) {
        i2c_write_byte(data[i]);
    }
    i2c_stop();
    return true;
}

static bool oled_try_addr(uint8_t addr)
{
    g_oled_addr = addr;
    i2c_start();
    if (!i2c_write_byte((uint8_t)(g_oled_addr << 1))) {
        i2c_stop();
        return false;
    }
    i2c_stop();
    return true;
}

static bool oled_init_detect(void)
{
    static const uint8_t init_seq[] = {
        0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB,
        0x40, 0x8D, 0x14, 0xAF
    };
    uint32_t i;

    if (!oled_try_addr(OLED_ADDR0)) {
        if (!oled_try_addr(OLED_ADDR1)) {
            g_oled_addr = OLED_ADDR0;
            return false;
        }
    }

    delay_ms(50);
    for (i = 0; i < sizeof(init_seq); ++i) {
        oled_write_cmd(init_seq[i]);
    }
    oled_clear_buf();
    oled_update();
    return true;
}

static void oled_clear_buf(void)
{
    memset(g_oled_buf, 0, sizeof(g_oled_buf));
}

static void oled_set_pos(uint8_t page, uint8_t col)
{
    oled_write_cmd((uint8_t)(0xB0U + page));
    oled_write_cmd((uint8_t)(0x00U + (col & 0x0FU)));
    oled_write_cmd((uint8_t)(0x10U + ((col >> 4) & 0x0FU)));
}

static void oled_update(void)
{
    uint8_t page;
    for (page = 0; page < OLED_PAGES; ++page) {
        oled_set_pos(page, 0);
        oled_write_data(&g_oled_buf[page * OLED_WIDTH], OLED_WIDTH);
    }
}

static const uint8_t *font5x7(char c)
{
    static const uint8_t blank[5] = {0,0,0,0,0};
    static const uint8_t colon[5] = {0x00,0x36,0x36,0x00,0x00};
    static const uint8_t dash[5]  = {0x08,0x08,0x08,0x08,0x08};
    static const uint8_t R[5]     = {0x7F,0x09,0x19,0x29,0x46};
    static const uint8_t X[5]     = {0x63,0x14,0x08,0x14,0x63};
    static const uint8_t W[5]     = {0x7F,0x20,0x18,0x20,0x7F};
    static const uint8_t A[5]     = {0x7E,0x11,0x11,0x11,0x7E};
    static const uint8_t I[5]     = {0x00,0x41,0x7F,0x41,0x00};
    static const uint8_t T[5]     = {0x01,0x01,0x7F,0x01,0x01};
    static const uint8_t S[5]     = {0x46,0x49,0x49,0x49,0x31};
    static const uint8_t Y[5]     = {0x03,0x04,0x78,0x04,0x03};
    static const uint8_t N[5]     = {0x7F,0x02,0x0C,0x10,0x7F};
    static const uint8_t C[5]     = {0x3E,0x41,0x41,0x41,0x22};
    static const uint8_t E[5]     = {0x7F,0x49,0x49,0x49,0x41};
    static const uint8_t O[5]     = {0x3E,0x41,0x41,0x41,0x3E};
    static const uint8_t K[5]     = {0x7F,0x08,0x14,0x22,0x41};
    static const uint8_t zero[5]  = {0x3E,0x45,0x49,0x51,0x3E};
    static const uint8_t one[5]   = {0x00,0x21,0x7F,0x01,0x00};
    static const uint8_t two[5]   = {0x23,0x45,0x49,0x51,0x21};
    static const uint8_t three[5] = {0x22,0x41,0x49,0x49,0x36};
    static const uint8_t four[5]  = {0x18,0x28,0x48,0x7F,0x08};
    static const uint8_t five[5]  = {0x72,0x51,0x51,0x51,0x4E};
    static const uint8_t six[5]   = {0x1E,0x29,0x49,0x49,0x06};
    static const uint8_t seven[5] = {0x40,0x47,0x48,0x50,0x60};
    static const uint8_t eight[5] = {0x36,0x49,0x49,0x49,0x36};
    static const uint8_t nine[5]  = {0x30,0x49,0x49,0x4A,0x3C};

    switch (c) {
    case ' ': return blank;
    case ':': return colon;
    case '-': return dash;
    case 'R': return R;
    case 'X': return X;
    case 'W': return W;
    case 'A': return A;
    case 'I': return I;
    case 'T': return T;
    case 'S': return S;
    case 'Y': return Y;
    case 'N': return N;
    case 'C': return C;
    case 'E': return E;
    case 'O': return O;
    case 'K': return K;
    case '0': return zero;
    case '1': return one;
    case '2': return two;
    case '3': return three;
    case '4': return four;
    case '5': return five;
    case '6': return six;
    case '7': return seven;
    case '8': return eight;
    case '9': return nine;
    default:  return blank;
    }
}

static void oled_draw_char(uint8_t x, uint8_t page, char c)
{
    const uint8_t *p = font5x7(c);
    uint8_t i;
    uint16_t base = (uint16_t)page * OLED_WIDTH + x;
    for (i = 0; i < 5U; ++i) {
        if ((uint16_t)(x + i) < OLED_WIDTH) {
            g_oled_buf[base + i] = p[i];
        }
    }
    if ((uint16_t)(x + 5U) < OLED_WIDTH) {
        g_oled_buf[base + 5U] = 0x00;
    }
}

static void oled_draw_text(uint8_t x, uint8_t page, const char *s)
{
    while (*s != '\0' && x < (OLED_WIDTH - 6U)) {
        oled_draw_char(x, page, *s++);
        x = (uint8_t)(x + 6U);
    }
}

static void oled_show_wait(void)
{
    oled_clear_buf();
    oled_draw_text(0, 0, "RX WAIT");
    oled_draw_text(0, 2, "NO SIGNAL");
    oled_update();
}

static void oled_show_sync(void)
{
    oled_clear_buf();
    oled_draw_text(0, 0, "RX WAIT");
    oled_draw_text(0, 2, "SYNC");
    oled_update();
}

static void oled_show_message(const char *s)
{
    oled_clear_buf();
    oled_draw_text(0, 0, "RX OK");
    oled_draw_text(0, 2, "RX:");
    oled_draw_text(24, 2, s);
    oled_update();
}
