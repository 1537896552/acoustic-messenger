#include <stdint.h>
#include <string.h>
#include <math.h>

/*
 * STM32F103 TX single-file 4FSK demo
 *
 * Function:
 *   - Repeatedly sends fixed message "TEST"
 *   - PA0 outputs PWM-based analog tone after RC low-pass
 *   - PC13 optional onboard LED indicates transmit activity
 *
 * Hardware:
 *   PA0 -> 3.3k -> nodeA -> 3.3k -> nodeB -> power amp input
 *                  |                 |
 *                 10nF              10nF
 *                  |                 |
 *                 GND               GND
 *
 * Symbol mapping:
 *   0 -> 1400 Hz
 *   1 -> 1600 Hz
 *   2 -> 1800 Hz
 *   3 -> 2000 Hz
 */

#define SYSCLK_HZ                 72000000UL
#define PWM_PERIOD_COUNTS              255U
#define PWM_CENTER_COUNTS              128U
#define PWM_AMPLITUDE_COUNTS            19U

#define FSK4_SYMBOL_MS                40U
#define FSK4_PREAMBLE_LEN             16U
#define FSK4_SYNC_LEN                  8U
#define FSK4_MAX_PAYLOAD              32U
#define FSK4_MAX_FRAME_SYMBOLS       (FSK4_PREAMBLE_LEN + FSK4_SYNC_LEN + ((1U + FSK4_MAX_PAYLOAD + 1U) * 4U))

#define TONE_1400_SAMPLES            201U
#define TONE_1600_SAMPLES            176U
#define TONE_1800_SAMPLES            156U
#define TONE_2000_SAMPLES            141U
#define FRAME_GAP_MS                1200U

#define RCC_BASE                0x40021000UL
#define FLASH_BASE              0x40022000UL
#define GPIOA_BASE              0x40010800UL
#define GPIOC_BASE              0x40011000UL
#define AFIO_BASE               0x40010000UL
#define TIM2_BASE               0x40000000UL
#define DMA1_BASE               0x40020000UL

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
    volatile uint32_t CCR;
    volatile uint32_t CNDTR;
    volatile uint32_t CPAR;
    volatile uint32_t CMAR;
    volatile uint32_t RESERVED;
} DMA_Channel_TypeDef;

typedef struct {
    volatile uint32_t ISR;
    volatile uint32_t IFCR;
    DMA_Channel_TypeDef CH1;
    DMA_Channel_TypeDef CH2;
    DMA_Channel_TypeDef CH3;
    DMA_Channel_TypeDef CH4;
    DMA_Channel_TypeDef CH5;
    DMA_Channel_TypeDef CH6;
    DMA_Channel_TypeDef CH7;
} DMA_TypeDef;

#define RCC                      ((RCC_TypeDef *)RCC_BASE)
#define FLASH                    ((FLASH_TypeDef *)FLASH_BASE)
#define GPIOA                    ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOC                    ((GPIO_TypeDef *)GPIOC_BASE)
#define AFIO                     ((AFIO_TypeDef *)AFIO_BASE)
#define TIM2                     ((TIM_TypeDef *)TIM2_BASE)
#define DMA1                     ((DMA_TypeDef *)DMA1_BASE)
#define DMA1_CH2                 (&DMA1->CH2)

static const uint8_t k_sync_symbols[FSK4_SYNC_LEN] = {0, 1, 2, 3, 3, 2, 1, 0};
static uint16_t g_wave_1400[TONE_1400_SAMPLES];
static uint16_t g_wave_1600[TONE_1600_SAMPLES];
static uint16_t g_wave_1800[TONE_1800_SAMPLES];
static uint16_t g_wave_2000[TONE_2000_SAMPLES];
static uint8_t g_symbols[FSK4_MAX_FRAME_SYMBOLS];

static void clock_init_72mhz(void);
static void delay_ms(uint32_t ms);
static void led_init_pc13(void);
static void led_on(void);
static void led_off(void);
static void gpio_init(void);
static void tim2_pwm_init(void);
static void dma1_ch2_init(void);
static void build_sine_table(uint16_t *dst, uint32_t samples);
static void start_wave_dma(const uint16_t *buf, uint32_t len);
static void stop_wave_dma(void);
static void play_symbol(uint8_t sym);
static uint8_t checksum8(const uint8_t *data, uint8_t len);
static void push_byte_symbols(uint8_t b, uint8_t *symbols, uint32_t *pos);
static uint32_t build_frame(const uint8_t *payload, uint8_t len, uint8_t *symbols, uint32_t capacity);

int main(void)
{
    static const uint8_t message[] = {'T', 'E', 'S', 'T'};
    uint32_t i;
    uint32_t frame_symbols;

    clock_init_72mhz();
    led_init_pc13();
    gpio_init();
    tim2_pwm_init();
    dma1_ch2_init();

    build_sine_table(g_wave_1400, TONE_1400_SAMPLES);
    build_sine_table(g_wave_1600, TONE_1600_SAMPLES);
    build_sine_table(g_wave_1800, TONE_1800_SAMPLES);
    build_sine_table(g_wave_2000, TONE_2000_SAMPLES);

    frame_symbols = build_frame(message, (uint8_t)sizeof(message), g_symbols, FSK4_MAX_FRAME_SYMBOLS);

    while (1) {
        led_on();
        for (i = 0; i < frame_symbols; ++i) {
            play_symbol(g_symbols[i]);
            delay_ms(FSK4_SYMBOL_MS);
        }
        stop_wave_dma();
        led_off();
        delay_ms(FRAME_GAP_MS);
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

static void gpio_init(void)
{
    RCC->APB2ENR |= (1UL << 0) | (1UL << 2);
    AFIO->MAPR &= ~(0x3UL << 8);

    GPIOA->CRL &= ~0xFUL;
    GPIOA->CRL |= 0xBUL; /* PA0 AF push-pull */
}

static void tim2_pwm_init(void)
{
    RCC->APB1ENR |= (1UL << 0);

    TIM2->PSC = 0;
    TIM2->ARR = PWM_PERIOD_COUNTS;
    TIM2->CCR1 = PWM_CENTER_COUNTS;
    TIM2->CCMR1 = 0;
    TIM2->CCMR1 |= (6UL << 4);
    TIM2->CCMR1 |= (1UL << 3);
    TIM2->CCER = 1UL;
    TIM2->CR1 = (1UL << 7);
    TIM2->EGR = 1UL;
    TIM2->CR2 |= (2UL << 4);
    TIM2->CR1 |= 1UL;
}

static void dma1_ch2_init(void)
{
    RCC->AHBENR |= 1UL;
    DMA1_CH2->CCR = 0;
    DMA1_CH2->CNDTR = 0;
    DMA1_CH2->CPAR = (uint32_t)&TIM2->CCR1;
    DMA1_CH2->CMAR = 0;
}

static void build_sine_table(uint16_t *dst, uint32_t samples)
{
    uint32_t i;
    for (i = 0; i < samples; ++i) {
        double angle = (2.0 * 3.14159265358979323846 * (double)i) / (double)samples;
        double s = sin(angle);
        int32_t v = (int32_t)(PWM_CENTER_COUNTS + (double)PWM_AMPLITUDE_COUNTS * s);
        if (v < 0) v = 0;
        if (v > (int32_t)PWM_PERIOD_COUNTS) v = PWM_PERIOD_COUNTS;
        dst[i] = (uint16_t)v;
    }
}

static void start_wave_dma(const uint16_t *buf, uint32_t len)
{
    DMA1_CH2->CCR &= ~1UL;
    TIM2->DIER &= ~(1UL << 8);
    DMA1->IFCR = (0x0FUL << 4);

    DMA1_CH2->CPAR = (uint32_t)&TIM2->CCR1;
    DMA1_CH2->CMAR = (uint32_t)buf;
    DMA1_CH2->CNDTR = len;
    DMA1_CH2->CCR =
        (1UL << 7)  |
        (1UL << 5)  |
        (1UL << 4)  |
        (1UL << 10) |
        (1UL << 8)  |
        (2UL << 12);

    DMA1_CH2->CCR |= 1UL;
    TIM2->DIER |= (1UL << 8);
}

static void stop_wave_dma(void)
{
    TIM2->DIER &= ~(1UL << 8);
    DMA1_CH2->CCR &= ~1UL;
    TIM2->CCR1 = PWM_CENTER_COUNTS;
}

static void play_symbol(uint8_t sym)
{
    switch (sym & 0x03U) {
    case 0: start_wave_dma(g_wave_1400, TONE_1400_SAMPLES); break;
    case 1: start_wave_dma(g_wave_1600, TONE_1600_SAMPLES); break;
    case 2: start_wave_dma(g_wave_1800, TONE_1800_SAMPLES); break;
    default:start_wave_dma(g_wave_2000, TONE_2000_SAMPLES); break;
    }
}

static uint8_t checksum8(const uint8_t *data, uint8_t len)
{
    uint16_t sum = 0;
    uint8_t i;
    for (i = 0; i < len; ++i) sum = (uint16_t)(sum + data[i]);
    return (uint8_t)sum;
}

static uint8_t byte_get_symbol(uint8_t b, uint8_t index)
{
    return (uint8_t)((b >> (6U - 2U * index)) & 0x03U);
}

static void push_byte_symbols(uint8_t b, uint8_t *symbols, uint32_t *pos)
{
    symbols[(*pos)++] = byte_get_symbol(b, 0);
    symbols[(*pos)++] = byte_get_symbol(b, 1);
    symbols[(*pos)++] = byte_get_symbol(b, 2);
    symbols[(*pos)++] = byte_get_symbol(b, 3);
}

static uint32_t build_frame(const uint8_t *payload, uint8_t len, uint8_t *symbols, uint32_t capacity)
{
    uint32_t pos = 0;
    uint8_t i;
    uint8_t crc;

    if (len > FSK4_MAX_PAYLOAD) return 0;
    if (capacity < (FSK4_PREAMBLE_LEN + FSK4_SYNC_LEN + ((1U + len + 1U) * 4U))) return 0;

    for (i = 0; i < FSK4_PREAMBLE_LEN; ++i) {
        symbols[pos++] = (i & 1U) ? 3U : 0U;
    }
    for (i = 0; i < FSK4_SYNC_LEN; ++i) {
        symbols[pos++] = k_sync_symbols[i];
    }

    push_byte_symbols(len, symbols, &pos);
    for (i = 0; i < len; ++i) push_byte_symbols(payload[i], symbols, &pos);

    crc = checksum8(payload, len);
    push_byte_symbols(crc, symbols, &pos);
    return pos;
}
