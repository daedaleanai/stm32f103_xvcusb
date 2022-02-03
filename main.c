/*
 */
#include "stm32f103_md.h"

#include "clock.h"
#include "gpio2.h"
#include "printf.h"
#include "usart.h"
#include "usb.h"

enum {
    XTCK_PIN      = PA0,
    XTDI_PIN      = PA1,
    XTDO_PIN      = PA2,
    XTMS_PIN      = PA3,
    USART1_TX_PIN = PA9,
    USART1_RX_PIN = PA10,
    LED0_PIN      = PC13,
};

/* clang-format off */
static struct gpio_config_t {
    enum GPIO_Pin  pins;
    enum GPIO_Conf mode;
} pin_cfgs[] = {
    {PAAll, Mode_INA}, // reset
//  {PBAll, Mode_INA}, // reset
    {PCAll, Mode_INA}, // reset
    {XTCK_PIN, Mode_AF_PP_50MHz}, // TIM2 Ch1
    {XTDI_PIN|XTMS_PIN, Mode_Out_PP_50MHz}, // gpio out bitbang
    {XTDO_PIN, Mode_IN },  // gpio in bitbang
    {USART1_TX_PIN, Mode_AF_PP_50MHz},
//  {USART1_RX_PIN, Mode_IPU},
    {LED0_PIN, Mode_Out_OD_2MHz},
    {0, 0}, // sentinel
};
/* clang-format on */

static inline void led0_on(void) { digitalLo(LED0_PIN); }
static inline void led0_off(void) { digitalHi(LED0_PIN); }
static inline void led0_toggle(void) { digitalToggle(LED0_PIN); }

/* clang-format off */
enum { IRQ_PRIORITY_GROUPING = 5 }; // prio[7:6] : 4 groups,  prio[5:4] : 4 subgroups
struct {
    enum IRQn_Type irq;
    uint8_t        group, sub;
} irqprios[] = {
    {SysTick_IRQn,          0, 0},
    {TIM2_IRQn,             1, 0},
//  {USB_LP_CAN1_RX0_IRQn,  2, 0},
    {USART1_IRQn,           3, 0},
    {None_IRQn, 0xff, 0xff},
};
/* clang-format on */
static struct Ringbuffer usart1tx;

void          USART1_IRQ_Handler(void) { usart_irq_handler(&USART1, &usart1tx); }
static size_t u1puts(const char* buf, size_t len) { return usart_puts(&USART1, &usart1tx, buf, len); }
// void USB_LP_CAN1_RX0_IRQ_Handler(void) {};

static uint8_t usbrxbuf[64];
static size_t  usbrxhead = 0;
static size_t  usbrxtail = 0;

static uint8_t getchar(void) {
    if (usbrxtail == usbrxhead) {
        usbrxtail = 0;
        usbrxhead = 0;
        while (usbrxhead == 0) {
            usbrxhead = usb_recv(usbrxbuf, sizeof usbrxbuf);
        }
    }
    return usbrxbuf[usbrxtail++];
}

int mcmp(const uint8_t* a, const uint8_t* b, size_t len) {
    for (size_t i = 0; i < len; i++)
        if (a[i] != b[i])
            return (a[i] < b[i]) ? -1 : 1;
    return 0;
}

enum tok_t { TOK_NONE, TOK_GETINFO, TOK_SETTCK, TOK_SHIFT };
enum tok_t getcmd(void) {
    uint8_t cmd[8];
    size_t  i;
    for (i = 0; i < sizeof cmd; ++i) {
        cmd[i] = getchar();
        if (cmd[i] == ':')
            break;
    }
    if (i == sizeof cmd) {
        cbprintf(u1puts, "discarded %d characters\n", i);
        return TOK_NONE;
    }
    if (mcmp(cmd, "getinfo:", 8) == 0)
        return TOK_GETINFO;
    if (mcmp(cmd, "settck:", 7) == 0)
        return TOK_SETTCK;
    if (mcmp(cmd, "shift:", 6) == 0)
        return TOK_SHIFT;

    cbprintf(u1puts, "discarded unknown command '%*s'\n", i, cmd);
    return TOK_NONE;
}

uint32_t getuint32() {
    uint32_t r = getchar();
    r |= ((uint32_t)getchar()) << 8;
    r |= ((uint32_t)getchar()) << 16;
    r |= ((uint32_t)getchar()) << 24;
    return r;
}

static uint8_t tms_vector[1024];
static uint8_t tdx_vector[1024]; // tdi out, tdo in
static size_t  num_bits    = 0;
static size_t  shift_count = 0;
static int     last_tdo    = 0;

// TIM2 shifts out the bitbangs.  CH1 is the TCK clock
void TIM2_IRQ_Handler(void) {
    if ((TIM2.SR & TIM_SR_UIF) == 0)
        return;
    TIM2.SR &= ~TIM_SR_UIF;

    last_tdo = digitalIn(XTDO_PIN);

    if (shift_count < num_bits) {
        size_t  idx = shift_count >> 3;
        uint8_t msk = 1U << (shift_count & 0x7);

        enum GPIO_Pin val = 0;

        if (tms_vector[idx] & msk)
            val |= XTMS_PIN;

        if (tdx_vector[idx] & msk)
            val |= XTDI_PIN;

        digitalSet(XTMS_PIN|XTDI_PIN, val);
    } else {
        TIM2.CR1 &= ~TIM_CR1_CEN;
    }

    if (shift_count) {
        if (last_tdo)
            tdx_vector[(shift_count - 1) >> 3] |= (1U << (shift_count - 1) & 7);
        else
            tdx_vector[(shift_count - 1) >> 3] &= ~(1U << (shift_count - 1) & 7);
    }

    ++shift_count;
}

// response to a getinfo: request. sizeof(tms_vector) + sizeof(tdx_vector)
static const char xvcInfo[] = "xvcServer_v1.0:2048\n";

int main(void) {

    uint8_t rf = (RCC.CSR >> 24) & 0xfc;
    RCC.CSR |= RCC_CSR_RMVF; // Set RMVF bit to clear the reset flags

    SysTick_Config(1U << 24); // tick at 72Mhz/2^24 = 4.2915 HZ

    NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING);
    for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
        NVIC_SetPriority(irqprios[i].irq, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, irqprios[i].group, irqprios[i].sub));
    }

    RCC.APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;
    RCC.APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_USBEN;
    delay(10); // let all clocks and peripherals start up

    for (const struct gpio_config_t* p = pin_cfgs; p->pins; ++p) {
        gpioConfig(p->pins, p->mode);
    }

    gpioLock(PAAll);
    gpioLock(PCAll);

    led0_off();

    usart_init(&USART1, 921600);

    cbprintf(u1puts, "SWREV:%s\n", __REVISION__);
    cbprintf(u1puts, "CPUID:%08lx\n", SCB.CPUID);
    cbprintf(u1puts, "DEVID:%08lx:%08lx:%08lx\n", UNIQUE_DEVICE_ID[2], UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
    cbprintf(u1puts, "RESET:%02x%s%s%s%s%s%s\n", rf, rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "", rf & 0x20 ? " IWDG" : "",
             rf & 0x10 ? " SFT" : "", rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");
    usart_wait(&USART1);

    // TIM2 ticks at the bitbang frequency and toggles TCK (CH1)
    // mid period
    // default jtag period is 1000ns (1MHz)
    TIM2.DIER |= TIM_DIER_UIE;
    TIM2.CCMR1 = 0b110 << 4; // PWM mode 1
    TIM2.CCER |= TIM_CCER_CC1E;
    TIM2.PSC  = 0;      // 72MHz,
    TIM2.ARR  = 72 - 1; //  1MHz
    TIM2.CCR1 = ((TIM2.ARR + 1) / 2) - 1;
    NVIC_EnableIRQ(TIM2_IRQn);

    usb_init();

    // TODO: a protocol error from the client side should be handled by resetting the usb
    // stack, forcing the /dev/ttyUSBx to disappear. could also reset the whole microcontroller.

    for (;;) {
        enum tok_t tok = getcmd();
        switch (tok) {
        case TOK_NONE:
            // should result in usb reset
            continue;

        case TOK_GETINFO:
            cbprintf(u1puts, "getinfo: -> %s", xvcInfo);
            for (;;)
                if (usb_send(xvcInfo, sizeof xvcInfo) != 0)
                    break;
            continue;

        case TOK_SETTCK: {
            uint64_t period = getuint32(); // desired TCK period in nanoseconds
            cbprintf(u1puts, "settck:%lld ->", period);
            // set TIM2 PSC and ARR
            if ((period > 100) && (period < 10000)) { // TODO sane limits
                TIM2.ARR  = (36 * period / 1000) - 1;
                TIM2.CCR1 = ((TIM2.ARR + 1) / 2) - 1;
            }
            // compute actual period set
            period = (TIM2.PSC + 1) * (TIM2.ARR + 1);
            period *= 1000;
            period /= 72;
            cbprintf(u1puts, " %lld [ns] %d * %d\n", period, (TIM2.PSC + 1), (TIM2.ARR + 1));
            uint8_t buf[4] = {period, period >> 8, period >> 16, period >> 24};
            for (;;)
                if (usb_send(buf, sizeof buf) != 0)
                    break;
            continue;
        }

        case TOK_SHIFT: {
            num_bits = getuint32();
            cbprintf(u1puts, "shift:%d bits ....", num_bits);
            size_t num_bytes = (num_bits + 7) / 8;
            if (num_bytes >= 1024) {
                cbprintf(u1puts, "too large. discarding.\n");
                for (size_t i = 0; i < num_bytes; ++i)
                    getchar();
                // should result in usb reset
                continue;
            }
            for (size_t i = 0; i < num_bytes; ++i) {
                tms_vector[i] = getchar();
            }
            for (size_t i = 0; i < num_bytes; ++i) {
                tdx_vector[i] = getchar();
            }

            shift_count = 0;
            TIM2.CR1 |= TIM_CR1_CEN;
            while (TIM2.CR1 & TIM_CR1_CEN)
                __WFI();

            usb_send(tdx_vector, num_bytes);
        }
        }
    } // forever

    return 0;
}
