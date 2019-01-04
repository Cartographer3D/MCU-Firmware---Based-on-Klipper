#ifndef __LPC176X_INTERNAL_H
#define __LPC176X_INTERNAL_H
// Local definitions for lpc176x code

#define GPIO(PORT, NUM) ((PORT) * 32 + (NUM))
#define GPIO2PORT(PIN) ((PIN) / 32)
#define GPIO2BIT(PIN) (1<<((PIN) % 32))

#define PCLK_TIMER0 1
#define PCLK_UART0 3
#define PCLK_ADC 12
#define PCLK_I2C1 19
#define PCLK_SSP0 21
#define PCLK_USB 31
int is_enabled_pclock(uint32_t pclk);
void enable_pclock(uint32_t pclk);
void gpio_peripheral(uint32_t gpio, int func, int pullup);

#endif // internal.h
