// ADC functions on STM32
//
// Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_from_us
#include "command.h" // shutdown
#include "compiler.h" // ARRAY_SIZE
#include "generic/armcm_timer.h" // udelay
#include "gpio.h" // gpio_adc_setup
#include "internal.h" // GPIO
#include "sched.h" // sched_shutdown
#include "n32g45x_adc.h" // ADC

DECL_CONSTANT("ADC_MAX", 4095);

#define ADC_TEMPERATURE_PIN 0xfe
DECL_ENUMERATION("pin", "ADC_TEMPERATURE", ADC_TEMPERATURE_PIN);

static const uint8_t adc_pins[] = {
    /* ADC1 */
    0, GPIO('A', 0), GPIO('A', 1), GPIO('A', 6),
    GPIO('A', 3), GPIO('F', 4), 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    ADC_TEMPERATURE_PIN, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    /* ADC2 */
    0, GPIO('A', 4), GPIO('A', 5), GPIO('B', 1),
    GPIO('A', 7), GPIO('C', 4), GPIO('C', 0), GPIO('C', 1),
    GPIO('C', 2), GPIO('C', 3), GPIO('F', 2), GPIO('A', 2),
    GPIO('C', 5), GPIO('B', 2), 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    // fixme: n32g455 only
    /* ADC3 */
    0, GPIO('B', 11), GPIO('E', 9), GPIO('E', 13),
    GPIO('E', 12), GPIO('B', 13), GPIO('E', 8), GPIO('D', 10),
    GPIO('D', 11), GPIO('D', 12), GPIO('D', 13), GPIO('D', 14),
    GPIO('B', 0), GPIO('E', 7), GPIO('E', 10), GPIO('E', 11),
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    /* ADC4 */
    0, GPIO('E', 14), GPIO('E', 15), GPIO('B', 12),
    GPIO('B', 14), GPIO('B', 15), 0, 0,
    0, 0, 0, 0,
    GPIO('D', 8), GPIO('D', 9), 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
};

// Perform calibration
static void
adc_calibrate(ADC_Module *adc)
{
    adc->CTRL2 = CTRL2_AD_ON_SET;
    while (!(adc->CTRL3 & ADC_FLAG_RDY))
        ;
    adc->CTRL3 &= (~ADC_CTRL3_BPCAL_MSK);
    udelay(10);
    adc->CTRL2 = CTRL2_AD_ON_SET | CTRL2_CAL_SET;
    while (adc->CTRL2 & CTRL2_CAL_SET)
        ;
}

struct gpio_adc
gpio_adc_setup(uint32_t pin)
{
    // Find pin in adc_pins table
    int chan;
    for (chan=0; ; chan++) {
        if (chan >= ARRAY_SIZE(adc_pins))
            shutdown("Not a valid ADC pin");
        if (adc_pins[chan] == pin)
            break;
    }

    // Determine which ADC block to use
    ADC_Module *adc;
    if ((chan >> 5) == 0)
        adc = NS_ADC1;
    if ((chan >> 5) == 1)
        adc = NS_ADC2;
    if ((chan >> 5) == 2)
        adc = NS_ADC3;
    if ((chan >> 5) == 3)
        adc = NS_ADC4;
    chan &= 0x1F;

    // Enable the ADC
    uint32_t reg_temp;
    reg_temp = ADC_RCC_AHBPCLKEN;
    reg_temp |= (RCC_AHB_PERIPH_ADC1 | RCC_AHB_PERIPH_ADC2 |
                RCC_AHB_PERIPH_ADC3 | RCC_AHB_PERIPH_ADC4);
    ADC_RCC_AHBPCLKEN = reg_temp;

    reg_temp = ADC_RCC_CFG2;
    reg_temp &= CFG2_ADCPLLPRES_RESET_MASK;
    reg_temp |= RCC_ADCPLLCLK_DIV16;
    ADC_RCC_CFG2 = reg_temp;

    adc_calibrate(adc);

    ADC_InitType ADC_InitStructure;
    ADC_InitStructure.WorkMode       = ADC_WORKMODE_INDEPENDENT;
    ADC_InitStructure.MultiChEn      = 0;
    ADC_InitStructure.ContinueConvEn = 0;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIGCONV_NONE;
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber      = 1;
    ADC_Init(adc, &ADC_InitStructure);

    if (pin == ADC_TEMPERATURE_PIN) {
        NS_ADC1->CTRL2 |= CTRL2_TSVREFE_SET;
        VREF1P2_CTRL |= (1<<10);
    } else {
        gpio_peripheral(pin, GPIO_ANALOG, 0);
    }

    return (struct gpio_adc){ .adc = adc, .chan = chan };
}

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
uint32_t
gpio_adc_sample(struct gpio_adc g)
{
    ADC_Module *adc = g.adc;
    uint32_t sr = adc->STS;
    if (sr & ADC_STS_STR) {
        if (!(sr & ADC_STS_ENDC))
            // Conversion still in progress or busy on another channel
            goto need_delay;
        // Conversion ready
        return 0;
    }
    ADC_ConfigRegularChannel(adc, g.chan, 1, ADC_SAMP_TIME_55CYCLES5);
    adc->CTRL2 |= CTRL2_AD_ON_SET;
    adc->CTRL2 |= CTRL2_EXT_TRIG_SWSTART_SET;

need_delay:
    return timer_from_us(20);
}

// Read a value; use only after gpio_adc_sample() returns zero
uint16_t
gpio_adc_read(struct gpio_adc g)
{
    ADC_Module *adc = g.adc;
    uint16_t result = adc->DAT;
    adc->STS &= ~ADC_STS_STR;
    adc->CTRL2 &= CTRL2_EXT_TRIG_SWSTART_RESET;
    return result;
}

// Cancel a sample that may have been started with gpio_adc_sample()
void
gpio_adc_cancel_sample(struct gpio_adc g)
{
    ADC_Module *adc = g.adc;
    irqstatus_t flag = irq_save();
    if (adc->STS & ADC_STS_STR)
        gpio_adc_read(g);
    irq_restore(flag);
}