/*
 * Swamp - cooperative multitasking operating system
 * Copyright (c) 2016 rksdna
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stm32/syscfg.h>
#include <stm32/usart.h>
#include <stm32/gpio.h>
#include <stm32/exti.h>
#include <stm32/nvic.h>
#include <stm32/rcc.h>
#include <stm32/tim.h>
#include <threads.h>
#include <timers.h>
#include <debug.h>
#include <tools.h>
#include <hid.h>

void debug_put(void *data, char value)
{
    while (~USART2->ISR & USART_ISR_TXE)
        continue;

    USART2->TDR = value;
}

struct stream debug_stream = {debug_put, 0};
static u32_t event;
static u32_t data;
struct hid_report report;

void irq6_handler(void)
{
    u32_t count = TIM14->CNT;
    TIM14->EGR = TIM_EGR_UG;

    switch (event)
    {
    case 0:
        event = 1;
        break;

    case 1:
        if ((count > 1350 - 10) && (count < 1350 + 10))
        {
            event = 2;
            break;
        }
        if ((count > 1125 - 10) && (count < 1125 + 10))
        {
            event = 34;
            break;
        }
        event = 0;
        break;

    default:
        if (event < 34)
        {
            if ((count > 225 - 10) && (count < 225 + 10))
            {
                data = data << 1 | 1;
                event++;
                break;
            }
            if ((count > 112 - 10) && (count < 112 + 10))
            {
                data = data << 1;
                event++;
                break;
            }
            event = 0;
        }
    }

    EXTI->PR = EXTI_PR_PR3;
}


static u32_t data_ready(void)
{
    return event >= 34;
}

void main(void)
{

    RCC->CR = RCC_CR_HSEON | RCC_CR_HSION;
    wait_status(&RCC->CR, RCC_CR_HSERDY);

    RCC->CFGR = RCC_CFGR_PLLMUL12 | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC_HSE_PREDIV;
    RCC->CR = RCC_CR_HSEON | RCC_CR_HSION | RCC_CR_PLLON;
    wait_status(&RCC->CR, RCC_CR_PLLRDY);

    RCC->CFGR = RCC_CFGR_PLLMUL12 | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_SW_PLL;
    RCC->CFGR3 = 0;

    RCC->AHBENR = RCC_AHBENR_SRAMEN | RCC_AHBENR_FLITFEN | RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR = RCC_APB1ENR_USART2EN | RCC_APB1ENR_USBEN | RCC_APB1ENR_TIM14EN;
    RCC->APB2ENR =  RCC_APB2ENR_SYSCFGCOMPEN;

    SYSCFG->CFGR1 = SYSCFG_CFGR1_PA11_PA12_RMP;
    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI3_PB;

    EXTI->IMR = EXTI_IMR_MR3;
    EXTI->FTSR = EXTI_FTSR_TR3;
    NVIC->ISER = NVIC_ISER_SETENA_6;

    GPIOA->MODER = GPIO_MODER_MODER0_0 | GPIO_MODER_MODER14_1;
    GPIOA->PUPDR = 0;
    GPIOA->AFRH = (1 << 6 * 4);

    GPIOB->PUPDR = GPIO_PUPDR_PUPDR3_0;

    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;
    USART2->CR2 = 0;
    USART2->BRR = 48000000 / 115200 + 1;

    TIM14->ARR = 65535;
    TIM14->PSC = 480 - 1;
    TIM14->CR1 = TIM_CR1_CEN;

    start_timers_clock(48000);

    debug("hello\n");


    start_hid_service();
    while (1)
    {
        wait_signal(has_hid_connection);
        debug("connected\n");
        while (has_hid_connection())
        {
            wait_signal(data_ready);
            debug("%8x", data);
            switch ((data >> 8) & 0xFF)
            {
            case 0x78:
                debug("+");
                report.keys[0] = 0x80;
                write_hid_report(&report);
                sleep(10);
                report.keys[0] = 0x00;
                write_hid_report(&report);
                break;

            case 0x50:
                debug("-");
                report.keys[0] = 0x81;
                write_hid_report(&report);
                sleep(10);
                report.keys[0] = 0x00;
                write_hid_report(&report);
                break;

            default:
                break;
            }

            event = 0;
        }
        debug("disconnected\n");
    }
}
