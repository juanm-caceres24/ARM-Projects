/*
 * Copyright 2022 NXP
 * NXP confidential.
 * This software is owned or controlled by NXP and may only be used strictly
 * in accordance with the applicable license terms.  By expressly accepting
 * such terms or by downloading, installing, activating and/or otherwise using
 * the software, you are agreeing that you have read, and that you agree to
 * comply with and are bound by, such license terms.  If you do not agree to
 * be bound by the applicable license terms, then you may not retain, install,
 * activate or otherwise use the software.
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"

#include <cr_section_macros.h>

#define ADC_PIN 23 // P0.25
#define DAC_PIN 26 // P0.26

#define ADC_CONVERSION_RATE 200000

#define GPDMA_CHANNEL_0 0
#define GPDMA_BUFFER_SIZE 16

uint32_t static average_value;
uint32_t static list_selection = 0;
GPDMA_LLI_Type static adc_lli_0;
GPDMA_LLI_Type static adc_lli_1;
uint32_t static adc_buffer_0[GPDMA_BUFFER_SIZE] = {1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023};
uint32_t static adc_buffer_1[GPDMA_BUFFER_SIZE] = {1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023};

void configADC();
void configDAC();
void configGPDMA();
void configNVIC();
void calculateAverage();
void updateDAC();
void toggleListSelection();

int main() {
	SystemInit();
	configADC();
	configDAC();
	configGPDMA();
	configNVIC();
	while (1) {
		updateDAC();
	}
	return 0;
}

/*
 * CONFIGURATION METHODS
 */

void configADC() {
	PINSEL_CFG_Type PinCfg;
	// P0.23 as ADC
	PinCfg.Portnum = PINSEL_PORT_0;
	PinCfg.Pinnum = ADC_PIN;
	PinCfg.Funcnum = PINSEL_FUNC_1;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&PinCfg);

	//GPIO_SetDir(0, SENSOR_PIN, 0);
	ADC_Init(LPC_ADC, ADC_CONVERSION_RATE);
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
	ADC_BurstCmd(LPC_ADC, ENABLE);
}

void configDAC() {
	PINSEL_CFG_Type PinCfg;
	// P0.26 as DAC
	PinCfg.Portnum = PINSEL_PORT_0;
	PinCfg.Pinnum  = DAC_PIN;
	PinCfg.Funcnum = PINSEL_FUNC_2;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&PinCfg);
	//GPIO_SetDir(0, DAC_PIN, 1);
	DAC_Init(LPC_DAC);
	//DAC_CONVERTER_CFG_Type configDAC;
	//configDAC.DBLBUF_ENA = RESET;
	//configDAC.CNT_ENA = SET;
	//configDAC.DMA_ENA = RESET;
}

void configGPDMA() {
	GPDMA_Init();

	GPDMA_Channel_CFG_Type gpdma_cfg;
	gpdma_cfg.ChannelNum = GPDMA_CHANNEL_0;
	gpdma_cfg.TransferSize = GPDMA_BUFFER_SIZE;
	gpdma_cfg.TransferWidth = GPDMA_WIDTH_WORD;
	gpdma_cfg.SrcMemAddr = 0;
	gpdma_cfg.DstMemAddr = (uint32_t)&adc_buffer_0;
	gpdma_cfg.TransferType = GPDMA_TRANSFERTYPE_P2M;
	gpdma_cfg.SrcConn = GPDMA_CONN_ADC;
	gpdma_cfg.DstConn = 0;
	gpdma_cfg.DMALLI = (uint32_t)&adc_lli_0;
	GPDMA_Setup(&gpdma_cfg);

	adc_lli_0.SrcAddr = LPC_ADC->ADDR0;
	adc_lli_0.DstAddr = (uint32_t)&adc_buffer_0[0];
	adc_lli_0.NextLLI = (uint32_t)&adc_lli_1;
	adc_lli_0.Control = (GPDMA_BUFFER_SIZE & 0xfff) | (2<<18) | (2<<21) | (1<<25) | (1<<27);

	adc_lli_1.SrcAddr = LPC_ADC->ADDR0;
	adc_lli_1.DstAddr = (uint32_t)&adc_buffer_1[0];
	adc_lli_1.NextLLI = (uint32_t)&adc_lli_0;
	adc_lli_1.Control = (GPDMA_BUFFER_SIZE & 0xfff) | (2<<18) | (2<<21) | (1<<25) | (1<<27);

	GPDMA_ChannelCmd(GPDMA_CHANNEL_0, ENABLE);
}

void configNVIC() {
    NVIC_EnableIRQ(DMA_IRQn);
}

/*
 * INTERRUPTION HANDLERS
 */

void DMA_IRQHandler() {
	GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, GPDMA_CHANNEL_0);
	calculateAverage();
	toggleListSelection();
}

/*
 * GENERAL METHODS
 */

void calculateAverage() {
	average_value = 0;
	if (list_selection == 0) {
		for (int i = 0; i < GPDMA_BUFFER_SIZE; i++) {
			average_value += adc_buffer_0[i];
		}
	} else {
		for (int i = 0; i < GPDMA_BUFFER_SIZE; i++) {
			average_value += adc_buffer_1[i];
		}
	}
	average_value /= GPDMA_BUFFER_SIZE;
	average_value >>= 2;
}

void updateDAC() {
	DAC_UpdateValue(LPC_DAC, average_value);
}

void toggleListSelection() {
	if (list_selection == 0) {
		list_selection = 1;
	} else {
		list_selection = 0;
	}
}
