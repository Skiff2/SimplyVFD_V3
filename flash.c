#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_flash.h"
#include "flash.h"

// ====================================================
// FLASH Settings struct
// ====================================================

void FLASH_Init(void) {
	/* Next commands may be used in SysClock initialization function
	   In this case using of FLASH_Init is not obligatorily */
	/* Enable Prefetch Buffer */
	FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);
	/* Flash 2 wait state */
	FLASH_SetLatency( FLASH_Latency_2);
}

void FLASH_ReadSettings(tpSettings* settings) {
	//Read settings
	uint32_t *source_addr = (uint32_t *)MY_FLASH_PAGE_ADDR;
	uint32_t *dest_addr = settings;//(void *)&settings;
	for (uint16_t i=0; i < sizeof(*settings)/4; i++) {
		*dest_addr = *(__IO uint32_t*)source_addr;
		source_addr++;
		dest_addr++;
	}
}

void FLASH_WriteSettings(tpSettings* settings) {
	FLASH_Unlock();
	FLASH_ErasePage(MY_FLASH_PAGE_ADDR);

	// Write settings
	uint32_t *source_addr = settings;//(void *)&settings;
	uint32_t *dest_addr = (uint32_t *) MY_FLASH_PAGE_ADDR;
	for (uint16_t i=0; i < sizeof(*settings)/4; i++) {
		FLASH_ProgramWord((uint32_t)dest_addr, *source_addr);
		source_addr++;
		dest_addr++;
	}

	FLASH_Lock();
}
// ====================================================