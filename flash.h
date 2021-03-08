#include "stm32f10x.h"

#define MY_FLASH_PAGE_ADDR 0x800FC00

//#define SETTINGS_WORDS sizeof(settings)/4

typedef	struct
  {
	uint16_t Hz;	
	uint16_t StartDelim;		
	uint16_t StopDelim;	
	uint8_t StartMode;	
	uint8_t SpinMode;
	uint16_t MotorAmperage;
	uint16_t Reserved;
//							char Parameter;		// 1 byte
//							uint8_t Parameter;		// 1 byte
//							uint16_t Parameter;	// 2 byte
//							uint32_t Parameter;	// 4 byte
							// 8 byte = 2  32-bits words.  It`s - OK
							// !!! Full size (bytes) must be a multiple of 4 !!!
  } tpSettings;