
#include "voice.h"

void Voice_Init(void)
{
		VOICE_INIT();
}

u8 Voice_MeasureOnce(void)
{
		u8 value;
		value = GPIO_ReadInputDataBit(VOICE_PORT, GPIO_Pin_VOICE_DAT);
	
		return value;
}
