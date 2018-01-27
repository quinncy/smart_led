
#include "smoke_fog.h"

void Smoke_Fog_Init(void)
{
		SMOKE_FOG_INIT();
}

u8 Smoke_Fog_MeasureOnce(void)
{
		u8 value;
		value = GPIO_ReadInputDataBit(SMOKE_FOG_PORT, GPIO_Pin_SMOKE_FOG_DAT);
	
		return value;
}
