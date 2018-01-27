
#include "tosic_gas.h"

void Tosic_Gas_Init(void)
{
		TOSIC_GAS_INIT();
}

u8 Tosic_Gas_MeasureOnce(void)
{
		u8 value;
		value = GPIO_ReadInputDataBit(TOSIC_GAS_PORT, GPIO_Pin_TOSIC_GAS_DAT);
	
		return value;
}
