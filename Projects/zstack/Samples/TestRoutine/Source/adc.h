#idndef adcH
#define adcH

void HalAdcInit (void);
uint16 HalAdcRead (uint8 channel, uint8 resolution);
void HalAdcSetReference ( uint8 reference );
bool HalAdcCheckVdd(uint8 vdd);s

#endif