
static uint8 adcRef;

void HalAdcInit (void)
{
  /*設置ad轉換的參考電壓
  這裡使用了宏定義，其實最終設置的是
  ADCCON3.EREF  ADCCON3寄存器的高兩位：擦考電壓選擇
  00:選擇內部參考電壓
  01:AIN7腳的電壓作為參考電壓
  10:AVDD5腳的電壓作為參考電壓
  11:AIN6、7的差分電壓作為參考電壓
  */
  adcRef = HAL_ADC_REF_VOLT;

}

uint16 HalAdcRead (uint8 channel, uint8 resolution)
{
  int16  reading = 0;


  uint8   i, resbits;
  uint8  adcChannel = 1;

  if (channel < 8)
  {
    for (i=0; i < channel; i++)
    {
      adcChannel <<= 1;
    }
  }

  /* Enable channel */
  /*
  配置相應通道所用到的模擬引腳，使能該引腳為模擬引腳，注意，該操作並不是必須的。
  TI的datasheet中是找不到ADCCFG這個寄存器，其實它是個別名，
  真正的名字叫：APCFG，Analog peripheral I/O configuration
  */
  ADCCFG |= adcChannel;

  /* Convert resolution to decimation rate */
  /*採樣率設置，這裡要注意了
  注意這裡：採樣率的設置影響最後adc值的處理，
  ADCH/L中的值最後要截取有效的MSBs，就是高多少位！
  比如設置的64位採樣率，即8位分辨率，ADC最中的結果就要捨棄ADCL，只保留ADCH（即整個ADC右移8位）
  如果是最高512採樣率，最終的ADC值要捨棄ADCL的低兩位（即整個ADC值右移2位）
  */
  switch (resolution)
  {
    case HAL_ADC_RESOLUTION_8:
      resbits = HAL_ADC_DEC_064;
      break;
    case HAL_ADC_RESOLUTION_10:
      resbits = HAL_ADC_DEC_128;
      break;
    case HAL_ADC_RESOLUTION_12:
      resbits = HAL_ADC_DEC_256;
      break;
    case HAL_ADC_RESOLUTION_14:
    default:
      resbits = HAL_ADC_DEC_512;
      break;
  }

  /* writing to this register starts the extra conversion */
  /*參見上述說明，設置採樣通道，採樣率以及基準電壓*/
  ADCCON3 = channel | resbits | adcRef;

  /* Wait for the conversion to be done */
  /*等待採樣結束，標誌位為ADCCON1的最高位EOC
	當轉換結束後EOC置1，while跳出循環
  */
  while (!(ADCCON1 & HAL_ADC_EOC));

  /* Disable channel after done conversion */
  /*轉換後取消採樣通道的模擬輸入設置*/
  ADCCFG &= (adcChannel ^ 0xFF);

  /* Read the result */
  /*保存ADC值，ADCH左移8+ADCL*/
  reading = (int16) (ADCL);
  reading |= (int16) (ADCH << 8);

  /* Treat small negative as 0 */
  if (reading < 0)
    reading = 0;

  /*參見上述說明*/
  switch (resolution)
  {
    case HAL_ADC_RESOLUTION_8:
	  /*64位採樣率，ADC的高8位為有效值，因此reading右移8位*/
      reading >>= 8;
      break;
    case HAL_ADC_RESOLUTION_10:
	  /*128位採樣率，ADC的高10位為有效值，因此reading右移6位*/
      reading >>= 6;
      break;
    case HAL_ADC_RESOLUTION_12:
	  /*256位採樣率，ADC的高12位為有效值，因此reading右移4位*/
      reading >>= 4;
      break;
    case HAL_ADC_RESOLUTION_14:
    default:
	  /*512位採樣率，ADC的高14位為有效值，因此reading右移2位*/
      reading >>= 2;
    break;
  }

  return ((uint16)reading);
}

void HalAdcSetReference ( uint8 reference )
{

  adcRef = reference;
}

/*採樣電壓，判斷供電是否已穩定*/
bool HalAdcCheckVdd(uint8 vdd)
{
  ADCCON3 = 0x0F;
  while (!(ADCCON1 & 0x80));
  return (ADCH > vdd);
}
