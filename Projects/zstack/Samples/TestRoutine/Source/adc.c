
static uint8 adcRef;

void HalAdcInit (void)
{
  /*�]�mad�ഫ���Ѧҹq��
  �o�̨ϥΤF���w�q�A���̲׳]�m���O
  ADCCON3.EREF  ADCCON3�H�s���������G���ҹq�����
  00:��ܤ����Ѧҹq��
  01:AIN7�}���q���@���Ѧҹq��
  10:AVDD5�}���q���@���Ѧҹq��
  11:AIN6�B7���t���q���@���Ѧҹq��
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
  �t�m�����q�D�ҥΨ쪺�����޸}�A�ϯ�Ӥ޸}�������޸}�A�`�N�A�Ӿާ@�ä��O�������C
  TI��datasheet���O�䤣��ADCCFG�o�ӱH�s���A��ꥦ�O�ӧO�W�A
  �u�����W�r�s�GAPCFG�AAnalog peripheral I/O configuration
  */
  ADCCFG |= adcChannel;

  /* Convert resolution to decimation rate */
  /*�ļ˲v�]�m�A�o�̭n�`�N�F
  �`�N�o�̡G�ļ˲v���]�m�v�T�̫�adc�Ȫ��B�z�A
  ADCH/L�����ȳ̫�n�I�����Ī�MSBs�A�N�O���h�֦�I
  ��p�]�m��64��ļ˲v�A�Y8�����v�AADC�̤������G�N�n�˱�ADCL�A�u�O�dADCH�]�Y���ADC�k��8��^
  �p�G�O�̰�512�ļ˲v�A�̲ת�ADC�ȭn�˱�ADCL���C���]�Y���ADC�ȥk��2��^
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
  /*�Ѩ��W�z�����A�]�m�ļ˳q�D�A�ļ˲v�H�ΰ�ǹq��*/
  ADCCON3 = channel | resbits | adcRef;

  /* Wait for the conversion to be done */
  /*���ݱļ˵����A�лx�쬰ADCCON1���̰���EOC
	���ഫ������EOC�m1�Awhile���X�`��
  */
  while (!(ADCCON1 & HAL_ADC_EOC));

  /* Disable channel after done conversion */
  /*�ഫ������ļ˳q�D��������J�]�m*/
  ADCCFG &= (adcChannel ^ 0xFF);

  /* Read the result */
  /*�O�sADC�ȡAADCH����8+ADCL*/
  reading = (int16) (ADCL);
  reading |= (int16) (ADCH << 8);

  /* Treat small negative as 0 */
  if (reading < 0)
    reading = 0;

  /*�Ѩ��W�z����*/
  switch (resolution)
  {
    case HAL_ADC_RESOLUTION_8:
	  /*64��ļ˲v�AADC����8�쬰���ĭȡA�]��reading�k��8��*/
      reading >>= 8;
      break;
    case HAL_ADC_RESOLUTION_10:
	  /*128��ļ˲v�AADC����10�쬰���ĭȡA�]��reading�k��6��*/
      reading >>= 6;
      break;
    case HAL_ADC_RESOLUTION_12:
	  /*256��ļ˲v�AADC����12�쬰���ĭȡA�]��reading�k��4��*/
      reading >>= 4;
      break;
    case HAL_ADC_RESOLUTION_14:
    default:
	  /*512��ļ˲v�AADC����14�쬰���ĭȡA�]��reading�k��2��*/
      reading >>= 2;
    break;
  }

  return ((uint16)reading);
}

void HalAdcSetReference ( uint8 reference )
{

  adcRef = reference;
}

/*�ļ˹q���A�P�_�ѹq�O�_�wí�w*/
bool HalAdcCheckVdd(uint8 vdd)
{
  ADCCON3 = 0x0F;
  while (!(ADCCON1 & 0x80));
  return (ADCH > vdd);
}
