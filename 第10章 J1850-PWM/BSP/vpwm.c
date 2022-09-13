//------------------------------------------------------------------------------
//  Purpose: PWM VPW ���벶�������������ģ��
//  Funtion: �ṩ PWM VPW ���������������벶��ӿ�
//  Dependent: 32F207 LIB
//  Designer: 
//  Date.Ver:
//  Other: ����ʱ��ÿ��ͨ���ⲿ����� VPWM_DATA �������Ԥ����� VPW PWM ���ε�
//         ����ʱ���ܳ�������ռ�ձȣ����ⲿ����� VPWM ����ָ��ʹ�ö�Ӧ��Ӳ����
//         Դ����ò��Ρ�����ʱ��ÿ��ͨ���ⲿ����� VPWM ����ָ��ʹ�ö�Ӧ��Ӳ��
//         ��Դ���������Ⱥ�ռ�ձȲ�˳�򱣴���  VPWM_DATA ���󲢴���
//------------------------------------------------------------------------------

#include "vpwm.h"
#include "bsp.h"
#include "timer.h"
static __IO uint32_t g_iDelayTime;
__IO u8 IFRType=0;//pwm Э��֡��Ӧ�� ��� 0x80=PWM_NOEOF��else 

/************************************************************************
ѭ������У��																														*


*************************************************************************/
uint8_t  CheckCRC (uint8_t *pReqBuff,uint32_t ReqBufLen)
{
	uint32_t crc;
	uint32_t ch[8],ch1;
	uint32_t i,j,k;
	crc = 0xff;
	for (i=0;i<ReqBufLen;i++)
	{
		ch1 = pReqBuff[i];
		for (j=0;j<8;j++)
		{
			ch[j] = ch1 & 0x01;
			ch1 >>= 1;
		}
		for (k=0;k<8;k++)
		{
			ch[7-k] <<=7;
			if ((crc ^ ch[7-k]) & 0x80)
				crc = (crc << 1)^0x1d ;
			else
				crc <<= 1;
		}
	}
	crc &= 0xff;
	crc ^= 0xff;
	return (uint8_t) crc;
}

/******************************
���CRC �ֽ��Ƿ�����

CRC �ֽ���ȷ   ����1
CRC �ֽڲ���ȷ ����0 
******************************/
uint8_t  CheckIfCRC (uint8_t *pReqBuff,uint32_t ReqBufLen)
{
	uint8_t value=0;
	if(ReqBufLen<5) return 0;
	
	//if(pReqBuff[0]==0x68&&pReqBuff[0]==0x6A)
	value=CheckCRC (pReqBuff,ReqBufLen-1);
	if(value==pReqBuff[ReqBufLen-1])
	{
			return 1;
	}
	

	return 0;
}




//------------------------------------------------------------------------------
// Funtion: VPW �ж����߿���״̬
// Input  : vpwm - VPW����
// Output : none
// Return : none
// Info   : none  �ȴ��Ϳ���>300us
//------------------------------------------------------------------------------
uint8_t VPW_CheckBus(void)
{
    uint32_t time;
   // SetDelay( 200000 );  //200ms
    time = 0;
	//if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_1))//�͵�ƽ
		GetTimer6Cnt();//����˼�����
    // �ȴ�һ���Ϳ���
    while(1)
    {
			if(iPartValue!=6) break;
        time = GetTimer6CntEx();
			  if(time>(200*1000)) break;//��ʱ��
				if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_1))
				{
					GetTimer6Cnt();//����˼�����
					 // һֱ�ȴ�һ����
					while( RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_1))//�͵�ƽʱ��
					{
						 time = GetTimer6CntEx();
						 if(time>300) return true;//EOF 
					}
					GetTimer6Cnt();//����˼�����
				}

    }
	
	
    return false;
}

//------------------------------------------------------------------------------
// Funtion: PWM �ж����߿���״̬
// Input  : pwm - PWM����
// Output : none
// Return : none
// Info   : none �ȴ��Ϳ���>96us
//------------------------------------------------------------------------------
uint8_t PWM_CheckBus(void)
{

		    uint32_t time;
   // SetDelay( 200000 );  //200ms
    time = 0;
	//if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_1))//�͵�ƽ
		GetTimer6Cnt();//����˼�����
    // �ȴ�һ���Ϳ���
    while(1)
    {
			if(iPartValue!=7) break;
        time = GetTimer6CntEx();
			  if(time>(200*1000)) break;//��ʱ��
				if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_0))
				{
					GetTimer6Cnt();//����˼�����
					 // һֱ�ȴ�һ����
					while( RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_0))//�͵�ƽʱ��
					{
						 time = GetTimer6CntEx();
						 if(time>96) return true;//EOF  96 us
						//if(time>1000) return true;//EOF  96 us
					}
					GetTimer6Cnt();//����˼�����
				}

    }
	
	
    return false;
}

//------------------------------------------------------------------------------
// Funtion: VPW �ж����߿���״̬
// Input  : vpwm - VPW����
// Output : none
// Return : none
// Info   : none  �ȴ��Ϳ���>300us
//------------------------------------------------------------------------------
//uint8_t VPW_CheckBus( VPWM *vpwm )
//{
//    uint32_t time;
//    SetDelay( 200000 );  //200ms
//    time = 0;
//    // �ȴ�һ���Ϳ���
//    while( DelayState() )
//    {
//        time = GetDelay();
//        // һֱ�ȴ�һ����
//        while( Bit_RESET == GPIO_ReadInputDataBit( vpwm->RxPin.Port, vpwm->RxPin.Pin ) )
//        {
//            if( time - GetDelay() > 300 )  //EOF   
//            {
//                return true;
//            }
//        }
//    }
//    return false;
//}

//------------------------------------------------------------------------------
// Funtion: PWM �ж����߿���״̬
// Input  : pwm - PWM����
// Output : none
// Return : none
// Info   : none �ȴ��Ϳ���>96us
//------------------------------------------------------------------------------
//uint8_t PWM_CheckBus( VPWM *vpwm )
//{
//    uint32_t time;
//    SetDelay( 200000 );  //200ms
//    time = 0;
//    // �ȴ�һ���߿���
//    while( DelayState() )
//    {
//        time = GetDelay();
//        // һֱ�ȴ�һ���߿���
//        while( Bit_RESET == GPIO_ReadInputDataBit( vpwm->RxPin.Port, vpwm->RxPin.Pin ) )
//        {
//            if( time - GetDelay() > 96 )  //EOF
//            {
//                return true;
//            }
//        }
//    }
//    return false;
//}


//------------------------------------------------------------------------------
// Funtion: ����VPWʱ��,ռ�ձ�
// Input  : port VPW��
// Output : none
// Return : none  
// Info   : none    CNTֵ��CCRX ֵ�Ƚ� ,�ı�CCRX ��ֵ ���Ǹı�ߵ͵�ƽ��ʱ��
//------------------------------------------------------------------------------
//uint8_t VPWM_Update( VPWM *vpwm, VPWM_DATA *vdat )
//{
//    if( vdat->Index < vdat->Num )
//    {
//        switch( vpwm->TxPPin.TimCh )
//        {
//            case 1:
//                vpwm->TxPPin.Tim->CCR1 = vdat->Duty[vdat->Index];
//                break;
//            case 2:
//                vpwm->TxPPin.Tim->CCR2 = vdat->Duty[vdat->Index];     
//                break;
//            case 3:
//                vpwm->TxPPin.Tim->CCR3 = vdat->Duty[vdat->Index];     
//                break;
//            case 4:
//                vpwm->TxPPin.Tim->CCR4 = vdat->Duty[vdat->Index];     //ռ�ձ�
//                break;
//            default:
//                return false;
//        }
//        vpwm->TxPPin.Tim->ARR = vdat->Period[vdat->Index];//����  ��װ�Ĵ���
//        vdat->Index++;
//        return true;
//    }
//    return false;
//}


////------------------------------------------------------------------------------
//// Funtion: VPW PWM �������
//// Input  : vpwm Ӳ������
////          vdat ����ʱ�估ռ�ձ�����
//// Output : none
//// Return : none
//// Info   : none
////------------------------------------------------------------------------------
//void VPWM_Output_TEST( VPWM *vpwm, VPWM_DATA *vdat )
//{
//	  TIM_DeInit(vpwm->TxPPin.Tim);
//    static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    static TIM_OCInitTypeDef  TIM_OCInitStructure;
//    /* Compute the prescaler value */
//    uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 10000000) - 1; 
//    
//    /* PWM1 Mode configuration: Channel1 */
//    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//PWM
//	  //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;//����Ƚ�ģʽ
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStructure.TIM_Pulse = 50;
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

//    TIM_OC3Init(vpwm->TxPPin.Tim, &TIM_OCInitStructure);
//    TIM_OC3PreloadConfig(vpwm->TxPPin.Tim, TIM_OCPreload_Enable);

//    TIM_ARRPreloadConfig(vpwm->TxPPin.Tim, ENABLE);
//    
//    /* Time base configuration */
//    TIM_TimeBaseStructure.TIM_Period = 100;////��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
//    //TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;//Ԥ��Ƶ��
//		TIM_TimeBaseStructure.TIM_Prescaler = 84-1;//pwm ��Ƶ��
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInit(vpwm->TxPPin.Tim, &TIM_TimeBaseStructure);
//    
//    /* TIM enable counter */
//    TIM_Cmd(vpwm->TxPPin.Tim, ENABLE);
//}

////------------------------------------------------------------------------------
//// Funtion: VPW PWM �������
//// Input  : vpwm Ӳ������
////          vdat ����ʱ�估ռ�ձ�����
//// Output : none
//// Return : none
//// Info   : none
////------------------------------------------------------------------------------
//void VPWM_Output( VPWM *vpwm, VPWM_DATA *vdat )
//{

//	
//    if( vdat->Num == 0 )
//    {
//        return;
//    }
//    vdat->Index = 0;
////     vdat->State=VPWM_OUTPUT;
//    
//    uint16_t Period = vdat->Period[vdat->Index];
//    uint16_t CCR_Val = vdat->Duty[vdat->Index];
//    vdat->Index++;
//    
//    TIM_DeInit(vpwm->TxPPin.Tim);
//    static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    static TIM_OCInitTypeDef  TIM_OCInitStructure;
//    /* Compute the prescaler value */
//    uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 10000000) - 1; 
//    
//    /* PWM1 Mode configuration: Channel1 */
//    //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//		
//		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStructure.TIM_Pulse = CCR_Val;
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//    
//    switch(vpwm->TxPPin.TimCh)
//    {
//        case 1:
//            TIM_OC1Init(vpwm->TxPPin.Tim, &TIM_OCInitStructure);
//            TIM_OC1PreloadConfig(vpwm->TxPPin.Tim, TIM_OCPreload_Enable);
//            break;
//        case 2:
//            TIM_OC2Init(vpwm->TxPPin.Tim, &TIM_OCInitStructure);
//            TIM_OC2PreloadConfig(vpwm->TxPPin.Tim,TIM_OCPreload_Enable);
//            break;
//        case 3:
//            TIM_OC3Init(vpwm->TxPPin.Tim, &TIM_OCInitStructure);
//            TIM_OC3PreloadConfig(vpwm->TxPPin.Tim, TIM_OCPreload_Enable);
//            break;
//        case 4:
//            TIM_OC4Init(vpwm->TxPPin.Tim, &TIM_OCInitStructure);
//            TIM_OC4PreloadConfig(vpwm->TxPPin.Tim, TIM_OCPreload_Enable);
//            break;
//    }
//    TIM_ARRPreloadConfig(vpwm->TxPPin.Tim, ENABLE);
//    
//    /* Time base configuration */
//    TIM_TimeBaseStructure.TIM_Period = Period;////��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
//   // TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;//Ԥ��Ƶ��
//		TIM_TimeBaseStructure.TIM_Prescaler = 84-1;//Ԥ��Ƶ�� 1us
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInit(vpwm->TxPPin.Tim, &TIM_TimeBaseStructure);
//    
//    /* TIM enable counter */
//    TIM_Cmd(vpwm->TxPPin.Tim, ENABLE);
//    
//    while(vdat->State!=VPWM_IDLE)
//    {
//        if( TIM_GetFlagStatus(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc) == RESET )
//        {
//           continue;
//        }
//        TIM_ClearFlag(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc);
//        if( !VPWM_Update(vpwm,vdat) )
//        {
//            TIM_Cmd(vpwm->TxPPin.Tim, DISABLE);   
//            vdat->State = VPWM_IDLE;            
//        }
//    }
//}

//------------------------------------------------------------------------------
// Funtion: VPW һ�������ƽ��2����λת��
// Input  : lowleveltime �͵�ƽʱ��
//          highleveltime �ߵ�ƽʱ��
// Output : none
// Return : VPWM_BIT11 VPWM_BIT01 VPWM_BIT10 VPWM_BIT00 VPWM_ERR
// Info   : none
//------------------------------------------------------------------------------
uint8_t VPWtoBit( uint16_t highleveltime, uint16_t lowleveltime )
{
#define SHORT_RX_MIN  34
#define SHORT_RX_MAX  97  
#define LONG_RX_MIN   96
#define LONG_RX_MAX   164  
#define SOF_RX_MIN    163
#define SOF_RX_MAX    240  
    if(highleveltime> SHORT_RX_MIN && highleveltime < SHORT_RX_MAX)
    {
        if(lowleveltime<LONG_RX_MAX && lowleveltime> LONG_RX_MIN)
        {
             return VPWM_BIT11;
        }
        else if(lowleveltime >SHORT_RX_MIN && lowleveltime <SHORT_RX_MAX)
        {
             return VPWM_BIT01;
        }
        else
        {
             return VPWM_ERR;
        }
    }    
    if(highleveltime > LONG_RX_MIN && highleveltime <LONG_RX_MAX)
    {
        if(lowleveltime<LONG_RX_MAX && lowleveltime> LONG_RX_MIN)
        {
            return VPWM_BIT10;
        }
        else if(lowleveltime >SHORT_RX_MIN && lowleveltime <SHORT_RX_MAX)
        {
            return VPWM_BIT00;
        }
        else
        {
            return VPWM_ERR;
        }
    }
    if(highleveltime<SOF_RX_MAX && highleveltime> SOF_RX_MIN)
    {
        return VPWM_SOF;    
    }
    return VPWM_ERR;
}

//------------------------------------------------------------------------------
// Funtion: VPW һ�����������ƽ��2����λת��
// Input  : vpwm - VPW��������λ
//          len - buf����󳤶�
// Output : buf - ת���������
// Return : ʵ�����ݳ���
// Info   : none
//------------------------------------------------------------------------------
uint8_t VPWtoDat( VPWM_DATA *vpwm, uint8_t *buf, uint8_t len )
{
    uint8_t j=0,k=0;
    uint8_t ch=0;
    uint8_t state;
    uint32_t i;
    for( i = 0; i < vpwm->Index; i++ )
    {
        state = VPWtoBit( vpwm->HighTime[i], vpwm->LowTime[ i ] );
        if( state & VPWM_SOF )
        {
            ch = 0;
            j = 0;
        }
    
        if( state & VPWM_BIT11 )
        {
            ch |= 1<<(7-j++);
            ch |= 1<<(7-j++);
        }
        else if( state & VPWM_BIT01 )
        {
            j++;
            ch |= 1<<(7-j++);
        }
        else if( state & VPWM_BIT10 )
        {
            ch |= 1<<(7-j++);
            j++;
        }
        else if( state & VPWM_BIT00 )
        {
            j += 2;
        }
        else if( state & VPWM_BIT1 )
        {
            ch |= 1<<(7-j++);
        }
        else if( state & VPWM_BIT0 )
        {
            j++;
        }
        else if( state == VPWM_ERR )
        {
            ch = 0;
            j = 0;
        }
        if( j == 8 )
        {
            if( k >= len )
            {
                return k;
            }
            buf[k++] = ch;
            j = 0;
            ch = 0;
        }
    }
    return k;
}

//------------------------------------------------------------------------------
// Funtion: PWM һ�������ƽ��2����λת��
// Input  : lowleveltime �͵�ƽʱ��
//          highleveltime �ߵ�ƽʱ��
// Output : none
// Return : VPWM_BIT1 VPWM_BIT0 VPWM_ERR
// Info   : none
//------------------------------------------------------------------------------
uint8_t PWMtoBit( unsigned char highleveltime, unsigned char lowleveltime )
{
    if(highleveltime <=35 && highleveltime >=30)
    {
        return VPWM_SOF;
    }
    else if(highleveltime>=6 && highleveltime <=11)
    {
        return VPWM_BIT1;
    }
    else if(highleveltime >=14 && highleveltime <=19)
    {
        return VPWM_BIT0;
    }
    return VPWM_ERR;
}

//------------------------------------------------------------------------------
// Funtion: PWM һ�����������ƽ��2����λת��
// Input  : vpwm - PWM ��������λ
//          len - buf����󳤶�
// Output : buf - ת���������
// Return : ʵ�����ݳ���
// Info   : none
//------------------------------------------------------------------------------
uint8_t PWMtoDat( VPWM_DATA *vpwm, uint8_t *buf, uint8_t len )
{
    uint8_t j=0,k=0;
    uint8_t ch=0;
    uint8_t state;
    uint32_t i;
    for( i = 0; i < vpwm->Index; i++ )
    {
        state = PWMtoBit( vpwm->LowTime[i], vpwm->HighTime[i] );
        if( state & VPWM_BIT1 )
        {
            ch |= 1<<(7-j++);
        }
        else if( state & VPWM_BIT0 )
        {
            j++;
        }
        else if( state == VPWM_ERR )
        {
            ch = 0;
            j = 0;
        }
        if( j == 8 )
        {
            if( k >= len ) return k;
            buf[k++] = ch;
            j = 0;
            ch = 0;
        }
    }
    return k;
}

//------------------------------------------------------------------------------
// Funtion: ��ƽ���� ( ����ĵ�ƽ�������� HighTime LowTime )
// Input  : MaxLenth - �����ճ���
//          us - ���ȴ�ʱ��  
//          buf - ���ջ�����
// Output : DataLenth - ʵ�ʽ��ճ���
// Return : ʵ�ʲ����ʱ
// Info   : none
//------------------------------------------------------------------------------
//uint32_t VPWM_Input( VPWM *vpwm, VPWM_DATA *vdat, uint8_t *buf, 
//                     uint8_t *DataLenth, uint8_t MaxLenth, uint32_t us )
//{
//    TIM_DeInit(vpwm->RxPin.Tim);
//    static TIM_ICInitTypeDef TIM_ICInitStructure;
//    static TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//    //1Mhz
//    uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 1000000) - 1;
//    vdat->Num = ( unsigned int )( ( MaxLenth << 3 ) + 2 );
//    if( vdat->Num > 160 )
//    {
//        vdat->Num = 160;
//    }
//    /* Time base configuration */
//    TIM_TimeBaseStructure.TIM_Period = 10000-1;
//    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInit(vpwm->RxPin.Tim, &TIM_TimeBaseStructure);
//    
//    TIM_ICInitStructure.TIM_Channel =(vpwm->RxPin.TimCh-1)<<2;
//    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
//    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//    TIM_ICInitStructure.TIM_ICFilter = 10;
//#ifndef PWMINPUT     
//    TIM_ICInit(vpwm->RxPin.Tim, &TIM_ICInitStructure);
//    /* ���벶��ģʽ���ã�TIM3��channel3��Ϊ���룬�Ƚ��������ش�����ʱֹͣ��ʱ�ж�*/ 
//#else
//    TIM_PWMIConfig(vpwm->RxPin.Tim, &TIM_ICInitStructure);     
//#endif  
//    /* Select the TIM  Input Trigger */
//    TIM_SelectInputTrigger(vpwm->RxPin.Tim,0x40|(vpwm->RxPin.TimCh-1)<<4);
//     /* Select the slave Mode: Reset Mode */
//    TIM_SelectSlaveMode(vpwm->RxPin.Tim, TIM_SlaveMode_Reset);//TIM��ģʽ�������źŵ����������³�ʼ���������ʹ����Ĵ����ĸ����¼�
//    /* Enable the Master/Slave Mode */
//    TIM_SelectMasterSlaveMode(vpwm->RxPin.Tim, TIM_MasterSlaveMode_Enable); //������ʱ���ı�������
//    /* TIM enable counter */
//    TIM_Cmd(vpwm->RxPin.Tim, ENABLE);
//    /* Enable the CC2 Interrupt Request */

//    uint32_t ustime = us;
//    uint32_t time;
//    __IO uint16_t htime,ltime;
//    unsigned char j = 0x80, k = 0;
//    unsigned char ch = 0;
//    unsigned char state;
//    
//    SetDelay( us );
//    time = GetDelay();
//    while( 1 )
//    {   
//        //�����벶��ģʽ�£�����⵽ICx�ź�����Ӧ�ı��غ󣬼������ĵ�ǰֵ������
//        //������/�ȽϼĴ���(TIMx_CCRx)�С������������¼�ʱ����Ӧ��CCxIF��־
//        //(TIMx_SR�Ĵ���)����1������������жϻ���DMA�������򽫲����жϻ���DMA��
//        //��������������¼�ʱCCxIF��־�Ѿ�Ϊ�ߣ���ô�ظ������־CCxOF
//        //(TIMx_SR�Ĵ���)����1��дCCxIF=0�����CCxIF�����ȡ�洢��TIMx_CCRx
//        //�Ĵ����еĲ�������Ҳ�����CCxIF��дCCxOF=0�����CCxOF�� 
//        //��������˵�������TI1�����������ʱ�����������ֵ��TIMx_CCR1�Ĵ����У�
//        //�������£�
//        //�� ѡ����Ч����ˣ�TIMx_CCR1�������ӵ�TI1���룬
//        //   ����д��TIMx_CCR1�Ĵ����е�CC1S=01��ֻҪCC1S��Ϊ��00����
//        //   ͨ��������Ϊ���룬����TIMx_CCR1�Ĵ�����Ϊֻ����
//        //�� ���������źŵ��ص㣬
//        //   ���������˲���Ϊ����Ĵ���(������ΪTIxʱ�������˲�������λ��TIMx_CCMRx
//        //   �Ĵ����е�ICxFλ)�����������ź������5���ڲ�ʱ�����ڵ�ʱ���ڶ�����
//        //   �����������˲����Ĵ�����5��ʱ�����ڣ�������ǿ���(��fDTSƵ��)������
//        //   ��8�Σ���ȷ����TI1��һ����ʵ�ı��ر任������TIMx_CCMR1�Ĵ�����д��
//        //   IC1F=0011��
//        //�� ѡ��TI1ͨ������Чת�����أ���TIMx_CCER�Ĵ�����д��CC1P=0(������)�� 
//        //�� ��������Ԥ��Ƶ�����ڱ����У�����ϣ����������ÿһ����Ч�ĵ�ƽת��
//        //   ʱ�̣����Ԥ��Ƶ������ֹ(дTIMx_CCMR1�Ĵ�����IC1PS=00)��
//        //�� ����TIMx_CCER�Ĵ�����CC1E=1���������������ֵ������Ĵ����С�
//        //�� �����Ҫ��ͨ������TIMx_DIER�Ĵ����е�CC1IEλ��������ж�����
//        //   ͨ������TIMx_DIER�Ĵ����е�CC1DEλ����DMA���� ������һ�����벶��ʱ��
//        //�� ������Ч�ĵ�ƽת��ʱ����������ֵ�����͵�TIMx_CCR1�Ĵ�
//        if((vpwm->RxPin.Tim->SR & vpwm->RxPin.TimFlagCc) == (uint16_t)RESET)
//        {
//            if( vdat->Index != 0 )
//            {
//                if(vdat->State == PWM_INPUT) 
//                {
//                    if( time - GetDelay() > 32 )
//                    {
//                        break;
//                    }
//                }
//                else//VPW
//                {
//                    uint32_t time1 = GetDelay();
//                    if( time - time1 > 270 )
//                    {
//                        break;
//                    }
//                }
//            }
//        
//            if( !DelayState() )
//            {
//                if( vdat->Index == 0 )
//                {
//                    break;
//                }
//            }
//            continue;
//        }
//        vpwm->RxPin.Tim->SR = (uint16_t)~vpwm->RxPin.TimFlagCc;
//        /*htime = vpwm->RxPin.Tim->CCR1;
//        ltime = vpwm->RxPin.Tim->CCR2;  
//        
//        if(htime==0 && ltime==0)
//        {
//            htime = vpwm->RxPin.Tim->CCR1;
//            ltime = vpwm->RxPin.Tim->CCR2;  
//        }*/
//        
//        /* Get the Input Capture value */
//        vdat->HighTime[vdat->Index] = vpwm->RxPin.Tim->CCR1;
//        vdat->LowTime[vdat->Index] = vpwm->RxPin.Tim->CCR2;
//        
//        if( vdat->State == PWM_INPUT )
//        {
//            state = PWMtoBit( vdat->HighTime[ vdat->Index ], 
//                              vdat->LowTime[ vdat->Index ] );
//            if( state & VPWM_BIT1 )
//            {
//                ch |= j;
//                j = j >> 1;
//            }
//            else if( state & VPWM_BIT0 )
//            {
//                j = j >> 1;
//            }
//            else if( state == VPWM_ERR )
//            {
//                ch = 0;
//                j = 0x80;
//            }
//            if( j == 0 )
//            {
//                buf[ k++ ] = ch;
//                if( k >= MaxLenth )
//                {
//                    break;
//                }
//                j = 0x80;
//                ch = 0;
//            }
//        }
//        else//VPW
//        {
//            state = VPWtoBit( vdat->HighTime[ vdat->Index ], 
//                              vdat->LowTime[ vdat->Index ] );
//            if( state & VPWM_SOF )
//            {
//                ch = 0;
//                j = 0x80;
//            }
//        
//            else if( state & VPWM_BIT11 )
//            {
//                ch |= j;
//                j = j >> 1;
//                ch |= j;
//                j = j >> 1;
//            }
//            else if( state & VPWM_BIT01 )
//            {
//                j = j >> 1;
//                ch |= j;
//                j = j >> 1;
//            }
//            else if( state & VPWM_BIT10 )
//            {
//                ch |= j;
//                j = j >> 2;
//            }
//            else if( state & VPWM_BIT00 )
//            {
//                j = j >> 2;
//            }
//            else if( state & VPWM_BIT1 )
//            {
//                ch |= j;
//                j = j >> 1;
//            }
//            else if( state & VPWM_BIT0 )
//            {
//                j = j >> 1;
//            }
//            else if( state == VPWM_ERR )
//            {
//                ch = 0;
//                j = 0x80;
//            }
//            if( j == 0 )
//            {
//                buf[ k++ ] = ch;
//                if( k >= MaxLenth )
//                {
//                    break;
//                }
//                j = 0x80;
//                ch = 0;
//            }
//        }
//        
//        vdat->Index++;
//        if( vdat->Index >= vdat->Num )
//        {
//            break;
//        }
//        time = GetDelay();
//    }
//    (*DataLenth) = k;
//    TIM_Cmd( vpwm->RxPin.Tim, DISABLE );
//    ustime -= GetDelay();
//    return ustime;
//}





//------------------------------------------------------------------------------
// Funtion: ��ƽת��Ϊʱ���ռ�ձ�
// Input  : time0 - �½���ʱ��
//          time1 - ������ʱ��
// Output : vdat - ת����ʱ���ռ�ձ�
// Return : none
// Info   : none
//------------------------------------------------------------------------------
void ToPWM(VPWM_DATA *vdat, uint16_t time0, uint16_t time1)
{
		//1us ����ʱ
	uint16_t Period = (time0+time1-1);//����
    vdat->Period[vdat->Num] = Period;
	vdat->Duty[vdat->Num++] =(uint32_t)(Period*time0)/(time0+time1);
	
	//1us/10 ����ʱ
//	 uint16_t Period = (10*(time0+time1)-1);//����
//	vdat->Period[vdat->Num] = Period;
//   vdat->Duty[vdat->Num++] = ((uint32_t)(Period*(10*time0+vdat->Offset)))/(10*(time0+time1));            
         
		
    if( vdat->Num > 160 )
    {
        vdat->Num = 160;
    }
}
 
//------------------------------------------------------------------------------
// Funtion: PWM ����ת��Ϊʱ���ռ�ձ�
// Input  : buf - ���������
//          len - buf ��󳤶�
//          datatype - ���ݵĸ�ʽ
// Output : vdat - ת����ʱ���ռ�ձ�
// Return : true
// Info   : none
//------------------------------------------------------------------------------
uint8_t DatToPWM( VPWM_DATA *vdat, uint8_t *buf, uint8_t len, uint8_t datatype )
{
    uint8_t i,j;
//    uint16_t time = 30;//32;//SOF   	������
//    uint16_t time1 = 16;						//�½���
	
	    uint16_t time = 31;//32;//SOF   	������
    uint16_t time1 = 17;						//�½���
	
    vdat->Num=0;
    //SOF
    if( !(PWM_NOSOF & datatype) )//32 16 
    {
        ToPWM(vdat,time,time1);
    }
    
    for( i = 0; i < len; i++ )
    {
        for( j = 0; j < 8; j++ )
        {
            if((buf[i]>>(7-j))&0x1)
            {
                //HIGH
                time = 8;//������
                time1 = 16;//�½���
            }
            else
            {
                //LOW
                time = 16;//������
                time1 = 8;//�½���
            }
						//PWM Э����һ������һ����ƽ
            ToPWM( vdat, time, time1 );   
        }
    }
    //EOF
    if( !(PWM_NOEOF & datatype) )
    {
        time = 16;
        time1 = 32;
        ToPWM( vdat, time, time1 );
    }
    return true;
}

//------------------------------------------------------------------------------
// Funtion: VPW ����ת��Ϊʱ���ռ�ձ�
// Input  : buf - ���������
//          len - buf ��󳤶�
// Output : vdat - ת����ʱ���ռ�ձ�
// Return : true
// Info   : none
//------------------------------------------------------------------------------
uint8_t DatToVPW( VPWM_DATA *vdat, uint8_t *buf, uint8_t len )
{
    uint8_t i, j;
    uint8_t state = true;//HIGH
    uint16_t time = 200;//200;//SOF198;//  
    uint16_t time1;
    vdat->Num = 0;
    for( i = 0; i < len; i++ )
    {
        for( j = 0; j < 8; j++ )
        {
            if( (buf[i]>>(7-j))&0x1)
            {
                //HIGH 
                time1 = (state)?128:64;
            }
            else
            {
                //LOW
                time1 = (state)?64:128;
            }
            state = !state;
            if( !state )//ÿ�ζ�����2����ƽ ����Ҫ��������
            {
							//VPW Э����һ����ƽ��תһ��BIT
                ToPWM( vdat, time, time1 );   //time=������ ,time1=�½���
							
            }
            time = time1;
        }
    }
    if( !state )
    {
        return false;
    }
    time1 = 160;//EOF
    ToPWM( vdat, time, time1 );
    return true;
}


VPWM_DATA PWM_Data = { (VPWM_STATUS)0 };
//------------------------------------------------------------------------------
// Funtion: ��ʼ��PWM����ģ�飨����ϵͳʱ�����ʼ����
// Input  : none
// Output : none
// Return : none
// Info   : none
//------------------------------------------------------------------------------
void PWM_Init()
{

 // VPWM_Init((VPWM*)&PWM);
	// VPWM_Init(&PWM);
}

//------------------------------------------------------------------------------
// ���ܣ�J1850 - PWM ���ݷ��ͺ���
// ���룺chbuf - ����������
//       chlen - ���ͳ���
//       IFRType - �Ƿ�����֡��Ӧ�� true or false
// �����FrameID - ֡��Ӧ�� ID
// ���أ�true ��ʾ���ͳɹ� false ��ʾ����ʧ��
// ���ӣ���
//------------------------------------------------------------------------------
char PWM_Send( unsigned char *chbuf, unsigned char chlen, unsigned char IFRType,
               unsigned char *FrameID )
{
    //uint32_t num=(uint32_t)((chlen<<3)+2+2);  
    //get space
    //uint8_t i = 0;
    
    //data convert
    if( true == IFRType )
    {
        if(!DatToPWM(&PWM_Data,chbuf,chlen,PWM_NOEOF))
        {
            return false;
        }
    }
    else
    {
        if(!DatToPWM(&PWM_Data,chbuf,chlen,0))
        {
            return false;
        }
    }
    //Check BUS
//    if(!PWM_CheckBus((VPWM*)&PWM))
//    {
//        return false;
//    }
    
    //SetDelay(16);//SOF
    //WaitDelay();  
    PWM_Data.State = PWM_OUTPUT;
	//	VPWM_Output_TEST((VPWM*)&PWM,&PWM_Data);
		
//    VPWM_Output((VPWM*)&PWM,&PWM_Data);
//    if( true == IFRType )
//    {
//        PWM_Data.Index = 0;
//        PWM_Data.State = PWM_INPUT;
//        VPWM_Input((VPWM*)&PWM, &PWM_Data, FrameID, &i, 1, 400 );
//        if( PWM_Data.HighTime[ 0 ] > 24 )   // No EOF
//        {
//            return false;
//        }
//    }

    return true;
}

//------------------------------------------------------------------------------
// Funtion: ����PWM����
// Input  : chlen  ���ܳ���
//          ms  ��ʱʱ��  
//          IFRType - �Ƿ�����֡��Ӧ�� true or false
//          FrameID - ֡��Ӧ�� ID
// Output : chbuf  ���ܻ�����
// Return : true/false
// Info   : none
//------------------------------------------------------------------------------
VPWM_DATA PWM_DataRecv = { (VPWM_STATUS)0 };
unsigned long PWM_Receive( unsigned char *chbuf, unsigned char chlen,
                           unsigned int *firstms, unsigned int ms, 
                           unsigned char IFRType, unsigned int FrameID, 
                           unsigned char Devaddr )
{
//    unsigned char id1;
//    unsigned char id2;
//    id1 = ( unsigned char )(FrameID & 0xff);
//    id2 = ( unsigned char )((FrameID & 0xff00)>>8);
    uint8_t num;
 
    //get space
    if(!DatToPWM(&PWM_DataRecv,&Devaddr,1,PWM_ONLYDATA))
    {
        return false;
    }
    (*firstms) = 0;
    
//    while( (*firstms) < ms )
//    {
//        PWM_Data.Index = 0;
//        PWM_Data.State = PWM_INPUT;
//        
//        (*firstms) += VPWM_Input((VPWM*)&PWM,&PWM_Data, chbuf, &num, 
//                                  chlen, ms*1000)/1000;
//        //num = PWMtoDat(&PWM_Data,chbuf,chlen);
//        // ע�⣺֡��Ӧ����Ӧʱ����� < 47us
//        // STM32�������˴��ѵȴ�32US��λ��ʱ��ʵ����Ӧ����ʱ��Ϊ 39US��MCU��������
//        // ʱ��Ϊ 7 US, ����������� MCU ��ע������ʱ�䵼�µ���ʱ
//        if( PWM_Data.Index &&  (true == IFRType) )
//        {
//            if(FrameID)
//            {
//                if( (chbuf[ 0 ] == id2) && (chbuf[ 1 ] == id1) )        
//                {
//                    PWM_DataRecv.State = PWM_OUTPUT;
//                    VPWM_Output((VPWM*)&PWM,&PWM_DataRecv);
//                    break;
//                }
//                else
//                {
//                    num = 0;
//                }
//            }
//            else
//            {
//                break;
//            }
//        }
//    }
    
    return num;
}



//------------------------------------------------------------------------------
// Funtion: ����VPW����
// Input  : chlen  ���ܳ���
//          ms  ��ʱʱ��  
// Output : chbuf  ���ܻ�����
// Return : �������ݳ���
// Info   : none
//------------------------------------------------------------------------------
uint32_t VPW_Receive( unsigned char *chbuf, unsigned char chlen,
                      unsigned int *firstms, unsigned int ms )
{
    uint8_t num;  

//    PWM_Data.Index = 0;
//    PWM_Data.State = VPW_INPUT;
//    
//    *firstms = (unsigned int )(VPWM_Input( (VPWM*)&VPW, &PWM_Data,
//                                           chbuf, &num, chlen, 1000 * ms ) / 1000);

    return num;
}



/* --------------------------����OK------------------------------------------------ */
uint8_t PWM_Test(void)
{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    uint8_t FrameID2;
//    unsigned int MS;
	
	static unsigned char dat[]={0x61, 0x6A, 0xF1, 0x01, 0x00, 0x0A};
	static unsigned char dat1[]={0x61, 0x6A, 0xF1, 0x01, 0x01, 0x0B};
	
//	unsigned char rxdat[11];
	// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	PWM_Init();
//	PWM_Open(PWM_ECUPORT);   
  PWM_Send(dat,sizeof(dat),false,0);
	
	PWM_Send(dat1,sizeof(dat1),false,0);
//	while(1)
//	{
//		   // My_delayms(500);
//        PWM_Send(dat,sizeof(dat),false,0);
//	}
	
	
//	while(1)
//	{
//       PWM_Receive(rxdat, sizeof(rxdat), &MS, 200, true, 0xc4f5, FrameID2);
//        if(rxdat[0] == dat[0])
//         {
//             My_delayus(100);
//             PWM_Send(dat,sizeof(dat),false,0);
//             return 1;	//׼����װ�л�Э����ߣ�������һ��Э��
//        }
//   }
return 1;

}



VPWM_DATA VPM_Data = { (VPWM_STATUS)0 };
//------------------------------------------------------------------------------
// Funtion: ��ʼ��VPW����ģ�飨����ϵͳʱ�����ʼ����
// Input  : none
// Output : none
// Return : none
// Info   : none
//------------------------------------------------------------------------------
void VPW_Init()
{
    //VPWM_Init( (VPWM*)&VPW );
}

 
//------------------------------------------------------------------------------
// ���ܣ�J1850 - VPW ���ݷ��ͺ���
// ���룺chbuf - ����������
//       chlen - ���ͳ���
// �������
// ���أ�true ��ʾ���ͳɹ� false ��ʾ����ʧ��
// ���ӣ���
//------------------------------------------------------------------------------
char VPW_Send( unsigned char *chbuf, unsigned char chlen )
{
    //data convert
    if(!DatToVPW(&PWM_Data,chbuf,chlen)) 
    {
        return false;
    }
    //Check BUS
//    if(!VPW_CheckBus((VPWM*)&VPW)) return false;
//    //VPW Output
//    PWM_Data.State = VPW_OUTPUT;
//    VPWM_Output( (VPWM*)&VPW, &PWM_Data);
    
    return true;
}


///* -------����OK��ͨѶ����------------------------------------------------------------------- */

uint8_t VPW_Test(void)
{
//	uint32_t time;
//	CMPHY_Relay Relaybuf={0};//2012-12-20 Relaybuf init
//	CMPHY_Relay_Init(&Relaybuf);


//	Relaybuf.CommType = COMM_VPW_TYPE; //ͨѶ���ʹ��п�
//	CMPHY_Relay_Reset();
//	CMPHY_Relay_Set(&Relaybuf ); 


	VPW_Init();
//	VPW_Open(VPW_ECUPORT);

	uint8_t dat[]={0x68,0x6A,0xF1,0x01,0x00,0x17};
	uint8_t dat1[]={0x68,0x6A,0xF1,0x01,0x01,0x18};
//	uint8_t rxdat[10] = {0};
	VPW_Send(dat,sizeof(dat));
	VPW_Send(dat1,sizeof(dat1));
//	while(1)
//	{
//		VPW_Receive(rxdat,sizeof(rxdat),&time,200);
//                if(rxdat[0] == dat[0])
//                {
//                    //delay(100);
//                    VPW_Send(dat,sizeof(dat));
//                    return 1;
//                }
//	}

	
	
	return 1;
}





//------------------------------------------------------------------------------
// Funtion: ��ƽ���� ( ����ĵ�ƽ�������� HighTime LowTime )
// Input  : MaxLenth - �����ճ���
//          us - ���ȴ�ʱ��  
//          buf - ���ջ�����
// Output : DataLenth - ʵ�ʽ��ճ���
// Return : ʵ�ʲ����ʱ
// Info   : none
//------------------------------------------------------------------------------
uint32_t VPWM_Input_Init( )
{
//		VPWM *vpwm=&VPW;
//		VPWM_DATA vdat;
//		PWM_Data.Index = 0;
//    PWM_Data.State = VPW_INPUT;

//  //  VPWM_Output( (VPWM*)&VPW, &PWM_Data);
//	
//		
//		uint16_t i=0;
//    TIM_DeInit(vpwm->RxPin.Tim);
//    static TIM_ICInitTypeDef TIM_ICInitStructure;
//    static TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//    //1Mhz
//    uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 1000000) - 1;
////    vdat->Num = 0;//( unsigned int )( ( MaxLenth << 3 ) + 2 );
////    if( vdat->Num > 160 )
////    {
////        vdat->Num = 160;
////    }
//    /* Time base configuration */
//    TIM_TimeBaseStructure.TIM_Period = 10000-1;
//    TIM_TimeBaseStructure.TIM_Prescaler = 84-1;//1US
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInit(vpwm->RxPin.Tim, &TIM_TimeBaseStructure);
//    
//    TIM_ICInitStructure.TIM_Channel =(vpwm->RxPin.TimCh-1)<<2;
//    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
//    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//    TIM_ICInitStructure.TIM_ICFilter = 10;
//if(PWM_Data.State==PWM_INPUT)   //PWM
//    TIM_ICInit(vpwm->RxPin.Tim, &TIM_ICInitStructure);
//    /* ���벶��ģʽ���ã�TIM3��channel3��Ϊ���룬�Ƚ��������ش�����ʱֹͣ��ʱ�ж�*/ 
//else//vpw
//    TIM_PWMIConfig(vpwm->RxPin.Tim, &TIM_ICInitStructure);     
// 
//    /* Select the TIM  Input Trigger */
//    TIM_SelectInputTrigger(vpwm->RxPin.Tim,0x40|(vpwm->RxPin.TimCh-1)<<4);
//     /* Select the slave Mode: Reset Mode */
//    TIM_SelectSlaveMode(vpwm->RxPin.Tim, TIM_SlaveMode_Reset);//TIM��ģʽ�������źŵ����������³�ʼ���������ʹ����Ĵ����ĸ����¼�
//    /* Enable the Master/Slave Mode */
//    TIM_SelectMasterSlaveMode(vpwm->RxPin.Tim, TIM_MasterSlaveMode_Enable); //������ʱ���ı�������


//	NVIC_InitTypeDef NVIC_InitStructure;
//	//�жϷ����ʼ��
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3�ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
//	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 
//	
//	//TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC4,ENABLE);//��������ж� ,����CC1IE�����ж�	
////TIM_ITConfig(TIM2,TIM_IT_CC4,ENABLE);//��������ж� ,����CC1IE�����ж�	

//    /* TIM enable counter */
//    TIM_Cmd(vpwm->RxPin.Tim, ENABLE);
//    /* Enable the CC2 Interrupt Request */
//		
//		while(1)
//		{
//			   if((vpwm->RxPin.Tim->SR & vpwm->RxPin.TimFlagCc) == (uint16_t)RESET)
//        {
//					i++;
//					
//				}
//				vpwm->RxPin.Tim->SR = (uint16_t)~vpwm->RxPin.TimFlagCc;
//		}

		return 0;

}




//msy �Զ���
//TIME5 PA2  TX
void Time5_VPWM_Init(void)
{
//    static GPIO_InitTypeDef GPIO_InitStructure;
//    /* TIM clock enable */
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);        
//    /* GPIO clock enable */
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);     
//		
//    /* GPIO Configuration: TIM CH */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;//����
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);     

//		GPIO_SetBits(GPIOA,GPIO_Pin_2);

//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
//    GPIO_Init(GPIOA, &GPIO_InitStructure);     

//    /* Connect TIM pins to AF */  
//		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM5); //GPIOA4 ����ΪUSART1		
//		GPIO_ResetBits(GPIOA,GPIO_Pin_2);//��ʼ��ƽΪ��
}


//------------------------------------------------------------------------------
// Funtion: ����VPWʱ��,ռ�ձ�
// Input  : port VPW��
// Output : none
// Return : none  
// Info   : none    CNTֵ��CCRX ֵ�Ƚ� ,�ı�CCRX ��ֵ ���Ǹı�ߵ͵�ƽ��ʱ��
//------------------------------------------------------------------------------
//uint8_t VPWM_UpdateEx( TIM_TypeDef *iTimer, VPWM_DATA *vdat )
//{
//    if( vdat->Index < vdat->Num )
//    {//CH3
//        iTimer->CCR3 = vdat->Duty[vdat->Index]; //�ߵ�ƽʱ��     
//        iTimer->ARR = vdat->Period[vdat->Index];//����  ��װ�Ĵ���
//			//�ߵ�ƽʱ��=vdat->Duty
//			//�͵�ƽʱ��=vdat->Period-vdat->Duty;
//			
////			  iTimer->CCR3 = 5+1; //�ߵ�ƽʱ��     
////        iTimer->ARR = 10+1;//����  ��װ�Ĵ���
//			
//			
//        vdat->Index++;
//        return true;
//    }
//    return false;
//}

//------------------------------------------------------------------------------
// Funtion: VPW PWM �������
// Input  : vpwm Ӳ������
//          vdat ����ʱ�估ռ�ձ�����
// Output : none
// Return : none
// Info   : none
//------------------------------------------------------------------------------
void VPWM_OutputEx(VPWM_DATA *vdat )
{

	
//    if( vdat->Num == 0 )
//    {
//        return;
//    }
//    vdat->Index = 0;
////     vdat->State=VPWM_OUTPUT;
//    
//    uint16_t Period = vdat->Period[vdat->Index];
//    uint16_t CCR_Val = vdat->Duty[vdat->Index];
//    vdat->Index++;
//    
//    TIM_DeInit(TIM5);
//    static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    static TIM_OCInitTypeDef  TIM_OCInitStructure;
//    /* Compute the prescaler value */
//    uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 10000000) - 1; 
//    
//    /* PWM1 Mode configuration: Channel1 */
//    //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//		
//		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStructure.TIM_Pulse = CCR_Val;
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//   
//    TIM_OC3Init(TIM5, &TIM_OCInitStructure);
//    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

//    TIM_ARRPreloadConfig(TIM5, ENABLE);
//    
//    /* Time base configuration */
//    TIM_TimeBaseStructure.TIM_Period = Period;////��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
//    //TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;//Ԥ��Ƶ��
//		TIM_TimeBaseStructure.TIM_Prescaler = 84-1;//Ԥ��Ƶ�� 1us
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
//    
//    /* TIM enable counter */
//    TIM_Cmd(TIM5, ENABLE);
//    
//    while(vdat->State!=VPWM_IDLE)
//    {
//        if( TIM_GetFlagStatus(TIM5,TIM_FLAG_CC3) == RESET )
//        {
//           continue;
//        }
//        TIM_ClearFlag(TIM5,TIM_FLAG_CC3);
//        if( !VPWM_UpdateEx(TIM5,vdat) )
//        {
//            TIM_Cmd(TIM5, DISABLE);   
//            vdat->State = VPWM_IDLE;            
//        }
//    }
}


//------------------------------------------------------------------------------
// ���ܣ�J1850 - VPW ���ݷ��ͺ���
// ���룺chbuf - ����������
//       chlen - ���ͳ���
// �������
// ���أ�true ��ʾ���ͳɹ� false ��ʾ����ʧ��
// ���ӣ���
//------------------------------------------------------------------------------
char VPW_SendEx( unsigned char *chbuf, unsigned char chlen )
{
    //data convert
    if(!DatToVPW(&PWM_Data,chbuf,chlen)) 
    {
        return false;
    }
    //Check BUS  �ж������Ƿ����
   // if(!VPW_CheckBus((VPWM*)&VPW)) return false;
		
		
    //VPW Output
    PWM_Data.State = VPW_OUTPUT;
    VPWM_OutputEx(&PWM_Data);
    
    return true;
}



//------------------------------------------------------------------------------
// ���ܣ�J1850 - PWM ���ݷ��ͺ���
// ���룺chbuf - ����������
//       chlen - ���ͳ���
//       IFRType - �Ƿ�����֡��Ӧ�� true or false
// �����FrameID - ֡��Ӧ�� ID
// ���أ�true ��ʾ���ͳɹ� false ��ʾ����ʧ��
// ���ӣ���
//------------------------------------------------------------------------------
char PWM_SendEx( unsigned char *chbuf, unsigned char chlen, unsigned char IFRType,
               unsigned char *FrameID )
{
    //uint32_t num=(uint32_t)((chlen<<3)+2+2);  
    //get space
//    uint8_t i = 0;
    
    //data convert
    if( true == IFRType )
    {
        if(!DatToPWM(&PWM_Data,chbuf,chlen,PWM_NOEOF))
        {
            return false;
        }
    }
    else
    {
        if(!DatToPWM(&PWM_Data,chbuf,chlen,0))
        {
            return false;
        }
    }
    //Check BUS
//    if(!PWM_CheckBus((VPWM*)&PWM))
//    {
//        return false;
//    }
    
    //SetDelay(16);//SOF
    //WaitDelay();  
    PWM_Data.State = PWM_OUTPUT;
	//	VPWM_Output_TEST((VPWM*)&PWM,&PWM_Data);
		
    VPWM_OutputEx(&PWM_Data);
    if( true == IFRType )
    {
//        PWM_Data.Index = 0;
//        PWM_Data.State = PWM_INPUT;
//        VPWM_Input((VPWM*)&PWM, &PWM_Data, FrameID, &i, 1, 400 );
//        if( PWM_Data.HighTime[ 0 ] > 24 )   // No EOF
//        {
//            return false;
//        }
    }

    return true;
}



//PB6 PB7
//PWM_OUT  PB6   TIMER3_CH0
//VPM_OUT   PB7  TIMER3_CH1

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void J1850VPW_Init(void)
{


		rcu_periph_clock_enable(RCU_TIMER3);
		rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
	
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
		
		gpio_init(GPIOB, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_6);//PWM �������ó����� ����Ӱ��
		//gpio_bit_reset(GPIOB,GPIO_PIN_6);//0���
	
	  TIM1_CH1_Cap_Init();//����
    
		
}



//PB6 PB7
//PWM_OUT  PB6   TIMER3_CH0
//VPM_OUT   PB7  TIMER3_CH1

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void J1850PWM_Init(void)
{

//     timer_oc_parameter_struct timer_ocinitpara;
//    timer_parameter_struct timer_initpara;

		rcu_periph_clock_enable(RCU_TIMER3);
		rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
		//gpio_pin_remap_config(GPIO_TIMER1_PARTIAL_REMAP0,ENABLE);//
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
		
		gpio_init(GPIOB, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_7);//VPW �������ó�����
		//gpio_bit_reset(GPIOB,GPIO_PIN_7);//0���
		
		TIM1_CH0_Cap_Init();//����
}


//------------------------------------------------------------------------------
// Funtion: ����VPWʱ��,ռ�ձ�
// Input  : port VPW CH1
// Output : none
// Return : none  
// Info   : none    CNTֵ��CCRX ֵ�Ƚ� ,�ı�CCRX ��ֵ ���Ǹı�ߵ͵�ƽ��ʱ��
//------------------------------------------------------------------------------
uint8_t VPWM_Update(VPWM_DATA *vdat )
{
    if( vdat->Index < vdat->Num )
    {
        
				TIMER_CH1CV(TIMER3) = (uint32_t)vdat->Duty[vdat->Index];     //ռ�ձ�;
			//TIMER_CH1CV(TIMER3) = vdat->Period[vdat->Index]-vdat->Duty[vdat->Index];     //ռ�ձ�;
				    /* configure the autoreload value */
				TIMER_CAR(TIMER3) = (uint32_t)vdat->Period[vdat->Index]+1;//����  ��װ�Ĵ���
			
        //vpwm->TxPPin.Tim->ARR = vdat->Period[vdat->Index];//����  ��װ�Ĵ���
			
        vdat->Index++;
        return true;
    }
    return false;
}


//------------------------------------------------------------------------------
// Funtion: ����PWMʱ��,ռ�ձ�
// Input  : port VPW CH0
// Output : none
// Return : none  
// Info   : none    CNTֵ��CCRX ֵ�Ƚ� ,�ı�CCRX ��ֵ ���Ǹı�ߵ͵�ƽ��ʱ��
//------------------------------------------------------------------------------
uint8_t PWM_Update(VPWM_DATA *vdat )
{
    if( vdat->Index < vdat->Num )
    {
        
				TIMER_CH0CV(TIMER3) = (uint32_t)vdat->Duty[vdat->Index]+0;     //ռ�ձ�;
			//TIMER_CH1CV(TIMER3) = vdat->Period[vdat->Index]-vdat->Duty[vdat->Index];     //ռ�ձ�;
				    /* configure the autoreload value */
				TIMER_CAR(TIMER3) = (uint32_t)vdat->Period[vdat->Index]+0;//����  ��װ�Ĵ���
			
        //vpwm->TxPPin.Tim->ARR = vdat->Period[vdat->Index];//����  ��װ�Ĵ���
			
        vdat->Index++;
        return true;
    }
    return false;
}



//------------------------------------------------------------------------------
// Funtion: VPW PWM �������
// Input  : vpwm Ӳ������
//          vdat ����ʱ�估ռ�ձ�����
// Output : none
// Return : none
// Info   : none
//------------------------------------------------------------------------------
void J1850_VPWM_Output(VPWM_DATA *vdat )
{

	
    if( vdat->Num == 0 )
    {
        return;
    }
    vdat->Index = 0;
		//vdat->State=VPWM_OUTPUT;    
    uint16_t Period = vdat->Period[vdat->Index];
		uint16_t CCR_Val = vdat->Duty[vdat->Index];
    //uint16_t CCR_Val = vdat->Period[vdat->Index]-vdat->Duty[vdat->Index];
    vdat->Index++;
		
		rcu_periph_clock_enable(RCU_TIMER3);
		rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
	
//		gpio_init(GPIOB, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ,GPIO_PIN_6);
//		gpio_bit_reset(GPIOB,GPIO_PIN_0);
//		
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
		//		gpio_bit_set(GPIOB,GPIO_PIN_7);

		
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;	
    timer_deinit(TIMER3);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
  

    /* initialize TIMER channel output parameter struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    /* CH0, CH1 and CH2 configuration in PWM mode */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;//�Ƿ�ʹ�����
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;//
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;//�������
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;//���������ʱ�ļ���
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;//����״̬ ������� 
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;//����״̬ �����������
    timer_channel_output_config(TIMER3, TIMER_CH_1, &timer_ocinitpara);
	
	

	
	
  /* TIMER1 configuration */
		timer_initpara.period            = Period;//����
    timer_initpara.prescaler         = 120-1;//1us
		
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    /* CH2 configuration in PWM mode0, duty cycle 75% */
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, CCR_Val);//ռ�ձ�  ���ߵ�ƽʱ��
    timer_channel_output_mode_config(TIMER3, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);//���� ENABLE


	
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);
		
    /* auto-reload preload enable */
    timer_enable(TIMER3);
		timer_flag_clear(TIMER3, TIMER_FLAG_CH1);//���־
		while(vdat->State!=VPWM_IDLE)
    {
        //if( TIM_GetFlagStatus(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc) == RESET )
				if(RESET == timer_flag_get(TIMER3, TIMER_FLAG_CH1))	//TIMER_INT_FLAG_CH1  TIMER_FLAG_CH1
        {
					//TIM_IT_CC3
           continue;
        }
//				if(RESET == timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP))	
//				{
//					continue;
//				}
				
				timer_flag_clear(TIMER3, TIMER_FLAG_CH1);//���־
				
        //TIM_ClearFlag(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc);
        if( !VPWM_Update(vdat) )//����ռ�ձ�
        {
						timer_disable(TIMER3);
						gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_7);
						gpio_bit_reset(GPIOB,GPIO_PIN_7);
            vdat->State = VPWM_IDLE;            
        }
    }
		

//    
//    TIM_DeInit(vpwm->TxPPin.Tim);
//    static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    static TIM_OCInitTypeDef  TIM_OCInitStructure;
//    /* Compute the prescaler value */
//    uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 10000000) - 1; 
//    
//    /* PWM1 Mode configuration: Channel1 */
//    //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//		
//		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStructure.TIM_Pulse = CCR_Val;
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//    
//    switch(vpwm->TxPPin.TimCh)
//    {
//        case 1:
//            TIM_OC1Init(vpwm->TxPPin.Tim, &TIM_OCInitStructure);
//            TIM_OC1PreloadConfig(vpwm->TxPPin.Tim, TIM_OCPreload_Enable);
//            break;
//        case 2:
//            TIM_OC2Init(vpwm->TxPPin.Tim, &TIM_OCInitStructure);
//            TIM_OC2PreloadConfig(vpwm->TxPPin.Tim,TIM_OCPreload_Enable);
//            break;
//        case 3:
//            TIM_OC3Init(vpwm->TxPPin.Tim, &TIM_OCInitStructure);
//            TIM_OC3PreloadConfig(vpwm->TxPPin.Tim, TIM_OCPreload_Enable);
//            break;
//        case 4:
//            TIM_OC4Init(vpwm->TxPPin.Tim, &TIM_OCInitStructure);
//            TIM_OC4PreloadConfig(vpwm->TxPPin.Tim, TIM_OCPreload_Enable);
//            break;
//    }
//    TIM_ARRPreloadConfig(vpwm->TxPPin.Tim, ENABLE);
//    
//    /* Time base configuration */
//    TIM_TimeBaseStructure.TIM_Period = Period;////��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
//   // TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;//Ԥ��Ƶ��
//		TIM_TimeBaseStructure.TIM_Prescaler = 84-1;//Ԥ��Ƶ�� 1us
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInit(vpwm->TxPPin.Tim, &TIM_TimeBaseStructure);
//    
//    /* TIM enable counter */
//    TIM_Cmd(vpwm->TxPPin.Tim, ENABLE);
//    
//    while(vdat->State!=VPWM_IDLE)
//    {
//        if( TIM_GetFlagStatus(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc) == RESET )
//        {
//           continue;
//        }
//        TIM_ClearFlag(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc);
//        if( !VPWM_Update(vpwm,vdat) )
//        {
//            TIM_Cmd(vpwm->TxPPin.Tim, DISABLE);   
//            vdat->State = VPWM_IDLE;            
//        }
//    }
}

//------------------------------------------------------------------------------
// ���ܣ�J1850 - VPW ���ݷ��ͺ���
// ���룺chbuf - ����������
//       chlen - ���ͳ���
// �������
// ���أ�true ��ʾ���ͳɹ� false ��ʾ����ʧ��
// ���ӣ���
//------------------------------------------------------------------------------
char J1850_VPW_Send( unsigned char *chbuf, unsigned char chlen )
{
    //data convert
    if(!DatToVPW(&PWM_Data,chbuf,chlen)) 
    {
        return false;
    }
    //Check BUS
//    if(!VPW_CheckBus((VPWM*)&VPW)) return false;
//    //VPW Output
    PWM_Data.State = VPW_OUTPUT;
    J1850_VPWM_Output( &PWM_Data);
    
    return true;
}


//test  OK  ��֤ͨ��
uint8_t J1850_VPW_Test(void)
{


	//J1850VPW_Init();
//	VPW_Open(VPW_ECUPORT);

	uint8_t dat[]={0x68,0x6A,0xF1,0x01,0x00,0x17};
//	uint8_t dat1[]={0x68,0x6A,0xF1,0x01,0x01,0x18};
//	uint8_t rxdat[10] = {0};
	
	
	J1850_VPW_Send(dat,sizeof(dat));
	
	
	return 1;
}



/*------------------------------------------------------------------------------
     VPWЭ���ʼ��ƽΪ0V��
�ڵ�1֡����ǰ��һ��163��239΢�루us���ĸߵ�ƽ��ʾSOF��֡ͷ�����ݿ�ʼ��־����
�������Բ�ͬ���̵ĸߵ͵�ƽ��ʾ����������0��1�����У�
�ߵ�ƽ��Ƚ���34-96us��ʾ��1�����ߵ�ƽ��Ƚ���96-163us��ʾ��0���� 
�͵�ƽ��Ƚ���34-96us��ʾ��0�����͵�ƽ��Ƚ���96-163us��ʾ��1����
����ʱ���ֽ�˳����ÿ���ֽڶ��Ǹ�λ��ǰ����λ�ں��˳��
�ߵ͵�ƽ������ڱ�ʾ��������ݣ��ֽ����ֽ�֮��û�м����
������һ֡����֮����һ����ȴ���239us�ĵ͵�ƽ��ʾEOF��֡β��֡������־����



// Funtion: ��ƽ���� ( ����ĵ�ƽ�������� HighTime LowTime )
// Input  : MaxLenth - �����ճ���
//          us - ���ȴ�ʱ��  
//          buf - ���ջ�����
// Output : DataLenth - ʵ�ʽ��ճ���
// Return : ʵ�ʲ����ʱ
// Info   : none
//------------------------------------------------------------------------------*/
uint32_t VPWM_Input( VPWM_DATA *vdat, uint8_t *buf, 
                     uint8_t *DataLenth, uint8_t MaxLenth, uint32_t us )
{
    
    uint32_t ustime = us;
//    uint32_t time;
    __IO uint16_t htime,ltime;
//    unsigned char /*j = 0x80,*/ k = 0;
//    unsigned char ch = 0;
//    unsigned char state;
    

//    while( 1 )
//    {   
        //�����벶��ģʽ�£�����⵽ICx�ź�����Ӧ�ı��غ󣬼������ĵ�ǰֵ������
        //������/�ȽϼĴ���(TIMx_CCRx)�С������������¼�ʱ����Ӧ��CCxIF��־
        //(TIMx_SR�Ĵ���)����1������������жϻ���DMA�������򽫲����жϻ���DMA��
        //��������������¼�ʱCCxIF��־�Ѿ�Ϊ�ߣ���ô�ظ������־CCxOF
        //(TIMx_SR�Ĵ���)����1��дCCxIF=0�����CCxIF�����ȡ�洢��TIMx_CCRx
        //�Ĵ����еĲ�������Ҳ�����CCxIF��дCCxOF=0�����CCxOF�� 
        //��������˵�������TI1�����������ʱ�����������ֵ��TIMx_CCR1�Ĵ����У�
        //�������£�
        //�� ѡ����Ч����ˣ�TIMx_CCR1�������ӵ�TI1���룬
        //   ����д��TIMx_CCR1�Ĵ����е�CC1S=01��ֻҪCC1S��Ϊ��00����
        //   ͨ��������Ϊ���룬����TIMx_CCR1�Ĵ�����Ϊֻ����
        //�� ���������źŵ��ص㣬
        //   ���������˲���Ϊ����Ĵ���(������ΪTIxʱ�������˲�������λ��TIMx_CCMRx
        //   �Ĵ����е�ICxFλ)�����������ź������5���ڲ�ʱ�����ڵ�ʱ���ڶ�����
        //   �����������˲����Ĵ�����5��ʱ�����ڣ�������ǿ���(��fDTSƵ��)������
        //   ��8�Σ���ȷ����TI1��һ����ʵ�ı��ر任������TIMx_CCMR1�Ĵ�����д��
        //   IC1F=0011��
        //�� ѡ��TI1ͨ������Чת�����أ���TIMx_CCER�Ĵ�����д��CC1P=0(������)�� 
        //�� ��������Ԥ��Ƶ�����ڱ����У�����ϣ����������ÿһ����Ч�ĵ�ƽת��
        //   ʱ�̣����Ԥ��Ƶ������ֹ(дTIMx_CCMR1�Ĵ�����IC1PS=00)��
        //�� ����TIMx_CCER�Ĵ�����CC1E=1���������������ֵ������Ĵ����С�
        //�� �����Ҫ��ͨ������TIMx_DIER�Ĵ����е�CC1IEλ��������ж�����
        //   ͨ������TIMx_DIER�Ĵ����е�CC1DEλ����DMA���� ������һ�����벶��ʱ��
        //�� ������Ч�ĵ�ƽת��ʱ����������ֵ�����͵�TIMx_CCR1�Ĵ�
        
//				
//				if((vpwm->RxPin.Tim->SR & vpwm->RxPin.TimFlagCc) == (uint16_t)RESET)
//        {
//            if( vdat->Index != 0 )
//            {
//                if(vdat->State == PWM_INPUT) 
//                {
//                    if( time - GetDelay() > 32 )
//                    {
//                        break;
//                    }
//                }
//                else//VPW
//                {
//                    uint32_t time1 = GetDelay();
//                    if( time - time1 > 270 )
//                    {
//                        break;
//                    }
//                }
//            }
//        
//            if( !DelayState() )
//            {
//                if( vdat->Index == 0 )
//                {
//                    break;
//                }
//            }
//            continue;
//        }
//        vpwm->RxPin.Tim->SR = (uint16_t)~vpwm->RxPin.TimFlagCc;
//        /*htime = vpwm->RxPin.Tim->CCR1;
//        ltime = vpwm->RxPin.Tim->CCR2;  
//        
//        if(htime==0 && ltime==0)
//        {
//            htime = vpwm->RxPin.Tim->CCR1;
//            ltime = vpwm->RxPin.Tim->CCR2;  
//        }*/
//        
//        /* Get the Input Capture value */
//        vdat->HighTime[vdat->Index] = vpwm->RxPin.Tim->CCR1;
//        vdat->LowTime[vdat->Index] = vpwm->RxPin.Tim->CCR2;
//        
//				
//				
//       // else//VPW
//        {
//            state = VPWtoBit( vdat->HighTime[ vdat->Index ], 
//                              vdat->LowTime[ vdat->Index ] );
//            if( state & VPWM_SOF )
//            {
//                ch = 0;
//                j = 0x80;
//            }
//        
//            else if( state & VPWM_BIT11 )
//            {
//                ch |= j;
//                j = j >> 1;
//                ch |= j;
//                j = j >> 1;
//            }
//            else if( state & VPWM_BIT01 )
//            {
//                j = j >> 1;
//                ch |= j;
//                j = j >> 1;
//            }
//            else if( state & VPWM_BIT10 )
//            {
//                ch |= j;
//                j = j >> 2;
//            }
//            else if( state & VPWM_BIT00 )
//            {
//                j = j >> 2;
//            }
//            else if( state & VPWM_BIT1 )
//            {
//                ch |= j;
//                j = j >> 1;
//            }
//            else if( state & VPWM_BIT0 )
//            {
//                j = j >> 1;
//            }
//            else if( state == VPWM_ERR )
//            {
//                ch = 0;
//                j = 0x80;
//            }
//            if( j == 0 )
//            {
//                buf[ k++ ] = ch;
//                if( k >= MaxLenth )
//                {
//                    break;
//                }
//                j = 0x80;
//                ch = 0;
//            }
//        }
//        
//        vdat->Index++;
//        if( vdat->Index >= vdat->Num )
//        {
//            break;
//        }
//      //  time = GetDelay();
//    }

    return ustime;
}


//�͵�ƽʱ�� ת bit
uint8_t GetLowBit(uint8_t time)
{

		uint8_t Value=0;
	//�͵�ƽ��Ƚ���34-96us��ʾ��0�����͵�ƽ��Ƚ���96-163us��ʾ��1����
	if(time>=34&&time<=96) 
		Value=0;
	else if(time>=96&&time<=163) 
		Value=1;
	
	return Value;
}
//�ߵ�ƽʱ�� ת bit
uint8_t GetHiBit(uint8_t time)
{
		uint8_t Value=0;
	//�ߵ�ƽ��Ƚ���34-96us��ʾ��1�����ߵ�ƽ��Ƚ���96-163us��ʾ��0��
	if(time>=34&&time<=96) 
		Value=1;
	else if(time>=96&&time<=163) 
		Value=0;
	
	return Value;
}


//VPWtoBit
/****************************
���ݸߵ͵�ƽʱ�� �õ�VPW �ֽ�
����:

����ֵ:

*****************************/
uint8_t GetVpwBitToByte(uint8_t*iHBuf,uint8_t*iLBuf)
{
	uint8_t Value=0;
	uint8_t j=0;
  //��λ��ǰ ��λ�ں�
	//VPWtoBit(iHBuf[0],iHBuf[1]);
//	for(i=0;i<4;i++)
//	{
//		j=GetLowBit(iLBuf[i]);iPos--;
//		Value+=j<<iPos;   
//		j=GetHiBit(iHBuf[i]);iPos--;
//		Value+=j<<iPos;
//	}
	
		j=GetLowBit(iLBuf[0]);Value+=j<<7; 
		j=GetHiBit(iHBuf[0]); Value+=j<<6; 
	
		j=GetLowBit(iLBuf[1]);Value+=j<<5; 
		j=GetHiBit(iHBuf[1]); Value+=j<<4; 
	
		j=GetLowBit(iLBuf[2]);Value+=j<<3; 
		j=GetHiBit(iHBuf[2]); Value+=j<<2; 	
		
		j=GetLowBit(iLBuf[3]);Value+=j<<1; 
		j=GetHiBit(iHBuf[3]); Value+=j; 	
	return Value;
}


//PWMtoBit
//�ߵ�ƽʱ�� ת bit
uint8_t GetPWMBit(uint8_t htime,uint8_t ltime)
{
		uint8_t Value=0;

	//һ��24us ���� һ����ƽ 
	//�ߵ�ƽ>���ƽʱ ��0 
	//�͵�ƽ>�ߵ�ƽʱ ��1
	if(htime>13&&ltime>24) Value=0;
	
	else if(htime>ltime)Value=0;
	else Value=1;
	
	return Value;
}

/****************************
���ݸߵ͵�ƽʱ�� �õ�pwm �ֽ�
����:

����ֵ:

*****************************/
uint8_t GetPWMBitToByte(uint8_t*iHBuf,uint8_t*iLBuf)
{
	uint8_t Value=0;
	uint8_t i=0,j=0;
  //��λ��ǰ ��λ�ں�
	//VPWtoBit(iHBuf[0],iHBuf[1]);
	
	for(i=0;i<8;i++)
	{
		j=GetPWMBit(iHBuf[i],iLBuf[i]);
		Value+=j<<(7-i);
	}
	

	return Value;
}



//------------------------------------------------------------------------------
// Funtion: VPW PWM �������
// Input  : vpwm Ӳ������
//          vdat ����ʱ�估ռ�ձ�����
// Output : none
// Return : none
// Info   : none
//------------------------------------------------------------------------------
u16 J1850_VPWM_SendAndGet(VPWM_DATA *vdat )
{
//		uint8_t value=0;
//		uint32_t tmp=0;
		uint16_t iRet=0;
    if( vdat->Num == 0 )
    {
        return 0;
    }
    vdat->Index = 0;
		//vdat->State=VPWM_OUTPUT;    
    uint16_t Period = vdat->Period[vdat->Index];
		uint16_t CCR_Val = vdat->Duty[vdat->Index];
    vdat->Index++;

    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;	
    timer_deinit(TIMER3);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
  

    /* initialize TIMER channel output parameter struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    /* CH0, CH1 and CH2 configuration in PWM mode */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;//�Ƿ�ʹ�����
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;//
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;//�������
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;//���������ʱ�ļ���
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;//����״̬ ������� 
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;//����״̬ �����������
    timer_channel_output_config(TIMER3, TIMER_CH_1, &timer_ocinitpara);
	
	
  /* TIMER1 configuration */
		timer_initpara.period            = Period;//����
    timer_initpara.prescaler         = 120-1;//1us
		
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    /* CH2 configuration in PWM mode0, duty cycle 75% */
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, CCR_Val);//ռ�ձ�  ���ߵ�ƽʱ��
    //timer_channel_output_mode_config(TIMER3, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);//���� ENABLE


	
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);
		
		//������չر��ж�
		//timer_interrupt_disable(TIMER1,TIMER_INT_CH1|TIMER_INT_UP); //TIME1 �����жϹر�
		timer_disable(TIMER1);//����رս���
		
		timer_channel_output_mode_config(TIMER3, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    /* auto-reload preload enable */
    timer_enable(TIMER3);
		timer_flag_clear(TIMER3, TIMER_FLAG_CH1);//���־
		while(vdat->State!=VPWM_IDLE)
    {
        //if( TIM_GetFlagStatus(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc) == RESET )
				if(RESET == timer_flag_get(TIMER3, TIMER_FLAG_CH1))	//TIMER_INT_FLAG_CH1  TIMER_FLAG_CH1
        {
					//TIM_IT_CC3
           continue;
        }
				
				timer_flag_clear(TIMER3, TIMER_FLAG_CH1);//���־
				
        //TIM_ClearFlag(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc);
        if( !VPWM_Update(vdat) )//����ռ�ձ�
        {
						timer_disable(TIMER3);
//						gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_7);
//						gpio_bit_reset(GPIOB,GPIO_PIN_7);
            vdat->State = VPWM_IDLE;            
        }
    }
		
		//�ȴ�����
		//������չر��ж�
		iJ1850EofFlag=0;
		iReCount=0;//

		//timer_interrupt_enable(TIMER1,TIMER_INT_CH1|TIMER_INT_UP);
		timer_enable(TIMER1);//��������
		GetTimer6Cnt();
		
		

		return iRet;
}


//------------------------------------------------------------------------------
// Funtion: PWM �������
// Input  : vpwm Ӳ������  TIMER3, TIMER_CH_0
//          vdat ����ʱ�估ռ�ձ�����
// Output : none
// Return : none
// Info   : none  
//------------------------------------------------------------------------------
u16 J1850_PWM_SendAndGet(VPWM_DATA *vdat )
{
//		uint8_t value=0;
//		uint32_t tmp=0;
		uint16_t iRet=0;
    if( vdat->Num == 0 )
    {
        return 0;
    }
    vdat->Index = 0;
		vdat->State=PWM_OUTPUT;    
    uint16_t Period = vdat->Period[vdat->Index]+0;
		uint16_t CCR_Val = vdat->Duty[vdat->Index];
    vdat->Index++;

    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;	
    timer_deinit(TIMER3);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
  

    /* initialize TIMER channel output parameter struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    /* CH0, CH1 and CH2 configuration in PWM mode */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;//�Ƿ�ʹ�����
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;//
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;//�������
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;//���������ʱ�ļ���
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;//����״̬ ������� 
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;//����״̬ �����������
    timer_channel_output_config(TIMER3, TIMER_CH_0, &timer_ocinitpara);
	
	
  /* TIMER1 configuration */
		timer_initpara.period            = Period;//����
    timer_initpara.prescaler         = 120-1;//1us
		
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    /* CH2 configuration in PWM mode0, duty cycle 75% */
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_0, CCR_Val);//ռ�ձ�  ���ߵ�ƽʱ��
    timer_channel_output_mode_config(TIMER3, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);//���� ENABLE


	
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);
		
		//������չر��ж�
		//timer_interrupt_disable(TIMER1,TIMER_INT_CH1|TIMER_INT_UP); //TIME1 �����жϹر�
		timer_disable(TIMER1);//����
		
    /* auto-reload preload enable */
    timer_enable(TIMER3);
		Delay_us(10);
		timer_flag_clear(TIMER3, TIMER_FLAG_CH0);//���־
		while(vdat->State!=VPWM_IDLE)
    {
        //if( TIM_GetFlagStatus(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc) == RESET )
				if(RESET == timer_flag_get(TIMER3, TIMER_FLAG_CH0))	//TIMER_INT_FLAG_CH1  TIMER_FLAG_CH1
        {
					//TIM_IT_CC3
           continue;
        }
				Delay_us(1);
				timer_flag_clear(TIMER3, TIMER_FLAG_CH0);//���־
				
        //TIM_ClearFlag(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc);
        if( !PWM_Update(vdat) )//����ռ�ձ�
        {
						timer_disable(TIMER3);
//						gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_7);
//						gpio_bit_reset(GPIOB,GPIO_PIN_7);
            vdat->State = VPWM_IDLE;            
        }
    }
		
		//�ȴ�����
		//������չر��ж�
		iJ1850EofFlag=0;
		iReCount=0;//
		iHBitSum=0;
		iLBitSum=0;

		//timer_interrupt_enable(TIMER1,TIMER_INT_CH1|TIMER_INT_UP);
		timer_enable(TIMER1);//����
		GetTimer6Cnt();
	
		//timer_disable(TIMER1);//����
		//timer_interrupt_disable(TIMER1,TIMER_INT_CH1|TIMER_INT_UP); //TIME1 �����жϹر�
		return iRet;
}
//------------------------------------------------------------------------------
// ���ܣ�J1850 - PWM ���ݷ��� ���պ���
// ���룺chbuf - ����������
//       chlen - ���ͳ���
// �������
// ���أ�����֡����
//------------------------------------------------------------------------------
u16 J1850_PWM_SendFrame( unsigned char *chbuf, unsigned char chlen )
{
		u16 iRet=0;
    //data convert

//	static unsigned char dat[]={0x61, 0x6A, 0xF1, 0x01, 0x00, 0x0A};
//	chlen=sizeof(dat);
	//IFRType=0X80;
		    //data convert
		// chbuf[chlen]=0xF1;//ģ��ECU ���һ���ֽ���F1
	IFRType=0X80;//Ҫ�����������ģ�� DT �豸
    if( 0x80 == IFRType )
    {
        if(!DatToPWM(&PWM_Data,chbuf,chlen,PWM_NOEOF))
        {
            return false;
        }
    }
    else
    {
        if(!DatToPWM(&PWM_Data,chbuf,chlen,0))
        {
            return false;
        }
    }
	
	
    //Check BUS
    //if(!PWM_CheckBus()) return false;
		
		iRet=J1850_PWM_SendAndGet( &PWM_Data);
		iRet=chlen+1;//��һ֡��ַ
		
		return iRet;
}

//------------------------------------------------------------------------------
// ���ܣ�J1850 - VPW ���ݷ��� ���պ���
// ���룺chbuf - ����������
//       chlen - ���ͳ���
// �������
// ���أ�����֡����
//------------------------------------------------------------------------------
u16 J1850_VPW_SendFrame( unsigned char *chbuf, unsigned char chlen )
{
	
		u16 iRet=0;

    //data convert
    if(!DatToVPW(&PWM_Data,chbuf,chlen)) 
    {
        return false;
    }
    //Check BUS
   // if(!VPW_CheckBus()) return false;
    PWM_Data.State = VPW_OUTPUT;
		
    iRet=J1850_VPWM_SendAndGet( &PWM_Data);
    iRet=chlen+1;//��һ֡��ַ
    return iRet;
}

