//------------------------------------------------------------------------------
//  Purpose: PWM VPW 输入捕获和输出脉宽调制模块
//  Funtion: 提供 PWM VPW 的输出脉宽调制输入捕获接口
//  Dependent: 32F207 LIB
//  Designer: 
//  Date.Ver:
//  Other: 发送时，每次通过外部传入的 VPWM_DATA 对象给出预想输出 VPW PWM 波形的
//         脉冲时间总长和脉冲占空比，由外部传入的 VPWM 对象指定使用对应的硬件资
//         源输出该波形。接收时，每次通过外部传入的 VPWM 对象指定使用对应的硬件
//         资源捕获脉冲宽度和占空比并顺序保存至  VPWM_DATA 对象并传出
//------------------------------------------------------------------------------

#include "vpwm.h"
#include "bsp.h"
#include "timer.h"
static __IO uint32_t g_iDelayTime;
__IO u8 IFRType=0;//pwm 协议帧内应答 标记 0x80=PWM_NOEOF，else 

/************************************************************************
循环冗余校验																														*


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
检查CRC 字节是否到来了

CRC 字节正确   返回1
CRC 字节不正确 返回0 
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
// Funtion: VPW 判断总线空闲状态
// Input  : vpwm - VPW对象
// Output : none
// Return : none
// Info   : none  等待低空闲>300us
//------------------------------------------------------------------------------
uint8_t VPW_CheckBus(void)
{
    uint32_t time;
   // SetDelay( 200000 );  //200ms
    time = 0;
	//if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_1))//低电平
		GetTimer6Cnt();//清空了计数器
    // 等待一个低空闲
    while(1)
    {
			if(iPartValue!=6) break;
        time = GetTimer6CntEx();
			  if(time>(200*1000)) break;//超时了
				if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_1))
				{
					GetTimer6Cnt();//清空了计数器
					 // 一直等待一个低
					while( RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_1))//低电平时间
					{
						 time = GetTimer6CntEx();
						 if(time>300) return true;//EOF 
					}
					GetTimer6Cnt();//清空了计数器
				}

    }
	
	
    return false;
}

//------------------------------------------------------------------------------
// Funtion: PWM 判断总线空闲状态
// Input  : pwm - PWM对象
// Output : none
// Return : none
// Info   : none 等待低空闲>96us
//------------------------------------------------------------------------------
uint8_t PWM_CheckBus(void)
{

		    uint32_t time;
   // SetDelay( 200000 );  //200ms
    time = 0;
	//if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_1))//低电平
		GetTimer6Cnt();//清空了计数器
    // 等待一个低空闲
    while(1)
    {
			if(iPartValue!=7) break;
        time = GetTimer6CntEx();
			  if(time>(200*1000)) break;//超时了
				if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_0))
				{
					GetTimer6Cnt();//清空了计数器
					 // 一直等待一个低
					while( RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_0))//低电平时间
					{
						 time = GetTimer6CntEx();
						 if(time>96) return true;//EOF  96 us
						//if(time>1000) return true;//EOF  96 us
					}
					GetTimer6Cnt();//清空了计数器
				}

    }
	
	
    return false;
}

//------------------------------------------------------------------------------
// Funtion: VPW 判断总线空闲状态
// Input  : vpwm - VPW对象
// Output : none
// Return : none
// Info   : none  等待低空闲>300us
//------------------------------------------------------------------------------
//uint8_t VPW_CheckBus( VPWM *vpwm )
//{
//    uint32_t time;
//    SetDelay( 200000 );  //200ms
//    time = 0;
//    // 等待一个低空闲
//    while( DelayState() )
//    {
//        time = GetDelay();
//        // 一直等待一个低
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
// Funtion: PWM 判断总线空闲状态
// Input  : pwm - PWM对象
// Output : none
// Return : none
// Info   : none 等待低空闲>96us
//------------------------------------------------------------------------------
//uint8_t PWM_CheckBus( VPWM *vpwm )
//{
//    uint32_t time;
//    SetDelay( 200000 );  //200ms
//    time = 0;
//    // 等待一个高空闲
//    while( DelayState() )
//    {
//        time = GetDelay();
//        // 一直等待一个高空闲
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
// Funtion: 更新VPW时间,占空比
// Input  : port VPW号
// Output : none
// Return : none  
// Info   : none    CNT值与CCRX 值比较 ,改变CCRX 的值 即是改变高低电平的时间
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
//                vpwm->TxPPin.Tim->CCR4 = vdat->Duty[vdat->Index];     //占空比
//                break;
//            default:
//                return false;
//        }
//        vpwm->TxPPin.Tim->ARR = vdat->Period[vdat->Index];//周期  重装寄存器
//        vdat->Index++;
//        return true;
//    }
//    return false;
//}


////------------------------------------------------------------------------------
//// Funtion: VPW PWM 脉宽输出
//// Input  : vpwm 硬件对象
////          vdat 脉冲时间及占空比数据
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
//	  //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;//输出比较模式
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStructure.TIM_Pulse = 50;
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

//    TIM_OC3Init(vpwm->TxPPin.Tim, &TIM_OCInitStructure);
//    TIM_OC3PreloadConfig(vpwm->TxPPin.Tim, TIM_OCPreload_Enable);

//    TIM_ARRPreloadConfig(vpwm->TxPPin.Tim, ENABLE);
//    
//    /* Time base configuration */
//    TIM_TimeBaseStructure.TIM_Period = 100;////设置在下一个更新事件装入活动的自动重装载寄存器周期的值
//    //TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;//预分频器
//		TIM_TimeBaseStructure.TIM_Prescaler = 84-1;//pwm 的频率
//    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInit(vpwm->TxPPin.Tim, &TIM_TimeBaseStructure);
//    
//    /* TIM enable counter */
//    TIM_Cmd(vpwm->TxPPin.Tim, ENABLE);
//}

////------------------------------------------------------------------------------
//// Funtion: VPW PWM 脉宽输出
//// Input  : vpwm 硬件对象
////          vdat 脉冲时间及占空比数据
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
//    TIM_TimeBaseStructure.TIM_Period = Period;////设置在下一个更新事件装入活动的自动重装载寄存器周期的值
//   // TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;//预分频器
//		TIM_TimeBaseStructure.TIM_Prescaler = 84-1;//预分频器 1us
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
// Funtion: VPW 一个脉宽电平到2进制位转换
// Input  : lowleveltime 低电平时间
//          highleveltime 高电平时间
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
// Funtion: VPW 一祯数据脉宽电平到2进制位转换
// Input  : vpwm - VPW脉宽数据位
//          len - buf的最大长度
// Output : buf - 转换后的数据
// Return : 实际数据长度
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
// Funtion: PWM 一个脉宽电平到2进制位转换
// Input  : lowleveltime 低电平时间
//          highleveltime 高电平时间
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
// Funtion: PWM 一祯数据脉宽电平到2进制位转换
// Input  : vpwm - PWM 脉宽数据位
//          len - buf的最大长度
// Output : buf - 转换后的数据
// Return : 实际数据长度
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
// Funtion: 电平捕获 ( 捕获的电平被保存在 HighTime LowTime )
// Input  : MaxLenth - 最大接收长度
//          us - 最大等待时间  
//          buf - 接收缓冲区
// Output : DataLenth - 实际接收长度
// Return : 实际捕获耗时
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
//    /* 输入捕获模式设置：TIM3的channel3作为输入，比较器上升沿触发计时停止计时中断*/ 
//#else
//    TIM_PWMIConfig(vpwm->RxPin.Tim, &TIM_ICInitStructure);     
//#endif  
//    /* Select the TIM  Input Trigger */
//    TIM_SelectInputTrigger(vpwm->RxPin.Tim,0x40|(vpwm->RxPin.TimCh-1)<<4);
//     /* Select the slave Mode: Reset Mode */
//    TIM_SelectSlaveMode(vpwm->RxPin.Tim, TIM_SlaveMode_Reset);//TIM从模式：触发信号的上升沿重新初始化计数器和触发寄存器的更新事件
//    /* Enable the Master/Slave Mode */
//    TIM_SelectMasterSlaveMode(vpwm->RxPin.Tim, TIM_MasterSlaveMode_Enable); //启动定时器的被动触发
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
//        //在输入捕获模式下，当检测到ICx信号上相应的边沿后，计数器的当前值被锁存
//        //到捕获/比较寄存器(TIMx_CCRx)中。当发生捕获事件时，相应的CCxIF标志
//        //(TIMx_SR寄存器)被置1，如果开放了中断或者DMA操作，则将产生中断或者DMA请
//        //求。如果发生捕获事件时CCxIF标志已经为高，那么重复捕获标志CCxOF
//        //(TIMx_SR寄存器)被置1。写CCxIF=0可清除CCxIF，或读取存储在TIMx_CCRx
//        //寄存器中的捕获数据也可清除CCxIF。写CCxOF=0可清除CCxOF。 
//        //以下例子说明如何在TI1输入的上升沿时捕获计数器的值到TIMx_CCR1寄存器中，
//        //步骤如下：
//        //● 选择有效输入端：TIMx_CCR1必须连接到TI1输入，
//        //   所以写入TIMx_CCR1寄存器中的CC1S=01，只要CC1S不为’00’，
//        //   通道被配置为输入，并且TIMx_CCR1寄存器变为只读。
//        //● 根据输入信号的特点，
//        //   配置输入滤波器为所需的带宽(即输入为TIx时，输入滤波器控制位是TIMx_CCMRx
//        //   寄存器中的ICxF位)。假设输入信号在最多5个内部时钟周期的时间内抖动，
//        //   我们须配置滤波器的带宽长于5个时钟周期；因此我们可以(以fDTS频率)连续采
//        //   样8次，以确认在TI1上一次真实的边沿变换，即在TIMx_CCMR1寄存器中写入
//        //   IC1F=0011。
//        //● 选择TI1通道的有效转换边沿，在TIMx_CCER寄存器中写入CC1P=0(上升沿)。 
//        //● 配置输入预分频器。在本例中，我们希望捕获发生在每一个有效的电平转换
//        //   时刻，因此预分频器被禁止(写TIMx_CCMR1寄存器的IC1PS=00)。
//        //● 设置TIMx_CCER寄存器的CC1E=1，允许捕获计数器的值到捕获寄存器中。
//        //● 如果需要，通过设置TIMx_DIER寄存器中的CC1IE位允许相关中断请求，
//        //   通过设置TIMx_DIER寄存器中的CC1DE位允许DMA请求。 当发生一个输入捕获时：
//        //● 产生有效的电平转换时，计数器的值被传送到TIMx_CCR1寄存
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
// Funtion: 电平转换为时间和占空比
// Input  : time0 - 下降沿时间
//          time1 - 上升沿时间
// Output : vdat - 转换后时间和占空比
// Return : none
// Info   : none
//------------------------------------------------------------------------------
void ToPWM(VPWM_DATA *vdat, uint16_t time0, uint16_t time1)
{
		//1us 精度时
	uint16_t Period = (time0+time1-1);//周期
    vdat->Period[vdat->Num] = Period;
	vdat->Duty[vdat->Num++] =(uint32_t)(Period*time0)/(time0+time1);
	
	//1us/10 精度时
//	 uint16_t Period = (10*(time0+time1)-1);//周期
//	vdat->Period[vdat->Num] = Period;
//   vdat->Duty[vdat->Num++] = ((uint32_t)(Period*(10*time0+vdat->Offset)))/(10*(time0+time1));            
         
		
    if( vdat->Num > 160 )
    {
        vdat->Num = 160;
    }
}
 
//------------------------------------------------------------------------------
// Funtion: PWM 数据转换为时间和占空比
// Input  : buf - 输入的数据
//          len - buf 最大长度
//          datatype - 数据的格式
// Output : vdat - 转换后时间和占空比
// Return : true
// Info   : none
//------------------------------------------------------------------------------
uint8_t DatToPWM( VPWM_DATA *vdat, uint8_t *buf, uint8_t len, uint8_t datatype )
{
    uint8_t i,j;
//    uint16_t time = 30;//32;//SOF   	上升沿
//    uint16_t time1 = 16;						//下降沿
	
	    uint16_t time = 31;//32;//SOF   	上升沿
    uint16_t time1 = 17;						//下降沿
	
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
                time = 8;//上升沿
                time1 = 16;//下降沿
            }
            else
            {
                //LOW
                time = 16;//上升沿
                time1 = 8;//下降沿
            }
						//PWM 协议是一个周期一个电平
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
// Funtion: VPW 数据转换为时间和占空比
// Input  : buf - 输入的数据
//          len - buf 最大长度
// Output : vdat - 转换后时间和占空比
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
            if( !state )//每次都设置2个电平 所以要这样操作
            {
							//VPW 协议是一个电平翻转一个BIT
                ToPWM( vdat, time, time1 );   //time=上升沿 ,time1=下降沿
							
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
// Funtion: 初始化PWM处理模块（进入系统时必须初始化）
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
// 功能：J1850 - PWM 数据发送函数
// 输入：chbuf - 待发送命令
//       chlen - 发送长度
//       IFRType - 是否允许帧内应答 true or false
// 输出：FrameID - 帧内应答 ID
// 返回：true 表示发送成功 false 表示发送失败
// 附加：无
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
// Funtion: 接收PWM数据
// Input  : chlen  接受长度
//          ms  超时时间  
//          IFRType - 是否允许帧内应答 true or false
//          FrameID - 帧内应答 ID
// Output : chbuf  接受缓冲区
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
//        // 注意：帧内应答相应时间必须 < 47us
//        // STM32运行至此处已等待32US的位超时，实际响应发出时间为 39US，MCU代码运算
//        // 时间为 7 US, 如果换成其他 MCU 请注意运算时间导致的延时
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
// Funtion: 接收VPW数据
// Input  : chlen  接受长度
//          ms  超时时间  
// Output : chbuf  接受缓冲区
// Return : 接收数据长度
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



/* --------------------------测试OK------------------------------------------------ */
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
//             return 1;	//准备工装切换协议口线，测试下一组协议
//        }
//   }
return 1;

}



VPWM_DATA VPM_Data = { (VPWM_STATUS)0 };
//------------------------------------------------------------------------------
// Funtion: 初始化VPW处理模块（进入系统时必须初始化）
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
// 功能：J1850 - VPW 数据发送函数
// 输入：chbuf - 待发送命令
//       chlen - 发送长度
// 输出：无
// 返回：true 表示发送成功 false 表示发送失败
// 附加：无
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


///* -------测试OK，通讯正常------------------------------------------------------------------- */

uint8_t VPW_Test(void)
{
//	uint32_t time;
//	CMPHY_Relay Relaybuf={0};//2012-12-20 Relaybuf init
//	CMPHY_Relay_Init(&Relaybuf);


//	Relaybuf.CommType = COMM_VPW_TYPE; //通讯类型串行口
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
// Funtion: 电平捕获 ( 捕获的电平被保存在 HighTime LowTime )
// Input  : MaxLenth - 最大接收长度
//          us - 最大等待时间  
//          buf - 接收缓冲区
// Output : DataLenth - 实际接收长度
// Return : 实际捕获耗时
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
//    /* 输入捕获模式设置：TIM3的channel3作为输入，比较器上升沿触发计时停止计时中断*/ 
//else//vpw
//    TIM_PWMIConfig(vpwm->RxPin.Tim, &TIM_ICInitStructure);     
// 
//    /* Select the TIM  Input Trigger */
//    TIM_SelectInputTrigger(vpwm->RxPin.Tim,0x40|(vpwm->RxPin.TimCh-1)<<4);
//     /* Select the slave Mode: Reset Mode */
//    TIM_SelectSlaveMode(vpwm->RxPin.Tim, TIM_SlaveMode_Reset);//TIM从模式：触发信号的上升沿重新初始化计数器和触发寄存器的更新事件
//    /* Enable the Master/Slave Mode */
//    TIM_SelectMasterSlaveMode(vpwm->RxPin.Tim, TIM_MasterSlaveMode_Enable); //启动定时器的被动触发


//	NVIC_InitTypeDef NVIC_InitStructure;
//	//中断分组初始化
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
//	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 
//	
//	//TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC4,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
////TIM_ITConfig(TIM2,TIM_IT_CC4,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	

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




//msy 自定义
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
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;//下拉
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);     

//		GPIO_SetBits(GPIOA,GPIO_Pin_2);

//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        
//    GPIO_Init(GPIOA, &GPIO_InitStructure);     

//    /* Connect TIM pins to AF */  
//		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM5); //GPIOA4 复用为USART1		
//		GPIO_ResetBits(GPIOA,GPIO_Pin_2);//初始电平为低
}


//------------------------------------------------------------------------------
// Funtion: 更新VPW时间,占空比
// Input  : port VPW号
// Output : none
// Return : none  
// Info   : none    CNT值与CCRX 值比较 ,改变CCRX 的值 即是改变高低电平的时间
//------------------------------------------------------------------------------
//uint8_t VPWM_UpdateEx( TIM_TypeDef *iTimer, VPWM_DATA *vdat )
//{
//    if( vdat->Index < vdat->Num )
//    {//CH3
//        iTimer->CCR3 = vdat->Duty[vdat->Index]; //高电平时间     
//        iTimer->ARR = vdat->Period[vdat->Index];//周期  重装寄存器
//			//高电平时间=vdat->Duty
//			//低电平时间=vdat->Period-vdat->Duty;
//			
////			  iTimer->CCR3 = 5+1; //高电平时间     
////        iTimer->ARR = 10+1;//周期  重装寄存器
//			
//			
//        vdat->Index++;
//        return true;
//    }
//    return false;
//}

//------------------------------------------------------------------------------
// Funtion: VPW PWM 脉宽输出
// Input  : vpwm 硬件对象
//          vdat 脉冲时间及占空比数据
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
//    TIM_TimeBaseStructure.TIM_Period = Period;////设置在下一个更新事件装入活动的自动重装载寄存器周期的值
//    //TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;//预分频器
//		TIM_TimeBaseStructure.TIM_Prescaler = 84-1;//预分频器 1us
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
// 功能：J1850 - VPW 数据发送函数
// 输入：chbuf - 待发送命令
//       chlen - 发送长度
// 输出：无
// 返回：true 表示发送成功 false 表示发送失败
// 附加：无
//------------------------------------------------------------------------------
char VPW_SendEx( unsigned char *chbuf, unsigned char chlen )
{
    //data convert
    if(!DatToVPW(&PWM_Data,chbuf,chlen)) 
    {
        return false;
    }
    //Check BUS  判断总线是否空闲
   // if(!VPW_CheckBus((VPWM*)&VPW)) return false;
		
		
    //VPW Output
    PWM_Data.State = VPW_OUTPUT;
    VPWM_OutputEx(&PWM_Data);
    
    return true;
}



//------------------------------------------------------------------------------
// 功能：J1850 - PWM 数据发送函数
// 输入：chbuf - 待发送命令
//       chlen - 发送长度
//       IFRType - 是否允许帧内应答 true or false
// 输出：FrameID - 帧内应答 ID
// 返回：true 表示发送成功 false 表示发送失败
// 附加：无
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
		
		gpio_init(GPIOB, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_6);//PWM 引脚设置成输入 避免影响
		//gpio_bit_reset(GPIOB,GPIO_PIN_6);//0输出
	
	  TIM1_CH1_Cap_Init();//输入
    
		
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
		
		gpio_init(GPIOB, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_7);//VPW 引脚设置成输入
		//gpio_bit_reset(GPIOB,GPIO_PIN_7);//0输出
		
		TIM1_CH0_Cap_Init();//输入
}


//------------------------------------------------------------------------------
// Funtion: 更新VPW时间,占空比
// Input  : port VPW CH1
// Output : none
// Return : none  
// Info   : none    CNT值与CCRX 值比较 ,改变CCRX 的值 即是改变高低电平的时间
//------------------------------------------------------------------------------
uint8_t VPWM_Update(VPWM_DATA *vdat )
{
    if( vdat->Index < vdat->Num )
    {
        
				TIMER_CH1CV(TIMER3) = (uint32_t)vdat->Duty[vdat->Index];     //占空比;
			//TIMER_CH1CV(TIMER3) = vdat->Period[vdat->Index]-vdat->Duty[vdat->Index];     //占空比;
				    /* configure the autoreload value */
				TIMER_CAR(TIMER3) = (uint32_t)vdat->Period[vdat->Index]+1;//周期  重装寄存器
			
        //vpwm->TxPPin.Tim->ARR = vdat->Period[vdat->Index];//周期  重装寄存器
			
        vdat->Index++;
        return true;
    }
    return false;
}


//------------------------------------------------------------------------------
// Funtion: 更新PWM时间,占空比
// Input  : port VPW CH0
// Output : none
// Return : none  
// Info   : none    CNT值与CCRX 值比较 ,改变CCRX 的值 即是改变高低电平的时间
//------------------------------------------------------------------------------
uint8_t PWM_Update(VPWM_DATA *vdat )
{
    if( vdat->Index < vdat->Num )
    {
        
				TIMER_CH0CV(TIMER3) = (uint32_t)vdat->Duty[vdat->Index]+0;     //占空比;
			//TIMER_CH1CV(TIMER3) = vdat->Period[vdat->Index]-vdat->Duty[vdat->Index];     //占空比;
				    /* configure the autoreload value */
				TIMER_CAR(TIMER3) = (uint32_t)vdat->Period[vdat->Index]+0;//周期  重装寄存器
			
        //vpwm->TxPPin.Tim->ARR = vdat->Period[vdat->Index];//周期  重装寄存器
			
        vdat->Index++;
        return true;
    }
    return false;
}



//------------------------------------------------------------------------------
// Funtion: VPW PWM 脉宽输出
// Input  : vpwm 硬件对象
//          vdat 脉冲时间及占空比数据
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
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;//是否使能输出
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;//
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;//输出极性
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;//输出死区延时的极性
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;//空闲状态 输出极性 
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;//空闲状态 互补输出极性
    timer_channel_output_config(TIMER3, TIMER_CH_1, &timer_ocinitpara);
	
	

	
	
  /* TIMER1 configuration */
		timer_initpara.period            = Period;//周期
    timer_initpara.prescaler         = 120-1;//1us
		
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    /* CH2 configuration in PWM mode0, duty cycle 75% */
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, CCR_Val);//占空比  即高电平时间
    timer_channel_output_mode_config(TIMER3, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);//必须 ENABLE


	
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);
		
    /* auto-reload preload enable */
    timer_enable(TIMER3);
		timer_flag_clear(TIMER3, TIMER_FLAG_CH1);//清标志
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
				
				timer_flag_clear(TIMER3, TIMER_FLAG_CH1);//清标志
				
        //TIM_ClearFlag(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc);
        if( !VPWM_Update(vdat) )//更新占空比
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
//    TIM_TimeBaseStructure.TIM_Period = Period;////设置在下一个更新事件装入活动的自动重装载寄存器周期的值
//   // TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;//预分频器
//		TIM_TimeBaseStructure.TIM_Prescaler = 84-1;//预分频器 1us
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
// 功能：J1850 - VPW 数据发送函数
// 输入：chbuf - 待发送命令
//       chlen - 发送长度
// 输出：无
// 返回：true 表示发送成功 false 表示发送失败
// 附加：无
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


//test  OK  验证通过
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
     VPW协议初始电平为0V，
在第1帧数据前有一个163至239微秒（us）的高电平表示SOF（帧头即数据开始标志），
接下来以不同长短的高低电平表示二进制数据0或1，其中：
高电平宽度介于34-96us表示“1”，高电平宽度介于96-163us表示“0”， 
低电平宽度介于34-96us表示“0”，低电平宽度介于96-163us表示“1”，
传输时按字节顺序，且每个字节都是高位在前，低位在后的顺序，
高低电平相间用于表示传输的数据，字节与字节之间没有间隔，
传送完一帧数据之后有一个宽度大于239us的低电平表示EOF（帧尾即帧结束标志）。



// Funtion: 电平捕获 ( 捕获的电平被保存在 HighTime LowTime )
// Input  : MaxLenth - 最大接收长度
//          us - 最大等待时间  
//          buf - 接收缓冲区
// Output : DataLenth - 实际接收长度
// Return : 实际捕获耗时
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
        //在输入捕获模式下，当检测到ICx信号上相应的边沿后，计数器的当前值被锁存
        //到捕获/比较寄存器(TIMx_CCRx)中。当发生捕获事件时，相应的CCxIF标志
        //(TIMx_SR寄存器)被置1，如果开放了中断或者DMA操作，则将产生中断或者DMA请
        //求。如果发生捕获事件时CCxIF标志已经为高，那么重复捕获标志CCxOF
        //(TIMx_SR寄存器)被置1。写CCxIF=0可清除CCxIF，或读取存储在TIMx_CCRx
        //寄存器中的捕获数据也可清除CCxIF。写CCxOF=0可清除CCxOF。 
        //以下例子说明如何在TI1输入的上升沿时捕获计数器的值到TIMx_CCR1寄存器中，
        //步骤如下：
        //● 选择有效输入端：TIMx_CCR1必须连接到TI1输入，
        //   所以写入TIMx_CCR1寄存器中的CC1S=01，只要CC1S不为’00’，
        //   通道被配置为输入，并且TIMx_CCR1寄存器变为只读。
        //● 根据输入信号的特点，
        //   配置输入滤波器为所需的带宽(即输入为TIx时，输入滤波器控制位是TIMx_CCMRx
        //   寄存器中的ICxF位)。假设输入信号在最多5个内部时钟周期的时间内抖动，
        //   我们须配置滤波器的带宽长于5个时钟周期；因此我们可以(以fDTS频率)连续采
        //   样8次，以确认在TI1上一次真实的边沿变换，即在TIMx_CCMR1寄存器中写入
        //   IC1F=0011。
        //● 选择TI1通道的有效转换边沿，在TIMx_CCER寄存器中写入CC1P=0(上升沿)。 
        //● 配置输入预分频器。在本例中，我们希望捕获发生在每一个有效的电平转换
        //   时刻，因此预分频器被禁止(写TIMx_CCMR1寄存器的IC1PS=00)。
        //● 设置TIMx_CCER寄存器的CC1E=1，允许捕获计数器的值到捕获寄存器中。
        //● 如果需要，通过设置TIMx_DIER寄存器中的CC1IE位允许相关中断请求，
        //   通过设置TIMx_DIER寄存器中的CC1DE位允许DMA请求。 当发生一个输入捕获时：
        //● 产生有效的电平转换时，计数器的值被传送到TIMx_CCR1寄存
        
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


//低电平时间 转 bit
uint8_t GetLowBit(uint8_t time)
{

		uint8_t Value=0;
	//低电平宽度介于34-96us表示“0”，低电平宽度介于96-163us表示“1”，
	if(time>=34&&time<=96) 
		Value=0;
	else if(time>=96&&time<=163) 
		Value=1;
	
	return Value;
}
//高电平时间 转 bit
uint8_t GetHiBit(uint8_t time)
{
		uint8_t Value=0;
	//高电平宽度介于34-96us表示“1”，高电平宽度介于96-163us表示“0”
	if(time>=34&&time<=96) 
		Value=1;
	else if(time>=96&&time<=163) 
		Value=0;
	
	return Value;
}


//VPWtoBit
/****************************
根据高低电平时间 得到VPW 字节
参数:

返回值:

*****************************/
uint8_t GetVpwBitToByte(uint8_t*iHBuf,uint8_t*iLBuf)
{
	uint8_t Value=0;
	uint8_t j=0;
  //高位在前 低位在后
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
//高电平时间 转 bit
uint8_t GetPWMBit(uint8_t htime,uint8_t ltime)
{
		uint8_t Value=0;

	//一个24us 周期 一个电平 
	//高电平>点电平时 是0 
	//低电平>高电平时 是1
	if(htime>13&&ltime>24) Value=0;
	
	else if(htime>ltime)Value=0;
	else Value=1;
	
	return Value;
}

/****************************
根据高低电平时间 得到pwm 字节
参数:

返回值:

*****************************/
uint8_t GetPWMBitToByte(uint8_t*iHBuf,uint8_t*iLBuf)
{
	uint8_t Value=0;
	uint8_t i=0,j=0;
  //高位在前 低位在后
	//VPWtoBit(iHBuf[0],iHBuf[1]);
	
	for(i=0;i<8;i++)
	{
		j=GetPWMBit(iHBuf[i],iLBuf[i]);
		Value+=j<<(7-i);
	}
	

	return Value;
}



//------------------------------------------------------------------------------
// Funtion: VPW PWM 脉宽输出
// Input  : vpwm 硬件对象
//          vdat 脉冲时间及占空比数据
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
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;//是否使能输出
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;//
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;//输出极性
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;//输出死区延时的极性
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;//空闲状态 输出极性 
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;//空闲状态 互补输出极性
    timer_channel_output_config(TIMER3, TIMER_CH_1, &timer_ocinitpara);
	
	
  /* TIMER1 configuration */
		timer_initpara.period            = Period;//周期
    timer_initpara.prescaler         = 120-1;//1us
		
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    /* CH2 configuration in PWM mode0, duty cycle 75% */
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, CCR_Val);//占空比  即高电平时间
    //timer_channel_output_mode_config(TIMER3, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);//必须 ENABLE


	
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);
		
		//必须接收关闭中断
		//timer_interrupt_disable(TIMER1,TIMER_INT_CH1|TIMER_INT_UP); //TIME1 接收中断关闭
		timer_disable(TIMER1);//必须关闭接收
		
		timer_channel_output_mode_config(TIMER3, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    /* auto-reload preload enable */
    timer_enable(TIMER3);
		timer_flag_clear(TIMER3, TIMER_FLAG_CH1);//清标志
		while(vdat->State!=VPWM_IDLE)
    {
        //if( TIM_GetFlagStatus(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc) == RESET )
				if(RESET == timer_flag_get(TIMER3, TIMER_FLAG_CH1))	//TIMER_INT_FLAG_CH1  TIMER_FLAG_CH1
        {
					//TIM_IT_CC3
           continue;
        }
				
				timer_flag_clear(TIMER3, TIMER_FLAG_CH1);//清标志
				
        //TIM_ClearFlag(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc);
        if( !VPWM_Update(vdat) )//更新占空比
        {
						timer_disable(TIMER3);
//						gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_7);
//						gpio_bit_reset(GPIOB,GPIO_PIN_7);
            vdat->State = VPWM_IDLE;            
        }
    }
		
		//等待接收
		//必须接收关闭中断
		iJ1850EofFlag=0;
		iReCount=0;//

		//timer_interrupt_enable(TIMER1,TIMER_INT_CH1|TIMER_INT_UP);
		timer_enable(TIMER1);//开启接收
		GetTimer6Cnt();
		
		

		return iRet;
}


//------------------------------------------------------------------------------
// Funtion: PWM 脉宽输出
// Input  : vpwm 硬件对象  TIMER3, TIMER_CH_0
//          vdat 脉冲时间及占空比数据
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
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;//是否使能输出
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;//
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;//输出极性
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;//输出死区延时的极性
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;//空闲状态 输出极性 
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;//空闲状态 互补输出极性
    timer_channel_output_config(TIMER3, TIMER_CH_0, &timer_ocinitpara);
	
	
  /* TIMER1 configuration */
		timer_initpara.period            = Period;//周期
    timer_initpara.prescaler         = 120-1;//1us
		
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    /* CH2 configuration in PWM mode0, duty cycle 75% */
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_0, CCR_Val);//占空比  即高电平时间
    timer_channel_output_mode_config(TIMER3, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER3, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);//必须 ENABLE


	
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);
		
		//必须接收关闭中断
		//timer_interrupt_disable(TIMER1,TIMER_INT_CH1|TIMER_INT_UP); //TIME1 接收中断关闭
		timer_disable(TIMER1);//开启
		
    /* auto-reload preload enable */
    timer_enable(TIMER3);
		Delay_us(10);
		timer_flag_clear(TIMER3, TIMER_FLAG_CH0);//清标志
		while(vdat->State!=VPWM_IDLE)
    {
        //if( TIM_GetFlagStatus(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc) == RESET )
				if(RESET == timer_flag_get(TIMER3, TIMER_FLAG_CH0))	//TIMER_INT_FLAG_CH1  TIMER_FLAG_CH1
        {
					//TIM_IT_CC3
           continue;
        }
				Delay_us(1);
				timer_flag_clear(TIMER3, TIMER_FLAG_CH0);//清标志
				
        //TIM_ClearFlag(vpwm->TxPPin.Tim,vpwm->TxPPin.TimFlagCc);
        if( !PWM_Update(vdat) )//更新占空比
        {
						timer_disable(TIMER3);
//						gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_7);
//						gpio_bit_reset(GPIOB,GPIO_PIN_7);
            vdat->State = VPWM_IDLE;            
        }
    }
		
		//等待接收
		//必须接收关闭中断
		iJ1850EofFlag=0;
		iReCount=0;//
		iHBitSum=0;
		iLBitSum=0;

		//timer_interrupt_enable(TIMER1,TIMER_INT_CH1|TIMER_INT_UP);
		timer_enable(TIMER1);//开启
		GetTimer6Cnt();
	
		//timer_disable(TIMER1);//开启
		//timer_interrupt_disable(TIMER1,TIMER_INT_CH1|TIMER_INT_UP); //TIME1 接收中断关闭
		return iRet;
}
//------------------------------------------------------------------------------
// 功能：J1850 - PWM 数据发送 接收函数
// 输入：chbuf - 待发送命令
//       chlen - 发送长度
// 输出：无
// 返回：接收帧长度
//------------------------------------------------------------------------------
u16 J1850_PWM_SendFrame( unsigned char *chbuf, unsigned char chlen )
{
		u16 iRet=0;
    //data convert

//	static unsigned char dat[]={0x61, 0x6A, 0xF1, 0x01, 0x00, 0x0A};
//	chlen=sizeof(dat);
	//IFRType=0X80;
		    //data convert
		// chbuf[chlen]=0xF1;//模拟ECU 最后一个字节是F1
	IFRType=0X80;//要加上这个才能模拟 DT 设备
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
		iRet=chlen+1;//下一帧地址
		
		return iRet;
}

//------------------------------------------------------------------------------
// 功能：J1850 - VPW 数据发送 接收函数
// 输入：chbuf - 待发送命令
//       chlen - 发送长度
// 输出：无
// 返回：接收帧长度
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
    iRet=chlen+1;//下一帧地址
    return iRet;
}

