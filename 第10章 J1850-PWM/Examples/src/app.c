

/*
	J1850-VPW 测试，OBD2
  
	
	siyuan 2022-09-07
*/

#include "drv_usb_hw.h"
#include "cdc_acm_core.h"

#include	<stdio.h>
#include "can.h"
#include "bsp.h"
#include "timer.h"
#include "usart.h"	 
#include "vpwm.h"

usb_core_driver cdc_acm;

extern  uint16_t iRxUsbLen;//接收Usb数据长度
extern uint16_t iRxUsbFlag;//接收完成标记  0x80接收完成

void GetKLineTime(void);

void EcuSendData(void);
u8 RecAdd(void);
u8 Wait5BpsAdd(void);
/*!
    \brief      main routine will construct a USB mass storage device
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{

		//uint8_t SendData1[100]={0x09,0X02,0X10,0x03,0x00,0x00,0x00,0x00,0x00};
		
		//uint8_t SendData2[100]={0x0F,0X02,0X10,0x01,0x02,0x03,0x04,0x05,0x00};
//		uint8_t i=0;
		iRxUsbLen=0;
		iRxUsbFlag=0;
		
    usb_rcu_config();

    usb_timer_init();

    usbd_init (&cdc_acm, USB_CORE_ENUM_FS, &cdc_desc, &cdc_class);

    usb_intr_config();
    
#ifdef USE_IRC48M
    /* CTC peripheral clock enable */
    rcu_periph_clock_enable(RCU_CTC);

    /* CTC configure */
    ctc_config();

    while (ctc_flag_get(CTC_FLAG_CKOK) == RESET) {
    }
#endif

		//
	//StCanInitTest();//CAN test
	//	FdCanInitTest();//FD CAN
		
		TIM5_config();//用于定时
		TIM6_config();//用于测量时间
		Led_Init();//初始化 普通IO

		
		iHBitSum=0;//
		iLBitSum=0;//
		//初始化变量
		iKDataMode=0;
		iPartValue=0;
		iEcuFlag=0;//采数模式
		
		Adc_Init();//初始化电压采样
		

		J1850PWM_Init();//PWM 初始化
		uint8_t dat[]={0x61,0x6A,0xF1,0x01,0x00,0x0a};
		uint8_t dat1[]={0x41,0x6B,0x10,0x41,0x00,0x56,0xF9,0x0A,0x99,0xF9};

    /* main loop */
  while (1) 
	{

		J1850_PWM_SendFrame(dat,sizeof(dat));
		Delay_ms(500);
		J1850_PWM_SendFrame(dat1,sizeof(dat1));
		Delay_ms(500);
//		//TEST
//	 if (USBD_CONFIGURED == cdc_acm.dev.cur_status) 
//	 {
//      if (0U == cdc_acm_check_ready(&cdc_acm)) 
//			{
//         cdc_acm_data_receive(&cdc_acm);		
//					// 准备接收数据 							           
//			}
//			else 
//			{							//发送数据
//        //cdc_acm_data_send(&cdc_acm);
//      }
//		} 
//		Delay_us(10);
//		continue;				
	
    }
}



