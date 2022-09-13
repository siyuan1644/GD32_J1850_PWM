#ifndef __VPWM_H
#define __VPWM_H
#include "gd32C10x.h"
#include "stdbool.h"

#define PWMINPUT

#define USTIMECLK  120//SystemCoreClock/1000000
#define MSTIMECLK  120000  

#define PWM_NOEOF       1
#define PWM_NOSOF       2
#define PWM_ONLYDATA    3

typedef enum
{
    VPWM_ERR = 0, VPWM_SOF = 1, VPWM_BIT1 = 2, VPWM_BIT0 = 4, VPWM_EOF = 8,
    VPWM_BIT00 = 0x10, VPWM_BIT01 = 0x20, VPWM_BIT10 = 0x40, VPWM_BIT11 = 0x80
}   VPWM_BIT;

typedef enum{ VPWM_IDLE=0, VPW_OUTPUT, VPW_INPUT, PWM_OUTPUT, PWM_INPUT } VPWM_STATUS;

typedef struct
{
    VPWM_STATUS State;
    uint32_t Index;
    uint32_t Num;
    uint16_t Period[ 120 ];
    uint16_t Duty[ 120 ];
    uint16_t HighTime[ 120 ];
    uint16_t LowTime[ 120 ];
		uint8_t  Offset;//误差100ns
	
}   VPWM_DATA;   

//typedef struct
//{
//    GPIO_TypeDef* Port;
//    uint32_t      PortClk;
//    uint16_t      Pin;
//    uint8_t       PinSource;
//    
//    TIM_TypeDef*  Tim;
//    uint32_t      TimClk;
//    uint32_t      Af;
//    
//    uint8_t       TimCh;//ch 1~4  和引脚对应
//    uint16_t      TimItCc;
//    uint16_t      TimFlagCc;
//}   VPWM_PIN;

//typedef struct
//{
//    VPWM_PIN RxPin;  
//    VPWM_PIN TxPPin;
//    VPWM_PIN TXNPin;
//}   VPWM;



uint8_t PWM_Test(void);
uint8_t VPW_Test(void);

uint32_t VPWM_Input_Init(void);


//VPWM_Input  //电平捕获函数


//初始化
void Time5_VPWM_Init(void);
char VPW_SendEx( unsigned char *chbuf, unsigned char chlen );
char PWM_SendEx( unsigned char *chbuf, unsigned char chlen, unsigned char IFRType,unsigned char *FrameID );


//j1850 定义
void J1850VPW_Init(void);
void J1850PWM_Init(void);

uint8_t J1850_VPW_Test(void);

uint8_t PWM_CheckBus(void);
uint8_t VPW_CheckBus(void);
uint8_t CheckCRC (uint8_t *pReqBuff,uint32_t ReqBufLen);//crc 校验
uint8_t  CheckIfCRC (uint8_t *pReqBuff,uint32_t ReqBufLen);
uint8_t GetVpwBitToByte(uint8_t*iHBuf,uint8_t*iLBuf);
uint8_t GetPWMBitToByte(uint8_t*iHBuf,uint8_t*iLBuf);

//j1850 定义
void J1850VPW_Init(void);
void J1850PWM_Init(void);
uint16_t J1850_VPW_SendFrame( unsigned char *chbuf, unsigned char chlen);
uint16_t J1850_PWM_SendFrame( unsigned char *chbuf, unsigned char chlen);
#endif


