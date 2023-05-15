/********************************** (C) COPYRIGHT *******************************
CH552e模拟USB多媒体键盘，旋转编码器为  EC16
2022/7/9 
HID报告描述符参考文章：https://www.cnblogs.com/AlwaysOnLines/p/4552840.html
参考书籍：圈圈教你玩USB，HID用途表1.12，HID1.11协议
以下很多都是复制大佬的程序，加了些自己的理解注释
*******************************************************************************/

#include "CH554.H"                                                      
#include "DEBUG.H"
#include"HIDDescriptor.h"
#include <string.h>
#include <stdio.h>

#define THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE
#define CapsLockLED 0x02

UINT8X  Ep0Buffer[8>(THIS_ENDP0_SIZE+2)?8:(THIS_ENDP0_SIZE+2)] _at_ 0x0000;    //端点0 OUT&IN缓冲区，必须是偶地址
UINT8X  Ep1Buffer[64>(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x000a;  //端点1 IN缓冲区,必须是偶地址
UINT8X  Ep2Buffer[64<(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x0050;  //端点2 IN缓冲区,必须是偶地址
UINT8   SetupReq,SetupLen,Ready,Count,FLAG,UsbConfig,LED_VALID;
PUINT8  pDescr;   //USB配置标志
USB_SETUP_REQ   SetupReqBuf;  //暂存Setup包
#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)
#define DEBUG 0
#pragma  NOAREGS

 //定义的4个按键以及EC16的A、B脚
sbit key1=P1^4;
sbit key2=P1^5;
sbit key3=P1^6;
sbit key4=P1^7;
sbit EC11_A = P3 ^ 1;
sbit EC11_B = P3 ^ 0;

UINT8 EC11_State = 1;//EC11现在的状态
UINT8 EC11_PState = 1;//EC11过去的状态
UINT8 EC11_KeyState;//EC11实现按键功能缓存值
UINT8 T0RH = 0;	//T0高8位重载值
UINT8 T0RL = 0;	//T0低8位重载值
UINT8 KeyState[4] = {1,1,1,1}; //按键状态
UINT8 BackState[4] = {1,1,1,1}; //按键上一次的状态
unsigned long pdata TimeThr[4] = {1000, 1000, 1000, 1000};
unsigned long pdata KeyDownTime[4]= {0, 0, 0, 0};
UINT8C key_code_map[6] = {
	0x31,0x32,0x33,0x34,0x35,0x36         //按键1,按键2,按键3,按键4,按键5,按键6
};
/*普通键盘数据*/
UINT8 HIDKey[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
/*多媒体按键数据*/
UINT8 HIDKeyMUL[4] = {0x00,0x00,0x00,0x00};

/*******************************************************************************
* Function Name  : USBDeviceInit()
* Description    : USB设备模式配置,设备模式启动，收发端点配置，中断开启
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
	  IE_USB = 0;
	  USB_CTRL = 0x00; //先设定USB设备模式
    UEP0_DMA = Ep0Buffer; //端点0数据传输地址
    UEP1_DMA = Ep1Buffer; //端点1数据传输地址
	UEP2_DMA = Ep2Buffer; //端点2数据传输地址
    UEP4_1_MOD = ~(bUEP4_RX_EN | bUEP4_TX_EN |bUEP1_RX_EN | bUEP1_BUF_MOD) | bUEP4_TX_EN;//端点1单64字节收发缓冲区,端点0收发
	UEP2_3_MOD = UEP2_3_MOD & ~bUEP2_BUF_MOD | bUEP2_TX_EN;
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK; //OUT事务返回ACK，IN事务返回NAK
    UEP1_CTRL = bUEP_T_TOG | UEP_T_RES_NAK;	//端点1手动翻转同步标志位，IN事务返回NAK
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK; //端点2自动翻转同步标志位，IN事务返回NAK
		
	  USB_DEV_AD = 0x00;
	  UDEV_CTRL = bUD_PD_DIS; // 禁止DP/DM下拉电阻
	  USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
	  UDEV_CTRL |= bUD_PORT_EN; // 允许USB端口
	  USB_INT_FG = 0xFF; // 清中断标志
	  USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
	  IE_USB = 1;
}
/*******************************************************************************
* Function Name  : Enp1IntIn()
* Description    : USB设备模式端点1的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp1IntIn( )
{
    memcpy( Ep1Buffer, HIDKey, sizeof(HIDKey)); //加载上传数据
    UEP1_T_LEN = sizeof(HIDKey); //上传数据长度
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //有数据时上传数据并应答ACK
}
/*******************************************************************************
* Function Name  : Enp2IntIn()
* Description    : USB设备模式端点2的中断上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp2IntIn( )
{
    memcpy( Ep2Buffer, HIDKeyMUL, sizeof(HIDKeyMUL)); //加载上传数据
    UEP2_T_LEN = sizeof(HIDKeyMUL); //上传数据长度
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //有数据时上传数据并应答ACK
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH559USB中断处理函数
*******************************************************************************/
void    DeviceInterrupt( void ) interrupt INT_NO_USB using 1                      //USB中断服务程序,使用寄存器组1
{
    UINT8 len;
    if(UIF_TRANSFER)                                                            //USB传输完成标志
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2:                                                  //endpoint 2# 中断端点上传
            UEP2_T_LEN = 0;                                                     //预使用发送长度一定要清空
//            UEP1_CTRL ^= bUEP_T_TOG;                                          //如果不设置自动翻转则需要手动翻转
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
			FLAG = 1; 															/*传输完成标志*/
            break;
        case UIS_TOKEN_IN | 1:                                                  //endpoint 1# 中断端点上传
            UEP1_T_LEN = 0;                                                     //预使用发送长度一定要清空
//            UEP2_CTRL ^= bUEP_T_TOG;                                          //如果不设置自动翻转则需要手动翻转
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
            FLAG = 1;                                                           /*传输完成标志*/
            break;
        case UIS_TOKEN_SETUP | 0:                                                //SETUP事务
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    // 限制总长度
                }
                len = 0;                                                        // 默认为成功并且上传0长度
                SetupReq = UsbSetupBuf->bRequest;								
                if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )/* HID类命令 */
                {
									switch( SetupReq ) 
									{
										case 0x01://GetReport
												 break;
										case 0x02://GetIdle
												 break;	
										case 0x03://GetProtocol
												 break;				
										case 0x09://SetReport										
												 break;
										case 0x0A://SetIdle
												 break;	
										case 0x0B://SetProtocol
												 break;
										default:
												 len = 0xFF;  								 					            /*命令不支持*/					
												 break;
								  }	
                }
                else
                {//标准请求
                    switch(SetupReq)                                        //请求码
                    {
                    case USB_GET_DESCRIPTOR:
                        switch(UsbSetupBuf->wValueH)
                        {
                        case 1:                                             //设备描述符
                            pDescr = DevDesc;                               //把设备描述符送到要发送的缓冲区
                            len = sizeof(DevDesc);
                            break;
                        case 2:                                             //配置描述符
                            pDescr = CfgDesc;                               //把设备描述符送到要发送的缓冲区
                            len = sizeof(CfgDesc);
                            break;
                        case 0x22:                                          //报表描述符
                            if(UsbSetupBuf->wIndexL == 0)                   //接口0报表描述符
                            {
                                pDescr = KeyRepDesc;                        //数据准备上传
                                len = sizeof(KeyRepDesc);								
                            }
							if(UsbSetupBuf->wIndexL == 1)                   //接口0报表描述符
                            {
                                pDescr = KeyMULRepDesc;                        //数据准备上传
                                len = sizeof(KeyMULRepDesc);
                                Ready = 1;                                  //如果有更多接口，该标准位应该在最后一个接口配置完成后有效
                                //IE_UART1 = 1;//开启串口中断															
															
                            }
                            else
                            {
                                len = 0xff;                                 //本程序只有2个接口，这句话正常不可能执行
                            }
                            break;
                        default:
                            len = 0xff;                                     //不支持的命令或者出错
                            break;
                        }
                        if ( SetupLen > len )
                        {
                            SetupLen = len;    //限制总长度
                        }
                        len = SetupLen >= 8 ? 8 : SetupLen;                  //本次传输长度
                        memcpy(Ep0Buffer,pDescr,len);                        //加载上传数据
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;                     //暂存USB设备地址
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if ( SetupLen >= 1 )
                        {
                            len = 1;
                        }
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:                                            //Clear Feature
                        if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// 端点
                        {
                            switch( UsbSetupBuf->wIndexL )
                            {
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                break;
                            case 0x01:
                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF;                                            // 不支持的端点
                                break;
                            }
                        }
                        else
                        {
                            len = 0xFF;                                                // 不是端点不支持
                        }
                        break;
                    case USB_SET_FEATURE:                                              /* Set Feature */
                        if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 )             /* 设置设备 */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                            {
                                if( CfgDesc[ 7 ] & 0x20 )
                                {
                                    /* 设置唤醒使能标志 */
                                }
                                else
                                {
                                    len = 0xFF;                                        /* 操作失败 */
                                }
                            }
                            else
                            {
                                len = 0xFF;                                            /* 操作失败 */
                            }
                        }
                        else if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x02 )        /* 设置端点 */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                            {
                                switch( ( ( UINT16 )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF;                               //操作失败
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF;                                   //操作失败
                            }
                        }
                        else
                        {
                            len = 0xFF;                                      //操作失败
                        }
                        break;
                    case USB_GET_STATUS:
                        Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if ( SetupLen >= 2 )
                        {
                            len = 2;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                    default:
                        len = 0xff;                                           //操作失败
                        break;
                    }
                }
            }
            else
            {
                len = 0xff;                                                   //包长度错误
            }
            if(len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
            }
            else if(len <= 8)                                                //上传数据或者状态阶段返回0长度包
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1，返回应答ACK
            }
            else
            {
                UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1,返回应答ACK
            }
            break;
        case UIS_TOKEN_IN | 0:                                               //endpoint0 IN
            switch(SetupReq)
            {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= 8 ? 8 : SetupLen;                          //本次传输长度
                memcpy( Ep0Buffer, pDescr, len );                            //加载上传数据
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;                                     //同步标志位翻转
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0;                                              //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0:  // endpoint0 OUT
            len = USB_RX_LEN;
            if((SetupReq == 0x09)&& (len == 1))
            {
              LED_VALID = Ep0Buffer[0];							
            }
            else if((SetupReq == 0x09) && (len == 8)){//SetReport						 
            }							
            UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
            UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA0,返回应答ACK
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0;                                                 //写0清空中断
    }
    if(UIF_BUS_RST)                                                       //设备模式USB总线复位中断
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                 //清中断标志
    }
    if (UIF_SUSPEND)                                                     //USB总线挂起/唤醒完成
    {
        UIF_SUSPEND = 0;
    }
    else {                                                               //意外的中断,不可能发生的情况
        USB_INT_FG = 0xFF;                                               //清中断标志
    }
}
/**键盘HID值上传函数**/
void HIDValueHandle1()
{
    //TR0 = 0; //发送前关定时器中断
	FLAG = 0; //清空USB中断传输完成标志，准备发送按键按下数据
	Enp1IntIn(); //USB设备模式端点1的中断上传
	while(FLAG == 0); //等待USB中断数据传输完成
	FLAG = 0; //清空USB中断传输完成标志，准备发送按键抬起数据	
	memset(&HIDKey[0],0,8);	//把HIDkey置0，发送0表示按键抬起
	Enp1IntIn(); //USB设备模式端点1的中断上传		
	while(FLAG == 0); //等待USB中断数据传输完成
	//TR0 = 1; //发送完打开定时器中断		
}
/**多媒体按键HID值上传函数**/
void HIDValueHandle2()
{
    //TR0 = 0; //发送前关定时器中断
	FLAG = 0; //清空USB中断传输完成标志，准备发送按键按下数据
	Enp2IntIn(); //USB设备模式端点2的中断上传
	while(FLAG == 0); //等待USB中断数据传输完成
	FLAG = 0; //清空USB中断传输完成标志，准备发送按键抬起数据	
	memset(&HIDKeyMUL[0],0,4); //把HIDKeyMUL置0，发送0表示按键抬起
	Enp2IntIn(); //USB设备模式端点2的中断上传		
	while(FLAG == 0); //等待USB中断数据传输
	//TR0 = 1; //发送完打开定时器中断		
}
/**按键行为函数**/
/*找到按键的HID值自由发挥部分*/
/*普通按键
  例如ctrl + c :
  HIDKey[0] = 0x01;// ctrl
  HIDKey[2] = 0x06;// c
  if(Ready) //枚举成功
	{
	    HIDValueHandle1();
	}
*/
/* 按键动作函数 */
void KeyAction(unsigned char keyCode)
{   
	if(keyCode == 0x31)//按键1
	{
        HIDKeyMUL[0] = 0x10;  //打开媒体播放器
        if (Ready) //枚举成功
        {
            HIDValueHandle2(); //多媒体按键HID值上传
        }
	}
	if(keyCode == 0x32)//按键2
	{
		HIDKeyMUL[0] = 0x80;  //上一曲
		if(Ready) //枚举成功
        {
            HIDValueHandle2(); //多媒体按键HID值上传
        }
	}
	if(keyCode == 0x33)	//按键3	
	{
		HIDKeyMUL[1] = 0x01;  //下一曲
		if(Ready) //枚举成功
        {
            HIDValueHandle2(); //普通按键HID值上传
        }
	}
	if(keyCode == 0x34)	//按键4	
	{
		HIDKeyMUL[0] = 0x40; //播放/暂停

		if(Ready) //枚举成功
        {
            HIDValueHandle2(); //多媒体按键HID值上传
        }
	}
	if(keyCode == 0x35)	//按键5	
	{
		HIDKeyMUL[0] = 0x02; //音量+

		if(Ready) //枚举成功
        {
            HIDValueHandle2(); //多媒体按键HID值上传
        }
	}
	if(keyCode == 0x36)	//按键6	
	{
		HIDKeyMUL[0] = 0x01; //音量-

		if(Ready) //枚举成功
        {
            HIDValueHandle2(); //多媒体按键HID值上传
        }
	}
}
/**按键驱动**/
void KeyDrive()
{
	unsigned char j;

	for(j=0;j<4;j++)
	{
		if(KeyState[j] != BackState[j])
		{
	    	if(BackState[j] != 0)
			{
				KeyAction(key_code_map[j]);
			}
			BackState[j] = KeyState[j]; 
		}

		if(KeyDownTime[j] > 0)
		{
			if(KeyDownTime[j] >= TimeThr[j])
			{
				KeyAction(key_code_map[j]);
				TimeThr[j] += 100;
			} 
		}
		else
		{
			TimeThr[j] = 1000;
		}		
	}
}
/* EC11驱动 */
void EC11Drive()
{
    if (EC11_State != EC11_PState)//EC11状态改版进入判断
    {
        if (EC11_PState == 1)
        {
            EC11_State = 1;
            KeyAction(key_code_map[EC11_KeyState]);
        }
        EC11_PState = EC11_State;//把当前EC11状态存为过去状态  为了下一次判断
    }
    if (EC11_A == 1) IE_EX |= bIP_GPIO;//当EC11_A高电平时打开GPIO中断
}
/**T0配置函数**/
void ConfigT0(UINT8 ms)
{
    unsigned long tmp = 0;

	tmp = 12000000/12;
	tmp = (tmp * ms)/1000;
	tmp = 65536 - tmp;
	tmp = tmp + 1;
    T0RH = (UINT8)(tmp >> 8);
	T0RL = (UINT8)tmp;

	TMOD = ( TMOD & ~( bT0_GATE | bT0_CT | bT0_M1 ) ) | bT0_M0;//* 模式1，16 位定时/计数器
	TH0 = T0RH;
	TL0 = T0RL;
	TF0 = 0;
	ET0 = 1;
	TR0 = 1;
}

/***************************************************主函数******************************************************/
main()
{
    CfgFsys(); //CH552时钟选择12M配置
    mDelaymS(5); //修改主频等待内部晶振稳定,必加	
	ConfigT0(10); //配置10ms T0中断
	USBDeviceInit(); //USB设备模式初始化
   
    UEP1_T_LEN = 0; //预使用发送长度一定要清空
  	UEP2_T_LEN = 0;	//清空端点2发送长度
    FLAG = 0;//清空USB中断传输完成标志
    Ready = 0;//清空USB枚举完成标志
	LED_VALID = 1;//给一个默认值

    //GPIO中断配置   配置EC11_A下降沿或者低电平触发中断
    IE_EX |= bIP_GPIO;//允许GPIO中断
    GPIO_IE |= bIE_P3_1_LO;//P3.1式低电平有效，边沿模式下降沿有效


    //用到的IO初始化，复位后IO 状态为 开漏输入输出，有上拉
	key1 = 1;
	key2 = 1;
	key3 = 1;
	key4 = 1;
    EC11_A = 1;
    EC11_B = 1;

    EA = 1; //允许单片机中断
    mDelaymS(10);
	while(1)
	{
	    KeyDrive();	//按键驱动
        EC11Drive();//EC11驱动
	}
}
/**按键扫描函数**/
void KeyScan()
{
	UINT8 i;
	static UINT8 keybuffer[4] = {0xFF,0xFF,0xFF,0xFF};

	keybuffer[0] = (keybuffer[0] <<1) | key1;//按键扫描
	keybuffer[1] = (keybuffer[1] <<1) | key2;
	keybuffer[2] = (keybuffer[2] <<1) | key3;
	keybuffer[3] = (keybuffer[3] <<1) | key4;
	
	for(i=0;i<4;i++)
	{
		if((keybuffer[i] & 0x0F) == 0x00)
		{
			KeyState[i] = 0;
			if(i>=3)
			    break;
			KeyDownTime[i] += 10;  //按下的持续时间累加
		}
		else if((keybuffer[i] & 0x0F) == 0x0F)
		{
			KeyState[i] = 1;
			if(i>=3)
			    break;
			KeyDownTime[i] = 0;   //按下的持续时间清零
		}
	} 

}
/**T0中断函数**/
void InterruptTimer0() interrupt INT_NO_TMR0 using 1 
{
	TH0 = T0RH;
	TL0 = T0RL;

	KeyScan(); //按键扫描
}
/*GPIO中断函数**/
void InterruptGPIO() interrupt INT_NO_GPIO
{
    IE_EX &= (~bIP_GPIO);//关闭GPIO中断  防止不断触发
    if (EC11_B == 1) EC11_KeyState = 4;//正转
    else EC11_KeyState = 5;//反转
    EC11_State = 0;//表示触发旋转编码器
}
