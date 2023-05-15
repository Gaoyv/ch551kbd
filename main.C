/********************************** (C) COPYRIGHT *******************************
CH552eģ��USB��ý����̣���ת������Ϊ  EC16
2022/7/9 
HID�����������ο����£�https://www.cnblogs.com/AlwaysOnLines/p/4552840.html
�ο��鼮��ȦȦ������USB��HID��;��1.12��HID1.11Э��
���ºܶ඼�Ǹ��ƴ��еĳ��򣬼���Щ�Լ�������ע��
*******************************************************************************/

#include "CH554.H"                                                      
#include "DEBUG.H"
#include"HIDDescriptor.h"
#include <string.h>
#include <stdio.h>

#define THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE
#define CapsLockLED 0x02

UINT8X  Ep0Buffer[8>(THIS_ENDP0_SIZE+2)?8:(THIS_ENDP0_SIZE+2)] _at_ 0x0000;    //�˵�0 OUT&IN��������������ż��ַ
UINT8X  Ep1Buffer[64>(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x000a;  //�˵�1 IN������,������ż��ַ
UINT8X  Ep2Buffer[64<(MAX_PACKET_SIZE+2)?64:(MAX_PACKET_SIZE+2)] _at_ 0x0050;  //�˵�2 IN������,������ż��ַ
UINT8   SetupReq,SetupLen,Ready,Count,FLAG,UsbConfig,LED_VALID;
PUINT8  pDescr;   //USB���ñ�־
USB_SETUP_REQ   SetupReqBuf;  //�ݴ�Setup��
#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)
#define DEBUG 0
#pragma  NOAREGS

 //�����4�������Լ�EC16��A��B��
sbit key1=P1^4;
sbit key2=P1^5;
sbit key3=P1^6;
sbit key4=P1^7;
sbit EC11_A = P3 ^ 1;
sbit EC11_B = P3 ^ 0;

UINT8 EC11_State = 1;//EC11���ڵ�״̬
UINT8 EC11_PState = 1;//EC11��ȥ��״̬
UINT8 EC11_KeyState;//EC11ʵ�ְ������ܻ���ֵ
UINT8 T0RH = 0;	//T0��8λ����ֵ
UINT8 T0RL = 0;	//T0��8λ����ֵ
UINT8 KeyState[4] = {1,1,1,1}; //����״̬
UINT8 BackState[4] = {1,1,1,1}; //������һ�ε�״̬
unsigned long pdata TimeThr[4] = {1000, 1000, 1000, 1000};
unsigned long pdata KeyDownTime[4]= {0, 0, 0, 0};
UINT8C key_code_map[6] = {
	0x31,0x32,0x33,0x34,0x35,0x36         //����1,����2,����3,����4,����5,����6
};
/*��ͨ��������*/
UINT8 HIDKey[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
/*��ý�尴������*/
UINT8 HIDKeyMUL[4] = {0x00,0x00,0x00,0x00};

/*******************************************************************************
* Function Name  : USBDeviceInit()
* Description    : USB�豸ģʽ����,�豸ģʽ�������շ��˵����ã��жϿ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
	  IE_USB = 0;
	  USB_CTRL = 0x00; //���趨USB�豸ģʽ
    UEP0_DMA = Ep0Buffer; //�˵�0���ݴ����ַ
    UEP1_DMA = Ep1Buffer; //�˵�1���ݴ����ַ
	UEP2_DMA = Ep2Buffer; //�˵�2���ݴ����ַ
    UEP4_1_MOD = ~(bUEP4_RX_EN | bUEP4_TX_EN |bUEP1_RX_EN | bUEP1_BUF_MOD) | bUEP4_TX_EN;//�˵�1��64�ֽ��շ�������,�˵�0�շ�
	UEP2_3_MOD = UEP2_3_MOD & ~bUEP2_BUF_MOD | bUEP2_TX_EN;
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK; //OUT���񷵻�ACK��IN���񷵻�NAK
    UEP1_CTRL = bUEP_T_TOG | UEP_T_RES_NAK;	//�˵�1�ֶ���תͬ����־λ��IN���񷵻�NAK
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK; //�˵�2�Զ���תͬ����־λ��IN���񷵻�NAK
		
	  USB_DEV_AD = 0x00;
	  UDEV_CTRL = bUD_PD_DIS; // ��ֹDP/DM��������
	  USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; // ����USB�豸��DMA�����ж��ڼ��жϱ�־δ���ǰ�Զ�����NAK
	  UDEV_CTRL |= bUD_PORT_EN; // ����USB�˿�
	  USB_INT_FG = 0xFF; // ���жϱ�־
	  USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
	  IE_USB = 1;
}
/*******************************************************************************
* Function Name  : Enp1IntIn()
* Description    : USB�豸ģʽ�˵�1���ж��ϴ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp1IntIn( )
{
    memcpy( Ep1Buffer, HIDKey, sizeof(HIDKey)); //�����ϴ�����
    UEP1_T_LEN = sizeof(HIDKey); //�ϴ����ݳ���
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //������ʱ�ϴ����ݲ�Ӧ��ACK
}
/*******************************************************************************
* Function Name  : Enp2IntIn()
* Description    : USB�豸ģʽ�˵�2���ж��ϴ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp2IntIn( )
{
    memcpy( Ep2Buffer, HIDKeyMUL, sizeof(HIDKeyMUL)); //�����ϴ�����
    UEP2_T_LEN = sizeof(HIDKeyMUL); //�ϴ����ݳ���
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //������ʱ�ϴ����ݲ�Ӧ��ACK
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH559USB�жϴ�������
*******************************************************************************/
void    DeviceInterrupt( void ) interrupt INT_NO_USB using 1                      //USB�жϷ������,ʹ�üĴ�����1
{
    UINT8 len;
    if(UIF_TRANSFER)                                                            //USB������ɱ�־
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2:                                                  //endpoint 2# �ж϶˵��ϴ�
            UEP2_T_LEN = 0;                                                     //Ԥʹ�÷��ͳ���һ��Ҫ���
//            UEP1_CTRL ^= bUEP_T_TOG;                                          //����������Զ���ת����Ҫ�ֶ���ת
            UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //Ĭ��Ӧ��NAK
			FLAG = 1; 															/*������ɱ�־*/
            break;
        case UIS_TOKEN_IN | 1:                                                  //endpoint 1# �ж϶˵��ϴ�
            UEP1_T_LEN = 0;                                                     //Ԥʹ�÷��ͳ���һ��Ҫ���
//            UEP2_CTRL ^= bUEP_T_TOG;                                          //����������Զ���ת����Ҫ�ֶ���ת
            UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //Ĭ��Ӧ��NAK
            FLAG = 1;                                                           /*������ɱ�־*/
            break;
        case UIS_TOKEN_SETUP | 0:                                                //SETUP����
            len = USB_RX_LEN;
            if(len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    // �����ܳ���
                }
                len = 0;                                                        // Ĭ��Ϊ�ɹ������ϴ�0����
                SetupReq = UsbSetupBuf->bRequest;								
                if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )/* HID������ */
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
												 len = 0xFF;  								 					            /*���֧��*/					
												 break;
								  }	
                }
                else
                {//��׼����
                    switch(SetupReq)                                        //������
                    {
                    case USB_GET_DESCRIPTOR:
                        switch(UsbSetupBuf->wValueH)
                        {
                        case 1:                                             //�豸������
                            pDescr = DevDesc;                               //���豸�������͵�Ҫ���͵Ļ�����
                            len = sizeof(DevDesc);
                            break;
                        case 2:                                             //����������
                            pDescr = CfgDesc;                               //���豸�������͵�Ҫ���͵Ļ�����
                            len = sizeof(CfgDesc);
                            break;
                        case 0x22:                                          //����������
                            if(UsbSetupBuf->wIndexL == 0)                   //�ӿ�0����������
                            {
                                pDescr = KeyRepDesc;                        //����׼���ϴ�
                                len = sizeof(KeyRepDesc);								
                            }
							if(UsbSetupBuf->wIndexL == 1)                   //�ӿ�0����������
                            {
                                pDescr = KeyMULRepDesc;                        //����׼���ϴ�
                                len = sizeof(KeyMULRepDesc);
                                Ready = 1;                                  //����и���ӿڣ��ñ�׼λӦ�������һ���ӿ�������ɺ���Ч
                                //IE_UART1 = 1;//���������ж�															
															
                            }
                            else
                            {
                                len = 0xff;                                 //������ֻ��2���ӿڣ���仰����������ִ��
                            }
                            break;
                        default:
                            len = 0xff;                                     //��֧�ֵ�������߳���
                            break;
                        }
                        if ( SetupLen > len )
                        {
                            SetupLen = len;    //�����ܳ���
                        }
                        len = SetupLen >= 8 ? 8 : SetupLen;                  //���δ��䳤��
                        memcpy(Ep0Buffer,pDescr,len);                        //�����ϴ�����
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;                     //�ݴ�USB�豸��ַ
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
                        if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// �˵�
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
                                len = 0xFF;                                            // ��֧�ֵĶ˵�
                                break;
                            }
                        }
                        else
                        {
                            len = 0xFF;                                                // ���Ƕ˵㲻֧��
                        }
                        break;
                    case USB_SET_FEATURE:                                              /* Set Feature */
                        if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x00 )             /* �����豸 */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                            {
                                if( CfgDesc[ 7 ] & 0x20 )
                                {
                                    /* ���û���ʹ�ܱ�־ */
                                }
                                else
                                {
                                    len = 0xFF;                                        /* ����ʧ�� */
                                }
                            }
                            else
                            {
                                len = 0xFF;                                            /* ����ʧ�� */
                            }
                        }
                        else if( ( UsbSetupBuf->bRequestType & 0x1F ) == 0x02 )        /* ���ö˵� */
                        {
                            if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                            {
                                switch( ( ( UINT16 )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* ���ö˵�2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* ���ö˵�2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* ���ö˵�1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF;                               //����ʧ��
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF;                                   //����ʧ��
                            }
                        }
                        else
                        {
                            len = 0xFF;                                      //����ʧ��
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
                        len = 0xff;                                           //����ʧ��
                        break;
                    }
                }
            }
            else
            {
                len = 0xff;                                                   //�����ȴ���
            }
            if(len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
            }
            else if(len <= 8)                                                //�ϴ����ݻ���״̬�׶η���0���Ȱ�
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//Ĭ�����ݰ���DATA1������Ӧ��ACK
            }
            else
            {
                UEP0_T_LEN = 0;  //��Ȼ��δ��״̬�׶Σ�������ǰԤ���ϴ�0�������ݰ��Է�������ǰ����״̬�׶�
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//Ĭ�����ݰ���DATA1,����Ӧ��ACK
            }
            break;
        case UIS_TOKEN_IN | 0:                                               //endpoint0 IN
            switch(SetupReq)
            {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= 8 ? 8 : SetupLen;                          //���δ��䳤��
                memcpy( Ep0Buffer, pDescr, len );                            //�����ϴ�����
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;                                     //ͬ����־λ��ת
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0;                                              //״̬�׶�����жϻ�����ǿ���ϴ�0�������ݰ��������ƴ���
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
            UEP0_T_LEN = 0;  //��Ȼ��δ��״̬�׶Σ�������ǰԤ���ϴ�0�������ݰ��Է�������ǰ����״̬�׶�
            UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_ACK;//Ĭ�����ݰ���DATA0,����Ӧ��ACK
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0;                                                 //д0����ж�
    }
    if(UIF_BUS_RST)                                                       //�豸ģʽUSB���߸�λ�ж�
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;                                                 //���жϱ�־
    }
    if (UIF_SUSPEND)                                                     //USB���߹���/�������
    {
        UIF_SUSPEND = 0;
    }
    else {                                                               //������ж�,�����ܷ��������
        USB_INT_FG = 0xFF;                                               //���жϱ�־
    }
}
/**����HIDֵ�ϴ�����**/
void HIDValueHandle1()
{
    //TR0 = 0; //����ǰ�ض�ʱ���ж�
	FLAG = 0; //���USB�жϴ�����ɱ�־��׼�����Ͱ�����������
	Enp1IntIn(); //USB�豸ģʽ�˵�1���ж��ϴ�
	while(FLAG == 0); //�ȴ�USB�ж����ݴ������
	FLAG = 0; //���USB�жϴ�����ɱ�־��׼�����Ͱ���̧������	
	memset(&HIDKey[0],0,8);	//��HIDkey��0������0��ʾ����̧��
	Enp1IntIn(); //USB�豸ģʽ�˵�1���ж��ϴ�		
	while(FLAG == 0); //�ȴ�USB�ж����ݴ������
	//TR0 = 1; //������򿪶�ʱ���ж�		
}
/**��ý�尴��HIDֵ�ϴ�����**/
void HIDValueHandle2()
{
    //TR0 = 0; //����ǰ�ض�ʱ���ж�
	FLAG = 0; //���USB�жϴ�����ɱ�־��׼�����Ͱ�����������
	Enp2IntIn(); //USB�豸ģʽ�˵�2���ж��ϴ�
	while(FLAG == 0); //�ȴ�USB�ж����ݴ������
	FLAG = 0; //���USB�жϴ�����ɱ�־��׼�����Ͱ���̧������	
	memset(&HIDKeyMUL[0],0,4); //��HIDKeyMUL��0������0��ʾ����̧��
	Enp2IntIn(); //USB�豸ģʽ�˵�2���ж��ϴ�		
	while(FLAG == 0); //�ȴ�USB�ж����ݴ���
	//TR0 = 1; //������򿪶�ʱ���ж�		
}
/**������Ϊ����**/
/*�ҵ�������HIDֵ���ɷ��Ӳ���*/
/*��ͨ����
  ����ctrl + c :
  HIDKey[0] = 0x01;// ctrl
  HIDKey[2] = 0x06;// c
  if(Ready) //ö�ٳɹ�
	{
	    HIDValueHandle1();
	}
*/
/* ������������ */
void KeyAction(unsigned char keyCode)
{   
	if(keyCode == 0x31)//����1
	{
        HIDKeyMUL[0] = 0x10;  //��ý�岥����
        if (Ready) //ö�ٳɹ�
        {
            HIDValueHandle2(); //��ý�尴��HIDֵ�ϴ�
        }
	}
	if(keyCode == 0x32)//����2
	{
		HIDKeyMUL[0] = 0x80;  //��һ��
		if(Ready) //ö�ٳɹ�
        {
            HIDValueHandle2(); //��ý�尴��HIDֵ�ϴ�
        }
	}
	if(keyCode == 0x33)	//����3	
	{
		HIDKeyMUL[1] = 0x01;  //��һ��
		if(Ready) //ö�ٳɹ�
        {
            HIDValueHandle2(); //��ͨ����HIDֵ�ϴ�
        }
	}
	if(keyCode == 0x34)	//����4	
	{
		HIDKeyMUL[0] = 0x40; //����/��ͣ

		if(Ready) //ö�ٳɹ�
        {
            HIDValueHandle2(); //��ý�尴��HIDֵ�ϴ�
        }
	}
	if(keyCode == 0x35)	//����5	
	{
		HIDKeyMUL[0] = 0x02; //����+

		if(Ready) //ö�ٳɹ�
        {
            HIDValueHandle2(); //��ý�尴��HIDֵ�ϴ�
        }
	}
	if(keyCode == 0x36)	//����6	
	{
		HIDKeyMUL[0] = 0x01; //����-

		if(Ready) //ö�ٳɹ�
        {
            HIDValueHandle2(); //��ý�尴��HIDֵ�ϴ�
        }
	}
}
/**��������**/
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
/* EC11���� */
void EC11Drive()
{
    if (EC11_State != EC11_PState)//EC11״̬�İ�����ж�
    {
        if (EC11_PState == 1)
        {
            EC11_State = 1;
            KeyAction(key_code_map[EC11_KeyState]);
        }
        EC11_PState = EC11_State;//�ѵ�ǰEC11״̬��Ϊ��ȥ״̬  Ϊ����һ���ж�
    }
    if (EC11_A == 1) IE_EX |= bIP_GPIO;//��EC11_A�ߵ�ƽʱ��GPIO�ж�
}
/**T0���ú���**/
void ConfigT0(UINT8 ms)
{
    unsigned long tmp = 0;

	tmp = 12000000/12;
	tmp = (tmp * ms)/1000;
	tmp = 65536 - tmp;
	tmp = tmp + 1;
    T0RH = (UINT8)(tmp >> 8);
	T0RL = (UINT8)tmp;

	TMOD = ( TMOD & ~( bT0_GATE | bT0_CT | bT0_M1 ) ) | bT0_M0;//* ģʽ1��16 λ��ʱ/������
	TH0 = T0RH;
	TL0 = T0RL;
	TF0 = 0;
	ET0 = 1;
	TR0 = 1;
}

/***************************************************������******************************************************/
main()
{
    CfgFsys(); //CH552ʱ��ѡ��12M����
    mDelaymS(5); //�޸���Ƶ�ȴ��ڲ������ȶ�,�ؼ�	
	ConfigT0(10); //����10ms T0�ж�
	USBDeviceInit(); //USB�豸ģʽ��ʼ��
   
    UEP1_T_LEN = 0; //Ԥʹ�÷��ͳ���һ��Ҫ���
  	UEP2_T_LEN = 0;	//��ն˵�2���ͳ���
    FLAG = 0;//���USB�жϴ�����ɱ�־
    Ready = 0;//���USBö����ɱ�־
	LED_VALID = 1;//��һ��Ĭ��ֵ

    //GPIO�ж�����   ����EC11_A�½��ػ��ߵ͵�ƽ�����ж�
    IE_EX |= bIP_GPIO;//����GPIO�ж�
    GPIO_IE |= bIE_P3_1_LO;//P3.1ʽ�͵�ƽ��Ч������ģʽ�½�����Ч


    //�õ���IO��ʼ������λ��IO ״̬Ϊ ��©���������������
	key1 = 1;
	key2 = 1;
	key3 = 1;
	key4 = 1;
    EC11_A = 1;
    EC11_B = 1;

    EA = 1; //������Ƭ���ж�
    mDelaymS(10);
	while(1)
	{
	    KeyDrive();	//��������
        EC11Drive();//EC11����
	}
}
/**����ɨ�躯��**/
void KeyScan()
{
	UINT8 i;
	static UINT8 keybuffer[4] = {0xFF,0xFF,0xFF,0xFF};

	keybuffer[0] = (keybuffer[0] <<1) | key1;//����ɨ��
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
			KeyDownTime[i] += 10;  //���µĳ���ʱ���ۼ�
		}
		else if((keybuffer[i] & 0x0F) == 0x0F)
		{
			KeyState[i] = 1;
			if(i>=3)
			    break;
			KeyDownTime[i] = 0;   //���µĳ���ʱ������
		}
	} 

}
/**T0�жϺ���**/
void InterruptTimer0() interrupt INT_NO_TMR0 using 1 
{
	TH0 = T0RH;
	TL0 = T0RL;

	KeyScan(); //����ɨ��
}
/*GPIO�жϺ���**/
void InterruptGPIO() interrupt INT_NO_GPIO
{
    IE_EX &= (~bIP_GPIO);//�ر�GPIO�ж�  ��ֹ���ϴ���
    if (EC11_B == 1) EC11_KeyState = 4;//��ת
    else EC11_KeyState = 5;//��ת
    EC11_State = 0;//��ʾ������ת������
}