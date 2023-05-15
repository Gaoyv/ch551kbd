#ifndef	__HIDDESCRIPTOR_H__
#define __HIDDESCRIPTOR_H__
/*HID�౨��������*/
unsigned char code KeyRepDesc[62] =
{
    /******************************************************************
    ���̷��͸�PC������ÿ��8���ֽڣ�BYTE0 BYTE1 BYTE2 BYTE3 BYTE4 BYTE5 BYTE6 BYTE7������ֱ��ǣ�
    BYTE0 --
           |--bit0:   Left Control
           |--bit1:   Left Shift
           |--bit2:   Left Alt
           |--bit3:   Left GUI
           |--bit4:   Right Control
           |--bit5:   Right Shift
           |--bit6:   Right Alt
           |--bit7:   Right GUI
    BYTE1 -- �ݲ�������еĵط�˵�Ǳ���λ
    BYTE2--BYTE7 -- ������Ϊ��ͨ����
    *******************************************************************/
        0x05,0x01,0x09,0x06,0xA1,0x01,0x05,0x07,
        0x19,0xe0,0x29,0xe7,0x15,0x00,0x25,0x01,
        0x75,0x01,0x95,0x08,0x81,0x02,0x95,0x01,
        0x75,0x08,0x81,0x01,0x95,0x03,0x75,0x01,
        0x05,0x08,0x19,0x01,0x29,0x03,0x91,0x02,
        0x95,0x05,0x75,0x01,0x91,0x01,0x95,0x06,
        0x75,0x08,0x26,0xff,0x00,0x05,0x07,0x19,
        0x00,0x29,0x91,0x81,0x00,0xC0
};

/*��ý����̱���������*/
unsigned char code KeyMULRepDesc[105] =
{
	/**********************************************************************************************
	���̷��͸�PC������ÿ��4���ֽڣ�BYTE1 BYTE2 BYTE3 BYTE4
	BYTE0 BYTE1 BYTE2 ��3���ֽڷֳ�24λ��ÿ��λ����һ��������1�����£�0̧��
	BYTE0 --
		   |--bit0:  Vol-
		   |--bit1:  Vol+
		   |--bit2:  Mute
		   |--bit3:  Email
		   |--bit4:  Media
		   |--bit5:  WWW Home
		   |--bit6:  Play/Pause
		   |--bit7:  Scan Pre Track
	BYTE1 BYTE2�������˳������ȥ��BYTE3 bit7�����һ��Usage( NULL )��
	BYTE3 --
		ϵͳ���ܰ������ػ�(0x81)������(0x82�������ѣ�0x83��
	***********************************************************************************************/
		0x05, 0x0C, //USAGE_PAGE ��;ҳѡ��0x0c(�û�ҳ)
		0x09, 0x01, //USAGE ��������Ӧ�ü��������û�����
		0xA1, 0x01, //COLLECTION ������
			0x15, 0x00, //LOGICAL_MINIMUM (0)
			0x25, 0x01, //LOGICAL_MAXIMUM (1)
			0x0A, 0xEA, 0x00,		/* Usage( Vol- ) */
			0x0A, 0xE9, 0x00,		/* Usage( Vol+ ) */
			0x0A, 0xE2, 0x00,		/* Usage( Mute ) */
			0x0A, 0x8A, 0x01,		/* Usage( Email ) */
			0x0A, 0x83, 0x01,		/* Usage( Media ) */
			0x0A, 0x23, 0x02,		/* Usage( WWW Home ) */
			0x0A, 0xCD, 0x00,		/* Usage( Play/Pause ) */
			0x0A, 0xB6, 0x00,		/* Usage( Scan Pre Track ) */
			0x0A, 0xB5, 0x00,		/* Usage( Scan Next Track ) */
			0x0A, 0xB7, 0x00,		/* Usage( Stop ) */
			0x0A, 0x70, 0x00,		/* Usage( NULL ) */
			0x0A, 0x6f, 0x00,		/* Usage( NULL ) */
			0x0A, 0x00, 0x00,		/* Usage( NULL ) */
			0x0A, 0x00, 0x00,		/* Usage( NULL ) */
			0x0A, 0x00, 0x00,		/* Usage( NULL ) */
			0x0A, 0x00, 0x00,		/* Usage( NULL ) */
			0x0A, 0x00, 0x00,		/* Usage( NULL ) */
			0x0A, 0x00, 0x00,		/* Usage( NULL ) */
			0x0A, 0x00, 0x00,		/* Usage( NULL ) */
			0x0A, 0x00, 0x00,		/* Usage( NULL ) */
			0x0A, 0x00, 0x00,		/* Usage( NULL ) */
			0x0A, 0x00, 0x00,		/* Usage( NULL ) */
			0x0A, 0x00, 0x00,		/* Usage( NULL ) */
			0x0A, 0x11, 0x22,		/* Usage( NULL ) */
			0x75, 0x01, //REPORT_SIZE (1)
			0x95, 0x18, //REPORT_COUNT (24)
			0x81, 0x02, //INPUT (Data,Var,Abs)����24bit����
			0x05, 0x01, //USAGE_PAGE ��;ҳ0x01(��ͨ����)
				0x19, 0x00, //USAGE_MINIMUM ��;��Сֵ0x00(δ����)
				0x29, 0x83, //USAGE_MAXIMUM ��;���ֵ0x83(ϵͳ����)
				0x15, 0x00, //LOGICAL_MINIMUM (0)
				0x25, 0x83, //LOGICAL_MAXIMUM (83)
				0x75, 0x08, //REPORT_SIZE (8)
				0x95, 0x01, //REPORT_COUNT (1)
				0x81, 0x00, //INPUT (Data,Ary,Abs)����1�ֽ�����
		0xC0//END_COLLECTION �պϼ���
};

/*�豸������*/
unsigned char code DevDesc[18] = {
   0x12,      //bLength�ֶΡ��豸�������ĳ���Ϊ18(0x12)�ֽ�
   0x01,	  //bDescriptorType�ֶΡ��豸�������ı��Ϊ0x01
   0x10,0x01, //bcdUSB�ֶΡ��������ð汾ΪUSB1.1����0x0110��
			  //������С�˽ṹ�����Ե��ֽ����ȣ���0x10��0x01��
   0x00,	  //bDeviceClass�ֶΡ����ǲ����豸�������ж����豸�࣬
			  //���ڽӿ��������ж����豸�࣬���Ը��ֶε�ֵΪ0��
   0x00,	  //bDeviceSubClass�ֶΡ�bDeviceClass�ֶ�Ϊ0ʱ�����ֶ�ҲΪ0��
   0x00,	  //bDeviceProtocol�ֶΡ�bDeviceClass�ֶ�Ϊ0ʱ�����ֶ�ҲΪ0��
   0x08,	  //bMaxPacketSize0�ֶΡ� �Ķ˵�0��С��8�ֽڡ�
   0x3d,0x41, //idVender�ֶ�,ע��С��ģʽ�����ֽ����ȡ�
   0x3a,0x55, //idProduct�ֶ� ��ƷID�š�ע��С��ģʽ�����ֽ�Ӧ����ǰ��
   0x00,0x00, //bcdDevice�ֶΡ�ע��С��ģʽ�����ֽ�Ӧ����ǰ��
   0x00,	  //iManufacturer�ֶΡ������ַ���������
   0x00,	  //iProduct�ֶΡ���Ʒ�ַ���������ֵ,ע���ַ�������ֵ��Ҫʹ����ͬ��ֵ��
   0x00,	  //iSerialNumber�ֶΡ��豸�����к��ַ�������ֵ��
   0x01		  //bNumConfigurations�ֶΡ����豸�����е���������
};
/*����������*/
unsigned char code CfgDesc[59] =
{
	/*����������*/
	   0x09, //bLength�ֶΡ������������ĳ���Ϊ9�ֽ�
	   0x02, //bDescriptorType�ֶΡ��������������Ϊ0x02
	   0x3b, //wTotalLength�ֶΡ��������������ϵ��ܳ���0x003b���������������������ӿ��������������������˵��������ȣ�LSB
	   0x00,
	   0x02, //bNumInterfaces�ֶΡ������ð����Ľӿ�����ֻ��2���ӿ�
	   0x01, //bConfiguration�ֶΡ������õ�ֵΪ1
	   0x01, //iConfigurationz�ֶΣ������õ��ַ���������
	   0xA0, //bmAttributes�ֶ�,bit4-bit7�����豸������
	   0x64, //bMaxPower�ֶΣ����豸��Ҫ������������ÿ��λ����Ϊ 2 mA    
	/*�ӿ�������*/
	   //�ӿ�1����ͨ����
	   0x09,0x04,0x00,0x00,0x01,0x03,0x01,0x01,0x00, //�ӿ�������,����  HID�豸�Ķ�������ڽӿ���������
	   0x09,0x21,0x11,0x01,0x00,0x01,0x22,0x3e,0x00, //HID��������
	   0x07,0x05,0x81,0x03,0x08,0x00,0x0a, //�˵�������
	   //�ӿ�2����ý�尴��
	   0x09,0x04,0x01,0x00,0x01,0x03,0x00,0x00,0x00, // �ӿ�������
	   0x09,0x21,0x00,0x01,0x00,0x01,0x22,0x69,0x00, // HID��������
	   0x07,0x05,0x82,0x03,0x04,0x00,0x0a,	// �˵������� 
};
#endif