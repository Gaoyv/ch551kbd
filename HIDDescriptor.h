#ifndef	__HIDDESCRIPTOR_H__
#define __HIDDESCRIPTOR_H__
/*HID类报表描述符*/
unsigned char code KeyRepDesc[62] =
{
    /******************************************************************
    键盘发送给PC的数据每次8个字节：BYTE0 BYTE1 BYTE2 BYTE3 BYTE4 BYTE5 BYTE6 BYTE7。定义分别是：
    BYTE0 --
           |--bit0:   Left Control
           |--bit1:   Left Shift
           |--bit2:   Left Alt
           |--bit3:   Left GUI
           |--bit4:   Right Control
           |--bit5:   Right Shift
           |--bit6:   Right Alt
           |--bit7:   Right GUI
    BYTE1 -- 暂不清楚，有的地方说是保留位
    BYTE2--BYTE7 -- 这六个为普通按键
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

/*多媒体键盘报表描述符*/
unsigned char code KeyMULRepDesc[105] =
{
	/**********************************************************************************************
	键盘发送给PC的数据每次4个字节：BYTE1 BYTE2 BYTE3 BYTE4
	BYTE0 BYTE1 BYTE2 这3个字节分成24位，每个位代表一个按键，1代表按下，0抬起。
	BYTE0 --
		   |--bit0:  Vol-
		   |--bit1:  Vol+
		   |--bit2:  Mute
		   |--bit3:  Email
		   |--bit4:  Media
		   |--bit5:  WWW Home
		   |--bit6:  Play/Pause
		   |--bit7:  Scan Pre Track
	BYTE1 BYTE2按下面的顺序排下去，BYTE3 bit7：最后一个Usage( NULL )。
	BYTE3 --
		系统功能按键，关机(0x81)，休眠(0x82），唤醒（0x83）
	***********************************************************************************************/
		0x05, 0x0C, //USAGE_PAGE 用途页选择0x0c(用户页)
		0x09, 0x01, //USAGE 接下来的应用集合用于用户控制
		0xA1, 0x01, //COLLECTION 开集合
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
			0x81, 0x02, //INPUT (Data,Var,Abs)输入24bit数据
			0x05, 0x01, //USAGE_PAGE 用途页0x01(普通桌面)
				0x19, 0x00, //USAGE_MINIMUM 用途最小值0x00(未定义)
				0x29, 0x83, //USAGE_MAXIMUM 用途最大值0x83(系统唤醒)
				0x15, 0x00, //LOGICAL_MINIMUM (0)
				0x25, 0x83, //LOGICAL_MAXIMUM (83)
				0x75, 0x08, //REPORT_SIZE (8)
				0x95, 0x01, //REPORT_COUNT (1)
				0x81, 0x00, //INPUT (Data,Ary,Abs)输入1字节数据
		0xC0//END_COLLECTION 闭合集合
};

/*设备描述符*/
unsigned char code DevDesc[18] = {
   0x12,      //bLength字段。设备描述符的长度为18(0x12)字节
   0x01,	  //bDescriptorType字段。设备描述符的编号为0x01
   0x10,0x01, //bcdUSB字段。这里设置版本为USB1.1，即0x0110。
			  //由于是小端结构，所以低字节在先，即0x10，0x01。
   0x00,	  //bDeviceClass字段。我们不在设备描述符中定义设备类，
			  //而在接口描述符中定义设备类，所以该字段的值为0。
   0x00,	  //bDeviceSubClass字段。bDeviceClass字段为0时，该字段也为0。
   0x00,	  //bDeviceProtocol字段。bDeviceClass字段为0时，该字段也为0。
   0x08,	  //bMaxPacketSize0字段。 的端点0大小的8字节。
   0x3d,0x41, //idVender字段,注意小端模式，低字节在先。
   0x3a,0x55, //idProduct字段 产品ID号。注意小端模式，低字节应该在前。
   0x00,0x00, //bcdDevice字段。注意小端模式，低字节应该在前。
   0x00,	  //iManufacturer字段。厂商字符串的索引
   0x00,	  //iProduct字段。产品字符串的索引值,注意字符串索引值不要使用相同的值。
   0x00,	  //iSerialNumber字段。设备的序列号字符串索引值。
   0x01		  //bNumConfigurations字段。该设备所具有的配置数。
};
/*配置描述符*/
unsigned char code CfgDesc[59] =
{
	/*配置描述符*/
	   0x09, //bLength字段。配置描述符的长度为9字节
	   0x02, //bDescriptorType字段。配置描述符编号为0x02
	   0x3b, //wTotalLength字段。配置描述符集合的总长度0x003b，包括配置描述符本身、接口描述符、类描述符、端点描述符等，LSB
	   0x00,
	   0x02, //bNumInterfaces字段。该配置包含的接口数，只有2个接口
	   0x01, //bConfiguration字段。该配置的值为1
	   0x01, //iConfigurationz字段，该配置的字符串索引。
	   0xA0, //bmAttributes字段,bit4-bit7描述设备的特性
	   0x64, //bMaxPower字段，该设备需要的最大电流量。每单位电流为 2 mA    
	/*接口描述符*/
	   //接口1，普通键盘
	   0x09,0x04,0x00,0x00,0x01,0x03,0x01,0x01,0x00, //接口描述符,键盘  HID设备的定义放置在接口描述符中
	   0x09,0x21,0x11,0x01,0x00,0x01,0x22,0x3e,0x00, //HID类描述符
	   0x07,0x05,0x81,0x03,0x08,0x00,0x0a, //端点描述符
	   //接口2，多媒体按键
	   0x09,0x04,0x01,0x00,0x01,0x03,0x00,0x00,0x00, // 接口描述符
	   0x09,0x21,0x00,0x01,0x00,0x01,0x22,0x69,0x00, // HID类描述符
	   0x07,0x05,0x82,0x03,0x04,0x00,0x0a,	// 端点描述符 
};
#endif