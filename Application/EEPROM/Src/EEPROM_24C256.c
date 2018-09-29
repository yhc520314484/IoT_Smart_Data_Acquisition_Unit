#include "EEPROM_24C256.h"
#include "EEPROM_Software_I2C_Definition.h"
#include "gpio.h"

/**
  * @brief AP端基本配置寄存器单独字节地址定义
  */
RegAddBasicSettings register_address_basic_settings = {
	.DataExistFlag    =   AP_REG_BASIC_SETTINGS,
	.SlaveID          =   AP_REG_BASIC_SETTINGS + ((uint16_t)0x0008),
	.MasterID         =   AP_REG_BASIC_SETTINGS + ((uint16_t)0x0018),
	.SensorNum        =   AP_REG_BASIC_SETTINGS + ((uint16_t)0x0028),
	.PowerSupply      =   AP_REG_BASIC_SETTINGS + ((uint16_t)0x0030),
	.SavePower        =   AP_REG_BASIC_SETTINGS + ((uint16_t)0x0038),
	.ComModuleType    =   AP_REG_BASIC_SETTINGS + ((uint16_t)0x0040),
	.SenConAuto       =   AP_REG_BASIC_SETTINGS + ((uint16_t)0x0048)
};

/**
  * @brief AP端时间寄存器单独字节地址定义
  */
RegAddTimeSettings register_address_time_settings = {
	.GapTime        =   AP_REG_TIME_SETTINGS,
	.WindowLength   =   AP_REG_TIME_SETTINGS + ((uint16_t)0x0018),
	.Year           =   AP_REG_TIME_SETTINGS + ((uint16_t)0x0020),
	.Month          =   AP_REG_TIME_SETTINGS + ((uint16_t)0x0028),
	.Day            =   AP_REG_TIME_SETTINGS + ((uint16_t)0x0030),
	.Hour           =   AP_REG_TIME_SETTINGS + ((uint16_t)0x0038),
	.Minute         =   AP_REG_TIME_SETTINGS + ((uint16_t)0x0040),
	.Second         =   AP_REG_TIME_SETTINGS + ((uint16_t)0x0048),
	.Week           =   AP_REG_TIME_SETTINGS + ((uint16_t)0x0050)
};

/**
  * @brief AP端错误告警寄存器单独字节地址定义
  */
RegAddWarningStore register_address_warning_store = {
	.AlarmUpdata              = AP_REG_WARNING_STORE,
	.InitAlarm                = AP_REG_WARNING_STORE + ((uint16_t)0x0008),
	.DataUploadAlarm          = AP_REG_WARNING_STORE + ((uint16_t)0x0010),
	.SyncAlarm                = AP_REG_WARNING_STORE + ((uint16_t)0x0018),
	.LowPowerAlarm            = AP_REG_WARNING_STORE + ((uint16_t)0x0020),
	.AlarmUploadNoACKCount    = AP_REG_WARNING_STORE + ((uint16_t)0x0040),
	.InitResetCount           = AP_REG_WARNING_STORE + ((uint16_t)0x0048),
	.DataUploadNoACKCount     = AP_REG_WARNING_STORE + ((uint16_t)0x0050),
};

/**
  * @brief 通信密钥存储寄存器单独字节地址定义
  */
RegAddCommunicationKey register_address_communication_key = {
	.CommunicationKey              = AP_REG_COMMUNICATION_KEY
};

/**
  * @brief 恢复出厂设置密钥存储寄存器单独字节地址定义
  */
RegAddWipeKey register_address_wipe_key = {
	.WipeKey              = AP_REG_WIPE_KEY
};

/**
  * @brief 传感器参数存储寄存器单独字节地址定义
  */

RegAddsensorParm register_address_sensor_parm = {
	.SensorType        =   AP_REG_SENSOR_PARM,
	.SensorParaNum     =   AP_REG_SENSOR_PARM + ((uint16_t)0x0008),
	.SensorIDLen       =   AP_REG_SENSOR_PARM + ((uint16_t)0x0010),
	.SensorComSpeed    =   AP_REG_SENSOR_PARM + ((uint16_t)0x0018),
	.sensorParameter   =   AP_REG_SENSOR_PARM + ((uint16_t)0x0040),
};

reg_data_basic_settings        register_data_basic_settings;
reg_data_time_settings         register_data_time_settings;
reg_data_warning_store         register_data_warning_store;
reg_data_communication_key     register_data_communication_key;
reg_data_wipe_key              register_data_wipe_key;
reg_data_sensor_parm           register_data_sensor_parm;
reg_data_sensor_ID             register_data_sensor_ID;
reg_data_sensor_parm_and_ID    register_data_sensor_parm_and_ID = {
	.sensor_parm = NULL,
	.sensor_ID = NULL
};

/**
  * @brief 传感器1 ID存储寄存器单独字节地址定义
  */
RegAddsensorID register_address_sensor_ID = {
	.sensorID              = AP_REG_SENSOR_ID
};




/**
  * @name         EEPROM_24C256_Init
  * @brief        用于24C256 EEPROM 初始化
  * @param        void
  * @retval       24C256 EEPROM是否初始化成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/25  11:18
	* @author       JackWilliam
  */
uint8_t EEPROM_24C256_Init(void){
	if(GPIO_EEPROM_24C256_Init()) return 1;
	if(ee_CheckOk()) return 1;
	
	return 0;
}


/**
  * @name         ee_CheckOk
  * @brief        判断24C256 EERPOM是否正常
  * @param        void
  * @retval       24C256 EEPROM是否初始化成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/25  11:18
	* @author       JackWilliam
  */
uint8_t ee_CheckOk(void)
{
	if (i2c_CheckDevice(EEPROM_DEV_ADDR) == 0)
	{
		return 0;
	}
	else
	{
		/* 失败后，切记发送I2C总线停止信号 */
		i2c_Stop();		
		return 1;
	}
}

/**
  * @name         ee_ReadBytes
  * @brief        从24C256 EERPOM指定地址处开始读取若干数据
  * @param        _usAddress : 起始地址
  * @param        _usSize : 数据长度，单位为字节
  * @param        _pReadBuf : 存放读到的数据的缓冲区指针
  * @retval       24C256 EEPROM数据读取是否成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/25  11:18
	* @author       JackWilliam
  */
uint8_t ee_ReadBytes(uint8_t *_pReadBuf, uint16_t _usAddress, uint16_t _usSize)
{
	uint16_t i;
	
	/* 采用串行EEPROM随即读取指令序列，连续读取若干字节 */
	
	/* 第1步：发起I2C总线启动信号 */
	i2c_Start();
	
	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	i2c_SendByte(EEPROM_DEV_ADDR | EEPROM_I2C_WR);	/* 此处是写指令 */
	 
	/* 第3步：等待ACK */
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址，24C02只有256字节，因此1个字节就够了，如果是24C04以上，那么此处需要连发多个地址 */
	i2c_SendByte(_usAddress << 8);
	
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	
	i2c_SendByte((uint8_t)_usAddress);
	/* 第5步：等待ACK */
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	
	/* 第6步：重新启动I2C总线。前面的代码的目的向EEPROM传送地址，下面开始读取数据 */
	i2c_Start();
	
	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	i2c_SendByte(EEPROM_DEV_ADDR | EEPROM_I2C_RD);	/* 此处是读指令 */
	
	/* 第8步：发送ACK */
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}	
	
	/* 第9步：循环读取数据 */
	for (i = 0; i < _usSize; i++)
	{
		_pReadBuf[i] = i2c_ReadByte();	/* 读1个字节 */
		
		/* 每读完1个字节后，需要发送Ack， 最后一个字节不需要Ack，发Nack */
		if (i != _usSize - 1)
		{
			i2c_Ack();	/* 中间字节读完后，CPU产生ACK信号(驱动SDA = 0) */
		}
		else
		{
			i2c_NAck();	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
		}
	}
	/* 发送I2C总线停止信号 */
	i2c_Stop();
	return 0;	/* 执行成功 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	i2c_Stop();
	return 1;
}


/**
  * @name         ee_WriteBytes
  * @brief        从24C256 EERPOM指定地址写入若干数据，采用页写操作提高写入效率
  * @param        _usAddress : 起始地址
  * @param        _usSize : 数据长度，单位为字节
  * @param        _pWriteBuf : 存放读到的数据的缓冲区指针
  * @retval       24C256 EEPROM数据写入是否成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/25  11:18
	* @author       JackWilliam
  */
uint8_t ee_WriteBytes(uint8_t *_pWriteBuf, uint16_t _usAddress, uint16_t _usSize)
{
	uint16_t i,m;
	uint16_t usAddr;
	
	/* 
		写串行EEPROM不像读操作可以连续读取很多字节，每次写操作只能在同一个page。
		对于24xx02，page size = 8
		简单的处理方法为：按字节写操作模式，没写1个字节，都发送地址
		为了提高连续写的效率: 本函数采用page wirte操作。
	*/

	usAddr = _usAddress;	
	for (i = 0; i < _usSize; i++)
	{
		/* 当发送第1个字节或是页面首地址时，需要重新发起启动信号和地址 */
		if ((i == 0) || (usAddr & (EEPROM_PAGE_SIZE - 1)) == 0)
		{
			/*　第０步：发停止信号，启动内部写操作　*/
			i2c_Stop();
			
			/* 通过检查器件应答的方式，判断内部写操作是否完成, 一般小于 10ms 			
				CLK频率为200KHz时，查询次数为30次左右
			*/
			for (m = 0; m < 1000; m++)
			{				
				/* 第1步：发起I2C总线启动信号 */
				i2c_Start();
				
				/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
				i2c_SendByte(EEPROM_DEV_ADDR | EEPROM_I2C_WR);	/* 此处是写指令 */
				
				/* 第3步：发送一个时钟，判断器件是否正确应答 */
				if (i2c_WaitAck() == 0)
				{
					break;
				}
			}
			if (m  == 1000)
			{
				goto cmd_fail;	/* EEPROM器件写超时 */
			}
		
			/* 第4步：发送字节地址，24C02只有256字节，因此1个字节就够了，如果是24C04以上，那么此处需要连发多个地址 */
			i2c_SendByte(usAddr << 8);
			
			if (i2c_WaitAck() != 0)
			{
				goto cmd_fail;	/* EEPROM器件无应答 */
			}
			
			i2c_SendByte((uint8_t)usAddr);
			
			/* 第5步：等待ACK */
			if (i2c_WaitAck() != 0)
			{
				goto cmd_fail;	/* EEPROM器件无应答 */
			}
		}
	
		/* 第6步：开始写入数据 */
		i2c_SendByte(_pWriteBuf[i]);
	
		/* 第7步：发送ACK */
		if (i2c_WaitAck() != 0)
		{
			goto cmd_fail;	/* EEPROM器件无应答 */
		}

		usAddr++;	/* 地址增1 */		
	}
	
	/* 命令执行成功，发送I2C总线停止信号 */
	i2c_Stop();
	return 0;

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	i2c_Stop();
	return 1;
}



/**
  * @name         ee_Erase
  * @brief        从24C256 EERPOM指定地址擦除数据
	* @param        _usAddress : 起始地址
  * @param        _usSize : 数据长度，单位为字节
  * @retval       24C256 EEPROM数据擦除是否成功  成功则返回0  失败则返回1
	* @lastModify   2018/9/25  11:18
	* @author       JackWilliam
  */
uint8_t ee_Erase(uint16_t _usAddress, uint16_t _usSize)
{
	uint16_t i;
	uint8_t buf[_usSize];
	
	/* 填充缓冲区 */
	for (i = 0; i < _usSize; i++)
	{
		buf[i] = 0xFF;
	}
	
	/* 写EEPROM, 起始地址 = 0，数据长度为 256 */
	if (ee_WriteBytes(buf, _usAddress, _usSize) == 0)
	{
		printf("擦除eeprom出错！\r\n");
		return 1;
	}
	else
	{
		printf("擦除eeprom成功！\r\n");
		return 0;
	}
}


/*--------------------------------------------------------------------------------------------------*/
static void ee_Delay(__IO uint32_t nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
}


/*
 * eeprom AT24C02 读写测试
 * 正常返回0，异常返回1
 */
uint8_t ee_Test(void) 
{
  uint16_t i;
	uint8_t write_buf[256];
  uint8_t read_buf[256];
  
/*-----------------------------------------------------------------------------------*/  
  if (ee_CheckOk() == 1)
	{
		/* 没有检测到EEPROM */
		printf("没有检测到串行EEPROM!\r\n");
				
		return 1;
	}
/*------------------------------------------------------------------------------------*/  
  /* 填充测试缓冲区 */
//	for (i = 0; i < EEPROM_SIZE; i++)
//	{		
//		write_buf[i] = i;
//	}
	for (i = 0; i < 256; i++)
	{		
		write_buf[i] = i;
	}
/*------------------------------------------------------------------------------------*/  
//  if (ee_WriteBytes(write_buf, 0, EEPROM_SIZE) == 0)
//	{
//		printf("写eeprom出错！\r\n");
//		return 0;
//	}
//	else
//	{		
//		printf("写eeprom成功！\r\n");
//	}
	if (ee_WriteBytes(write_buf, 0, 256) == 0)
	{
		printf("写eeprom出错！\r\n");
		return 1;
	}
	else
	{		
		printf("写eeprom成功！\r\n");
	}
  
  /*写完之后需要适当的延时再去读，不然会出错*/
  ee_Delay(0x0FFFFF);
/*-----------------------------------------------------------------------------------*/
//  if (ee_ReadBytes(read_buf, 0, EEPROM_SIZE) == 0)
//	{
//		printf("读eeprom出错！\r\n");
//		return 0;
//	}
//	else
//	{		
//		printf("读eeprom成功，数据如下：\r\n");
//	}
	  if (ee_ReadBytes(read_buf, 0, 256) == 0)
	{
		printf("读eeprom出错！\r\n");
		return 1;
	}
	else
	{		
		printf("读eeprom成功，数据如下：\r\n");
	}
/*-----------------------------------------------------------------------------------*/  
//  for (i = 0; i < EEPROM_SIZE; i++)
//	{
//		if(read_buf[i] != write_buf[i])
//		{
//			printf("0x%02X ", read_buf[i]);
//			printf("错误:EEPROM读出与写入的数据不一致");
//			return 0;
//		}
//    printf(" %02X", read_buf[i]);
//		
//		if ((i & 15) == 15)
//		{
//			printf("\r\n");	
//		}		
//	}
	
	for (i = 0; i < 256; i++)
	{
		if(read_buf[i] != write_buf[i])
		{
			printf("0x%02X ", read_buf[i]);
			printf("错误:EEPROM读出与写入的数据不一致");
			return 1;
		}
    printf(" %02X", read_buf[i]);
		
		if ((i & 15) == 15)
		{
			printf("\r\n");	
		}		
	}
  printf("eeprom读写测试成功\r\n");
  return 0;
}
/*********************************************END OF FILE**********************/
