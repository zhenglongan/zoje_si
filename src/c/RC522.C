#include "..\..\include\common.h"         // Common constants definition

#if ENABLE_RFID_FUNCTION

#include "..\..\include\sfr62p.h"         // M16C/62P special function register definitions
#include "..\..\include\variables.h" 
#include "..\..\include\delay.h" 
#include "..\..\include\action.h" 
#include "..\..\include\MFRC522.h" 


#define MAXRLEN 18

//CRC16计算方式选择,多项式：0x1021(x16+x12+x5+1),起始值：0x6363
#define CRC16_CALCULATE_METHOD_MFRC522					1//使用RC522计算CRC16，最慢
#define CRC16_CALCULATE_METHOD_DIRECTLY_CALCULATING		2//主控直接计算CRC16，快
#define CRC16_CALCULATE_METHOD_LOOKUP_TABLE				3//主控查表法计算CRC16，最快，但是增加ROM开销
#define CRC16_CALCULATE_METHOD_DIRECTLY_CALCULATING1	4//主控直接计算CRC16，快，官方提供的计算方式

#define CRC16_CALCULATE_METHOD CRC16_CALCULATE_METHOD_DIRECTLY_CALCULATING1



/////////////////////////////////////////////////////////////////////
//功  能：寻卡
//参数说明: req_code[IN]:寻卡方式
//        0x52 = 寻感应区内所有符合14443A标准的卡
//        0x26 = 寻未进入休眠状态的卡
//        pTagType[OUT]：卡片类型代码
//        0x4400 = Mifare_UltraLight
//        0x0400 = Mifare_One(S50)
//        0x0200 = Mifare_One(S70)
//        0x0800 = Mifare_Pro(X)
//        0x4403 = Mifare_DESFire
//返  回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
INT8 PcdRequest(UINT8 req_code,UINT8 *pTagType)
{
   INT8   status;  
   UINT16 unLen;
   UINT8 ucComMF522Buf[MAXRLEN]; 

   ClearBitMask(Status2Reg,0x08);//验证密码成功后才能置位此位bit3
   WriteRawRC(BitFramingReg,0x07);//bit2-0定义发送的最后一个字节的位数
   SetBitMask(TxControlReg,0x03);//使能天线发送
 
   ucComMF522Buf[0] = req_code;

   status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);
   
   if ((status == MI_OK) && (unLen == 0x10))//接收到16bit
   {  
       *pTagType   = ucComMF522Buf[0];
       *(pTagType+1) = ucComMF522Buf[1];
   }
   else
   {   status = MI_ERR;   }
   
   return status;
}

/////////////////////////////////////////////////////////////////////
//功  能：防冲撞
//参数说明: pSnr[OUT]:卡片序列号，4字节
//返  回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////  
INT8 PcdAnticoll(UINT8 *pSnr)
{
    INT8 status;
    UINT8 i,snr_check=0;
    UINT16  unLen;
    UINT8 ucComMF522Buf[MAXRLEN]; 
  
	//寻卡时已经清理，此处应该可以屏蔽
  ClearBitMask(Status2Reg,0x08);//验证密码成功后才能置位此位bit3
  WriteRawRC(BitFramingReg,0x00);//bit2-0定义发送的最后一个字节的位数
  ClearBitMask(CollReg,0x80);//所有接收的位在冲突后将被清除
 
  ucComMF522Buf[0] = PICC_ANTICOLL1;//4字节的卡号，只能使用防碰撞等级1，即前4字节参与防冲突选择
  ucComMF522Buf[1] = 0x20;//表示此命令长度为2Byte加0bit

  status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

  if (status == MI_OK)
  {
  	 for (i=0; i<4; i++)//获取4byte卡号,第5字节为BCC，这里没有进行校验
     {   
       *(pSnr+i)  = ucComMF522Buf[i];
       snr_check ^= ucComMF522Buf[i];
     }
     if (snr_check != ucComMF522Buf[i])
     {   status = MI_ERR;  }
  }
  
  SetBitMask(CollReg,0x80);
  return status;
}

/////////////////////////////////////////////////////////////////////
//功  能：选定卡片
//参数说明: pSnr[IN]:卡片序列号，4字节
//返  回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
INT8 PcdSelect(UINT8 *pSnr)
{
    INT8 status;
    UINT8 i;
    UINT16  unLen;
    UINT8 ucComMF522Buf[MAXRLEN]; 
  
  ucComMF522Buf[0] = PICC_ANTICOLL1;
  ucComMF522Buf[1] = 0x70;
  ucComMF522Buf[6] = 0;
  for (i=0; i<4; i++)
  {
  	ucComMF522Buf[i+2] = *(pSnr+i);
  	ucComMF522Buf[6]  ^= *(pSnr+i);
  }
  CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);//计算CRC16
  
  ClearBitMask(Status2Reg,0x08);//验证密码成功后才能置位此位bit3

  status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
  
  if ((status == MI_OK) && (unLen == 0x18))
  {   status = MI_OK;  }
  else
  {   status = MI_ERR;  }

  return status;
}

/////////////////////////////////////////////////////////////////////
//功  能：验证卡片密码
//参数说明: auth_mode[IN]: 密码验证模式
//         0x60 = 验证A密钥
//         0x61 = 验证B密钥 
//      addr[IN]：块地址
//      pKey[IN]：密码
//      pSnr[IN]：卡片序列号，4字节
//返  回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////         
INT8 PcdAuthState(UINT8 auth_mode,UINT8 addr,UINT8 *pKey,UINT8 *pSnr)
{
    INT8 status;
    UINT16  unLen;
    UINT8 i,ucComMF522Buf[MAXRLEN]; 

  ucComMF522Buf[0] = auth_mode;
  ucComMF522Buf[1] = addr;
  for (i=0; i<6; i++)
  {  
	  ucComMF522Buf[i+2] = *(pKey+i);  
  }
  for (i=0; i<6; i++)
  {  
	  ucComMF522Buf[i+8] = *(pSnr+i); //卡号加CRC16 
  }
  
  status = PcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
  if ((status != MI_OK) || (!(ReadRawRC(Status2Reg) & 0x08)))
  {   status = MI_ERR;   }
  
  return status;
}

/////////////////////////////////////////////////////////////////////
//功  能：读取M1卡一块数据
//参数说明: addr[IN]：块地址
//      pData[OUT]：读出的数据，16字节
//返  回: 成功返回MI_OK
///////////////////////////////////////////////////////////////////// 
INT8 PcdRead(UINT8 addr,UINT8 *pData)
{
    INT8 status;
    UINT16  unLen;
    UINT8 i,ucComMF522Buf[MAXRLEN]; 

  ucComMF522Buf[0] = PICC_READ;
  ucComMF522Buf[1] = addr;
  CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
   
  status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
  if ((status == MI_OK) && (unLen == 0x90))
  {
    for (i=0; i<16; i++)
    {  *(pData+i) = ucComMF522Buf[i];   }
  }
  else
  {   status = MI_ERR;   }
  
  return status;
}

/////////////////////////////////////////////////////////////////////
//功  能：写数据到M1卡一块
//参数说明: addr[IN]：块地址
//      pData[IN]：写入的数据，16字节
//返  回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////          
INT8 PcdWrite(UINT8 addr,UINT8 *pData)
{
    INT8 status;
    UINT16  unLen;
    UINT8 i,ucComMF522Buf[MAXRLEN]; 
  
  ucComMF522Buf[0] = PICC_WRITE;
  ucComMF522Buf[1] = addr;
  CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
  status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
	//返回4bit数据，且为0xA表示成功，可以发送要写入的块数据了
  if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
  {   status = MI_ERR;   }
    
  if (status == MI_OK)
  {
    for (i=0; i<16; i++)
    {  
		ucComMF522Buf[i] = *(pData+i);  //块数据
	}
    CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);//后面再加上CRC16

    status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {   status = MI_ERR;   }
  }
  
  return status;
}

/////////////////////////////////////////////////////////////////////
//功  能：命令卡片进入休眠状态
//返  回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
INT8 PcdHalt(void)
{
    INT8 status;
    UINT16  unLen;
    UINT8 ucComMF522Buf[MAXRLEN]; 

  	ucComMF522Buf[0] = PICC_HALT;
  	ucComMF522Buf[1] = 0;
  	CalulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
 
  	status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
  	return MI_OK;
}

#if CRC16_CALCULATE_METHOD==CRC16_CALCULATE_METHOD_MFRC522
/////////////////////////////////////////////////////////////////////
//用MFRC522计算CRC16函数
/////////////////////////////////////////////////////////////////////
void CalulateCRC(UINT8 *pIndata,UINT8 len,UINT8 *pOutData)
{
    UINT8 i,n;
  ClearBitMask(DivIrqReg,0x04);
  WriteRawRC(CommandReg,PCD_IDLE);
  SetBitMask(FIFOLevelReg,0x80);
  for (i=0; i<len; i++)
  {   
	  WriteRawRC(FIFODataReg, *(pIndata+i));   
  }
  WriteRawRC(CommandReg, PCD_CALCCRC);
  i = 0xFF;
  do 
  {
    	n = ReadRawRC(DivIrqReg);
    	i--;
  }
  while ((i!=0) && !(n&0x04));
  pOutData[0] = ReadRawRC(CRCResultRegL);//低位
  pOutData[1] = ReadRawRC(CRCResultRegM);//高位
}

#elif CRC16_CALCULATE_METHOD==CRC16_CALCULATE_METHOD_DIRECTLY_CALCULATING
//直接主控计算，速度比从MFRC522中获取快，但是比查表法慢
void CalulateCRC(UINT8 *pIndata,UINT8 len,UINT8 *pOutData)
{
	UINT16 CRCin = 0x6363;//初值
	UINT16 CPoly = 0x1021;//多项式0x1021:x16+x12+x5+1
	UINT8 data = 0;
	UINT8 i;
	
	while (len--)	
	{
		data = *(pIndata++);
		CRCin ^= (data << 8);
		for(i = 0;i < 8;i++)
		{
			if(CRCin & 0x8000)
			{
				CRCin = (CRCin << 1) ^ CPoly;
			}
			else
			{
				CRCin = CRCin << 1;
			}
		}
	}
	pOutData[0]=(UINT8)CRCin;//低8位
	pOutData[1]=(UINT8)(CRCin>>8);//高8位
	
}
#elif CRC16_CALCULATE_METHOD==CRC16_CALCULATE_METHOD_LOOKUP_TABLE
//参考：https://blog.csdn.net/zhangxuechao_/article/details/80366843
//查表法，比主控直接计算快,多项式为0x1021:x16+x12+x5+1
//但是查表法不能随意更改多项式，多项式一改，表格也需要更改
const UINT16 CRC16_table[]={
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 
	0x8108, 0x9129, 0xa14a,	0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6,	0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 
	0x3653, 0x2672,	0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,	0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af,	0x8948, 0x9969, 0xa90a, 0xb92b, 
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd,	0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03,	0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5,	0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004,	0x4025, 0x7046, 0x6067, 
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 
	0x02b1, 0x1290,	0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,	0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8,	0xe75f, 0xf77e, 0xc71d, 0xd73c, 
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c,	0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1,	0x3882, 0x28a3, 
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 
	0x4a75, 0x5a54, 0x6a37,	0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b,	0xbfba, 0x8fd9, 0x9ff8, 
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 
};
 void CalulateCRC(UINT8 *pIndata,UINT8 len,UINT8 *pOutData)
{
	UINT16 CRCin = 0x6363; //CRC16初始值，根据CRC类型设定
	
	if(len<1)
		return;
	
	while(len--)
	{
		CRCin = ((CRCin << 8) ^ CRC16_table[(UINT16)((CRCin >> 8) ^ *pIndata) & 0xFF]) & 0xFFFF;
		pIndata++;
	}
	pOutData[0]=(UINT8)CRCin;//低8位
	pOutData[1]=(UINT8)(CRCin>>8);//高8位

}

#elif CRC16_CALCULATE_METHOD==CRC16_CALCULATE_METHOD_DIRECTLY_CALCULATING1
//主控直接计算CRC16，快，官方提供的计算方式
UINT16 UpdateCrc(UINT8 ch, UINT16 *lpwCrc)
{
	ch = (ch^(UINT8)((*lpwCrc) & 0x00FF));
	ch = (ch^(ch<<4));

	*lpwCrc = (*lpwCrc >> 8)^((UINT16)ch << 8)^((UINT16)ch<<3)^((UINT16)ch>>4);
	return(*lpwCrc);
}
//CRC_A官方校验计算程序，自己写的程序无法模拟出实际情况，两个字节的时候，自己的程序可以正常
//计算，但是超过3个以后，计算结果就会和官方结果不一样，原因待深入研究
void CalulateCRC(UINT8 *pIndata,UINT8 len,UINT8 *pOutData)
{
	UINT8 chBlock;
	UINT16 wCrc;

	wCrc = 0x6363; // ITU-V.41，CRC16初值
	do{
		chBlock = *pIndata++;
		UpdateCrc(chBlock, &wCrc);
	} while (--len);

	pOutData[0] = (UINT8) (wCrc & 0xFF);//低8位
	pOutData[1] = (UINT8) ((wCrc >> 8) & 0xFF);//高8位
	
	return;
}

 
#endif



/////////////////////////////////////////////////////////////////////
//功  能：复位RC522
//返  回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
INT8 PcdReset(void)
{	
  	WriteRawRC(CommandReg,PCD_RESETPHASE);//参考手册5.2.1.2、15.3、15.4.8节，b1111表示复位MFRC522
	delay_ms(30); 
/*
ModeReg:0x11  default value:0X3F
bit7~bit6:RFU 00
bit5:txwaitRF 1 发送器等RF启动后有效
bit4: 1
bit3;PolSigin  SIGIN管脚极性 1＝高电平有效
bit2: 1
bit1~bit0:CRCPreset  CRC预设值 10--A671

*/	
	//参考手册5.2.2.2
  	WriteRawRC(ModeReg,0x3D);            //和Mifare卡通讯，CRC初始值0x6363
	delay_ms(30); 
/*
定时器重装值 1e00 应该是001e？？？
*/	
  	WriteRawRC(TReloadRegL,30); //参考手册5.2.3.11
	delay_ms(30);          
  	WriteRawRC(TReloadRegH,0);
	delay_ms(30); 
/*
b7: 1 定时器在所有速率发送结束时自动启动，接收第一个数据位定时器立即停止
b6~b5: 00 非门控模式
b4: 0 定时器递减到0,timerIRq置1
b3~b0: 分频的高4位  ftimer= 6.78MHz/分频值
*/	
  	WriteRawRC(TModeReg,0x8D);//参考手册5.2.3.10
	delay_ms(30); 
  	WriteRawRC(TPrescalerReg,0x3E);//分频系数3390，定时器频率2KHz
	delay_ms(30); 
/*

*/	
  	WriteRawRC(TxAskReg,0x40); //保留寄存器，需要写入？？
	delay_ms(30); 
  	ClearBitMask(TestPinEnReg, 0x80);//off MX and DTRQ out
	
 
  	return MI_OK;
}
//////////////////////////////////////////////////////////////////////
//设置RC522的工作方式 
//////////////////////////////////////////////////////////////////////
INT8 M500PcdConfigISOType(UINT8 type)
{
   if (type == 'A')                     //ISO14443_A
   { 
       ClearBitMask(Status2Reg,0x08);

       WriteRawRC(ModeReg,0x3D);//3F

       WriteRawRC(RxSelReg,0x86);//84


       WriteRawRC(RFCfgReg,0x7F);   //4F

   	   WriteRawRC(TReloadRegL,30);//tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 
	   WriteRawRC(TReloadRegH,0);
       WriteRawRC(TModeReg,0x8D);
	   WriteRawRC(TPrescalerReg,0x3E);

       PcdAntennaOn();
   }
   else{ return -1; }
   
   return MI_OK;
}


/////////////////////////////////////////////////////////////////////
//功  能：置RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//      mask[IN]:置位值
/////////////////////////////////////////////////////////////////////
void SetBitMask(UINT8 reg,UINT8 mask)  
{
    INT8 tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg,tmp | mask);  // set bit mask
}

/////////////////////////////////////////////////////////////////////
//功  能：清RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//      mask[IN]:清位值
/////////////////////////////////////////////////////////////////////
void ClearBitMask(UINT8 reg,UINT8 mask)  
{
    INT8 tmp = 0x0;
    tmp = ReadRawRC(reg);
    WriteRawRC(reg, tmp & ~mask);  // clear bit mask
} 

/////////////////////////////////////////////////////////////////////
//功  能：通过RC522和ISO14443卡通讯
//参数说明：Command[IN]:RC522命令字
//      pInData[IN]:通过RC522发送到卡片的数据
//      InLenByte[IN]:发送数据的字节长度
//      pOutData[OUT]:接收到的卡片返回数据
//      *pOutLenBit[OUT]:返回数据的位长度
/////////////////////////////////////////////////////////////////////
INT8 PcdComMF522(UINT8 Command, 
                 UINT8 *pInData, 
                 UINT8 InLenByte,
                 UINT8 *pOutData, 
                 UINT16  *pOutLenBit)
{
    INT8 status = MI_ERR;
    UINT8 irqEn   = 0x00;
    UINT8 waitFor = 0x00;
    UINT8 lastBits;
    UINT8 n;
    UINT16 i;
    switch (Command)
    {
       case PCD_AUTHENT://验证密钥
          irqEn   = 0x12;
          waitFor = 0x10;
          break;
       case PCD_TRANSCEIVE://发送并接收数据
          irqEn   = 0x77;
          waitFor = 0x30;
          break;
       default:
         break;
    }
   
    WriteRawRC(ComIEnReg,irqEn|0x80);//中断请求传递的使能和禁能控制寄存器
    ClearBitMask(ComIrqReg,0x80);//清除bit7，表示屏蔽位清零
    //可尝试删除,因为本文最后的部分已经执行过此指令了，节约1ms
    WriteRawRC(CommandReg,PCD_IDLE);//取消当前命令的执行
    //可尝试替代为WriteRawRC(FIFOLevelReg,0x80),节约1ms
    SetBitMask(FIFOLevelReg,0x80);//置位bit7，表示内部FIFO缓冲区的读写指针和ErrReg的BufferOvfl标志位立刻被清除
    
    for (i=0; i<InLenByte; i++)
    {   
		WriteRawRC(FIFODataReg, pInData[i]);//数据写入到FIFO中
	}
    WriteRawRC(CommandReg, Command);//将FIFO中的数据发送的天线并在发送完成后自动激活接收器
   
    
    if (Command == PCD_TRANSCEIVE)
    {    
		SetBitMask(BitFramingReg,0x80); //启动数据的发送
	}
    
  	i = 600;//600;//根据时钟频率调整，操作M1卡最大等待时间25ms
    do 
    {
         n = ReadRawRC(ComIrqReg);
         i--;
		 //delay_ms(1);
    }//等待600次，即25ms到了，或者定时器计时时间到了，或者天线接收到数据了，跳出循环
    while ((i!=0) && !(n&0x01) && !(n&waitFor));
    ClearBitMask(BitFramingReg,0x80);//清除启动发送位
	      
    if (i!=0)//非超时退出
    {    
    	//如果接收无错误
         if(!(ReadRawRC(ErrorReg)&0x1B))
         {
             status = MI_OK;
             if (n & irqEn & 0x01)//定时器溢出
             {   status = MI_NOTAGERR;   }
			 //是否应该加一个else,这样溢出后不会读取数据
             if (Command == PCD_TRANSCEIVE)
             {
               	n = ReadRawRC(FIFOLevelReg);//获取FIFO中保存的字节数
              	lastBits = ReadRawRC(ControlReg) & 0x07;//获取最后接收到的字节的有效位的数目
                if (lastBits)
                {   *pOutLenBit = (n-1)*8 + lastBits;   }
                else
                {   *pOutLenBit = n*8;   }
                if (n == 0)
                {   n = 1;    }
                if (n > MAXRLEN)
                {   n = MAXRLEN;   }
                for (i=0; i<n; i++)
                {   pOutData[i] = ReadRawRC(FIFODataReg);    }//从FIFO中读取接收的数据
            }
         }
         else//接收出现错误
         {   
			 status = MI_ERR;   
		  }
        
   }

   SetBitMask(ControlReg,0x80);           // stop timer now
   WriteRawRC(CommandReg,PCD_IDLE);//取消当前命令的执行 
   return status;
}


/////////////////////////////////////////////////////////////////////
//开启天线  
//每次启动或关闭天险发射之间应至少有1ms的间隔
/////////////////////////////////////////////////////////////////////
void PcdAntennaOn()
{
    UINT8 i;
    i = ReadRawRC(TxControlReg);
    if (!(i & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}


/////////////////////////////////////////////////////////////////////
//关闭天线
/////////////////////////////////////////////////////////////////////
void PcdAntennaOff()
{
    ClearBitMask(TxControlReg, 0x03);//后两位清零
}

void RFID_initial(void)
{
	PcdReset();//复位MFRC522芯片并初步配置
    PcdAntennaOff();//关闭天线使能
	delay_ms(20);
    PcdAntennaOn();//开启天线使能
	delay_ms(20);
	WriteRawRC(SerialSpeedReg,0x7A);//115200，参考手册2.2.2.14、7.3.2
	//WriteRawRC(SerialSpeedReg,0x9A);//57600
	//WriteRawRC(SerialSpeedReg,0xAB);//38400
	//WriteRawRC(SerialSpeedReg,0xCB);//19200
	delay_ms(20);
}
UINT8 rc_write_data[16] = {0x12,0x34,0x56,0x78,0xED,0xCB,0xA9,0x87,0x12,0x34,0x56,0x78,0x01,0xFE,0x01,0xFE};
//M1卡的某一块写为如下格式，则该块为钱包，可接收扣款和充值命令
//4字节金额（低字节在前）＋4字节金额取反＋4字节金额＋1字节块地址＋1字节块地址取反＋1字节块地址＋1字节块地址取反 
UINT8 data2[4]  = {0x12,0,0,0};
UINT8 DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};   
UINT8 g_ucTempbuf[20];

UINT8 RFID_SCAN(void)
{
	INT8 status,i;
	UINT16 SXX_ID;	
	#if DEBUG_RFID_DA1_WRITE_READ
	da1 = 0;
	#endif
	status = PcdRequest(PICC_REQALL, g_ucTempbuf);//寻卡
    if (status != MI_OK)//寻卡失败
    {   
		serail_number = 0;
		return 0;
	}
	#if DEBUG_RFID_DA1_WRITE_READ
	da1 = 50;
	#endif
	status = PcdAnticoll(g_ucTempbuf);//防重叠
    if (status != MI_OK)
	{
	   serail_number = 0;
	   return 0;  
	} 
    #if DEBUG_RFID_DA1_WRITE_READ
	da1 = 100;
	#endif
	status = PcdSelect(g_ucTempbuf);//选卡
    if (status != MI_OK)
	{
	   serail_number = 0;
	   return 0;  
	}     
    #if DEBUG_RFID_DA1_WRITE_READ
	da1 = 150;
	#endif     
	status = PcdAuthState(PICC_AUTHENT1A, 1, DefaultKey, g_ucTempbuf);//认证
    if (status != MI_OK)
	{
	   serail_number = 0;
	   return 0;  
	} 
    #if DEBUG_RFID_DA1_WRITE_READ
	da1 = 200;
	#endif     
    if(1 ==  rc522_write_falg)//需要写卡，写入值为Rfid_Nom
	{	
		//M1卡的数值块结构
		rc_write_data[0]  = Rfid_Nom&0xFF;//值
		rc_write_data[1]  = (Rfid_Nom>>8)&0xFF;
		rc_write_data[2]  = 0x00;
		rc_write_data[3]  = 0x00;
		rc_write_data[4]  = ~rc_write_data[0];//值按位取反
		rc_write_data[5]  = ~rc_write_data[1];
		rc_write_data[6]  = ~rc_write_data[2];
		rc_write_data[7]  = ~rc_write_data[3];
		rc_write_data[8]  = rc_write_data[0];//值
		rc_write_data[9]  = rc_write_data[1];
		rc_write_data[10] = rc_write_data[2];
		rc_write_data[11] = rc_write_data[3];
		rc_write_data[12] = 0x01;//块地址
		rc_write_data[13] = 0xFE;//块地址按位取反
		rc_write_data[14] = 0x01;//块地址
		rc_write_data[15] = 0xFE;//块地址按位取反
	    status = PcdWrite(1, rc_write_data);//写
	    if (status != MI_OK)
		{
		   rc522_write_ret_falg = 1; 
	       return 0; 
		}
		else rc522_write_ret_falg = 0;  //写成功后清除标志位
	}
	else
	{    
    	status = PcdRead(1, g_ucTempbuf);//读
    	if (status != MI_OK)
		{
	   		serail_number = 0;
	   		return 0;  
		} 
		else
		{
	       serail_number =(UINT16)g_ucTempbuf[0];//获取卡号
		   serail_number = serail_number|((UINT16)g_ucTempbuf[1])<<8;
		 	while(serail_number>999)
			{
				serail_number-=999;
			}
		}
	}
	#if DEBUG_RFID_DA1_WRITE_READ
	da1 = 225;
	#endif
    PcdHalt();	
	#if DEBUG_RFID_DA1_WRITE_READ
	da1 = 255;
	#endif
	return 1;
	
}
#endif
