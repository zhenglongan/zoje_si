#include "..\..\include\common.h"         // Common constants definition

#if ENABLE_RFID_FUNCTION

#include "..\..\include\sfr62p.h"         // M16C/62P special function register definitions
#include "..\..\include\variables.h" 
#include "..\..\include\delay.h" 
#include "..\..\include\action.h" 
#include "..\..\include\MFRC522.h" 


#define MAXRLEN 18

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

   ClearBitMask(Status2Reg,0x08);
   WriteRawRC(BitFramingReg,0x07);
   SetBitMask(TxControlReg,0x03);
 
   ucComMF522Buf[0] = req_code;

   status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);
   
   if ((status == MI_OK) && (unLen == 0x10))
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
  

  ClearBitMask(Status2Reg,0x08);
  WriteRawRC(BitFramingReg,0x00);
  ClearBitMask(CollReg,0x80);
 
  ucComMF522Buf[0] = PICC_ANTICOLL1;
  ucComMF522Buf[1] = 0x20;

  status = PcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

  if (status == MI_OK)
  {
  	 for (i=0; i<4; i++)
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
  CalulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
  
  ClearBitMask(Status2Reg,0x08);

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
	  ucComMF522Buf[i+8] = *(pSnr+i);   
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

  if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
  {   status = MI_ERR;   }
    
  if (status == MI_OK)
  {
    for (i=0; i<16; i++)
    {  
		ucComMF522Buf[i] = *(pData+i);   
	}
    CalulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);

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

/////////////////////////////////////////////////////////////////////
//用MF522计算CRC16函数
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
  pOutData[0] = ReadRawRC(CRCResultRegL);
  pOutData[1] = ReadRawRC(CRCResultRegM);
}

/////////////////////////////////////////////////////////////////////
//功  能：复位RC522
//返  回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
INT8 PcdReset(void)
{	
  	WriteRawRC(CommandReg,PCD_RESETPHASE);
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
  	WriteRawRC(ModeReg,0x3D);            //和Mifare卡通讯，CRC初始值0x6363
	delay_ms(30); 
/*
定时器重装值 1e00
*/	
  	WriteRawRC(TReloadRegL,30); 
	delay_ms(30);          
  	WriteRawRC(TReloadRegH,0);
	delay_ms(30); 
/*
b7: 1 定时器在所有速率发送结束时自动启动，接收第一个数据位定时器立即停止
b6~b5: 00 非门控模式
b4: 0 定时器递减到0,timerIRq置1
b3~b0: 分频的高4位  ftimer= 6.78MHz/分频值
*/	
  	WriteRawRC(TModeReg,0x8D);
	delay_ms(30); 
  	WriteRawRC(TPrescalerReg,0x3E);
	delay_ms(30); 
/*

*/	
  	WriteRawRC(TxAskReg,0x40); 
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
       case PCD_AUTHENT:
          irqEn   = 0x12;
          waitFor = 0x10;
          break;
       case PCD_TRANSCEIVE:
          irqEn   = 0x77;
          waitFor = 0x30;
          break;
       default:
         break;
    }
   
    WriteRawRC(ComIEnReg,irqEn|0x80);
    ClearBitMask(ComIrqReg,0x80);
    WriteRawRC(CommandReg,PCD_IDLE);
    SetBitMask(FIFOLevelReg,0x80);
    
    for (i=0; i<InLenByte; i++)
    {   
		WriteRawRC(FIFODataReg, pInData[i]);    
	}
    WriteRawRC(CommandReg, Command);
   
    
    if (Command == PCD_TRANSCEIVE)
    {    
		SetBitMask(BitFramingReg,0x80);  
	}
    
  	i = 600;//600;//根据时钟频率调整，操作M1卡最大等待时间25ms
    do 
    {
         n = ReadRawRC(ComIrqReg);
         i--;
		 //delay_ms(1);
    }
    while ((i!=0) && !(n&0x01) && !(n&waitFor));
    ClearBitMask(BitFramingReg,0x80);
	      
    if (i!=0)
    {    
         if(!(ReadRawRC(ErrorReg)&0x1B))
         {
             status = MI_OK;
             if (n & irqEn & 0x01)
             {   status = MI_NOTAGERR;   }
             if (Command == PCD_TRANSCEIVE)
             {
               	n = ReadRawRC(FIFOLevelReg);
              	lastBits = ReadRawRC(ControlReg) & 0x07;
                if (lastBits)
                {   *pOutLenBit = (n-1)*8 + lastBits;   }
                else
                {   *pOutLenBit = n*8;   }
                if (n == 0)
                {   n = 1;    }
                if (n > MAXRLEN)
                {   n = MAXRLEN;   }
                for (i=0; i<n; i++)
                {   pOutData[i] = ReadRawRC(FIFODataReg);    }
            }
         }
         else
         {   
			 status = MI_ERR;   
		  }
        
   }

   SetBitMask(ControlReg,0x80);           // stop timer now
   WriteRawRC(CommandReg,PCD_IDLE); 
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
    ClearBitMask(TxControlReg, 0x03);
}

void RFID_initial(void)
{
	PcdReset();
    PcdAntennaOff(); 
	delay_ms(20);
    PcdAntennaOn();
	delay_ms(20);
	WriteRawRC(SerialSpeedReg,0x7A);//115200
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
    if (status != MI_OK)
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
    if(1 ==  rc522_write_falg)
	{	
		rc_write_data[0]  = Rfid_Nom&0xFF;
		rc_write_data[1]  = (Rfid_Nom>>8)&0xFF;
		rc_write_data[2]  = 0x00;
		rc_write_data[3]  = 0x00;
		rc_write_data[4]  = ~rc_write_data[0];
		rc_write_data[5]  = ~rc_write_data[1];
		rc_write_data[6]  = ~rc_write_data[2];
		rc_write_data[7]  = ~rc_write_data[3];
		rc_write_data[8]  = rc_write_data[0];
		rc_write_data[9]  = rc_write_data[1];
		rc_write_data[10] = rc_write_data[2];
		rc_write_data[11] = rc_write_data[3];
		rc_write_data[12] = 0x01;
		rc_write_data[13] = 0xFE;
		rc_write_data[14] = 0x01;
		rc_write_data[15] = 0xFE;
	    status = PcdWrite(1, rc_write_data);//写
	    if (status != MI_OK)
		{
		   rc522_write_ret_falg = 1; 
	       return 0; 
		}
		else rc522_write_ret_falg = 0;  
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
	       serail_number =(UINT16)g_ucTempbuf[0];
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