//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : iic_bus.c
//  Description: 
//  Version    Date     Author    Description
//  0.01     19/07/06   liwenz        created
//  ...
//--------------------------------------------------------------------------------------
#include "..\..\include\common.h"       	//Common constants definition

#if ( ENABLE_CONFIG_PARA == 1 )||( INSTALLMENT ==1)
#include "..\..\include\sfr62p.h"           //M16C/62P special function register definitions
#include "..\..\include\iic_bus_eeprom.h"   
#include "..\..\include\typedef.h"     		//Data type define
#include "..\..\include\variables.h"        //External variables declaration       
#include "..\..\include\delay.h"

#define iic_sda_d	pd7_0
#define iic_sda		p7_0
#define iic_scl_d	pd7_1
#define iic_scl		p7_1

void _WaitTime0us(void);
void _WaitTime1us(void);
void _WaitTime2us(void);

#define _Wait_tHIGH		_WaitTime1us()	/* Clock pulse width high */
#define _Wait_tLOW		_WaitTime2us()	/* Clock pulse width low */
#define _Wait_tHD_STA	_WaitTime1us()	/* Start hold time */
#define _Wait_tSU_STA	_WaitTime1us()	/* Start setup time */
#define _Wait_tHD_DAT	_WaitTime0us()	/* Data in hold time */
#define _Wait_tSU_DAT	_WaitTime1us()	/* Data in setup time */
#define _Wait_tAA		_WaitTime1us()	/* Access time */
#define _Wait_tSU_STO	_WaitTime1us()	/* Stop setup time */
#define _Wait_tBUF		_WaitTime2us()	/* Bus free time for next mode */

#if FOLLOW_INPRESS_FUN_ENABLE
extern void calculate_inpress_angle(INT16 speed);
#endif
/************************************************************************************
 Name			: initIicBus
 Parameters		: None
 Returns		: None
 Description	: initialize I2C-BUS port
 Note			: 
************************************************************************************/
void initIicBus(void)
{
	iic_sda_d = 0;			/* SDA input ("H" state) */
	iic_scl_d = 0;			/* SCL input ("H" state) */
}


/************************************************************************************
 Name			: IicBusRead
 Parameters		: structure IicPack pointer
 Returns		: Acknowledge
 Description	: Sequential Ramdom Read Cycle (I2C-BUS)
 Note			: 
************************************************************************************/
UINT8 IicBusRead(IicPack *IicData)
{
	UINT8 i,ret;

	/* Ramdom Read Cycle / Sequential Ramdom Read Cycle */
	IicData->iic_DeviceAddress &= 0xFE;						/* WRITE Setting Device Address */
	StartCondition();										/* Start Condition */
	while (1) {
		if ((ret=ByteWrite(IicData->iic_DeviceAddress)) == NOACK)	/* WRITE Device Address */
			break;											/* NoAck Detect */
		if ((ret=ByteWrite(IicData->iic_MemoryAddress_h)) == NOACK)	/* WRITE Memory Address */
			break;											/* NoAck Detect */
		if ((ret=ByteWrite(IicData->iic_MemoryAddress_l)) == NOACK)	/* WRITE Memory Address */
			break;
		IicData->iic_DeviceAddress |= 0x01;					/* READ Setting Device Address */
		StartCondition();									/* ReStart Condition */
		if ((ret=ByteWrite(IicData->iic_DeviceAddress)) == NOACK)	/* WRITE Device Address */
			break;											/* NoAck Detect */
		for (i=1; i<IicData->iic_NumberOfByte; i++) {		/* specified bytes as loop */
			ByteRead(IicData->iic_Data, ACK);				/* Read data (Ack output) */
			IicData->iic_Data++;							/*  */
		}
		ByteRead(IicData->iic_Data, NOACK);					/* Read data (NoAck output) */
		break;
	}
	StopCondition();										/* Stop Condition */
	return(ret);
}


/************************************************************************************
 Name			: IicBusWrite
 Parameters		: structure IicPack pointer
 Returns		: Acknowledge
 Description	: Byte Write or Page Write Cycle (I2C-BUS)
 Note			: 
************************************************************************************/
UINT8 IicBusWrite(IicPack *IicData)
{
	UINT8 i,ret;

	/* Byte Write / Page Write */
	IicData->iic_DeviceAddress &= 0xFE;						/* WRITE Setting Device Address */
	StartCondition();										/* Start Condition */
	while (1) {
		if ((ret=ByteWrite(IicData->iic_DeviceAddress)) == NOACK)	/* Write Device Address */
			break;											/* NoAck Detect */
		if ((ret=ByteWrite(IicData->iic_MemoryAddress_h)) == NOACK)	/* Write Memory Addreess */
			break;											/* NoAck Detect */
		if ((ret=ByteWrite(IicData->iic_MemoryAddress_l)) == NOACK)	/* Write Memory Addreess */
			break;
		for (i=0; i<IicData->iic_NumberOfByte; i++) {		/* specified bytes as loop */
			if ((ret=ByteWrite(*(IicData->iic_Data))) == NOACK)	/* Write Data */
				break;										/* NoAck Detect */
			IicData->iic_Data++;							/*  */
		}
		break;
	}
	StopCondition();										/* Stop Condition */
	return(ret);
}


/************************************************************************************
 Name			: StartCondition
 Parameters		: None
 Returns		: None
 Description	: Output Start Condition (I2C-BUS)
 Note			: *1 adjust a wait time
************************************************************************************/
void StartCondition(void)
{
	iic_scl = 0;				/* SCL="L" */
	iic_scl_d = 1;				/* SCL output */
	_WaitTime1us();				/* wait *1 */
	iic_sda_d = 0;				/* SDA="H" */
	_WaitTime1us();				/* wait */
	_WaitTime1us();				/* wait *! */
	iic_scl = 1;				/* SCL="H" */
	_Wait_tSU_STA;				/* wait */
	iic_sda = 0;				/* SDA="L" */
	iic_sda_d = 1;				/* SDA output */
	_Wait_tHD_STA;				/* wait */
	_WaitTime1us();				/* wait *1 */
	iic_scl = 0;				/* SCL="L" */
}


/************************************************************************************
 Name			: StopCondition
 Parameters		: None
 Returns		: None
 Description	: Output Stop Condition (I2C-BUS)
 Note			: *1 adjust a wait time
************************************************************************************/
void StopCondition(void)
{
	iic_scl = 0;				/* SCL="L" */
	iic_scl_d = 1;				/* SCL output */
	_WaitTime1us();				/* wait *1 */
	iic_sda = 0;				/* SDA="L" */
	iic_sda_d = 1;				/* SDA output */
	_WaitTime1us();				/* wait *1 */
	iic_scl = 1;				/* SCL="H" */
	_Wait_tSU_STO;				/* wait */
	iic_sda_d = 0;				/* SDA="H" */
	_WaitTime1us();				/* wait */
	_WaitTime1us();				/* wait *1 */
	iic_scl = 0;				/* SCL="L" */
}


/************************************************************************************
 Name			: ByteWrite
 Parameters		: Write data
 Returns		: Acknowledge
 Description	: byte data Output (I2C-BUS)
 Note			: *1 adjust a wait time
************************************************************************************/
UINT8 ByteWrite(UINT8 iic_writeData)
{
	UINT8 maskData=0x80;		/* MSB first */
	UINT8 ret=ACK;				/* Ack/NoAck */

	while (maskData) {						/* 8times as loop */
		iic_sda = 0;						/* initialize port-latch */
		if (iic_writeData & maskData) {		/* "H" output ? */
			iic_sda_d = 0;					/*Yes SDA="H" */
		}else{
			iic_sda_d = 1;					/* No  SDA="L" */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
		}
		_Wait_tSU_DAT;						/* wait */
		_WaitTime1us();						/* wait *1 */
		iic_scl = 1;						/* SCL="H" */
		_Wait_tHIGH;						/* wait */
		_WaitTime1us();						/* wait *1 */
		iic_scl = 0;						/* SCL="L" */
		maskData >>= 1;						/* change mask data */
		_WaitTime1us();						/* wait *1 */
	}
	iic_sda_d = 0;							/* SDA input */
	_Wait_tAA;								/* wait */
	_WaitTime2us();							/* wait *1 */
	iic_scl = 1;							/* SCL="H" */
	if (iic_sda) ret=NOACK;					/* NoAck Detect */
	_Wait_tHIGH;							/* wait */
	_WaitTime1us();							/* wait *1 */
	iic_scl = 0;							/* SCL="L" */
	_Wait_tHD_DAT;							/* wait */
	return(ret);
}


/************************************************************************************
 Name			: ByteRead
 Parameters		: Read data strage location pointer, Select Ack/NoAck
 Returns		: None
 Description	: byte data input with Ack output (I2C-BUS)
 Note			: *1 adjust a wait time
************************************************************************************/
void ByteRead(UINT8 *iic_readData, UINT8 ackData)
{
	UINT8 maskData=0x80;		/* MSB first */
	UINT8 readData;

	*iic_readData = 0;						/*  */
	while (maskData) {						/* 8times as loop */
		readData = *iic_readData | maskData;	/*  */
		iic_sda_d = 0;						/* initialize port-latch */
		_Wait_tAA;							/* wait */
		iic_scl = 1;						/* SCL="H" */
		if (iic_sda) {						/* SDA="H" ? */
			*iic_readData = readData;		/* Yes  */
		}else{
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
		}
		_Wait_tHIGH;						/* wait */
		_WaitTime1us();						/* wait *1 */
		iic_scl = 0;						/* SCL="L" */
		maskData >>= 1;						/* Change mask data */
		_WaitTime1us();						/* wait *1 */
	}
	if (!ackData) {							/* Ack output ? */
	/* Ack output */
		iic_sda = ACK;						/* Yes SDA="L" */
		iic_sda_d = 1;						/* SDA output */
	}else{
	/* NoAck output */
		iic_sda = NOACK;					/* No  SDA="H" */
		iic_sda_d = 0;						/* SDA input */
	}
	_Wait_tSU_DAT;							/* wait */
	_WaitTime1us();							/* wait *1 */
	iic_scl = 1;							/* SCL="H" */
	_Wait_tHIGH;							/* wait */
	_WaitTime1us();							/* wait *1 */
	iic_scl = 0;							/* SCL="L" */
	iic_sda_d = 0;							/* SDA input */
	_WaitTime1us();							/* wait *1 */
}


/************************************************************************************
 Name			: _WaitTime0us
 Parameters		: None
 Returns		: None
 Description	: a 0us wait
************************************************************************************/
void _WaitTime0us(void)
{
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
}


/************************************************************************************
 Name			: _WaitTime1us
 Parameters		: None
 Returns		: None
 Description	: a 1us wait
************************************************************************************/
void _WaitTime1us(void)
{
	/* +14cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle = 24cycle */
}


/************************************************************************************
 Name			: _WaitTime2us
 Parameters		: None
 Returns		: None
 Description	: a 2us wait
************************************************************************************/
void _WaitTime2us(void)
{
	/* +14cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle = 48cycle */
}


//-------------------------------------------------------------------------------------------------------
//  以下为eeprom操作部分
//-------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
//  Name:		
//  Parameters:	
//  Returns:    	
//  Description: 
//--------------------------------------------------------------------------------------
void init_eeprom(void)
{	
    initIicBus();
}
/************************************************************************************
 Name			: _WaitTime6ms
 Parameters		: None
 Returns		: None
 Description	: a 6ms wait
************************************************************************************/
void _WaitTime6ms(void)
{
	INT16 iee;
	for(iee=0;iee<6000;iee++)
    {
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle = 24cycle */	    
	}
}
UINT16 read_par(UINT16 par_num)
{
	UINT16 a1 = 0; 
	IicPack IicData_r; 	 
	IicData_r.iic_DeviceAddress = 0xA0;
	IicData_r.iic_MemoryAddress_h = (UINT8)(par_num>>7);
	IicData_r.iic_MemoryAddress_l = (UINT8)(par_num<<1);
	IicData_r.iic_Data = (UINT8*)(&a1);
	IicData_r.iic_NumberOfByte = 2;
	if(IicBusRead(&IicData_r) == NOACK) 
	{	
		sys.error = ERROR_20;
		sys.status = ERROR;
	}	
	return a1;	 
}
void write_par(UINT16 par_num,INT16 par)
{	
	IicPack IicData_w;				
	IicData_w.iic_DeviceAddress = 0xA0;
	IicData_w.iic_MemoryAddress_h = (UINT8)(par_num>>7);
	IicData_w.iic_MemoryAddress_l = (UINT8)(par_num<<1);
	IicData_w.iic_Data = (UINT8*)(&par);
	IicData_w.iic_NumberOfByte = 2;
	if(IicBusWrite(&IicData_w)== NOACK) 
	{	
		sys.error = ERROR_21;
		sys.status = ERROR;
	}
	_WaitTime6ms();
}

	#if  ENABLE_CONFIG_PARA == 1
	void read_para_group( UINT16 address,UINT8 *point,UINT16 len)
	{
		UINT16 i,data;
		i = 0;
		while( i<len )
		{
			data = read_par(address + i);
			*point++ = (data >> 8)&0xff;
			*point++ = data&0xff;
			i+=2;
		}
	}

	void write_para_group( UINT16 address,UINT8 *point,UINT16 len)
	{
		UINT16 i,data;
		i = 0;
		while( i<len )
		{
			data = *point++;
			data = (data <<8)+ *point++;
			write_par(address + i,data);
			_WaitTime6ms();
			i+=2;
		}
	}



	UINT16 string2int(UINT8 *src)
	{
		return(  (((UINT16)src[0])<<8 ) + src[1] );
	}

	void int2string( UINT8 *src,UINT16 dat)
	{
		src[0] = (dat>>8) &0xff;
		src[1] =  dat&0xff;
	}

	void restore_para_from_eeprom(void)
	{
		UINT16 index,i;
		index = 0;
		read_para_group(100,svpara_disp_buf,205);

		para.DSP1_para_1F   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP1_para_20   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP1_para_21   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP1_para_22   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP1_para_23   = string2int(&svpara_disp_buf[index]);  index +=2;
		para.DSP1_para_27   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP1_para_28H  = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP1_para_28M1 = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP1_para_28M2 = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP1_para_28L  = string2int(&svpara_disp_buf[index]);	index +=2;
	
		para.DSP2_para_1F   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP2_para_20   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP2_para_21   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP2_para_22   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP2_para_23   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP2_para_27   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP2_para_28H  = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP2_para_28M1 = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP2_para_28M2 = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP2_para_28L  = string2int(&svpara_disp_buf[index]);	index +=2;
	
		para.dsp1A_half_current = svpara_disp_buf[index++];
		para.dsp1B_half_current = svpara_disp_buf[index++];
		para.dsp2A_half_current = svpara_disp_buf[index++];
		para.dsp2B_half_current = svpara_disp_buf[index++];
		
		para.platform_type = svpara_disp_buf[index++];			
		para.mainmotor_type = svpara_disp_buf[index++];
		para.x_origin_mode = svpara_disp_buf[index++];		
		para.yj_org_direction = svpara_disp_buf[index++];
		para.Corner_deceleration_speed = svpara_disp_buf[index++];
		para.wipper_type = svpara_disp_buf[index++];
		para.x_sensor_open_level = svpara_disp_buf[index++];
		para.y_sensor_open_level = svpara_disp_buf[index++];
		para.laser_function_enable = svpara_disp_buf[index++];
		para.dvb_open_level = svpara_disp_buf[index++];
		para.last_8_speed = svpara_disp_buf[index++];
		para.last_7_speed = svpara_disp_buf[index++];
		para.last_6_speed = svpara_disp_buf[index++];
		para.last_5_speed = svpara_disp_buf[index++];
		para.last_4_speed = svpara_disp_buf[index++];
		para.last_3_speed = svpara_disp_buf[index++];
		para.last_2_speed = svpara_disp_buf[index++];
		para.last_1_speed = svpara_disp_buf[index++];
		para.dva_open_level = svpara_disp_buf[index++]; 
		para.dsp1_step_crc   = string2int(&svpara_disp_buf[index]);			index +=2;
		para.dsp2_step_crc   = string2int(&svpara_disp_buf[index]);			index +=2;
	
		para.y_backward_dis   = string2int(&svpara_disp_buf[index]);		index +=2;
		para.x_take_offset   = string2int(&svpara_disp_buf[index]);			index +=2;
		para.x_take_offset2   = string2int(&svpara_disp_buf[index]);		index +=2;
	
		para.left_barcode_position   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.right_barcode_position   = string2int(&svpara_disp_buf[index]);index +=2;
		para.catch_delay_time   = string2int(&svpara_disp_buf[index]);		index +=2;
	
		para.y_barcode_position = string2int(&svpara_disp_buf[index]);	index +=2;
		para.blow_air_counter = string2int(&svpara_disp_buf[index]);	index +=2;
		para.cut_air_counter = string2int(&svpara_disp_buf[index]);		index +=2;
		para.yj_go_origin_enable = svpara_disp_buf[index++];
		para.second_origin_footer_status = svpara_disp_buf[index++];
		para.go_special_position_mode = svpara_disp_buf[index++];
		para.dsp3_step_crc   = string2int(&svpara_disp_buf[index]);			index +=2;
		para.dsp4_step_crc   = string2int(&svpara_disp_buf[index]);			index +=2;
		para.zx_org_direction = svpara_disp_buf[index++];//93
	
		para.DSP3_para_1F   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP3_para_20   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP3_para_21   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP3_para_22   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP3_para_23   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP3_para_27   = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP3_para_28H  = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP3_para_28M1 = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP3_para_28M2 = string2int(&svpara_disp_buf[index]);	index +=2;
		para.DSP3_para_28L  = string2int(&svpara_disp_buf[index]);	index +=2;
	
		para.dsp3A_half_current = svpara_disp_buf[index++];//114
		para.dsp3B_half_current = svpara_disp_buf[index++];//115
		para.dsp3A_current = svpara_disp_buf[index++];//116
		para.dsp3B_current = svpara_disp_buf[index++];//117
		para.dsp3_enable = svpara_disp_buf[index++];//118
	
		para.dsp3a_motor_dir = svpara_disp_buf[index++];//119
		para.dsp3b_motor_dir = svpara_disp_buf[index++];//120
		para.dsp3a_sensor_dir = svpara_disp_buf[index++];//121
		para.dsp3b_sensor_dir = svpara_disp_buf[index++];//122
		para.bobbin_platform_speed = svpara_disp_buf[index++];//123
		para.bobbin_shake_distance = svpara_disp_buf[index++];//124
		para.bobbin_shake_time = svpara_disp_buf[index++];//125
		para.rotate_cutter_delaytime1 = svpara_disp_buf[index++];//126
		para.rotate_cutter_delaytime2 = svpara_disp_buf[index++];//127
		para.rotate_cutter_movetime = svpara_disp_buf[index++];//128
		para.rotate_cutter_working_mode = svpara_disp_buf[index++];//129
		para.rotate_cutter_detect_valve_down = svpara_disp_buf[index++];//130
		
		para.x_motor_dir = svpara_disp_buf[index++];//131
		para.y_motor_dir = svpara_disp_buf[index++];//132
		para.zx_motor_dir = svpara_disp_buf[index++];//133
		para.cutter_speed = string2int(&svpara_disp_buf[index]);	index +=2;//134,135
		para.speed_limit_switch = svpara_disp_buf[index++];//136
		para.speed_percent = svpara_disp_buf[index++];//137
		para.one_key_run_function = svpara_disp_buf[index++];//138
		para.thread_break_backward_switch = svpara_disp_buf[index++];//139
		para.thread_break_backward_stitchs = svpara_disp_buf[index++];//140
		
		para.slow_start_mode = svpara_disp_buf[index++];//141
		
		para.stitch1_speed = svpara_disp_buf[index++];//142
		para.stitch2_speed = svpara_disp_buf[index++];//143
		para.stitch3_speed = svpara_disp_buf[index++];//144
		para.stitch4_speed = svpara_disp_buf[index++];//145
		para.stitch5_speed = svpara_disp_buf[index++];//146

		para.first_5_adjust = svpara_disp_buf[index++];//147
		
		para.Corner_deceleration_speed1 = svpara_disp_buf[index++];//148
		para.Corner_deceleration_speed2 = svpara_disp_buf[index++];//149
		para.Corner_deceleration_speed3 = svpara_disp_buf[index++];//150
		para.Corner_deceleration_speed4 = svpara_disp_buf[index++];//151
		
		para.slow_start_stitchs = svpara_disp_buf[index++];//152
		if( para.slow_start_stitchs < 5)
			para.slow_start_stitchs = 5;
		if( para.slow_start_stitchs >15)
			para.slow_start_stitchs = 15;
		for(i=0;i<15;i++)
		{
			para.slow_start_speed[i] = svpara_disp_buf[index++];//153~167
		}
		para.adjust_mode_enable = svpara_disp_buf[index++];//168

		//2018-8-4
		para.start_sew_change_inpress_high_enable = svpara_disp_buf[index++];//169
		para.start_sew_change_inpress_high_stitchs = svpara_disp_buf[index++];//170
		//针数是否应该加一个限制
		para.start_sew_change_inpress_high_range = svpara_disp_buf[index++];//171
		#if FOLLOW_INPRESS_FUN_ENABLE
		if(para.start_sew_change_inpress_high_enable == 55)
		{
			if(para.start_sew_change_inpress_high_stitchs == 0 || para.start_sew_change_inpress_high_range == 0)
			{
				para.start_sew_change_inpress_high_enable = 0;//只要针数和随动范围有一个为0，那么就关闭此功能
			}
			else
			{
				if(connect_flag == 1)//如果联机以后才调用此函数，说明对系统参数第1组进行了修改
				{
					calculate_inpress_angle(200);//参数实际上是随意设置的，我们使用此函数只为更新inpress_follow_range
					if(para.start_sew_change_inpress_high_range > inpress_follow_range)
					{
						//取值太大了，直接关闭此功能
						para.start_sew_change_inpress_high_range = 0;
						para.start_sew_change_inpress_high_enable = 0;
					}
				}
			}
		}
		#endif
		
	}

	void cpy_para_buff(void)
	{
		UINT16 index,i;
		index = 0;
		int2string(&svpara_disp_buf[index],para.DSP1_para_1F);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP1_para_20);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP1_para_21);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP1_para_22);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP1_para_23);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP1_para_27);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP1_para_28H); index += 2;
		int2string(&svpara_disp_buf[index],para.DSP1_para_28M1);index += 2;
		int2string(&svpara_disp_buf[index],para.DSP1_para_28M2);index += 2;
		int2string(&svpara_disp_buf[index],para.DSP1_para_28L); index += 2;

		int2string(&svpara_disp_buf[index],para.DSP2_para_1F);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP2_para_20);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP2_para_21);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP2_para_22);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP2_para_23);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP2_para_27);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP2_para_28H); index += 2;
		int2string(&svpara_disp_buf[index],para.DSP2_para_28M1);index += 2;
		int2string(&svpara_disp_buf[index],para.DSP2_para_28M2);index += 2;
		int2string(&svpara_disp_buf[index],para.DSP2_para_28L); index += 2;	
	
		svpara_disp_buf[index++] = para.dsp1A_half_current;
		svpara_disp_buf[index++] = para.dsp1B_half_current;
		svpara_disp_buf[index++] = para.dsp2A_half_current;
		svpara_disp_buf[index++] = para.dsp2B_half_current;
		
		svpara_disp_buf[index++] = para.platform_type;			
		svpara_disp_buf[index++] = para.mainmotor_type;
		svpara_disp_buf[index++] = para.x_origin_mode;		
		svpara_disp_buf[index++] = para.yj_org_direction;
		svpara_disp_buf[index++] = para.Corner_deceleration_speed;
		svpara_disp_buf[index++] = para.wipper_type;
		svpara_disp_buf[index++] = para.x_sensor_open_level;
		svpara_disp_buf[index++] = para.y_sensor_open_level;
		svpara_disp_buf[index++] = para.laser_function_enable;
		svpara_disp_buf[index++] = para.dvb_open_level;
		svpara_disp_buf[index++] = para.last_8_speed;
		svpara_disp_buf[index++] = para.last_7_speed;
		svpara_disp_buf[index++] = para.last_6_speed;
		svpara_disp_buf[index++] = para.last_5_speed;
		svpara_disp_buf[index++] = para.last_4_speed;
		svpara_disp_buf[index++] = para.last_3_speed;
		svpara_disp_buf[index++] = para.last_2_speed;
		svpara_disp_buf[index++] = para.last_1_speed;
		svpara_disp_buf[index++] = para.dva_open_level;
		int2string(&svpara_disp_buf[index],para.dsp1_step_crc);  index += 2;
		int2string(&svpara_disp_buf[index],para.dsp2_step_crc);  index += 2;//66,67
	
		int2string(&svpara_disp_buf[index],para.y_backward_dis);  index += 2;//68,69
		int2string(&svpara_disp_buf[index],para.x_take_offset);  index += 2; //70,71
		int2string(&svpara_disp_buf[index],para.x_take_offset2);  index += 2;//72,73

		int2string(&svpara_disp_buf[index],para.left_barcode_position);  index += 2; //74,75
		int2string(&svpara_disp_buf[index],para.right_barcode_position);  index += 2;//76,77

		int2string(&svpara_disp_buf[index],para.catch_delay_time);  index += 2;//78,79
		int2string(&svpara_disp_buf[index],para.y_barcode_position);  index += 2;//80,81
	
		int2string(&svpara_disp_buf[index],para.blow_air_counter);  index += 2;//82,83
		int2string(&svpara_disp_buf[index],para.cut_air_counter);  index += 2;//
	
		svpara_disp_buf[index++] = para.yj_go_origin_enable;
		svpara_disp_buf[index++] = para.second_origin_footer_status;
		svpara_disp_buf[index++] = para.go_special_position_mode;
	
		int2string(&svpara_disp_buf[index],para.dsp3_step_crc);  index += 2;
		int2string(&svpara_disp_buf[index],para.dsp4_step_crc);  index += 2;
	
		svpara_disp_buf[index++] = para.zx_org_direction;//93
	
		int2string(&svpara_disp_buf[index],para.DSP3_para_1F);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP3_para_20);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP3_para_21);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP3_para_22);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP3_para_23);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP3_para_27);  index += 2;
		int2string(&svpara_disp_buf[index],para.DSP3_para_28H); index += 2;
		int2string(&svpara_disp_buf[index],para.DSP3_para_28M1);index += 2;
		int2string(&svpara_disp_buf[index],para.DSP3_para_28M2);index += 2;
		int2string(&svpara_disp_buf[index],para.DSP3_para_28L); index += 2;	
	
		svpara_disp_buf[index++] = para.dsp3A_half_current;//114
		svpara_disp_buf[index++] = para.dsp3B_half_current;//115
		svpara_disp_buf[index++] = para.dsp3A_current;//116
		svpara_disp_buf[index++] = para.dsp3B_current;//117
		svpara_disp_buf[index++] = para.dsp3_enable;//118
	
		svpara_disp_buf[index++] = para.dsp3a_motor_dir;//119
		svpara_disp_buf[index++] = para.dsp3b_motor_dir;//120
		svpara_disp_buf[index++] = para.dsp3a_sensor_dir;//121
		svpara_disp_buf[index++] = para.dsp3b_sensor_dir;//122
		svpara_disp_buf[index++] = para.bobbin_platform_speed;//123
		svpara_disp_buf[index++] = para.bobbin_shake_distance;//124
		svpara_disp_buf[index++] = para.bobbin_shake_time;//125
		svpara_disp_buf[index++] = para.rotate_cutter_delaytime1;//126
		svpara_disp_buf[index++] = para.rotate_cutter_delaytime2;//127
		svpara_disp_buf[index++] = para.rotate_cutter_movetime;  //128
		svpara_disp_buf[index++] = para.rotate_cutter_working_mode;//129
		svpara_disp_buf[index++] = para.rotate_cutter_detect_valve_down;//130
		
		svpara_disp_buf[index++] = para.x_motor_dir;//131
		svpara_disp_buf[index++] = para.y_motor_dir;//132
		svpara_disp_buf[index++] = para.zx_motor_dir;//133
	    
		int2string(&svpara_disp_buf[index],para.cutter_speed); index += 2;//134,135
		svpara_disp_buf[index++] = para.speed_limit_switch;//136
		svpara_disp_buf[index++] = para.speed_percent;//137
		svpara_disp_buf[index++] = para.one_key_run_function;//138
		svpara_disp_buf[index++] = para.thread_break_backward_switch;//139
		svpara_disp_buf[index++] = para.thread_break_backward_stitchs;//140
		svpara_disp_buf[index++] = para.slow_start_mode;//141
		
		svpara_disp_buf[index++] = para.stitch1_speed;//142
		svpara_disp_buf[index++] = para.stitch2_speed;//143
		svpara_disp_buf[index++] = para.stitch3_speed;//144
		svpara_disp_buf[index++] = para.stitch4_speed;//145
		svpara_disp_buf[index++] = para.stitch5_speed;//146
		svpara_disp_buf[index++] = para.first_5_adjust;//147
		
		svpara_disp_buf[index++] = para.Corner_deceleration_speed1;//148		
		svpara_disp_buf[index++] = para.Corner_deceleration_speed2;//149
		svpara_disp_buf[index++] = para.Corner_deceleration_speed3;//150
		svpara_disp_buf[index++] = para.Corner_deceleration_speed4;//151
		
		svpara_disp_buf[index++] = para.slow_start_stitchs;//152
		
		for(i=0;i<15;i++)
		{
			svpara_disp_buf[index++] = para.slow_start_speed[i];//153~167
		}
		svpara_disp_buf[index++] = para.adjust_mode_enable;//168

		//2018-8-4
		svpara_disp_buf[index++] = para.start_sew_change_inpress_high_enable;//169
		svpara_disp_buf[index++] = para.start_sew_change_inpress_high_stitchs;//170
		svpara_disp_buf[index++] = para.start_sew_change_inpress_high_range;//171

		
		svpara_disp_buf[index++] = 0;
		svpara_disp_buf[index++] = 0;
	}

	void get_para_from_eeprom(void)
	{
		UINT8 i,index;
		UINT16 tmp16;
		read_para_group(700,svpara_disp_buf,205);
		//send_para_to_stm32();//前边参数先下发给STM32
		//index = MAIN_CONTROL_PARA_OFFSET<<1;
		//for(i=1;i<64;i++)
		//{
		//	tmp16 = svpara_disp_buf[index++];
		//	tmp16 = tmp16*256 + svpara_disp_buf[index++];
			//system_delay_time[i] = tmp16;
		//}
		index = 0;
		for( i=0;i<40;i++)
			para5.x_time_adjust[i] = svpara_disp_buf[index++];
		for( i=0;i<40;i++)
			para5.x_angle_adjust[i] = svpara_disp_buf[index++];
		for( i=0;i<40;i++)
			para5.y_time_adjust[i] = svpara_disp_buf[index++];
		for( i=0;i<40;i++)
			para5.y_angle_adjust[i] = svpara_disp_buf[index++];
	}

	void init_para_variables(void)
	{
		restore_para_from_eeprom();
		if((para.DSP1_para_1F ==0XFFFF)||(para.DSP1_para_1F ==0X0000))
		{
			para.platform_type = 1;
	
			para.DSP1_para_1F   = 0x0101;
			para.DSP1_para_20   = 1000;
			para.DSP1_para_21   = 1000;
			para.DSP1_para_22   = 2048;
			para.DSP1_para_23   = 2048;

			para.DSP1_para_27   = 100;
			para.DSP1_para_28H  = (128<<6)+40;
			para.DSP1_para_28M1 = (128<<6)+40;
			para.DSP1_para_28M2 = (3<<12);
			para.DSP1_para_28L  = 3<<12;
	
			para.DSP2_para_1F   = 0x0101;
			para.DSP2_para_20   = 400;
			para.DSP2_para_21   = 1000;
			para.DSP2_para_22   = 8192;
			para.DSP2_para_23   = 8192;

			para.DSP2_para_27   = 100;
			para.DSP2_para_28H  = (128<<6)+40;
			para.DSP2_para_28M1 = (128<<6)+40;
			para.DSP2_para_28M2 = (3<<12)+4095;
			para.DSP2_para_28L  = (3<<12)+4095;
			para.x_origin_mode = 0;
			para.Corner_deceleration_speed = 12;
			para.wipper_type = 0;
			para.x_sensor_open_level = 0;
			para.y_sensor_open_level = 0;
			para.laser_function_enable = 0;
			para.last_1_speed = 4;
			para.last_2_speed = 6;
			para.last_3_speed = 8;
			para.last_4_speed = 10;
			para.last_5_speed = 13;
			para.last_6_speed = 16;
			para.last_7_speed = 18;
			para.last_8_speed = 22;
			para.dvb_open_level = 0;
			para.dva_open_level = 0;
			para.dsp1_step_crc = 0;
			para.dsp2_step_crc = 0;
			para.y_backward_dis = 0;
			para.x_take_offset = 0;
			para.x_take_offset2 = 0;
			para.catch_delay_time = 0;
			para.y_barcode_position = 0;
			para.blow_air_counter = 0;
			para.cut_air_counter = 0;
			para.yj_go_origin_enable = 0;
			para.second_origin_footer_status = 0;
		
			para.DSP3_para_1F   = 0x0101;
			para.DSP3_para_20   = 1000;
			para.DSP3_para_21   = 1000;
			para.DSP3_para_22   = 2048;
			para.DSP3_para_23   = 4096;

			para.DSP3_para_27   = 100;
			para.DSP3_para_28H  = (128<<6)+40;
			para.DSP3_para_28M1 = (128<<6)+40;
			para.DSP3_para_28M2 = (3<<12)+4095;
			para.DSP3_para_28L  = 3<<12;
			para.dsp3A_current = 3;
			para.dsp3B_current = 3;
			para.dsp3A_half_current = 2;
			para.dsp3B_half_current = 2;
			para.cutter_speed  = 1000;
			cpy_para_buff();
			write_para_group(100,svpara_disp_buf,205);		
		}	
		get_para_from_eeprom();
				
	}
	
	#endif
#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
