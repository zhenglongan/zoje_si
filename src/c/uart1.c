#include "..\..\include\sfr62p.h"           // M16C/62P special function register definitions
#include "..\..\include\typedef.h"          // data type define
#include "..\..\include\common.h"           // External variables declaration
#include "..\..\include\variables.h"        // External variables declaration
#include "..\..\include\initial.h"          // sine table 
#include "..\..\include\delay.h"            // delay time definition    
#include "..\..\include\stepmotor.h"        // stepmotor driver

#define  BUF_MIN  254
#define  BUF_MID  254


//static UINT16 rec1_ind_r;             // receive  buffer reading index
static UINT16 rec1_ind_w;             // receive  buffer writing index

static UINT8 rec1_buf[BUF_MIN+1];       // receive  buffer
static UINT8 send1_command[60],comm1_status;
static UINT16 data_length;

#pragma INTERRUPT/E uart1_tra_int 
void uart1_tra_int(void);
#pragma INTERRUPT/E uart1_rec_int 
void uart1_rec_int(void);
void tra1_com(UINT8* command,UINT8 length);

#define START_FLAG                0x5A
#define END_FLAG                  0xFF

#define CRC_CHECK_INIT            0xFFFF

#define WRITESYSPARAM_RX          0x10
#define WRITESYSPARAM_TX          0x80
#define WRITESTEPPARAM_RX         0x11
#define WRITESTEPPARAM_TX         0x81
#define WRITESTEPCURVE_RX         0x12
#define WRITESTEPCURVE_TX         0x82

#if DMA_UART1_MODE

#define RESET_UART1  do{dmae_dm1con = 1;}while(BUF_MID -1- tcr1) 
void reset_uart1(void)
{
	RESET_UART1;
}
void init_dma1(void)
{
	dm1con = 0x21;				//目标正向，源固定，禁止，0，单次,8bit
	dm1sl = 0x0f;				//触发源uart1接收中断
	sar1 = (UINT32)&u1rb;		//源uart1
	dar1 = (UINT32)rec1_buf; 	//目标数组
	tcr1 = BUF_MID-1;
	dmae_dm1con = 1;
	dm1ic = 0;
	ir_dm1ic = 0;
	s1ric = UART1_RECEIVE_IPL_0; 
}

#endif

void initial_uart1_variable(void)
{
	  UINT16 i;
	  
	  tra1_ind_r = 0; 
	  tra1_ind_w = 0; 

	  rec1_ind_w = 0;  

	  for(i=0;i<BUF_MIN;i++)
	  {
		  tra1_buf[i] = 0;   
	  } 
	  for(i=0;i<BUF_MIN;i++)
	  {      
		  rec1_buf[i] = 0;  
	  } 
	  comm1_status = 0;
	  rec1_datalength = 0;
	  rec1_status_machine = 0;
	  rec1_package_length = 0;
	  wirte_stepermotor_para_flag = 0;
	  write_stepmotor_curve_flag = 0;
}


#if UART1_DEBUG_OUTPUT_MODE
void printf_uart_string(UINT8 far *str)
{
	while( *str!= '@' )
	{
	   uart1_send_char(*str);
	   str++;
	}
}
#endif

void init_uart1(void)
{   
  	u1mr = 0x00; 
  	u1mr = 0x05; 
  	u1c0 = 0x10;
  	u1c1 = 0x04;
  	ucon &= ~0x00;
	
#if UART1_DEBUG_OUTPUT_MODE 

	u1brg = BAUD_RATE_115200 ;
  	re_u1c1 = 1;
  	s1tic = 0;   
  	s1ric = UART1_RECEIVE_IPL_7; 
  	initial_uart1_variable(); 
	te_u1c1 = 1;
	u1tb = '0';
#else	
    u1brg = BAUD_RATE_9600 ;
  	re_u1c1 = 1;
  	s1tic = UART1_TRANSMIT_IPL_7;   
  	s1ric = UART1_RECEIVE_IPL_7; 
  	initial_uart1_variable(); 
	#if DMA_UART1_MODE
	init_dma1();
    #endif 
#endif
}

void set_uart1_debug_mode(void)
{
  	u1mr = 0x00; 
  	u1mr = 0x05; 
  	u1c0 = 0x10;
  	u1c1 = 0x04;
  	ucon &= ~0x00;
  	u1brg = BAUD_RATE_115200 ;
  	re_u1c1 = 1;
  	s1tic = UART1_TRANSMIT_IPL_7;   
  	s1ric = UART1_RECEIVE_IPL_7; 
  	initial_uart1_variable();  
	dmae_dm1con = 0;
}


void uart1_tra_int(void)
{
	#if UART1_DEBUG_OUTPUT_MODE 
	
	#else
	if( formwork_identify_device == 3)
	{
	}
	else
	{
	  	if(tra1_ind_r != tra1_ind_w)
	  	{
	    	u1tb = tra1_buf[tra1_ind_r++];
			uart1_sending_flag = 1;
	  	}
	  	else                             
	  	{
	    	te_u1c1 = 0;
			uart1_sending_flag = 0;
	  	}
	}
}

void uart1_rec_int(void)
{
	if( formwork_identify_device == 3)
	{
	}
	else
	{
		if( rec1_ind_w < BUF_MIN)
	        rec1_buf[rec1_ind_w++] = (UINT8)u1rb; 
		else
		{
			rec1_ind_w = 0;
			rec1_buf[rec1_ind_w++] = (UINT8)u1rb;
		}
	}
}

void tra1_com(UINT8* command,UINT8 length)
{
	INT16 i;
	if(!te_u1c1)
	{
		for(i=1;i<length;i++)
		{
			tra1_buf[tra1_ind_w++] = command[i];
		}
		u1tb = command[0];
		te_u1c1 = 1;
	}    
	else
	{    
		for(i=0;i<length;i++)
		{
			tra1_buf[tra1_ind_w++] = command[i];
		}
	}
	uart1_sending_flag = 1;
}


#define CRC_POLY 0x1021

UINT16 cal_crc16(UINT8 *in, UINT16 len, UINT16 crc)
{
  UINT8 i;
  for (; len>0; --len)
  {
      crc = crc ^ ((UINT16)(*in++) << 8);
      for (i=0; i<8; ++i)
      {
	      if (crc & 0x8000)
	      {
	        crc = (crc << 1) ^ CRC_POLY;
	      }
	      else
	      {
	        crc = crc << 1;
	      }
      }
      crc = crc & 0xFFFF;
  }

  return crc;
}

static INT8 verify_rec(UINT8 * in, UINT16 len)
{
  UINT16 crc_read = ((UINT16)in[len-2] << 8) | (in[len-1] & 0xFF);
  UINT16 crc_cal = cal_crc16(in, len - 2, CRC_CHECK_INIT);

  if (crc_read == crc_cal)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void rec1_com(void)
{
  	UINT16 i;
		if( comm1_status != 0)
		{
			comm1_status = 0;
			init_uart1();
			return;
		}
	    if (serail_config_flag == 0)
		{
			serail_config_flag = 1;
			initial_uart1_variable();
	    }

	#if DMA_UART1_MODE
		rec1_ind_w = BUF_MID - 1 - tcr1;
		if(ir_dm1ic)//有溢出？
		{
			 ir_dm1ic = 0;
			 rec1_ind_w = 0;//BUF_MID;	  
			 //init_dma1();
			 RESET_UART1;
			 return;
		}
	#endif
	
	#if  ENABLE_RFID_FUNCTION	
	if( formwork_identify_device == 3)
	{
		#if SECOND_GENERATION_PLATFORM
		if( (ms_scan_counter >= 150)&&(pattern_change_flag ==0) )
		#else
		if( (ms_scan_counter >= para.left_barcode_position )&&(pattern_change_flag ==0) )
		#endif
		{
			#if DEBUG_RFID_DA1_FUN	
			da1 = 50;
			#endif
			if( RFID_SCAN() )//直接变化serial_number 非0表示识读到了
			{
				;//SUM = 1;
			}
			#if DEBUG_RFID_DA1_FUN	
			da1 = 100;
			#endif
			ms_scan_counter = 0;
		}
	}
	else
	#endif
	{		
			 
		if( (pattern_change_flag == 0) && ( (sys.status == READY)||(sys.status == CHECKI03) ) )
		{
			 if( (rec1_buf[rec1_ind_w-1]==0xA)&&(rec1_buf[rec1_ind_w-2]==0xD))
			 {			
						if (((rec1_buf[rec1_ind_w-7]=='D')&&(rec1_buf[rec1_ind_w-6]=='H'))||((rec1_buf[rec1_ind_w-7]==0x39)&&(rec1_buf[rec1_ind_w-6]==0x37)))
						{
							i= (rec1_buf[rec1_ind_w-5]-0x30)*100+(rec1_buf[rec1_ind_w-4]-0x30)*10+(rec1_buf[rec1_ind_w-3]-0x30);
							while(i>999)
							{
								i-=999;
							}
							serail_number = i;
						}	


						#if DMA_UART1_MODE
						RESET_UART1;
						#else
						rec1_ind_w = 0;
						#endif
			}
			#if DMA_UART1_MODE
			else if( rec1_ind_w == BUF_MID)
			{
				RESET_UART1;
			}
			#endif 
		}
	}
	if( (formwork_identify_device < 2)||(auto_function_flag == 0) )
			     serail_number = 0;

}


#if UART1_DEBUG_OUTPUT_MODE || SUPPORT_UNIFY_DRIVER

	#define VAL2ASC(VAL) ((VAL)>=10?(VAL)+('A'-10):(VAL)+'0')
	void printf_uart(const unsigned char* p,...)
	{
	  int data;
	  int data_5,data_4,data_3,data_2,data_1,data_r;
	  int index; 
	  int *arg = ((int *)&p); // argument
	  arg+=2;
	  index  = 0;
	  while(*p != 0)
	  {
	    switch(*p)
	    {
	      case '%':
	      if(p[1]=='x')
	      {
	         	data = *arg++;
	         	send1_command[index++] = VAL2ASC((data>>12)&0xf);
	         	send1_command[index++] = VAL2ASC((data>>8)&0xf);
	         	send1_command[index++] = VAL2ASC((data>>4)&0xf);
	         	send1_command[index++] = VAL2ASC((data)&0xf);
	         	p+=2;
	      }
	      else if(p[1]=='d')
	      {
	        	data = *arg++;
				if( data <0 )
				{
				    data = -data;
					send1_command[index++] = '-';
				}
		        data_5 = data/10000;
		        data_r = data%10000;
		        data_4 = data_r/1000;
		        data_r = data_r%1000;
		        data_3 = data_r/100;
		        data_r = data_r%100; 
		        data_2 = data_r/10;
		        data_1 = data_r%10;

		        if( data_5 > 0)
		          	send1_command[index++] = VAL2ASC(data_5);
		        if( data_4 >=0)
		          	send1_command[index++] = VAL2ASC(data_4);
		        if( data_3 >= 0)
		          	send1_command[index++] = VAL2ASC(data_3);
		        if( data_2 >=0) 
		        	send1_command[index++] = VAL2ASC(data_2);
				
		        send1_command[index++] = VAL2ASC(data_1);
		        p+=2;

	      }
	      else  if(p[1]=='l')
	      {
	        	data = arg[1];

	        	send1_command[index++] = VAL2ASC((data>>12)&0xf);
	        	send1_command[index++] = VAL2ASC((data>>8)&0xf);
	        	send1_command[index++] = VAL2ASC((data>>4)&0xf);
	        	send1_command[index++] = VAL2ASC((data)&0xf);
            
				data = arg[0];
				send1_command[index++] = VAL2ASC((data>>12)&0xf);
	        	send1_command[index++] = VAL2ASC((data>>8)&0xf);
	        	send1_command[index++] = VAL2ASC((data>>4)&0xf);
	        	send1_command[index++] = VAL2ASC((data)&0xf);
	            arg += 2;
	        	p   += 2;
	      }
		  break;
	      default:
	        	send1_command[index++] = *p;
	             p++;
	      break;
	    }
	  }
	  tra1_com(send1_command,index); 
	  //delay_ms(100);
	}
#endif

#if ENABLE_RFID_FUNCTION
void init_uart1_RC522(void)
{   
  	u1mr = 0x00; 
  	u1mr = 0x05; 
  	u1c0 = 0x10;
  	u1c1 = 0x04;
  	ucon &= ~0x00;
	//u1brg = BAUD_RATE_19200;
	//u1brg = BAUD_RATE_57600;
	u1brg = BAUD_RATE_115200;
  	re_u1c1 = 1;
  	//s1tic = UART1_TRANSMIT_IPL_0;  
	//s1ric = UART1_RECEIVE_IPL_0; 
	s1tic = UART1_TRANSMIT_IPL_7;
  	s1ric = UART1_RECEIVE_IPL_7; 
  	initial_uart1_variable();  
}

void uart1_send_char(UINT8 ch)
{
	ti_u1c1 = 0;
	u1tb = ch;
	te_u1c1 = 1;
	while( !ti_u1c1)
		rec_com();
	ti_u1c1 = 0;
}

UINT8 uart1_get_char(void)
{
	counter_1ms = 0;
	while (ri_u1c1 == 0)
	{
		if( counter_1ms >1000)
		{
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
			if( sys.error == 0)
	      	    sys.error = ERROR_97;
			break;
		}
	  	rec_com();
	}
	ri_u1c1 = 0;  
	return (UINT8)u1rb;
}
/////////////////////////////////////////////////////////////////////
//功  能：读RC632寄存器
//参数说明：Address[IN]:寄存器地址
//返  回：读出的值
//94 14 28 94  --0x14
/////////////////////////////////////////////////////////////////////
UINT8 ReadRawRC(UINT8 Address)
{
 	 UINT8 ucAddr,i=0;
	 ucAddr =  Address | 0x80;//最高位＝1 表示读
	 uart1_send_char(ucAddr);
	 return uart1_get_char();

} 

/////////////////////////////////////////////////////////////////////
//功  能：写RC632寄存器
//参数说明：Address[IN]:寄存器地址
//      value[IN]:写入的值
/////////////////////////////////////////////////////////////////////
void WriteRawRC(UINT8 Address, UINT8 value)
{
   UINT8 ucAddr,i ;   
   ucAddr = Address&0x7F;
   uart1_send_char(ucAddr);
   uart1_send_char(value);
   i = uart1_get_char();
}
#endif

#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
