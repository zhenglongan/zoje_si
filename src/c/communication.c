//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : communication.c
//  Description: Core program to control UART communication and protocol
//  Version    Date     Author    Description
//  0.01     09/08/08   lm        created
//  ...
///--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"           // M16C/62P special function register definitions
#include "..\..\include\typedef.h"          // Data type define
#include "..\..\include\common.h"           // Common constants definition
#include "..\..\include\variables.h"        // External variables declaration
#include "..\..\include\stepmotor.h"        // constants definition
#include "..\..\include\action.h"           // action function
#include "..\..\include\delay.h"            // delay time definition
#include "..\..\include\iic_bus_eeprom.h"
#include "..\..\include\easystep.h"

#if UART1_DEBUG_OUTPUT_MODE
extern void uart1_send_char(UINT8 ch);
#endif
//--------------------------------------------------------------------------------------
//  Internal macro define
//--------------------------------------------------------------------------------------

#define BUF_MID  300
#define BUF_MIN  216
//--------------------------------------------------------------------------------------
//  Global variables define
//--------------------------------------------------------------------------------------
static UINT16 tra_ind_r;             // transmit buffer reading index
static UINT16 tra_ind_w;             // transmit buffer writing index
static UINT16 rec_ind_r;             // receive  buffer reading index
static UINT16 rec_ind_w;             // receive  buffer writing index
static UINT8 tra_buf[BUF_MIN+1];     // transmit buffer  100 byte
UINT8 rec_buf[BUF_MID+1];     		 // receive  buffer  300 byte
UINT8 rec_status;                    // receive status


static UINT8 link_count;             // linked count
INT16 waiting_count;                 // linked count

UINT8 test_value1;                   // receive status
UINT8 test_value2;                   // receive status
UINT8 send_command[BUF_MIN];  		 // transmit buffer  100 byte

//--------------------------------------------------------------------------------------
// 	Internal constants definition
//--------------------------------------------------------------------------------------
#define DATA_START   0x5A
#define DATA_END     0xFF


#define DOWNLOAD_STEP_CURVE			0x39
#define DOWNLOAD_STEP_CURVE_RET 	0x99

#define DOWNLOAD_STEP_CURVE_DONE		0x3A
#define DOWNLOAD_STEP_CURVE_DONE_RET 	0x9A

#define RFID_WRITE 				0x44
#define RFID_WRITE_RET 			0xa4

#define RFID_READ 				0x45
#define RFID_READ_RET 			0xa5

#define TEST_IOOUTPUT    		0x48//多路IO 输出设置
#define TEST_IOOUTPUT_RET  		0xa8

#define READY_GO_SETOUT 		0x4e
#define READY_GO_SETOUT_RET 	0xae

#define CUT_COMMAND            0x61
#define CUT_COMMAND_RET        0xB1

#define AUTO_SELECT_PATTERN     0x66  
#define AUTO_SELECT_PATTERN_RET 0xb6

#define CONNECT      0x80
#define CONNECT_RET  0xE0
#define MISTAKE_RET  0xA0

#define CHANGE       0x81
#define CHANGE_RET   0xE1

#define QUERY        0x82
#define QUERY_RET    0xE2

#define PARA         0x83
#define PARA_RET     0xE3

#define PATTERN      0x84
#define PATTERN_RET  0xE4

#define SHIFT        0x85
#define SHIFT_RET    0xE5

#define STEP         0x86
#define STEP_RET     0xE6

#define VERSION      0x87
#define VERSION_RET  0xE7

#define MACHINE      0x88
#define MACHINE_RET  0xE8

#define COOR         0x89
#define COOR_RET     0xE9

#define INMOVE       0x8A
#define INMOVE_RET   0xEA

#define HIGH         0x8B
#define HIGH_RET     0xEB

#define SMOTOR       0x8C
#define SMOTOR_RET   0xEC

#define SPEED        0x8D
#define SPEED_RET    0xED

#define INPUT        0x8E
#define INPUT_RET    0xEE

#define OUTPUT       0x8F
#define OUTPUT_RET   0xEF



#define XYSENSOR     0x70
#define XYSENSOR_RET 0xD0

#define PCSENSOR     0x71
#define PCSENSOR_RET 0xD1

#define CSENSOR      0x72
#define CSENSOR_RET  0xD2

#define ISENSOR      0x73
#define ISENSOR_RET  0xD3

#define MOTOCWW      0x74
#define MOTOCWW_RET  0xD4

#define STEPMOVE     0x75
#define STEPMOVE_RET 0xD5

#define MOTOSTA      0x76
#define MOTOSTA_RET  0xD6

#define COORCOM      0x77
#define COORCOM_RET  0xD7

#define COORADJUST   			0x78
#define COORADJUST_RET 			0xD8



#define CUR_COORDINATES 		0x79
#define CUR_COORDINATES_RET 	0xD9

#define PREDIT_SHIFT 			0x7A
#define PREDIT_SHIFT_RET 		0xDA

#define STACOOR   				0x7B
#define STACOOR_RET   			0xDB

#define MOTORMECHANICANGLECHECK 0x7C
#define MOTORMECHANICANGLECHECK_RET 0xDC

#define MOTORMECHANICANGLEENTER 0x7D
#define MOTORMECHANICANGLEENTER_RET 0xDD

#define CURRENTSTITCHQUERY     	0x7E
#define CURRENTSTITCHQUERY_RET  0xDE

#define SEWSPEEDADJUST			0x7F
#define SEWSPEEDADJUST_RET		0XDF

#define ELEMENTSHIFT			0x60
#define ELEMENTSHIFT_RET		0xB0

//#define EDITCURRENTCOOR			0x61
//#define EDITCURRENTCOOR_RET		0xB1

#define CANCELALLCOOR			0x62
#define CANCELALLCOOR_RET		0xB2

#define STITCHUP				0x63
#define STITCHUP_RET			0xB3

#define POINTSHIFT				0x64
#define POINTSHIFT_RET			0xB4

#define ENDSTYLECHOOSE			0x65
#define ENDSTYLECHOOSE_RET		0xB5

#define MOVETOCURRENTCOOR		0x66
#define MOVETOCURRENTCOOR_RET	0xB6

#define CURRENTPOINTSHIFT		0x67
#define CURRENTPOINTSHIFT_RET	0xB7

#define EDITELEMENTCOMMAND		0x68
#define EDITELEMENTCOMMAND_RET	0xB8

#define SEWTESTENDFLAGCHECK     0x69
#define SEWTESTENDFLAGCHECK_RET 0xB9

#define CANCELOVERTABPOINT     	0x6A
#define CANCELOVERTABPOINT_RET 	0xBA

#define PATTERNPOINTSHIFT		0x6B
#define PATTERNPOINTSHIFT_RET	0xBB

#define FOOTSTATEQUERY			0x6C
#define FOOTSTATEQUERY_RET		0xBC

#define INIFLAG					0x6D
#define INIFLAG_RET				0xBD

#define ORIGIN					0x6E
#define ORIGIN_RET				0xBE

#define STORE_OFFSET       		0x90
#define STORE_OFFSET_RET   		0xf0

#define DSP_DOWNLOAD_CODE      0x92
#define DSP_DOWNLOAD_CODE_RET  0xF2

#define DSP_DOWNLOAD_VERIFY      0x93
#define DSP_DOWNLOAD_VERIFY_RET  0xF3

#define SET_BASELINE_ALARM      0x94
#define SET_BASELINE_ALARM_RET  0xF4

#define PARA2                   0x95
#define PARA2_RET               0xF5

#define CHECKI11_TEST           0x96
#define CHECKI11_TEST_RET       0xF6

#define BOBBIN_CASE_INPUT       0x97
#define BOBBIN_CASE_INPUT_RET   0xF7

#define DISPLAY_SERVOPARA 		0x98    
#define DISPLAY_SERVOPARA_RET 	0xf8

#define SERVOPARA      			0x99
#define SERVOPARA_RET  			0xf9

#define SYS_PARAM_GROUP			0x9A
#define SYS_PARAM_GROUP_RET		0xFA

#define RESET_USERPARAM			0x9B
#define RESET_USERPARAM_RET		0xFB

#define QUERY_ID      				0x9c
#define QUERY_ID_RET  				0xFc

#define SET_MAIN_CONTROL_LOCK       0x9d
#define SET_MAIN_CONTROL_LOCK_RET   0xFd

//--------------------------------------------------------------------------------------
//  Internal functions and subroutines declaration
//--------------------------------------------------------------------------------------
void init_uart0(void);
void init_comm(void);
#pragma INTERRUPT/E uart_tra_int 
void uart_tra_int(void);
#pragma INTERRUPT/E uart_rec_int 
void uart_rec_int(void);

UINT8 verify(UINT8* a);  
void tra_com(UINT8* command,UINT8 length);
void tra1_com(UINT8* command,UINT8 length);
void rec_com(void);
void protocol(UINT8* command);
void com_error(void);
UINT8 verify_code(UINT8 number);


//--------------------------------------------------------------------------------------
//  Name: init_uart0 routine
//  pars: None
//  Returns: None
//  Description: initial UART0 register
//--------------------------------------------------------------------------------------
void init_uart0(void)
{    
  u0mr = 0x00; // I have no idea why, but this line is required to make the UART
               // respond properly after waking up.  It shouldn't be, because
               // it is set again further down -- strange. 

  u0mr = 0x05; // 00000101 - UART_0 mode register
               // ||||||||
               // |||||+++-- 101 - UART mode, 8 bit transfer unit
               // ||||+----- 0 - Use internal clock
               // |||+------ 0 - One stop select
               // ||+------- Not used
               // |+-------- 0 - Parity disabled
               // +--------- Not used

  u0c0 = 0x10; // 00010000 - UART_0 control register 0
               // ||||||||
               // ||||||++-- 00 - use f1 count source
               // |||||+---- Not used
               // ||||+----- Read only bit
               // |||+------ 1 - CTS/RTS disabled
               // |++------- Nothing is assigned
               // +--------- 0 - transfer LSB first

  u0c1 = 0x04; // 00000100 - UART_0 control register 1
               // ||||||||
               // |||||||+-- 0 - Disabled transmission
               // ||||||+--- Transmit buffer empty flag (Read Only)
               // |||||+---- 1 - Enabled reception
               // ||||+----- Receive complete flag (Read Only)
               // ++++------ Nothing is assigned

  ucon &= ~0x00;// xxxxxxx0 - UART_0/UART_1 control register 2
                // ||||||||
                // |||||||+-- 0 - interrupt cause select: on transmit buffer empty
                // ||||||+--- Not used (applies to UART_1)
                // ||++++---- Not used 
                // |+-------- CTS/RTS shared pin
                // +--------- Nothing is assigned
  
  u0brg = BAUD_RATE_115200;//2014-06-11 modified
               // n = (fx / (16*Baud_rate) ) - 1 
               //   rearranging:
               //   Baud_rate = fx/((n+1)*16)
               //   For 57k6 Baud:
               //   let fx = f1 = 10MHz, now 
               //   n = (10e6 / (16 * 57600)) - 1 = 9.85
               //   so load 10 into u0brg to give an actual Baud 
               //   rate of 56.82k, which is close enough.

  // set ISR priorities
  s0tic = UART_TRANSMIT_IPL;  // UART0 TX: TXR transmit
  s0ric = UART_RECEIVE_IPL;   // UART0 RX: TXR receive
}

//--------------------------------------------------------------------------------------
//  Name: init_comm routine
//  pars: None
//  Returns: None
//  Description: initial communication routine
//--------------------------------------------------------------------------------------
void init_comm(void)
{
  INT16 i;
  
  //variables initial
  rec_status = 0;
  link_count = 0;
  waiting_count = 0;
  
  tra_ind_r = 0; 
  tra_ind_w = 0; 
  rec_ind_r = 0; 
  rec_ind_w = 0;   
  
  
  for(i=0;i<BUF_MIN;i++)
  {
      tra_buf[i] = 0; 
  } 
  for(i=0;i<BUF_MID;i++)
  {      
      rec_buf[i] = 0;
  } 
  init_uart0(); //call initial uart0 function
  #if CODE_BAR
	  init_uart1();
  #endif  
  
}
//--------------------------------------------------------------------------------------
//  Name: uart_tra_int routine
//  pars: None
//  Returns: None
//  Description: UART transmit interrupt routine
//--------------------------------------------------------------------------------------
void uart_tra_int(void)
{
  UINT16 i=300;
  if(tra_ind_r != tra_ind_w)       //transmit buffer no empty
  {
     if( tra_ind_r < BUF_MIN )
	 	 u0tb = tra_buf[tra_ind_r++];
  }
  else                             //transmit buffer empty
  {
	    te_u0c1 = 0;
	    while(i--);					   
	    RDSET = 0;									 
  }
}


//--------------------------------------------------------------------------------------
//  Name: uart_rec_int routine
//  pars: None
//  Returns: None
//  Description: UART receive interrupt routine
//--------------------------------------------------------------------------------------
void uart_rec_int(void)
{
  if( rec_ind_w < BUF_MID)
  	  rec_buf[rec_ind_w++] = (UINT8)u0rb;  
  else
  {
     //SUM =1; 
     rec_buf[rec_ind_w] = (UINT8)u0rb;
  }
}

//--------------------------------------------------------------------------------------
//  Name:  verify
//  pars: *a
//  Returns: UINT8
//  Description: verify receive data
//--------------------------------------------------------------------------------------
UINT8 verify(UINT8* a)
{
	INT16 i;
	UINT8 result;
	
	result = 0xff;
	for(i=0;i<rec_ind_w;i++)
	{
		result ^= a[i];	
	}
	return result;
}
//--------------------------------------------------------------------------------------
//  Name: tra_com routine
//  pars: UINT8* command
//  Returns: None
//  Description: transmit command routine
//--------------------------------------------------------------------------------------
void tra_com(UINT8* command,UINT8 length) // length == transmiting data package length
{
	INT16 i;
	RDSET = 1;	
	if(!te_u0c1)
	{
		for(i=1;i<length;i++)
		{
			if( tra_ind_w < BUF_MIN )
				tra_buf[tra_ind_w++] = command[i];
		}
		u0tb = command[0];
		te_u0c1 = 1;
	}    
	else
	{    
		for(i=0;i<length;i++)
		{
			if( tra_ind_w < BUF_MIN )
				tra_buf[tra_ind_w++] = command[i];
		}
	}
	find_communication_start_flag = 0; 
}


//--------------------------------------------------------------------------------------
//  Name: rec_com routine
//  pars: None
//  Returns: None
//  Description: receive command routine
//--------------------------------------------------------------------------------------
void rec_com(void)
{
  UINT8 data_length;
  
	if(rec_ind_w>=2)                     // data transmitting
	{
		if( find_communication_start_flag ==1)
		{
			data_length=rec_buf[1];
			if(rec_ind_w < data_length+2)      // data has not been transmited
			{
				if(flag_1ms)
				{
					waiting_count++;
					flag_1ms = 0;
				}
				if(waiting_count >= 100)         // if wait time > 100ms 
				{
					waiting_count = 0;
					com_error();                   // return communication error to panel
				}
			}
			else
			{		
				if(rec_ind_w == data_length+2)   // data length is right
				{	
					if(verify(rec_buf) == 0)
					{	
						protocol(rec_buf);           // process penal protocol 
					}
					else
					{
						com_error();                 // return communication error to panel
					}
				}
				else                             // data length too much
				{
					com_error();                   // return communication error to panel
				}
				waiting_count = 0;	                 
			}	
		}
		else
		{
			if( rec_buf[0]!=0x5a )
			{
				tra_ind_r = 0; 
				tra_ind_w = 0; 
			    find_communication_start_flag = 0;
				waiting_count = 0;	
				rec_ind_r = 0; 
				rec_ind_w = 0; 
			}
			else
			{
				data_length=rec_buf[1];
				find_communication_start_flag = 1;
				waiting_count = 0;	
			}
			
		}	
	}	  
}

//--------------------------------------------------------------------------------------
//  Name: protocol routine
//  pars: UINT8* command
//  Returns: None
//  Description: process panel protocol routine
//--------------------------------------------------------------------------------------
void protocol(UINT8* command)
{	
	INT16 temp16;
	INT16 i;
	UINT16 temp;
	UINT8 temp8;
	UINT8 para_length; 
	INT32 temp32;
	UINT32 utemp32;

	if(command[0] == 0x5A && command[rec_ind_w-1] == 0xFF)   // data package is correct
	{	
		switch(command[2])        // process the function code
		{    	
			//--------------------------------------------------------------------------------------      
			//  connect communication
			//--------------------------------------------------------------------------------------
			case CONNECT:             
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = CONNECT_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;                   	                         
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break; 
			case STORE_OFFSET:
				if( rec_buf[3] == 0)
				    pat_buff_write_offset = 0;
				else
				    pat_buff_write_offset = HALF_WRITE_OFFSET;//12000
					
				if(rec_buf[4] == 0)
				   ;
				else
				  ;
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = STORE_OFFSET_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;                   	                         
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;
			//--------------------------------------------------------------------------------------      
			//  system status query
			//--------------------------------------------------------------------------------------              
			case QUERY:  

				tra_ind_r = 0; 
				tra_ind_w = 0; 
				send_command[0] = DATA_START;
			
			#if DEBUG_DLG	
				send_command[1] = 24;
			#elif DEBUG_SPEED_OUT
			    send_command[1] = 24;
			#elif QUERY_NEW_PROTOCOL
				send_command[1] = 150;  //22+32*4 = 150
			#elif DEBUG_POINT_COOR_MODE
				send_command[1] = 19;	
			#else 
				send_command[1] = 12;  
			#endif	 
			 
				send_command[2] = QUERY_RET;
				send_command[3] = sys.status;
				send_command[4] = sys.error;
				send_command[5] = pattern_change_flag;	
				#if DEBUG_RFID_DA1_FUN	
				if( (pattern_change_flag == 1)&&(rfid_debug_flag == 0) )
				{
					 da1 = 200;
					 rfid_debug_flag = 1;
				}
			    #endif
				temp = pattern_number;
				send_command[6] = (UINT8)(temp>>8);
				send_command[7] = (UINT8)temp;		
				
									 
			#if DEBUG_DLG							
				send_command[8] = de_bug.test1;
				send_command[9] = de_bug.test2;
			    send_command[10] = de_bug.test3;
				send_command[11] = de_bug.test4;
				
				temp = trans_x.word;
				send_command[12] = (UINT8)(temp>>8);
				send_command[13] = (UINT8)temp;	
				
				temp = trans_y.word;
				send_command[14] = (UINT8)(temp>>8);
				send_command[15] = (UINT8)temp;	
				
				temp = trans_z.word;
				send_command[16] = (UINT8)(temp>>8);
				send_command[17] = (UINT8)temp;	
				
				temp = recieve_x.word;
				send_command[18] = (UINT8)(temp>>8);
				send_command[19] = (UINT8)temp;	

				temp = recieve_y.word;
				send_command[20] = (UINT8)(temp>>8);
				send_command[21] = (UINT8)temp;	
				
				temp = recieve_z.word;
				send_command[22] = (UINT8)(temp>>8);
				send_command[23] = (UINT8)temp;	
				
				
				send_command[24] = verify_code(24);
				send_command[25] = DATA_END;                   	                         
				tra_com(send_command,26);
				
			#elif DEBUG_SPEED_OUT	
				send_command[8] = 0x12;
				send_command[9] = 0x34;
				for(i=0;i<13;i++)
				{
					send_command[10+i] = ratio_array[i]/100;
				}
				send_command[24] = verify_code(24);
				send_command[25] = DATA_END;                   	                         
				tra_com(send_command,26);
				
				
			#elif QUERY_NEW_PROTOCOL
				//get_func_debug_info();
				send_command[8] = debug_write_point;
				send_command[9] = debug_counter;
			    send_command[10] = 0;
				send_command[11] = 0;
				
				temp32 = monitor_allx_step;
				//temp32 = allx_step;
				send_command[12] = (UINT8)( temp32 >>24);
				send_command[13] = (UINT8)( temp32 >>16);
				send_command[14] = (UINT8)( temp32 >> 8);
				send_command[15] = (UINT8)  temp32;
				
				temp32 = monitor_ally_step;
				//temp32 = ally_step;
				send_command[16] = (UINT8)( temp32 >>24);
				send_command[17] = (UINT8)( temp32 >>16);
				send_command[18] = (UINT8)( temp32 >> 8);
				send_command[19] = (UINT8)  temp32;				
				
				
				if( (over_counter != 0 )&&( monitor_refresh_flag == 1) )
				{
				    temp = (monitor_pat_point - (PATTERN_DATA *)(pat_buf)) + over_counter*8000;					
				}
				else
					temp = (monitor_pat_point - (PATTERN_DATA *)(pat_buf));
				 
				send_command[20] = (UINT8)(temp>>8);
				send_command[21] = (UINT8)temp;	
				
				for(i=0;i<32;i++)
				{
					temp =22 + (i<<2);
					if( debug_write_point > i)
					{
					    send_command[temp] =   sys_de_bug[i].test1;
						send_command[temp+1] = sys_de_bug[i].test2;
						send_command[temp+2] = sys_de_bug[i].test3;
						send_command[temp+3] = sys_de_bug[i].test4;
						#if UART1_DEBUG_OUTPUT_MODE
						uart1_send_char(sys_de_bug[i].test1);
						uart1_send_char(sys_de_bug[i].test2);
						uart1_send_char(sys_de_bug[i].test3);
						uart1_send_char(sys_de_bug[i].test4);
						#endif
						
					}
					else
					{
						send_command[temp] =   0;
						send_command[temp+1] = 0;
						send_command[temp+2] = 0;
						send_command[temp+3] = 0;
					}
				}
				debug_write_point = 0;
				debug_counter = 0;
				
				send_command[150] = verify_code(150);
				send_command[151] = DATA_END;                   	                         
				tra_com(send_command,152);
			#else
				send_command[8] = inpress_position;//de_bug.test1;
				#if LASER_DEBUG_MODE
					send_command[9] =  com_monitor_coder; //0;//de_bug.test2;
				    send_command[10] = x_monitor_steps;//de_bug.fun_ID>>8;
					send_command[11] = y_monitor_steps;//de_bug.fun_ID;
				#elif DEBUG_POINT_COOR_MODE
					/*
					temp32 = allx_step;
					send_command[9] = (UINT8)( temp32 >>24);
					send_command[10] = (UINT8)( temp32 >>16);
					send_command[11] = (UINT8)( temp32 >> 8);
					send_command[12] = (UINT8)  temp32;
					temp32 = ally_step;
					send_command[13] = (UINT8)( temp32 >>24);
					send_command[14] = (UINT8)( temp32 >>16);
					send_command[15] = (UINT8)( temp32 >> 8);
					send_command[16] = (UINT8)  temp32;	
							
					temp = pat_point - (PATTERN_DATA *)(pat_buf);
				 	send_command[17] = (UINT8)(temp>>8);
					send_command[18] = (UINT8)temp;	
					
					*/
					
					send_command[9]  = 0x12;
					send_command[10] = 0x34;
					send_command[11] = monitor_x_step;
					send_command[12] = monitor_tabx_time;					
					send_command[13] = monitor_x_time;
					
					send_command[14] = monitor_y_step;
					send_command[15] = monitor_taby_time;
					send_command[16] = monitor_y_time;	
							
					temp = monitor_point;
				 	send_command[17] = (UINT8)(temp>>8);
					send_command[18] = (UINT8)temp;	
					send_command[19] = verify_code(19);
					send_command[20] = DATA_END;                   	                         
					tra_com(send_command,21);
				#else
					send_command[9] =  de_bug.test2;
				    send_command[10] = (UINT8)(monitor_cutter_angle >>8);//de_bug.fun_ID>>8;
					send_command[11] = (UINT8)monitor_cutter_angle;//de_bug.fun_ID;
					send_command[12] = verify_code(12);
					send_command[13] = DATA_END;                   	                         
					tra_com(send_command,14);
				#endif
				
			#endif   
				com_refresh_flag = 1;
				rec_ind_r = 0; 
				rec_ind_w = 0;
				
				break;   
			//--------------------------------------------------------------------------------------      
			//  system status change
			//--------------------------------------------------------------------------------------              
			case CHANGE:  
				switch(command[3])
				{
					//--------------------------------------------------------------------------------------      
					//  system status change to free
					//--------------------------------------------------------------------------------------  
					case FREE: 
						if(sys.status == ERROR)
						{
							switch(sys.error)
							{
								case 16:
								case 21:
								case 52:		    					                       
									sys.error = OK;	
									sys.status = ERROR;
									StatusChangeLatch = FREE;
									break;
								default:
									break;
							}        	            		                                   
						}
						else
						{
							switch(sys.status)
							{ 
								case NEEDLE:    
									StatusChangeLatch = FREE;
									foot_com = 1;            
									inpress_com = 1;
									sys.status = NEEDLE;
									break;	            		          		      	            		                           
								case INPRESS: 
									StatusChangeLatch = FREE;
									foot_com = 1;           
									inpress_com = 1;
									sys.status = INPRESS;
									break;
								case PREEDIT:
									StatusChangeLatch = FREE;
									inpress_com = 1;
									sys.status = PREEDIT;
									stop_flag = 0;
									origin2_lastmove_flag = 0;
									return_from_preddit = 1;
									inpress_high = inpress_high_hole;
									break;
								case NOEDIT:
									StatusChangeLatch = FREE;
									foot_com = 1;             
									inpress_com = 1;
									sys.status = NOEDIT;
									break;
								case EDIT:
									StatusChangeLatch = FREE;
									foot_com = 1;             
									inpress_com = 1;
									sys.status = EDIT;
									break;
								case CHECKI03:
									StatusChangeLatch = FREE;
									foot_com = 1;             
									inpress_com = 1;
									sys.status = CHECKI03;
									break;
								case CHECKI04:
									StatusChangeLatch = FREE;
									foot_com = 1;             
									inpress_com = 1;
									sys.status = CHECKI04;
									break;
								case CHECKI05:
									StatusChangeLatch = FREE;
									foot_com = 1;             
									inpress_com = 1;
									sys.status = CHECKI05;
									L_AIR = 0;
									FL = 0;
									FL_pwm_action_flag = 0;
									cutter_output_test = 0;
									break; 
								case CHECKI06:
									StatusChangeLatch = FREE;
									foot_com = 1;             
									inpress_com = 1;
									sys.status = CHECKI06;
									break;
								case CHECKI07:
									StatusChangeLatch = FREE;
									inpress_com = 1;
									sys.status = CHECKI07;
									break; 
								case CHECKI08:
									StatusChangeLatch = FREE;
									foot_com = 1;       
									inpress_com = 1;
									sys.status = CHECKI08;
									break;  
								case CHECKI10:
									StatusChangeLatch = FREE;
									foot_com = 1;         
									inpress_com = 1;
									checki10_follow_flag = 0;
									sys.status = CHECKI10;
									#if SECOND_GENERATION_PLATFORM || CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
									L_AIR = 0;
									#else	
									FA = 0;
									#endif
									break; 
								case CHECKI11:
									StatusChangeLatch = FREE;
									foot_com = 1;         
									inpress_com = 1;
									sys.status = CHECKI11;
									for(i=0;i<5;i++)
									   checki11_output_status[i] = 0;
									DRILL_MOTOR_UPDOWN = 0;
									DRILL_FOOTER = 0;									
									drill_motor_run_enable = 0;
									BOBBIN_CASE_ARM_SCRATH = 0;
									BOBBIN_CASE_ARM_OUT = 0;
									release_tension_value_flag = 1;
									#if ENABLE_LASER_CUTTER
									LASET_POWER_SWITCH = 0;
									LASER_POWER_PIN = 0;
									LASER_FUN_PIN = 0;
									LASET_MIRROR_COVER = 0;
									LASER_INDICATIOR_LED = LASER_LED_ON;
									#endif
									break;
								case SLACK:
									StatusChangeLatch = FREE;
									sys.status = SLACK;
									break;
								case PREWIND:
									StatusChangeLatch = FREE;
									foot_com = 1;           
									sys.status = PREWIND;
									
									break;  
								case WIND:
									StatusChangeLatch = FREE;
									foot_com = 1;            
									sys.status = WIND;
									break; 
								case FINISH:
									StatusChangeLatch = FREE;
									foot_com = 1;             
									sys.status = FINISH;
									break; 
								case READY:
									StatusChangeLatch = FREE;
									sys.status = READY;
									break;
                   	            case DOWNLOAD_DSP_CURVE:
									StatusChangeLatch = FREE;
									sys.status = DOWNLOAD_DSP_CURVE;
									break;	
								case DOWNLOAD_DSP1:  
								case DOWNLOAD_DSP2:
								case DOWNLOAD_DSP3:
								case DOWNLOAD_DSP4:  StatusChangeLatch = FREE;  break;	            	                  	            		            		
								default: 	   
									break;        	
							}
						}	
						break;
					//--------------------------------------------------------------------------------------      
					//  system status change to ready
					//--------------------------------------------------------------------------------------  	
					case READY:    
						if(sys.status == ERROR)
						{
							switch(sys.error)
							{
								case 45:
								    bottom_thread_remain = set_default_length;
									if( bobbin_case_alarm_mode == 0)
									    bobbin_case_once_done_flag = 1;
								case 29:
								case 25:
								case 26:
								case 98:
								    turnoff_ledbuz();
									sys.error = OK;	
									StatusChangeLatch = READY;
									sys.status = ERROR; 
									
								break;
								case 17:
								case 92:
									sys.error = OK;	
									StatusChangeLatch = READY;
									sys.status = ERROR; 
								break;
								case 16:	     					                         
									sys.error = OK;	
									StatusChangeLatch = READY;
									sys.status = ERROR; 
									origin_com = 1;
									break;
								case 82:
								case 2:
								case 0:
								case 51:
								case 80:
									aging_com = 0;
									origin_com = 0;	
									sys.error = OK;	
									StatusChangeLatch = READY;
									sys.status = ERROR; 
									break;
								case 22:
									aging_com = 0;
									origin_com = 0;	
									sys.error = OK;	
									StatusChangeLatch = READY;
									sys.status = ERROR;
									break;
								default:
									break;
							}	     	            		                                   
						}
						else
						{	      
							switch(sys.status)
							{
								case MANUAL: 
									StatusChangeLatch = READY;									   
									sys.status =  MANUAL;  	            		          			                 	            		            	             
									break;
   
								case SINGLE:
									StatusChangeLatch = READY;
									origin_com = 0;            // feed do not move
									//sys.status =  MANUAL;      //10.05.28 wr modify
									sys.status =  SINGLE;        //10.05.28 wr modify
									break;
								case SETOUT: 
									StatusChangeLatch = READY;   		          			           
									origin_com = 0;            // feed do not move
									sys.status =  SETOUT;
									break;

								case FREE:  
									StatusChangeLatch = READY;
						
									if( release_poweron_ready  == 0 )
										origin_com = 1;            // feed move
									release_poweron_ready = 0;
									sys.status =  FREE;
									origin_com_after_pattern_done = 0;
									//=============================
			  						if( (sewingcontrol_flag == 2)&&(sewingcontrol_stitchs != 0))
			   							need_backward_sewing = 1; 
									if( sewingcontrol_flag == 1)
				    					need_action_two = 1;
									already_auto_find_start_point = 0;	
									
									
									#if AUTO_CHANGE_PATTERN_FUNCTION 
									
								    AUTO_ALLOW_FEEDARM_RELEASE  = 0; //气阀6 模板到位以后，夹紧压框，通知PLC机械臂离开
									AUTO_NOTIFY_READY_FOR_CHANGE = 0;////辅助气阀 缝纫真正完成了，允许机械臂更换模板
									AUTO_NEED_CHANGE_NOW = 0;
									AUTO_ALLOW_TAKE_OUT_FRAMEWORK = 0;
									AUTO_FIRST_ASK_FRAMEWORK  = 1;//气阀4 当前没有模板，请求送模板过来
  
									#endif
									break;

								case NEEDLE:  
									StatusChangeLatch = READY;
									inpress_com = 1;
									sys.status =  NEEDLE;
									break;           
								case INPRESS:
									StatusChangeLatch = READY;
									inpress_com = 1;
									sys.status = INPRESS;
									break;             
								case SLACK:
									StatusChangeLatch = READY;
									sys.status = SLACK;
									break;                                	            		            	                  	            		            		
								case PREWIND:
									StatusChangeLatch = READY;
									sys.status = PREWIND;
								    origin_com = 0;  
									break; 
								case PREEDIT:
									origin_com_after_pattern_done = 0;
									StatusChangeLatch = READY;
									sys.status = PREEDIT;
									predit_shift = 12;
									inpress_com = 1;
									inpress_high = inpress_high_hole;
									if( return_from_setout ==1)
										origin_com = 1; 
									else
								    	origin_com = 0;  
									break;
									/*
								case DOWNLOAD_DSP_CURVE:
									StatusChangeLatch = READY;
									origin_com = 1;         
									release_poweron_ready = 0;
									origin_com_after_pattern_done = 0;
			  						if( (sewingcontrol_flag == 2)&&(sewingcontrol_stitchs != 0))
			   							need_backward_sewing = 1; 
									if( sewingcontrol_flag == 1)
				    					need_action_two = 1;
									already_auto_find_start_point = 0;	
									break;
									*/
								case RFID_WR:
									StatusChangeLatch = READY;
									sys.status = RFID_WR;
									
									break;               
									
								default: 	   
									break;        	
							}
						                 	            		          	      	            		          	 		                 	            		                  	            		           
						}
						break;      	            	
					//--------------------------------------------------------------------------------------      
					//  system status change to prewind
					//--------------------------------------------------------------------------------------	      	         	      	            		      	            		
					case PREWIND:  
						if(sys.status == ERROR)
						{
                           
						}
						else
						{	
							if(sys.status == WIND)
							{
								StatusChangeLatch = PREWIND;
								motor.stop_angle = 150;   // 53 degree
								motor.spd_obj = 0;                                          
								sys.status = WIND; 
								wind_com = 0;
							}
							else
							{
								if(u205 == 1)
								{
									StatusChangeLatch = PREWIND;
									sys.status = WIND; 
									wind_com = 1;
								}
								else if(u205 == 0)
								{
									sys.status = FREE;
								} 
							}	
						}
						break;      	            	
					//--------------------------------------------------------------------------------------      
					//  system status change to inpress
					//--------------------------------------------------------------------------------------	
					case INPRESS:  
						if(sys.status == ERROR)
						{

						}
						else
						{	
							StatusChangeLatch = INPRESS;     		            	
							sys.status = INPRESS;  
							if( inpress_type == AIR_INPRESS)
							    inpress_com = 0; 
							else
							{
								inpress_high = inpress_high_hole;
							}     
							inpress_act_flag = 1;         
						}
						break;	      	            					
					//--------------------------------------------------------------------------------------      
					//  system status change to single
					//--------------------------------------------------------------------------------------
					case SINGLE:   
						if(sys.status == ERROR)
						{
							if(sys.error == ERROR_15)
							{
								StatusChangeLatch = SINGLE;           		            	
								sys.status = ERROR;    
							} 		                                   
						}
						else
						{
							
							if(sys.status == READY)
							{

								single_comm = 0x00; 
								StatusChangeLatch = SINGLE;       		            	
								sys.status = READY; 
							}
							else if(sys.status == INPRESS) 
							{
								single_comm = 0x00; 
								StatusChangeLatch = SINGLE;        		            	
								sys.status = INPRESS; 
							}
						}
						break;
					//--------------------------------------------------------------------------------------      
					//  system status change to manual
					//--------------------------------------------------------------------------------------	
					case MANUAL:   
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{ 
							if(sys.status == READY)
							{
								if(TestStatusChangeFlag == 0)
								{
									StatusChangeLatch = MANUAL;
									manx_step = allx_step;
									many_step = ally_step;
									sys.status = READY;  
									shift_flag = 0x00; 
									shift_reply = 0x00; 
								}                  
								else if(TestStatusChangeFlag == 1)
								{
									sys.status = READY;
									StatusChangeLatch = READY;
								}
							}
						}
						break;	
					//--------------------------------------------------------------------------------------      
					//  system status change to setout
					//--------------------------------------------------------------------------------------
					case SETOUT: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{ 
							if(sys.status == FINISH)
							{
								StatusChangeLatch = SETOUT;
								sys.status = FINISH;
							}
						}                   
						break;      	            		           
					//--------------------------------------------------------------------------------------      
					//  system status change to emerstop
					//--------------------------------------------------------------------------------------
					case EMERSTOP: 
						if(sys.status == ERROR)
						{
							switch(sys.error)
							{
								case 2:
									StatusChangeLatch = EMERSTOP;
									sys.status = ERROR;	
									sys.error = OK; 
									break;
								case 15:
									OutOfRange_flag = 1;
									sys.status = ERROR;
									StatusChangeLatch = EMERSTOP;
									sys.error = OK;
									emermove_high = inpress_high;
									break;
								default:
									break;
							} 		                                   
						}      	            		                            
						break;	         	            		                                                         
					//--------------------------------------------------------------------------------------      
					//  system status change to preedit
					//--------------------------------------------------------------------------------------
					case PREEDIT: 
						if(sys.status == ERROR)
						{
							switch(sys.error)
							{
								case 15:						// out of sewing range
									StatusChangeLatch = PREEDIT;
									OutOfRange_flag = 0;
									sys.error = OK;
									sys.status = ERROR;	   
									break;
								default:
									break;
							}                                
						}
						else
						{ 
							switch(sys.status)
							{
								case FREE:
								
									StatusChangeLatch = PREEDIT;
									origin_com = 1;
									repeat_com = 0;
				
									commandpoint_com = 0;
									sys.status = FREE;
									if(k110 == 1)
										R_AIR = 0; 
									FootRotateFlag = 0;
									break;
								case NOEDIT:
									StatusChangeLatch = PREEDIT;
									origin_com = 0;
									repeat_com = 0;
									commandpoint_com = 1;
									sys.status = NOEDIT;
									break;
								case EDIT:
									StatusChangeLatch = PREEDIT;
									origin_com = 0;
									repeat_com = 1; 
									sys.status = EDIT; 
									predit_shift = 1;
									break;
								case SLACK:
									StatusChangeLatch = PREEDIT;
									sys.status = SLACK;
									repeat_com = 1;
									break;
								default:
									break;
							}	
						}                 
						break;
					//--------------------------------------------------------------------------------------      
					//  system status change to edit
					//--------------------------------------------------------------------------------------
					case EDIT: 
						if(sys.status == ERROR)
						{
                               
						}
						else
						{ 
							if(sys.status == PREEDIT)
							{
								StatusChangeLatch = EDIT;
								coor_com = 0;
								sys.status = PREEDIT;
							}
						}                   
						break;	 
					//--------------------------------------------------------------------------------------      
					//  system status change to noedit
					//--------------------------------------------------------------------------------------
					case NOEDIT: 
						if(sys.status == ERROR)
						{
                               
						}
						else
						{ 
							switch(sys.status)
							{
								case PREEDIT:
									StatusChangeLatch = NOEDIT;
									sys.status = PREEDIT;
									break;
								default:
									break;
							}


						}                   
						break;	 	         
					//--------------------------------------------------------------------------------------      
					//  system status change to finish
					//--------------------------------------------------------------------------------------
					case FINISH: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{ 
							sys.status = FINISH;
						}                   
						break;	  
					//--------------------------------------------------------------------------------------      
					//  system status change to needle
					//--------------------------------------------------------------------------------------
					case NEEDLE: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{ 
							switch(sys.status)
							{
								case FREE:
									StatusChangeLatch = NEEDLE;
									sys.status = FREE;
									break;
								case READY:
									StatusChangeLatch = NEEDLE;
									sys.status = READY;
									break;
								default:
									break;
							}
						}                   
						break;	   
					//--------------------------------------------------------------------------------------      
					//  system status change to waitoff
					//--------------------------------------------------------------------------------------
					case WAITOFF: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{ 
							if(sys.status == EMERSTOP)
							{
								foot_com = 1;            
							}	
							sys.status = WAITOFF;
							StatusChangeLatch = WAITOFF;
						}                   
						break;	
					//--------------------------------------------------------------------------------------      
					//  system status change to trim
					//--------------------------------------------------------------------------------------
					case TRIM: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{  
							if(sys.status == EMERSTOP)
							{
								StatusChangeLatch = TRIM;
								sys.status = EMERSTOP;
							}	     	            		           	 	      	            		             
						}                   
						break;		       
					//--------------------------------------------------------------------------------------      
					//  system status change to slack
					//--------------------------------------------------------------------------------------
					case SLACK: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{    
							switch(sys.status)
							{  	            		           	 	
								case READY:
									StatusChangeLatch = SLACK;
									sys.status = READY;
									break;
								case FREE:
									StatusChangeLatch = SLACK;
									sys.status = FREE;
									break;
								case PREEDIT:
									StatusChangeLatch = SLACK;
									sys.status = PREEDIT;
									break;
								case DOWNLOAD_DSP_CURVE:
									StatusChangeLatch = SLACK;
									break;
								default:
									break;
							}
						}                   
						break;		 
					//--------------------------------------------------------------------------------------      
					//  system status change to checki03
					//--------------------------------------------------------------------------------------
					case CHECKI03: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{    
							switch(sys.status)
							{  
								case FREE:
									StatusChangeLatch = CHECKI03;
									sys.status = FREE;
									break;
								default:
									break;
							}
						}                   
						break;		
					//--------------------------------------------------------------------------------------      
					//  system status change to checki04
					//--------------------------------------------------------------------------------------
					case CHECKI04: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{    
							switch(sys.status)
							{  
								case FREE:
									StatusChangeLatch = CHECKI04;
									origin_com = 1;            // feed move 	                	            		           	  
									smotor_speed = 0; 
									sys.status = FREE;
									break;
								default:
									break;
							}  
						}                   
						break;		              
					//--------------------------------------------------------------------------------------      
					//  system status change to checki05
					//--------------------------------------------------------------------------------------
					case CHECKI05: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{       	
							switch(sys.status)
							{  
								case FREE:
									StatusChangeLatch = CHECKI05;
									output_com = 0; 
									sys.status = FREE;
									break;
								default:
									break;
							}
						}                   
						break;		 
					//--------------------------------------------------------------------------------------      
					//  system status change to checki06
					//--------------------------------------------------------------------------------------
					case CHECKI06: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{   
							switch(sys.status)
							{  
								case FREE:
									StatusChangeLatch = CHECKI06;
									origin_com = 1;            // feed move 	       	            		           	 	            		             
									shift_flag = 0x00; 
									shift_reply = 0x00; 
									sys.status = FREE;
									break;
								default:
									break;
							}
						}                   
						break;		              
					//--------------------------------------------------------------------------------------      
					//  system status change to checki07
					//--------------------------------------------------------------------------------------
					case CHECKI07: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{ 
							switch(sys.status)
							{  
								case FREE:
									StatusChangeLatch = CHECKI07;
									origin_com = 1;            // feed move 
									stepmotor_comm = 0xff; 
									stepmotor_state = 0xff; 
									sys.status = FREE;
									break;
								default:
									break;
							} 
						}                   
						break;
					//--------------------------------------------------------------------------------------      
					//  system status change to checki08
					//--------------------------------------------------------------------------------------
					case CHECKI08: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{ 
							switch(sys.status)
							{  
								case FREE:
									StatusChangeLatch = CHECKI08;
									origin_com = 1;            // feed move 
									stepmotor_comm = 0xff; 
									stepmotor_state = 0xff; 
									sys.status = FREE;
									break;
								default:
									break;
							} 
						}                   
						break;		           		
					//--------------------------------------------------------------------------------------      
					//  system status change to checki10
					//--------------------------------------------------------------------------------------
					case CHECKI10: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{   
							switch(sys.status)
							{  
								case FREE:
									StatusChangeLatch = CHECKI10;
									//origin_com = 1;            // feed move   
									stepmotor_comm = 0xff; 
									stepmotor_state = 0xff; 
									sys.status = FREE;
									break;
								default:
									break;
							}
						}                   
						break;	
					case CHECKI11: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{   
							switch(sys.status)
							{  
								case FREE:
									StatusChangeLatch = CHECKI11;
									stepmotor_comm = 0xff; 
									stepmotor_state = 0xff; 
									sys.status = FREE;
									cutter_test_cnt = 0;
									break;
								default:
									break;
							}
						}                   
						break;
					//--------------------------------------------------------------------------------------      
					//  system status change to emermove
					//--------------------------------------------------------------------------------------
					case EMERMOVE: 
						if(sys.status == ERROR)
						{
							switch(sys.error)
							{
								case 2:
								case 17:
									sys.status = ERROR;
									StatusChangeLatch = EMERMOVE;
									sys.error = OK;
									emermove_high = inpress_high;
								break;
								case 15:
									OutOfRange_flag = 1;
									sys.status = ERROR;
									StatusChangeLatch = EMERMOVE;
									sys.error = OK;
									emermove_high = inpress_high;
									break;
								default:
									break;
							}      		                                   
						}      	            		                            
						else if ( (sys.status == EMERSTOP)&&(u97==1) )//2010-7-2 manual & not cut
						{
							sys.status = EMERMOVE;  
							StatusChangeLatch = EMERMOVE;   
							emermove_high = IN_ORIGIN;	 
						}      	            		                            
						break;	  
					//--------------------------------------------------------------------------------------      
					//  system status change to boardtest
					//--------------------------------------------------------------------------------------	            
					case BOARDTEST: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{            	            		           	              	            		           	  
							smotor_speed = 0; 		           	 	
							sys.status = BOARDTEST;
						}                   
						break;			           	                                                     	            	                            	           
					case DOWNLOAD:
						sys.status = DOWNLOAD;  
						StatusChangeLatch = DOWNLOAD; 
					break;       	                                                     	            	                            	           
					case DOWNLOAD_DSP1:
					    sys.status = DOWNLOAD_DSP1;
						StatusChangeLatch = DOWNLOAD_DSP1;
						break;
					case DOWNLOAD_DSP2:
					    sys.status = DOWNLOAD_DSP2;
						StatusChangeLatch = DOWNLOAD_DSP2;
						break;
					case DOWNLOAD_DSP3:
					    sys.status = DOWNLOAD_DSP3;
						StatusChangeLatch = DOWNLOAD_DSP3;
						break;
					case DOWNLOAD_DSP4:
					    sys.status = DOWNLOAD_DSP4;
						StatusChangeLatch = DOWNLOAD_DSP4;
						break;
					case DOWNLOAD_DSP_CURVE:
						sys.status = DOWNLOAD_DSP_CURVE;
						StatusChangeLatch = DOWNLOAD_DSP_CURVE;
					break;
					case RFID_WR:
						StatusChangeLatch = RFID_WR;
						pattern_change_flag = 0;
						last_pattern_number = 0;
					break;
					//--------------------------------------------------------------------------------------      
					//  system status change to com_error
					//--------------------------------------------------------------------------------------			      	               	            		                					                      
					default:  
						com_error();       
						//return;			
						break;
				}
				tra_ind_r = 0; 
				tra_ind_w = 0; 
				send_command[0] = DATA_START;
				send_command[1] = 0x05;                 
				send_command[2] = CHANGE_RET;
				send_command[3] = sys.status;
				send_command[4] = sys.error;
				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;                   	                         
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;                                        
				break;  
			//--------------------------------------------------------------------------------------      
			//  receive parameter
			//--------------------------------------------------------------------------------------
			case PARA:    //  parameter                    
				sew_speed = (UINT16)rec_buf[3] * 100; //设定的缝制转速 *25（默认值）
				MotorSpeedRigister = rec_buf[3]; 
				temp = (UINT16)rec_buf[4] << 8;                
				sew_stitch = temp | (UINT16)rec_buf[5]; //花样的落针点针数
				
				u02 = rec_buf[6];  //u02 第一针速度（有抓线）  *8
				u03 = rec_buf[7];  //u03 第二针速度（有抓线）  *15
				u04 = rec_buf[8];  //u04 第三针速度（有抓线）  *20
				u05 = rec_buf[9];  //u05 第四针速度（有抓线）  *25
				u06 = rec_buf[10]; //u06 第五针速度（有抓线）  *27
				u07 = rec_buf[11]; //u07 第一针线张力（有抓线）*200	
				u08 = rec_buf[12]; //u08 切线时线张力设定	   *0
				u09 = rec_buf[13]; //u09 切线时线张力切换时相  *0
				if( u09 < -4 )
					u09 = -4;       
				u10 = rec_buf[14]; //u10 第一针速度（无抓线）  *3
				u11 = rec_buf[15]; //u11 第二针速度（无抓线）  *4
				u12 = rec_buf[16]; //u12 第三针速度（无抓线）  *6   
				u13 = rec_buf[17]; //u13 第四针速度（无抓线）  *12
				u14 = rec_buf[18]; //u14 第五针速度（无抓线）  *18
				u15 = rec_buf[19]; //u15 第一针线张力（无抓线）*0
				u16 = rec_buf[20]; //u16 启缝时的线张力切换时相 *-5
				if(u16 > -1)
					u16 = -1;       
				u26 = rec_buf[21]; //u26 高低段行程时的压脚高度 *70 ，程序中未使用
				u26 = u26-15;
				u33 = rec_buf[22]; //u33 设定抓线的放开针数     *2
				u34 = rec_buf[23]; //u34 抓线器的抓线延时时相	*0
				u35 = rec_buf[24]; //u35 抓线开关控制 0-允许 1*-禁止	
				u36 = rec_buf[25]; //u36 选择送布动作时相    *-4  -8~16
				u37 = rec_buf[26]; //u37 缝制结束后压脚状态 *0-始缝后上升
				u38 = rec_buf[27]; //u38 自动加工完成后压板抬起 *0-压板上升 1-禁止压板上升
				u39 = rec_buf[28]; //u39 缝制结束后是否检索原点（非组合缝） 0-无 *1-有
				u40 = rec_buf[28]; //u40 设定组合缝制时的原点检索 *0-无 1-每图案 2-每循环
				u41 = rec_buf[29]; //u41 中途停止时的压脚状态 *0-自动上升 1-通过压脚开关			
				u42 = rec_buf[30]; //u42 机针停止位置 *0-上位置 1-上死点			
				u46 = rec_buf[31]; //u46 是否允许剪线 *0-允许 1-禁止 
				u48 = rec_buf[32]; //u48 设置起缝点复位路径 *0-直线 1-图案 2-原点检索 
				u49 = rec_buf[33]; //u49 绕线速度设置 *16
				u51 = rec_buf[34]; //u51 拨线器是否打开 0-关闭 *1-打开  0-起缝打开，1-结束打开，2-关
				u68 = rec_buf[35]; //u68 线张力设定时的线张力输出时间 *0
				u69 = rec_buf[36]; //u69 抓线的弯曲位置 *0
				u70 = rec_buf[37]; //u70 抓线位置 *0-标准 1-后方 
				u71 = rec_buf[38]; //u71 断线检测是否有效 *0-无效 1-有效
				u72 = rec_buf[39]; //u72 断线检测时缝制开始的无效针数 *8
				u73 = rec_buf[40]; //u73 断线检测时缝制中途的无效针数 *3
				u81 = rec_buf[41]; //u81 外压脚-脚踏开闭 *0-1段
				u82 = rec_buf[42]; //u82 外压脚控制-中途停止时开闭 *0
				u84 = rec_buf[43]; //u84 踏板SW1是否锁定 0-无 *1-有
				u85 = rec_buf[44]; //u85 踏板SW2是否锁定 0-无 *1-有
				u86 = rec_buf[45]; //u86 踏板SW3是否锁定 0-无 *1-有
				u87 = rec_buf[46]; //u87 踏板SW4是否锁定 0-无 *1-有
				u89 = rec_buf[47]; //u89 移动模式 0-禁止 1-平行 *2-后设第二原点
				u91 = rec_buf[48]; //u91 滚珠轨道补正动作 *0-无 1-有
				u94 = rec_buf[49]; //u94 原点检索时是否选择上死点 *0-无 1-是
				u97 = rec_buf[50]; //u97 暂停时切线方式  0-自动 *1-手动
				u101 = rec_buf[51]; //u101 主马达XY传送同步 *0-2700rpm/3.0mm
				u103 = rec_buf[52]; //u103 中压脚控制 0-无 *1-有（缝制数据） 2-有（前进、后退）
				u104 = rec_buf[53]; //u104 中压脚下降同步 *0-启动之前 1-外压脚同步
				u105 = rec_buf[54]; //u105 中压脚/拨线器挑线位置 *0-中压脚上拨 1-下降位置 2-中压脚下
				u108 = rec_buf[55]; //u108 气动压力检测 *0-无 1-有
				u112 = rec_buf[56]; //u112 中压脚下降位置设定 *35 ,程序中未使用
				u129 = rec_buf[57];	//u129 机针冷却有无 *0-无 1-有
				u245 = rec_buf[58];	//u245 加润滑脂异常	
				
				k01 = rec_buf[59];	//k01 取消范围保护 *0-关闭 1-打开
				k31 = rec_buf[60];	//k31 暂停输入选择 0-无效 *1-有效
				k43 = rec_buf[61];  //k43 切线速度 *24
			#if DOUBLE_X_60MOTOR
				k43 -= 8;
			#endif
				k52 = rec_buf[62];	//k52 电磁拨线器：打开输出时间 *5
				k53 = rec_buf[63];	//k53 电磁拨线器：关闭推迟时间 *8
				k54 = rec_buf[64];	//k54 上死点停止时的拨线输出时相选择 *0-上位置 1-上死点
				k56 = rec_buf[65];	//k56 移动界限+x方向 （小幅面） *1000
				k57 = rec_buf[66];	//k57 移动界限-x方向 *1000
				k58 = rec_buf[67];  //k58 移动界限+Y方向 *0
				k59 = rec_buf[68];  //k59 移动界限-Y方向 *750
				k63 = rec_buf[69];	//k63 针杆停止上下针停止模式选择 0-无效 *1-有效
				k67 = rec_buf[70];	//k67 拨线器输出时的线张力输出 *0-无输出 1-最大输出
				k74 = rec_buf[71];	//k74 电机/气动压脚选择 0-电机 *1-气动压脚
				k75 = rec_buf[72];	//k75 气动压脚下降推迟时间 *10
				k92 = rec_buf[73];	//k92 平时原点检索/原点复位线路选择 *0-标准 1-反转 2-Y-X 3-X-Y 4-只动Y轴
				k93 = rec_buf[74];	//k93 反转时原点检索/原点复位线路选择
				k95 = rec_buf[75];	//k95 正向切线时相 *0
				k98 = rec_buf[76];	//k98 空送布命令:顶点休止时间 *2
				k98 = k98 <<1;
				k100 = rec_buf[77];	//k100 结束命令:停止控制 *0-无 1-有
				k110 = rec_buf[78]; //k110 翻转装置与伸缩压脚控制 *0-无 1-翻转 2-伸缩
				temp = (UINT16)rec_buf[79] << 8;                
				k111 = temp | (UINT16)rec_buf[80];  //k111 翻转装置自动翻转Y坐标 *170
				k131 = rec_buf[81]; //k131 暂停出错时有无抬压脚 *0-无 1-有
				k150 = rec_buf[82]; //k150 *0
				tension = rec_buf[83]; //  线张力基准值
				tension_hole  = rec_buf[83];
			
				inpress_high_hole  = rec_buf[84]; //中压脚高度基准值
				temp16 = (INT16)rec_buf[85]<<8;
				x_bios = temp16 | (INT16)rec_buf[86]; //X轴起缝点偏移
				x_bios = x_bios<<1;                               
				temp16 = (INT16)rec_buf[87]<<8;
				y_bios = temp16 | (INT16)rec_buf[88]; //Y轴起缝点偏移
				y_bios = y_bios<<1;                               
				clamp_com = rec_buf[89];    //抓线临时使能       *0
				aging_flag = rec_buf[90];   //老化功能标志       *0
				aging_delay = rec_buf[91];  //老化时间间隔       *2
				u233 = rec_buf[92];         //老化时是否检测原点 *0-无
				u234 = rec_buf[93];         //老化时压脚动作次数 *0
				k60 = rec_buf[94];          //k60 三级踏板使能 *0-无效 1-有效
				k61 = rec_buf[95];          //k61 主轴电机停车角度 *53
				temp16 = (INT16)rec_buf[96]<<8;
				AdjustAngleSet = temp16 | (INT16)rec_buf[97]; //主轴电机校正角度
				if(AdjustAngleSet > 1024 | AdjustAngleSet < 0)
					AdjustAngleSet = 0;
				else
					AdjustAngle = AdjustAngleSet;  
				
				x_step_current_level = rec_buf[98];       //X轴步进电机电流档位
				y_step_current_level = rec_buf[99];       //Y轴步进电机电流档位
				inpress_step_current_level = rec_buf[100];//中压脚电机电流档位   
				
				//inpress_step_current_level = inpress_step_current_level<<1;
				
				foot_step_current_level = rec_buf[101];   //外压脚电机电流档位     
	
					
				for(i=0;i<10;i++)
					motor_para[i]= rec_buf[102+i];        //主轴电机的控制参数
				trim_origin_pos = rec_buf[112];           //k42 切线找原点位置微调 *0
				k02 = rec_buf[113];       //k02 缝纫机类型选择 0-1510 *1-3020 2-2213
				k03 = rec_buf[114];       //k03 夹线器类型选择 *0-机械 1-电子
				k06 = rec_buf[115];       //k06 物料类型选择 *0-薄 1-中 2-厚
				k112 = rec_buf[116];      //k112 伸缩压脚伸出延时 *25
				k113 = rec_buf[117];      //k113 伸缩压脚抬起延时 *0
				k114 = rec_buf[118];      //k114 伸缩压脚下降延时 *0
				k07 = rec_buf[119];       //k07  物料厚度设置 *0
				
				para_length = rec_buf[1];
				
				if(para_length > 120)
		        {
				   cut_control_flag = rec_buf[120]; //k139 剪线模式 *0
				}
				if(para_length > 121)
				{
				    k115 = rec_buf[121];    //k115 导轨方式 0-普通导轨  1--型材导轨
				}
                if(u103 == 0)  
                {
                	inpress_high = 0;                               
                	inpress_high_hole = 0;					  
				}
                if(u35 == 1) 
                {
                	clamp_com = 0; 
                }
				else
				    clamp_com = 1;
				
				temp16 = (INT16)rec_buf[122]<<8;
				k56 = temp16|(INT16)rec_buf[123];//k56 移动界限+x方向 *1000
				temp16 = (INT16)rec_buf[124]<<8;
				k57 = temp16|(INT16)rec_buf[125];//k57 移动界限-x方向 *1000
				temp16 = (INT16)rec_buf[126]<<8;
				k58 = temp16|(INT16)rec_buf[127];//k58 移动界限+Y方向 *0
				temp16 = (INT16)rec_buf[128]<<8;
				k59 = temp16|(INT16)rec_buf[129];//k59 移动界限-Y方向 *750
				
				#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER18
					k02 = 1;
				#else
				if( (k58 > 500)||(k59 >500)) 
				    k02 = 1;
				else
				    k02 = 0;
				#endif
				temp16 = (INT16)rec_buf[130]<<8;
				pen_x_bios_offset = temp16|(INT16)rec_buf[131]; //k122 画笔X轴偏移 *0
				pen_x_bios_offset *= 20;
				temp16 = (INT16)rec_buf[132]<<8;
				pen_y_bios_offset = temp16|(INT16)rec_buf[133]; //k123 画笔Y轴偏移 *0
				pen_y_bios_offset *= 20;
				
				marking_speed = rec_buf[134];		//k124 画笔移动速度 *1
				x_motor_dir = rec_buf[135];			//k127 X轴电机转向 0-正 *1-反
				y_motor_dir = rec_buf[136];			//k128 Y轴电机转向 *0-正 1-反
				
				auto_function_flag = rec_buf[137];	//k125 模板识别使能开关 0-关闭 *1-打开
				led_light_adjust = rec_buf[138];  	//k130 照明灯亮度调节 *50
		#if 1				
				if(led_light_adjust == 0)
				{
				    da1 = 0;
				}
			
				if( led_light_adjust >0 && led_light_adjust <= 100)
				{
				    da1 = 150 + led_light_adjust/5;
				}
			
        #endif				
				formwork_identify_device = rec_buf[139];//k129 模板识别设备 *0-5路传感器 2-条码 1--8路传感器
				go_origin_speed	 = rec_buf[140];	    //k08  回原点速度 *2			 
				if (go_origin_speed < 6)
				    go_origin_speed = 6;				  
				go_setout_speed  = rec_buf[141]; 	    //k09 回起缝点速度 *2
				nop_move_speed	 = rec_buf[142];	    //k10 空送速度 *2					
				temp8 = rec_buf[143];                   //k11 打版移框速度 *2
				one_step_delay = temp8;
				if(one_step_delay > 3)
				  one_step_delay = 3;										   
				go_original_flag = rec_buf[144];        //k12 上电是否找原点 *0-否 1-是
				release_tension_current = rec_buf[145]; //k13 松线电磁铁打开电流 *0
				if( release_tension_current == 0)
				{
				    release_tension_current = 100;
					//release_tension_time = 10;
				}
				if(para_length > 147)
				{
					sewingcontrol_flag = rec_buf[146];         //k18 缝制起始针加固方式设置 0-不加固 1-第一针加固 *2-在前几针加固 3-曲折缝加固
					sewingcontrol_stitchs = (INT8)rec_buf[147];//k19 起缝加固针 *-3
					sewingcontrol_tail_flag = rec_buf[148];    //k20 结束针加固方式设置 0-不加固 1-结束一针 *2-结束2针 3--结束3针 4-结束4针
				}
				
				k04 = rec_buf[149]; 
				k05 = (INT8)rec_buf[150];
				
			    xy_move_mode = k04 ;//0-同步动框 1-分时动框	
		        if( k02 == 0)
				    xy_move_mode = 0;  
				//u35 = 1;
				//k02 = 1;				
				//clamp_com = 0; 
				
				if( para_length >=152)
				    super_pattern_flag = rec_buf[151];         //u203 是否支持大针数花样 0-否 *1-是
				if( para_length >=154)
				{
					temp16 = (INT16)rec_buf[152]<<8;
					cool_air_close_time = temp16|(INT16)rec_buf[153];//u132 注油间隔时间 *90
					temp16 = (INT16)rec_buf[154]<<8;			
					cool_air_open_time = temp16|(INT16)rec_buf[155]; //u133 注油工作时间 *1000
				}
				if( para_length >=156)
				{
					check_bottom_thread_switch = rec_buf[158];	      //底线提前报警开关
					temp16 = (INT16)rec_buf[156]<<8;
					bottom_thread_remain = temp16|(INT16)rec_buf[157];//底线剩余长度或针数
					
					if( bottom_thread_alarm_type == 0)
						bottom_thread_remain = bottom_thread_remain*2000;  //0.1mm  5000*2000=0x989680
					temp16 = (INT16)rec_buf[159]<<8;
					set_default_length = temp16|(INT16)rec_buf[160];  //底线报警的默认设定值
					if( bottom_thread_alarm_type == 0)//distance
						set_default_length = set_default_length*2000;
					
					open_or_close_loop = rec_buf[161];           //k132 电机工作方式 *0-闭环 1-开环
					alarm_thread_length = rec_buf[162];               
					alarm_thread_length = alarm_thread_length*20;//k133 底线报警长度 *100    
					bottom_thread_alarm_type = rec_buf[163];     //k134 底线报警方式 0-长度 *1-针数
					thread_break_detect_level = rec_buf[164];    //k135 断线传感器触发方式 0-低电平 *1-高电平
					if( thread_break_detect_level != 0)
					    thread_break_detect_level = 1;
				}
				if( para_length >=166)
				{
					power_on_ready = rec_buf[165]; 				//k137 开机是否直接进入可缝制状态 0-否 1-是
					second_start_switch =rec_buf[166];          //k138 二次启动设置 0-关闭 *1-打开 2--自启动 3--一键启动
					if( second_start_switch == 2)
						autosewing_control_flag = 1;
					else
						autosewing_control_flag = 0;
					if( second_start_switch == 3 )
						one_step_run_flag = 1;
					else
						one_step_run_flag = 0;
					second_start_counter = 1;
				}
				if( para_length >=167)
				{
					alarm_output_enable = rec_buf[167];			//k141 报错信号输出 *0-不输出 1-输出
					enable_stop_in_nopmove = rec_buf[168];      //k140 空送中急停响应设置 0-关闭 *1-打开
					enable_thread_broken_alarm = rec_buf[169];  //u206 断线检测报警是否自动跳过 *0-无 1-是
				}
				if( para_length >=170)
				{
					inpress_port = rec_buf[170];                //k142  0-X21 1,2,4  1-x23 1,2,3
									
					ct_holding_steps = rec_buf[171];            //k143
				}
				if( para_length >=173)
				{
					x_compensation = rec_buf[172];              //K144 X轴步距角补偿
				}
				if( para_length >=174)
				{
					x_sensor_pos = rec_buf[173];
				}
				/*******************************/
				if( para_length >= 182 )
				{
				    k21 = (INT8)rec_buf[174];//k1_X轴起缝动框微调
					k22 = (INT8)rec_buf[175];//k1_Y轴起缝动框微调
					k23 = (INT8)rec_buf[176];//k1_X轴动框微调
					k24 = (INT8)rec_buf[177];//k1_Y轴动框微调
					k25 = (INT8)rec_buf[178];//k0_起缝动框微调
					k26 = (INT8)rec_buf[179];//k0_动框微调
					k27 = (INT8)rec_buf[180];//X轴动框时间
					k28 = (INT8)rec_buf[181];//Y轴动框时间 
				}
				else
				{
					if(k04 == 1)
					{
						k21 = k05;
						k22 = 0;  
						k23 = k05;
					    k24 = u36;
					}
					else if(k04 == 0)
					{
					    k25 = k05;
					    k26 = u36;
					}	
				}
				if( para_length >= 183 )
				{
					front2stitchs_tension_off = rec_buf[182]; //K29 起针夹线器打开
				}
				if( para_length >= 186 )
				{
					y_compensation = rec_buf[183];//K145 Y轴步距角补偿
					inpress_type = rec_buf[184];  //K146 中压脚类型 0-电机1 1-气动 2-电机2
					y_gear_compensation = rec_buf[185];  //K147 Y轴齿轮间隙补偿
				}
				if( para_length >= 187 )
				    Corner_deceleration_speed = rec_buf[186];//K148 拐点降速速度设置
					
				if( para_length >= 188 )
				    release_tension_before_nopmove = rec_buf[187];
					
				if( para_length >= 189 )
				{
				    identify_mode = rec_buf[188];//u208 模板识别时间点 0-不受压板状态影响  1-在压板落下后进行识别
					x_gear_compensation = rec_buf[189];          //K149
					x_gear_compensation = x_gear_compensation*2;
				}
				if( para_length >= 196 )
				{
					/*********************************************
					 * 吹气功能使能及吹气时间
					 *********************************************/
					k165 = rec_buf[190];      //吹气功能开关
					blow_air_enable = k165;
					k166 = rec_buf[191];      //吹气时间
				
					k167 = rec_buf[192];      //断线后辅助压脚动作 0-辅助压脚抬起* 1-辅助压脚落下
					k168 = rec_buf[193];      //起缝加固速度选择   0-第一针速度*   1-前5针针速度
					k169 = rec_buf[194];      //机型选择  0-普通机型* 1-8060机型 2-旋转切刀与换梭机型 3-自动机型
											  //          4-10070机型 5-直线切刀
					k170 = rec_buf[195];      //X原点传感器类型：A(0)-老传感器；B(1)-新传感器，安装于台面
					k171 = rec_buf[196];      //CZ137功能选择：0-记号笔；1-吹气

					if( k169 == MACHINE_MID)
					{
						ZJ_8060 = DISABLE_FUN;
						X_AXIS_ORGIN_ENABLE = DISABLE_FUN;       //X轴找原点
						AUTO_SEARCH_START_POINT = ENABLE_FUN;
						SEND_SERVO_PARA = ENABLE_FUN;           //下发调试参数
						ROTATED_CUTTER = DISABLE_FUN;            //旋转切刀
						SUPPORT_CS3_FUN = DISABLE_FUN;           //支持扩展SC074A
						ENABLE_BOBBIN_CASE_FUN = DISABLE_FUN;    //换梭功能

						if( y_motor_dir == 0)
						    y_motor_dir = 1;
						else 
						    y_motor_dir = 0;
			        }
				    else if(k169 == MACHINE_BOBBIN_CUTTER)
					{
						ZJ_8060 = DISABLE_FUN;
						X_AXIS_ORGIN_ENABLE = DISABLE_FUN;       //X轴找原点
						AUTO_SEARCH_START_POINT = ENABLE_FUN;
						SEND_SERVO_PARA = DISABLE_FUN;           //下发调试参数
						ROTATED_CUTTER = ENABLE_FUN;            //旋转切刀
						SUPPORT_CS3_FUN = ENABLE_FUN;           //支持扩展SC074A
						ENABLE_BOBBIN_CASE_FUN = ENABLE_FUN;    //换梭功能

					}
			 		else if( k169 == MACHINE_AUTO_LOCATE)	
					{
						ZJ_8060 = DISABLE_FUN;
						X_AXIS_ORGIN_ENABLE = DISABLE_FUN;       //X轴找原点
						AUTO_SEARCH_START_POINT = ENABLE_FUN;
						SEND_SERVO_PARA = ENABLE_FUN;           //下发调试参数
						ROTATED_CUTTER = ENABLE_FUN;            //旋转切刀
						SUPPORT_CS3_FUN = ENABLE_FUN;           //支持扩展SC074A
						ENABLE_BOBBIN_CASE_FUN = ENABLE_FUN;    //换梭功能

					}
					else if (k169 == MACHINE_SPEPPER_CUTTER)
					{
						ZJ_8060 = DISABLE_FUN;
						X_AXIS_ORGIN_ENABLE = DISABLE_FUN;       //X轴找原点
						AUTO_SEARCH_START_POINT = ENABLE_FUN;
						SEND_SERVO_PARA = ENABLE_FUN;            //下发调试参数
						ROTATED_CUTTER = DISABLE_FUN;            //旋转切刀
						SUPPORT_CS3_FUN = DISABLE_FUN;           //不支持扩展SC074A
						ENABLE_BOBBIN_CASE_FUN = DISABLE_FUN;    //换梭功能

						if( y_motor_dir == 0)
						    y_motor_dir = 1;
						else 
						    y_motor_dir = 0;
					}
					else if ( (k169 == MACHINE_900_NEW )||(k169 == MACHINE_800_NORMAL) )
					{
						ZJ_8060 = DISABLE_FUN;
						X_AXIS_ORGIN_ENABLE = DISABLE_FUN;      //X轴不找原点
						AUTO_SEARCH_START_POINT = ENABLE_FUN;   //
						SEND_SERVO_PARA = ENABLE_FUN;           //下发调试参数
						ROTATED_CUTTER = DISABLE_FUN;            
						SUPPORT_CS3_FUN = DISABLE_FUN;
						ENABLE_BOBBIN_CASE_FUN = DISABLE_FUN; 
						
					#if CURRENT_MACHINE == MACHINE_900_BOBBIN
						SUPPORT_CS3_FUN = ENABLE_FUN;
						ENABLE_BOBBIN_CASE_FUN = ENABLE_FUN;
					#endif	
					
					#if CURRENT_MACHINE == MACHINE_900_CUTTER
						SUPPORT_CS3_FUN = ENABLE_FUN;
						ROTATED_CUTTER  = ENABLE_FUN;
					#endif
					
					#if CURRENT_MACHINE == MACHINE_900_SPEPPER_CUTTER
						SUPPORT_CS3_FUN = DISABLE_FUN;
						ROTATED_CUTTER  = DISABLE_FUN;
					#endif
					
					#if CURRENT_MACHINE == MACHINE_900_FIFTH_BOBBIN
						SUPPORT_CS3_FUN = DISABLE_FUN;
						ROTATED_CUTTER  = DISABLE_FUN;
						ENABLE_BOBBIN_CASE_FUN = ENABLE_FUN;    //换梭功能
					#endif
					#if CURRENT_MACHINE == MACHINE_900_FIFTH_CUTTER
						SUPPORT_CS3_FUN = DISABLE_FUN;
						ROTATED_CUTTER  = ENABLE_FUN;//旋转切刀
						ENABLE_BOBBIN_CASE_FUN = DISABLE_FUN;
					#endif
				         
					
						if( y_motor_dir == 0)
						    y_motor_dir = 1;
						else 
						    y_motor_dir = 0;
						if ( k169 == MACHINE_900_NEW )	
						{
							if( x_motor_dir == 0)
							    x_motor_dir = 1;
							else 
							    x_motor_dir = 0;
						}
						#if CURRENT_MACHINE == MACHINE_800_NORMAL
						if( x_motor_dir == 0)
						    x_motor_dir= 1;
		                else
						 	x_motor_dir= 0;	
						#endif
						
						#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER35
						if( x_motor_dir == 0)
						    x_motor_dir= 1;
		                else
						 	x_motor_dir= 0;	
						#endif

					}
					else if ( k169 == 7 )//6037机型
					{
						ZJ_8060 = DISABLE_FUN;
						X_AXIS_ORGIN_ENABLE = DISABLE_FUN;      //X轴不找原点
						AUTO_SEARCH_START_POINT = ENABLE_FUN;//ENABLE_FUN;   //
						SEND_SERVO_PARA = ENABLE_FUN;           //下发调试参数
						ROTATED_CUTTER = DISABLE_FUN;            
						SUPPORT_CS3_FUN = DISABLE_FUN;
						ENABLE_BOBBIN_CASE_FUN = DISABLE_FUN; 
						if( y_motor_dir == 0)
						    y_motor_dir = 1;
						else 
						    y_motor_dir = 0;
					}
					else //if( k169 == MACHINE_NORMAL)
					{
						ZJ_8060 = DISABLE_FUN;
						X_AXIS_ORGIN_ENABLE = ENABLE_FUN;       //X轴找原点
						AUTO_SEARCH_START_POINT = DISABLE_FUN;
						SEND_SERVO_PARA = DISABLE_FUN;           //下发调试参数
						ROTATED_CUTTER = DISABLE_FUN;            //旋转切刀
						SUPPORT_CS3_FUN = DISABLE_FUN;           //支持扩展SC074A
						ENABLE_BOBBIN_CASE_FUN = DISABLE_FUN;    //换梭功能

					}
				}
				if(para_length >= 197)
				{
					start_sewcontrol_mode = rec_buf[197];       //起缝原地加固针数0-2 k172
				}
				if(para_length >= 200)
				{
					inpress_follow_range_pc = rec_buf[198];                        //随动高度  K173
					temp = (UINT16)rec_buf[199] << 8;                
					inpress_follow_down_angle_pc = temp | (UINT16)rec_buf[200];    //K174 中压脚下降起始角度
					temp = (UINT16)rec_buf[201] << 8;                
					inpress_follow_down_speed_pc = temp | (UINT16)rec_buf[202];//K175 中压脚下降动作时间
					temp = (UINT16)rec_buf[203] << 8;                
					inpress_follow_up_angle_pc = temp | (UINT16)rec_buf[204];  	//K176 中压脚上升起始角度
					temp = (UINT16)rec_buf[205] << 8;                
					inpress_follow_up_speed_pc = temp | (UINT16)rec_buf[206];  //K177 中压脚上升动作时间
				}
				
				if(para_length >= 210)
				{
					ct_bump_workingtime = rec_buf[207]; //k178
					if( ct_bump_workingtime > 60)
						ct_bump_workingtime = 60;
					ct_bump_workingtime = ct_bump_workingtime * 1000;
					last1_speed = rec_buf[208];//k179
					last2_speed = rec_buf[209];//k180
					last3_speed = rec_buf[210];//k181
					last4_speed = rec_buf[211];//k182
					cutter_syn_delay = rec_buf[212];//k183
				}
				
				if(para_length >= 215)
				{
					temp16 = (INT16)rec_buf[213]<<8;//K189---x轴原点偏移
					x_origin_offset = temp16|(INT16)rec_buf[214];
					x_origin_offset = x_origin_offset * 20;
					debug_dsp_flag = rec_buf[215];// k190; 0-dsp1 1-dsp2
				}
				DEADPOINT = 10;
				if( para_length >= 216)
				{
					second_point_passby_flag = rec_buf[216];//k32 0-直接到起缝点 1-按空送路线走
					DEADPOINT = rec_buf[217];
				}
				if( para_length >= 218)
				{
					pause_active_level = rec_buf[218];//K193 急停开关极性 0-常闭OFF  1-常开ON
					if( pause_active_level != 1)
						pause_active_level = 0;
				}
				if( para_length >= 219)
				{
					enable_footer_up = rec_buf[219];//K194 急停后是否允许压框抬起 0-禁止  1-允许
					bar_coder_refresh_enable = rec_buf[220];//k195 启动前扫码确认后才允许缝制  0-关闭  1-打开
					oil_empty_alarm_enable = rec_buf[221];//k199 油量报警使能开关  0 -关闭  1 - 打开
				}
				remain_stitchs = sew_stitch - 150;
				#if CHANGE_DOUBLE_FRAMEWORK
				if( para_length >= 222)
				{
					temp = (UINT16)rec_buf[222] << 8;                
					para_left_barcode_position = temp | (UINT16)rec_buf[223];//k201 X扫码位置补偿左
					
					temp = (UINT16)rec_buf[224] << 8;                
					para_right_barcode_position = temp | (UINT16)rec_buf[225];//k202 X扫描码位置补偿右
					
					temp = (UINT16)rec_buf[226] << 8;                
					para_x_take_offset_left = temp | (UINT16)rec_buf[227];//k203 X接料位置左
					
					temp = (UINT16)rec_buf[228] << 8;                
					para_x_take_offset_right = temp | (UINT16)rec_buf[229];//k204 X接料位置右
				
				    temp = (UINT16)rec_buf[230] << 8;                
					para_y_backward_dis = temp | (UINT16)rec_buf[231];//k205 Y接料位置补偿
					
					temp = (UINT16)rec_buf[232] << 8;                
					para_catch_delay_time = temp | (UINT16)rec_buf[233];//k206 抓取时间延时
					
				}
				#else
					temp = (UINT16)rec_buf[224] << 8;                
					para_right_barcode_position = temp | (UINT16)rec_buf[225];//k202 X扫描码位置补偿右
					temp = (UINT16)rec_buf[226] << 8;                
					para_x_take_offset_left = temp | (UINT16)rec_buf[227];//k203 X接料位置左
					temp = (UINT16)rec_buf[228] << 8;                
					para_x_take_offset_right = temp | (UINT16)rec_buf[229];//k204 X接料位置右
				    temp = (UINT16)rec_buf[230] << 8;                
					para_y_backward_dis = temp | (UINT16)rec_buf[231];//k205 Y接料位置补偿
					temp = (UINT16)rec_buf[232] << 8;                
					para_catch_delay_time = temp | (UINT16)rec_buf[233];//k206 抓取时间延时
				#endif
				
				delay_before_inpress_down = rec_buf[112];//K42 切线找原点位置
				delay_before_inpress_down *= 10;
							
				
				if(para_length > 235)
				{
					z_motor_dir = rec_buf[234]; //K207					
					inpress_not_working_stitchs = rec_buf[235];//K208
					inpress_follow_mode = rec_buf[236];//K209
				}
				else
				{
					inpress_not_working_stitchs = 1;
					inpress_follow_mode = k114;
				}
				#if SUPPORT_UNIFY_DRIVER//MainChildVersion == MACHINE_CONFIG_NUMBER15 || MainChildVersion == MACHINE_CONFIG_NUMBER20 || MainChildVersion == MACHINE_CONFIG_NUMBER22
#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER23 || COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER20||COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER21 || COMPILE_MACHINE_TYPE ==MACHINE_CONFIG_NUMBER17 || COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER29
						z_motor_dir = (z_motor_dir==0)?1:0;
#endif						
#if  COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER26 ||COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER27||COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER28 
					   z_motor_dir = (z_motor_dir==0)?1:0;	
#endif

				   #if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER22
				 	   z_motor_dir = (z_motor_dir==0)?1:0;	
					   x_motor_dir = (x_motor_dir==0)?1:0;	
				   #endif
					x_motor_dir = (x_motor_dir==0)?1:0;	
					y_motor_dir = (y_motor_dir==0)?1:0;						 
				#endif
				
				#if  COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER42||COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER40|| COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39 || COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER37
				z_motor_dir = (z_motor_dir==0)?1:0;
				#endif
				#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER38
				x_motor_dir = (x_motor_dir==0)?1:0;	
				#endif
				#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER41
				x_motor_dir = (x_motor_dir==0)?1:0;	
				y_motor_dir = (y_motor_dir==0)?1:0;
				#endif
				#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER36
				x_motor_dir = (x_motor_dir==0)?1:0;	
				#endif
				
				#if ENABLE_CONFIG_PARA
				if( para.x_motor_dir == 55)
					x_motor_dir = (x_motor_dir==0)?1:0;	
				if( para.y_motor_dir == 55)
					y_motor_dir = (y_motor_dir==0)?1:0;
				if( para.zx_motor_dir == 55)
					z_motor_dir = (z_motor_dir==0)?1:0;
				#endif
				
				
				/******************************/
				if(connect_flag == 0)                           // first connect with panel
				{
					if(para.start_sew_change_inpress_high_range > 13)
					{
						
						para.start_sew_change_inpress_high_range = 13;
					}
				 	connect_flag = 1;
				}
				
				#if FOLLOW_INPRESS_FUN_ENABLE
				//参数发送后，根据K137(inpress_follow_range_pc)和k209(inpress_follow_mode)计算出的
				//中压脚随动范围inpress_follow_range来限制起缝数针降低中压脚随动范围策略中的
				//随动范围 的取值范围
				if(para.start_sew_change_inpress_high_enable == 0x55)
				{
					calculate_inpress_angle(200);//参数实际上是随意设置的，我们使用此函数只为更新inpress_follow_range
					if(para.start_sew_change_inpress_high_range > inpress_follow_range)
					{
						//取值太大了，直接关闭此功能
						para.start_sew_change_inpress_high_range = 0;
						para.start_sew_change_inpress_high_enable = 0;
					}
				}
				#endif
					
				//--------------------------------------------------------------------------------------      
				// compart symbol  
				//--------------------------------------------------------------------------------------	                 		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = PARA_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;  
				    
			case PARA2:
				rotated_cutter_enable = rec_buf[3]; 		//k150 旋转切刀使能
				bobbin_case_enable = rec_buf[4];			//k151 自动换梭使能
				temp = (UINT16)rec_buf[5] << 8;                
				rotated_cutter_running_delay = temp | (UINT16)rec_buf[6]; 
				temp = (UINT16)rec_buf[7] << 8;                
				rotated_cutter_up_delay = temp | (UINT16)rec_buf[8];//k153
			    rotated_cutter_speed = rec_buf[9];
				
			
				drill_motor_pwm = rotated_cutter_speed;
			
				rotated_cutter_current_level = rec_buf[10];
				bobbin_case_arm_offset = rec_buf[11];
				bobbin_case_platform_offset = rec_buf[12];
				temp = (UINT16)rec_buf[13] << 8;                
				bobbin_case_inout_delay = temp | (UINT16)rec_buf[14];
				temp = (UINT16)rec_buf[15] << 8;                
				bobbin_case_scrath_delay = temp | (UINT16)rec_buf[16];
				bobbin_case_current_level = rec_buf[17];
			    
				bobbin_case_stop_position = rec_buf[18]; //k161 换梭停止位置 0-梭盘侧 1-机头侧
				bobbin_case_alarm_mode = rec_buf[19];	//k162
				bobbin_case_restart_mode = rec_buf[20];
				bobbin_case_workmode = rec_buf[21];
				
				temp16 = (INT16)rec_buf[22] << 8;
				cutter_motor_initial_angle = temp16 |(INT16) rec_buf[23];//K184	
							
				//if( cutter_motor_initial_angle >359)
				//    cutter_motor_initial_angle = 359;
					
				stepper_cutter_enable = rec_buf[24];//k185 直线切刀使能
				if( stepper_cutter_shake_rage < 25)
					stepper_cutter_shake_rage = 25;
				temp = (UINT16)rec_buf[25] << 8;
				stepper_cutter_position = temp | rec_buf[26];//K186直线切刀下降高度
				stepper_cutter_shake_rage = rec_buf[27]; //k187 直线切刀摆动幅度
				stepper_cutter_delay = rec_buf[28]; //k188 直线切刀同步延时 
				stepper_cutter_origin_offset = (INT8)rec_buf[29];//K191直线切刀原点补偿	
				
				para_length = rec_buf[1];
				
				if( para_length >= 30)
					bobbin_plateform_org_offset = (INT8)(rec_buf[30]);
				if( para_length >= 31)
				{
					laser_power_on_enable = rec_buf[31];			//k196 激光使能开关
					temp16 = (INT16)rec_buf[32] << 8;
					laser_offset_x = temp16 | (INT16)rec_buf[33];  //k197激光切刀X偏移
					laser_offset_x *= 2;
					temp16 = (INT16)rec_buf[34] << 8;
					laser_offset_y = temp16 | (INT16)rec_buf[35];  //k198激光切刀X偏移
					laser_offset_y *= 2;
					
					x_bios_offset = laser_offset_x;
					y_bios_offset = laser_offset_y;
				}
					
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = PARA2_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;          
			//--------------------------------------------------------------------------------------      
			//  receive pattern data
			//--------------------------------------------------------------------------------------
			case PATTERN:    
				recpat_point = (UINT8 *)&(rec_buf+4);   // data address  
				#if SECOND_GENERATION_PLATFORM
				if(rec_buf[3] > 50)   
				{
					com_error();
					return;
				}
				#else
				if(rec_buf[3] > 100)   
				{
					com_error();
					return;
				}
				#endif
				temp = 250*(rec_buf[3]-1);
		        if( super_pattern_flag == 0)
				{    
					for(i=0;i<rec_buf[1]-4;i++)
					{
						pat_buf[temp+i] = *recpat_point;
						recpat_point++;
					}
				}
				else
				{
					for(i=0;i<rec_buf[1]-4;i++)
					{
						pat_buf[pat_buff_write_offset+temp+i] = *recpat_point;
						recpat_point++;    		            	
					}   
				}
				if( return_from_setout == 0)
                    origin_com_after_pattern_done = 1;
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = PATTERN_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break; 
			//--------------------------------------------------------------------------------------      
			//  receive single step command
			//--------------------------------------------------------------------------------------
			case STEP:                
				if(sys.status == PREEDIT)
				{
					switch(rec_buf[3])                     
					{
						case 0x01:
						    if( predit_shift == 0)
							{
						       predit_shift = 2;
							   //single_edit_next();     
							   single_next();     
							}
							break;
						case 0x81: 
						    if( predit_shift == 0)
							{
							    predit_shift = 2;
							    //single_edit_back();     
								single_back();     
							}
							break;    		            							
						case 0x31: 
						    predit_shift = 2; 
							back_edit_startpoint(); 
							break; 
					
						case 0x02:
						    predit_shift = 2;
							single_end();
							break;
						case 0x82:
						if( predit_shift == 0)
							{
							   predit_shift = 2;
							   single_start();
							   single_reply = 0x82;      
							}
							break;							
						case 0x52:
							predit_shift = 2;
							single_stop();
							break;							
							    		            											
						default:   
							com_error();      
							//return;	
							break;
					}
				}
				else if(sys.status == EDIT)	
				{
					switch(rec_buf[3])                     
					{
						case 0x01: 
						    predit_shift = 2; 
							//single_edit_next();   
							single_next();   
							break;
						case 0x81: 
						    predit_shift = 2; 
							//single_edit_back();   
							single_back();    
							break;    		            							  		            								    		            											
						default:  
							break;	
					}    
				}       		         
				else if(sys.status == ERROR)
				{
					
				}
				else
				{
					single_comm = rec_buf[3]; 
					if( single_comm != 0x10)
					{
						single_comm = rec_buf[3]; 
					}
					switch(rec_buf[3])                
					{
						case 0x01: 
						    predit_shift = 2;
							single_next();     
							break;
						case 0x03: 
						    predit_shift = 2;
							single_inpresser_next();     
							break;
						case 0x05: 
						    predit_shift = 2;
							single_thread_next();     
							break;
						case 0x81: 
						    predit_shift = 2;
							single_back();     
							
							break; 
						case 0x83:
						    predit_shift = 2;
							single_inpresser_back();     
							break; 
						case 0x85: 
						    predit_shift = 2;
							single_thread_back();     
							break;  		            							
						case 0x31: 
						    predit_shift = 2;
							back_startpoint(); 
							break;    		            								
						case 0x02: 
						    predit_shift = 2;
							single_end();      
							break;  
						case 0x04: 
						    predit_shift = 2;
							single_inpresser_end();      
							break;
						case 0x06: 
						    predit_shift = 2;
							single_thread_end();      
							break;  		            								
						case 0x82: 
					     	predit_shift = 2;
							single_start();   
							break;
						case 0x84: 
						    predit_shift = 2;
							single_inpresser_start();    
							break; 
						case 0x86: 
						    predit_shift = 2;
							single_thread_start();    
							break;   		            								
						case 0x52: 
						    predit_shift = 2;
							single_stop();     
							break;    
						case 0x10:  
						   if( (single_flag==6) ||(single_flag == 7) )
						       single_reply = 0x2;
						   else
						       single_reply = 0x51;
						    break;		            											
						default:   
							com_error();      
							break;//return;	
					}           
				}
				if( (foot_flag == 1)||(single_reply == 0x51) )
				     predit_shift = 0;
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = STEP_RET;
				send_command[3] = single_reply;
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				
				break;     
			//--------------------------------------------------------------------------------------      
			//  receive manual shift step command
			//--------------------------------------------------------------------------------------
			case SHIFT:          		             		                		            
				switch(rec_buf[3])                      // data package
				{
					case 0x0C: 
						shift_flag = 0x0C; 
						shift_reply = 0x0C; 
						break;
					case 0x01: 
						shift_flag = 0x01; 
						shift_reply = 0x01; 
						break;    		            							
					case 0x03: 
						shift_flag = 0x03; 
						shift_reply = 0x03; 
						break;  
					case 0x04: 
						shift_flag = 0x04; 
						shift_reply = 0x04; 
						break;
					case 0x06: 
						shift_flag = 0x06; 
						shift_reply = 0x06; 
						break;    		            							
					case 0x07: 
						shift_flag = 0x07; 
						shift_reply = 0x07; 
						break;
					case 0x09: 
						shift_flag = 0x09; 
						shift_reply = 0x09; 
						break;
					case 0x0A: 
						shift_flag = 0x0A; 
						shift_reply = 0x0A; 
						break;    		            									  		            								
					case 0xEC: 
						shift_flag = 0xEC; 
						shift_reply = 0xEC; 
						break;
					case 0xE1: 
						shift_flag = 0xE1; 
						shift_reply = 0xE1; 
						break;    		            							
					case 0xE3: 
						shift_flag = 0xE3; 
						shift_reply = 0xE3; 
						break;  
					case 0xE4: 
						shift_flag = 0xE4; 
						shift_reply = 0xE4; 
						break;
					case 0xE6: 
						shift_flag = 0xE6; 
						shift_reply = 0xE6; 
						break;    		            							
					case 0xE7: 
						shift_flag = 0xE7; 
						shift_reply = 0xE7; 
						break;
					case 0xE9: 
						shift_flag = 0xE9; 
						shift_reply = 0xE9; 
						break;
					case 0xEA: 
						shift_flag = 0xEA; 
						shift_reply = 0xEA; 
						break;  	
					case 0x88: 
						shift_flag = 0x88; 
						shift_reply = 0x88; 
						break;     		            		 	
					case 0x5C: 
						shift_flag = 0x5C; 
						shift_reply = 0x5C; 
						break;
					case 0x51: 
						shift_flag = 0x51; 
						shift_reply = 0x51; 
						break;    		            							
					case 0x53: 
						shift_flag = 0x53; 
						shift_reply = 0x53; 
						break;  
					case 0x54: 
						shift_flag = 0x54; 
						shift_reply = 0x54; 
						break;
					case 0x56: 
						shift_flag = 0x56; 
						shift_reply = 0x56; 
						break;    		            							
					case 0x57: 
						shift_flag = 0x57; 
						shift_reply = 0x57; 
						break;
					case 0x59: 
						shift_flag = 0x59; 
						shift_reply = 0x59; 
						break;
					case 0x5A: 
						shift_flag = 0x5A; 
						shift_reply = 0x5A; 
						break;  	    		            		    		            		
					default:   
						break;
				}	                                     		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = SHIFT_RET;
				send_command[3] = shift_reply;
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;  
			//--------------------------------------------------------------------------------------      
			//  receive query version information command
			//--------------------------------------------------------------------------------------
			case VERSION:  
				version_check(); //new
				tra_ind_r = 0; 
				tra_ind_w = 0; 
				                         		            
				send_command[0] = DATA_START;
			#if NEW_VERSION_CONTROL
				#if MULTIPULE_IO_ENABLE == 1
				send_command[1] = 37;
				#else
			    send_command[1] = 30;
				#endif
			#else
				send_command[1] = 0x12;
			#endif
				send_command[2] = VERSION_RET;

				send_command[3] = Step1Version.MachineType;
				
				#if SUPPORT_UNIFY_DRIVER
				send_command[4] = crc1>>8;
				send_command[5] = crc1;
				#else
				send_command[4] = Step1Version.FatherVersion;
				send_command[5] = Step1Version.ChildVersion;
				#endif
                send_command[6] = stepversion1>>8;   
				send_command[7] = stepversion1;
				
				send_command[8] = Step2Version.MachineType;
				if(SUPPORT_CS3_FUN == 1)
				{
					send_command[9] = stepversion3>>8;   
					send_command[10] = stepversion3;
				}
				else
				{
					#if SUPPORT_UNIFY_DRIVER
					send_command[9] = crc2>>8;
					send_command[10] = crc2;
					#else
					send_command[9] = Step2Version.FatherVersion;
					send_command[10] = Step2Version.ChildVersion;
					#endif
				}
				
	            send_command[11] = stepversion2>>8;   
				send_command[12] = stepversion2;
				send_command[13] = MACHINE_TYPE;
				send_command[14] = MainFatherVersion;
				send_command[15] = MainChildVersion;
				send_command[16] = MainSVNVersion>>8;
				send_command[17] = MainSVNVersion;
				
			#if NEW_VERSION_CONTROL
			    send_command[18] = 0x55;
				
				#if FIFTH_SC013K_PLATFORM
					send_command[19] = PLATFORM_MASC; 
				#elif SECOND_GENERATION_PLATFORM
					send_command[19] = PLATFORM_TASC; 
				#else
					send_command[19] = PLATFORM_ASC;      /* 硬件平台 */
				#endif
				send_command[20] = PATTERN_MACHINE;       /* 机种 */
				send_command[21] = JUKI_STYLE;            /* 原型机种类 */
			#if CURRENT_MACHINE == MACHINE_800_NORMAL     /* 机型分支1 */
				send_command[22] = ENGINE_TYPE0;     
				#if SUPPORT_UNIFY_DRIVER
					#if SECOND_GENERATION_PLATFORM
					send_command[23] = ' ';
					#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER37 || COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER42
					send_command[23] = '1';					//4001
					#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER35 || COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER36 || COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER38
					send_command[23] = ' ';
					#else
				    send_command[23] = 'C';					//400C
					#endif
				#elif FOLLOW_INPRESS_FUN_ENABLE
					send_command[23] = 'D'; 
				#else    
					send_command[23] = 'B';        			 /* 机型分支2 */
				#endif
			#elif (CURRENT_MACHINE == MACHINE_SECOND_GENERATION)
				send_command[22] = ENGINE_TYPE0; 
				send_command[23] = 'E';    
				send_command[24] = ENGINE_DEFAULT;
			#elif CURRENT_MACHINE ==  MACHINE_800_BALLSCREW
				send_command[22] = ENGINE_TYPE0;          
				send_command[23] = '2'; 
				send_command[24] = 'C';
			#elif CURRENT_MACHINE ==  MACHINE_800_STEPPER_CUTTER
				send_command[22] = ENGINE_TYPE0;          
				send_command[23] = '3'; 
				send_command[24] = 'C';	
				
			#elif (CURRENT_MACHINE == MACHINE_900_NEW)//405C
				send_command[22] = ENGINE_TYPE5;   				      
				send_command[23] = 'C';       
				send_command[24] = ENGINE_DEFAULT; 
			#elif CURRENT_MACHINE == MACHINE_900_FIFTH_BOBBIN
				send_command[22] = ENGINE_TYPE5;   				      
				send_command[23] = 'C';       
				send_command[24] = ENGINE_DEFAULT; 
			#elif (CURRENT_MACHINE ==MACHINE_900_FIFTH_CUTTER)
				send_command[22] = ENGINE_TYPE5;          
				send_command[23] = '2';                  //4052C
				send_command[24] = 'C';
			#elif (CURRENT_MACHINE ==MACHINE_900_BOBBIN)
				send_command[22] = ENGINE_TYPE5;          
				send_command[23] = '1';                  //4051C
				send_command[24] = 'C';
			#elif (CURRENT_MACHINE ==MACHINE_900_CUTTER)
				send_command[22] = ENGINE_TYPE5;          
				send_command[23] = '2';                  //4052C
				send_command[24] = 'C';
			#elif (CURRENT_MACHINE ==MACHINE_900_SPEPPER_CUTTER)
				send_command[22] = ENGINE_TYPE5;          
				send_command[23] = '3';  				 //4053C
				send_command[24] = 'C';
			#elif (CURRENT_MACHINE ==MACHINE_800_LASER_CUTTER)
				send_command[22] = ENGINE_TYPE0;          
				send_command[23] = 'C'; 
			#elif (CURRENT_MACHINE == MACHINE_BOBBIN_CUTTER)//4054C
				send_command[22] = ENGINE_TYPE5;          
				send_command[23] = '4'; 
				send_command[24] = 'C';
			#elif (CURRENT_MACHINE == MACHINE_900_LASER_CUTTER2)||(CURRENT_MACHINE == MACHINE_900_LASER_CUTTER)
				send_command[22] = ENGINE_TYPE5;          
				send_command[23] = '5'; 	
				send_command[24] = ENGINE_DEFAULT;
			#elif (CURRENT_MACHINE ==MACHINE_900_CLAMP)
				send_command[22] = ENGINE_TYPE5;          
				send_command[23] = '6'; 
				send_command[24] = ENGINE_DEFAULT;
			#elif CURRENT_MACHINE ==  MACHINE_900_BALLSCREW
				send_command[22] = ENGINE_TYPE5;          
				send_command[23] = '7'; 
				send_command[24] = 'C';
			#else	
				send_command[22] = ENGINE_TYPE0;          
				send_command[23] = 'A';        			  /* 机型分支2 */
				send_command[24] = ENGINE_DEFAULT;
			#endif	
				//send_command[24] = ENGINE_DEFAULT;        /* 机型分支3 */

				send_command[25] = MAIN_MOTOR_1635;       /* 主轴电机类型 */
				send_command[26] = XY_FRAMES_CLOSE_400;   /* 框架类型 */
				send_command[27] = EXPAND_RESERVE;        /* 外设扩展预留 */  
				send_command[28] = SOFTWARE_PLATFORM_SC;  /* 软件平台 */ 
				
				#if MULTIPULE_IO_ENABLE	== 1	 	
					send_command[29] = crc3>>8;
					send_command[30] = crc3;				
	                send_command[31] = stepversion3>>8;   
					send_command[32] = stepversion3;
					
					send_command[33] = 4;
					send_command[34] = crc3;				
	                send_command[35] = stepversion4>>8;   
					send_command[36] = stepversion4;
							
					send_command[37] = verify_code(37);
	  				send_command[38] = DATA_END;  	                  
	                tra_com(send_command,39);
				#else
				    send_command[29] = 0;
				    send_command[30] = verify_code(30);
	  				send_command[31] = DATA_END;  	                  
	                tra_com(send_command,32);
				#endif
	#else	
		send_command[18] = verify_code(18);
		send_command[19] = DATA_END;  
		tra_com(send_command,20);    
	#endif
							
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;     
			//--------------------------------------------------------------------------------------      
			//  receive query machine type information command
			//--------------------------------------------------------------------------------------
			case MACHINE:          		             		                		                                               		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x07;
				send_command[2] = MACHINE_RET;
		        temp = FACTORY_TYPE;
				send_command[3] = (UINT8)(temp >> 8);
				send_command[4] = (UINT8)temp;
				temp = DAHAO_TYPE;
				send_command[5] = (UINT8)(temp >> 8);
				send_command[6] = (UINT8)temp;
				send_command[7] = verify_code(7);
				send_command[8] = DATA_END; 
				tra_com(send_command,9);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;          
			//--------------------------------------------------------------------------------------      
			//  receive query x and y coordinate command
			//--------------------------------------------------------------------------------------
			case COOR:          		             		                		                                               		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 11;
				send_command[2] = COOR_RET;
				if( sys.status == MANUAL || sys.status == RUN)
				{
				    temp32 = -allx_step;
				}
				else
				{
					temp32 = allx_step;
				}
				send_command[7] = (UINT8)( temp32 >>24);
				send_command[8] = (UINT8)( temp32 >>16);
				send_command[3] = (UINT8)( temp32 >> 8);
				send_command[4] = (UINT8)  temp32;
				if(sys.status == MANUAL || sys.status == RUN)
				{
					temp32 = -ally_step;
				}
				else
				{
					temp32 = ally_step;
				}
				send_command[9]  = (UINT8)(temp32 >>24);
				send_command[10] = (UINT8)(temp32 >>16);
				send_command[5]  = (UINT8)(temp32 >> 8);
				send_command[6]  = (UINT8) temp32;  
							  				          	          
				send_command[11] = verify_code(11);
				send_command[12] = DATA_END;  	                
				tra_com(send_command,13);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;       
			//--------------------------------------------------------------------------------------      
			//  receive move inpresser command
			//--------------------------------------------------------------------------------------
			case INMOVE:   
				if(sys.status == EMERMOVE)
				{	
					emermove_high = rec_buf[3];   
				}
				else
				{
					if(sys.status !=0x14)
					   inpress_high = rec_buf[3];   
				}	
				predit_shift = 3; //ok
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = INMOVE_RET;  				           				          
				send_command[3] = inpress_high;  				           				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				inpress_act_flag = 1;
				break;     

			//--------------------------------------------------------------------------------------
			// receive current stitch query command
			//--------------------------------------------------------------------------------------
			case CURRENTSTITCHQUERY:
				tra_ind_r = 0; 
				tra_ind_w = 0;    
				if(super_pattern_flag == 1) 
				    PatternShiftValue = pat_buff_total_counter;
				else
					PatternShiftValue = pat_point - (PATTERN_DATA *)(pat_buf);  
				send_command[0] = DATA_START;  
				send_command[1] = 0x07;   
				send_command[2] = CURRENTSTITCHQUERY_RET;
				send_command[3] = PatternShiftValue>>8;
				send_command[4] = PatternShiftValue;
				send_command[5] = SewTestStitchCounter>>8;
				send_command[6] = SewTestStitchCounter;
				send_command[7] = verify_code(7);
				send_command[8] = DATA_END;
				tra_com(send_command,9);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;
			case FOOTSTATEQUERY:
				tra_ind_r = 0; 
				tra_ind_w = 0;                        		            
				send_command[0] = DATA_START;  
				send_command[1] = 0x04;   
				send_command[2] = FOOTSTATEQUERY_RET;
				send_command[3] = foot_flag;
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;
			case INIFLAG:
				tra_ind_r = 0; 
				tra_ind_w = 0;                        		            
				send_command[0] = DATA_START;  
				send_command[1] = 0x04;   
				send_command[2] = INIFLAG_RET;
				send_command[3] = IniFlag;
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;
			//--------------------------------------------------------------------------------------
			// receive speed adjust command in sewing process
			//--------------------------------------------------------------------------------------
			case SEWSPEEDADJUST:
				MotorSpeedRigister = rec_buf[3];     		                        		                                               		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = SEWSPEEDADJUST_RET;  				           				          
				//  			send_command[3] = MotorSpeedRigister;  				           				          
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break; 
			//--------------------------------------------------------------------------------------      
			//  receive move inpresser command
			//--------------------------------------------------------------------------------------
			case HIGH:       		                      		                                               		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = HIGH_RET;  				           				          
				send_command[3] = inpress_position;  				           				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;   
			//--------------------------------------------------------------------------------------      
			//  receive servo motor speed self test command
			//--------------------------------------------------------------------------------------
			case SMOTOR:       		            	                         		          
				smotor_speed = rec_buf[3];     		                        		                                               		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = SMOTOR_RET;  				           				          
				send_command[3] = smotor_speed;  				           				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;                 
			//--------------------------------------------------------------------------------------      
			//  receive servo motor speed query command
			//--------------------------------------------------------------------------------------
			case SPEED:       	
				temp8 = (motor.spd+50) / 100;	   //2010-5-26            		  
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = SPEED_RET;  				           				          
				send_command[3] = temp8;  				           				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;                
			//--------------------------------------------------------------------------------------      
			//  receive input query command
			//--------------------------------------------------------------------------------------
			/*
			输入信号对应关系：
			启动开关：          send_command[13] DVA
			压脚开关：          send_command[14] DVB
			暂停开关：          send_command[12] PAUSE
			断线检测：          send_command[11] TH_BRK
			X原点传感器：       send_command[3]  XORG
			Y原点传感器：       send_command[4]  YORG
			外压脚原点传感器：  send_command[5]  PORG
			外压脚电机传感器：  send_command[6]  PSENS
			抓线原点传感器：    send_command[7]  CSENS
			抓线传感器：        send_command[8]  CORG
			中压脚传感器：      send_command[9]  IORG
			机头翻传感器：      send_command[10] SFSW
			三联踏板检测：      send_command[15] DVSM
			气阀检测1           send_command[16] PORG*
			气阀检测2           send_command[17] CORG*
			气阀检测3           send_command[18] CSENS*
			气阀检测4           send_command[19] PSENS*
			气阀检测5           send_command[20] ADTCSM
			气阀检测6           send_command[21] SENSOR6
			气阀检测7           send_command[22] SENSOR7
			气阀检测8           send_command[23] SENSOR8			
			*/
			case INPUT:       	    
	            	               		                        		                                               		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 28;
				send_command[2] = INPUT_RET;  
				
				//--------------------------------------------------------------------------------------      
				//  XORG
				//--------------------------------------------------------------------------------------
				if(XORG == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[3] = temp8;  				
				//--------------------------------------------------------------------------------------      
				//  YORG
				//--------------------------------------------------------------------------------------  
				if(YORG == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[4] = temp8;     
				//--------------------------------------------------------------------------------------      
				//  PORG
				//--------------------------------------------------------------------------------------  
				if(PORG == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[5] = temp8; 
				#if DEBUG_CS3_INPUT
					send_command[16] = dsp3_input_value &0x01;
				#else
					send_command[16] = temp8;// 1
				#endif
				//--------------------------------------------------------------------------------------      
				//  PSENS
				//--------------------------------------------------------------------------------------  
				if(PSENS == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[6] = temp8; 
				#if DEBUG_CS3_INPUT
				if( dsp3_input_value &0x08 )
					send_command[19] = 1;
                else
					send_command[19] = 0;					
				#else
				if(formwork_identify_device == 1) //8路识别
				   send_command[19] = temp8;// 2
				else 
				   send_command[17] = temp8;// 2
				#endif
				//--------------------------------------------------------------------------------------      
				//  CORG
				//--------------------------------------------------------------------------------------  
				if(CSENS == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				#if DEBUG_CS3_INPUT
			    if( dsp3_input_value &0x04 )
					send_command[18] = 1;
				else
					send_command[18] = 0;
				#else
					send_command[18] = temp8;// 3
				#endif
				//--------------------------------------------------------------------------------------      
				//  CSENS
				//--------------------------------------------------------------------------------------  
				if(CORG == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}	
				#if DEBUG_CS3_INPUT
				if( dsp3_input_value &0x02 )
					send_command[17] = 1;
				else
					send_command[17] = 0;
				#else			           				          
				if(formwork_identify_device == 1) //8路识别
					send_command[17] = temp8;  
				else
				    send_command[19] = temp8; 
			    #endif					
				if(P_TSENS == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[7] = temp8; 
				if(P_TORG)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[8] = temp8; 
				//--------------------------------------------------------------------------------------      
				//  IORG
				//--------------------------------------------------------------------------------------  
				if(IORG == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[9] = temp8; 
				//--------------------------------------------------------------------------------------      
				//  SFSW
				//--------------------------------------------------------------------------------------  
				if(SFSW == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[10] = temp8; 
				//--------------------------------------------------------------------------------------      
				//  TH_BRK
				//--------------------------------------------------------------------------------------  
				if(TH_BRK == ON)
				{
					temp8 = ON;
				}	
				else
				{
					temp8 = OFF;
				}				           				          
				send_command[11] = temp8; 
				//--------------------------------------------------------------------------------------      
				//  PAUSE
				//--------------------------------------------------------------------------------------  
				if(PAUSE == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[12] = temp8;  
				//--------------------------------------------------------------------------------------      
				//  DVA
				//--------------------------------------------------------------------------------------  
				if(DVA == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[13] = temp8; 
				//--------------------------------------------------------------------------------------      
				//  DVB
				//--------------------------------------------------------------------------------------  
				if(DVB == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}		
				send_command[14] = temp8;
				//--------------------------------------------------------------------------------------      
				//  DVSM
				//--------------------------------------------------------------------------------------  
				if(DVSM == ON)   //2012-9-13 add
				{
					temp8 = OFF; //ON;
				}	
				else
				{
					temp8 = ON;  //OFF;
				}			           				          
				send_command[15] = temp8;
				if(ADTCSM) 
				{
					temp8 = OFF; //ON;
				}	
				else
				{
					temp8 = ON;  //OFF;
				}
				#if DEBUG_CS3_INPUT
				if(dsp3_input_value &0x10 )
					send_command[20] = 1;
				else
					send_command[20] = 0;
				#else
					send_command[20] = temp8;// 5 
				#endif	
					                				          
				if(SENSOR6 == 1)
			    {
					temp8 = OFF; //ON;
				}	
				else
				{
					temp8 = ON;  //OFF;
				}
				#if DEBUG_CS3_INPUT
				if( dsp3_input_value &0x20 )
					send_command[21] = 1;
				else
					send_command[21] = 0;
				#else
					send_command[21] = temp8;//6
				#endif
				
				if(SENSOR7 == 1)
				{
					temp8 = OFF; //ON;
				}	
				else
				{
					temp8 = ON;  //OFF;
				}
				send_command[22] = temp8; //7
			  	if(SENSOR8 )
				{
					temp8 = OFF; //ON;
				}	
				else
				{
					temp8 = ON;  //OFF;
				}
	 		    send_command[23] = temp8;	    
				//if( SUPPORT_CS3_FUN == 1)            				          
				//{
				//	send_command[24] = (UINT8)(dsp3_input_value >> 8);
				//	send_command[25] = (UINT8)dsp3_input_value;	
				//}
				//else
				//{
					send_command[24] = (UINT8)(pattern_number >> 8);
					send_command[25] = (UINT8)pattern_number;	
					pattern_change_flag = 0;

				//}
				#if AUTO_CHANGE_PATTERN_FUNCTION
				send_command[26] = ADTCSM;
				send_command[27] = PSENS;
				#else
					#if FUNCTION_AUTOSEWING && AUTOSEWING_DETECTOR
				
					if( autosewing_control_flag == 1)
					{
						temp8 =	check_DSP3_input();
						if( temp8 & 0x10)
							send_command[26] = OFF; 
						else
							send_command[26] = ON; 
					
						if( temp8 & 0x08 )
							send_command[27] = OFF; 
						else
							send_command[27] = ON; 				
					}
					else
					{
						send_command[26] = 0;
						send_command[27] = 0;
					}
					#else
					send_command[26] = 0;
					send_command[27] = 0;
					#endif
				#endif	
				send_command[28] = verify_code(28);
				send_command[29] = DATA_END;  	                
				tra_com(send_command,30);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;   
			//--------------------------------------------------------------------------------------      
			//  receive output query command
			//--------------------------------------------------------------------------------------
			case OUTPUT:       	 
				switch(rec_buf[3])                      // data package
				{
					case 0x00: 
						output_com = 0;  
						break;
					case 0x01: 
						output_com = 1;  
						break;
					case 0x02: 
						output_com = 2;  
						break;  
					case 0x03: 
						output_com = 3;  
						break; 
					case 0x04: 
						output_com = 4;  
						break; 
					case 0x05: 
						output_com = 5;  
						break; 			  		            							    		            	 	
					case 0x06:  
						output_com = 6;  
						break;		   
					case 0x7:
					    output_com = 7;  
						break;  
					case 0x8:
					    output_com = 8;  
						break;
					case 0x9:
					    output_com = 9;  
						break;
					case 0xa:
					    output_com = 10;  
						break;
					case 0xb:
					    output_com = 11;  
						break;  	
											    		            	 	
					default:   
						output_com = rec_buf[3];
						break;
				}	    
				temp8 = rec_buf[3];      
				if( rec_buf[1] <= 0x04 )
				{
					confirm_barcode = pattern_number;
				}  	        
				else
				{
					temp = (UINT16)rec_buf[4]<<8;
					confirm_barcode = temp | (UINT16)rec_buf[5];
				}     		                        		                                               		                  ;  
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x06;
				send_command[2] = OUTPUT_RET;  				           				          
				send_command[3] = temp8;  	
				
				send_command[4] = rec_buf[4];  
				send_command[5] = rec_buf[5];  			           				          
				send_command[6] = verify_code(6);
				send_command[7] = DATA_END;  	                
				tra_com(send_command,8);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break; 
			//--------------------------------------------------------------------------------------
			// send motor mechanic angle for display
			//--------------------------------------------------------------------------------------
			case MOTORMECHANICANGLECHECK: 
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;  
				send_command[1] = 0x05;   
				send_command[2] = MOTORMECHANICANGLECHECK_RET;
				//			DisMotorAngle = ((INT32)motor.angle * 23040)>>16;
				send_command[3] = motor.angle>>8;
				send_command[4] = motor.angle;
				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;

			//--------------------------------------------------------------------------------------
			// send motor mechanic angle for adjust
			//--------------------------------------------------------------------------------------
			case MOTORMECHANICANGLEENTER: 
				temp16 = (INT16)rec_buf[3]<<8;
				AdjustAngle = temp16 | (INT16)rec_buf[4];
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;  
				send_command[1] = 0x05;   
				send_command[2] = MOTORMECHANICANGLEENTER_RET;
				temp8 = rec_buf[3];
				send_command[3] = temp8;
				temp8 = rec_buf[4];
				send_command[4] = temp8;
				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				//send_dsp_command(DSP1,0x0018);
				break;	
			//--------------------------------------------------------------------------------------      
			//  receive stepping motor next pattern coordinate command //09.2.9 wr add
			//--------------------------------------------------------------------------------------
			case STACOOR:                              		                         
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x10;
				send_command[2] = STACOOR_RET; 

				temp8 = rec_buf[3]; 		          				           				          
				send_command[3] = temp8;  
				temp16 = (INT16)temp8<<8;     				       
				temp8 = rec_buf[4];  				                 				          
				send_command[4] = temp8;
				temp16 = temp16|(INT16)temp8; 
				stacoorx_step = temp16;

				temp8 = rec_buf[5];
				send_command[5] = temp8;  
				temp16 = (INT16)temp8<<8; 
				temp8 = rec_buf[6]; 
				send_command[6] = temp8; 
				temp16 = temp16|(INT16)temp8;
				stacoory_step = temp16;

				temp8 = rec_buf[7]; 		          				           				          
				send_command[7] = temp8;  
				temp16 = (INT16)temp8<<8;     				       
				temp8 = rec_buf[8];  				                 				          
				send_command[8] = temp8;
				temp16 = temp16|(INT16)temp8; 
				origin2x_step = temp16;

				temp8 = rec_buf[9];
				send_command[9] = temp8;  
				temp16 = (INT16)temp8<<8; 
				temp8 = rec_buf[10]; 
				send_command[10] = temp8; 
				temp16 = temp16|(INT16)temp8;
				origin2y_step = temp16;  				          

				temp8 = rec_buf[11]; 		          				           				          
				send_command[11] = temp8;  
				temp16 = (INT16)temp8<<8;     				       
				temp8 = rec_buf[12];  				                 				          
				send_command[12] = temp8;
				temp16 = temp16|(INT16)temp8; 
				currentpyx_step = temp16;

				temp8 = rec_buf[13];
				send_command[13] = temp8;  
				temp16 = (INT16)temp8<<8; 
				temp8 = rec_buf[14]; 
				send_command[14] = temp8; 
				temp16 = temp16|(INT16)temp8;
				currentpyy_step = temp16;  

				curpat_have_origin2 = rec_buf[15];				          

				send_command[16] = verify_code(16); 
				send_command[17] = DATA_END;                 
				tra_com(send_command,18);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break; 
			//--------------------------------------------------------------------------------------      
			//  receive xysensor query command
			//--------------------------------------------------------------------------------------
			case XYSENSOR:        		                         
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x05;
				send_command[2] = XYSENSOR_RET;
				//--------------------------------------------------------------------------------------      
				//  XORG
				//--------------------------------------------------------------------------------------
				if(XORG == ON)
				{
					temp8 = OFF;
				}	
				else
				{
					temp8 = ON;
				}				           				          
				send_command[3] = temp8;  				
				//--------------------------------------------------------------------------------------      
				//  YORG
				//--------------------------------------------------------------------------------------  
				if(YORG == ON)
				{
					temp8 = OFF;
				}	
				else
				{
					temp8 = ON;
				}				           				          
				send_command[4] = temp8;       				          
				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;                  
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;      
			//--------------------------------------------------------------------------------------      
			//  receive pcsensor query command
			//--------------------------------------------------------------------------------------
			case PCSENSOR:        		                         
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x05;
				send_command[2] = PCSENSOR_RET;
				//--------------------------------------------------------------------------------------      
				//  CORG
				//--------------------------------------------------------------------------------------  
				if(CORG == ON)
				{
					temp8 = ON;
				}	
				else
				{
					temp8 = OFF;
				}				           				          
				send_command[3] = temp8; 
				//--------------------------------------------------------------------------------------      
				//  CSENS
				//--------------------------------------------------------------------------------------  
				if(CSENS == ON)
				{
					temp8 = ON;
				}	
				else
				{
					temp8 = OFF;
				}			           				          
				send_command[4] = temp8;             				                 				          
				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;                  
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;     
				//--------------------------------------------------------------------------------------      
			//  receive csensor query command
			//--------------------------------------------------------------------------------------
			case CSENSOR:        		                         
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x05;
				send_command[2] = CSENSOR_RET;
				
				if(TSENS == ON)
				{
					temp8 = ON;
				}	
				else
				{
					temp8 = OFF;
				}				           				          
				send_command[4] = temp8; 
				if(TORG)
				{
					temp8 = ON;
				}	
				else
				{
					temp8 = OFF;
				}
           				          
				send_command[3] = temp8;          				                 				          
				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;                  
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;   
			//--------------------------------------------------------------------------------------      
			//  receive isensor query command
			//--------------------------------------------------------------------------------------
			case ISENSOR:        		                         
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = ISENSOR_RET;
				//--------------------------------------------------------------------------------------      
				//  IORG
				//--------------------------------------------------------------------------------------  
				if(IORG == ON)
				{
					temp8 = ON;
				}	
				else
				{
					temp8 = OFF;
				}				           				          
				send_command[3] = temp8;       				                 				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;                  
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
                    break; 
      		//--------------------------------------------------------------------------------------      
      		//  receive stepping motor single act command 09.3.26 wr add for self test single act of stepmotor
      		//--------------------------------------------------------------------------------------
      		case MOTOCWW: 
      			stepmotor_single = rec_buf[3];
      			         		                         
      			tra_ind_r = 0; 
      			tra_ind_w = 0;                  
  	  			send_command[0] = DATA_START;
  	  			send_command[1] = 0x04;
  				send_command[2] = MOTOCWW_RET;  				          				           				          
  				send_command[3] = stepmotor_single;       				                 				          
  				send_command[4] = verify_code(4);
  				send_command[5] = DATA_END;                  
      			tra_com(send_command,6);    
      			rec_ind_r = 0; 
      			rec_ind_w = 0;
				predit_shift = 4;//2014-8-15
      			break;  
            //--------------------------------------------------------------------------------------      
			//  receive stepping motor command
			//--------------------------------------------------------------------------------------
			case STEPMOVE: 
				stepmotor_comm = rec_buf[3];
 
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = STEPMOVE_RET;  				          				           				          
				send_command[3] = stepmotor_comm;       				                 				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;                  
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				predit_shift = 5 ;
				break;                                                       
			//--------------------------------------------------------------------------------------      
			//  receive stepping motor state query command
			//--------------------------------------------------------------------------------------
			case MOTOSTA:        		                         
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = MOTOSTA_RET;  				          				           				          
				send_command[3] = stepmotor_state;       				                 				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;                  
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;        
			//--------------------------------------------------------------------------------------      
			//  receive stepping motor coordinate command
			//--------------------------------------------------------------------------------------
			case COORCOM:                                    		                         
				tra_ind_r = 0; 
				tra_ind_w = 0;     
				para_length = rec_buf[1];
				if(  para_length < 8)
				{            
					send_command[0] = DATA_START;
					send_command[1] = 0x07;
					send_command[2] = COORCOM_RET; 

					temp8 = rec_buf[3]; 		          				           				          
					send_command[3] = temp8;  
					temp16 = (INT16)temp8<<8;     				       
					temp8 = rec_buf[4];  				                 				          
					send_command[4] = temp8;
					temp16 = temp16|(INT16)temp8; 
					comx_step = temp16;

					temp8 = rec_buf[5];
					send_command[5] = temp8;  
					temp16 = (INT16)temp8<<8; 
					temp8 = rec_buf[6]; 
					send_command[6] = temp8; 
					temp16 = temp16|(INT16)temp8;
					comy_step = temp16;

					send_command[7] = verify_code(7); 
					send_command[8] = DATA_END;                 
					tra_com(send_command,9);  
				}  
				else
				{
					send_command[0] = DATA_START;
					send_command[1] = 11;
					send_command[2] = COORCOM_RET; 
					send_command[3] = rec_buf[7];
					send_command[4] = rec_buf[8];
					send_command[5] = rec_buf[3];
					send_command[6] = rec_buf[4];
					send_command[7] = rec_buf[9];
					send_command[8] = rec_buf[10];
					send_command[9] = rec_buf[5];
					send_command[10] = rec_buf[6];
					send_command[11] = verify_code(11); 
					send_command[12] = DATA_END;                 
					tra_com(send_command,13);
					
					utemp32= ((UINT32)(rec_buf[7])<<24) + ((UINT32)(rec_buf[8])<<16) + ((UINT32)(rec_buf[3])<<8) + (UINT32)(rec_buf[4]);
					comx_step =(INT32) utemp32;
					utemp32= ((UINT32)(rec_buf[9])<<24) + ((UINT32)(rec_buf[10])<<16) + ((UINT32)(rec_buf[5])<<8) + (UINT32)(rec_buf[6]);
					comy_step = (INT32) utemp32;	
				}
				rec_ind_r = 0; 
				rec_ind_w = 0;
				coor_com = 1;
				predit_shift = 6;
				monitor_allx_step = comx_step;
				monitor_ally_step = comy_step;
	
				break; 


			case CANCELALLCOOR:
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x07;
				send_command[2] = CANCELALLCOOR_RET; 

				temp8 = rec_buf[3]; 		          				           				          
				send_command[3] = temp8;  
				temp16 = (INT16)temp8<<8;     				       
				temp8 = rec_buf[4];  				                 				          
				send_command[4] = temp8;
				temp16 = temp16|(INT16)temp8; 
				CancelAllXCoor = temp16>>1;

				temp8 = rec_buf[5];
				send_command[5] = temp8;  
				temp16 = (INT16)temp8<<8; 
				temp8 = rec_buf[6]; 
				send_command[6] = temp8; 
				temp16 = temp16|(INT16)temp8;
				CancelAllYCoor = temp16>>1;

				send_command[7] = verify_code(7); 
				send_command[8] = DATA_END;                 
				tra_com(send_command,9);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				CancelAllCoorFlag = 1;
				break;  

			//--------------------------------------------------------------------------------------      
			//  receive stepping motor coordinate command
			//--------------------------------------------------------------------------------------
			case CUR_COORDINATES:                                    		                         
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x07;
				send_command[2] = CUR_COORDINATES_RET; 

				temp8 = rec_buf[3]; 		          				           				          
				send_command[3] = temp8;  
				temp16 = (INT16)temp8<<8;     				       
				temp8 = rec_buf[4];  				                 				          
				send_command[4] = temp8;
				temp16 = temp16|(INT16)temp8; 
				xcurrent_coordinates = temp16;

				temp8 = rec_buf[5];
				send_command[5] = temp8;  
				temp16 = (INT16)temp8<<8; 
				temp8 = rec_buf[6]; 
				send_command[6] = temp8; 
				temp16 = temp16|(INT16)temp8;
				ycurrent_coordinates = temp16;

				send_command[7] = verify_code(7); 
				send_command[8] = DATA_END;                 
				tra_com(send_command,9);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;
			//--------------------------------------------------------------------------------------      
			//  receive stepping motor state query command
			//--------------------------------------------------------------------------------------
			case PREDIT_SHIFT:        		                         
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x06;
				send_command[2] = PREDIT_SHIFT_RET;  				          				           				          
				send_command[3] = predit_shift;  
				send_command[4] = (UINT8)(drv_satus >> 8);
				send_command[5] = (UINT8)drv_satus;  				                 				          
				send_command[6] = verify_code(6);
				send_command[7] = DATA_END;                  
				tra_com(send_command,8);    
				rec_ind_r = 0; 
				rec_ind_w = 0;

				break; 
			//----------------------------------------------------------------------------- 
			// receive the compensation of shift
			//-----------------------------------------------------------------------------
			case COORADJUST:
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x07;
				send_command[2] = COORADJUST_RET; 

				temp8 = rec_buf[3]; 
				send_command[3] = temp8;
				temp16 = (INT16)temp8<<8;
				temp8 = rec_buf[4];
				send_command[4] = temp8;
				temp16 = temp16|(INT16)temp8;
				cooradjustx_step = temp16;
				//					cooradjustx_step = cooradjustx_step<<1;	
				cooradjustxtotal_step = cooradjustxtotal_step + cooradjustx_step;	
			          

				temp8 = rec_buf[5];
				send_command[5] = temp8;
				temp16 = (INT16)temp8<<8;
				temp8 = rec_buf[6];
				send_command[6] = temp8;
				temp16 = temp16|(INT16)temp8;
				cooradjusty_step = temp16;
				//					cooradjusty_step = cooradjusty_step<<1;
				cooradjustytotal_step = cooradjustytotal_step + cooradjusty_step; 				 


				send_command[7] = verify_code(7); 
				send_command[8] = DATA_END;                 
				tra_com(send_command,9);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break; 	
			case STITCHUP:
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03; 
				send_command[2] = STITCHUP_RET;  
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END; 
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				StitchUpFlag = 1;
				break;     
			case POINTSHIFT:
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = POINTSHIFT_RET;
				PointShiftDirection = rec_buf[3];
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;
				
				temp = (UINT16)rec_buf[4] << 8;                
				PointShiftNumber = temp | (UINT16)rec_buf[5]; 
			    if( rec_buf[6] == 0)
				    pat_buff_write_offset = 0;
				  else
				    pat_buff_write_offset = HALF_WRITE_OFFSET;//12000
				laser_already_begin_flag = rec_buf[9];//
				
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				PointShiftFlag = 1;
				predit_shift = 7 ;//2014-8-15
				break;    
			case CANCELOVERTABPOINT:
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = CANCELOVERTABPOINT_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;  
				CancelOverlabPointFlag = 1;   
				break;
			case ENDSTYLECHOOSE:
				tra_ind_r = 0; 
				tra_ind_w = 0;   
				EndStyleChooseFlag = rec_buf[3];            
				send_command[0] = DATA_START;
				send_command[1] = 0x03; 
				send_command[2] = ENDSTYLECHOOSE_RET; 
				send_command[3] = verify_code(3);  
				send_command[4] = DATA_END; 
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;     
				
			case CURRENTPOINTSHIFT:
				tra_ind_r = 0; 
				tra_ind_w = 0; 
				CurrentPointShiftFlag = 1;
				temp8 = rec_buf[3];
				temp16 = (INT16)temp8<<8;
				temp8 = rec_buf[4];
				CurrentPointShiftPosition = temp16|(INT16)temp8;  
				if( CurrentPointShiftPosition == 0 )
				{
					pat_point = (PATTERN_DATA *)(pat_buf);

					pat_buff_write_offset = 0;
					pat_buff_total_counter = 0;
				}
				send_command[0] = DATA_START;
				send_command[1] = 0x03;   
				send_command[2] = CURRENTPOINTSHIFT_RET; 
				send_command[3] = verify_code(3);  
				send_command[4] = DATA_END; 
				tra_com(send_command,5);    
				rec_ind_r = 0;
				rec_ind_w = 0;          
				break;   
			case PATTERNPOINTSHIFT:
			    predit_shift = 8;  
				tra_ind_r = 0; 
				tra_ind_w = 0;  

				temp8 = rec_buf[3];
				temp16 = (INT16)temp8<<8;
				temp8 = rec_buf[4];
				CurrentPointShiftPosition = temp16|(INT16)temp8;  
				temp8 = rec_buf[5];
				temp16 = (INT16)temp8<<8;
				temp8 = rec_buf[6];
				DestinationPointShiftPosition = temp16|(INT16)temp8;  
				DestinationPointShiftPosition += 2;

				MotionSet =  rec_buf[7];  
				MotiongSetFlag = 1;

				temp8 = rec_buf[8];
				temp16 = (INT16)temp8<<8;
				temp8 = rec_buf[9];
				comx_step = temp16|(INT16)temp8; 
				temp8 = rec_buf[10];
				temp16 = (INT16)temp8<<8;
				temp8 = rec_buf[11];
				comy_step = temp16|(INT16)temp8;  

				send_command[0] = DATA_START;
				send_command[1] = 0x03; 
				send_command[2] = PATTERNPOINTSHIFT_RET; 
				send_command[3] = verify_code(3);  
				send_command[4] = DATA_END; 
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0; 
				break; 
			case EDITELEMENTCOMMAND:
				tra_ind_r = 0; 
				tra_ind_w = 0;
				EditElementCommand = rec_buf[3];   
				send_command[0] = DATA_START;
				send_command[1] = 0x03;   
				send_command[2] = EDITELEMENTCOMMAND_RET; 
				send_command[3] = verify_code(3);  
				send_command[4] = DATA_END; 
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;     
				break;     
			case SEWTESTENDFLAGCHECK:
				tra_ind_r = 0; 
				tra_ind_w = 0;  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;   
				send_command[2] = SEWTESTENDFLAGCHECK_RET; 
				send_command[3] = TestStatusChangeFlag;
				send_command[4] = verify_code(4);  
				send_command[5] = DATA_END; 
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;       
				break;     
			//=============================================================
			case READY_GO_SETOUT:
				predit_shift =9;//ok
				ready_go_setout_com = 1;
				if( sys.status == READY )
				     new_pattern_done = 1;
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = READY_GO_SETOUT_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;
			
			case AUTO_SELECT_PATTERN:
			    temp = pattern_number;
			    tra_ind_r = 0; 
				tra_ind_w = 0;   
			    send_command[0] = DATA_START;
				send_command[1] = 0x05;
				send_command[2] = AUTO_SELECT_PATTERN_RET;  
				send_command[3] = (UINT8)(temp>>8);
				send_command[4] = (UINT8)temp; 				           				           				           				          
				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;  	                
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				id_alarm_flag = 1;
				
			break;		
			
			case ORIGIN:
				
				if( origin_com ==0)
				{
					origin_com = 1;
					predit_shift =10;
				}
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = ORIGIN_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;
		     case CUT_COMMAND:
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = CUT_COMMAND_RET;  				          				           				          
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;                  
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				cut_test_flag = 1;
				predit_shift = 11 ;//2014-8-15
				break;	
			case SET_BASELINE_ALARM:
						baseline_alarm_flag = rec_buf[3];
						temp = (UINT16)rec_buf[4]<<8;
				        baseline_alarm_stitchs = temp | (UINT16)rec_buf[5];//总针数
						if( pat_buff_total_counter >= baseline_alarm_stitchs)
						    baseline_alarm_stitchs += pat_buff_total_counter;
						bobbin_case_once_done_flag = 0;
						tra_ind_r = 0; 
						tra_ind_w = 0;                  
						send_command[0] = DATA_START;
						send_command[1] = 0x03;
						send_command[2] = SET_BASELINE_ALARM_RET;  				          				           				          
						send_command[3] = verify_code(3);
						send_command[4] = DATA_END;                  
						tra_com(send_command,5);    
						rec_ind_r = 0; 
						rec_ind_w = 0;
			break;
			case CHECKI11_TEST:
						checki11_test_flag = 1;
						checki11_item = rec_buf[3];
						checki11_action = rec_buf[4];
						tra_ind_r = 0; 
						tra_ind_w = 0;                  
						send_command[0] = DATA_START;
						send_command[1] = 0x03;
						send_command[2] = CHECKI11_TEST_RET;  				          				           				          
						send_command[3] = verify_code(3);
						send_command[4] = DATA_END;                  
						tra_com(send_command,5);    
						rec_ind_r = 0; 
						rec_ind_w = 0;
			break;
			case BOBBIN_CASE_INPUT:
						tra_ind_r = 0; 
						tra_ind_w = 0;                  
						send_command[0] = DATA_START;
						send_command[1] = 0x08;
						send_command[2] = BOBBIN_CASE_INPUT_RET;  
						#if USE_ENCODER_YJ_PULSE   
						if( get_stepper_cutter_ORG()== 1)
							send_command[3] = 1;
                        else 
						    send_command[3] = 0;
						#else
						if( ROTATED_CUTTER_ORG == 1)
							send_command[3] = 1;
                        else 
						    send_command[3] = 0;
						#endif
						if( BOBBIN_CASE_PLATFORM_ORG )
						 	send_command[4] = 1;
                        else 
						    send_command[4] = 0;
						if( bobbin_case_enable == 1)
						{
							if( get_bobbin_case_arm_org_status() == 1)
							 	send_command[5] = 1;
	                        else 
							    send_command[5] = 0;
						}
						else
							send_command[5] = 0;
						if( BOBBIN_CASE_EMPTY_CHECK == 1)
						 	send_command[6] = 1;
                        else 
						    send_command[6] = 0;
						if ( LASER_HEADER_PROTECT == 1)
							send_command[7] = 1;
						else
							send_command[7] = 0;										          				           				          
						send_command[8] = verify_code(8);
						send_command[9] = DATA_END;                  
						tra_com(send_command,10);    
						rec_ind_r = 0; 
						rec_ind_w = 0;
			break;
			#if SUPPORT_UNIFY_DRIVER
			
			case DISPLAY_SERVOPARA:
			    read_stepmotor_config_para(debug_dsp_flag+1);
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 208;
				send_command[2] = DISPLAY_SERVOPARA_RET;  				          				           				          
				for(i=0;i<205;i++)  
				{
					send_command[3+i] = svpara_disp_buf[i];
				}       				                 				          
				send_command[208] = verify_code(208);
				send_command[209] = DATA_END;                  
				tra_com(send_command,210);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;	
				
			case SERVOPARA:
			    for(i=0;i<rec_buf[1]-3;i++)
				{
					svpara_disp_buf[i]=rec_buf[3+i];
				} 
				wirte_stepermotor_para_flag = 1;					
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = SERVOPARA_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break; 
			#else
			case DISPLAY_SERVOPARA:
			    read_stepmotor_para();
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x60;
				send_command[2] = DISPLAY_SERVOPARA_RET;  				          				           				          
				for(i=0;i<93;i++)  
				{
					send_command[3+i] = svpara_disp_buf[i];
				}       				                 				          
				send_command[96] = verify_code(96);
				send_command[97] = DATA_END;                  
				tra_com(send_command,98);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;	
				
			case SERVOPARA:
			    for(i=0;i<rec_buf[1]-3;i++)
				{
					svpara_buf[i]=rec_buf[3+i];
				} 
				svpara_trans_flag = 1;					
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = SERVOPARA_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break; 
            #endif				
			//--------------------------------------------------------------------------------------			  
			//  DSP update  function code
			//--------------------------------------------------------------------------------------
			case DSP_DOWNLOAD_CODE://92
			  
			    data_length_drv = rec_buf[1];
				if((0 == rec_buf[4]) && (0 == rec_buf[5])&& (0 == rec_buf[6]))
				   erase_falg = 1;
				else 
				   erase_falg = 0;				
				
				recpat_point = (UINT8 *)(rec_buf);      
	            
				for(i=0;i<rec_buf[1]+2;i++)
				{
					pat_buf[i] = *recpat_point;
					recpat_point++;
				} 
				
				DRV_DATA_LEN++;    
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = DSP_DOWNLOAD_CODE_RET;
				send_command[3] = DRV_DATA_LEN;//
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				predit_shift = 5;//1
	
				break; 
			case DSP_DOWNLOAD_VERIFY:
			    
			    data_length_drv = rec_buf[1]; 
				recpat_point = (UINT8 *)(rec_buf);   // data address   
	            
				for(i=0;i<rec_buf[1]+2;i++)
				{
					pat_buf[i] = *recpat_point;
					recpat_point++;
				} 

				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;//data_length;
				send_command[2] = DSP_DOWNLOAD_VERIFY_RET;
				send_command[3] = DRV_DATA_LEN;
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				predit_shift = 6;//2
				DRV_DATA_LEN =0;
				break; 
			//#endif
			//=============================================================                                                                                                                                                                  
			case DOWNLOAD_STEP_CURVE:
			    predit_shift = 9;
				debug_dsp_flag = rec_buf[3]+1;//dsp1-0,dsp2-1
				temp = rec_buf[4];			  //package number								
				temp = 240*temp;
				recpat_point = (UINT8 *)&(rec_buf + 5);
			    for(i=0;i< rec_buf[1] - 5;i++)
				{
					pat_buf[ temp + i] = *recpat_point;
					recpat_point++;
					step_curve_write_counter ++;
				}
				temp = rec_buf[1]- 2;
				crc_value = rec_buf[temp + 1] ;   
				crc_value = (crc_value<<8) + (UINT16)rec_buf[temp];//最后两个字节是校验CRC
				
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = DOWNLOAD_STEP_CURVE_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				predit_shift = 0;
			break;
			case DOWNLOAD_STEP_CURVE_DONE:
				predit_shift = 10;
				write_stepmotor_curve_flag = debug_dsp_flag;			
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x05;
				send_command[2] = DOWNLOAD_STEP_CURVE_DONE_RET;
				send_command[3] = (UINT8)(crc_value);
				send_command[4] = (UINT8)(crc_value>>8);
				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;  	                
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;
			#if ENABLE_RFID_FUNCTION
			case RFID_WRITE:
				 rc522_control_falg = 1;
				 rc522_write_falg = 1;
				 rc522_write_ret_falg = 1;
				 temp16 = (UINT16)rec_buf[3]<<8;
			     Rfid_Nom = temp16 | (UINT16)rec_buf[4];
			  break;
			case RFID_READ:		
				 rc522_write_falg = 0;
				 rc522_control_falg = 1; 
				 
			  break;
			#endif
			
			#if ENABLE_CONFIG_PARA == 1
			
		    case SYS_PARAM_GROUP://读参数
					temp = ((UINT16)rec_buf[3] << 8) + rec_buf[4];
					tra_ind_r = 0; 
					tra_ind_w = 0;
		
					if( rec_buf[5] == 0 )//read
					{
						send_command[0] = DATA_START;
						send_command[1] = 210;
						send_command[2] = SYS_PARAM_GROUP_RET;						
						send_command[3] = rec_buf[5];
						send_command[4] = 0;//ok
						if( temp == 1)
							read_para_group(100,svpara_disp_buf,205);
						else if( temp == 2)
							read_para_group(400,svpara_disp_buf,205);
						else if( temp == 3)
							read_stepmotor_config_para(1);
						else if( temp == 4)
							read_stepmotor_config_para(2);
						else if( temp == 5)
							read_para_group(700,svpara_disp_buf,205);
						else if( temp == 6)
							read_stepmotor_config_para(3);

						for( i=0; i< 205 ; i++)  
						{
							send_command[5+i] = svpara_disp_buf[i];
						}      
						send_command[210] = verify_code(210);
						send_command[211] = DATA_END;                  
						tra_com(send_command,212);    
						rec_ind_r = 0; 
						rec_ind_w = 0;
					}
					else  //write;
					{
						//SUM = 1;
					    //delay_us(20000);
					    //SUM = 0;
						send_command[0] = DATA_START;
						send_command[1] = 5;
						send_command[2] = SYS_PARAM_GROUP_RET;						
						send_command[3] = rec_buf[5];
						send_command[4] = 0;//ok
						for( i=0; i< 205 ; i++)  
						{
							svpara_disp_buf[i] = rec_buf[6 + i];
						}  
						if( temp == 1)
						{
							write_eeprom_para_flag = 1;							
						}
						else if( temp == 2)
						{
							write_eeprom_para_flag = 2;
							
						}
						else if( temp == 3)
						{
						    wirte_stepermotor_para_flag = 1;
							debug_dsp_flag = 0;
						}
						else if( temp == 4)
						{
							wirte_stepermotor_para_flag = 2;
							debug_dsp_flag = 1;
						}
						else if( temp == 5)
							write_eeprom_para_flag = 3;
						else if( temp == 6)
						{
							wirte_stepermotor_para_flag = 3;
							debug_dsp_flag = 2;
						}
						send_command[5] = verify_code(5);
						send_command[6] = DATA_END;                  
						tra_com(send_command,7);    
						rec_ind_r = 0; 
						rec_ind_w = 0;
					}
			break;
			case RESET_USERPARAM:
					tra_ind_r = 0; 
					tra_ind_w = 0;
					send_command[0] = DATA_START;
					send_command[1] = 0x04;
					send_command[2] = RESET_USERPARAM_RET;
					send_command[3] = 0;	
					send_command[4] = verify_code(4);
					send_command[5] = DATA_END;
					tra_com(send_command,6);
					rec_ind_r = 0; 
					rec_ind_w = 0;
			break;
			#endif
			
			case TEST_IOOUTPUT:
				#if MULTIPULE_IO_ENABLE ==1
				if(rec_buf[4]==0)
				   set_output_to_IO(SPI_STM32_PORT,rec_buf[3],0);
				else
				   set_output_to_IO(SPI_STM32_PORT,rec_buf[3],1);				   
				#endif
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;  
				send_command[1] = 0x07;   
				send_command[2] = TEST_IOOUTPUT_RET;
		        send_command[3] = stm32_output[0];
				send_command[4] = stm32_output[1];
		        send_command[5] = stm32_output[2];
		        send_command[6] = stm32_output[3];        
				send_command[7] = verify_code(7);
				send_command[8] = DATA_END;
				tra_com(send_command,9);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;
				
			case    QUERY_ID:
					tra_ind_r = 0; 
					tra_ind_w = 0;
					send_command[0] = DATA_START;
					send_command[1] = 0x0C;
					send_command[2] = QUERY_ID_RET;
					send_command[3] = 0x11;
					send_command[4] = 0x22;
					send_command[5] = 0x33;
					send_command[6] = 0x44;
					send_command[7] = 0x55;
					send_command[8] = 0x66;
					send_command[9] = 0x77;
					send_command[10] = 0x88;
					send_command[11] = main_control_lock_flag;
					send_command[12] = verify_code(12);
					send_command[13] = DATA_END;
					tra_com(send_command,14);
					rec_ind_r = 0; 
					rec_ind_w = 0; 
			  break;		
			 
			  case   SET_MAIN_CONTROL_LOCK:			 
			        main_control_lock_flag = rec_buf[3];					
			 		tra_ind_r = 0; 
					tra_ind_w = 0;
					send_command[0] = DATA_START;
					send_command[1] = 0x04;
					send_command[2] = SET_MAIN_CONTROL_LOCK_RET;
					send_command[3] = main_control_lock_flag;
					send_command[4] = verify_code(4);
					send_command[5] = DATA_END;                  
					tra_com(send_command,6);    
					rec_ind_r = 0; 
					rec_ind_w = 0;					
					main_control_lock_setup = 1;			
					
			  break;
			default:   
				com_error();       
				break;            
		}
	}
	else
	{
		com_error();                  // return communication error to panel
	}	
}

#if ENABLE_RFID_FUNCTION

void rfid_wr_ret(void)
{
	if(rc522_write_falg ==1)
	{	
		tra_ind_r = 0; 
		tra_ind_w = 0;           
		send_command[0] = DATA_START;
		send_command[1] = 0x04;
		send_command[2] = RFID_WRITE_RET; 
		send_command[3] = 0;//rc522_write_ret_falg;//rc522_write_falg_ret;				          				           				          
		send_command[4] = verify_code(4);
		send_command[5] = DATA_END;                  
		tra_com(send_command,6);    
		rec_ind_r = 0; 
		rec_ind_w = 0;
	}
	else
	{
		
		tra_ind_r = 0; 
		tra_ind_w = 0;                        		            
		send_command[0] = DATA_START;  
		send_command[1] = 0x05;   
		send_command[2] = RFID_READ_RET;		
		send_command[3] = serail_number>>8;
		send_command[4] = serail_number;
		send_command[5] = verify_code(5);
		send_command[6] = DATA_END;
		tra_com(send_command,7);    
		rec_ind_r = 0; 
		rec_ind_w = 0;
	}
	rc522_write_falg = 0;
	rc522_control_falg = 0; 
	SUM = 0;
}
#endif
//--------------------------------------------------------------------------------------
//  Name:  com_error
//  pars: NONE
//  Returns: NONE
//  Description: return communication error to panel
//--------------------------------------------------------------------------------------
void com_error(void)
{

	/*
	tra_ind_r = 0; 
	tra_ind_w = 0; 
	send_command[0] = DATA_START;
	send_command[1] = 0x03;
	send_command[2] = MISTAKE_RET;
	send_command[3] = verify_code(3);
	send_command[4] = DATA_END;
	tra_com(send_command,5);     // return 5A,03,AA,0C,FF
	*/
	find_communication_start_flag = 0;  //2013-9-17 add
	rec_ind_r = 0; 
	rec_ind_w = 0; 
	//predit_shift = 0;//2014-8-15
}
//--------------------------------------------------------------------------------------
//  Name:  verify_code
//  pars: UINT8 number
//  Returns: UINT8
//  Description: make verify code
//--------------------------------------------------------------------------------------
UINT8 verify_code(UINT8 number)
{
	UINT16 i; 
	UINT8 result; 
	
	result = send_command[0];
	for(i=1;i<number;i++)
	{
    	result = result ^ send_command[i];
	}	
	return result;
}

//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------