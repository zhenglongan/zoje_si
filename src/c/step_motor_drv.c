//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : stepmotor.c
//  Description: stepmotor control 
//  Version    Date     Author    Description
//  ...
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"       //M16C/62P special function register definitions
#include "..\..\include\typedef.h"      //Data type define
#include "..\..\include\variables.h"    //External variables declaration
#include "..\..\include\common.h"       //Common constants definition
#include "..\..\include\delay.h"        // delay time definition
#include "..\..\include\action.h"       // delay time definition
#include "..\..\include\M_debug.h" 
//--------------------------------------------------------------------------------------
//  local variables define
//--------------------------------------------------------------------------------------
static UINT32 x_data_nf;
static UINT32 y_data_nf;
static UINT16 yj_data_nf;
static UINT16 zx_data_nf;
static UINT16 ct_data_nf;

static UINT16 step_cfg_data_i1;
static UINT16 step_cfg_data_i2;
static UINT16 step_cfg_data_a1;
static UINT16 step_cfg_data_a2;
static UINT16 step_cfg_data_h;
static UINT16 step_cfg_data;

static UINT8 re_trans_flag;
static UINT8 re_trans_time;

static UINT16 trans_dsp1;
static UINT16 trans_dsp2;
static UINT16 trans_dsp3;
static UINT16 trans_dsp4;
static UINT8 dsp1;
static UINT8 dsp2;
static UINT8 dsp3;
static UINT8 dsp4;
//--------------------------------------------------------------------------------------
//  global variables declaration
//--------------------------------------------------------------------------------------
union TRANS trans_x;
union TRANS trans_y;
union TRANS trans_z;
union RECV recieve_x;
union RECV recieve_y;
union RECV recieve_z;


void quickmove_y2(int y_data);

void ready_time(void);
void quickmove_time(unsigned int y_time);
void adjust_step_angle(void);
void quickmove_x2(INT32 x_data);
void rotated_cutter_single_stop(void);
void movestep_cs3(UINT16 command,INT16 x_data,UINT8 timer_need);
void output_cs3(UINT8 x_data,UINT8 timer_need);
UINT8 find_a_bobbin_case(UINT8 full);
void rotated_cutter_single_next(void);
void rotated_cutter_single_back(void);
void send_dsp1_command(UINT16 command,UINT16 data);    
void send_dsp2_command(UINT16 command,UINT16 data);    
void send_dsp3_command(UINT16 command,UINT16 data); 
void send_dsp4_command(UINT16 command,UINT16 data);
void send_dsp_command(UINT8 port,UINT16 command);
void quickmove_x_process(INT32 time, INT32 data);
void quickmove_y_process(INT32 time, INT32 data);
UINT8 get_bobbin_case_arm_org_status(void);
UINT8 requery_dsp1_status(void);
UINT8 check_motion_done(void);
UINT16 get_YJORG_status(void);
UINT16 get_stepper_cutter_ORG(void);
UINT16 get_cutter_motor_angle(void);
void set_rotated_cutter_speed(UINT16 speed);

void movestep_xy3(INT8 * buf_x,INT8 * buf_y,UINT8 length,UINT8 *spd_tab);
const UINT16 x_step_configure_tab[]={0x0f00,0x1f00,0x2e00,0x3c00,0x4a00,0x5900,0x6800,0x7700,0x8700,0x9600,0xa600,0xb500,0xc500,0xd500,0xe400,0xf400};
const UINT16 y_step_configure_tab[]={0x00f0,0x00f1,0x00e2,0x00c3,0x00a4,0x0095,0x0086,0x0077,0x0078,0x0069,0x006a,0x005b,0x005c,0x005d,0x004e,0x004f};

//bit 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
//    [����1    ] [   ����1 ] [ ����2   ] [ ����2   ]

//����1 ��ѹ�ŵ�3· + ץ�ߵ�4·
const UINT16 inpress_step_configure_tab[]= {0x0700,0x1600,0x2600,0x3500,0x4500,0x5500,0x6500,0x7500,0x8400,0x9400,0xaa00,0xbb00,0xcc00,0xd400,0xe300,0xf300};
const UINT16 foot_step_configure_tab[]=    {0x0080,0x0081,0x0072,0x0073,0x0074,0x0065,0x0066,0x0067,0x0028,0x0059,0x005a,0x005b,0x005c,0x004d,0x004e,0x004f};

//����2 ץ�ߵ�3· + ��ѹ�ŵ�4·
const UINT16 inpress_step_configure_tab4[]={0x0070,0x0061,0x0062,0x0053,0x0054,0x0055,0x0056,0x0057,0x0068,0x0069,0x00aa,0x00bb,0x00cc,0x004d,0x003e,0x003f};
const UINT16 foot_step_configure_tab4[]=   {0x0800,0x1800,0x2700,0x3700,0x4700,0x5600,0x6600,0x7600,0x8200,0x9500,0xa500,0xb500,0xc500,0xd400,0xe400,0xf400};

#if SUPPORT_NEW_DRIVER
const UINT16 foot_step_configure_tab5[]=    {0x0800,0x1800,0x2700,0x3700,0x4700,0x5600,0x6600,0x7600,0x8200,0x9500,0xa500,0xb500,0xc500,0xd400,0xe400,0xf400};
const UINT16 inpress_step_configure_tab5[]= {0x0011,0x0012,0x0022,0x0032,0x0042,0x0052,0x0063,0x0073,0x0083,0x0093,0x00a2,0x00b2,0x00c2,0x00d2,0x00e2,0x00f2};
#else
const UINT16 foot_step_configure_tab5[]=    {0x0800,0x1800,0x2700,0x3700,0x4700,0x5600,0x6600,0x7600,0x8200,0x9500,0xa500,0xb500,0xc500,0xd400,0xe400,0xf400};
const UINT16 inpress_step_configure_tab5[]= {0x0007,0x0016,0x0026,0x0035,0x0045,0x0055,0x0065,0x0072,0x0082,0x0094,0x00aa,0x00bb,0x00cc,0x00d4,0x00e3,0x00f3};
#endif

const UINT16 cutter_current_configure_tab[]= {0x0011,0x0011,0x0022,0x0033,0x0034,0x0035,0x0036,0x0037,0x0038,0x0039,0x003a,0x003a,0x003a,0x003a,0x003a,0x003a};

//--------------------------------------------------------------------------------------
//  Internal functions and subroutines declaration
//--------------------------------------------------------------------------------------
void init_stepmotor_drv(void);
void quickmove_yj_process(INT32 quick_time, INT32 tempx_step);
UINT8 check_yj_done(void);

#pragma	INTERRUPT/E spiint
void spiint(void);
//--------------------------------------------------------------------------------------
//  Name:		SPI_init
//  pars:	    None
//  Returns:	None
//  Description: initial SPI
//--------------------------------------------------------------------------------------
void SPI_init(void)
{
	SPISTE1 = 1;     // DSP1 SPI disable 
	SPISTE2 = 1;     // DSP2 SPI disable 
	SPISTE3 = 1;     // DSP3 SPI disable 
	SPISTE4 = 1;     // DSP4 SPI disable 
	
	prc2 = 1;        // protect disable
	pd9_5 = 1;       // set SPI CLK  pin to output
	pd9_7 = 0;       // set SPI SIN  pin to input
	pd9_6 = 1;       // set SPI SOUT pin to output
	prc2 = 0;        // protect enable
	
	p9_5 = 0;	     // leave CLK low
	
	prc2 = 1;        // protect disable
	s4c = 0x68;      // internal clock, MSB, CLK, f1SIO 

    s4brg = 0x0B;    // Divide clock by 12(0x0B)---1M    40(0x27)---300K

	prc2 = 0;        // protect enable
	
	ir_s4ic = 0;	 // clear the ir bit
	s4ic = SPI_IPL;  // 7 level
	ifsr = 0;        // interrupt source select
}
/********************************
 * SPI
 ********************************/
void spi_err_dis(UINT16 err)
{
	if( sys.error == 0)
		sys.error = err;				     // stepping motor overcurrent
	sys.status = ERROR;
	StatusChangeLatch = ERROR;
	spi_flag=0;
	re_trans_time=0;
	dsp1 = 0;
	dsp2 = 0;	
	dsp3 = 0;
	dsp4 = 0;
	SPISTE1 = 1;
	SPISTE2 = 1;
	SPISTE3 = 1;
	SPISTE4 = 1;
}
//--------------------------------------------------------------------------------------
//  Name:		spiint
//  pars:	    None
//  Returns:	None
//  Description: SPI interrupt
//--------------------------------------------------------------------------------------
void spiint(void)
{
	switch(spi_flag)
	{
	 //--------------------------------------------------------------------------------------
     //  spi_flag==6
     //--------------------------------------------------------------------------------------		
    	case 6:
	  	
	  		ir_int5ic=0;
	  		recieve_z.byte.byte2= s4trr;
	  		SPISTE1 = 1;
	  		SPISTE2 = 1;
			SPISTE3 = 1;
			SPISTE4 = 1;
	  		if((dsp1==1)&&(recieve_z.word!=trans_dsp1))
	  		{
	  		  #if DOUBLE_X_60MOTOR
				if( (recieve_z.word == OVC_DSP1)||(recieve_z.word == OVC_DSP2) )//0XD1A1
    			{
	  				spi_err_dis(ERROR_66);
    			}
				else if( (recieve_z.word == OVS_DSP1)||(recieve_z.word == OVS_DSP2) )//0XD3A3
    			{
    				spi_err_dis(ERROR_11);
    			}
				else if( (recieve_z.word == OVD_DSP1)||(recieve_z.word == OVD_DSP2) )//0XD5A5
    			{
    				spi_err_dis(ERROR_12);
    			}
				else if( (recieve_z.word == DSP_VERIFY_ERROR)||(recieve_z.word == DSP_UNDEFINE_COMMAND) )
    			{
    				spi_err_dis(ERROR_52);
    			}
				else //if(recieve_z.word == 0xffff)
				{
	  				re_trans_flag=1;
					re_trans_time++;
				}				
				
			#else				
				
				if(recieve_z.word == OVC_DSP1)//0XD1A1
    			{
	  				spi_err_dis(ERROR_66);
    			}
				else if(recieve_z.word == OVS_DSP1)//0XD3A3
    			{
    				spi_err_dis(ERROR_11);
    			}
				else if(recieve_z.word == OVD_DSP1)//0XD5A5
    			{
    				spi_err_dis(ERROR_12);
    			}
				else if(recieve_z.word == OVC_DSP2)//0XD2A2
    			{
    				spi_err_dis(ERROR_47);
    			}
				else if(recieve_z.word == OVS_DSP2)//0XD4A4
    			{
    				spi_err_dis(ERROR_43);
    			}
				else if(recieve_z.word == OVD_DSP2)//0XD6A6
    			{
    				spi_err_dis(ERROR_44);
    			}
				else if( (recieve_z.word == DSP_VERIFY_ERROR)||(recieve_z.word == DSP_UNDEFINE_COMMAND) )
    			{
    				spi_err_dis(ERROR_52);
    			}
				else if(recieve_z.word == 0xffff)
				{
	  				re_trans_flag=1;
					re_trans_time++;
				}	
				#endif	
	  		}
			
		  	if((dsp2==1)&&(recieve_z.word!=trans_dsp2))
	  		{
	  			#if DOUBLE_X_60MOTOR
				
				if(recieve_z.word == OVC_DSP1)//0XD1A1
    			{
	  				spi_err_dis(ERROR_47);
    			}
				else if(recieve_z.word == OVS_DSP1)//0XD3A3
    			{
    				spi_err_dis(ERROR_43);
    			}
				else if(recieve_z.word == OVD_DSP1)//0XD5A5
    			{
    				spi_err_dis(ERROR_44);
    			}
				else if(recieve_z.word == OVC_DSP2)//0XD2A2
    			{
    				spi_err_dis(ERROR_46);
    			}
				else if( (recieve_z.word == DSP_VERIFY_ERROR)||(recieve_z.word == DSP_UNDEFINE_COMMAND) )
    			{
    				spi_err_dis(ERROR_52);
    			}
				else //if(recieve_z.word == 0xffff)
				{
	  				re_trans_flag=1;
					re_trans_time++;					
				}	
				
				#else
				
				if( recieve_z.word == OVC_DSP1)//clamp
				{
					spi_err_dis(ERROR_67);
				}
				else if(recieve_z.word == OVC_DSP2)//middle inpresser
    			{
    				spi_err_dis(ERROR_46);
    			}
				else if(recieve_z.word == OVD_DSP1)//0XD5A5
    			{
    				spi_err_dis(ERROR_71);
    			}
				else if( (recieve_z.word == DSP_VERIFY_ERROR)||(recieve_z.word == DSP_UNDEFINE_COMMAND) )
    			{
    				spi_err_dis(ERROR_52);
    			}
				else if(recieve_z.word == 0xffff)
				{
	  				re_trans_flag=1;
					re_trans_time++;

				}	
				#endif		
	  		}
			#if MULTIPULE_IO_ENABLE == 1
			if((dsp3==1)&&(recieve_z.word!=trans_dsp3))
	  		{
				if(recieve_z.word == OVC_DSP1)//0XD1A1
    			{				     
	  				spi_err_dis(ERROR_89);// stepping motor overcurrent
    			}
				else if(recieve_z.word == 0xffff)
				{
					re_trans_flag=1;
					re_trans_time++;
				}				
			}
			if((dsp4==1)&&(recieve_z.word != trans_dsp4))
	  		{
		
			}	
			#else 
			if((dsp3==1)&&(recieve_z.word!=trans_dsp3))
	  		{
				if(recieve_z.word == OVC_DSP1)//0XD1A1
    			{				     
	  				spi_err_dis(ERROR_89);// stepping motor overcurrent
    			}
				else if(recieve_z.word == 0xffff)
				{
					re_trans_flag = 1;
					re_trans_time ++;
				}				
			}
			#endif

		  	if( re_trans_time >= 1)
		  	{									
		  		if((dsp1==1)||(dsp2==1) )
				{
					if( sys.error == 0)
					{
					    sys.error = ERROR_30;						
					}	
					de_bug.test4 = sys.status;
					if( dsp1 == 1)					
				 		de_bug.test3 = 1;
					else
						de_bug.test3 = 2;
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
				}
				else if( dsp3 == 1)
				{
					if( sys.error == 0)
					    sys.error = ERROR_90;
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
				}
				else if( dsp4 == 1)
				{
					
				}		  		
	  		}
			spi_flag = 0;
			re_trans_time = 0;
			re_trans_flag = 0;
			dsp1 = 0;
			dsp2 = 0;
			dsp3 = 0;
			dsp4 = 0;
	  		break;
	  	
    	//--------------------------------------------------------------------------------------
    	//  spi_flag==5
    	//--------------------------------------------------------------------------------------		
    	case 5:
	    	ir_int5ic=0;
	    	recieve_z.byte.byte1=s4trr;
	    	s4trr=trans_z.byte.byte2;
	    	spi_flag=6;
	    	break;
    	//--------------------------------------------------------------------------------------
    	//  spi_flag==4
    	//--------------------------------------------------------------------------------------		
    	case 4:         
    		ir_int5ic=0;
    		recieve_y.byte.byte2= s4trr;              	
    		if((dsp1==1)&&(recieve_y.word!=trans_dsp1))
    		{
				#if DOUBLE_X_60MOTOR
				if( (recieve_y.word == OVC_DSP1)||(recieve_y.word == OVC_DSP2) )
    			{				     
	  				spi_err_dis(ERROR_66);// stepping motor overcurrent
    			}
				else if( (recieve_y.word == OVS_DSP1)||(recieve_y.word == OVS_DSP2) )
    			{
    				spi_err_dis(ERROR_11);
    			}
				else if( (recieve_y.word == OVD_DSP1)||(recieve_y.word == OVD_DSP2) )
    			{
    				spi_err_dis(ERROR_12);
    			}
				else if( (recieve_y.word == DSP_VERIFY_ERROR)||(recieve_y.word == DSP_UNDEFINE_COMMAND) )
    			{
    				spi_err_dis(ERROR_52);
    			}
				else //if(recieve_y.word == 0xffff)
				{
					re_trans_flag=1;
					re_trans_time++;
				}
				#else
				
    			if(recieve_y.word == OVC_DSP1)
    			{				     
	  				spi_err_dis(ERROR_31);// stepping motor overcurrent
    			}
				else if(recieve_y.word == OVS_DSP1)
    			{
    				spi_err_dis(ERROR_11);
    			}
				else if(recieve_y.word == OVD_DSP1)
    			{
    				spi_err_dis(ERROR_12);
    			}
				else if(recieve_y.word == OVC_DSP2)//0XD2A2
    			{
    				spi_err_dis(ERROR_47);
    			}
				else if(recieve_y.word == OVS_DSP2)//0XD4A4
    			{
    				spi_err_dis(ERROR_43);
    			}
				else if(recieve_y.word == OVD_DSP2)//0XD6A6
    			{
    				spi_err_dis(ERROR_44);
    			}
				else if( (recieve_y.word == DSP_VERIFY_ERROR)||(recieve_y.word == DSP_UNDEFINE_COMMAND) )
    			{
    				spi_err_dis(ERROR_52);
    			}
				else if(recieve_y.word == 0xffff)
				{
					re_trans_flag=1;
					re_trans_time++;

				}	
				#endif	    				
    		}
	    	if((dsp2==1)&&(recieve_y.word!=trans_dsp2))
    		{
				#if DOUBLE_X_60MOTOR
				if(recieve_y.word == OVC_DSP1)//0XD1A1
    			{
	  				spi_err_dis(ERROR_47);
    			}
				else if(recieve_y.word == OVS_DSP1)//0XD3A3
    			{
    				spi_err_dis(ERROR_43);
    			}
				else if(recieve_y.word == OVD_DSP1)//0XD5A5
    			{
    				spi_err_dis(ERROR_44);
    			}
				else if(recieve_y.word == OVC_DSP2)//0XD2A2
    			{
    				spi_err_dis(ERROR_46);
    			}
				else if( (recieve_y.word == DSP_VERIFY_ERROR)||(recieve_y.word == DSP_UNDEFINE_COMMAND) )
    			{
    				spi_err_dis(ERROR_52);
    			}
				else //if(recieve_y.word == 0xffff)
				{
	  				re_trans_flag=1;
					re_trans_time++;

				}	
				#else
				
				if(recieve_y.word == OVC_DSP1)
				{
					spi_err_dis(ERROR_67);
				}
				if(recieve_y.word == OVC_DSP2)
    			{
    				spi_err_dis(ERROR_46);
    			}
				else if(recieve_y.word == OVD_DSP1)//0XD5A5
    			{
    				spi_err_dis(ERROR_71);
    			}
				else if( (recieve_y.word == DSP_VERIFY_ERROR)||(recieve_y.word == DSP_UNDEFINE_COMMAND) )
    			{
    				spi_err_dis(ERROR_52);
    			}
				else if(recieve_y.word == 0xffff)
				{
					re_trans_flag=1;	
					re_trans_time++;		
				}
				#endif 
    		}
			
			#if MULTIPULE_IO_ENABLE == 1
			if((dsp3==1)&&(recieve_y.word!=trans_dsp3))
    		{
				if(recieve_y.word == OVC_DSP1)//0XD1A1
    			{				     
	  				spi_err_dis(ERROR_89);// stepping motor overcurrent
    			}
				else if(recieve_y.word == 0xffff)
				{
					re_trans_flag=1;
					re_trans_time++;
				}
			}
			if((dsp4==1)&&(recieve_y.word!=trans_dsp4))
    		{
				
			}
			#else
			if((dsp3==1)&&(recieve_y.word!=trans_dsp3))
    		{
				if(recieve_y.word == OVC_DSP1)//0XD1A1
    			{				     
	  				spi_err_dis(ERROR_89);// stepping motor overcurrent
    			}
				else if(recieve_y.word == 0xffff)
				{
					re_trans_flag=1;
					re_trans_time++;
				}
			}
			#endif 
			 
	    	if(dsp1 == 1)
	    	{
    			trans_dsp1 = trans_y.word;
	    	}
	    	if(dsp2 == 1)
	    	{
    			trans_dsp2 = trans_y.word;    		
    		}    
			if(dsp3 == 1)
	    	{
    			trans_dsp3 = trans_y.word;    		
    		} 	
			if(dsp4 == 1)
	    	{
    			trans_dsp4 = trans_y.word;    		
    		} 
			s4trr = trans_z.byte.byte1;									
			spi_flag = 5;

	    	break;
    	//--------------------------------------------------------------------------------------
    	//  spi_flag==3
    	//--------------------------------------------------------------------------------------		
    	case 3:
	    	ir_int5ic = 0;
	    	recieve_y.byte.byte1 = s4trr;
	    	s4trr = trans_y.byte.byte2;
	    	spi_flag = 4;
	    	break;
    	//--------------------------------------------------------------------------------------
    	//  spi_flag==2
    	//--------------------------------------------------------------------------------------		
    	case 2:        
    		ir_int5ic = 0;
    		recieve_x.byte.byte2 = s4trr;              	
    		if(dsp1 == 1)
    		{
    			trans_dsp1 = trans_x.word;    		
    		}
    		if(dsp2 == 1)
    		{
    			trans_dsp2 = trans_x.word;    		
    		}
			if(dsp3 == 1)
    		{
    			trans_dsp3 = trans_x.word;    		
    		}
			if(dsp4 == 1)
    		{
    			trans_dsp4 = trans_x.word;    		
    		}
    		s4trr = trans_y.byte.byte1;	    										
    		spi_flag = 3;
    		break;
    	//--------------------------------------------------------------------------------------
    	//  spi_flag==1
    	//--------------------------------------------------------------------------------------		
    	case 1:
	    	ir_int5ic = 0;
	    	recieve_x.byte.byte1 = s4trr;
	    	s4trr = trans_x.byte.byte2;		    	
	    	spi_flag = 2;
	    	break;
  	}

}
//--------------------------------------------------------------------------------------
//  Name:		init_stepmotor_drv
//  pars:	    None
//  Returns:	None
//  Description: initial step motor drv
//--------------------------------------------------------------------------------------
void init_stepmotor_drv(void)
{	
	x_data_nf = 0;
	y_data_nf = 0;	
	yj_data_nf = 0; 
	zx_data_nf = 0; 
    timer_x = 0;
	timer_y = 0;
	timer_yj = 0;
	timer_zx = 0;
	
	SPI_init();
	spi_flag = 0;
	
	trans_dsp1 = 0x0d155;
	trans_dsp2 = 0x0d255;
	trans_dsp3 = 0x0d355;
	trans_dsp4 = 0x0d455;
	dsp1 = 0;
	dsp2 = 0;
	dsp3 = 0;
	dsp4 = 0;
	
	re_trans_flag=0;
	re_trans_time=0;

}

//--------------------------------------------------------------------------------------
//  Name:		config_stepmotor_drv
//  pars:	    None
//  Returns:	None
//  Description: config stepping motor driver
//--------------------------------------------------------------------------------------
void config_stepmotor_drv(void) 
{
    if(x_step_current_level > 12)  
	   x_step_current_level = 12;
	if(y_step_current_level > 12)
	   y_step_current_level = 12;	  
	step_cfg_data = x_step_configure_tab[x_step_current_level] + y_step_configure_tab[y_step_current_level];    
    send_dsp1_command(0x11,step_cfg_data); 
	
    if(inpress_step_current_level > 11) 
	   inpress_step_current_level = 11;
	if(foot_step_current_level > 11)
	   foot_step_current_level = 11;
	if( inpress_port ==1 ) //��ѹ�ŵ���·
  	    step_cfg_data = inpress_step_configure_tab[inpress_step_current_level] + foot_step_configure_tab[foot_step_current_level]; 
	else//����·
	    step_cfg_data = inpress_step_configure_tab4[inpress_step_current_level] + foot_step_configure_tab4[foot_step_current_level];
	send_dsp2_command(0x11,step_cfg_data); 
	delay_ms(50);
	
	if(SUPPORT_CS3_FUN == 1)
	{
		step_cfg_data =0x5633;
		send_dsp3_command(0x11,step_cfg_data); 
		delay_ms(150);
	}

}

//--------------------------------------------------------------------------------------
//  Name:	     version_check                 
//  Parameters:	 None
//  Returns:	 None
//  Description: check the step soft version 
//--------------------------------------------------------------------------------------
void version_check(void)
{
		send_dsp1_command(0x0006,0x5555);
		if( recieve_x.word == 0x5555)
		{
			send_dsp1_command(0x0001,0x5555);
		}	
		stepversion1 = recieve_x.word;
		
		send_dsp2_command(0x0006,0x5555);
		if( recieve_x.word == 0x5555)
		{
			send_dsp2_command(0x0001,0x5555);
		}	
		stepversion2 = recieve_x.word;	
		
#if MULTIPULE_IO_ENABLE == 1
		send_dsp3_command(0x0006,0x5555);
		if( recieve_x.word == 0x5555)
		{
			send_dsp3_command(0x0001,0x5555);
		}	
		stepversion3 = recieve_x.word;
		
		send_dsp4_command(0x0006,0x5555);
		if( recieve_x.word == 0x5555)
		{
			send_dsp4_command(0x0001,0x5555);
		}	
		stepversion4 = recieve_x.word;
#else		
		if(SUPPORT_CS3_FUN == 1)
		{ 
			send_dsp3_command(0x0001,0x5555);
			stepversion3 = recieve_x.word;
		}
#endif

}

#if Y_COMPENSATION
int process_y_compensation(int xydata)
{
	int retdata;
	retdata = xydata;
	if( xydata != 0 )//�����ǵ�ǰ��λ�Ʋ�Ϊ0
	{
	     if( last_direction == 0 )//û�в�����
	     {
		     if ( xydata > 0)
		          last_direction = 1;
			 else
			      last_direction = 2;
		  }
		  else
		  {
			  if( (last_direction == 1)&&(xydata < 0) )//ǰһ�������򣬵�ǰ�Ƿ���
			  {
				  last_direction = 2;
				  retdata = xydata- y_gear_compensation;
			  }
			  else if( (last_direction == 2)&&(xydata > 0) )//�����
			  {
			      last_direction = 1;
				  retdata = xydata + y_gear_compensation;	   
			  }
		  }
    }
	return retdata;
}
#endif
#if X_COMPENSATION
INT32 process_x_compensation(INT32 xydata)
{
	INT32 retdata;
	retdata = xydata;
	if( xydata != 0 )//�����ǵ�ǰ��λ�Ʋ�Ϊ0
	{
	     if( last_x_direction == 0 )//û�в�����
	     {
		     if ( xydata > 0)
		          last_x_direction = 1;
			 else
			      last_x_direction = 2;
		  }
		  else
		  {
			  if( (last_x_direction == 1)&&(xydata < 0) )//ǰһ�������򣬵�ǰ�Ƿ���
			  {
				  last_x_direction = 2;
				  retdata = xydata- x_gear_compensation;
			  }
			  else if( (last_x_direction == 2)&&(xydata > 0) )//�����
			  {
			      last_x_direction = 1;
				  retdata = xydata + x_gear_compensation;	   
			  }
		  }
    }
	return retdata;
}
#endif


#if NEW_CONTROL_PROTOCOL

void movestep_x_y_both(int x_data,int y_data)
{
	while(spi_flag > 0);
		 
	if( (x_data !=0) || (y_data !=0) )
	{
	    SPISTE1 = 0;
		SPISTE2 = 1;
		SPISTE3 = 1;
	}
	else
		return;
		
	spi_flag=1;
	
	if( x_data >= 0)
	{
		if( x_motor_dir == 0)			
		    trans_x.word=(UINT16)0x0000+((UINT16)x_data<<6)+(UINT16)timer_x;
		else
			trans_x.word=(UINT16)0x4000+((UINT16)x_data<<6)+(UINT16)timer_x;	
	}
	else
	{
		x_data_nf = -x_data;
		if( x_motor_dir == 0)	
			trans_x.word=(UINT16)0x4000+((UINT16)x_data_nf<<6)+(UINT16)timer_x;
		else
			trans_x.word=(UINT16)0x0000+((UINT16)x_data_nf<<6)+(UINT16)timer_x;
	}
	
	if(y_data >= 0)
	{
		if( y_motor_dir == 0)
		    trans_y.word=(UINT16)0xc000+((UINT16)y_data<<6)+(UINT16)timer_y;
		else
		    trans_y.word=(UINT16)0x8000+((UINT16)y_data<<6)+(UINT16)timer_y;
	}
	else
	{
		y_data_nf=-y_data;
		if( y_motor_dir == 0)
		    trans_y.word=(UINT16)0x8000+((UINT16)y_data_nf<<6)+(UINT16)timer_y;
		else
		    trans_y.word=(UINT16)0xc000+((UINT16)y_data_nf<<6)+(UINT16)timer_y;
	}
	trans_z.word = 0x6600 | (motor.spd_obj/100);
	dsp1 = 1;
	SPISTE1 = 0;                                                                   
	s4trr=trans_x.byte.byte1;                                                 
		
}
	
#endif
//--------------------------------------------------------------------------------------
//  Name:	     movestep_x
//  Parameters:	 None
//  Returns:	 None
//  Description: move x step motor 
//--------------------------------------------------------------------------------------
#if SUPPORT_UNIFY_DRIVER
void movestep_x2(int x_data)
{ 
	
	while(spi_flag > 0);
	if( (sys.status == ERROR) || (x_data == 0) )//�����0���ݲ���������
	 	return;
		
	if( timer_x > fabsm(x_data)*20 )
	 	timer_x = fabsm(x_data)*20;
	if( timer_x >63)
	    timer_x = 63;
		
	SPISTE1 = 0;
	SPISTE2 = 1;
	SPISTE3 = 1;
	SPISTE4 = 1;
	spi_flag=1;	
	
	if(x_data>0)
	{   
		if( x_motor_dir == 0)			
		   trans_x.word=(UINT16)0x0000+((UINT16)x_data<<6)+(UINT16)timer_x;
		else
		   trans_x.word=(UINT16)0x4000+((UINT16)x_data<<6)+(UINT16)timer_x;	   
	}
	else	
	{
		x_data_nf=-x_data;
		if( x_motor_dir == 0)	
		   trans_x.word=(UINT16)0x4000+((UINT16)x_data_nf<<6)+(UINT16)timer_x;		// set transmit data 
		else
		   trans_x.word=(UINT16)0x0000+((UINT16)x_data_nf<<6)+(UINT16)timer_x;		// set transmit data 
	}
	if( trans_x.word == 0x5555 )
	{
		trans_x.word = trans_x.word + 1;
	}
	trans_y.word=(~trans_x.word)&0x7fff;
	
	trans_z.word = 0x5555;
	dsp1 = 1;
	SPISTE1=0;                                                            // DSP1 SPI enable 
	s4trr=trans_x.byte.byte1;                                             // transmit data
	
}
void movestep_x3(int x_data)
{
	while(spi_flag > 0);
	if(sys.status == ERROR)//�����0���ݲ���������
	 	return;
		
	if( timer_x >63)
	    timer_x = 63;
		
	SPISTE1 = 0;
	SPISTE2 = 1;
	SPISTE3 = 1;
	SPISTE4 = 1;
	spi_flag=1;	
	
	if(x_data>0)
	{   
		if( x_motor_dir == 0)			
		   trans_x.word=(UINT16)0x0000+((UINT16)x_data<<6)+(UINT16)timer_x;
		else
		   trans_x.word=(UINT16)0x4000+((UINT16)x_data<<6)+(UINT16)timer_x;	   
	}
	else	
	{
		x_data_nf=-x_data;
		if( x_motor_dir == 0)	
		   trans_x.word=(UINT16)0x4000+((UINT16)x_data_nf<<6)+(UINT16)timer_x;		// set transmit data 
		else
		   trans_x.word=(UINT16)0x0000+((UINT16)x_data_nf<<6)+(UINT16)timer_x;		// set transmit data 
	}
	if( trans_x.word == 0x5555 )
	{
		trans_x.word = trans_x.word + 1;
	}
	trans_y.word=(~trans_x.word)&0x7fff;
	
	trans_z.word = 0x5555;
	dsp1 = 1;
	SPISTE1=0;                                                            // DSP1 SPI enable 
	s4trr=trans_x.byte.byte1;                                             // transmit data
	
}

#else
void movestep_x2(int x_data)
{ 
	while(spi_flag > 0);

	if( timer_x >63)
	    timer_x = 63;
	if( x_data !=0)
	{
	    SPISTE1 = 0;
		SPISTE2 = 1;
		SPISTE3 = 1;
		SPISTE4 = 1;
	}
	if(x_data>0)
	{   
		spi_flag=1;			
		if( (x_data <6)&&(timer_x >20) )
		   timer_x =20;
		   
		if( x_motor_dir == 0)			
		   trans_x.word=(UINT16)0x0000+((UINT16)x_data<<6)+(UINT16)timer_x;
		else
		   trans_x.word=(UINT16)0x4000+((UINT16)x_data<<6)+(UINT16)timer_x;	   
		
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		{
			trans_x.word = trans_x.word + 1;
		}
		   
		trans_y.word=(~trans_x.word)&0x7fff;
		#if SUPPORT_NEW_DRIVER
		//trans_z.word = 0x5500 | (motor.spd_obj/100);
		if( double_xy_time_flag )
		    trans_z.word = 0x5566;
		else
			trans_z.word = 0x5555;
		#else
			if(step_movetype == 1)
			   trans_z.word = 0x55aa;
			else
			#if ENABLE_SEND_STEPPER_SPEED
				trans_z.word = 0x5500 | (motor.spd_obj/100);
			#else
				trans_z.word = 0x5555;
			#endif 
		#endif		
		dsp1 = 1;
		SPISTE1=0;                                                            // DSP1 SPI enable 
		s4trr=trans_x.byte.byte1;                                             // transmit data
	}	
	else if(x_data<0)
	{
		x_data_nf=-x_data;
		if( (x_data_nf < 6)&&(timer_x>20) )
		  timer_x = 20;
		spi_flag=1;	
		if( x_motor_dir == 0)	
		   trans_x.word=(UINT16)0x4000+((UINT16)x_data_nf<<6)+(UINT16)timer_x;		// set transmit data 
		else
		   trans_x.word=(UINT16)0x0000+((UINT16)x_data_nf<<6)+(UINT16)timer_x;		// set transmit data 
        if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		{
			trans_x.word = trans_x.word + 1;
		}		   
		trans_y.word=(~trans_x.word)&0x7fff;
		#if SUPPORT_NEW_DRIVER	

		//trans_z.word = 0x5500 | (motor.spd_obj/100);
		if( double_xy_time_flag )
		    trans_z.word = 0x5566;
		else
			trans_z.word = 0x5555;
	
		#else
				if(step_movetype == 1)
				   trans_z.word = 0x55aa;
				else
			#if ENABLE_SEND_STEPPER_SPEED
				trans_z.word = 0x5500 | (motor.spd_obj/100);
			#else
				trans_z.word = 0x5555;
			#endif 
		#endif
		dsp1 = 1;
		SPISTE1=0;		                                                        // DSP1 SPI enable         
		s4trr=trans_x.byte.byte1;                                             // transmit data      
	}
}
#endif

void movestep_x(int x_data)
{ 
	#if X_COMPENSATION
	int xtmp;
    UINT16 quick_time,tmp16;
	if( timer_x != 0 )//����ԭ��
		xtmp = process_x_compensation(x_data);
	else
	    xtmp = x_data;
	tmp16 = fabsm(xtmp);	
	   
	if( tmp16 < 255)
	{
		movestep_x2(xtmp);
	}
	else
	{
		quick_time = Calculate_QuickMove_Time(temp16,0);
		quickmove_x_process(quick_time,xtmp);
	}
	#else
		movestep_x2(x_data);
	#endif
}
//--------------------------------------------------------------------------------------
//  Name:	     movestep_y
//  Parameters:	 None
//  Returns:	 None
//  Description: move y step motor 
//--------------------------------------------------------------------------------------
#if SUPPORT_UNIFY_DRIVER && DOUBLE_X_60MOTOR
void movestep_y2(int y_data)
{  
	while(spi_flag > 0);
	if( (sys.status == ERROR) || (y_data == 0) )
	 	return;
	if( timer_y > fabsm(y_data)*20)
		timer_y = fabsm(y_data)*20;
	if( timer_y >63)
	    timer_y = 63;
		
	SPISTE1 = 1;
	SPISTE2 = 0;
	SPISTE3 = 1;
	SPISTE4 = 1;
	spi_flag =1;
	
	if(y_data>0)
	{
		if( y_motor_dir == 0)
		    trans_x.word=(UINT16)0x4000+((UINT16)y_data<<6)+(UINT16)timer_y;
		else
		    trans_x.word=(UINT16)0x0000+((UINT16)y_data<<6)+(UINT16)timer_y;
	}
	else
	{
		y_data_nf =- y_data;
		if( y_motor_dir == 0)
		    trans_x.word=(UINT16)0x0000+((UINT16)y_data_nf<<6)+(UINT16)timer_y;
		else
		    trans_x.word=(UINT16)0x4000+((UINT16)y_data_nf<<6)+(UINT16)timer_y;
	}
	if( trans_x.word == 0x5555 )
		trans_x.word = trans_x.word + 1;
	trans_y.word=((~trans_x.word)&0x7fff);
	trans_z.word = 0x5555;
	dsp2 = 1;
	SPISTE2 = 0;         
	s4trr=trans_x.byte.byte1;    

}
#elif SUPPORT_UNIFY_DRIVER 
void movestep_y2(int y_data)
{  
	while(spi_flag > 0);
	if( (sys.status == ERROR) || (y_data == 0) )
	 	return;
	if( timer_y > fabsm(y_data)*20)
		timer_y = fabsm(y_data)*20;
	if( timer_y >63)
	    timer_y = 63;
		
	SPISTE1 = 0;
	SPISTE2 = 1;
	SPISTE3 = 1;
	SPISTE4 = 1;
	spi_flag =1;
	
	if(y_data>0)
	{
		if( y_motor_dir == 0)
		    trans_x.word=(UINT16)0xc000+((UINT16)y_data<<6)+(UINT16)timer_y;
		else
		    trans_x.word=(UINT16)0x8000+((UINT16)y_data<<6)+(UINT16)timer_y;
	}
	else
	{
		y_data_nf =- y_data;
		if( y_motor_dir == 0)
		    trans_x.word=(UINT16)0x8000+((UINT16)y_data_nf<<6)+(UINT16)timer_y;
		else
		    trans_x.word=(UINT16)0xc000+((UINT16)y_data_nf<<6)+(UINT16)timer_y;
	}
	if( trans_x.word == 0x5555 )
		trans_x.word = trans_x.word + 1;
	trans_y.word=((~trans_x.word)&0x7fff);
	trans_z.word = 0x5555;
	dsp1 = 1;
	SPISTE1 = 0;         
	s4trr=trans_x.byte.byte1;    

}	
void movestep_y3(int y_data)
{
	while(spi_flag > 0);
	if(sys.status == ERROR) 
	 	return;

	if( timer_y >63)
	    timer_y = 63;
		
	SPISTE1 = 0;
	SPISTE2 = 1;
	SPISTE3 = 1;
	SPISTE4 = 1;
	spi_flag =1;
	
	if(y_data>0)
	{
		if( y_motor_dir == 0)
		    trans_x.word=(UINT16)0xc000+((UINT16)y_data<<6)+(UINT16)timer_y;
		else
		    trans_x.word=(UINT16)0x8000+((UINT16)y_data<<6)+(UINT16)timer_y;
	}
	else
	{
		y_data_nf =- y_data;
		if( y_motor_dir == 0)
		    trans_x.word=(UINT16)0x8000+((UINT16)y_data_nf<<6)+(UINT16)timer_y;
		else
		    trans_x.word=(UINT16)0xc000+((UINT16)y_data_nf<<6)+(UINT16)timer_y;
	}
	if( trans_x.word == 0x5555 )
		trans_x.word = trans_x.word + 1;
	trans_y.word=((~trans_x.word)&0x7fff);
	trans_z.word = 0x5555;
	dsp1 = 1;
	SPISTE1 = 0;         
	s4trr=trans_x.byte.byte1;    

}		
#elif DOUBLE_X_60MOTOR

void movestep_y2(int y_data)
{  	
	while(spi_flag > 0);
	if( timer_y >63)
	    timer_y = 63;
	if( y_data !=0 )
	{
	    SPISTE1 = 1;
		SPISTE2 = 0;
		SPISTE3 = 1;
	}

	if(y_data>0)
	{
		spi_flag=1;		
		if( (y_data < 6)&&(timer_y >20) )
		   timer_y = 20;
		   
		if( y_motor_dir == 0)
		    trans_x.word=(UINT16)0x4000+((UINT16)y_data<<6)+(UINT16)timer_y;
		else
		    trans_x.word=(UINT16)0x0000+((UINT16)y_data<<6)+(UINT16)timer_y;
		if( trans_x.word == 0x5555 )
			trans_x.word = trans_x.word + 1;
		trans_y.word=((~trans_x.word)&0x7fff);
		
		//trans_z.word = 0x5500 | (motor.spd_obj/100);
		if( double_xy_time_flag )
		    trans_z.word = 0x5566;
		else
			trans_z.word = 0x5555;
			
		dsp2 = 1;
		SPISTE2 = 0;         
		s4trr=trans_x.byte.byte1;    
	}	
	else if(y_data<0)
	{
		spi_flag=1;	
		y_data_nf=-y_data;		
		if( (y_data_nf <6)&&(timer_y>20) )
		  timer_y =20;

		if( y_motor_dir == 0)
		    trans_x.word=(UINT16)0x0000+((UINT16)y_data_nf<<6)+(UINT16)timer_y;
		else
		    trans_x.word=(UINT16)0x4000+((UINT16)y_data_nf<<6)+(UINT16)timer_y;
		if( trans_x.word == 0x5555 )
			trans_x.word = trans_x.word + 1;
		trans_y.word=(~trans_x.word)&0x7fff;
		
		//trans_z.word = 0x5500 | (motor.spd_obj/100);
		if( double_xy_time_flag )
		    trans_z.word = 0x5566;
		else
			trans_z.word = 0x5555;
			
		dsp2 = 1;
		SPISTE2 = 0;                                                                   
		s4trr=trans_x.byte.byte1;                                                 
	}
}

#else

void movestep_y2(int y_data)
{  	
	while(spi_flag > 0);
	if( timer_y >63)
	    timer_y = 63;
	if( y_data !=0 )
	{
	    SPISTE1 = 0;
		SPISTE2 = 1;
		SPISTE3 = 1;
		SPISTE4 = 1;
	}

	if(y_data>0)
	{
		spi_flag=1;
		
		if( (y_data < 6)&&(timer_y >20) )
		   timer_y = 20;
		   
		if( y_motor_dir == 0)
		    trans_x.word=(UINT16)0xc000+((UINT16)y_data<<6)+(UINT16)timer_y;	    // set transmit data  
		else
		    trans_x.word=(UINT16)0x8000+((UINT16)y_data<<6)+(UINT16)timer_y;	    // set transmit data  
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		{
			trans_x.word = trans_x.word + 1;
		}	
		trans_y.word=((~trans_x.word)&0x7fff);
		#if SUPPORT_NEW_DRIVER	
		trans_z.word = 0x5500 | (motor.spd_obj/100);
		#else
				if(step_movetype == 1)
				   trans_z.word = 0x55aa;
				else
			#if ENABLE_SEND_STEPPER_SPEED
				trans_z.word = 0x5500 | (motor.spd_obj/100);
			#else
				trans_z.word = 0x5555;
			#endif
		#endif
		dsp1 = 1;
		SPISTE1=0;                                                            // DSP1 SPI enable         
		s4trr=trans_x.byte.byte1;                                             // transmit data      
	}	
	else if(y_data<0)
	{
		y_data_nf =- y_data;
		
		if( (y_data_nf <6)&&(timer_y>20) )
		  timer_y =20;
		  
		spi_flag=1;	
		if( y_motor_dir == 0)
		   trans_x.word=(UINT16)0x8000+((UINT16)y_data_nf<<6)+(UINT16)timer_y;	  // set transmit data 
		else
		   trans_x.word=(UINT16)0xc000+((UINT16)y_data_nf<<6)+(UINT16)timer_y;	  // set transmit data 
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		{
			trans_x.word = trans_x.word + 1;
		}   
		trans_y.word=(~trans_x.word)&0x7fff;
	#if SUPPORT_NEW_DRIVER	
		trans_z.word = 0x5500 | (motor.spd_obj/100);
	#else
				if(step_movetype == 1)
				   trans_z.word = 0x55aa;
				else
			#if ENABLE_SEND_STEPPER_SPEED
				trans_z.word = 0x5500 | (motor.spd_obj/100);
			#else
				trans_z.word = 0x5555;
			#endif 
	#endif
		dsp1 = 1;
		SPISTE1=0;                                                                   
		s4trr=trans_x.byte.byte1;                                                 
	}
}
#endif 

void movestep_y(int y_data)
{  	
	#if Y_COMPENSATION
	int ytmp;
    UINT16 quick_time,tmp16;
	if( timer_y != 0 )//����ԭ��
		ytmp = process_y_compensation(y_data);
	else
	    ytmp = y_data;
	tmp16 = fabsm(ytmp);	
	   
	if( tmp16 < 255)
	{
		movestep_y2(ytmp);
	}
	else
	{
		//quick_time = Calculate_QuickMove_Time(tmp16);
		quick_time = Calculate_QuickMove_Time(0,temp16);
		quickmove_y_process(quick_time,ytmp);
	}
	#else
		movestep_y2(y_data);
	#endif
}

//--------------------------------------------------------------------------------------
//  Name:	     movestep_yj
//  Parameters:	 None
//  Returns:	 None
//  Description: move yj step motor 
//--------------------------------------------------------------------------------------
#if SUPPORT_UNIFY_DRIVER

void movestep_yj(int yj_data,UINT16 time)
{ 
	while(spi_flag > 0);
	if( time >63)
	    time = 63;	
		
	if( yj_data !=0)
	{
	    SPISTE2 = 0;
		SPISTE1 = 1;
		SPISTE3 = 1;
		SPISTE4 = 1;
	}
	if(yj_data>0)
	{
		spi_flag=1;
		trans_x.word=(UINT16)0xc000+((UINT16)yj_data<<6)+(UINT16)time;
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2 = 0;                                                              
		s4trr=trans_x.byte.byte1;                                               
	}	
	else if(yj_data<0)
	{
		yj_data_nf=-yj_data;
		spi_flag=1;		
		trans_x.word=(UINT16)0x8000+((UINT16)yj_data_nf<<6)+(UINT16)time;
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2 = 0;                                                                  
		s4trr=trans_x.byte.byte1;                                   
	}
}

#else

void movestep_yj(int yj_data,UINT16 time)
{ 
	while(spi_flag > 0);
	if( yj_data !=0)
	{
	    SPISTE2 = 0;
		SPISTE1 = 1;
		SPISTE3 = 1;
	}
	if(yj_data>0)
	{
		spi_flag=1;
		trans_x.word=(UINT16)0xc000+((UINT16)yj_data<<6)+(UINT16)time;	      // set transmit data  
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                                            // DSP2 SPI enable         
		s4trr=trans_x.byte.byte1;                                             // transmit data      
	}	
	else if(yj_data<0)
	{
		yj_data_nf=-yj_data;
		spi_flag=1;
		trans_x.word=(UINT16)0x8000+((UINT16)yj_data_nf<<6)+(UINT16)time;	    // set transmit data  
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                                            // DSP2 SPI enable         
		s4trr=trans_x.byte.byte1;                                             // transmit data      
	}
}
#endif
//=============
/**
  * @�������� ץ�ߵ���ײ㶯������
  * @����      
  *     @ct_data: ץ�ߵ������λ��
  *     @time: λ�ƶ�Ӧ��ʱ��
  *
  */
  
#if SUPPORT_UNIFY_DRIVER

void movestep_ct(int ct_data,UINT16 time)
{  		
	while(spi_flag > 0);
	
#if	ENABLE_LOOPER_CUTTER || DOUBLE_X_60MOTOR
    time = time <<3;
	movestep_cs3(0x4000,ct_data,time);
#else	
	if( ct_data !=0)
	{
	    SPISTE2=0;
		SPISTE1 = 1;
		SPISTE3 = 1;
		SPISTE4 = 1;
	}
	if(ct_data>0)
	{
		spi_flag=1;
		time = time <<3;
		if(time >63)
		   time = 63;
	    if( inpress_port == 1)//1,2,3
			trans_x.word=(UINT16)0xc000+((UINT16)ct_data<<6)+(UINT16)time;
		else
			trans_x.word=(UINT16)0x4000+((UINT16)ct_data<<6)+(UINT16)time;
	   
		if( trans_x.word == 0x5555 )
		    trans_x.word+= 1;
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                                                   
		s4trr=trans_x.byte.byte1;                                               
	}	                                                                      
	else if(ct_data<0)
	{                                                                       
		ct_data_nf=-ct_data;                                                  
		spi_flag=1;			
		time = time <<3;
		if(time >63)
		   time = 63;		
		if( inpress_port == 1)//1,2,3
		     trans_x.word=(UINT16)0x8000+((UINT16)ct_data_nf<<6)+(UINT16)time;
		else
		     trans_x.word=(UINT16)0x0000+((UINT16)ct_data_nf<<6)+(UINT16)time;

		if( trans_x.word == 0x5555 )
		    trans_x.word+= 1;
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                   
		s4trr=trans_x.byte.byte1;                                              
	}
#endif
}

#else

void movestep_ct(int ct_data,UINT16 time)
{  		
	while(spi_flag > 0);
	
#if	ENABLE_LOOPER_CUTTER || DOUBLE_X_60MOTOR
    time = time <<3;
	movestep_cs3(0x4000,ct_data,time);
#else	
	if( ct_data !=0)
	{
	    SPISTE2 = 0;
		SPISTE1 = 1;
		SPISTE3 = 1;
		SPISTE4 = 1;
	}
	if(ct_data>0)
	{
		spi_flag=1;
		#if SUPPORT_NEW_DRIVER
		time = time <<3;
		if(time >63)
		   time = 63;
	    if( inpress_port == 1)//1,2,3
			trans_x.word=(UINT16)0xc000+((UINT16)ct_data<<7)+(UINT16)time;
		else
			trans_x.word=(UINT16)0x4000+((UINT16)ct_data<<7)+(UINT16)time;
	   #else
			#if NEW_CLAMP_TYPE
			if( inpress_port == 1)//1,2,3
			   trans_x.word=(UINT16)0xc000+((UINT16)ct_data<<5)+(UINT16)time;
			else
			   trans_x.word=(UINT16)0x4000+((UINT16)ct_data<<5)+(UINT16)time;
			#else
			if( inpress_port == 1)//1,2,3
			   trans_x.word=(UINT16)0x8000+((UINT16)ct_data<<5)+(UINT16)time;
			else
			   trans_x.word=(UINT16)0x0000+((UINT16)ct_data<<5)+(UINT16)time;
		    #endif
		#endif
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		  trans_x.word+= 1;
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                                                   
		s4trr=trans_x.byte.byte1;                                               
	}	                                                                      
	else if(ct_data<0)
	{                                                                       
		ct_data_nf=-ct_data;                                                  
		spi_flag=1;			
		#if SUPPORT_NEW_DRIVER
		time = time <<3;
		if(time >63)
		   time = 63;		
		if( inpress_port == 1)//1,2,3
		     trans_x.word=(UINT16)0x8000+((UINT16)ct_data_nf<<7)+(UINT16)time;
		else
		     trans_x.word=(UINT16)0x0000+((UINT16)ct_data_nf<<7)+(UINT16)time;

		#else   
			#if NEW_CLAMP_TYPE
			if( inpress_port == 1)//1,2,3
			     trans_x.word=(UINT16)0x8000+((UINT16)ct_data_nf<<5)+(UINT16)time;
			else
			     trans_x.word=(UINT16)0x0000+((UINT16)ct_data_nf<<5)+(UINT16)time;
			#else
			if( inpress_port == 1)//1,2,3
			     trans_x.word=(UINT16)0xc000+((UINT16)ct_data_nf<<5)+(UINT16)time;
			else
			     trans_x.word=(UINT16)0x4000+((UINT16)ct_data_nf<<5)+(UINT16)time;
			#endif
		#endif
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		  trans_x.word+= 1;
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                   
		s4trr=trans_x.byte.byte1;                                              
	}
#endif
}
#endif


#if SUPPORT_UNIFY_DRIVER
void movestep_lct(int ct_data,UINT16 time)
{  		
	while(spi_flag > 0);
	if( ct_data !=0)
	{
	    SPISTE2 = 0;
		SPISTE1 = 1;
		SPISTE3 = 1;
		SPISTE4 = 1;
	}
	if(ct_data>0)
	{
		spi_flag=1;
		if(time >63)
		   time = 63;
		if( yj_motor_dir == 0)
		   trans_x.word=(UINT16)0x0000+((UINT16)ct_data<<6)+(UINT16)time;
		else
		   trans_x.word=(UINT16)0x4000+((UINT16)ct_data<<6)+(UINT16)time;
		if( trans_x.word == 0x5555 )
		    trans_x.word+= 1;
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                                                   
		s4trr=trans_x.byte.byte1;                                               
	}	                                                                      
	else if(ct_data<0)
	{                                                                       
		ct_data_nf=-ct_data;                                                  
		spi_flag=1;			
		if(time >63)
		   time = 63;
		if( yj_motor_dir == 0)//1,2,3
		     trans_x.word=(UINT16)0x4000+((UINT16)ct_data_nf<<6)+(UINT16)time;
		else
		     trans_x.word=(UINT16)0x0000+((UINT16)ct_data_nf<<6)+(UINT16)time;

		if( trans_x.word == 0x5555 )
		    trans_x.word+= 1;
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                   
		s4trr=trans_x.byte.byte1;                                              
	}
}
#else
void movestep_lct(int ct_data,UINT16 time)
{  		
	while(spi_flag > 0);
	if( ct_data !=0)
	{
	    SPISTE2 = 0;
		SPISTE1 = 1;
		SPISTE3 = 1;
		SPISTE4 = 1;
	}
	if(ct_data>0)
	{
		spi_flag=1;
		if(time >63)
		   time = 63;
		if( inpress_port == 1)//1,2,3
		   trans_x.word=(UINT16)0xc000+((UINT16)ct_data<<7)+(UINT16)time;
		else
		   trans_x.word=(UINT16)0x4000+((UINT16)ct_data<<7)+(UINT16)time;
		if( trans_x.word == 0x5555 )
		    trans_x.word+= 1;
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                                                   
		s4trr=trans_x.byte.byte1;                                               
	}	                                                                      
	else if(ct_data<0)
	{                                                                       
		ct_data_nf=-ct_data;                                                  
		spi_flag=1;			
		if(time >63)
		   time = 63;
		if( inpress_port == 1)//1,2,3
		     trans_x.word=(UINT16)0x8000+((UINT16)ct_data_nf<<7)+(UINT16)time;
		else
		     trans_x.word=(UINT16)0x0000+((UINT16)ct_data_nf<<7)+(UINT16)time;

		if( trans_x.word == 0x5555 )
		    trans_x.word+= 1;
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                   
		s4trr=trans_x.byte.byte1;                                              
	}
}
#endif

//--------------------------------------------------------------------------------------
//  Name:	     movestep_zx
//  Parameters:	 None
//  Returns:	 None
//  Description: move zx step motor 
//--------------------------------------------------------------------------------------
#if SUPPORT_UNIFY_DRIVER

void movestep_zx(int zx_data,UINT16 time)
{  		
	if( inpress_type == AIR_INPRESS) 
	    return;
	//if( (sys.status == ERROR) || (zx_data == 0) )
	if( zx_data == 0 )
	 	return;
	while(spi_flag > 0);	
 	  
	SPISTE2 = 0;
	SPISTE1 = 1;
	SPISTE3 = 1;
	SPISTE4 = 1;
	#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER40
	//if( para.rotate_cutter_working_mode == 55)	
		inpress_port = 1;	
	#endif
	if( fabsm(zx_data) < 127 )
	{	
		if( time > fabsm(zx_data)*20 )
	 	    time = fabsm(zx_data)*20;
		if( time > 63)
	   	    time = 63;
		if( zx_data > 0 )
		{
			if( inpress_port == 0)//1,2,4  X21
			{
				if( z_motor_dir== 1)
					trans_x.word=(UINT16)0xc000+((UINT16)zx_data<<6)+(UINT16)time;
				else
				    trans_x.word=(UINT16)0x8000+((UINT16)zx_data<<6)+(UINT16)time;
			}
			else
			{
				if( z_motor_dir== 1)
					trans_x.word=(UINT16)0x4000+((UINT16)zx_data<<6)+(UINT16)time;
				else
				    trans_x.word=(UINT16)0x0000+((UINT16)zx_data<<6)+(UINT16)time;
			}	
					
		}	                                                                      
		else if(zx_data<0)
		{                                                                       
			zx_data_nf=-zx_data;    
			if( inpress_port == 0)//1,2,4
			{
				if( z_motor_dir== 1)	                                                        
					trans_x.word=(UINT16)0x8000+((UINT16)zx_data_nf<<6)+(UINT16)time;
				else
					trans_x.word=(UINT16)0xc000+((UINT16)zx_data_nf<<6)+(UINT16)time;
			}
			else
			{
				if( z_motor_dir== 1)	                                                        
					trans_x.word=(UINT16)0x0000+((UINT16)zx_data_nf<<6)+(UINT16)time;
				else
					trans_x.word=(UINT16)0x4000+((UINT16)zx_data_nf<<6)+(UINT16)time;
			}
			
		}
		spi_flag=1;	
		if( trans_x.word == 0x5555 )
		    trans_x.word+= 1;
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2 = 0;                                                                   
		s4trr=trans_x.byte.byte1; 
	}
	else
	{
		quickmove_z_process(time,zx_data);
	}
	
	movezx_delay_counter = time ;
	movezx_delay_flag =1;
	
}

#else

void movestep_zx(int zx_data,UINT16 time)
{  		
	if( inpress_type == AIR_INPRESS) 
	    return;
	while(spi_flag > 0);
	if( (sys.status == ERROR) || (zx_data == 0) )
	 	return;

	if( zx_data !=0)
	{
	    SPISTE2 = 0;
		SPISTE1 = 1;
		SPISTE3 = 1;
		SPISTE4 = 1;
	}
		
	#if SUPPORT_NEW_DRIVER
	
	if( fabsm(zx_data) <= 127 )
	{
		
		if( time > 63)
	    	time = 63;
			
		if(zx_data>0)
		{
			if( inpress_port == 0)//1,2,4  X21
			{
				if( z_motor_dir== 1)
					trans_x.word=(UINT16)0xc000+((UINT16)zx_data<<7)+(UINT16)time;
				else
				    trans_x.word=(UINT16)0x8000+((UINT16)zx_data<<7)+(UINT16)time;
			}
			else
			{
				if( z_motor_dir== 1)
					trans_x.word=(UINT16)0x4000+((UINT16)zx_data<<7)+(UINT16)time;
				else
				    trans_x.word=(UINT16)0x0000+((UINT16)zx_data<<7)+(UINT16)time;
			}			
		}	                                                                      
		else if(zx_data<0)
		{                                                                       
			zx_data_nf=-zx_data;                                                  
			if( inpress_port == 0)//1,2,4
			{
				if( z_motor_dir== 1)	                                                        
					trans_x.word=(UINT16)0x8000+((UINT16)zx_data_nf<<7)+(UINT16)time;
				else
					trans_x.word=(UINT16)0xc000+((UINT16)zx_data_nf<<7)+(UINT16)time;
			}
			else
			{
				if( z_motor_dir== 1)	                                                        
					trans_x.word=(UINT16)0x0000+((UINT16)zx_data_nf<<7)+(UINT16)time;
				else
					trans_x.word=(UINT16)0x4000+((UINT16)zx_data_nf<<7)+(UINT16)time;
			}
		}
		spi_flag=1;	
		if( trans_x.word == 0x5555 )
		    trans_x.word+= 1;
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                                                   
		s4trr=trans_x.byte.byte1;                                               
	}
	else
	{

		send_dsp2_command(0x0000,0x0003);

		send_dsp_command(DSP2,time);		
		if(zx_data > 0)
		{	
			zx_data = zx_data<<1;
			if( inpress_port == 0)//1,2,4
			{
				if( z_motor_dir== 1)	                                                        
					trans_x.word=(UINT16)0x8000+(UINT16)zx_data;
				else
					trans_x.word=(UINT16)0xc000+(UINT16)zx_data;
			}
			else
			{
				if( z_motor_dir== 1)	                                                        
					trans_x.word=(UINT16)0x0000+(UINT16)zx_data;
				else
					trans_x.word=(UINT16)0x4000+(UINT16)zx_data;
			}
		}	
		else 
		{
			zx_data_nf=-zx_data; 
			zx_data_nf =  zx_data_nf<<1;
			if( inpress_port == 0)//1,2,4
			{
				if( z_motor_dir== 1)	                                                        
					trans_x.word=(UINT16)0x8000+(UINT16)zx_data_nf;
				else
					trans_x.word=(UINT16)0xc000+(UINT16)zx_data_nf;
			}
			else
			{
				if( z_motor_dir== 1)	                                                        
					trans_x.word=(UINT16)0x0000+(UINT16)zx_data_nf;
				else
					trans_x.word=(UINT16)0x4000+(UINT16)zx_data_nf;
			}  		
		}	
		spi_flag=1;	
		if( trans_x.word == 0x5555)
	        trans_x.word = trans_x.word + 1;	
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word= 0x5550 ;
		dsp2 = 1;
		SPISTE2=0;                                                           
		s4trr=trans_x.byte.byte1;
		delay_us(400);
		while( spi_flag > 0 );
	}
	#else
	if(zx_data>0)
	{
		spi_flag=1;
		if( inpress_port == 0)//1,2,4  X21
		{
			if( z_motor_dir== 0)
				trans_x.word=(UINT16)0xc000+((UINT16)zx_data<<5)+(UINT16)time;
			else
			    trans_x.word=(UINT16)0x8000+((UINT16)zx_data<<5)+(UINT16)time;
		}
		else
		{
			if( z_motor_dir== 0)
				trans_x.word=(UINT16)0x4000+((UINT16)zx_data<<5)+(UINT16)time;	      // set transmit data
			else
			    trans_x.word=(UINT16)0x0000+((UINT16)zx_data<<5)+(UINT16)time;
		}
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		  trans_x.word+= 1;
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                                                   
		s4trr=trans_x.byte.byte1;                                               
	}	                                                                      
	else if(zx_data<0)
	{                                                                       
		zx_data_nf=-zx_data;                                                  
		spi_flag=1;	
		if( inpress_port == 0)//1,2,4
		{
			if( z_motor_dir== 0)	                                                        
				trans_x.word=(UINT16)0x8000+((UINT16)zx_data_nf<<5)+(UINT16)time; 
			else
				trans_x.word=(UINT16)0xc000+((UINT16)zx_data_nf<<5)+(UINT16)time;
		}
		else
		{
			if( z_motor_dir== 0)	                                                        
				trans_x.word=(UINT16)0x0000+((UINT16)zx_data_nf<<5)+(UINT16)time;
			else
				trans_x.word=(UINT16)0x4000+((UINT16)zx_data_nf<<5)+(UINT16)time;
		}
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		   trans_x.word+= 1;
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                                                   
		s4trr=trans_x.byte.byte1;                                              
	}
	#endif
}

#endif

//--------------------------------------------------------------------------------------
//  Name:	     quickmove_x
//  Parameters:	 None
//  Returns:	 None
//  Description: quick move x step motor 
//--------------------------------------------------------------------------------------
void quickmove_x2(INT32 x_data)
{ 
    UINT16 h_data; 	
	while(spi_flag > 0);
	
	if(x_data != 0)
	{
    	SPISTE1 = 0; 
		SPISTE2 = 1;
		SPISTE3 = 1;
		SPISTE4 = 1;
	}
	h_data = 0;

	if(x_data>0)
	{
		spi_flag=1;	
		debug_para2 = x_data;
		h_data = (x_data & 0xc000) >>14;
		x_data = x_data &0x3fff;
		debug_para1 = h_data;
		
	    if( x_motor_dir == 0)					
		   trans_x.word=(UINT16)0x0000+((UINT16)x_data);	    
		else
		   trans_x.word=(UINT16)0x4000+((UINT16)x_data);	    
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		{
			trans_x.word = trans_x.word + 1;
		}   
		trans_y.word=(~trans_x.word)&0x7fff;
		
		trans_z.word= (0x5550 | h_data);
		dsp1 = 1;
		SPISTE1=0;                                                            // DSP1 SPI enable 
		s4trr=trans_x.byte.byte1;                                             // transmit data
	}	
	else if(x_data<0)
	{
		x_data_nf = -x_data;
		debug_para2 = -x_data;
		h_data = (x_data_nf & 0xc000) >>14;
		x_data_nf = x_data_nf &0x3fff;

		debug_para1 = h_data;
		
		spi_flag=1;	
		if( x_motor_dir == 0)
		   trans_x.word=(UINT16)0x4000+((UINT16)x_data_nf);		// set transmit data  
		else
		   trans_x.word=(UINT16)0x0000+((UINT16)x_data_nf);		// set transmit data  
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		{
			trans_x.word = trans_x.word + 1;
		}
		trans_y.word=(~trans_x.word)&0x7fff;

		trans_z.word= (0x5550 | h_data);

		dsp1 = 1;
		SPISTE1=0;		                                                        // DSP1 SPI enable         
		s4trr=trans_x.byte.byte1;                                             // transmit data      
	}	
}

void quickmove_x(INT32 x_data)
{ 
#if X_COMPENSATION
	INT32 xtmp;
	xtmp = process_x_compensation(x_data);
	quickmove_x2(xtmp);	
#else
    quickmove_x2(x_data);
#endif 
}
//--------------------------------------------------------------------------------------
//  Name:	     quickmove_y
//  Parameters:	 None
//  Returns:	 None
//  Description: quick move y step motor 
//--------------------------------------------------------------------------------------
#if DOUBLE_X_60MOTOR
void quickmove_y2(int y_data)
{  	
    UINT16 h_data; 
	while(spi_flag > 0);
	if( y_data >0 )
	{	  
	   SPISTE1 = 1;
	   SPISTE2 = 0;
	   SPISTE3 = 1;
	}
	if(y_data>0)
	{
		spi_flag=1;
		
		h_data = (y_data & 0x1c000) >>14;
		y_data = y_data &0x3fff;
		
		if( y_motor_dir == 0)
		   trans_x.word=(UINT16)0x4000+((UINT16)y_data);	    // set transmit data  
		else
		   trans_x.word=(UINT16)0x0000+((UINT16)y_data);	    // set transmit data  
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		{
			trans_x.word = trans_x.word + 1;
		}   
		trans_y.word=((~trans_x.word)&0x7fff);
		trans_z.word= (0x5550 | h_data);
		dsp2 = 1;
		SPISTE2=0;                                                            // DSP1 SPI enable         
		s4trr=trans_x.byte.byte1;                                             // transmit data      
	}	
	else if(y_data<0)
	{
		y_data_nf=-y_data;
		spi_flag=1;
		h_data = (y_data_nf & 0x1c000) >>14;
		y_data_nf = y_data_nf &0x3fff;
			
		if( y_motor_dir == 0)
		   trans_x.word=(UINT16)0x0000+((UINT16)y_data_nf);	  // set transmit data  
		else
		   trans_x.word=(UINT16)0x4000+((UINT16)y_data_nf);	  // set transmit data  
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		{
			trans_x.word = trans_x.word + 1;
		}   
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word= (0x5550 | h_data);
		dsp2 = 1;
		SPISTE2=0;                                                            // DSP1 SPI enable         
		s4trr=trans_x.byte.byte1;                                             // transmit data      
	}
}
#else
void quickmove_y2(int y_data)
{  	
    while(spi_flag > 0);
	if( y_data >0 )
	{
	  
	   SPISTE1 = 0;
	   SPISTE2 = 1;
	   SPISTE3 = 1;
	   SPISTE4 = 1;
	}
	if(y_data>0)
	{
		spi_flag=1;
		if( y_motor_dir == 0)
		   trans_x.word=(UINT16)0xc000+((UINT16)y_data);	    // set transmit data  
		else
		   trans_x.word=(UINT16)0x8000+((UINT16)y_data);	    // set transmit data  
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		{
			trans_x.word = trans_x.word + 1;
		}   
		trans_y.word=((~trans_x.word)&0x7fff);
		trans_z.word=0x5555;
		dsp1 = 1;
		SPISTE1=0;                                                            // DSP1 SPI enable         
		s4trr=trans_x.byte.byte1;                                             // transmit data      
	}	
	else if(y_data<0)
	{
		y_data_nf=-y_data;
		spi_flag=1;
		if( y_motor_dir == 0)
		   trans_x.word=(UINT16)0x8000+((UINT16)y_data_nf);	  // set transmit data  
		else
		   trans_x.word=(UINT16)0xc000+((UINT16)y_data_nf);	  // set transmit data  
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		{
			trans_x.word = trans_x.word + 1;
		}   
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;	
		dsp1 = 1;
		SPISTE1=0;                                                            // DSP1 SPI enable         
		s4trr=trans_x.byte.byte1;                                             // transmit data      
	}
}
#endif

void quickmove_y(int y_data)
{  	
#if Y_COMPENSATION
	INT32 ytmp;
	ytmp = process_y_compensation(y_data);
	quickmove_y2(ytmp);	
#else
    quickmove_y2(y_data);
#endif 
}

//==============================================================
#if SUPPORT_UNIFY_DRIVER
UINT8 check_yj_done(void)
{
	send_dsp2_command(0x0012,0x5555);		
	if( (recieve_x.word&0xff00) == 0x1200 )
	    return(1);
	else
	    return(0);
}
/*
�������32λ�����2λ��������ͷ��򣬹���30λ������
*/
void quickmove_yj_process(INT32 quick_time, INT32 tempx_step)
{
	UINT16 low16,high16;
	UINT32 tmp32;
	send_dsp_command(2,0x0000);
	
	delay_us(500);	
	if( tempx_step > 0)
	{
		tmp32 = tempx_step;
		low16  = tmp32 & 0xffff;		
		if( yj_motor_dir == 0)	
		    high16 = (UINT16)0x0000 + ((tmp32>>16)&0xff);
		else
			high16 = (UINT16)0x4000 + ((tmp32>>16)&0xff);
	}
	else
	{
		tmp32 = -tempx_step;
		low16  = tmp32 & 0xffff;		
		if( yj_motor_dir == 0)		
		    high16 = (UINT16)0x4000 + ((tmp32>>16)&0xff);
		else
			high16 = (UINT16)0x0000 + ((tmp32>>16)&0xff);
	}
	#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER40
	high16 += 0x8000;
	#endif
	
	send_dsp_command(2,high16);	
	delay_ms(1);
	if( low16 == 0x5555)
	    low16 = low16 +1;
	send_dsp_command(2,low16);	
	delay_ms(1);
	if( quick_time == 0x5555)
	    quick_time = quick_time +1;
	send_dsp_command(2,quick_time);
	delay_ms(1);
}

void quickmove_x_process(INT32 quick_time, INT32 tempx_step)
{
	UINT16 low16,high16;
	UINT32 tmp32;
	send_dsp_command(1,0x0000);
	
	delay_us(500);	
	if( tempx_step > 0)
	{
		tmp32 = tempx_step;
		low16  = tmp32 & 0xffff;
		if( x_motor_dir == 0)	
		    high16 = (UINT16)0x0000 + ((tmp32>>16)&0xff);
		else
			high16 = (UINT16)0x4000 + ((tmp32>>16)&0xff);
	}
	else
	{
		tmp32 = -tempx_step;
		low16  = tmp32 & 0xffff;		
		if( x_motor_dir == 0)		
		    high16 = (UINT16)0x4000 + ((tmp32>>16)&0xff);
		else
			high16 = (UINT16)0x0000 + ((tmp32>>16)&0xff);
	}
	//if(fabsm(tempx_step)<(quick_time>>3))
	//{
	//	high16 |= (1<<13);
	//}
	send_dsp_command(1,high16);	
	delay_ms(1);
	if( low16 == 0x5555)
	    low16 = low16 +1;
	send_dsp_command(1,low16);	
	delay_ms(1);
	if( quick_time == 0x5555)
	    quick_time = quick_time +1;
	send_dsp_command(1,quick_time);
	delay_ms(1);
	
	#if 0
	printf_uart("high16=%d",high16);
	printf_uart("low16=%d",low16);
	printf_uart("tempx_step=%d",tempx_step);
	printf_uart("quick_time=%d",quick_time);
	#endif
}	

void quickmove_z_process(INT32 quick_time, INT32 tempx_step)
{
	UINT16 low16,high16;
	UINT32 tmp32;
	send_dsp_command(2,0x0000);	
	delay_us(500);	
	if( tempx_step > 0)
	{
		tmp32 = tempx_step;
		low16  = tmp32 & 0xffff;
		#if COMPILE_MACHINE_TYPE != MACHINE_CONFIG_NUMBER40
		if( inpress_port == 0)//1,2,4  X21
		{
			if( z_motor_dir == 0)	
			    high16 = (UINT16)0x8000 + ((tmp32>>16)&0xff);
			else
				high16 = (UINT16)0xc000 + ((tmp32>>16)&0xff);
		}
		else
		#endif
		{
			if( z_motor_dir == 0)	
			    high16 = (UINT16)0x0000 + ((tmp32>>16)&0xff);
			else
				high16 = (UINT16)0x4000 + ((tmp32>>16)&0xff);
		}
		
	}
	else
	{
		tmp32 = -tempx_step;
		low16  = tmp32 & 0xffff;
		#if COMPILE_MACHINE_TYPE != MACHINE_CONFIG_NUMBER40	
		if( inpress_port == 0)//1,2,4  X21
		{	
			if( z_motor_dir == 0)		
			    high16 = (UINT16)0xc000 + ((tmp32>>16)&0xff);
			else
				high16 = (UINT16)0x8000 + ((tmp32>>16)&0xff);
		}
		else
		#endif
		{
			if( z_motor_dir == 0)		
			    high16 = (UINT16)0x4000 + ((tmp32>>16)&0xff);
			else
				high16 = (UINT16)0x0000 + ((tmp32>>16)&0xff);
		}
		
	}
	high16 |= (1<<13);
	send_dsp_command(2,high16);	
	delay_ms(1);
	if( low16 == 0x5555)
	    low16 = low16 +1;
	send_dsp_command(2,low16);	
	delay_ms(1);
	if( quick_time == 0x5555)
	    quick_time = quick_time +1;
	send_dsp_command(2,quick_time);
	delay_ms(1);
}



#if DOUBLE_X_60MOTOR
void quickmove_y_process(INT32 quick_time, INT32 tempy_step)
{
	UINT16 low16,high16;
	UINT32 tmp32;
	send_dsp_command(2,0x0000);	

	delay_us(500);	
	if( tempy_step > 0)
	{
		tmp32 = tempy_step;
		low16  = tmp32 & 0xffff;
		if( y_motor_dir == 0)
		{	
		    high16 = 0x4000;
			high16 += (tmp32>>16)&0xff;
		}
		else
		{
			high16 = 0x0000; 
			high16 += (tmp32>>16)&0xff;
		}
	}
	else
	{
		tmp32 = -tempy_step;
		low16  = tmp32 & 0xffff;		
		if( y_motor_dir == 0)
		{		
		    high16 = 0x0000; 
			high16 += (tmp32>>16)&0xff;
		}
		else
		{
			high16 = 0x4000;
			high16 += (tmp32>>16)&0xff;
		}
	}
	//if(fabsm(tempy_step)<(quick_time>>3))
	//{
	//	high16 |= (1<<13);
	//}
	send_dsp_command(2,high16);	
	delay_ms(1);
	if( low16 == 0x5555)
	    low16 = low16 +1;
	send_dsp_command(2,low16);	
	delay_ms(1);
	if( quick_time == 0x5555)
	    quick_time = quick_time +1;
	send_dsp_command(2,quick_time);
	delay_ms(1);
	
	#if 0
	printf_uart("yhigh16=%d",high16);
	printf_uart("ylow16=%d",low16);
	printf_uart("tempy_step=%d",tempy_step);
	printf_uart("yquick_time=%d",quick_time);
	#endif
}
#else
void quickmove_y_process(INT32 quick_time, INT32 tempy_step)
{
	UINT16 low16,high16;
	UINT32 tmp32;
	send_dsp_command(1,0x0000);	

	delay_us(500);	
	if( tempy_step > 0)
	{
		tmp32 = tempy_step;
		low16  = tmp32 & 0xffff;
		if( y_motor_dir == 0)
		{	
		    high16 = 0xc000;
			high16 += (tmp32>>16)&0xff;
		}
		else
		{
			high16 = 0x8000; 
			high16 += (tmp32>>16)&0xff;
		}
	}
	else
	{
		tmp32 = -tempy_step;
		low16  = tmp32 & 0xffff;		
		if( y_motor_dir == 0)
		{		
		    high16 = 0x8000; 
			high16 += (tmp32>>16)&0xff;
		}
		else
		{
			high16 = 0xc000;
			high16 += (tmp32>>16)&0xff;
		}
	}
	//if(fabsm(tempy_step)<(quick_time>>3))
	//{
	//	high16 |= (1<<13);
	//}
	send_dsp_command(1,high16);	
	delay_ms(1);
	if( low16 == 0x5555)
	    low16 = low16 +1;
	send_dsp_command(1,low16);	
	delay_ms(1);
	if( quick_time == 0x5555)
	    quick_time = quick_time +1;
	send_dsp_command(1,quick_time);
	delay_ms(1);
	
	#if 0
	printf_uart("yhigh16=%d",high16);
	printf_uart("ylow16=%d",low16);
	printf_uart("tempy_step=%d",tempy_step);
	printf_uart("yquick_time=%d",quick_time);
	#endif
}
#endif


#else
void quickmove_x_process(INT32 time, INT32 sdata)
{
	send_dsp1_command(0x0000,0x0003);
	if( (time == 0x5555)||(time == 0x55aa))	
	     time += 1;
	send_dsp_command(DSP1,time); 
   	quickmove_x(sdata);
}	

void quickmove_y_process(INT32 time, INT32 data)
{
	#if DOUBLE_X_60MOTOR
	send_dsp2_command(0x0000,0x0003);
	if( (time == 0x5555)||(time == 0x55aa))	
	     time += 1;
	send_dsp_command(DSP2,time); 
   	quickmove_y(data);
	#else
	send_dsp1_command(0x0000,0x0003);
	if( (time == 0x5555)||(time == 0x55aa))	
	     time += 1;
	send_dsp_command(DSP1,time); 
	quickmove_y(data);
	#endif	
}
#endif
//======================================================
UINT16 get_x_distance(void)
{
	send_dsp1_command(0x0007,0x5555);
	return recieve_x.word;
}

UINT16 get_y_distance(void)
{
#if DOUBLE_X_60MOTOR
    send_dsp2_command(0x0007,0x5555);
	return recieve_x.word;
#else
    send_dsp1_command(0x0008,0x5555);
	return recieve_x.word;
#endif	
}

void nop_move_emergency(UINT16 x, UINT16 y)
{		
	while(spi_flag > 0);
	if( x > 255)
	{
		send_dsp_command(DSP1,0x0005);
	}
	while(spi_flag > 0);
	if( y > 255 )
	{
	#if DOUBLE_X_60MOTOR
		send_dsp_command(DSP2,0x0005);
	#else
		send_dsp_command(DSP1,0x0009);
	#endif		
	}
}


/*
һ����ת���
һ����������
һ��ԭ�����
һ���е��������
2016-4-28 ���е��ṹ���򣬲����ȳݽṹ��������²�������ת�����Բ��û�ת�����ˡ�����������λ����0�Ƚ�
*/

void go_origin_rotated_cutter(void)
{
	UINT8 i;
    UINT16 run_counter;
	
	rotated_cutter_position = 0;
	sum_rotated_angle = 0;
	cutter_motoer_remain_angle = 0;
	last_rotated_dir = 0;
	cutter_syn_counter = 0;
	
	if(sys.status == ERROR)
	   return;
	if( ROTATED_CUTTER_ORG) //��ס���˳���
	{
		run_counter = 0;
		while(ROTATED_CUTTER_ORG)						
		{
		    movestep_cs3(0x4000,1,3);//1.0 ��������ת(˳ʱ�룩���˳�����
			run_counter++;
			if(run_counter > 800)//
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				if( sys.error == 0)
      			    sys.error = ERROR_68;
      			return;
			}
			delay_ms(3);	
		}
		for(i=0;i<4;i--)
		{
			movestep_cs3(0x4000,1,3);
			delay_ms(3);
		}
	}
	if(!ROTATED_CUTTER_ORG)//������û���ϣ�Ҫ��������ԭ��
	{
		  run_counter = 0;
		  for(i=0;i<3;i++)
		  {
				while(!ROTATED_CUTTER_ORG)						
				{
					movestep_cs3(0x4000,-1,3);   
					run_counter++;
					if(run_counter > 800)
					{
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
						if( sys.error == 0)
		      			    sys.error = ERROR_68;
		      			return;
					}
					delay_ms(3);	
				}
			    delay_ms(20);
		  }
	}
	delay_ms(5);
	if( cutter_motor_initial_angle != 0 )
	{
		movestep_cs3(0x4000,cutter_motor_initial_angle,31);  
		delay_ms(50);
	}
    rotated_cutter_position = 0;//cutter_motor_initial_angle;//2016-6-22
	sum_rotated_angle = 0;
	cutter_motoer_remain_angle = 0;
	last_rotated_dir = 0;
	cutter_syn_counter = 0;
	dsp3 = 0;
	SPISTE3=1;
}

#if ENABLE_LOOPER_CUTTER
void go_origin_stepper_cutter(void)
{
	UINT8 i,time;
    UINT16 run_counter;
	#if SUPPORT_UNIFY_DRIVER
	time = 1;
	#else
	time = 0;
	#endif
	if(sys.status == ERROR)
	   return;
	
	#if USE_ENCODER_YJ_PULSE   
	if( get_stepper_cutter_ORG()== 1)
	#else   
	if( ROTATED_CUTTER_ORG) //��ס���˳���
	#endif
	{
		run_counter = 0;
		#if USE_ENCODER_YJ_PULSE 
		while(get_stepper_cutter_ORG()==1)
		#else
		while(ROTATED_CUTTER_ORG)
		#endif						
		{
		    movestep_lct(1,time);
			run_counter++;
			if(run_counter > 800)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				if( sys.error == 0)
      			    sys.error = ERROR_68;
      			return;
			}
			#if USE_ENCODER_YJ_PULSE 
			delay_us(600);
			#else
			delay_ms(2);
			#endif	
		}
		for(i=0;i<4;i--)
		{
			movestep_lct(1,time);
			#if USE_ENCODER_YJ_PULSE 
			delay_us(600);
			#else
			delay_ms(2);
			#endif
		}
	}
	#if USE_ENCODER_YJ_PULSE 
	if( get_stepper_cutter_ORG()== 0)
	#else
	if(!ROTATED_CUTTER_ORG)//������û���ϣ�Ҫ��������ԭ��
	#endif
	{
		  run_counter = 0;
		  for(i=0;i<3;i++)
		  {
				#if USE_ENCODER_YJ_PULSE 
				while(get_stepper_cutter_ORG()== 0 )
				#else
				while(!ROTATED_CUTTER_ORG)	
				#endif					
				{
					movestep_lct(-1,time);  
					run_counter++;
					if(run_counter > 800)
					{
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
						if( sys.error == 0)
		      			    sys.error = ERROR_68;
		      			return;
					}
					#if USE_ENCODER_YJ_PULSE 
					delay_us(600);
					#else
					delay_ms(2);
					#endif
				}
			    delay_ms(10);
		  }
	}
	delay_ms(5);
	if( stepper_cutter_origin_offset != 0)
	{
		movestep_lct(stepper_cutter_origin_offset,63);  
		delay_ms(100);
	}
}
#endif

/*
  �µĴ����㷨�ǰ��������ݵ��е�8x 0e xx �������Զ�ʶ�����ת�Ƕȡ�
  2016-4-29
  �Ƕ���С�������ʱ����ת
  movestep_cs3(0x4000,-1,1);//��ʱ����ת	
  movestep_cs3(0x4000,1,1);//˳ʱ����ת
  rotated_cutter_position  ----��ǰ���е�λ��
  cutter_rotated_abs_angle ----Ҫ��ת��Ŀ��λ��		 
  Լ�����ǵ�һ������0�����䣬�淽����ת�ǽǶ��𲽱��Ĺ��̡�
  ������
  ��0����270����ת���� Ŀ�꡷��ǰ�ķ�֧��delta1=270,delata2=90 ���ѡ��delta2˳ʱ��ת������
*/
void rotated_cutter_by_data(void)
{
		INT16 temp16;
		UINT16 delta1,delta2;
		INT16 n_tmp;
		UINT8 rotated_dir,flag;

		if ( rotated_cutter_position == cutter_rotated_abs_angle)
			 return;
		//ȷ����ѵ���ת�����λ��
	
		rotated_dir = 0;		
		if( rotated_cutter_position > cutter_rotated_abs_angle)//����ǴӸߵ��ͣ�������˳ʱת
		{
			delta1 = rotated_cutter_position  - cutter_rotated_abs_angle;		//˳ʱ��ת�Ķ�����Χ
			delta2 = cutter_rotated_abs_angle + 360 - rotated_cutter_position;  //��ʱ��ת�Ķ�����Χ
			if( delta1 < delta2 ) 
			{
				rotated_dir = 0;  //˳ʱ��
				temp16 = delta1;
			}
			else
			{
				rotated_dir = 1;  //��ʱ��
				temp16 = delta2;
			}
		}
		else
		{
			delta1 = cutter_rotated_abs_angle - rotated_cutter_position;
			delta2 = rotated_cutter_position  + 360 - cutter_rotated_abs_angle;
			if( delta1 > delta2 ) //��ʱ��ת�ķ�Χ ���� ˳ʱ�뷶Χ ������˳ʱ��ת
			{
				rotated_dir = 0;  //˳ʱ��
				temp16 = delta2;
			}
			else
			{
				rotated_dir = 1;  //��ʱ��
				temp16 = delta1;	
			}
		}
		flag = 0;
		#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER40
		{
			/*
			���20:����32 �����ȣ�1��1.6
			1.6�Ȳ���ǣ�һȦ360�����壬
			������������Ǵ�����λ����������ó�0.8�ȣ�����*2			
			*/
			temp16 = temp16 << 1;
			if( temp16 > 0 )
			{
					while( temp16 > 0 )	//ÿ��0.9�ȣ��۵���ת������0.5625��					
					{
					      if( rotated_dir)
						  	  movestep_cs3(0x4000,-1,1);//��ʱ����ת
						  else
						  	  movestep_cs3(0x4000,1,1);//˳ʱ����ת
						  //delay_ms(1);
						  delay_us(1000);
					      temp16--;	
					}
			}				
			last_rotated_dir = rotated_dir;		
		}
		#else
		{
			//��������160����Ӧ90�ȣ���ÿ����0.5625��		0.025 72
			//temp16 = temp16 *16/9 ;//0.9d  ���20:����32  =>*16/9  
			temp16 = temp16<<4;
			if( last_rotated_dir == rotated_dir )//ת��һ��
				temp16 += cutter_motoer_remain_angle;
			else
				temp16 -= cutter_motoer_remain_angle;
			cutter_motoer_remain_angle = temp16 % 9;//�µ�����
			temp16 = temp16 /9;
			last_rotated_dir = rotated_dir;		
			if( temp16 > 0 )
			{
					while( temp16 > 0 )	//ÿ��0.9�ȣ��۵���ת������0.5625��					
					{
					      if( rotated_dir)
						  	  movestep_cs3(0x4000,-1,1);//��ʱ����ת
						  else
						  	  movestep_cs3(0x4000,1,1);//˳ʱ����ת
						  delay_ms(2);//2
					      temp16--;	
					}
			}
		}
		#endif
	    rotated_cutter_position = cutter_rotated_abs_angle;
}

/*
��ת�е�����,����������8x 0e XX �� Cx 0e XX ���룬��������еĶ������������˳�
*/
void process_rotated_cutter(void)
{
	INT32 allx_step_tmp,ally_step_tmp,delay;
	UINT8 slow_flag ,action_flag ,i ,protect_flag;
	UINT16 x_tmp,y_tmp,sum_x,sum_y,dly_time,tmp_speed,j,ret;

	slow_flag = 0;
	action_flag = 0;
	//1���ߵ�ƫ��λ��
	allx_step_tmp = allx_step;
	ally_step_tmp = ally_step;
	if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 0) )//ƫ�Ʋ�Ϊ0
	{
	     go_commandpoint(x_bios_offset + allx_step,y_bios_offset + ally_step);	  
		 delay_ms(300);
		 milling_first_move = 1;
	}
	allx_step = allx_step_tmp;//Ϊ��ʾ�Ĺ����ȷ
	ally_step = ally_step_tmp;
	//2����ת�����ԭ��
	go_origin_rotated_cutter();//��ת������ҵ�ԭ��
	delay_ms(50);
	go_origin_yj();//��ת�е������ԭ��
	milling_cutter_stop_flag = 1;
	protect_flag = 0;
	rotated_cutter_running_flag = 0;
	while(1)
    {
		if( sys.status == ERROR)
		   break;
		#if DSP3_CUTER_DRIVER
		if( (indraft_control_counter==0) &&( indraft_control_flag == 1) )
		{
			indraft_control_flag = 0;
			output_cs3(4,0);
		}
		#endif
		if( PAUSE == pause_active_level)//press sotp button when stepper motor moving
	  	{
			delay_ms(20);
			if( PAUSE ==pause_active_level)
			{
				rotated_cutter_single_stop();				
				sys.status = READY;
				while( 1 )
				{
					if( DVA == 0) 
					{
						delay_ms(20);
						if( DVA == 0)
						{
						   sys.status = RUN;
						   break;
						}
					}
					delay_ms(1);
					if( origin_com == 1 )
					{
						end_flag = 1;
						predit_shift = 0; 
						single_flag = 0;

						break;
					}
					if(coor_com  == 1)
					{
						predit_shift = 0;
						coor_com = 0;
					}
					if( PointShiftFlag == 1)
					{
						PointShiftFlag = 0;
						predit_shift = 0 ;
					}
  					switch(single_flag)
			  		{			
			   		case 1:							
						rotated_cutter_single_next();
						move_next();						  
						predit_shift = 0;  
						single_flag = 0;                  
						break;
			  		case 2:
						rotated_cutter_single_back();	
						move_back(); 
						predit_shift = 0;
						single_flag = 0; 
						break;
			  		case 6:	 					    
						course_next();  
						delay_ms(2);  
						if(PAUSE == pause_active_level)
						{
							delay_ms(10);
							if(PAUSE == pause_active_level)
							{				
								predit_shift = 0; 
								single_flag = 0;
						  	}
	 
					  	}
						break;		
			  		case 7:	 	
					    course_back();
						delay_ms(2); 
						if(PAUSE == pause_active_level)
						{
							delay_ms(10);
							if(PAUSE == pause_active_level)
							{				
								predit_shift = 0; 
								single_flag = 0;
						  	}
	 
					  	}
						break;			
			  		case 8:	 
						course_stop();       
						predit_shift = 0;   
						break;	
					}
				}
			}
		}
		 
		if(end_flag == 1)
		{
			if(inpress_flag == 0)  
			   inpress_up();
			break;
		}
		
		process_data();		//��ʱ���ŵ������꣬ͣ��ָ��λ�ã�Ȼ�󵶲������ٶ���
			
		if( para.rotate_cutter_working_mode == 55)//���ǵ��ŷ�������
		{
		    if( milling_cutter_stop_flag == 1)
			{
				DRILL_FOOTER = 1;//����ѹ
				delay_ms(rotated_cutter_running_delay);	
				DRILL_MOTOR_UPDOWN = 1;  //ϳ��������
				delay_ms(rotated_cutter_up_delay);
				protect_flag = 1;
				milling_cutter_stop_flag = 0;
				if( para.cutter_speed >2000)
					para.cutter_speed = 2000;
				set_rotated_cutter_speed( para.cutter_speed );
				delay_ms(10);
				rotated_cutter_running_flag = 1;
			}
			
			/*
			if( rotated_cutter_running_flag == 1)//����Ѿ�ת������
			{
				#if DEBUG_ROTATED_CUTTER_DA1
				da1 = 50;
				#endif
				ret = get_YJORG_status();
				while(ret == 0)	//�ȴ�һȦ��ͬ���ź�
				{
					ret = get_YJORG_status();
					rec_com();
					monitor_cutter_angle = get_cutter_motor_angle();
				}
				#if DEBUG_ROTATED_CUTTER_DA1
				da1 = 100;
				#endif
				ret = get_YJORG_status();
			}
			*/
			da1 = 50;
			ret = get_cutter_motor_angle();
			while( ret > 40)
			{
					ret = get_cutter_motor_angle();
					delay_ms(1);
					da1 = ret >>1;
			}
			da1 = 100;		
			//ret = get_cutter_motor_angle();
			//while( ret < cutter_angle_adjust )
			//	{
			//		ret = get_cutter_motor_angle();
			//		delay_ms(1);
			//		da1 = ret >>1;
			//	}
			//da1 = 150;
			
			if( cutter_function_flag == 1)
			{	
				da1 = 150;
				delay_ms(para.rotate_cutter_delaytime1);			
				rotated_cutter_by_data();
				cutter_function_flag = 0;
				process_data();
			}
			
			/*
			���е�һȦ��ʱ��Ϊ����
			1��������ת�Ƕȵ�λ
			�е�Ĭ���ڵ��£�Ĭ��Ϊ0,Ȼ��������̴����ϣ�����Ԥ�ȶ���
			2���е��½��д�����
			3���е�̧��
			4������
			�е�0.9�ȣ�һȦ400����
			*/			
			if( move_flag == 1  )
			{
					if( (protect_flag == 0) && (ally_step <= -300) )
					{
						DRILL_MOTOR_UPDOWN = 1;  //ϳ��������
						delay_ms(rotated_cutter_up_delay);
						protect_flag = 1;
					}
					move_flag = 0;
					nopmove_flag = 0;
					allx_step = allx_step + xstep_cou;                        
			  		ally_step = ally_step + ystep_cou;
					x_tmp = fabsm( xstep_cou);
					y_tmp = fabsm( ystep_cou);
					slow_flag = 0;
				
					//����				
					if( x_tmp > y_tmp)
					    tmp_speed =	spdlimit1_tab[x_tmp-1];
					else
						tmp_speed =	spdlimit1_tab[y_tmp-1];
							
					timer_x = MoveTime1_Speed[tmp_speed/100];	
					if( timer_x > para.rotate_cutter_movetime	)
						timer_x = para.rotate_cutter_movetime;
					
					timer_y = timer_x;
					dly_time = timer_x;	
					
				
				da1 = 200;		
						
					delay_ms(para.rotate_cutter_delaytime2);
				    
					if(fabsm(ystep_cou) > 0)
					{
						movestep_y(-ystep_cou); 
						delay_us(400);
					}
					if(fabsm(xstep_cou) > 0)
					{
						movestep_x(-xstep_cou);
					}
					delay_ms(dly_time);
				da1 = 0;
			}
		}		
		else //���ŷ�������
		{
			if( cutter_function_flag == 1)
			{				
				rotated_cutter_by_data();
				cutter_function_flag = 0;
			}
			if( move_flag == 1  )
			{
						if( milling_cutter_stop_flag == 1)
						{
							DRILL_FOOTER = 1;
							#if DSP3_CUTER_DRIVER
							delay_ms(350);
							output_cs3(1,1);//port0 ����ѹ
							#endif
							delay_ms(rotated_cutter_running_delay);	
							DRILL_MOTOR_UPDOWN = 1;  //ϳ��������
							delay_ms(rotated_cutter_up_delay);
							protect_flag = 1;
							/*				
							if( ally_step <= -300) //Y���Ӳ������1140
							{
								DRILL_MOTOR_UPDOWN = 1;  //ϳ��������
								delay_ms(rotated_cutter_up_delay);
								protect_flag = 1;	
							}
							else
							{
								sys.status = ERROR;
								sys.error = ERROR_82;
								StatusChangeLatch = ERROR;
								return;
							}
							*/
							#if DSP3_CUTER_DRIVER
							output_cs3(1,0);//���ֵ�ѹ
							#endif
							drill_motor_run_enable = 1;     //  �ŵ������ϣ�ϳ����ʼ��ת
							milling_cutter_stop_flag = 0;
						}
						if( (protect_flag == 0)&&(ally_step <= -300) )
						{
							DRILL_MOTOR_UPDOWN = 1;  //ϳ��������
							delay_ms(rotated_cutter_up_delay);
							protect_flag = 1;
						}
						move_flag = 0;
						nopmove_flag = 0;
						allx_step = allx_step + xstep_cou;                        
		  			    ally_step = ally_step + ystep_cou;
						x_tmp = fabsm( xstep_cou);
						y_tmp = fabsm( ystep_cou);
						slow_flag = 0;
						/*
				        if( (x_tmp == 0) ||( y_tmp == 0) )
						{
							while( (x_tmp > 0 )||(y_tmp > 0) )
							{
								if(y_tmp >0)
								{
									timer_y = 1;
									if( ystep_cou >0 )
										movestep_y(-2); 
									else
									    movestep_y(2);
									y_tmp -= 2;
									delay_us(400);	
								}
								if(x_tmp > 0)
								{
									timer_x = 1;
									if(xstep_cou >0)
										movestep_x(-2); 
									else
									    movestep_x(2); 
									x_tmp -= 2;
								}
								if( rotated_cutter_speed == 0)
								    delay_us(800);
								else
							  	    delay_ms(rotated_cutter_speed);
							}
						}
					    else
						*/
						{
							if( (x_tmp<=2)&&(y_tmp <=2) )
							{	
								slow_flag =1;
								timer_y = 2;
								timer_x = 2;	
							}
							else
							{
								if( x_tmp > y_tmp)
								    tmp_speed =	spdlimit1_tab[x_tmp-1];
								else
								    tmp_speed =	spdlimit1_tab[y_tmp-1];
							
								timer_x = MoveTime1_Speed[tmp_speed/100];
								if( timer_x > cutter_syn_delay)
									timer_x -= cutter_syn_delay;
							
								timer_y = timer_x;
								dly_time = timer_x;
							}
				
							if(fabsm(ystep_cou) > 0)
							{
								movestep_y(-ystep_cou); 
								delay_us(400);
							}
							if(fabsm(xstep_cou) > 0)
							{
								movestep_x(-xstep_cou);
							}
							if( slow_flag == 1)
							{
							  	delay_ms(2);
							}
							else
							  delay_ms(dly_time+2);//+2
						}  
			}
		}
		if(nopmove_flag == 1)
		{
				rotated_cutter_single_stop();
			    while( nopmove_flag == 1 )
				{
					do_pat_point_sub_one();
					sys.status = READY;
				    go_beginpoint(0); //�ֶο��ʹ���
					sys.status = RUN;
					process_data();	
					if(OutOfRange_flag == 1)
					{
					   end_flag = 1;
					   sys.status = ERROR;
					   sys.error = ERROR_15;
					   status_now = READY;
					   break;
					}
					if( nopmove_flag != 1)
					{
						do_pat_point_sub_one();
						break;
					}
				}
		}
			if(end_flag == 1)
			{
				if(inpress_flag == 0)  
				   inpress_up();
				break;
			}
			
			if( milling_cutter_action_flag == 2)//����������
			{
				rotated_cutter_single_stop();
				if( milling_first_move == 1)
				{
					if( (x_bios_offset!=0)||(y_bios_offset!=0) )//ƫ�Ʋ�Ϊ0
					{
						 allx_step_tmp = allx_step;
					     ally_step_tmp = ally_step;			
					     go_commandpoint(allx_step-x_bios_offset,ally_step- y_bios_offset);							 
						 allx_step = allx_step_tmp;//Ϊ��ʾ�Ĺ����ȷ
						 ally_step = ally_step_tmp;
						 delay_ms(300);
						 milling_first_move = 0;
					}	
				}		
			milling_cutter_action_flag = 0;
	
			break;
			}
	
	}
	if( (milling_cutter_stop_flag == 0)||(drill_motor_run_enable == 1) )
	{
	    delay_ms(100);
		drill_motor_run_enable = 0; //ϳ���ص�
		if( para.rotate_cutter_working_mode == 55)
		{
			//if( rotated_cutter_running_flag == 1)
			//{
				//send_dsp_command(DSP2,0x0009);//��ͣ����
				//for(j=0;j<500;j++)
				//{
				//	delay_ms(1);
				//	if( check_yj_done() )
				//	   break;
				//}			
			//}
			set_rotated_cutter_speed( 0 );
		}
		#if DSP3_CUTER_DRIVER
			output_cs3(2,2);		//�е�ͣ����λ
			delay_ms(200);
			output_cs3(2,0);
		#endif
		//DRILL_MOTOR_UPDOWN = 0;//ϳ���Ƚ���ȥ
		rotated_cutter_down_positon();
		delay_ms(500);
		DRILL_FOOTER = 0;
		delay_ms(100);
	}
	#if DSP3_CUTER_DRIVER
		output_cs3(4,4);			//������
		indraft_control_flag = 1;
		indraft_control_counter = 2500;
	#endif
	delay_ms(100);
	go_origin_rotated_cutter();
	go_origin_yj();
	//if( (milling_first_move == 1) && (k115 ==1) &&( (x_bios_offset!=0)||(y_bios_offset!=0) ) )
	if( (milling_first_move == 1) &&( (x_bios_offset!=0)||(y_bios_offset!=0) ) )
	{ 
		 allx_step_tmp = allx_step;
		 ally_step_tmp = ally_step;			
		 go_commandpoint(allx_step-x_bios_offset,ally_step- y_bios_offset);	 		 
		 allx_step = allx_step_tmp;
		 ally_step = ally_step_tmp;	 
		 //for(i=0;i<k112;i++)
		 delay_ms(300);
	}

	milling_cutter_action_flag = 0;
	milling_first_move = 0;
	if( end_flag == 1)
	{
		origin_com = 0;
		sys.status = FINISH;
		StatusChangeLatch = FINISH;
	}
	if( sys.status == ERROR)
	{
		StatusChangeLatch = ERROR;
	}
}

#if ENABLE_LASER_CUTTER
void process_laser_offset_move(UINT8 dir)
{
	INT32 allx_step_tmp,ally_step_tmp;
	if( (milling_cutter_action_flag == 0)||((x_bios_offset==0)&&(y_bios_offset==0)) ||(find_laser_start_stop_flag == 0) )
		return;
	allx_step_tmp = allx_step;
	ally_step_tmp = ally_step;
	if( dir == 0)//����ǰ��
	{
		if( (milling_cutter_action_flag == 1)&& (milling_first_move == 0) )//�����иʼ������ûƫ�ƹ�
		{
			go_commandpoint(x_bios_offset + allx_step, y_bios_offset + ally_step);	 
			milling_first_move = 1;
		}
		else if( (milling_cutter_action_flag == 2)&& (milling_first_move == 1) )//�����и������ƫ�ƻָ�
		{
			go_commandpoint(allx_step - x_bios_offset ,ally_step - y_bios_offset);	 
			milling_first_move = 0;
		}
	}
	else 		//�������
	{
		if( (milling_cutter_action_flag == 2)&& (milling_first_move == 0) )//����������ֱ��ƫ��
		{
			go_commandpoint(x_bios_offset + allx_step, y_bios_offset + ally_step);	 
			milling_first_move = 1;
		}
		else if( (milling_cutter_action_flag==1)&& (milling_first_move == 1) )//ƫ�Ʋ�Ϊ0
		{
			go_commandpoint(allx_step - x_bios_offset ,ally_step - y_bios_offset);	 
			milling_first_move = 0;
		}		
	}
	allx_step = allx_step_tmp;
	ally_step = ally_step_tmp;		
	find_laser_start_stop_flag = 0;			
}

#if LASER_DSP_PROCESS

void turnon_laser_power(void)
{
	UINT16 i;	
	laser_power_on_flag = 1;
	laser_power_on_counter = 0;
	LASER_INDICATIOR_LED = 0;
	LASET_POWER_SWITCH = 1;		//��ϵͳ�ܵ�Դ
	LASER_FUN_PIN = 1;
	delay_ms(200);
	LASET_MIRROR_COVER = 1;	
	delay_ms(50);	 
	if( power_on_laser_cool_flag == 0)//�״���ˮ
	{
		power_on_laser_cool_flag = 1;
	    #if SECOND_GENERATION_PLATFORM
			  for( i=0;i<800;i++)
		#else
			  for( i=0;i<400;i++)
		#endif
			  delay_ms(10);
	}
	if( laser_power_on_enable == 1)
		LASER_POWER_PIN = 1;
		
	if( first_start_flag == 0)
		delay_ms(para_x_take_offset_left);//k203
	else
		delay_ms(para_right_barcode_position);//k202
	first_start_flag = 1;
	milling_cutter_stop_flag = 0;
}



UINT8 check_pause_status(void)
{
	if( PAUSE == pause_active_level)//press sotp button when stepper motor moving
	  	{
			delay_ms(20);
			if( PAUSE == pause_active_level)
			{
				if( laser_emergency_stop == 0)
				{
					laser_emergency_stop = 1;
					emergency_restart_flag = 1;//��ͣ���������ݻ���������������£�����û����
					sys.status = ERROR;
					sys.error = ERROR_02;
					StatusChangeLatch = ERROR;
					return 1;
				}
			}
		}
	return 0;
}

void process_laser_cutter(void)
{
	INT32 allx_step_tmp,ally_step_tmp;
	UINT8 slow_flag ,action_flag ,i ,protect_flag,stop_status,stitch_cnt;
	UINT16 x_tmp,y_tmp,sum_x,sum_y,dly_time,tmp_speed,tmp16_stitchs,dly,j,dsp_ret;
    INT8 x_buf[50],y_buf[50];
	UINT8 speed_tab[50],spd_down_flag,spd_down_cnt,spd_delta[20],spd_tmp;
	UINT8 cutting_in_progress,restart_flag;//�Ѿ���ʼ�и���

	slow_flag = 0;
	action_flag = 0;
	stop_status = 0;
	stitch_cnt = 0;
	cutting_in_progress = 0;	
	emergency_restart_flag = 0;	
	restart_flag = 1;
	//1���ߵ�ƫ��λ��
	allx_step_tmp = allx_step;
	ally_step_tmp = ally_step;
	
	if( ((laser_offset_x!=0)||(laser_offset_y!=0))&& (milling_first_move == 0) )//ƫ�Ʋ�Ϊ0
	{
	     go_commandpoint(laser_offset_x + allx_step,laser_offset_y + ally_step);	  
		 delay_ms(300);
		 milling_first_move = 1;
	}
	allx_step = allx_step_tmp;//Ϊ��ʾ�Ĺ����ȷ
	ally_step = ally_step_tmp;

	delay_ms(50);
	milling_cutter_stop_flag = 1;
	protect_flag = 0;

	inflection_poin_flag = 0;
	laser_cutter_aciton_flag = 0;
	rec1_total_counter = 0;
	frame_start_counter = 0;
	if( cutter_syn_delay < 15)//�ٶ�����
		cutter_syn_delay = 15;
	for(i=0;i<50;i++)
	{
		x_buf[i] = 0;
		y_buf[i] = 0;
		speed_tab[i] = cutter_syn_delay;
	}
	spd_down_flag = 0;
	spd_down_cnt = 0;
	if( cutter_syn_delay > 250)				//̫���˾Ͳ���Ҫ������
	    spd_tmp = 0;
	else
		spd_tmp = ( 250 - cutter_syn_delay )/9;
	for( i = 0;i<10;i++)//�����յ㽵��ʱ���ٶȸ���
	{
		spd_delta[i]   = cutter_syn_delay + spd_tmp*i;//0, 1,  2, 3, 4,5,6,7,8,9
		spd_delta[19-i] = spd_delta[i];				  //19,18,17,16,15 �ٶ�Խ��Խ����ʱ��Խ��Խ�������߶Գ�
	}

	while(1)
    {
		if( (sys.status == ERROR)&&(sys.error != ERROR_98)&&(sys.error != ERROR_02) )
		     break;
		
		if( (LASER_HEADER_PROTECT == 1)||(laser_power_error_flag==1) )//���Ᵽ������
		{
			delay_ms(20);
			if( (LASER_HEADER_PROTECT == 1)||(laser_power_error_flag ==1) )
				{					
					if(frame_start_counter >0)
					{
						movestep_xy3(x_buf,y_buf,frame_start_counter,speed_tab);
					}
					while( requery_dsp1_status() != 1)
			  		 		rec_com();
					for( i=0;i<k112;i++)
				   		delay_ms(200);
					LASER_POWER_PIN = 0;
					LASET_MIRROR_COVER = 0;	
					LASER_INDICATIOR_LED = LASER_LED_ON;
					laser_fun_delay_counter = 0;			
					laser_fun_delay_off_flag = 1;
				
					sys.error = ERROR_98;
					sys.status = ERROR;
					milling_cutter_stop_flag = 1;
					stop_status = 1;
				} 
		}
		/*
		�и��еļ�ͣ����
		�ȰѲ���ֹͣ������������������û�ߵ�λ�ƣ���������Ҳ�������µ�����û��
		�������ĸ���������Ӧ��
		ǰ��+���ˣ�  ����Ӧ�˶���
		��λ��ԭ�㣺 �����Ҵ�����ԭ��
		���������и �������������и����������������ݼ�������
		*/

		dsp_ret = check_pause_status();
		if( (laser_emergency_stop == 1)&&(stop_status == 0))
		 {
				laser_fun_delay_counter = 0;			
				laser_fun_delay_off_flag = 1;
				if( cutting_in_progress == 1)//�Ѿ��ڼ��⴦�������
				{
					send_dsp_command(DSP1,0x0005);//��ͣ
					for(j=0;j<200;j++)			  //�ȴ��������� 
					{
						delay_ms(1);
						if( check_motion_done() )
						   break;
				  	}
				    LASER_POWER_PIN = 0;	
					//LASET_MIRROR_COVER = 0;					 				
					LASER_INDICATIOR_LED = LASER_LED_ON;
				 	send_dsp1_command(0x0007,0x5555);
					if( recieve_x.word != 0xd0a0) //��������û���������
					{
						dsp_ret = recieve_x.word;		//���ж�����û��
						cutting_in_progress = 0;						
					}
					else//����ͣ�ˣ������Ѿ�������
					{
						stop_status = 1;
						laser_emergency_stop = 0;
					}
				}	            
				milling_cutter_stop_flag = 1;
				stop_status = 1;
		} 
		
		if( stop_status == 1)
		{
				stitch_cnt = 0;
				while( 1 )
				{
					if( (sys.status == ERROR)&&(StatusChangeLatch == READY))//�����ָ�
					{
						sys.status = READY;
						predit_shift = 0;
						sys.error = 0;
					}
					if( (sys.status == ERROR)&&(sys.error ==ERROR_98) )
					{
						if(LASER_HEADER_PROTECT == 0)
						{
							delay_ms(20);
							if( LASER_HEADER_PROTECT == 0)
							{
								turnoff_ledbuz();					
				           		sys.status = READY;
								StatusChangeLatch = READY;
			                	sys.error = OK; 								
							}
						}
						else
						{
							flash_buz();
							flash_led(); 
						}
					}
					else if( LASER_HEADER_PROTECT == 1)
					{
						delay_ms(20);
						if( LASER_HEADER_PROTECT == 1)
						{
							LASER_POWER_PIN = 0;
							LASET_MIRROR_COVER = 0;	
							LASER_INDICATIOR_LED = LASER_LED_ON;
							laser_fun_delay_counter = 0;			
							laser_fun_delay_off_flag = 1;
				
							sys.error = ERROR_98;
							sys.status = ERROR;
							milling_cutter_stop_flag = 1;
						}
					}					
					else if( DVA == 0) 
					{
						delay_ms(20);
						if( DVA == 0)
						{
						   if(laser_emergency_stop == 1)//��������ͣ�������������ﻹ��û���������
						   {
							  turnon_laser_power();		  //�򿪼���
							  send_dsp1_command(0x0014,0);//��ͣ��ָ���������
							  
							  if(end_flag == 1 )//�Ƚ���ʱ������ͣ
							  {
								  while( requery_dsp1_status() != 1)
								  {
								  	 rec_com();
									 dsp_ret = check_pause_status();
									 if( laser_emergency_stop == 1)
									 	 break;
								  }
								  if( laser_emergency_stop == 0)
								  {
								  	  for( i=0;i<k112;i++)
									      delay_ms(200);
									  laser_emergency_stop = 0;	
							  		  restart_flag = 0;	
								  }
								  if( inpress_flag == 0)  
							   		  inpress_up();								  
							  }
							  else
							  {
								  if(frame_start_counter > 0)//֮ǰ�ڻ���Ҫ���͵�����û������
								  {
									 movestep_xy3(x_buf,y_buf,frame_start_counter,speed_tab);
								  }
								  if( laser_emergency_stop == 0)
								  {
									  sys.status = RUN;
								   	  stop_status = 0;
									  laser_emergency_stop = 0;	
								  	   restart_flag = 0;
								  }
							  }							  
						   }
						   else
						   {
							   sys.status = RUN;
							   stop_status = 0;
							   restart_flag = 1;								   
						   }
						   if( laser_emergency_stop == 0)
						   		break;
						}
					}			
					
					delay_ms(1);
					if( origin_com == 1 )
					{
						end_flag = 1;
						predit_shift = 0; 
						single_flag = 0;
						if( laser_emergency_stop == 1)//��ͣ���ԭ�㴦��
						{
							opl_origin_flag = 0;//��������ԭ�㴦��							
							stop_status = 0;
						}
						break;
					}
					if(coor_com  == 1)
					{
						predit_shift = 0;
						coor_com = 0;
					}
					if( PointShiftFlag == 1)
					{
						PointShiftFlag = 0;
						predit_shift = 0 ;
					}
					//milling_cutter_action_flag = 0;
					if ( laser_emergency_stop == 0)
					{
	  					switch(single_flag)
				  		{			
				   		case 1:							
							move_next();						  
							predit_shift = 0;  
							single_flag = 0;                  
							break;
				  		case 2:
							move_back(); 
							predit_shift = 0;
							single_flag = 0; 
							break;
				  		case 6:	 					    
							course_next();  						
							delay_ms(2);  
							if(PAUSE == pause_active_level)
							{
								delay_ms(10);
								if(PAUSE == pause_active_level)
								{				
									predit_shift = 0; 
									single_flag = 0;
							  	}	 
						  	}
							break;		
				  		case 7:	 	
						    course_back();
							delay_ms(2); 
							if(PAUSE == pause_active_level)
							{
								delay_ms(10);
								if(PAUSE == pause_active_level)
								{				
									predit_shift = 0; 
									single_flag = 0;
							  	}
						  	}
							break;			
				  		case 8:	 
							course_stop();       
							predit_shift = 0;   
							break;	
						}
					}//laser_emergency_stop =0
				}//while 1
		}//stop_status
		if(end_flag == 1)
		{
			if(inpress_flag == 0)  
			   inpress_up();
			break;
		}
	    //�յ㽵��ɨ��
		if( (((pat_point+9)->func ==0x80)||((pat_point+9)->func ==0xc0))&& ((pat_point+9)->xstep==0x13) )
		{
			spd_down_flag = 1;
			spd_down_cnt = 19;	
			//SUM = 1;
		}
		if( restart_flag == 1)
			process_data();		//��ʱ���ŵ������꣬ͣ��ָ��λ�ã�Ȼ�󵶲������ٶ���
		else
			restart_flag = 1;
		
		if( cutter_function_flag == 1)
		{				
			cutter_function_flag = 0;
		}
		
		if( move_flag == 1  )
		{
			laser_fun_delay_counter = 0;				
			if( frame_start_counter < 49 )
			{
				if((0 != xstep_cou) || (0 != ystep_cou))
				{ 
					x_buf[frame_start_counter] = -xstep_cou;	//��¼����
					y_buf[frame_start_counter] = -ystep_cou;
					if( spd_down_flag == 1)						//�յ㽵�ٵĴ���
					{
						speed_tab[frame_start_counter] = spd_delta[spd_down_cnt];
						if( spd_down_cnt > 0)
						{						
						   spd_down_cnt--;	
						}
						else
						{
							spd_down_cnt = 0;
							spd_down_flag = 0;
						}
					}
					else
					{
						if( stitch_cnt< 10)
						{
							speed_tab[frame_start_counter] = spd_delta[9 -stitch_cnt];
							stitch_cnt++;
						}
						else
						{
							x_tmp = fabsm(xstep_cou);
							y_tmp = fabsm(ystep_cou);
							if( (x_tmp<=10) && (y_tmp <=10 ) )//��С��0.5mm���
							{
								if( cutter_syn_delay < 200)
									speed_tab[frame_start_counter] = 200;
								else
									speed_tab[frame_start_counter] = cutter_syn_delay;
							}
							else
						    	speed_tab[frame_start_counter] = cutter_syn_delay;
						}
					}
					allx_step = allx_step + xstep_cou;                        
				  	ally_step = ally_step + ystep_cou;				

					frame_start_counter++;
					move_flag = 0;
					nopmove_flag = 0;
					
				}	
			}
			else
			{
					if( milling_cutter_stop_flag == 1)
					{
						turnon_laser_power();
					}
					x_buf[frame_start_counter] = -xstep_cou;
					y_buf[frame_start_counter] = -ystep_cou;
					if( spd_down_flag == 1)
					{
						speed_tab[frame_start_counter] = spd_delta[spd_down_cnt];
						if( spd_down_cnt > 0)
						{						
						   spd_down_cnt--;	
						}
						else
						{
							spd_down_cnt = 0;
							spd_down_flag = 0;
							//SUM = 0;
						}     
					}
					else
						speed_tab[frame_start_counter] = cutter_syn_delay;
					move_flag = 0;
					nopmove_flag = 0;
					allx_step = allx_step + xstep_cou;                        
				  	ally_step = ally_step + ystep_cou;
					movestep_xy3(x_buf,y_buf,50,speed_tab);					
					cutting_in_progress = 1;
			}
		}
		
		
		if( cut_flag == 1)
		    cut_flag = 0;
		if(nopmove_flag == 1)
		{
				stitch_cnt = 0;
				laser_cutter_aciton_flag = 0; 
				if(frame_start_counter >0)
				{
					if( milling_cutter_stop_flag == 1)
					{
						turnon_laser_power();
					}				
					movestep_xy3(x_buf,y_buf,frame_start_counter,speed_tab);
					cutting_in_progress = 1;
				}
				if( laser_emergency_stop == 1)//����ǰ���ּ�ͣ��
				{
					emergency_restart_flag = 3;//����������Ȼ������յĹ�����������ͣ
				}
				else
				{
					while( requery_dsp1_status() != 1)
					{
						if( check_pause_status()== 1)
							break;
			  		 	   rec_com();
					}
					if( laser_emergency_stop == 0)
					{
						for( i=0;i<k112;i++)
						   delay_ms(200);
						LASER_POWER_PIN = 0;	
						LASET_MIRROR_COVER = 0;					 				
						LASER_INDICATIOR_LED = LASER_LED_ON;
						laser_fun_delay_counter = 0;			
						laser_fun_delay_off_flag = 1;
						milling_cutter_stop_flag = 1;
						cutting_in_progress = 0;
					    while( nopmove_flag == 1 )
						{
							do_pat_point_sub_one();
							sys.status = READY;
						    go_beginpoint(0); //�ֶο��ʹ���
							sys.status = RUN;
							process_data();	
							if(OutOfRange_flag == 1)
							{
							   end_flag = 1;
							   sys.status = ERROR;
							   sys.error = ERROR_15;
							   status_now = READY;
							   break;
							}
							if( nopmove_flag != 1)
							{
								do_pat_point_sub_one();
								break;
							}
						}
					}
				}
		}
		
		if(end_flag == 1)
		{
				if(frame_start_counter >0)//�����Ѿ����������ˣ�����������������û����
				{
					if( milling_cutter_stop_flag == 1)
					{
						turnon_laser_power();
					}					
					movestep_xy3(x_buf,y_buf,frame_start_counter,speed_tab);
					cutting_in_progress = 1;
				}
				if( laser_emergency_stop == 1)//����ǰ���ּ�ͣ��
				{
					emergency_restart_flag = 2;//��������������Ȼ������յĹ�����������ͣ
				}
				else
				{
					while( requery_dsp1_status() != 1)
					{
						if( check_pause_status()== 1)
							break;
				  		rec_com();
					}
					if( laser_emergency_stop == 0)
					{
						for( i=0;i<k112;i++)
						    delay_ms(200);
						if(inpress_flag == 0)  
				   			inpress_up();
						cutting_in_progress = 0;   
						break;
					}
				}
			
		}
			
		if( milling_cutter_action_flag == 2)//����������
		{
				if(frame_start_counter >0)
				{
					if( milling_cutter_stop_flag == 1)
					{
						turnon_laser_power();
					}				
					movestep_xy3(x_buf,y_buf,frame_start_counter,speed_tab);
					cutting_in_progress = 1;
				}
				if( laser_emergency_stop == 1)//����ǰ���ּ�ͣ��
				{
					emergency_restart_flag = 3;//�����е�������Ȼ������յĹ�����������ͣ
				}
				else
				{
					while( requery_dsp1_status() != 1)
					{
						if( check_pause_status()== 1)
							break;
			  		 		rec_com();
					}
					if( laser_emergency_stop == 0)
					{
						for( i=0;i<k112;i++)
						   delay_ms(200);
						laser_cutter_aciton_flag = 0;
						LASER_POWER_PIN = 0;
						LASET_MIRROR_COVER = 0;	
						LASER_INDICATIOR_LED = LASER_LED_ON;
						laser_fun_delay_counter = 0;			
						laser_fun_delay_off_flag = 1;
						delay_ms(200);
						cutting_in_progress = 0;
						milling_cutter_action_flag = 0;
						if( (laser_offset_x!=0)||(laser_offset_y!=0) )//ƫ�Ʋ�Ϊ0
						{
							 allx_step_tmp = allx_step;
						     ally_step_tmp = ally_step;			
						     go_commandpoint(allx_step-laser_offset_x,ally_step- laser_offset_y);	 					 
							 allx_step = allx_step_tmp;//Ϊ��ʾ�Ĺ����ȷ
							 ally_step = ally_step_tmp;
							 delay_ms(300);
							 milling_first_move = 0;
						}
						break;
					}
				}			
		}
	}
	stitch_cnt = 0;
	double_xy_time_flag = 0;
	LASER_POWER_PIN = 0;
	LASET_MIRROR_COVER = 0;	
    LASER_INDICATIOR_LED = LASER_LED_ON;
	laser_fun_delay_counter = 0;			
	laser_fun_delay_off_flag = 1;	
	laser_cutter_aciton_flag = 0;
	LASER_POWER_PIN = 0;
	milling_cutter_action_flag = 0;
	
	if( (laser_emergency_stop == 1)&&(origin_com ==1) )//��ͣ��λ����ԭ��
	{
		origin_com = 1;//����READY״̬ȥ��Ӧ
		end_flag = 1;
		opl_origin_flag = 0;
		already_auto_find_start_point = 0;			
		sys.status = READY;
		StatusChangeLatch = READY;
	}
	else
	{
		if( (milling_first_move == 1)  &&( (laser_offset_x!=0)||(laser_offset_y!=0) ) )
		{ 
			 allx_step_tmp = allx_step;
			 ally_step_tmp = ally_step;			
			 go_commandpoint(allx_step - laser_offset_x,ally_step- laser_offset_y);	  
			 allx_step = allx_step_tmp;
			 ally_step = ally_step_tmp;	 
		}
		milling_first_move = 0;
		if( end_flag == 1)
		{
			origin_com = 0;
			sys.status = FINISH;
			StatusChangeLatch = FINISH;
		}
	}
	laser_emergency_stop = 0;
	
	if( sys.status == ERROR)
	{
		StatusChangeLatch = ERROR;
	}
}

#else

void process_laser_cutter(void)
{
	INT32 allx_step_tmp,ally_step_tmp;
	UINT8 slow_flag ,action_flag ,i ,protect_flag,stop_status,first_flag,stitch_cnt;
	UINT16 x_tmp,y_tmp,sum_x,sum_y,dly_time,tmp_speed,tmp16_stitchs,dly;

	slow_flag = 0;
	action_flag = 0;
	stop_status = 0;
	stitch_cnt = 0;
			
	//1���ߵ�ƫ��λ��
	allx_step_tmp = allx_step;
	ally_step_tmp = ally_step;
	
	if( ((laser_offset_x!=0)||(laser_offset_y!=0))&& (milling_first_move == 0) )//ƫ�Ʋ�Ϊ0
	{
	     go_commandpoint(laser_offset_x + allx_step,laser_offset_y + ally_step);	  
		 delay_ms(300);
		 milling_first_move = 1;
	}
	allx_step = allx_step_tmp;//Ϊ��ʾ�Ĺ����ȷ
	ally_step = ally_step_tmp;

	delay_ms(50);
	milling_cutter_stop_flag = 1;
	protect_flag = 0;
	first_flag = 0;
	inflection_poin_flag = 0;
	laser_cutter_aciton_flag = 0;
	rec1_total_counter = 0;
	/*
	ʱ����24MHz ���Ǽ���24000000����ʱ1�� 24000��Ӧ1ms
	1.5ms = 36000
	*/
	dly = cutter_syn_delay;
	dly = dly*11 + 250;
	if( dly > 2730 )
		dly = 2730;
	tb4 = dly *24;
	
	while(1)
    {
		if( (sys.status == ERROR)&&(sys.error != ERROR_98) )
		     break;
		
		if( (LASER_HEADER_PROTECT == 1)||(laser_power_error_flag==1) )
		{
			delay_ms(20);
			if( (LASER_HEADER_PROTECT == 1)||(laser_power_error_flag ==1) )
				{
					#if INSERPOINT_ENABLE
					while( rec1_total_counter > 0 )
					rec_com();
					stitch_cnt = 0;
					tb4s = 0;
					laser_cutter_aciton_flag = 0; 
					#endif
					LASER_POWER_PIN = 0;
					LASET_MIRROR_COVER = 0;	
					LASER_INDICATIOR_LED = LASER_LED_ON;
					laser_fun_delay_counter = 0;			
					laser_fun_delay_off_flag = 1;
				
					sys.error = ERROR_98;
					sys.status = ERROR;
					milling_cutter_stop_flag = 1;
					stop_status = 1;
				} 
		}
		
		if( PAUSE == pause_active_level)//press sotp button when stepper motor moving
	  	{
			delay_ms(20);
			if( PAUSE == pause_active_level)
			{
				#if INSERPOINT_ENABLE
					while( rec1_total_counter > 0 )
					rec_com();
					stitch_cnt = 0;
					tb4s = 0;
					laser_cutter_aciton_flag = 0; 
				#endif
				LASER_POWER_PIN = 0;
				LASET_MIRROR_COVER = 0;	
				LASER_INDICATIOR_LED = LASER_LED_ON;
				laser_fun_delay_counter = 0;			
				laser_fun_delay_off_flag = 1;
	            
				sys.status = READY;
				milling_cutter_stop_flag = 1;
				stop_status = 1;
			}
		}
		 
		
		if( stop_status == 1)
		{
				stitch_cnt = 0;
				while( 1 )
				{
					if( (sys.status == ERROR)&&(sys.error ==ERROR_98) )
					{
						if(LASER_HEADER_PROTECT == 0)
						{
							delay_ms(20);
							if( LASER_HEADER_PROTECT == 0)
							{
								turnoff_ledbuz();					
				           		sys.status = READY;
								StatusChangeLatch = READY;
			                	sys.error = OK; 								
							}
						}
						else
						{
							flash_buz();
							flash_led(); 
						}
					}
					else if( LASER_HEADER_PROTECT == 1)
					{
						delay_ms(20);
						if( LASER_HEADER_PROTECT == 1)
						{
							LASER_POWER_PIN = 0;
							LASET_MIRROR_COVER = 0;	
							LASER_INDICATIOR_LED = LASER_LED_ON;
							laser_fun_delay_counter = 0;			
							laser_fun_delay_off_flag = 1;
				
							sys.error = ERROR_98;
							sys.status = ERROR;
							milling_cutter_stop_flag = 1;
						}
					}					
					else if( DVA == 0) 
					{
						delay_ms(20);
						if( DVA == 0)
						{
						   sys.status = RUN;
						   stop_status = 0;
						   predit_shift = 0; 
						   break;
						}
					}			
					
					delay_ms(1);
					if( origin_com == 1 )
					{
						end_flag = 1;
						predit_shift = 0; 
						single_flag = 0;
				   		//sys.status = READY;
				   		//status_now = READY;
						//StatusChangeLatch = READY;
						//return;
						break;
					}
					if(coor_com  == 1)
					{
						predit_shift = 0;
						coor_com = 0;
					}
					if( PointShiftFlag == 1)
					{
						PointShiftFlag = 0;
						predit_shift = 0 ;
					}
					milling_cutter_action_flag = 0;
  					switch(single_flag)
			  		{			
			   		case 1:							
						move_next();						  
						predit_shift = 0;  
						single_flag = 0;                  
						break;
			  		case 2:
						move_back(); 
						predit_shift = 0;
						single_flag = 0; 
						break;
			  		case 6:	 					    
						course_next();  						
						delay_ms(2);  
						if(PAUSE == pause_active_level)
						{
							delay_ms(10);
							if(PAUSE == pause_active_level)
							{				
								predit_shift = 0; 
								single_flag = 0;
						  	}	 
					  	}
						break;		
			  		case 7:	 	
					    course_back();
						delay_ms(2); 
						if(PAUSE == pause_active_level)
						{
							delay_ms(10);
							if(PAUSE == pause_active_level)
							{				
								predit_shift = 0; 
								single_flag = 0;
						  	}
	 
					  	}
						break;			
			  		case 8:	 
						course_stop();       
						predit_shift = 0;   
						break;	
					}
				}
		}
		if(end_flag == 1)
		{
			if(inpress_flag == 0)  
			   inpress_up();
			break;
		}

		process_data();		//��ʱ���ŵ������꣬ͣ��ָ��λ�ã�Ȼ�󵶲������ٶ���
		
		if( cutter_function_flag == 1)
		{				
			cutter_function_flag = 0;
		}
		
		if( move_flag == 1  )
		{
				if( milling_cutter_stop_flag == 1)
				{
						//delay_ms(3000);		
					  if( first_flag == 1)
					  {
						  laser_power_on_flag = 1;
						  laser_power_on_counter = 0;
						  LASER_INDICATIOR_LED = 0;
						  LASER_FUN_PIN = 1;						
						  delay_ms(200);
						  LASET_MIRROR_COVER = 1;
						  delay_ms(50);	
						  if( laser_power_on_enable == 1)
						  	  LASER_POWER_PIN = 1;		
						  milling_cutter_stop_flag = 0;
						  delay_ms(150);
					  }
					  else
					  {
							first_flag = 1;
							laser_power_on_flag = 1;
							laser_power_on_counter = 0;
							LASER_INDICATIOR_LED = 0;
							LASET_POWER_SWITCH = 1;		//��ϵͳ�ܵ�Դ
							LASER_FUN_PIN = 1;
							delay_ms(500);
							LASET_MIRROR_COVER = 1;						
							delay_ms(1000);	
							if( laser_power_on_enable == 1)				  
								LASER_POWER_PIN = 1;
							milling_cutter_stop_flag = 0;
							delay_ms(150);	
					  }					
				}
				if( (protect_flag == 0)&&(ally_step <= -300) )
				{
					protect_flag = 1;
				}
				move_flag = 0;
				nopmove_flag = 0;
				allx_step = allx_step + xstep_cou;                        
		  		ally_step = ally_step + ystep_cou;				
				slow_flag = 0;					
				laser_fun_delay_counter = 0;			
				laser_fun_delay_off_flag = 1;
					
			#if INSERPOINT_ENABLE
					double_xy_time_flag = 0;
					if( inflection_poin_flag == 1)
					{
					    stitch_cnt = 0;
						inflection_poin_flag = 0;
					}
					PBP_Line(stitch_cnt);

					if (laser_cutter_aciton_flag == 0)//��������
					{
						tb4 = dly *24; 
						laser_cutter_aciton_flag = 1;						
						tb4s = 1;
					}
		
					//#if  UART1_DEBUG_OUTPUT_MODE
			    	//tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
					//set_func_code_info(x_tmp,y_tmp,tmp16_stitchs>>8,tmp16_stitchs);
					//#endif	
					//rec_com();
			#else
					double_xy_time_flag = 1;
					x_tmp = fabsm( xstep_cou);
					y_tmp = fabsm( ystep_cou);
					if( (x_tmp<=2)&&(y_tmp <=2) )
					{	
							slow_flag =1;
							timer_y = cutter_syn_delay;
							timer_x = cutter_syn_delay;	
					}
					else
					{
							if( x_tmp > y_tmp)
							    tmp_speed =	spdlimit1_tab[x_tmp-1];
							else
							    tmp_speed =	spdlimit1_tab[y_tmp-1];
							timer_x = MoveTime1_Speed[tmp_speed/100] + cutter_syn_delay;
							if( timer_x >63)
								timer_x = 63;
							timer_y = timer_x;
							dly_time = timer_x;
					}
					if(fabsm(ystep_cou) > 0)
					{
						movestep_y(-ystep_cou); 
						delay_us(400);
					}
					if(fabsm(xstep_cou) > 0)
					{
						movestep_x(-xstep_cou);
					}
					if( slow_flag == 1)
					{
						for( i=0;i<cutter_syn_delay;i++)
						   delay_us(1000);
						//delay_ms(cutter_syn_delay);
					}
					else
					{
					    delay_ms(dly_time<<1);
					}
			#endif
		}
		if( cut_flag == 1)
		    cut_flag = 0;
		if(nopmove_flag == 1)
		{
				while( rec1_total_counter > 0 )
					rec_com();
				stitch_cnt = 0;
				tb4s = 0;
				laser_cutter_aciton_flag = 0; 
				
				LASER_POWER_PIN = 0;	
				LASET_MIRROR_COVER = 0;	
				for( i=0;i<k112;i++)
				   delay_ms(200);					 
				 				
				LASER_INDICATIOR_LED = LASER_LED_ON;
				laser_fun_delay_counter = 0;			
				laser_fun_delay_off_flag = 1;
				milling_cutter_stop_flag = 1;
			    while( nopmove_flag == 1 )
				{
					do_pat_point_sub_one();
					sys.status = READY;
				    go_beginpoint(0); //�ֶο��ʹ���
					sys.status = RUN;
					process_data();	
					if(OutOfRange_flag == 1)
					{
					   end_flag = 1;
					   sys.status = ERROR;
					   sys.error = ERROR_15;
					   status_now = READY;
					   break;
					}
					if( nopmove_flag != 1)
					{
						do_pat_point_sub_one();
						break;
					}
				}
		}
		
		if(end_flag == 1)
		{
			if(inpress_flag == 0)  
			   inpress_up();
			break;
		}
			
		if( milling_cutter_action_flag == 2)//����������
		{
			milling_cutter_action_flag = 0;
			break;			
		}
	}
	#if INSERPOINT_ENABLE 
	while( rec1_total_counter > 0 )
		   rec_com();
	stitch_cnt = 0;
	tb4s = 0;
	#endif
	double_xy_time_flag = 0;
	LASER_POWER_PIN = 0;
	LASET_MIRROR_COVER = 0;	
	for( i=0;i<k112;i++)
		 delay_ms(200);
    LASER_INDICATIOR_LED = LASER_LED_ON;
	laser_fun_delay_counter = 0;			
	laser_fun_delay_off_flag = 1;	
	laser_cutter_aciton_flag = 0;

	if( (milling_first_move == 1)  &&( (laser_offset_x!=0)||(laser_offset_y!=0) ) )
	{ 
		 allx_step_tmp = allx_step;
		 ally_step_tmp = ally_step;			
		 go_commandpoint(allx_step-laser_offset_x,ally_step- laser_offset_y);	  
		 allx_step = allx_step_tmp;
		 ally_step = ally_step_tmp;	 
	}
	LASER_POWER_PIN = 0;
	milling_cutter_action_flag = 0;
	milling_first_move = 0;
	if( end_flag == 1)
	{
		origin_com = 0;
		sys.status = FINISH;
		StatusChangeLatch = FINISH;
	}
	if( sys.status == ERROR)
	{
		StatusChangeLatch = ERROR;
	}
}
#endif
#endif

//========================================================
#if ENABLE_LOOPER_CUTTER

void dsp2_quickmove_x(INT32 x_data)
{ 
    UINT16 h_data; 	
	while(spi_flag > 0);
	if(x_data != 0)
	{
    	SPISTE1 = 1; 
		SPISTE2 = 0;
		SPISTE3 = 1;
	}
	h_data = 0;
	if(x_data>0)
	{
		spi_flag=1;	
	    h_data = (x_data & 0x1c000) >>14;
		x_data = x_data &0x3fff;
		
	    if( yj_motor_dir == 0)					
		   trans_x.word=(UINT16)0x0000+((UINT16)x_data);	    
		else
		   trans_x.word=(UINT16)0x4000+((UINT16)x_data);	    
		if(  trans_x.word == 0x5555 )
		{
			trans_x.word = trans_x.word + 1;
		}   
		trans_y.word=(~trans_x.word)&0x7fff;
	    trans_z.word= (0x5550 | h_data);
		dsp2 = 1;
		SPISTE2 = 0;
		s4trr=trans_x.byte.byte1;
	}	
	else if(x_data<0)
	{
		x_data_nf=-x_data;
		h_data = (x_data_nf & 0x1c000) >>14;
		x_data_nf = x_data_nf &0x3fff;
		spi_flag=1;	
		if( yj_motor_dir == 0)
		   trans_x.word=(UINT16)0x4000+((UINT16)x_data_nf);	
		else
		   trans_x.word=(UINT16)0x0000+((UINT16)x_data_nf);	
		if( trans_x.word == 0x5555 )
		{
			trans_x.word = trans_x.word + 1;
		}
		trans_y.word=(~trans_x.word)&0x7fff;
	    trans_z.word= (0x5550 | h_data);
		
		dsp2 = 1;
		SPISTE2=0;
		s4trr=trans_x.byte.byte1;
	}	
}

void process_stepper_cutter(void)
{
	INT32 allx_step_tmp,ally_step_tmp,tmp;
	UINT8 slow_flag ,action_flag ,protect_flag,timexy,start_flag, nopstop_tmp,once_flag;
	UINT16 x_tmp,y_tmp,sum_x,sum_y,dly_time,tmp_speed,i;

	slow_flag = 0;
	action_flag = 0;
	//1���ߵ�ƫ��λ��
	allx_step_tmp = allx_step;
	ally_step_tmp = ally_step;
	if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 0) )//ƫ�Ʋ�Ϊ0
	{
	     go_commandpoint(x_bios_offset + allx_step,y_bios_offset + ally_step);	  
		 delay_ms(300);
		 milling_first_move = 1;
	}
	allx_step = allx_step_tmp;
	ally_step = ally_step_tmp;
	//2����ԭ��
	go_origin_stepper_cutter();
	delay_ms(50);
	protect_flag = 0;
	milling_cutter_stop_flag = 1;
	/*
	if( aging_com == 1)
	{
		sys.status = RUN;
	    start_flag = 1;
	}
	else
	{
		while( DVA ==0)
		{
		     rec_com();
			 delay_ms(50);
		}
		start_flag = 0;
		sys.status = READY;
	}
	*/
	sys.status = RUN;
    start_flag = 1;
	once_flag = 0;
	while(1)
    {
		if( sys.status == ERROR)
		   break;

		if( PAUSE == pause_active_level )//press sotp button when stepper motor moving
	  	{
			delay_ms(20);
			if( PAUSE == pause_active_level )
			{
				sys.status = READY;
				go_origin_stepper_cutter();
				delay_ms(10);
				DRILL_FOOTER = 0;
				milling_cutter_stop_flag = 1;
				start_flag = 0;
				predit_shift = 0; 
			}
		}
		if( special_pause_flag  == 1)
		{
			special_pause_flag = 0;
			sys.status = READY;
			go_origin_stepper_cutter();
			delay_ms(10);
			DRILL_FOOTER = 0;
			milling_cutter_stop_flag = 1;
			start_flag = 0;
			predit_shift = 0; 
			//turnoff_buz();
		}
		if( start_flag ==0 )
		{
					if( DVA == 0) 
					{
						delay_ms(20);
						if( DVA == 0)
						{
						   sys.status = RUN;
						   start_flag = 1;
						   predit_shift = 0; 
						}
					}
					delay_ms(1);
					if( origin_com == 1 )
					{
						end_flag = 1;
						predit_shift = 0; 
						single_flag = 0;
					}
					if(coor_com  == 1)
					{
						predit_shift = 0;
						coor_com = 0;
					}
					if( PointShiftFlag == 1)
					{
						PointShiftFlag = 0;
						predit_shift = 0 ;
					}
  					switch(single_flag)
			  		{			
			   		case 1:							
						move_next();						  
						predit_shift = 0;  
						single_flag = 0;                  
						break;
			  		case 2:
						move_back(); 
						predit_shift = 0;
						single_flag = 0; 
						break;
			  		case 6:	 					    
						course_next();  
						delay_ms(2);  
						if(PAUSE == pause_active_level)
						{
							delay_ms(10);
							if(PAUSE == pause_active_level)
							{				
								predit_shift = 0; 
								single_flag = 0;
						  	}
					  	}
						break;		
			  		case 7:	 	
					    course_back();
						delay_ms(2); 
						if(PAUSE == pause_active_level)
						{
							delay_ms(10);
							if(PAUSE == pause_active_level)
							{				
								predit_shift = 0; 
								single_flag = 0;
						  	}
	 
					  	}
						break;			
			  		case 8:	 
						course_stop();       
						predit_shift = 0;   
						break;	
					}
	}
	else	
	{
		process_data();	
		if( cutter_function_flag == 1)
		{				
			cutter_function_flag = 0;
			//move_flag = 0;
			//nopmove_flag = 0;
		}
		if( cut_flag == 1)
		    cut_flag = 0;
		if( move_flag == 1  )
		{
					if( milling_cutter_stop_flag == 1)
					{
						if( ally_step <= -300) //Y���Ӳ������1140
						{
							protect_flag = 1;	
						}
						//�½����̴����ϵ�λ��			
						STEPPER_CUTTER_FOOTER = 1;
						delay_ms(200);
				#if SUPPORT_UNIFY_DRIVER						
						quickmove_yj_process(300,-400);//155
						delay_ms(500);
						for(i=0;i<tmp_speed;i++)
						{
							delay_ms(1);
							if( check_yj_done() )
							break;
						}
				#else		
						send_dsp2_command(0x0000,0x0003);
						send_dsp_command(DSP2,155);	
					    dsp2_quickmove_x(600);
						delay_ms(200);
				#endif						
						go_origin_stepper_cutter();
						delay_ms(250);				
						for( i=0;i< stepper_cutter_position;i++)
						{
							movestep_lct(1,0);
							//delay_us(600);
							delay_ms(1);
						}					
						milling_cutter_stop_flag = 0;
					}
			
					move_flag = 0;
					nopmove_flag = 0;
					allx_step = allx_step + xstep_cou;                        
	  			    ally_step = ally_step + ystep_cou;
					x_tmp = fabsm( xstep_cou);
					y_tmp = fabsm( ystep_cou);
					slow_flag = 0;
					if( (x_tmp<=2)&&(y_tmp <=2) )
					{	
						slow_flag =1;
						timer_y = 2;
						timer_x = 2;	
					}
					else
					{
						if( x_tmp > y_tmp)
						    tmp_speed =	spdlimit1_tab[x_tmp-1];
						else
						    tmp_speed =	spdlimit1_tab[y_tmp-1];
						timer_x = MoveTime1_Speed[tmp_speed/100]+cutter_syn_delay;
						timer_y = timer_x;
						dly_time = timer_x;
					}
				
								
				    tmp = stepper_cutter_shake_rage;
					if( y_tmp > x_tmp )
					    sum_x = y_tmp;
					else
						sum_x = x_tmp;
						
					if( sum_x <=100 )//<5mm
					{
						dly_time = 15+stepper_cutter_delay;
					}
					else if( sum_x <=120 )//<6mm
					{
						dly_time = 20+stepper_cutter_delay;
					}
					else if( sum_x <=160 )//<7mm
					{
						dly_time = 23+stepper_cutter_delay;
					}
					else if( sum_x <=170 )//<8mm
					{
						dly_time = 25+stepper_cutter_delay;
					}
					else if( sum_x <=180 )//<9mm
					{
						dly_time = 27+stepper_cutter_delay;
					}
					else //if( sum_x <=120 )//<6mm
					{
						dly_time = 30+stepper_cutter_delay;
					}
						
					movestep_lct(-tmp,dly_time);
					delay_ms(dly_time);
					
					movestep_lct(stepper_cutter_shake_rage,10 + dly_time);
					delay_ms(5);
					
					if(y_tmp > 0)
					{
						timer_y = 25;
						movestep_y(-ystep_cou); 
						delay_us(300);
					}
					if( x_tmp > 0)
					{
						timer_x = 25;
						movestep_x(-xstep_cou);
						delay_us(300);
					}
					delay_ms(20);						
			}
		
			if(nopmove_flag == 1)
			{
			    while( nopmove_flag == 1 )
				{
					delay_ms(200);
					for( i=0;i< stepper_cutter_position;i++)
					{
						movestep_lct(-1,0);
						delay_us(600);
					}					
					go_origin_stepper_cutter();
					delay_ms(10);
					STEPPER_CUTTER_FOOTER = 0;
					delay_ms(100);
					milling_cutter_stop_flag = 1;
					do_pat_point_sub_one();
					sys.status = READY;					
					
				    go_beginpoint(1); //�ֶο��ʹ���					
										
					sys.status = RUN;
					process_data();	
					if(OutOfRange_flag == 1)
					{
					   end_flag = 1;
					   sys.status = ERROR;
					   sys.error = ERROR_15;
					   status_now = READY;
					   break;
					}
					if( nopmove_flag != 1)
					{
						do_pat_point_sub_one();
						nopmove_flag = 0;
						move_flag = 0;
						break;
					}
				}
			}
			
			if(end_flag == 1)
			{
				if(inpress_flag == 0)  
				   inpress_up();
				break;
			}
			
			if( milling_cutter_action_flag == 2)//����������
			{
				if( (x_bios_offset!=0)||(y_bios_offset!=0) )//ƫ�Ʋ�Ϊ0
				{
					 allx_step_tmp = allx_step;
				     ally_step_tmp = ally_step;			
				     go_commandpoint(allx_step-x_bios_offset,ally_step- y_bios_offset);						 
					 allx_step = allx_step_tmp;//Ϊ��ʾ�Ĺ����ȷ
					 ally_step = ally_step_tmp;
					 delay_ms(300);
					 milling_first_move = 0;
				}			
			}
		}
		if(end_flag == 1)
		{
			if(inpress_flag == 0)  
			   inpress_up();
			break;
		}
	}
	//delay_ms(100);
	for( i=0;i< stepper_cutter_position;i++)
	{
		movestep_lct(-1,0);
		delay_us(600);
	}	
	go_origin_stepper_cutter();
	STEPPER_CUTTER_FOOTER = 0;
	delay_ms(100);
		 
	//if( (milling_first_move == 1) && (k115 ==1) &&( (x_bios_offset!=0)||(y_bios_offset!=0) ) )
	if( (milling_first_move == 1)  &&( (x_bios_offset!=0)||(y_bios_offset!=0) ) )
	{ 
		 allx_step_tmp = allx_step;
		 ally_step_tmp = ally_step;			
		 go_commandpoint(allx_step-x_bios_offset,ally_step- y_bios_offset);	 		 
		 allx_step = allx_step_tmp;
		 ally_step = ally_step_tmp;	 
		 delay_ms(200);
		 milling_first_move = 0;
	}
	

	if( end_flag == 1)
	{
		origin_com = 0;
		sys.status = FINISH;
		StatusChangeLatch = FINISH;
	}
	if( sys.status == ERROR)
	{
		StatusChangeLatch = ERROR;
	}
}
#endif
//========================================================================
void rotated_cutter_single_next(void)
{
	INT32 allx_step_tmp,ally_step_tmp;
	if( milling_first_move == 0)
	{
		if( (x_bios_offset!=0)||(y_bios_offset!=0) )//ƫ�Ʋ�Ϊ0
		{
		     allx_step_tmp = allx_step;
		 	 ally_step_tmp = ally_step;
			 go_commandpoint(x_bios_offset + allx_step,y_bios_offset + ally_step);	  
			 delay_ms(300);
			 allx_step = allx_step_tmp;
		 	 ally_step = ally_step_tmp;
		}
		go_origin_rotated_cutter();//��ת������ҵ�ԭ��
	    delay_ms(50);
		
		rotated_cutter_by_data();		
		delay_ms(100);
		DRILL_MOTOR_UPDOWN = 1;  //ϳ��������		
		delay_ms(rotated_cutter_up_delay);	
		//drill_motor_run_enable = 1;     //  �ŵ������ϣ�ϳ����ʼ��ת
		milling_first_move = 1;
	}
	else
	{
	    rotated_cutter_by_data();
		delay_ms(10);
		if( (milling_cutter_stop_flag == 1)&&(drill_motor_updown_flag == 0) )
		{
			drill_motor_updown_flag = 1;
			//milling_cutter_stop_flag = 0;	//2017-6-17		
			//DRILL_FOOTER = 1;
			//delay_ms(rotated_cutter_running_delay);
			DRILL_MOTOR_UPDOWN = 1;  //ϳ��������
			delay_ms(rotated_cutter_up_delay);	
			//drill_motor_run_enable = 1;     //  �ŵ������ϣ�ϳ����ʼ��ת
		}
	}
}
void rotated_cutter_single_back(void)
{
	rotated_cutter_by_data();
}
void rotated_cutter_single_stop(void)
{
	UINT16 j;
	delay_ms(100);
	drill_motor_run_enable = 0; //ϳ���ص�	
	if( para.rotate_cutter_working_mode == 55)
	{
		if( rotated_cutter_running_flag == 1)
		{
			//send_dsp_command(DSP2,0x0009);//��ͣ����
			//for(j=0;j<500;j++)
			//{
			//	delay_ms(1);
			//	if( check_yj_done() )
			//	   break;
		//	}
		 set_rotated_cutter_speed( 0);
			rotated_cutter_running_flag = 0;
			//go_origin_yj();
		}
	}
	drill_motor_updown_flag = 0;
	#if DSP3_CUTER_DRIVER
		output_cs3(2,2);
		delay_ms(200);
		output_cs3(2,0);
	#endif

	//DRILL_MOTOR_UPDOWN = 0;//ϳ������ȥ
	rotated_cutter_down_positon();
	delay_ms(500);
	DRILL_FOOTER = 0;
	delay_ms(100);
	milling_cutter_stop_flag = 1;
}
/*
������������Ŀ����֣�
mmm xdddddd  pppppp
mmm---������ 
dddddd-----�˵�ţ��������
pppppp-----�����ƽ,��Ϊ1,�ر�Ϊ0
���ڵ���Ŀ�����
mmm x ddddddd ttttt
mmm---������  x--����  ddddddd--7λλ�� ttttt-5λʱ��
001---��һ·90V���� 	CZ7411  X34
010---�ڶ�·90V���� 	CZ7412  X33
011---�ڶ�·7078     	CZ749   X30
100---��һ·7078��� 	CZ7410  X29
101---6064����������  CZ7413  X28
110---�������          CZ748   X31
						1-4  AIR1 ---PORTF4  --BIT0   1100 0000 01 00 000x
						2-5  AIR2 ---PORTF3  --BIT1   1100 0000 10 00 00x0
						3-6  AIR3 ---PORTF2  --BIT2   1100 0001 00 00 0x00
110---�������          CZ749   X32
						1-4  AIR4 ---PORTB6  --BIT3   1100 0010 00 00 x000
						2-5  AIR5 ---PORTC0  --BIT4   1100 0100 00 0x 0000
						3-6  AIR6 ---PORTB5  --BIT5   1100 1000 00 x0 0000
mmm xdddddd  pppppp          


X34    0X2000    ����۵��  0.36d
X33    0X4000    ץ���ߵ��/��ת�е��Ƕȵ��/ 0.9d
X28    0XA000    ���̵�� 0.9d

*/
void movestep_cs3(UINT16 command,INT16 x_data,UINT8 timer_need)
{ 
	#if MULTIPULE_IO_ENABLE == 1
	UINT8 dir;
	while(spi_flag > 0);
    
	if( timer_need >= 63 )
	    timer_need = 63;
		
	if( x_data !=0 )
	{
	   SPISTE3 = 0;
	   SPISTE1 = 1;
	   SPISTE2 = 1;
	   SPISTE4 = 1;
	}
	else 
		return;
	if( (command == 0x2000)||(command == 0x4000) ) //����۵��/��ת�е����
	{
		command = 0x0000;//��һ·���
		dir = para.dsp3a_motor_dir;
	}
	else if( command == 0xA000) //���̵��
	{
		command = 0x8000;//�ڶ�·���
		dir = para.dsp3b_motor_dir;
	}
	if(x_data > 0)
	{   
		spi_flag = 1;	
		trans_x.word = command + ((UINT16)x_data<<6)+(UINT16)timer_need;
		if( dir == 0)
	       	trans_x.word += 0x4000; 
	}	
	else if(x_data<0)
	{
		x_data_nf = -x_data;
		spi_flag = 1;
		trans_x.word= command + ((UINT16)x_data_nf<<6)+(UINT16)timer_need;	
		if( dir != 0)
	    	trans_x.word += 0x4000; 
	}
	if( trans_x.word == 0x5555 )
	{
		trans_x.word = trans_x.word + 1;
	}
	trans_y.word=(~trans_x.word)&0x7fff;
	trans_z.word=0x5555;
	dsp3 = 1;
	SPISTE3=0;   
	s4trr=trans_x.byte.byte1;  
	
	#else
 	while(spi_flag > 0);
    if( timer_need >= 32 )
	    timer_need = 31;
	if( x_data !=0 )
	{
	   SPISTE3 = 0;
	   SPISTE1 = 1;
	   SPISTE2 = 1;
	}
	if(x_data>0)
	{   
		spi_flag=1;	
		if( dsp3_moto1_direction == 0)
	        trans_x.word=command+((UINT16)x_data<<5)+(UINT16)timer_need;
		else
			trans_x.word=(command|0x1000) + ((UINT16)x_data<<5)+(UINT16)timer_need; 
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		{
			trans_x.word = trans_x.word + 1;
		}
		trans_y.word=(~trans_x.word)&0x7fff;
	    trans_z.word=0x5555;
		dsp3 = 1;
		SPISTE3=0;   
		s4trr=trans_x.byte.byte1;  
	}	
	else if(x_data<0)
	{
		x_data_nf=-x_data;
		spi_flag=1;	
		if( dsp3_moto1_direction == 0)
	    	trans_x.word=(command|0x1000) + ((UINT16)x_data_nf<<5)+(UINT16)timer_need; 
		else
			trans_x.word= command + ((UINT16)x_data_nf<<5)+(UINT16)timer_need; 
	    if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		{
			trans_x.word = trans_x.word + 1;
		}		   
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp3 = 1;
		SPISTE3=0;		             
		s4trr=trans_x.byte.byte1;    
	}
	#endif
}
//DSP3��� ��6λΪ�ߵ͵�ƽ������6λ��ָ���˿�
//X31
//1-4  AIR1 ---PORTF4  --BIT0   1100 0000 01 00 000x  1 ������ѹѹ��
//2-5  AIR2 ---PORTF3  --BIT1   1100 0000 10 00 00x0  2 �ŷ�������26,34���ţ�����λ
//3-6  AIR3 ---PORTF2  --BIT2   1100 0001 00 00 0x00  4 ������
//X32
//1-4  AIR4 ---PORTB6  --BIT3   1100 0010 00 00 x000  8      �Ǻű�
//2-5  AIR5 ---PORTC0  --BIT4   1100 0100 00 0x 0000  0X10   ����
//3-6  AIR6 ---PORTB5  --BIT5   1100 1000 00 x0 0000  0X20   �е���λ
void output_cs3(UINT8 x_data,UINT8 timer_need)
{ 
 	UINT8 data,cmd,i;
	if( x_data == 0)
	    return;
	while(spi_flag > 0);
	
	#if MULTIPULE_IO_ENABLE == 1
	
	send_dsp4_command(0x1000,0x20);//дSTM32�Ĵ���
	if( timer_need == 0 )
		data = 0;
	else 
		data = 1;
	switch( x_data)
	{
		case 1:
			cmd = 3;//ѹ��STM32����4
		break;
		case 2:
			cmd = 1;//ͣ����λSTM32����2
		break;
		case 4:
			cmd = 2;//������STM32����3
		break;
		case 8:
			cmd = 4;
		break;
		case 0x10:
			cmd = 5;
		break;
		case 0x20:
			cmd = 6;
		break;
	}
	send_dsp4_command(cmd,data);

	#else
	SPISTE3 = 0;
	delay_us(100);
	SPISTE1 = 1;
	SPISTE2 = 1;
	spi_flag=1;	
	trans_x.word = 0xc000 + ((UINT16)x_data<<6)+timer_need;
	trans_y.word=(~trans_x.word)&0x7fff;
	trans_z.word=0x5555;
	dsp3 = 1;
	SPISTE3 = 0;   
	s4trr=trans_x.byte.byte1;  
	#endif
}


//#if ENABLE_BOBBIN_CASE_FUN
//�һ�е��ԭ��
//001x
/*
��е�۶�λ�̽Ƕ�220�ȣ���Ӧ����λ�ã���ͷ��ץȡλ�� �� �����̶Խ�λ��
��Ƭ��סʱ����е����ʱ��ת���ӵ�סһֱ���˳���Ƭ ��Ӧ��λ���� �����̶Խ�λ��
            ��е��˳ʱ��ת���ӵ�סһֱ���˳���Ƭ ��Ӧ��λ���� ��ͷ��ץλ��
��Ƭδ��סʱ�����˳ʱ��ת��
ԭ��Ĭ���� �����̶Խ�λ��

bobbin_case_arm_position
0           �ϵ�û�ҹ�ԭ��
50          λ���ڻ�ͷ
100         λ��������
pos 
0-------������ת
1-------���ͷת
��Ƭ���ڵ�����ϣ���һ��Ƥ�����������Ի�е�ۺ͵�Ƭ����ת�����෴��������˵�ķ����ǻ�е�۵ķ���
*/
void go_origin_bobbin_case_arm(UINT8 pos)
{
	UINT8 i,ret;
	UINT16 temp16;
	
	if( bobbin_case_arm_position == 0) //�ϵ�û�ҹ�ԭ��
	{
		if( get_bobbin_case_arm_org_status() ==1)//���Ƭû�е��ϣ��ڹ�����Χ֮����
		{
			ret = find_a_bobbin_case(1);//���Ҹ��������
			if(ret == 0)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				if(sys.error == 0)    
	      		   sys.error = ERROR_51;
				return;
			}
			//��˳ʱ���ҵ���Ƭ
			temp16 = 0;
			while( get_bobbin_case_arm_org_status() == 1)
			{
				temp16 = temp16 + 1;
				movestep_cs3(0x2000,1,1);   
				delay_ms(5);
				#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
	  	  		if(temp16 > 1600)// 0.45�����
				#else
				if(temp16 > 800)// 0.45�����
				#endif
		  	  	{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
					if(sys.error == 0)    
	      			   sys.error = ERROR_49;     
	      			return;
				}
			}
			//����������λ�ã�ץһ�¿���
			BOBBIN_CASE_ARM_SCRATH = 0;  //��е���ɿ�
			BOBBIN_CASE_ARM_OUT = 1;     //��е�����ȥ
			delay_ms(bobbin_case_inout_delay);
			#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			movestep_cs3(0xa000,para.bobbin_shake_distance,para.bobbin_shake_time);
			#else
			movestep_cs3(0xa000,50,5);//shake_move1			
			#endif
			delay_ms(250);
			BOBBIN_CASE_ARM_SCRATH = 1;  //��е�ּн�
			#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			movestep_cs3(0xa000,-para.bobbin_shake_distance,para.bobbin_shake_time);
			#else
			movestep_cs3(0xa000,-50,5);//shake_move2
			#endif
			delay_ms(200);
			BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
			delay_ms(300);
			if( BOBBIN_CASE_EMPTY_CHECK == 0)//ץ�����ˣ�λ����ȷ
			{
				BOBBIN_CASE_ARM_OUT = 1;     //��е�����ȥ
				delay_ms(bobbin_case_inout_delay);
				BOBBIN_CASE_ARM_SCRATH = 0;  //��е���ɿ�
				delay_ms(200);
				BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
				delay_ms(300);
			}
			else //û�ҵ�
			{
				temp16 = 0;
				for(i = 0;i<20;i++)
			    {
					movestep_cs3(0x2000,-1,1);   
					#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
					delay_ms(2);
					#else
					delay_us(600);
					#endif
				 }
				while( get_bobbin_case_arm_org_status() == 1)
				{
					temp16 = temp16 + 1;
					movestep_cs3(0x2000,-1,1);   
					delay_ms(5);
					#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
		  	  		if(temp16 > 1600)// 0.45�����
					#else
					if(temp16 > 800)// 0.36�����
					#endif
			  	  	{
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
						if(sys.error == 0)    
		      			   sys.error = ERROR_49;     
		      			return;
					}
				}
			}
			bobbin_case_arm_position = 100;
		}
		else //�ڵ������
		{
			if( pos == 0 ) //Ҫ������
			    bobbin_case_arm_position = 50;
			else
			    bobbin_case_arm_position = 100;
		}
	}
	
	if ( (pos == 0 )&&(bobbin_case_arm_position == 50) )//�� ��ͷ �� ���� ��ת У������
	{
		temp16 = 0;
		while( get_bobbin_case_arm_org_status() == 1)//�����Ƭû����
		{
			movestep_cs3(0x2000,-1,1);   //��ʱ��
			#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			delay_ms(1);
			#else
			delay_us(300);
			#endif
			temp16 = temp16 + 1;
  	  		#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
		  	if(temp16 > 1600)// 0.45�����
			#else
			if(temp16 > 800)// 0.36�����
			#endif
	  	  	{
	  	  			sys.status = ERROR;
					StatusChangeLatch = ERROR;
					if(sys.error == 0)    
	      			   sys.error = ERROR_49;     
	      			return;
	  	  	}

		}
		for(i=0;i<5;i++)//���߼���ȷ��
		{
			movestep_cs3(0x2000,-1,1);   //��ʱ��
			delay_ms(1);
		}
	}
	if ( (pos == 1 )&&(bobbin_case_arm_position == 100) )//�� ���� �� ��ͷ ��ת У������
	{
		temp16 = 0;
		while( get_bobbin_case_arm_org_status() == 1)
		{
			movestep_cs3(0x2000,1,1);  //˳ʱ��
			#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			delay_ms(1);
			#else
			delay_us(300);
			#endif
			temp16 = temp16 + 1;
			#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
  	  		if(temp16 > 1600)
			#else
			if(temp16 > 800)
			#endif
	  	  	{
	  	  			sys.status = ERROR;
					StatusChangeLatch = ERROR;
					if(sys.error == 0)    
	      			   sys.error = ERROR_49;     
	      			return;
	  	  	}
	
		}
	}
	
	if( get_bobbin_case_arm_org_status() == 0)//�����Ƭ�Ѿ���ס��
	{	
		temp16 = 0;
		//������һ�ο���
		
		while(get_bobbin_case_arm_org_status() == 0)
	   	{
			if( pos == 0)//����
				movestep_cs3(0x2000,-1,1);   //��ʱ��
			else
			    movestep_cs3(0x2000,1,1);  //˳ʱ��
			#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			delay_ms(1);
			#else
			delay_us(300);
			#endif			

			temp16 = temp16 + 1;
  	  		#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
  	  		if(temp16 > 1600)
			#else
			if(temp16 > 800)
			#endif
	  	  	{
	  	  			sys.status = ERROR;
					StatusChangeLatch = ERROR;
					if(sys.error == 0)    
	      			   sys.error = ERROR_49;     
	      			return;
	  	  	}
	
		}
		for(i=0;i<15;i++)//���߼���ȷ���˳���
		{
			if( pos == 0)//����
				movestep_cs3(0x2000,-1,1);   //��ʱ��
			else
			    movestep_cs3(0x2000,1,1);  //˳ʱ��
			#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			delay_ms(1);
			#else
			delay_us(600);
			#endif
		}
	}
	
	temp16 = 0;
	while(get_bobbin_case_arm_org_status() == 1)//�ҵ����ϵ��� =1��ʾû���ϣ���һ�����ţ�
	{
		if( pos == 0)//����
			movestep_cs3(0x2000,1,1); 
		else
			movestep_cs3(0x2000,-1,1);
		delay_ms(8);
		
		temp16 = temp16 + 1;
  	  	#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
  	  		if(temp16 > 1600)
			#else
			if(temp16 > 800)
			#endif
	  	{
	  	  	sys.status = ERROR;
			StatusChangeLatch = ERROR;
			if(sys.error == 0)    
	      	   sys.error = ERROR_49;     
	      	return;
	  	}
	}
	
	if( pos == 0)//����
	{
		if( bobbin_case_platform_offset != 0 )
		{
			movestep_cs3(0x2000,bobbin_case_platform_offset,31); 
			delay_ms(70);
		}
		bobbin_case_arm_position = 100; //��ǰ������
	}
	else
	{
		if( bobbin_case_arm_offset != 0 )
		{
			movestep_cs3(0x2000,bobbin_case_arm_offset,31);
			delay_ms(70);
		}
		bobbin_case_arm_position = 50; //��ǰ�ڻ�ͷ
	}
	
}

//��һ������λ��
/*
 �������ת���򣺸���-����˳ʱ����ת ����-������ʱ����ת
 ԭ�㴫������ �ߵ�ƽ ��ʾ�ڵ�Ƭȱ����  �͵�ƽ��ʾ�������ˡ� ������0528 NPN��©  ����ʱΪ�ߵ�ƽ
 �����һ�׼λ���ǣ� 
  1�����˳ʱ��ת������һ����Ƭ�ɵ�������������
  2���ҵ�ǰ��λ�������ʱ��ת�ҵ����ϵ�λ�ã�Ȼ����˳ʱ��ת����������
  full=0 ��ʾ�ҿ�λ����0��ʾҪ������о��λ��
*/
UINT8 find_a_bobbin_case(UINT8 full)
{
	UINT8 i,j;
	UINT16 temp16;
	//�Ӹ�ƫ��λ��
	if( bobbin_plateform_org_offset !=0 )
	{
		movestep_cs3(0xa000,-bobbin_plateform_org_offset,30);
		delay_ms(100);
	}
	
	for( j=1; j<=8 ; j++)//һȦ���8����о
	{
		//Ҫ��һ������о�������Ѿ���ȱ������ҿ�λ
		if( (full == 0)&&(BOBBIN_CASE_PLATFORM_ORG )&&(BOBBIN_CASE_EMPTY_CHECK == 0) )//Ҫ�ҿ�λ���ֵ�ǰ���ǿ�λ
		{
				temp16 = 0;
				while(BOBBIN_CASE_PLATFORM_ORG != 0)
			   	{
					#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
					movestep_cs3(0xa000,1,1);//��������ȱ�����˳���
					delay_ms(para.bobbin_platform_speed);
					#else
					movestep_cs3(0xa000,8,5);//��������ȱ�����˳���		
					delay_ms(1);			
					#endif					
					temp16 = temp16 + 1;
		  	  		if( temp16 > 20000)
			  	  	{
			  	  		sys.status = ERROR;
						StatusChangeLatch = ERROR;
						if(sys.error == 0)    
			      		   sys.error = ERROR_50;     
			      		return 0;
			  	  	}
				}
		}
		else
		{
			if( BOBBIN_CASE_PLATFORM_ORG !=0 )//����Ѿ���ȱ��
			{
				temp16 = 0;
				while(BOBBIN_CASE_PLATFORM_ORG != 0)
			   	{
					#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
					movestep_cs3(0xa000,-1,1);//����ǰ���������һ��
					delay_ms(para.bobbin_platform_speed);
					#else
					movestep_cs3(0xa000,-8,5);//����ǰ���������һ��	
					delay_ms(1);				
					#endif
					
					temp16 = temp16 + 1;
		  	  		if( temp16 > 20000)
			  	  	{
			  	  		sys.status = ERROR;
						StatusChangeLatch = ERROR;
						if(sys.error == 0)    
			      		   sys.error = ERROR_50;     
			      		return 0;
			  	  	}

				}
				for( i=0; i<3; i++)
				{
					#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
					movestep_cs3(0xa000,-1,1);//�����һ��
					delay_ms(para.bobbin_platform_speed);
					#else
					movestep_cs3(0xa000,-8,5);//�����һ��
					delay_ms(1);
					#endif
					
				}
			}
		}
		//��������϶����ڵ��ϵ�״̬��û���뵽ȱ����ҿ�λ���ҵ�ǰ��λ�ͻ��ˣ�����ֱ���ߵ�����Ϊֹ��
		temp16 = 0;
		while(BOBBIN_CASE_PLATFORM_ORG == 0)//��Ƭ�����ˣ���ȱ�ڷ�����
		{
					#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
					movestep_cs3(0xa000,-1,1);//���˳ʱ����һ�����ҵ�������������
					delay_ms(para.bobbin_platform_speed);
					#else
					movestep_cs3(0xa000,-2,5);//���˳ʱ����һ�����ҵ�������������
					delay_ms(1);
					#endif
					temp16 = temp16 + 1;
		  	  		if( temp16 > 20000)
			  	  	{
			  	  		sys.status = ERROR;
						StatusChangeLatch = ERROR;
						if(sys.error == 0)    
			      		   sys.error = ERROR_50;     
			      		return 0;
			  	  	}

					if ( BOBBIN_CASE_PLATFORM_ORG !=0 )
					    delay_ms(100);
		}
		//�Ӹ�ƫ��λ��
		if( bobbin_plateform_org_offset !=0 )
		{
			movestep_cs3(0xa000,bobbin_plateform_org_offset,30);
		}
		
			if( full == 1)//�Ҹ�����о��
			{
				if( BOBBIN_CASE_EMPTY_CHECK == 1)//���źŷ���
				{
					delay_ms(100);
					if( BOBBIN_CASE_EMPTY_CHECK == 1)
					{
						//bobbin_case_platform_position = (bobbin_case_platform_position + j)%9;
						//return bobbin_case_platform_position;
						return 1;
					}
				}
			}
			else if( full == 0)
			{
				if( BOBBIN_CASE_EMPTY_CHECK == 0)//����
				{
					delay_ms(100);
					if( BOBBIN_CASE_EMPTY_CHECK == 0)
					{
						//bobbin_case_platform_position = (bobbin_case_platform_position + j)%9;
						//return bobbin_case_platform_position;
						return 1;
					}
				}
			}
			else
			  break;
	}
	return 0;
}

/*
ͣ�����ض�λ��
*/
void bobbin_case_motor_adjust(void)
{	 
	UINT8 temp8;	
	INT16 temp16;	
	while(motor.stop_flag == 0)    
  	{
    	rec_com(); 
  	}

	temp16 = motor.angle_adjusted;
	if( (temp16 >256)||(temp16 <200) )
	{
		inpress_down(inpress_high_hole);
		motor.dir = 0;
		motor.spd_obj = 100;
		while(1)
	   	{
	    		rec_com(); 
		    	if(motor.spd_ref == motor.spd_obj)
		    	{
			    	break;
		    	}
	   	}
		motor.stop_angle = 228;//80d
		motor.spd_obj = 0;  
	    while(motor.stop_flag == 0)    
		{
		  rec_com(); 
	     
		}
		delay_ms(280);
		inpress_up();
		delay_ms(80);
	}
}
//�� �ӻ�ͷȡ�� �� ѡһ������
UINT8 bobbin_case_workflow1(void)
{
	UINT8 empty,full,j,i,k;
	#if MACHINE_900_BOBBIN_DEBUG_MODE
	#else
	bobbin_case_motor_adjust();
	#endif
	
	for( i = 0; i<8 ; i++)
	{
		BOBBIN_CASE_ARM_SCRATH = 0;  //��е���ɿ�
		BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
		go_origin_bobbin_case_arm(1);//��ת����ͷ��ץλ��
		delay_ms(100);
		BOBBIN_CASE_ARM_OUT = 1;     //��е�����ȥ
		delay_ms(bobbin_case_inout_delay);
		BOBBIN_CASE_ARM_SCRATH = 1;  //��е�ּн�
		delay_ms(bobbin_case_scrath_delay);
		BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
		delay_ms(500);
		if( bobbin_case_workmode == 0)//����о�ŵ�����
		{
			go_origin_bobbin_case_arm(0);//ת�����̶�Ӧλ��
			delay_ms(100);
			empty = find_a_bobbin_case(0);
			if( empty == 0 )
			{
				//�����˳�
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				if(sys.error == 0)    
	      		   sys.error = ERROR_51;
				return 0;
			}
			BOBBIN_CASE_ARM_OUT = 1;     //��е�����ȥ
			delay_ms(bobbin_case_inout_delay);
			#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			movestep_cs3(0xa000,para.bobbin_shake_distance,para.bobbin_shake_time);
			#else
			movestep_cs3(0xa000,90,31);//shake_move1
			#endif
			delay_ms(250);
			BOBBIN_CASE_ARM_SCRATH = 0;  //��е���ɿ�
			delay_ms(300);
			#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			movestep_cs3(0xa000,-para.bobbin_shake_distance,para.bobbin_shake_time);
			#else
			movestep_cs3(0xa000,-90,31);//shake_move2
			#endif
			delay_ms(200);
			BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
			delay_ms(200);
			if( BOBBIN_CASE_EMPTY_CHECK == 0)//������ֵ�ǰλ��û���źţ����п�����Ϊ��ûץ����
			{
				if( i< 3)//��ץ3��
				  continue;
				else     //ֱ�Ӵ�����ץһ��
				{
				}
			}
		}
		else //����
		{
			go_origin_bobbin_case_arm(0);//ת���ռ�λ��
			delay_ms(50);
			for(k=0; k<bobbin_case_dump_position; k++)//��̽����50������
			{
				movestep_cs3(0x2000,-1,1);   //��ʱ��
				delay_ms(2);
			}
			BOBBIN_CASE_ARM_OUT = 1;     //��е�����ȥ
			delay_ms(bobbin_case_inout_delay);
			BOBBIN_CASE_ARM_SCRATH = 0;  //��е���ɿ�
			delay_ms(200);
			BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
			delay_ms(200);
			go_origin_bobbin_case_arm(0);//ת���ռ�λ��
			delay_ms(100);
		}

			//������Ǻ�ȷ�ϴ�����û�б��ζ������
			k=0;
			while((BOBBIN_CASE_PLATFORM_ORG == 0)&&(k<20) )//��Ƭ������
			{
				movestep_cs3(0xa000,-2,5);//���˳ʱ����һ�����ҵ�������������
				delay_ms(1);
				k++;
			}
			for( j = 0; j<8 ; j++)
			{
				full = find_a_bobbin_case(1);
				if( full == 0)
				{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
					if(sys.error == 0)    
		      		   sys.error = ERROR_51;
					return 0;
			    } 
				delay_ms(200);
				BOBBIN_CASE_ARM_OUT = 1;     //��е�����ȥ
				delay_ms(bobbin_case_inout_delay);
				#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
				movestep_cs3(0xa000,para.bobbin_shake_distance,para.bobbin_shake_time);
				#else
				movestep_cs3(0xa000,90,31);//shake_move1
				#endif
				delay_ms(250);
				BOBBIN_CASE_ARM_SCRATH = 1;  //��е�ּн�
				delay_ms(bobbin_case_scrath_delay);
				#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
				movestep_cs3(0xa000,-para.bobbin_shake_distance,para.bobbin_shake_time);
				#else
				movestep_cs3(0xa000,-90,31);//shake_move1
				#endif
				delay_ms(250);
				BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
				delay_ms(300);
				//�˲�һ��
				if( BOBBIN_CASE_EMPTY_CHECK == 1)//���źŷ���,û�ҳ���
				{
					BOBBIN_CASE_ARM_SCRATH = 0;  //��е���ɿ�
					go_origin_bobbin_case_arm(0);//ת�����̶�Ӧλ��
					delay_ms(200);
				    continue;
				}
				go_origin_bobbin_case_arm(1);//ת����ͷ�Խ�λ��
				delay_ms(100);
				BOBBIN_CASE_ARM_OUT = 1;     //��е�����ȥ
				delay_ms(1000);
				BOBBIN_CASE_ARM_SCRATH = 0;  //��е���ɿ�
				delay_ms(200);
				BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
				delay_ms(500);
				if( bobbin_case_stop_position == 0)
				    go_origin_bobbin_case_arm(0);//ת�����̶�Ӧλ��
				break;
			}
		break;
	  }	
	return 1;
}

//========================================================================================
void stepmotor_para(void)   
{
	 UINT8 k;	 
	 if( debug_dsp_flag == 1)
		 send_dsp_command(DSP2,0x000a);
	 else
	     send_dsp_command(DSP1,0x000a);  
		 
	for(k=0;k<93;k++)
	{
		if( debug_dsp_flag == 1)
		    send_dsp_command(DSP2,((UINT16)k<<8) + (UINT16)svpara_buf[k]);
		else
			send_dsp_command(DSP1,((UINT16)k<<8) + (UINT16)svpara_buf[k]);
		delay_us(1000);
	    while(spi_flag > 0);
	}
} 
void read_stepmotor_para(void)
{
    UINT8 m;	
	for(m=0;m<93;m++)
	{
		if( debug_dsp_flag == 1)
		    send_dsp2_command(0x0002,0x5555);
		else
			send_dsp1_command(0x0002,0x5555);
		svpara_disp_buf[m] = recieve_x.word;
	}
}


//return 0x1234 ---ok 
UINT8 check_motion_done(void)
{
	send_dsp1_command(0x0012,0x5555);		
	if( recieve_x.word == 0x1234 )
	    return(1);
	else
	    return(0);
}


/*
��������ͨ��SPI��������
*/
void send_stepmotor_up_drv(void)    
{
	UINT8 k;
	if( download_drv_flag==1)
	    send_dsp_command(DSP1,0x00f0);
	else if( download_drv_flag==2)
	    send_dsp_command(DSP2,0x00f0);
	else if( download_drv_flag==3)
	    send_dsp_command(DSP3,0x00f0);
	else if( download_drv_flag==4)
	    send_dsp_command(DSP4,0x00f0); 
		
	for(k=0;k<data_length_drv+2;k++)
	{
		if( download_drv_flag==1)
	    	send_dsp_command(DSP1,((UINT16)k<<8) + pat_buf[k]);
		else if( download_drv_flag==2)
	    	send_dsp_command(DSP2,((UINT16)k<<8) + pat_buf[k]);	
		else if( download_drv_flag==3)
	    	send_dsp_command(DSP3,((UINT16)k<<8) + pat_buf[k]);
		else if( download_drv_flag==4)
	    	send_dsp_command(DSP4,((UINT16)k<<8) + pat_buf[k]);	
		rec_com();//2016-10-28
	}
} 

void send_stepmotor_end_drv(void)    
{
	 if( download_drv_flag==1)
	  	 send_dsp_command(DSP1,0x00ff);
	 else if( download_drv_flag==2)
	 	 send_dsp_command(DSP2,0x00ff);
	 else if( download_drv_flag==3)
	 	 send_dsp_command(DSP3,0x00ff);
	 else 
	 	 send_dsp_command(DSP4,0x00ff);
} 
/*
�ض�����״̬
*/
UINT16 read_stepmotor_up_drv(void)
{
	 if( download_drv_flag==1)
	     send_dsp1_command(0x0010,0x5555);
	 else if( download_drv_flag==2)
	 	 send_dsp2_command(0x0010,0x5555);
	 else if( download_drv_flag==3)
	 	 send_dsp3_command(0x0010,0x5555);
	 else if( download_drv_flag==4)
	 	 send_dsp4_command(0x0010,0x5555);
		 
	 return recieve_x.word;
}
/*
����������״̬�£��ɲ�������л�״̬������
֪ͨ����׼����������״̬
*/
void jump_to_begin(void)
{
    if((1==download_drv_flag)&&(stepversion1<60000))
	{
	    send_dsp_command(DSP1,0x000f);		
		delay_ms(1000);
	}
	if((2==download_drv_flag)&&(stepversion2<60000))
	{
	    send_dsp_command(DSP2,0x000f);		
		delay_ms(1000);
	}
	if((3==download_drv_flag)&&(stepversion3<60000))
	{
	    send_dsp_command(DSP3,0x000f);		
		delay_ms(1000);
	}
	if((4==download_drv_flag)&&(stepversion4<60000))
	{
	    send_dsp_command(DSP4,0x000f);		
		delay_ms(1000);
	}
}
//==============================================================
void select_dsp( UINT8 port)
{
	dsp1 = 0;
	dsp2 = 0;
	dsp3 = 0;
	dsp4 = 0;
	SPISTE1 = 1;
	SPISTE2 = 1;
	SPISTE3 = 1;
	SPISTE4 = 1;
	if( port == DSP1)      //dsp1
	{
	    SPISTE1 = 0;
		dsp1 = 1;		
	}
	else if( port == DSP2) //dsp2
	{
	 	SPISTE2 = 0;
		dsp2 = 1;
	}
	else if( port == DSP3)  //dsp3
	{
		SPISTE3 = 0;
		dsp3 = 1;
	}
	else
	{
		SPISTE4 = 0;
		dsp4 = 1;
	}
}
void send_dsp1_command2(UINT16 command)
{
	while(spi_flag > 0);
	spi_flag=1;
	select_dsp(1);	
	trans_x.word = command;      
	trans_y.word = (~trans_x.word)&0x7fff;
	trans_z.word = 0x5555;
	s4trr=trans_x.byte.byte1;   
    delay_us(500);//800
	while(spi_flag > 0);
}

void send_dsp_command(UINT8 port,UINT16 command)
{
	while(spi_flag > 0);
	spi_flag = 1;
	select_dsp(port);	
	trans_x.word = command;      
	trans_y.word = (~trans_x.word)&0x7fff;
	trans_z.word = 0x5555;
	s4trr=trans_x.byte.byte1;   
//	if( sys.status != RUN)
    delay_us(800);//800
	while(spi_flag > 0);
}

void send_dsp_command_data(UINT8 port,UINT16 command,UINT16 data)
{
	send_dsp_command(port,command);
	send_dsp_command(port,data);
}

void send_dsp1_command(UINT16 command,UINT16 data)    
{
	send_dsp_command_data(DSP1,command,data);
} 
void send_dsp2_command(UINT16 command,UINT16 data)    
{
	send_dsp_command_data(DSP2,command,data);
} 

void send_dsp3_command(UINT16 command,UINT16 data)    
{
	send_dsp_command_data(DSP3,command,data);
} 
void send_dsp4_command(UINT16 command,UINT16 data)    
{
	send_dsp_command_data(DSP4,command,data);
} 

//===============================================
UINT16 check_DSP3_input(void)
{

	while(spi_flag > 0);
	spi_flag=1;
	select_dsp(DSP3);
	delay_us(100);	
	trans_x.word = 0x0018;      
	trans_y.word = (~trans_x.word)&0x7fff;
	trans_z.word = 0x5555;
	s4trr=trans_x.byte.byte1;   
	while(spi_flag > 0);
	spi_flag=1;
	select_dsp(DSP3);	
	delay_us(100);
	trans_x.word = 0x0018;      
	trans_y.word = (~trans_x.word)&0x7fff;
	trans_z.word = 0x5555;
	s4trr=trans_x.byte.byte1;
	while(spi_flag > 0);
	delay_us(100);			
	return recieve_x.word;					
}
/*
x32:5V
1-vcc 2-signal 3-gnd :0x01
4-vcc 5-signal 6-gnd :0x02
7-vcc 8-signal 9-gnd :0x04
24V
11-vcc 12-signal 13-gnd :0x08
14-vcc 15-signal 16-gnd :0x10
17-vcc 18-signal 19-gnd :0x20
*/
UINT8 get_bobbin_case_arm_org_status(void)
{
	#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
	if( BOBBIN_CASE_ARM_ORG == 1)
		return 1;
	else
		return 0;
	#else
	return (check_DSP3_input()&0x01);
	#endif
}

UINT8 check_autosewing_status(void)
{
	#if AUTO_CHANGE_PATTERN_FUNCTION ==  1
	//if( (ADTCSM)&&(PSENS ==1)&&(AUTO_FRAMEWORK_IN_POSITION==1) )//���������� ����2 ����5
	if( AUTO_FRAMEWORK_IN_POSITION==1 )//���������� ����2 ����5
	{
			return 1;
	}
	
	return 0;
	#else
	if( second_start_switch == 2)
	{
		return ( check_DSP3_input()&0x18);//
	}
	else
		return 0;
	#endif
}

//#if SUPPORT_UNIFY_DRIVER

void write_stepmotor_config_para(UINT8 port,UINT8 *pdata)    
{
	 UINT8 k;
	 SUM = 1;
	 send_dsp_command(port,0x0003);//д�벽�����ò���
	 for( k = 0;k < 205; k++ )
	 {
		send_dsp_command(port,((UINT16)k<<8) + (UINT16)pdata[k] );
		//printf_uart("%x,",((UINT16)k<<8) + (UINT16)pdata[k]);
		delay_ms(1);
	 }
	 SUM = 0;
} 

void read_stepmotor_config_para(UINT8 port)
{
    UINT8 m;
	
	for(m=0; m<205; m++)
	{
		if( port == 1 )//dsp1
		    send_dsp1_command(0x0004,0x5555);
		else if( port == 2 )
			send_dsp2_command(0x0004,0x5555);
		else if( port == 3 )
			send_dsp3_command(0x0004,0x5555);
		else if( port == 4 )
			send_dsp4_command(0x0004,0x5555);
		svpara_disp_buf[m] = recieve_x.word;
		delay_us(1000);
	}
}


/*
ֱ�Ӱ�16λд�뵽DSP��
һ��4008�����ݣ�ǰ��1+���ݳ���2+����1+DSP��1+data....+У��2+����1
���ݹ�4000���ֽڣ���2000���֣�ǰ1999���������ݣ�������ֽ���ǰ1999У���
*/

UINT16 crc_calcu(UINT16 far *crc_in, UINT16 length, UINT16 init)
{
  UINT16 crc_i,crc_j,crc_out;
  crc_out = init;
  for(crc_j = 0; crc_j < length; ++crc_j)
  {
      crc_out ^= crc_in[crc_j];
      for (crc_i = 0; crc_i < 16; ++crc_i)
      {
    	  if (crc_out & 1)
              crc_out = (crc_out >> 1) ^ 0x1021;
    	  else
              crc_out = (crc_out >> 1);
      }
  }
  return crc_out;
}

UINT16 read_stepmotor_curve_crc(UINT8 port)
{
	if( port == 1)
	    send_dsp1_command(0x000C,0x5555);//��ȡ�������ߵ�CRCֵ
	else if( port == 2)
		send_dsp2_command(0x000C,0x5555);
	else if( port == 3)
		send_dsp3_command(0x000C,0x5555);
	else if( port == 4)
		send_dsp4_command(0x000C,0x5555);	
	return recieve_x.word;
}
#if UART1_DEBUG_OUTPUT_MODE
extern void uart1_send_char(UINT8 ch);
#endif

#if SUPPORT_UNIFY_DRIVER
UINT8 write_stepmotor_curve(UINT8 port,UINT8 *pdata)    
{
	UINT16 k,point,val;	 
	UINT8 ret,flag;
    flag = 0;
	if( crc_value == 0)
	{
	    crc_value = crc_calcu((UINT16*)pdata,3499,0XFFFF);
		flag = 1;
	}

	 send_dsp_command(port,0x000B);//д�벽�����߲���
	 point = 0;
	 for( k = 0;k < 3499; k++ )
	 {
		val = pdata[point++]+((UINT16)pdata[point++]<<8 )  ;
		send_dsp_command(port,val );
		delay_ms(1);
	 }
	 //У��
	 send_dsp_command(port,crc_value );
	 printf_uart("crc[%d]=%x",port,crc_value);

	 delay_ms(10000);
	 val = read_stepmotor_curve_crc(port);
	 printf_uart("read=%x",val);
	 readback_crc_value = val;

	 if( val == crc_value)
	     return 1;
	 else
	 	 return 0;
	 
}
#endif


//===============================================================================

#if CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037// Сģ���
	#if SINGLE_X_MOTOR
		#if NEW_STEEPER_ANGLE_MODE16
		const UINT16 config_para[]=//24546 20513
		{
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
		//	0x0005,0x0101,1000,1000,1707,2046,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
			0x0005,0x0101,1000,1000,2048,2046,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0005,0x0101,1000,1000,2046,2561 ,100, (128<<7)+40,(30<<7)+5,(1<<12),(1<<12)+4095,//DSP2
		};
		#elif NEW_STEEPER_ANGLE_MODE
		const UINT16 config_para[]=//24546 20513
		{
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0005,0x0101,1000,1000,107, 128,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0005,0x0101,1000,1000,20513,16384 ,100, (128<<7)+40,(30<<7)+5,(1<<12),(1<<12)+4095,//DSP2
		};
		#else
		const UINT16 config_para[]=
		{
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0005,0x0101,1000,1000,24546,20513,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0005,0x0101,1000,1000,20513,16384 ,100, (128<<7)+40,(30<<7)+5,(1<<12),(1<<12)+4095,//DSP2
		};
		#endif
	#else
	const UINT16 config_para[]=
	{
		//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
		0x0005,0x0101,1000,1000,24546,20513,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
		//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
		0x0005,0x0101,1000,1000,20513,16384,100, (128<<7)+40,(30<<7)+5,(1<<12),(1<<12)+4095,//DSP2
	};
	#endif
#endif

#if CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_55// ��װ������
		const UINT16 config_para[]=//24546 20513
		{
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0005,0x0101,1000,1000,1707,2046,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0005,0x0101,1000,400, 2046,2561 ,100, (128<<7)+40,(30<<7)+5,(1<<12),(1<<12)+4095,//DSP2
		};
#endif

#if CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800//4��ϵͳСģ���+�����и�
	#if SINGLE_X_MOTOR
	const UINT16 config_para[]=
	{
		//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
		0x0004,0x0101,1000,1000,24546,20513,100, (128<<7)+40,(128<<7)+40,(1<<12)+3,(1<<12)+3,//DSP1
		//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
		0x0004,0x0101,1000,1000,20513,16384 ,100, (128<<7)+40,(30<<7)+5,(1<<12)+3,(1<<12)+4095,//DSP2
	};
	#else
	const UINT16 config_para[]=
	{
		//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
		0x0004,0x0101,1000,1000,24546,20513,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
		//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
		0x0004,0x0101,1000,1000,20513,16384,100, (128<<7)+40,(30<<7)+5,(1<<12),(1<<12)+4095,//DSP2
	};
	#endif
#endif

#if CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_12080_SCREW// 12080˿�� ��X
const UINT16 config_para[]=
{
	//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
	0x0004,0x0101,1000, 400,8192,20480,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
	//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
	0x0004,0x0201, 400, 400,8192,8192, 100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
};
#endif
#if CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_12080_SCREW_A// 12080˿�� ˫X
const UINT16 config_para[]=
{
	//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
	0x0004,0x0101,1000, 400,8192,8192  ,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
	//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
	0x0004,0x0201, 400, 400,20480,8192 ,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
};
#endif

#if CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_12080_BELT// 12080Ƥ�� ��X

	#if NEW_STEEPER_ANGLE_MODE
	//�²���Ǽ��㹫ʽ 
		const UINT16 config_para[]=
		{
			//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0004,0x0101, 400, 400, 2049,2049,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0004,0x0201, 400, 400, 5122,5122 ,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
		};
	#else
		const UINT16 config_para[]=
		{
			//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0004,0x0101, 400, 400,20480,20480,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0004,0x0201, 400, 400,8192,8192  ,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
		};
	#endif
#endif
#if CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_12080_BELT_A// 12080Ƥ�� ˫X
const UINT16 config_para[]=
{
	//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
	0x0004,0x0101, 400, 400,20480,20480,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
	//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
	0x0004,0x0201, 400, 400,20480,8192,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
};
#endif

#if CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_FIFTH_BOBBIN //5���Զ�����
const UINT16 config_para[]=
		{
			//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0005,0x0101, 1000,1000,1707,2051,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0005,0x0202, 1000,1000,2048,5120,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
		};
#endif

#if CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_DOUBLE_X// ��ͨ800,900
	
	#if SINGLE_X_MOTOR
		#if NEW_STEEPER_ANGLE_MODE16		
			const UINT16 config_para[]=
			{
				//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
				0x0004,0x0101, 400, 400, 1707, 2051,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
				//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
				0x0004,0x0201, 400, 400, 5120, 5120,100, (128<<7)+40,(128<<7)+40,(1<<12)+4095,(1<<12)+4095,//DSP2  27[100=>
			};
	
		#elif NEW_STEEPER_ANGLE_MODE
		const UINT16 config_para[]=
		{
			//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0004,0x0101, 400, 400, 107, 128,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0004,0x0201, 400, 400, 128,320,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
		};
		#else
		const UINT16 config_para[]=
		{
			//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0004,0x0101, 400, 400,24546,20480,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0004,0x0201, 400, 400,20480,8192,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
		};
		#endif
	#else
		#if NEW_STEEPER_ANGLE_MODE
		const UINT16 config_para[]=
		{
			//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0004,0x0505, 400, 400, 1707, 1707, 100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0004,0x0201, 400, 400, 2051, 5120,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
		};
		#else
		const UINT16 config_para[]=
		{
			//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0004,0x0505, 400, 400,24546,24546,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0004,0x0201, 400, 400,20480,8192,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
		};
		#endif
	#endif
#endif

#if CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_FIFTH_DOUBLE_X// 5����ͨ800,900

	#if NEW_STEEPER_ANGLE_MODE16
	    #if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER30  
			#if COMPARE_NOPMOVE_SPEED
			const UINT16 config_para[]=
			{
				//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
				0x0005,0x0101, 1000,1000,10240,10240,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
				//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
				0x0005,0x0201, 1000,1000,2048,2561,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
			};
			#else
			const UINT16 config_para[]=//x=0.30 y=0.360
			{
				//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
				0x0005,0x0101, 1000,1000,1707,2051,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
				//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
				0x0005,0x0201, 1000,1000,2048,2561,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
			};
			#endif
		#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER35 || COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER38 || COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER36
			const UINT16 config_para[]=//x=0.30 y=0.360
			{
				//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
				0x0005,0x0101, 1000,1000,2048,2051,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
				//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
				0x0005,0x0201, 1000,1000,2048,2561,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
			};
		#else
		const UINT16 config_para[]=
		{
			//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0005,0x0101, 1000,1000,1707,2051,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0005,0x0201, 1000,1000,2048,5120,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
		};
		#endif
	#else
		const UINT16 config_para[]=
		{
			//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0005,0x0101, 400, 400,24546,24546,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
			//0x11 0x1f   0x20 0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
			0x0005,0x0201, 400, 400,20480,8192,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
		};
	#endif
	
#endif

#if CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_SINGLE_X_INPERSSER// ��ͨ800,900
	const UINT16 config_para[]=
			{
				//0x11 0x1f   0x20  0x21 0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
				0x0004,0x0101, 400, 400, 1707, 2051,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12),//DSP1
				//0x11 0x1f   0x20 0x21  0x22 0x23  0x27 0x28-0      0x28-1     0x28-2  0x28-3
				0x0004,0x0101, 400, 1000,2049, 2561,100, (128<<7)+40,(128<<7)+40,(1<<12),(1<<12)+4095,//DSP2
			};
#endif



#if SUPPORT_UNIFY_DRIVER

	#if ENABLE_CONFIG_PARA == 1 
	void setup_stepper_moter(void)
	{
	 	UINT16 current1,current2;
		
		send_dsp1_command(0x0011,0x0005);
		send_dsp1_command(0x001F,para.DSP1_para_1F);  //�����ջ��л� 1��ʾ�ջ���2��ʾ������3��ʾת��ģʽ��4��ʾ�涯ģʽ��5��ʾ˫��ͬ��
		send_dsp1_command(0x0020,para.DSP1_para_20);  //����������
		send_dsp1_command(0x0021,para.DSP1_para_21);		
		send_dsp1_command(0x0022,para.DSP1_para_22);  //�����
		send_dsp1_command(0x0023,para.DSP1_para_23);
		current1 = x_step_current_level;
		current2 = y_step_current_level;
		current1 = ((current1<<3) + (UINT16)para.dsp1A_half_current)<<8;
		current2 = ((current2<<3) + (UINT16)para.dsp1B_half_current) + current1;	
		send_dsp1_command(0x0026,current2 );
		send_dsp1_command(0x0027,para.DSP1_para_27);  
		send_dsp1_command(0x0028,para.DSP1_para_28H); //��һ·����	
		send_dsp1_command(para.DSP1_para_28M1,para.DSP1_para_28M2);  //�ڶ�·����+��һ·����ϵ��  X��ȷY�ŷ�
	    send_dsp_command(1,para.DSP1_para_28L);
	
		send_dsp2_command(0x0011,0X0005);
		send_dsp2_command(0x001F,para.DSP2_para_1F);  //�����ջ��л� 1��ʾ�ջ���2��ʾ������3��ʾת��ģʽ��4��ʾ�涯ģʽ��5��ʾ˫��ͬ��
		send_dsp2_command(0x0020,para.DSP2_para_20);  //����������
		send_dsp2_command(0x0021,para.DSP2_para_21);		
		send_dsp2_command(0x0022,para.DSP2_para_22);  //�����
		send_dsp2_command(0x0023,para.DSP2_para_23);  
		#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER40
		current1 = inpress_step_current_level;
		current2 = foot_step_current_level;
		
		#else
		current1 = foot_step_current_level;
		current2 = inpress_step_current_level;
		#endif
		current1 = ((current1<<3) + (((UINT16)(para.dsp2A_half_current))&0x0007))<<8;
		current2 = ((current2<<3) + (((UINT16)(para.dsp2B_half_current))&0x0007)) + current1;
		
		send_dsp2_command(0x0026,current2 );
		send_dsp2_command(0x0027,para.DSP2_para_27);  
		send_dsp2_command(0x0028,para.DSP2_para_28H); //��һ·����	
		send_dsp2_command(para.DSP2_para_28M1,para.DSP2_para_28M2);  //�ڶ�·����+��һ·����ϵ��  X��ȷY�ŷ�
		send_dsp_command(2,para.DSP2_para_28L);
	#if MULTIPULE_IO_ENABLE	 == 1
		send_dsp3_command(0x0011,0X0005);
		send_dsp3_command(0x001F,para.DSP3_para_1F);  //�����ջ��л� 1��ʾ�ջ���2��ʾ������3��ʾת��ģʽ��4��ʾ�涯ģʽ��5��ʾ˫��ͬ��
		send_dsp3_command(0x0020,para.DSP3_para_20);  //����������
		send_dsp3_command(0x0021,para.DSP3_para_21);		
		send_dsp3_command(0x0022,para.DSP3_para_22);  //�����
		send_dsp3_command(0x0023,para.DSP3_para_23);  
		current1 = para.dsp3A_current;
		current2 = para.dsp3B_current;
		current1 = ((current1<<3) + (((UINT16)(para.dsp3A_half_current))&0x0007))<<8;
		current2 = ((current2<<3) + (((UINT16)(para.dsp3B_half_current))&0x0007)) + current1;
		
		send_dsp3_command(0x0026,current2 );
		send_dsp3_command(0x0027,para.DSP3_para_27);  
		send_dsp3_command(0x0028,para.DSP3_para_28H); //��һ·����	
		send_dsp3_command(para.DSP3_para_28M1,para.DSP3_para_28M2);  //�ڶ�·����+��һ·����ϵ��  X��ȷY�ŷ�
		send_dsp_command(3,para.DSP3_para_28L);	
	#endif	
	}
	
	#else
 	void setup_stepper_moter(void)
	{
		 UINT16 current1,current2;	 
		 //DSP1
		 send_dsp1_command(0x0011,config_para[0]);//ƽ̨����	
		 send_dsp1_command(0x001F,config_para[1]);//����
		 send_dsp1_command(0x0020,config_para[2]);//DSP:1�����������
		 send_dsp1_command(0x0021,config_para[3]);//DSP:2�����������
		 send_dsp1_command(0x0022,config_para[4]);//DSP:1�Ჽ���
		 send_dsp1_command(0x0023,config_para[5]);//DSP:2�Ჽ��� 
	#if DOUBLE_X_60MOTOR 
		 current1 = x_step_current_level;
		 current2 = x_step_current_level;
	#else
		 current1 = x_step_current_level;
		 current2 = y_step_current_level;
	#endif
		 current1 = ( (current1<<3) + 2) <<8 ;
		 current2 = (current2<<3) +  2 + current1;	 
		 send_dsp1_command(0x0026,current2 );
		 send_dsp1_command(0x0027,config_para[6]); 			//���������Ͱ���
		 send_dsp1_command(0x0028,config_para[7]);   		//��һ·����	
		 send_dsp1_command(config_para[8],config_para[9]);  //�ڶ�·����+��һ·����ϵ��
	     send_dsp_command(1,config_para[10]);				//�ڶ�·����ϵ��
		 
		 //DSP2
		 send_dsp2_command(0x0011,config_para[11]);//ƽ̨����	
		 send_dsp2_command(0x001F,config_para[12]);//����
		 
	  #if SECOND_GENERATION_PLATFORM 
	  	if( DVB == 0)	
		{
			send_dsp2_command(0x0020,400);//DSP:1�����������
		    send_dsp2_command(0x0021,400);//DSP:2�����������
		}
		else
		{
			send_dsp2_command(0x0020,1000);//DSP:1�����������
		    send_dsp2_command(0x0021,1000);//DSP:2�����������
		}
	  #else
	  if( special_encoder_mode == 1)
	  {
		  send_dsp2_command(0x0020,400);//DSP:1�����������
		  send_dsp2_command(0x0021,400);//DSP:2�����������
	  }
	  else
	  {
		  send_dsp2_command(0x0020,config_para[13]);//DSP:1�����������
		  send_dsp2_command(0x0021,config_para[14]);//DSP:2�����������
	  }
	  #endif
	  
		 send_dsp2_command(0x0022,config_para[15]);//DSP:1�Ჽ���
		 send_dsp2_command(0x0023,config_para[16]);//DSP:2�Ჽ��� 
	#if DOUBLE_X_60MOTOR 
		 current1 = y_step_current_level;
		 current2 = inpress_step_current_level;
	#else
		 current1 = foot_step_current_level;
		 current2 = inpress_step_current_level;
	#endif
		 current1 = ( (current1<<3) + 2) <<8 ;
		 current2 = (current2<<3) +  2 + current1;	 
		 send_dsp2_command(0x0026,current2 );
		 send_dsp2_command(0x0027,config_para[17]); 		//���������Ͱ���
		 send_dsp2_command(0x0028,config_para[18]);   		//��һ·����	
		 send_dsp2_command(config_para[19],config_para[20]);//�ڶ�·����+��һ·����ϵ��
	     send_dsp_command(2,config_para[21]);				//�ڶ�·����ϵ��
	
		 if(SUPPORT_CS3_FUN == 1)
		{
			step_cfg_data = 0x0011; 
			spi_flag = 1;
			trans_x.word = step_cfg_data; 
			trans_y.word=(~trans_x.word)&0x7fff;
			trans_z.word=0x5555;
			dsp3 = 1;
			SPISTE3 = 0;
			SPISTE1 = 1;
			SPISTE2 = 1;
			delay_ms(2);
			s4trr = trans_x.byte.byte1;
			delay_ms(5);
			step_cfg_data =0x5636;//5644;//5636
			spi_flag = 1;
			trans_x.word = step_cfg_data; 
			trans_y.word=(~trans_x.word)&0x7fff;
			trans_z.word=0x5555;
			dsp3 = 1;
			SPISTE3 = 0;
			delay_ms(2);
			s4trr = trans_x.byte.byte1;
			delay_ms(150);
		}

	}
	#endif
	
	
#elif (CURRENT_MACHINE == MACHINE_900_SPEPPER_CUTTER)

void setup_stepper_moter(void)
{
	send_dsp1_command(0x0011,0x0004);//ƽ̨����	
	send_dsp1_command(0x001F,0X0101);//��һ·�ջ����ڶ�·�ջ�
	send_dsp1_command(0x0020,400);
	send_dsp1_command(0x0021,400);
	send_dsp1_command(0x0022,12288);  //0.6d  X��12288 14734
	send_dsp1_command(0x0023,10240);  //0.72d Y��
	send_dsp1_command(0x0024,0x0003); //DH-60BYGH250210ENXY001
	send_dsp1_command(0x0025,0x0007); //DH-86BYGH250I001
	send_dsp1_command(0x0027,30);
	
	send_dsp2_command(0x0011,0x0005);//ƽ̨����	 dsp2 20m
	send_dsp2_command(0x001F,0X0201);//����·�ջ�������·����
	send_dsp2_command(0x0020,400);   //ֱ���е� 60�ջ� DH-60GH070M-BQ01-Q-2.7
	send_dsp2_command(0x0021,400);   //��ѹ��   60���� DH-60BYGH250B002
	send_dsp2_command(0x0022,8192);  //0.9 d
	send_dsp2_command(0x0023,8192);  //0.9 d
	send_dsp2_command(0x0024,0x0001);//DH-60GH070M-BQ01-Q-2.7
	send_dsp2_command(0x0025,0x0003);//DH-60BYGH250B002
	send_dsp2_command(0x0026,inpress_step_configure_tab5[inpress_step_current_level]+foot_step_configure_tab5[foot_step_current_level]);
	send_dsp2_command(0x0027,300);
}	
#elif DOUBLE_X_60MOTOR 

#if CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800

	void setup_stepper_moter(void)
	{
			//dsp1 
			send_dsp1_command(0x0011,0x0004);//ƽ̨����	
			send_dsp1_command(0x001F,0X0101);//0101-˫X�ջ�
			send_dsp1_command(0x0020,1000);   //X��  60�ջ�
			send_dsp1_command(0x0021,1000);   //X��  60�ջ�

			send_dsp1_command(0x0022,12273); //0.6d 12288
			send_dsp1_command(0x0023,12273); //0.6d	
			send_dsp1_command(0x0024,0x0003);//DH-60BYGH250210ENXY001
			send_dsp1_command(0x0025,0x0003);//DH-60BYGH250210ENXY001
			//send_dsp1_command(0x0026,inpress_step_configure_tab5[inpress_step_current_level]+foot_step_configure_tab5[foot_step_current_level]);
		    send_dsp1_command(0x0027,30);
			
			//=====================================================
			//dsp2
			send_dsp2_command(0x0011,0x0005);//ƽ̨����	 dsp2 20m
			send_dsp2_command(0x001F,0X0101);//0101-˫�ջ� 0202-˫����   Y��+��ѹ���涯
			send_dsp2_command(0x0020,1000);   //Y��   86�ջ�
			send_dsp2_command(0x0021,1000);  //��ѹ��60�ջ�
			send_dsp2_command(0x0022,10256); //0.72d
			send_dsp2_command(0x0023,8192);  //0.9d				
			send_dsp2_command(0x0024,0x0007);//DH-86BYGH250I001
		    send_dsp2_command(0x0025,0x0002);//DH-24HS5402-21K-X		 
			//send_dsp2_command(0x0026,inpress_step_configure_tab5[inpress_step_current_level]+foot_step_configure_tab5[foot_step_current_level]);
			send_dsp2_command(0x0027,0);		
	}

#else
	void setup_stepper_moter(void)
	{
			//dsp1 
			send_dsp1_command(0x0011,0x0004);//ƽ̨����	
			send_dsp1_command(0x001F,0X0101);//0101-˫X�ջ�
			send_dsp1_command(0x0020,400);   //X��  60�ջ�
			send_dsp1_command(0x0021,400);   //X��  60�ջ�

			send_dsp1_command(0x0022,12273); //0.6d 12288
			send_dsp1_command(0x0023,12273); //0.6d	
			send_dsp1_command(0x0024,0x0003);//DH-60BYGH250210ENXY001
			send_dsp1_command(0x0025,0x0003);//DH-60BYGH250210ENXY001
		    send_dsp1_command(0x0027,0);
			//=====================================================
			//dsp2
			send_dsp2_command(0x0011,0x0005);//ƽ̨����	 dsp2 20m
		#if FOLLOW_INPRESS_FUN_ENABLE
			send_dsp2_command(0x001F,0X0101);//0101-˫�ջ� 0202-˫����   Y��+��ѹ���涯
		#else
			send_dsp2_command(0x001F,0X0201);//����·�ջ�������·����
		#endif
			send_dsp2_command(0x0020,400);   //Y��   86�ջ�
			send_dsp2_command(0x0021,1000);  //��ѹ��60�ջ�
			send_dsp2_command(0x0022,10240); //0.72d
		#if NEW_STRUCTURE_800_INPRESS
			send_dsp2_command(0x0023,16384);  //0.45d	
		#else
			send_dsp2_command(0x0023,8192);  //0.9d	
		#endif				
			send_dsp2_command(0x0024,0x0007);//DH-86BYGH250I001
		#if FOLLOW_INPRESS_FUN_ENABLE	
	     
			 #if NEW_STRUCTURE_800_INPRESS
			     send_dsp2_command(0x0025,0x0002);//DH-24HS5402-21K-X		 
			 #elif NEW_STRUCTURE_900_INPRESS || MACHINE_800_NORMAL
			     send_dsp2_command(0x0025,0x0000);//DH-60BYGH2501790QD001
			 #else
				 send_dsp2_command(0x0025,0x0001);//DH-60BYGH2501790BZ001
			 #endif
		#else
			send_dsp2_command(0x0025,0x0003);//��ѹ��   DH-60BYGH250B002
			send_dsp2_command(0x0026,inpress_step_configure_tab5[inpress_step_current_level]+foot_step_configure_tab5[foot_step_current_level]);
		#endif
			send_dsp2_command(0x0027,0);
			//=====================================================
			//dsp3
			if( SUPPORT_CS3_FUN == ENABLE_FUN)
			{
			   step_cfg_data = 0x5620 + 3;//foot_step_current_level;
			   send_dsp3_command(0x0011,step_cfg_data);//DH-60BYGH450A003-1  5623 8637 5636 16λ����ֵ CHHC C--�������� H--���ֵ���
			   //7---8.3A  6--7.6A  5-6.3A  4-5.86A 3--4.0A 
			   delay_ms(150);
			}
		
			//=====================================================
			if( DVB == 0)
			{
				send_dsp_command(DSP1,0x0018);
				delay_ms(200);
				counter_1ms = 0;
				turnon_buz();
				send_dsp_command(DSP1,0x0013);
				while(1)
				{
					send_dsp1_command(0x0015,0x5555);
					if( recieve_x.word == 0 )
					    break;
					if( counter_1ms >= 20000)
					    break;
					rec_com();
					flash_buz();
				}
				//if( recieve_x.word == 0 )
				{
					send_dsp_command(DSP1,0x0017);
					counter_1ms = 0;
					while(1)
					{
						send_dsp1_command(0x0016,0x5555);
						if( recieve_x.word == 0 )
						    break;
						if( counter_1ms >= 20000)
						    break;
						rec_com();
						flash_buz();
					}
				}
				turnoff_buz();
			}		
	}
	#endif

#else

void setup_stepper_moter(void)
{
	send_dsp1_command(0x0011,0x0004);//ƽ̨����	
	send_dsp1_command(0x001F,0X0101);
	send_dsp1_command(0x0020,400);
	send_dsp1_command(0x0021,400);
	send_dsp1_command(0x0022,10240); //0.72d
	send_dsp1_command(0x0023,10240);	
	send_dsp1_command(0x0024,0x0007);
	send_dsp1_command(0x0025,0x0007);
	send_dsp1_command(0x0027,300);
//=====================================================
	send_dsp2_command(0x0011,0x0004);//ƽ̨����	
	
	#if ENABLE_LOOPER_CUTTER
	 	send_dsp2_command(0x001F,0X0201);//����·�ջ�������·����
		send_dsp2_command(0x0020,400);   
		send_dsp2_command(0x0021,400);   
		send_dsp2_command(0x0022,8192);  //0.9d
		send_dsp2_command(0x0023,8192);  //0.9d	
		send_dsp2_command(0x0024,0x0001);//�е��ջ� DH-60BYGH2501790YJ001
		send_dsp2_command(0x0025,0x0003);//��ѹ��   DH-60BYGH250B002
		send_dsp2_command(0x0026,inpress_step_configure_tab5[inpress_step_current_level]+foot_step_configure_tab5[foot_step_current_level]);
		send_dsp2_command(0x0027,300);
		
		if(SUPPORT_CS3_FUN == 1)
		{
			step_cfg_data = 0x0011; 
			spi_flag = 1;
			trans_x.word = step_cfg_data; 
			trans_y.word=(~trans_x.word)&0x7fff;
			trans_z.word=0x5555;
			dsp3 = 1;
			SPISTE3 = 0;
			SPISTE1 = 1;
			SPISTE2 = 1;
			delay_ms(2);
			s4trr = trans_x.byte.byte1;
			delay_ms(5);
			step_cfg_data =0x5636;//5644;//5636
			spi_flag = 1;
			trans_x.word = step_cfg_data; 
			trans_y.word=(~trans_x.word)&0x7fff;
			trans_z.word=0x5555;
			dsp3 = 1;
			SPISTE3 = 0;
			delay_ms(2);
			s4trr = trans_x.byte.byte1;
			delay_ms(150);
		}
	#else
		
		#if FOLLOW_INPRESS_FUN_ENABLE
			send_dsp2_command(0x001F,0X0102);//�����ջ��л� 0101-˫�ջ� 0202-˫����
		#else
			send_dsp2_command(0x001F,0X0202);
		#endif
			send_dsp2_command(0x0020,400);   //��ѹ��
			send_dsp2_command(0x0021,400);   //���ߵ��
			send_dsp2_command(0x0022,8192);  //0.9d
			send_dsp2_command(0x0023,8192);  //0.9d	
			send_dsp2_command(0x0024,0x0003);//ץ�ߵ��      DH-60BYGH450A003-1  //4
		#if FOLLOW_INPRESS_FUN_ENABLE
			send_dsp2_command(0x0025,0x0001);//��ѹ��60�ջ�  DH-60BYGH2501790BZ001
		#else
			send_dsp2_command(0x0025,0x0003);//��ѹ��60�ջ�  DH-60BYGH250B002
		#endif 
			send_dsp2_command(0x0026,inpress_step_configure_tab5[inpress_step_current_level]+foot_step_configure_tab5[foot_step_current_level]);
			send_dsp2_command(0x0027,300);
    #endif	

}
#endif


UINT8 requery_dsp1_status(void)
{
	send_dsp1_command(0x0015,0x5555);		
	return( recieve_x.word );
}

#if LASER_DSP_PROCESS

void movestep_xy3(INT8 * buf_x,INT8 * buf_y,UINT8 length,UINT8 *spd_tab)
{
	UINT8 i,spd,j;	
	delay_us(500);
	while( requery_dsp1_status() >= 3)//�ȴ���������  1-��ʾ���黺�������� 2-��һ��������Ϊ�� 3-��ʾ������������������
	{
		   rec_com();
		   if( PAUSE == pause_active_level)
	  	   {
				delay_ms(20);
				if( PAUSE == pause_active_level)
				{
					laser_emergency_stop = 1;	
					sys.status = ERROR;
					sys.error = ERROR_02;
					StatusChangeLatch = ERROR;							
					return ;
				}
		   }
	}
	send_dsp1_command(0x0014,length);
	for(i=0;i<length;i++)
	{
		spd = spd_tab[i];
		if( spd > 63)
		{
			timer_y = spd>>6 ;
		    timer_x = spd %64;
		}
		else
		{
			timer_x = spd;
		    timer_y = 0;
		}
		movestep_x3(buf_x[i]); 
		movestep_y3(buf_y[i]);
		#if UART1_DEBUG_OUTPUT_MODE
			set_func_code_info(i,spd,fabsm(buf_x[i])>>1,fabsm(buf_y[i])>>1);
		#endif
	}
	frame_start_counter = 0;

}
#endif


//�๦��IO��������
void multipule_program_beginning(UINT8 port)
{
	while(spi_flag > 0);
	send_dsp_command(port,0x1100);//�л�����
	delay_ms(1000);
}

void send_multipule_program_data(UINT8 port)    
{
	 UINT8 k;
	 while(spi_flag > 0);	 
	 send_dsp_command(port,0x1101);//д�����	    
     delay_ms(1);
	 for(k=0;k<data_length_drv+2;k++)
	 {
		send_dsp_command(port,((UINT16)k<<8) + pat_buf[k]);
		rec_com();
	}
} 

UINT16 read_multipule_program_status(UINT8 port)
{
	send_dsp_command(port,0x1102);//״̬��ѯ
	send_dsp_command(port,0x5555);
	return (UINT16)recieve_x.word;
}
void multipule_program_end(UINT8 port)    
{
	 while(spi_flag > 0);
	 send_dsp_command(port,0x1104);//�������	 
	 delay_ms(2);
} 
/*
bit15:0-��һ· 1-�ڶ�·
bit14~bit13:01-��������λ��ƽ 10--�������Ƿ����λ�� 11-��ѯ������
bit12~bit0:0
*/

UINT16 get_YJORG_status(void)
{
	send_dsp2_command(0x0029,0xc000);
	send_dsp_command(2,0x5555);
	return recieve_x.word;
}

UINT16 get_IORG_status(void)
{
	send_dsp2_command(0x0029,0xA000);
	send_dsp_command(2,0x5555);
	if( recieve_x.word != 0) 
		return 1;
	else
		return 0;		
}
UINT16 get_stepper_cutter_ORG(void)
{
	send_dsp2_command(0x0029,0x2000);
	send_dsp_command(2,0x5555);
	if( recieve_x.word != 0) 
		return 0;
	else
		return 1;
}
/*

*/
UINT16 get_cutter_motor_angle(void)
{
	send_dsp2_command(0x001d,0x0001);
	return (recieve_x.word >>6  );
}

void set_rotated_cutter_speed(UINT16 speed)
{
	UINT16 tmp;
	send_dsp2_command(0x001F,(para.DSP2_para_1F&0x00ff)|0x0300 );  //�����ջ��л� 1��ʾ�ջ���2��ʾ������3��ʾת��ģʽ��4��ʾ�涯ģʽ��5��ʾ˫��ͬ��
	delay_ms(2);
	tmp = (speed & 0x0fff) + 0xa000;
	if( yj_motor_dir == 1)
		tmp += 0x4000;
	send_dsp_command(2,tmp);
	if( speed == 0)
	{
		delay_ms(20);
		send_dsp2_command(0x001F,para.DSP2_para_1F); 
	}
	
}
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
