//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : motor.c
//  Description: motor control arithmetic
//  Version    Date     Author    Description
//  0.01     09/08/08   lm        created
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"            //M16C/62P special function register definitions
#include "..\..\include\typedef.h"           //data type define
#include "..\..\include\common.h"           //External variables declaration
#include "..\..\include\variables.h"        //External variables declaration
#include "..\..\include\math.h"         

#if USE_SEVERO_MOTOR_1635

#include "..\..\include\motor_def.h"        //constants definition
#include "..\..\include\math_tab.h"          //sine table     

#define VOL_ADJUST

//--------------------------------------------------------------------------------------
//  ROM constants declaration - the values are declared at the end of the code
//--------------------------------------------------------------------------------------
const INT16 acc_spd_tab_const[] =
{
	12,25,33,50,70,70,70,70,70,70,70,70,70,70,70,70
};

//--------------------------------------------------------------------------------------
//  Global variables define
//--------------------------------------------------------------------------------------


//************
static INT8 kk;
static INT16 sum_spd_s;
static UINT8 current_response_speed;
//************


static INT16 m_count;
static INT16 ta3z_elec;
static INT16 ta3z_mach;
static INT16 alpha;
static INT16 vol; 
static INT16 m_pos_ref;	    //motor referense position normalize
static UINT16 m_pos_ref_add;
static INT32 a_m_spd;
static INT16 count_spd;

static INT32 sum_err_s;
static INT16 sum_err_p_stop;

static UINT8 start_status;
static INT16 ta3_start;

static INT16 s_count;

//static UINT8 m_status;
static INT16 Y_k;
static UINT8 ta3_ind;

static INT16 ta3_buf[1<<SAMPLES_BIT];

static INT8	stop_status;
static UINT8 l_ism;


//****************************
//****************************
static INT16 sum_err_stop;
static INT16 rpm_keeper = 0;
static INT16 counter_H = 0;
static INT16 counter_L = 0;
static INT16 tactic_allowed;			
static INT16 delta_s;		
static INT16 pos = 0;
static INT16 transfer_sp = 0;
static INT16 transfer_si = 0;
//*****************************
//*****************************
volatile INT16 spd_tmp;
INT16 spd_last_value;
INT16 TaccTmp; 
static INT16 ta3n;
static UINT16 top = 255;
static UINT16 bottom = 0;
static UINT16 error = 1;
static UINT16 mid = 0;

static INT16 ta3_last;
static INT16 ta3_begin;
static INT16 delta_l;
static INT16 e_start;
static INT16 real_dir;
static INT16 zero_vol_counter;
static INT16 zero_vol_counter_1;
static INT16 stop_change_counter;
static INT16 vol_last;

//--------------------------------------------------------------------------------------
//  Internal functions and subroutines declaration
//--------------------------------------------------------------------------------------
#pragma	INTERRUPT/E pwm_int
void pwm_int(void);
#pragma	INTERRUPT/E int0_int
void int0_int(void);
void motor_control(void);
void m_start(INT16 ta3_t);
void cal_forward(INT16 theta_input, INT16 u);
INT16 cal_pid_s(INT16 pos_spd_ref, INT16 sp, INT16 si);
void cal_spd_ref(INT16 speed, INT16 acc_spd, INT16 dcc_spd);
//*********************
INT16 dither_tactic();
INT16 acc_tactic();
INT16 normal_control();
INT16 fall_back_tactic();
INT16 pi_adjust_p();
INT16 pi_adjust_i();
//*********************
//--------------------------------------------------------------------------------------
// 	Constants definition
//--------------------------------------------------------------------------------------
#define A_1 27853//1638  //(25Hz,0x71b5),(50Hz,0x664d),(100Hz,0x5532),(120Hz,0x4fdb),(150Hz,0x48fa),(200Hz,0x3fd8),(220Hz,0x3cc8),(250Hz,0x38b8),(280Hz 27853)                        
#define B0_1 4915//31130 //(25Hz,0x0e4a),(50Hz,0x19B2),(100Hz,0x2acd),(120Hz,0x3024),(150Hz,0x3705),(200Hz,0x4027),(220Hz,0x4337),(250Hz,0x4747),(280Hz 4915)

//--------------------------------------------------------------------------------------
//  Name:		init_tb0
//  pars:	None
//  Returns:	None
//  Description: initial timer B0
//-------------------------------------
//-------------------------------------------------
void init_tb0(void)
{
    //tb0mr = 0x4a;
	tb0mr = 0x42;       // XX0X XX00 
                            // |||| |||+- must always be 0 in timer mode
                            // |||| ||+-- must always be 0 in timer mode
                            // |||| |+--- 0: pulse is not output at pin TA0out
                            // |||| |     1: pulse is output at pin TA0out
                            // |||| |        TA0out is automatically  output
                            // |||| +---- 0: gate function: timer counts only 
                            // ||||          when TA0in is held "L"
                            // ||||       1: gate function: timer counts only
                            // ||||          when TA0in is held "H"
                            // |||+------ 0: gate function not available
                            // |||        1: gate function available
                            // ||+------- must always be 0 in timer mode
                            // |+-------- count source select bits:
                            // +--------- count source select bits:
                            //            00:  f1
                            //            01:  f8
                            //            10:  f32
                            //            11:  fc32
	tb0 = 0; 
	tb0s = 1;  //	1: start timer B0 (count flag)
    tb0ic = 0;
} 

//--------------------------------------------------------------------------------------
//  Name:		init_motor
//  Parameters:	None
//  Returns:	None
//  Description: initial motor control module 
//--------------------------------------------------------------------------------------
void init_motor(void)
{ 
    INT16 i; 
	INT32 temp32;
    
    //QEP_initialization
    ta3s = 0;                           // count disable,tabsr_addr.bit.b3 
    ta3ic = 0;                          // TA3 interrupt disable
    ta3mr = 0xd1;                       // 4tuple-phase event count mode,free_run type 
    udf |= 0x40;                        // two-phase pulse signal processing function enable 
    onsf |= 0x20;                       // Z-phase input is valid 
    trgsr &= 0xcf;                      // input on TA3IN is selected
    ta3 = 0;                            // counter clear 
    ta3ic = 0x00;                       // TA3 interrupt disable 
    ifsr0 = 0;                          // rising/falling edge select,single edge 
    int0ic = 0;			                // falling edge select 
    ta3s = 1;                           // count enable 
        
    //PWM_initialization
    ictb2 = 1;                          // one TB2 underflow interrupt 
    prcr  = 0x02;
    invc0 = 0x16;                       // triangular wave modulation mode, no two active at an instance 
    invc1 = 0x02;                       // short-circuit protection time is valid, L is active 
    prcr  = 0x00;
    idb0 = 0x3f;                        // set three-phase output buffer register 0 
    idb1 = 0x3f;                        // set three-phase output buffer register 1 
    dtt = DTT_CNT;                      //16*(1/16*10^6) short-circuit protection time setting 
    prcr  = 0x02;
    tb2sc = 0x00;
    prcr  = 0x00;
    ta1mr = 0x12;                       // one-shot pulse mode 
    ta2mr = 0x12;                       // one-shot pulse mode 
    ta4mr = 0x12;                       // one-shot pulse mode 
    tb2mr = 0x00;                       // timer mode 
    trgsr |= 0x45;                      // trigger select register TB2 trigger 
    tb2 = CARR_CNT - 1;                 // carrier cycle 
    ta4 = 1;
	ta1 = 1;
	ta2 = 1;
    ta41 = CARR_CNT-1;
	ta11 = CARR_CNT-1;
	ta21 = CARR_CNT-1;

    tb2ic = 0;	                	// TB2 interrupt enable 
	prcr=0x02;
	
    inv03 = 0;
	prcr=0x00;
	
    //start  
    m_count = 0;
	
    ta3z_elec = 0;
    ta3z_mach = 0;

    m_pos_ref = 0;
    m_spd_ref_n = 0;
	m_spd_n = 0;
	a_m_spd = 0;
	count_spd = 0;
	
    sum_err_s = 0; 
	sum_err_stop = 0;

    TaccTmp = 0;
	spd_tmp = 0;
    alpha = 0;
    vol = 0;
	
	
	init_tb0();
	stop_status = 0;
    
    start_status = 0;
	ta3_start = 0;




    idb0 = 0x2a;                      
    idb1 = 0x15;                      	    
    tabsr |= 0x96;
	m_pos_ref_add = 0;
	
	s_count = 0;
	
	Y_k = 0;
    for(i=0;i<(1<<SAMPLES_BIT);i++)
        ta3_buf[i] = 0;
    ta3_ind = 0;
	
	
	m_status = INIT;
	l_ism = 0;
	
	m_spd_n_last = 0;
	
//******************
kk = 0;
sum_spd_s = 0;
	current_response_speed = 0;
pwm_forbid_flag = 0;
over_spd_flag = 0;
//******************
find_deadpoint_flag = 0;

allow = 1;
	ta3_last = 0;
	ta3_begin = 0;
	delta_l = 0;
	e_start = 0;
	real_dir = 0;
	zero_vol_counter = 0;
	zero_vol_counter_1 = 0;
	stop_change_counter = 0;
	vol_last = 0;
}

//--------------------------------------------------------------------------------------
//  Name:	pwm_int
//  Parameters:	None
//  Returns:	None
//  Description: pwm interrupt routine
//--------------------------------------------------------------------------------------
void pwm_int(void)
{   

    INT16 temp16,spd,temp8,dir;
    INT16 l_theta_elec;                  //Q15,Motor Electrical Angle
	INT16 delta_p;
	INT16 m_spd_n_temp;	
	INT16 leading_angle,temp16c;	

#if DA1_OUTPUT_ANGLE
	temp16c = motor.angle_adjusted;
    if( (test_flag>0)&&(test_flag<20) )
	{
		da1 = 200;
		test_flag++;
	}
	else if( (test_flag>20)&&(test_flag<40) )
	{
		da1 = 250;		
		test_flag++;
	}
	else if( (test_flag>40)&&(test_flag<60) )
	{
		da1 = 150;		
		test_flag++;
	}

	else
		da1 = motor.angle_adjusted >> 3;
	
	#endif
	#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER40
	if( para.rotate_cutter_working_mode != 55)	
	#endif
	{
		if( drill_motor_run_enable == 1 )
		{
			drill_motor_counter ++;
			
			if( drill_motor_counter >= drill_motor_pwm)
			{
				DRILL_MOTOR_RUN = DRILL_MOTOR_RUN^1;
				drill_motor_counter = 0;
			}
		}
	}
    		 
	ta3n = (INT16)ta3;
		
	//calculate speed
    delta_p = ta3n - ta3_buf[ta3_ind];
    ta3_buf[ta3_ind++] = ta3n;
    ta3_ind &= SAMPLES;//8
	

//	da0 = (motor.iq >> 7) + 128;
//	if(motor.iq > 0)
//	da0 = 200;
//	else if(motor.iq < 0)
//	da0 = 50;
//	else
//	da0 = 100;

	delta_l = ta3 - ta3_begin;
	
    spd = ((INT32)delta_p * RPM_K)>>RPM_Q;
	
	
//------------------------------------------
//不敏感速度采样
//------------------------------------------
	if(abs(spd) > 400)
	{
		if(ir_tb0ic)
		{
			ir_tb0ic = 0;
			if(mr3_tb0mr == 0)
			{
				//spd = (((INT32)SPD_K)/((INT16)(tb0>>1)))>>1;
				spd = ((INT32)SPD_K)/((INT16)(tb0>>1));
				if(delta_p < 0)
				    spd = -spd;
			}
			else
				mr3_tb0mr = 0;
		}
	}
	
    m_spd_n = ((INT32)Y_k * A_1+(INT32)spd * B0_1)>>15;
	
    Y_k = m_spd_n;
	
	
//	if(m_spd_n > 7000 || m_spd_n < (-1800))
//	{
//		over_spd_flag = 1;
//	}
//	da0 = m_spd_ref_n >> 4;

//2.5V => 128RPM   1V=>51.2RPM



   
    //motor status
	switch(m_status)
    {
		case INIT:	//dummy status
         	ta3_start = ta3n;
		 	ta3_buf[ta3_ind] = ta3n;
			break;
			
	    case OPEN_START:     //start status
			int0ic = INT0_IPL | 0x10;
			m_start(ta3);
			temp16 = ISM;
				temp8 = ta3 - ta3_start;
			
				while(temp8 >= ENCODER)
				{
					temp8 -= ENCODER;
				}
				while(temp8 < 0)
				{
					temp8 += ENCODER;
				}
			
				leading_angle = alpha;
				leading_angle = ((INT32)leading_angle * ENCODER) >> 10;
			
				while(leading_angle >= ENCODER)
				{
					leading_angle -= ENCODER;
				}
				while(leading_angle < 0)
				{
					leading_angle += ENCODER;
				}			

				l_theta_elec = (temp8 << 1) + THETA_INI + leading_angle;
			
				while(l_theta_elec >= ENCODER)
				{
					l_theta_elec -= ENCODER;
				}
				while(l_theta_elec < 0)
				{
					l_theta_elec += ENCODER;
				}
				
				l_theta_elec = (((INT32)l_theta_elec * 23040)>>14);//(0~1439)
	        
			    cal_forward(l_theta_elec, vol);	
			
			if(motor.dir)//反转		
			{
				if( temp16 == 0 && delta_l < 0 )
				{
					U=1;U_=1;V=1;V_=1;W=1;W_=1;
					prcr = 0x02;
				    inv03 = 0;
					prcr = 0x00;
					U_=0;V_=0;W_=0;
					ta3z_elec = ta3n;
					ta3z_mach = ta3z_elec;
					FindZeroFlag = 1;
					motor.spd_obj = 0;
					motor.dir = 0;
					motor.iq = 0;
					m_status = STOP;
				}
			}
				
			else
			{
				if( temp16 == 1 && delta_l > 0)
				{
					U=1;U_=1;V=1;V_=1;W=1;W_=1;
					prcr = 0x02;
				    inv03 = 0;
					prcr = 0x00;
					U_=0;V_=0;W_=0;
					ta3z_elec = ta3n;
					ta3z_mach = ta3z_elec;
					FindZeroFlag = 1;
					motor.spd_obj = 0;
					motor.dir = 0;
					motor.iq = 0;
					m_status = STOP;
				}
			}
			l_ism = temp16;
			

			break;
			
        case CLOSE_RUN:     //run of closed loop status
			//calculate mechanical angle
			temp16 = ta3 - ta3z_mach;
			while(temp16 >= ENCODER)
			{
				temp16 -= ENCODER;
			}
			while(temp16 < 0)
			{
				temp16 += ENCODER;
			}
			motor.angle = temp16;
			
			motor.angle_adjusted = motor.angle - AdjustAngle; //2010-4-7
			while(motor.angle_adjusted < 0)
			{
				motor.angle_adjusted = motor.angle_adjusted + ENCODER;
			}
			
			
			//calculate electronical angle
			temp16 = ta3 - ta3z_elec;
			while(temp16 >= ENCODER)
			{
				temp16 -= ENCODER;
			}
			while(temp16 < 0)
			{
				temp16 += ENCODER;
			}
			
			leading_angle = alpha;
			leading_angle = ((INT32)leading_angle * ENCODER) >> 10;
			
			while(leading_angle >= ENCODER)
			{
				leading_angle -= ENCODER;
			}
			while(leading_angle < 0)
			{
				leading_angle += ENCODER;
			}			
			
			l_theta_elec = (temp16 << 1) + THETA_INI + leading_angle;
			
			while(l_theta_elec >= ENCODER)
			{
				l_theta_elec -= ENCODER;
			}
			while(l_theta_elec < 0)
			{
				l_theta_elec += ENCODER;
			}
				l_theta_elec = (((INT32)l_theta_elec * 23040)>>14);//(0~1439)

			//produce SVPWM signal
            cal_forward(l_theta_elec, vol);	
	        break;
			
	    case STOP:     //stopped status
            sum_err_s = 0;		//clear 
            sum_err_stop = 0;  
            sum_spd_s = 0;     
			//calculate mechanical angle
			temp16 = ta3 - ta3z_mach;
			while(temp16 >= ENCODER)
			{
				temp16 -= ENCODER;
			}
			while(temp16 < 0)
			{
				temp16 += ENCODER;
			}
			motor.angle = temp16;

			motor.angle_adjusted = motor.angle - AdjustAngle;//2010-4-7
			while(motor.angle_adjusted < 0)
			{
				motor.angle_adjusted = motor.angle_adjusted + ENCODER;
			}
	        break;
    }	
	if( FL_pwm_action_flag == 1)
	{
		if( FL_pwm_period > 0 )		
			FL_pwm_period --;
		else	
		{
			if( FL_pwm_counter >= 10)
				FL_pwm_counter = 0 ;
			else
				FL_pwm_counter ++;
			if( FL_pwm_counter < 5 )
				FL = 0;
			else
				FL = 1; 
		}
	
	}
}

//--------------------------------------------------------------------------------------
//  Name:	cal_forward
//  pars:	None
//  Returns:None
//  Description: 
//--------------------------------------------------------------------------------------
void cal_forward(INT16 theta_input, INT16 u)
{	  
    INT16 sin1,sin2;                    //Q15,sin of theta
    INT16 Ta,Tb,Tc;                          //duty ratio
    INT16 t0,t1,t2;
	
	if(theta_input < 720)
	{
		if(theta_input < 240)
		{
			if(theta_input >= 0)
			{
				sin1 = sin_tab[theta_input];
			    t2 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
			    t1 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
                Ta = t0;
				Tb = Ta + t1;
				Tc = Tb + t2;
			}
		}
		else
		{
			if(theta_input < 480)
			{	
				theta_input -= 240;
				sin1 = sin_tab[theta_input];
				t1 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
				t2 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
                Tb = t0;
				Ta = Tb + t1;
				Tc = Ta + t2;
				
			}
			else 
			{
				theta_input -= 480;
				sin1 = sin_tab[theta_input];
			    t2 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
			    t1 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
				Tb = t0;
				Tc = Tb + t1;
				Ta = Tc + t2;
			}
		}
	}
	else
	{
		if(theta_input < 1200)
		{
			if(theta_input < 960)
			{
				theta_input -= 720;
				sin1 = sin_tab[theta_input];
                t1 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
				t2 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
				Tc = t0;
				Tb = Tc + t1;
				Ta = Tb + t2;
			}
			else 
			{
				theta_input -= 960;
				sin1 = sin_tab[theta_input];
			    t2 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
			    t1 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
				Tc = t0;
				Ta = Tc + t1;
				Tb = Ta + t2;
			}
		}
		else 
		{
			if(theta_input < 1440)
			{
				theta_input -= 1200;
				sin1 = sin_tab[theta_input];
				t1 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
				t2 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
				Ta = t0;
				Tc = Ta + t1;
				Tb = Tc + t2;
			}
   		}
	}
	    
	if(Ta <= 0)
	    Ta = 1;
	else if(Ta >= CARR_CNT)	
	    Ta = CARR_CNT - 1;
	

	if(Tb <= 0)
	    Tb = 1;
	else if(Tb >= CARR_CNT)	
	    Tb = CARR_CNT - 1;
		
	if(Tc <= 0)
	    Tc = 1;
	else if(Tc >= CARR_CNT)	
	    Tc = CARR_CNT - 1;
	
	
//	da0 = Ta>>3;
	
    ta4 = Ta;
    ta1 = Tb;
    ta2 = Tc;                           
    ta41 = CARR_CNT - Ta;
    ta11 = CARR_CNT - Tb;
    ta21 = CARR_CNT - Tc;
    asm("nop");
    ta41 = CARR_CNT - Ta;
    ta11 = CARR_CNT - Tb;
    ta21 = CARR_CNT - Tc;
	
}

INT16 cal_pid_s(INT16 pos_spd_ref, INT16 sp, INT16 si)
{	
 	INT32 preSatOut;
	pos = pos_spd_ref;
	transfer_sp = sp;
	transfer_si = si;
	tactic_flag_last = tactic_flag;
		
if( (tactic_allowed == 1) && ( (m_spd_ref_n <= 2000) && (m_spd_ref_n >= 400) && (allow == 1)) )
{
	if(tactic_flag == FB)
	{
		if(m_spd_n <= ((INT32)m_spd_ref_n<<1))
//		if(m_spd_n <= (((INT32)m_spd_ref_n*4)>>1))
//		if(m_spd_n <= ((m_spd_ref_n * 4) >> 3))
		{
			preSatOut = normal_control();
		}
		else
		{
			preSatOut = fall_back_tactic();
		}
	}
	else
	{
		if(door_ac == 1)
		{
			if(door_dt == 1)	
			{
				if(tactic_flag == DT)
				{
					if(m_spd_n < ( m_spd_ref_n * 3 ))//modified by zz on 20100311
					{
						preSatOut = dither_tactic();
					}
					else			
					{	
						preSatOut = fall_back_tactic();
					}
				}
				else				
				{
					if(iq_max_tester >= 20) 
					{
						preSatOut = dither_tactic();
					}
					
					else				
					{
			
						if(m_spd_n < 60)	
						{
								
							if(speed_min_tester >= 30)
							{
								speed_min_tester = 0;
								preSatOut = dither_tactic();
							}
							else
							{
								preSatOut = acc_tactic();
							}
				
						}
			
						else				
						{
							switch(tactic_flag)
							{
								case AC:	
					
									if(m_spd_n > ( ((INT32)m_spd_ref_n * 10) >> 3))//modified by zz on 20100311
									{
							
										if(m_spd_n_last > ( ((INT32)m_spd_ref_n * 10) >> 3))//modified by zz on 20100311
										{
											rpm_keeper++;
										}
										else				
										{
											rpm_keeper = 0;
										}
						
													
										if(rpm_keeper >= AC_HOLD_TIME)		
										{
											rpm_keeper = 0;
											preSatOut = fall_back_tactic();
										}
										else				
										{
											preSatOut = acc_tactic();
										}
							
									}
									else				 
									{
										preSatOut = acc_tactic();
									}
						
									break;
					
								case NC:	
						
									if(m_spd_n < ( (m_spd_ref_n * 6) >> 3))	
									{
										transfer_sp = 7;
										transfer_si = 30;
										preSatOut = normal_control();
									}
									else				
									{
										if(m_spd_n < ( ((INT32)m_spd_ref_n * 12) >> 3))//modified by zz on 20100311	
										{
											preSatOut = normal_control();
										}
										else				
										{
											transfer_sp = 7;
											transfer_si = 50;
											preSatOut = normal_control();
										}
									}
				
									break;
							}
				
						}
					}
				}
			}
			else
			{
				preSatOut = normal_control();
			}
		}
		else
		{
			preSatOut = normal_control();	
		}
		
	}
}


else				
{
		preSatOut = normal_control();	
}



	
	m_spd_n_last = m_spd_n;

	return (INT16)preSatOut;
	
}

//*************************
//normal_control()
//*************************
INT16 normal_control()
{
	INT32 preSatOut;
    INT16 err;
	INT32 up;
	INT16 pos_spd_ref;
	INT16 sp;
	INT16 si;
	
	pos_spd_ref = pos;
	sp = transfer_sp;
	si = transfer_si;
	
	tactic_flag = NC;
	
	err = m_spd_ref_n  - m_spd_n + pos_spd_ref ;
	
	up = ((INT32)err)*sp;
	
    sum_err_s = sum_err_s + ((((INT32)si)*up)>>11);

		if((sum_err_s) > 8000)                                       //2010-08-13
			sum_err_s = 8000;
		else if((sum_err_s) < -8000)	
			sum_err_s = -8000;
	
	preSatOut = up + sum_err_s;
	
	if(preSatOut > PID_SPD_MAX_P)

	    { 
			preSatOut = PID_SPD_MAX_P;
		}

	if(preSatOut < -PID_SPD_MAX_N)
		{
	 		preSatOut = -PID_SPD_MAX_N;	
		}
	
	return (INT16)preSatOut;	
}

//****************************
//acc_tactic()
//****************************
INT16 acc_tactic()
{	
	INT16 preSatOut;
	
	current_response_speed = 4;
	
	tactic_flag = AC;
	
	preSatOut = PID_SPD_MAX_P;

	return preSatOut;
}


//****************************
//dither_tactic()
//****************************
INT16 dither_tactic()
{	
	INT16 preSatOut;
	INT16 move_bit;
	
		move_bit = 2;
	current_response_speed = 4;
	
	tactic_flag = DT;
	
	if(motor.iq == PID_SPD_MAX_P)	
	{
		if(motor.iq_last == PID_SPD_MAX_P)	
		{
			counter_H++;
		}
		else								
		{
			counter_H = 0;
		}
		if(counter_H > H_time)		
		{
			counter_H = 0;
			preSatOut = PID_SPD_MAX_P >> move_bit;
		}
		else					
		{
			preSatOut = PID_SPD_MAX_P;
		}
		
	}
	else							
	{
		if(motor.iq_last == (PID_SPD_MAX_P >> move_bit))	
		{
			counter_L++;
		}
		else
		{
			counter_L = 0;
		}
		
		if(counter_L >= L_time)
		{
			counter_L = 0;
			preSatOut = PID_SPD_MAX_P;
		}
		else
		{
			preSatOut = PID_SPD_MAX_P >> move_bit;
		}
	}
	
	return preSatOut;
	
}


//****************************
//fall_back_tactic()
//****************************
INT16 fall_back_tactic()
{
	INT16 preSatOut;
	
	current_response_speed = 4;
	tactic_flag = FB;
	
	preSatOut = -4000;
	
	return preSatOut;
}


//--------------------------------------------------------------------------------------
//  Name:	cal_pid_p_stop
//  Parameters:	None
//  Returns:	None
//  Description: calculate position loop stop PID
//--------------------------------------------------------------------------------------
INT16 cal_pid_p_stop(INT16 pp, INT16 ps, INT16 is)
{
	INT16 preSatOut;
	INT16 err_p;
	INT16 spd_err;
	
	err_p =  motor.stop_angle1- motor.angle_adjusted;  //2010-4-7
	

     
	spd_err = err_p * pp - m_spd_n;

	sum_spd_s = sum_spd_s + spd_err;
	sum_spd_s = (ps * is * sum_spd_s)>>10;
	limit(sum_spd_s,500);
	preSatOut = spd_err * ps + sum_spd_s;
		
    limit(preSatOut,PID_SPD_MAX_P);
	
//	if ((strong > 0)||(motor.stop_angle1 == motor.stop_angle))
	if ( (abs(motor.stop_angle1 - motor.stop_angle) <= 5) || (abs(motor.stop_angle1 - motor.stop_angle)>ANGLE_P) )
	 motor.stop_angle1 = motor.stop_angle;
	else
     motor.stop_angle1 = motor.stop_angle1+kk*1;
	
	return (INT16)preSatOut;
}
//--------------------------------------------------------------------------------------
//  Name:	m_start
//  Parameters:	None
//  Returns:	None
//  Description: motor start function
//--------------------------------------------------------------------------------------
void m_start(INT16 ta3_t)
{
    INT16 delta_ta3;
	INT16 temp16;
    INT16 Ud_r;
    INT16 Uq_r;
    INT16 i;

	if(abs(delta_l) >= ((ENCODER>>1) + DEGREE_20))
	{
		motor_stuck_flag = 1;//ERROR_14
	}
	
	zero_vol_counter ++;
	if(zero_vol_counter >= 2000)
		zero_vol_counter = 2000;
	m_count++;
	if(stop_change_counter <= 20)
	{	
		if(m_count >= 200)
		{
			m_count = 0;
			delta_ta3 = ta3 - ta3_last;
			ta3_last = ta3;
			if(delta_ta3 != 0)
			{
				if(delta_ta3 > 0)
					real_dir = 0;
				else if(delta_ta3 < 0)
					real_dir = 1;
			
				if(motor.dir ^ real_dir)
				{
					stop_change_counter ++;
					ta3_start = ta3_start + (M_CODER - 70);
					if(ta3_start >= ENCODER)
					{	
						ta3_start -= ENCODER;
					}
				}
				else
				{
					if((abs(delta_ta3) <= 2*ENCODER/360) && (zero_vol_counter >= 2000))
					{
						stop_change_counter ++;
						ta3_start = ta3_start + (M_CODER >> 4);
						if(ta3_start >= ENCODER)
						{	
							ta3_start -= ENCODER;
						}
					}
				}
			}
			else
			{
				stop_change_counter ++;
				ta3_start = ta3_start + (M_CODER - 70);
				if(ta3_start >= ENCODER)
				{	
					ta3_start -= ENCODER;
				}
			}
			
		}
		else
		{
			delta_ta3 = ta3 - ta3_last;
			if(motor.dir == 1 && delta_ta3 >= 2*ENCODER/360)
			{
				stop_change_counter ++;
				ta3_last = ta3;
				m_count = 0;
				ta3_start = ta3_start + (M_CODER - 70);
				if(ta3_start >= ENCODER)
				{	
					ta3_start -= ENCODER;
				}
			}
			else if(motor.dir == 0 && delta_ta3 <= -2*ENCODER/360)
			{
				stop_change_counter ++;
				ta3_last = ta3;
				m_count = 0;
				ta3_start = ta3_start + (M_CODER - 70);
				if(ta3_start >= ENCODER)
				{	
					ta3_start -= ENCODER;
				}
			}
		}
	}
			
			
		cal_spd_ref(50,0,0);
				
	  	motor.iq = cal_pid_s(0,20,2);
	  	
			if(motor.iq >= 0)
				e_start = 20;
			else
				e_start = 6;	

			
		        
		         
				
		Ud_r = (((INT32)(motor.iq) * L *m_spd_n)>>(LQ_Q+CURRENT_Q)); 

	    Uq_r = ((INT32)(motor.iq) * R>>CURRENT_Q) + (((INT32)(m_spd_n) * e_start)>>(KE_Q)) ;
		
            temp16 = ((INT32)Ud_r<<7)/Uq_r;

			if(temp16 > 0)
			{					
				while(abs(top - bottom) > error)
				{
					mid = (top + bottom);
					mid = mid>>1;
					
					if(temp16 > tan_tab[mid])
					    bottom = mid;
					else
					    top = mid;	
			    }
			    i = mid;
			}
			else
			{
				temp16 = abs(temp16);	
				while(abs(top - bottom) > error)
				{
					mid = (top + bottom);
					mid = mid>>1;
					
					if(temp16 > tan_tab[mid])
					    bottom = mid;
					else
					    top = mid;	
				}
				i = mid;
				i = -i;
			}
					
			if(motor.dir)
				i = -i;
  
       		temp16 = (((abs(Ud_r))>>2) + abs(Uq_r));					
			if(Uq_r>0)
			{
				alpha = 0xff+i; 					
			}
			else
			{
				alpha = 0x2ff+i;
			}

				
				if(abs(motor.iq) > 6000)
				{
					temp16 += 95;
				}
				else if(abs(motor.iq) > 5000)
				{
					temp16 += 85;
				}
				else if(abs(motor.iq) > 4000)
				{
					temp16 += 75;
				}		
				else if(abs(motor.iq) > 3000)
				{
					temp16 += 65;
				}		
				else if(abs(motor.iq) > 2000)
				{
					temp16 += 55;
				}		
				else if(abs(motor.iq) > 1500)
				{
					temp16 += 45;
				}
				else if(abs(motor.iq) > 1000)
				{
					temp16 += 35;
				}
				else if(abs(motor.iq) > 800)
				{
					temp16 += 25;
				}
				else if(abs(motor.iq) > 400)
				{
					temp16 += 15;
				}
				else if(abs(motor.iq) > 200)
				{
					temp16 += 5;
				}	
  


       
#ifdef VOL_ADJUST
	//voltage adjust
			temp16 = ((INT32)(temp16*(INT32)DC300))/ (sys.uzk_val);   //Q12
#endif

		if(zero_vol_counter >= 20)
		{  
			if(temp16 > U_MAX)
				vol = U_MAX;
			else
				vol = temp16; 
		}	
		else
		{
				vol = 0;
		}		


		vol = (vol_last*7 + vol ) >> 3;
		vol_last = vol;
		

}

//--------------------------------------------------------------------------------------
//  Name:	cal_spd_ref
//  Parameters:	None
//  Returns:	None
//  Description: calculate reference speed
//--------------------------------------------------------------------------------------
void cal_spd_ref(INT16 speed, INT16 acc_spd, INT16 dcc_spd)
{
	static INT16 W1max;
	static INT16 AlphaMax;   // Maximum acceleration
	static INT16 Wmax;       // Maximum speed according to the acceleration 
	static INT16 spd_tmp_test;
	static INT16 Dec;

	if(speed < STOP_SPD)
	{
		motor.spd_ref = speed;
	}
	
	if(speed == motor.spd_ref)
    {
		if(motor.dir)
	        m_spd_ref_n = -motor.spd_ref<<1;   
	    else
	        m_spd_ref_n = motor.spd_ref<<1;
        return; 
    }
		
			
	spd_tmp_test = abs(spd_tmp);
	
	if(spd_tmp < 0)
	{
	    if(speed == STOP_SPD)
	    {
		
			Dec = 8; 
	        Wmax = 600;
	    }
		else if(speed < STOP_SPD)
	    {
	        Dec = 4;
	        Wmax = 600;
	    }
		else
		{
			Wmax = 1800;
			Dec = 6;
		}
	}	
	else
		Wmax = 800;
	
	if(spd_tmp_test > (Wmax<<1))
		W1max = spd_tmp_test - (Wmax<<1);
	else
	{
		W1max = 0;
		Wmax = spd_tmp_test>>1;
	}
 
	if(spd_tmp >= 10 || spd_tmp <= -10)
	{
		if(speed - motor.spd_ref != 0)
		{		
		    if(speed - motor.spd_ref > 10)
			{
				if(speed - motor.spd_ref >= W1max + Wmax)
				{
					TaccTmp = TaccTmp + 3;
					motor.spd_ref = motor.spd_ref + TaccTmp;
				}
				else if(speed - motor.spd_ref < W1max + Wmax && speed - motor.spd_ref >= Wmax )
				{
					motor.spd_ref = motor.spd_ref + TaccTmp;
				}
				else if(speed - motor.spd_ref < Wmax)
				{
					TaccTmp = TaccTmp - 3;
					motor.spd_ref = motor.spd_ref + TaccTmp;
					if(TaccTmp <= 5)
					{
						TaccTmp = 5;
					}
					if(abs(speed - motor.spd_ref) <= 10)
					{
						motor.spd_ref = speed;
						spd_tmp = 0;
						TaccTmp = 0;
					}
				}
			}
			else if(speed - motor.spd_ref < -10)
			{
				if(speed - motor.spd_ref <= -(W1max + Wmax))
				{
					TaccTmp = TaccTmp - Dec;
					motor.spd_ref = motor.spd_ref + TaccTmp;
				}
				else if(speed - motor.spd_ref > -(W1max + Wmax) && speed - motor.spd_ref <= -Wmax )
				{
					motor.spd_ref = motor.spd_ref + TaccTmp;
				}
				else if(speed - motor.spd_ref > -Wmax)
				{
					TaccTmp = TaccTmp + Dec;
					motor.spd_ref = motor.spd_ref + TaccTmp;
					if(TaccTmp >= -2*Dec)
					{
						TaccTmp = -2*Dec;
					}
					if(abs(motor.spd_ref - speed) <= 5*Dec)
					{
						motor.spd_ref = speed;
						spd_tmp = 0;
						TaccTmp = 0;
					}
				}
			}
			else
			{
				motor.spd_ref = speed;
			    spd_tmp = 0;
			    TaccTmp = 0;
			}
		}
		else
		{
			motor.spd_ref = speed;
			spd_tmp = 0;
			TaccTmp = 0;
		}
	}
	else
	{
		spd_tmp = speed - motor.spd_ref;//when there is no spt_tmp caculate before call 'cal_spd_ref()',it works
	}
		
	
    if(motor.dir)
        m_spd_ref_n = -motor.spd_ref<<1;
    else
        m_spd_ref_n = motor.spd_ref<<1;
		

}

void int0_int (void)
{
	
	if((motor.angle < 25) || (motor.angle > 1000))
	{	
		ta3z_elec = ta3;
		ta3z_mach = ta3z_elec;		
	//-----------------------------------------------------------------------	
    // modify for 210E 
    //-----------------------------------------------------------------------	
    zpl_pass = 1;                // ZPL have passed
	
	}
}
//--------------------------------------------------------------------------------------
//  Name:	motor_start
//  Parameters:	None
//  Returns:	None
//  Description: start motor
//--------------------------------------------------------------------------------------
void motor_start(void)
{
	if(m_status >= OPEN_START)
	{
	}
	else
	{
		l_ism = ISM;			
		if(l_ism)
			motor.dir = 1;
		else
		 	motor.dir = 0;

		U=1;U_=1;V=1;V_=1;W=1;W_=1;
		prcr = 0x02;
		inv03 = 1;
		prcr = 0x00;
		ta3_start = ta3;
		m_status = OPEN_START;
	}
}

//--------------------------------------------------------------------------------------
//  Name:	motor_stop
//  Parameters:	None
//  Returns:	None
//  Description: stop motor
//--------------------------------------------------------------------------------------
UINT8 motor_stop(INT16 stop_speed)
{
	INT16 S;
	INT16 temp16;
	
	if(motor.spd_ref <= 800)
	{
		spd_tmp = STOP_SPD - motor.spd_ref;
		return 2;
	}
	
	if(motor.spd_ref > 800)
	{
		spd_tmp = 800 - motor.spd_ref;
		cal_spd_ref(motor.spd_obj, motor.acc, motor.dec);
	}
	return 1;	

} 

//--------------------------------------------------------------------------------------
//  Name:	motor_control
//  Parameters:	None
//  Returns:	None
//  Description: motor control function
//--------------------------------------------------------------------------------------
INT8 e = 30;
void motor_control(void)
{
    INT16 temp16,dir;
    INT16 i;
    INT16 Ud_r;
    INT16 Uq_r;
	INT16 iq_r;
	INT32 didt;
	INT16 object_speed;
  	
    top = 255;
	bottom = 0;

	object_speed = motor.spd_obj;
	
    //save the histroy 	
	if(object_speed - spd_last_value != 0)
	{
		spd_tmp = object_speed - motor.spd_ref;
	}	
		
	spd_last_value = object_speed;
	
    a_m_spd += abs(m_spd_n);
	count_spd ++;
    if(count_spd >= 512)
    {
         count_spd = 0;
         motor.spd = (a_m_spd+512)>>10;
         a_m_spd = 0;
    }

	if(m_status > OPEN_START)
	{
		if(object_speed > 0)
		{
			motor.stop_flag = 0;
			stop_status = 0;
			pwm_forbid_flag = 0;
			
			if(DIDT_SPEED_RANGE == 0)
			{
				if(m_spd_ref_n <= 2000)
					current_response_speed = 4;
				else
					current_response_speed = 0;
			}
			else
			{
				current_response_speed = 0;
			}
			if(m_status == CLOSE_RUN)
		    {
				
				if(m_spd_n == 0)
				{
					cal_spd_ref(object_speed, motor.acc, motor.dec);
					if((m_spd_ref_n >= 400) && (s_count >= 50))
					{
							tactic_allowed = 1;
					}
					else
					{
							tactic_allowed = 0;
					}						
    			    
					motor.iq = cal_pid_s(0, (KPs_L), (KIs_L));
										
				}
				else
				{
					if((object_speed != motor.spd_ref))
					{
						cal_spd_ref(object_speed, motor.acc, motor.dec);
						s_count = 0;

	               		if (spd_tmp > 0)
						{
					      	if (m_spd_n < 1500)
						    {	
									tactic_allowed = 0;
								motor.iq = cal_pid_s(0, (KPs_up_L), KIs_up_L);  
						    }
						    else
						    {	
									tactic_allowed = 0;
							    motor.iq = cal_pid_s(0, (KPs_up_H), KIs_up_H);  
							}
						}
						else if (spd_tmp < 0)
						{
							if (m_spd_n < 4000)
						  	{
								  	tactic_allowed = 0;
								motor.iq = cal_pid_s(0, (KPs_down_L), KIs_down_L); 
					        }
					    	else
					        {	
								   	tactic_allowed = 0;
				 				motor.iq = cal_pid_s(0, (KPs_down_H), KIs_down_H);  //KIp 
							 }
		 				}
					}
					else
					{

						cal_spd_ref(object_speed, motor.acc, motor.dec);
						if(s_count >= 50)
						{
							if(m_spd_ref_n <= 2000)
								tactic_allowed = 1;
							else
								tactic_allowed = 0;
							
							if (m_spd_n <= 2000)
							{	
								motor.iq = cal_pid_s(0, (KPs_L), KIs_L);
							}
						    else if(m_spd_n < 3000)
							{	
								motor.iq = cal_pid_s(0, (KPs_M), KIs_M);
							}
							else
							{	
							    motor.iq = cal_pid_s(0, (KPs_H), KIs_H);	
							}
							
						}
						else
						{
							s_count++;
						    
							if (m_spd_n < 3000)
							{
								tactic_allowed = 0;
								motor.iq = cal_pid_s(0, (KPs_trans_L), (KIs_trans_L)); //reach objtect speed for 50ms
							}
							else
							{
		   		     			tactic_allowed = 0;
		   		    			motor.iq = cal_pid_s(0, (KPs_trans_H), (KIs_trans_H)); //reach objtect speed for 50ms    	
							}

						}
					}
				}
					
		    }
			else
			{
				U=1;U_=1;V=1;V_=1;W=1;W_=1;
	   			prcr = 0x02;
			    inv03 = 1;
	   			prcr = 0x00;
	            m_status = CLOSE_RUN;
			}
		}
		else 
		{
			current_response_speed = 0;
			tactic_allowed = 0;
			if(m_status == CLOSE_RUN)
			{
				
				switch(stop_status)
				{
				        case 0:
				        	if(find_deadpoint_flag == 0)
							{
					        	if(motor.spd_ref <= 800)
					        	{
									if(motor.spd_ref > STOP_SPD)
									{
									#if ONESTEP_STOP
										if( motor.angle_adjusted <= DEGREE_202)
                                    #else
										if(  motor.angle_adjusted >= DEGREE_165&&motor.angle_adjusted <= DEGREE_202)
									#endif
										{
											spd_tmp = STOP_SPD - motor.spd_ref;
							        		cal_spd_ref(STOP_SPD, motor.acc, motor.dec);
							        		stop_status = 2;
										}
									}
									else
									{
										spd_tmp = STOP_SPD - motor.spd_ref;
							        	cal_spd_ref(STOP_SPD, motor.acc, motor.dec);
							        	stop_status = 2;
									}
					        	}
					        	else
					        	{
					        		if(motor.angle_adjusted > DEGREE_202)
									{
										spd_tmp = DEC_SPD - motor.spd_ref;
										cal_spd_ref(DEC_SPD, motor.acc, motor.dec);
										stop_status = 1;	
									}
								}
								motor.iq = cal_pid_s(0, KPs_stop_0, KIs_stop_0);
							}
							else
							{
								cal_spd_ref(DEADPOINT_SPD, motor.acc, motor.dec);
								stop_status = 2;
								motor.iq = cal_pid_s(0, KPs_deadpoint, KIs_deadpoint);
							}
							
							
							break;
						case 1:
							if(motor.spd_ref <= 800)
							{
								if(motor.angle_adjusted >= DEGREE_165 && motor.angle_adjusted <= DEGREE_202)
								{
									spd_tmp = STOP_SPD - motor.spd_ref;
								    cal_spd_ref(STOP_SPD, motor.acc, motor.dec);
								    stop_status = 2;
									motor.iq = cal_pid_s(0, (KPs_stop_1),KIs_stop_1);
								}
								else
								{
									cal_spd_ref(DEC_SPD, motor.acc, motor.dec);
									motor.iq = cal_pid_s(0, (KPs_stop_1),KIs_stop_1);
								}
				        	}
							else
							{
								cal_spd_ref(DEC_SPD, motor.acc, motor.dec);
								motor.iq = cal_pid_s(0, KPs_down_L, KIs_down_L);
							}
				        	break;	
						case 2:

							if(find_deadpoint_flag == 0)
							{
								current_response_speed = DIDT_CUT_THREAD;
								cal_spd_ref(STOP_SPD, motor.acc, motor.dec);
							}
							else
							{
								cal_spd_ref(DEADPOINT_SPD, motor.acc, motor.dec);
							}
							//holding axes  M_CODER  (M_CODER<<1)  ((M_CODER<<1)-1)
							if(motor.stop_angle >= 0 && motor.stop_angle <= ((M_CODER<<1)-1) )
							{
								if(motor.angle_adjusted >= 0 && motor.angle_adjusted <= (motor.stop_angle + (M_CODER<<1)))
								{
									motor.angle_hold = motor.stop_angle - motor.angle_adjusted;
								}
								else//(motor.angle_adjusted <= 1023 && motor.angle_adjusted > (motor.stop_angle + 512))
								{
									motor.angle_hold =  (M_CODER<<2) - motor.angle_adjusted + motor.stop_angle;
								}
								while(motor.angle_hold > (M_CODER<<1))
								{
									motor.angle_hold -= (M_CODER<<1);
								}
								while(motor.angle_hold < -((M_CODER<<1)-1))
								{
									motor.angle_hold += (M_CODER<<1);
								}
							}
							else//(motor.stop_angle >= 512 && motor.stop_angle <= 1023)
							{
								if(motor.angle_adjusted > (motor.stop_angle - (M_CODER<<1)) && motor.angle_adjusted <=((M_CODER<<2)-1))
								{
									motor.angle_hold = motor.stop_angle - motor.angle_adjusted;
								}
								else//(motor.angle_adjusted >= 0 && motor.angle_adjusted <= (motor.stop_angle - 512))
								{
									motor.angle_hold = motor.stop_angle - (M_CODER<<2) - motor.angle_adjusted ;
								}
								while(motor.angle_hold > (M_CODER<<1))
								{
									motor.angle_hold -= (M_CODER<<1);
								}
								while(motor.angle_hold < -((M_CODER<<1)-1))
								{
									motor.angle_hold += (M_CODER<<1);
								}
							
							}
							if(motor.dir)
							{
								temp16 = -motor.angle_hold;
							}
							else
							{
								temp16 = motor.angle_hold;
							}
						#if ONESTEP_STOP
							if(find_deadpoint_flag == 0)
							{
								motor.iq = cal_pid_s(0, KPs_stop, KIs_stop);//???
							}
							else
							{
								motor.iq = cal_pid_s(0, KPs_deadpoint, KIs_deadpoint);
							}
							if(temp16 <= ANGLE_P && temp16 > 0)//2010-4-7
							{ 
							   if(motor.spd_ref > STOP_SPD)
								   break;
								if(motor.angle_hold > 0)//2010-4-7
							   kk = 1;
							   else
							   kk = -1;
							   motor.stop_angle1 = motor.angle_adjusted + 2*kk;//2010-4-7
							   stop_status = 3;
							   m_spd_ref_n = 0;  
							   m_count = 0;
														
							}
					#else
							if(temp16 <= ANGLE_P && temp16 > 0)//2010-4-7
							{ 
							   if(motor.angle_hold > 0)//2010-4-7
							   kk = 1;
							   else
							   kk = -1;
							   motor.stop_angle1 = motor.angle_adjusted + 2*kk;//2010-4-7
							   stop_status = 3;
							   m_spd_ref_n = 0;  
							   m_count = 0;
														
							}
							if(find_deadpoint_flag == 0)
							{
								motor.iq = cal_pid_s(0, KPs_stop, KIs_stop);//???
							}
							else
							{
								motor.iq = cal_pid_s(0, KPs_deadpoint, KIs_deadpoint);
							}
					#endif
							break;
						
						case 3:
							m_count++;
							if(m_count < 150)  
								 motor.iq = cal_pid_p_stop(STOP_KPp_1,STOP_KPs_1,0);
							else
								motor.iq = cal_pid_p_stop(STOP_KPp_2,STOP_KPs_2,0);
							if(m_count > 100)
							{
								if(sys.status == RUN || sys.status == PREWIND || sys.status == WIND)
								{
									motor.stop_flag = 1;
									motor.spd_ref = 0;	
							
									if(motorconfig_flag == 1)
									{
										if(inpress_flag == 0)
										{
											MotorPositionSet = 1;
										}
										else if(inpress_flag == 1)
										{
											MotorPositionSet = 0;
										}
									}
									else
									{
										MotorPositionSet = 0;
									}
								}
								else
								{
									motor.stop_flag = 1;
									motor.spd_ref = 0;
									MotorPositionSet = 0;
								}
							}
						
							if(MotorPositionSet == 0 && motor.stop_flag == 1 && m_count >= 400)
							{
								U=1;U_=1;V=1;V_=1;W=1;W_=1;
				    			prcr = 0x02;
							    inv03 = 0;
								prcr = 0x00;	    			
								U_=0;V_=0;W_=0;
								stop_status = 5;
								TaccTmp = 0;
								s_count = 0;
								motor.spd_ref = 0;
								m_status = STOP;
								motor.iq = 0;
								spider_man = 0;
								spider_lim = 0;
							}
							break;
					}
				
			}
			else 
			{
				m_pos_ref_add = motor.angle_adjusted<<6;
				m_pos_ref = m_pos_ref_add >> 6;
				motor.stop_flag = 1;
				stop_status = 4;
			}
		}

  
if (abs(m_spd_n) < 100)
	{ 
		if (motor.iq>=0)  
	    {
	    	if (abs(motor.iq) < 548)	
		      e=6;//14;
		    else if (abs(motor.iq) < 913)
		      e=8;//14
		    else if (abs(motor.iq) < 2372)
		      e=12;//14
			else if (abs(motor.iq) < 3000)
			  e = 13;//15
		    else if (abs(motor.iq) < 4300)
			  e=16;//18
			else if (abs(motor.iq) < 6388)
		      e=20;
		    else if (abs(motor.iq) < 9600)
		      e=22;
		    else 
		      e=22;
		   }
	  	else
		  {		
				e = 14;//14
		  }
	}
     
	else if (abs(m_spd_n) < 700)
	  { 
		if (motor.iq>=0)  
	    {
	    	if (abs(motor.iq) < 548)	
		      e=14;
		    else if (abs(motor.iq) < 913)
		      e=14;//14
		    else if (abs(motor.iq) < 2372)
		      e=14;//14
			else if (abs(motor.iq) < 3000)
			  e = 15;//15
		    else if (abs(motor.iq) < 4300)
			  e=18;//18
			else if (abs(motor.iq) < 6388)
		      e=25;
		    else if (abs(motor.iq) < 9600)
		      e=25;
		    else 
		      e=25;
		   }
	  else
		  {		
			e = 14;//14
			  			  
		   }
		}	
		
		
	else if (abs(m_spd_n) < 1200)
	  {
	    if (motor.iq>=0)  
		{
		  	if(abs(motor.iq) < 540)
				e = 14;
			else if (abs(motor.iq) < 913)
		    	e=14;//21
			else if(abs(motor.iq) < 1100)
				e=14;//24
			else if(abs(motor.iq) < 1500)
				e = 14;
			else if(abs(motor.iq) < 2300)
				e = 15;
			else if(abs(motor.iq) < 2800)
				e = 16;			
			else if (abs(motor.iq) < 3200)
		   		e=19;
			else if (abs(motor.iq) < 9600)
		   		e=27;
			else 
		   		e=28;
		}
		else
		{		
			e = 14;
			}
		}
		 
	else if (abs(m_spd_n) < 2800)
      {  
		 if (motor.iq>=0)  
		 {
			if (abs(motor.iq) < 1460)
			   e=22;  
			else if (abs(motor.iq) < 6388)
			   e=27;
			else
			   e=28;
			   }
		 else
		   {
				e=15;  
		   }
		   }
    else if (abs(m_spd_n) < 3600)
	  {
		if (motor.iq>=0) 
		{
			if (abs(motor.iq) < 1460)
			   e=24;
			else if (abs(motor.iq) < 4000)
			   e=24;  //26 	   
			else if (abs(motor.iq) < 6388)
			   e=25;   //28
			else
			   e=30;
			}
		else
		{
				e=15;
			}
		}
	else if (abs(m_spd_n) < 5600)// already done
	  {  
		if (motor.iq>=0) 
		{
			if (abs(motor.iq) < 913)
		   e=25;//28
		else if (abs(motor.iq) < 3650)
		   e=25;//28
		else if (abs(motor.iq) < 6388)
		   e=28;  //29
		else
		   e=29;
		   
		   }
		else
		 {
				e=15;
			}
		  }
	else
	   { 
		   if (motor.iq>=0) 
		   {
			if (abs(motor.iq) < 1460)
			   e=28;
			else if (abs(motor.iq) < 2372)
			   e=28;
			else if (abs(motor.iq) < 4927)
			   e=28;      // 29
	        else
			   e=30;
			   }
		   else
		   {
					e=15;
			   }
		   }
		
		didt = (INT32)(motor.iq-motor.iq_last);			

		if(didt >= 0)
		{
			if(didt > DERV)
				didt = DERV;
		}
		else
		{
			if(tactic_flag == 0)
			{
				didt = 0;
			}
			else
			{
				if(didt < (-DERV))
					didt = (-DERV);
			}
		}	
		
		
		Ud_r = (((INT32)(motor.iq) * L*m_spd_n)>>(LQ_Q+CURRENT_Q)); 

	    Uq_r = ((INT32)(motor.iq) * R>>CURRENT_Q) + (((INT32)(m_spd_n) * e)>>(KE_Q)) + (((((INT32)didt*L*5)>>9)*current_response_speed)>>2);//modified by zz on 20100311


			motor.iq_last = motor.iq;	
		   
		    temp16 = ((INT32)Ud_r<<7)/Uq_r;

			if(temp16 > 0)
			{					
				while(abs(top - bottom) > error)
				{
					mid = (top + bottom);
					mid = mid>>1;
					
					if(temp16 > tan_tab[mid])
					    bottom = mid;
					else
					    top = mid;	
			    }
			    i = mid;
			}
			else
			{
				temp16 = abs(temp16);	
				while(abs(top - bottom) > error)
				{
					mid = (top + bottom);
					mid = mid>>1;
					
					if(temp16 > tan_tab[mid])
					    bottom = mid;
					else
					    top = mid;	
				}
				i = mid;
				i = -i;
			}
					
			if(motor.dir)
				i = -i;
       
        temp16 = (((abs(Ud_r))>>2) + abs(Uq_r));
					
			      	 if(Uq_r>0)
					    	{
						    	alpha = 0xff+i;
					      	}
					     else
						    {
							    alpha = 0x2ff+i;
					       	}
		
		if(abs(motor.iq) > 6000)
		{
			temp16 += 95;
		}
		else if(abs(motor.iq) > 5000)
		{
			temp16 += 85;
		}
		else if(abs(motor.iq) > 4000)
		{
			temp16 += 75;
		}		
		else if(abs(motor.iq) > 3000)
		{
			temp16 += 65;
		}		
		else if(abs(motor.iq) > 2000)
		{
			temp16 += 55;
		}		
		else if(abs(motor.iq) > 1500)
		{
			temp16 += 45;
		}
		else if(abs(motor.iq) > 1000)
		{
			temp16 += 35;
		}
		else if(abs(motor.iq) > 800)
		{
			temp16 += 25;
		}
		else if(abs(motor.iq) > 400)
		{
			temp16 += 15;
		}
		else if(abs(motor.iq) > 200)
		{
			temp16 += 5;
		}	
		   
		   
#ifdef VOL_ADJUST
		    //voltage adjust
			temp16 = ((INT32)(temp16*(INT32)DC300))/ (sys.uzk_val);   //Q12
#endif
          	if(temp16 > U_MAX)
				vol = U_MAX;
			else
				vol = temp16; 
				
	}
	else if(m_status < OPEN_START)
	{
		if(object_speed > 0)
		{	
			U=1;U_=1;V=1;V_=1;W=1;W_=1;
			prcr = 0x02;
			inv03 = 1;
			prcr = 0x00;
			m_status = OPEN_START;
		}
		m_pos_ref_add = motor.angle_adjusted<<6;
		m_pos_ref = m_pos_ref_add >> 6;
	}
	else
	{
		m_pos_ref_add = motor.angle_adjusted<<6;
		m_pos_ref = m_pos_ref_add >> 6;
	}

}

#endif


//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------