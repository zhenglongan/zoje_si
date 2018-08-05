//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : motor.c
//  Description: motor control arithmetic
//  Version    Date     Author    Description
//  ...
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------

#include "..\..\include\sfr62p.h"            // M16C/62P special function register definitions
#include "..\..\include\typedef.h"           // data type define
#include "..\..\include\common.h"            // External variables declaration
#include "..\..\include\variables.h"         // External variables declaration
#include "..\..\include\action.h"            // action function
#include "..\..\include\solenoid.h"          // solenoid driver definition

#if USE_SEVERO_MOTOR_0830
#include "..\..\include\motor_def_0830.h"    // constants definitio
#include "..\..\include\math_tab.h"          // sine table     

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
static UINT8 index_flag;

static INT16 m_count;
static INT16 ta3z_elec;
static INT16 ta3z_mach;
static INT16 alpha;
static INT16 vol; 
//static INT16 m_spd_ref_n;   
//static INT16 m_spd_n; 	
static INT16 m_pos_ref;	  
static UINT16 m_pos_ref_add;
static INT16 m_pos_n;
static INT32 a_m_spd;
static INT16 count_spd;
static INT16 kk;
static INT32 sum_err_s;
static INT16 sum_err_p;
static INT16 sum_err_p_stop;

static UINT8 start_flg;
static UINT8 start_status;
static UINT8 start_count;
static INT16 ta3_start;
static INT16 ta3_dir;

static INT16 s_count;

//static UINT8 m_status;
static INT16 pos_const;
static INT16 Y_k;
static INT32 m_spd_n_add;
static UINT8 ta3_ind;

static INT16 ta3_buf[1<<SAMPLES_BIT];
static INT16 sum_spd_s;

static stop_status;
static INT16 didt_enable;
UINT8 l_ism;

volatile INT16 spd_tmp;
volatile INT16 spd_tmp_good; 
static INT16 etable_iq = 0;

INT16 spd_last_value;
INT16 spd_lasttime;
INT16 TaccTmp;
UINT16 zerocounter;
INT16 PI_out;
INT16 SatErr;
INT16 Spd_Tmp;
UINT16 r;
UINT16 e;
UINT32 StopCounter;
UINT16 z;
INT16 ta3n;
INT16 DelayCount;
INT16 DelayCountLimit;
UINT16 PauseFlag = 0;
INT16 good=0;
UINT16 motor_stuck_counter = 0;
UINT8 motor_run_status = 0;
UINT16 choose_flag = 0; 


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
INT16 cal_pid_p(INT16 pp, INT16 pi);
void cal_spd_ref(INT16 speed, INT16 acc_spd, INT16 dcc_spd);

//--------------------------------------------------------------------------------------
// 	Constants definition
//--------------------------------------------------------------------------------------
#define A_1 27853  //(25Hz,0x71b5),(50Hz,0x664d),(100Hz,0x5532),(120Hz,0x4fdb),(150Hz,0x48fa),(200Hz,0x3fd8),(220Hz,0x3cc8),(250Hz,0x38b8)                        
#define B0_1 4915 //(25Hz,0x0e4a),(50Hz,0x19B2),(100Hz,0x2acd),(120Hz,0x3024),(150Hz,0x3705),(200Hz,0x4027),(220Hz,0x4337),(250Hz,0x4747)

#define A_0 16306 //27853  //(25Hz,0x71b5),(50Hz,0x664d),(100Hz,0x5532),(120Hz,0x4fdb),(150Hz,0x48fa),(200Hz,0x3fd8),(220Hz,0x3cc8),(250Hz,0x38b8)                        
#define B0_0 16462//4915 //(25Hz,0x0e4a),(50Hz,0x19B2),(100Hz,0x2acd),(120Hz,0x3024),(150Hz,0x3705),(200Hz,0x4027),(220Hz,0x4337),(250Hz,0x4747)


const UINT8 etablenew1[53][21]=
{
	13,16,21,21,26,26,26,26,26,25,25,25,26,26,26,26,26,26,27,27,26,
	13,16,21,21,26,26,26,26,26,25,25,25,26,26,26,26,26,26,27,27,26,
	13,16,22,22,26,26,26,26,26,26,26,26,26,26,26,26,26,26,27,27,26,
	13,17,22,22,26,26,26,26,26,26,26,26,26,26,26,26,26,26,27,26,26,
	14,18,22,22,26,26,26,27,27,26,26,26,27,27,27,27,26,26,27,27,26,
	14,18,22,22,26,26,26,27,27,26,26,26,27,27,27,26,26,26,27,27,26,
	14,18,22,22,26,26,26,27,27,26,26,26,26,26,26,26,26,26,27,27,27,
	14,19,22,22,26,26,26,27,27,25,25,25,26,26,26,26,26,26,27,27,27,
	15,19,22,22,26,26,26,27,27,25,25,25,26,26,26,26,26,26,27,27,27,
	15,20,22,22,26,25,25,26,26,25,25,25,26,26,26,26,26,26,27,26,26,
	16,20,22,22,26,25,25,26,26,26,26,26,26,26,26,26,26,25,26,26,26,
	16,20,22,22,26,25,25,26,26,26,26,26,26,26,26,26,25,25,26,26,26,
	16,21,23,23,26,25,25,26,26,26,26,26,26,26,26,26,25,25,26,26,26,
	17,21,23,23,25,25,25,26,26,26,26,26,26,26,26,25,25,25,26,26,26,
	18,22,23,23,25,25,25,26,26,26,26,26,26,26,26,25,25,25,26,26,26,
	19,22,23,23,25,25,25,26,26,26,26,26,26,26,26,25,25,25,26,25,25,
	20,22,23,23,25,25,25,26,26,26,26,26,25,25,25,25,25,24,25,25,25,
	21,23,23,23,25,25,25,26,26,27,27,27,25,25,25,25,25,24,25,25,25,
	22,23,23,23,25,25,25,26,26,27,27,27,25,25,25,25,25,24,25,25,25,
	23,24,23,23,25,25,25,26,26,27,27,27,25,25,25,25,24,24,25,25,25,
	24,24,23,23,25,25,25,26,26,26,26,26,25,25,25,25,24,24,25,24,24,
	25,25,23,23,25,25,25,26,26,26,26,26,25,25,25,25,24,24,25,24,24,
	26,26,23,23,25,25,25,26,26,26,26,26,25,25,25,24,24,24,25,24,24,
	27,26,23,23,25,25,25,26,26,26,26,26,25,25,25,24,24,23,24,24,24,
	28,27,23,23,25,25,25,26,26,26,26,26,25,25,25,24,24,23,24,24,24,
	29,27,23,23,25,25,25,26,26,26,26,26,25,25,25,24,24,23,24,24,24,
	29,27,23,23,25,24,24,25,25,26,26,26,24,24,24,24,23,23,24,23,23,
	30,27,23,23,25,24,24,25,25,26,26,26,24,24,24,24,23,23,24,23,23,
	30,28,23,23,25,24,24,25,25,26,26,26,24,24,24,24,23,23,24,23,23,
	31,28,23,23,25,24,24,25,25,26,26,26,24,24,24,24,23,22,23,23,23,
	31,28,23,23,25,24,24,25,25,26,26,26,24,24,24,23,23,22,23,23,23,
	32,28,23,23,25,24,24,25,25,26,26,26,24,24,24,23,23,22,23,23,23,
	32,28,24,24,25,24,24,25,25,25,25,25,24,24,24,23,23,22,23,22,22,
	33,28,24,24,25,24,24,25,25,25,25,25,24,24,24,23,22,22,23,22,22,
	34,28,24,24,25,24,24,25,25,25,25,25,24,24,24,23,22,22,23,22,22,
	34,28,24,24,25,24,24,25,25,25,25,25,24,24,24,23,22,22,23,22,22,
	34,28,24,24,25,24,24,25,25,25,25,25,23,23,23,23,22,21,22,22,22,
	34,28,24,24,25,24,24,25,25,25,25,25,23,23,23,23,22,21,22,21,21,
	34,28,24,24,25,24,24,25,25,25,25,25,23,23,23,23,22,21,22,21,21,
	35,28,24,24,24,24,24,25,25,25,25,25,23,23,23,22,22,21,22,21,21,
	35,28,24,24,24,24,24,25,25,25,25,25,23,23,23,22,22,21,22,21,21,
	35,28,24,24,24,24,24,25,25,25,25,25,23,23,23,22,21,21,22,21,21,
	36,29,25,25,24,24,24,25,25,25,25,25,23,23,23,22,21,20,21,21,21,
	36,29,25,25,24,23,23,24,24,25,25,25,23,23,23,22,21,20,21,20,20,
	36,29,25,25,24,23,23,24,24,25,25,25,23,23,23,22,21,20,21,20,20,
	36,29,25,25,24,23,23,24,24,24,24,24,23,23,23,22,21,20,21,20,20,
	36,29,25,25,24,23,23,24,24,24,24,24,22,22,22,22,21,20,21,20,20,
	36,30,26,26,24,23,23,24,24,24,24,24,22,22,22,21,21,20,21,20,20,
	36,30,26,26,24,23,23,24,24,24,24,24,22,22,22,21,20,19,20,20,20,
	36,30,26,26,24,23,23,24,24,24,24,24,22,22,22,21,20,19,20,19,19,
	36,30,26,26,24,23,23,24,24,24,24,24,22,22,22,21,20,19,20,19,19,
	36,30,27,27,24,23,23,24,24,24,24,24,22,22,22,21,20,19,20,19,19
};

const UINT8 etablenew2[10][29]=
{
	7,7,7,7,6,6,5,5,6,6,7,8,9,9,10,11,12,13,14,15,16,16,17,17,18,19,19,20,20,
	7,7,7,7,8,8,8,8,9,9,10,10,11,12,13,13,13,14,15,15,16,17,17,18,19,19,20,20,21,
	8,8,8,8,9,9,10,10,11,11,12,12,13,13,14,15,16,16,17,17,18,18,19,19,20,20,21,21,22,
	9,9,9,9,9,9,10,10,11,11,12,13,14,14,15,16,17,17,18,18,19,19,20,20,21,21,22,22,23,
	9,9,9,9,9,9,10,10,11,11,12,13,14,14,15,16,17,17,18,18,19,19,20,20,21,21,22,22,23,
	9,9,9,9,9,9,10,10,11,11,12,13,14,14,15,16,17,17,18,18,19,19,20,20,21,21,22,22,23,
	9,9,9,9,9,9,10,10,11,11,12,13,14,14,15,16,17,17,18,18,19,19,20,20,21,21,22,22,23,
	9,9,9,9,9,9,10,10,10,10,11,11,11,12,12,13,14,16,17,11,11,11,13,15,17,19,20,21,23,
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,13,15,17,19,20,21,23,
	10,10,10,10,10,10,10,10,10,10,9,9,9,8,8,8,7,7,6,6,6,6,6,6,6,6,6,6,6	
};



//--------------------------------------------------------------------------------------
//  Name:		init_tb0
//  pars:	None
//  Returns:	None
//  Description: initial timer B0
//--------------------------------------------------------------------------------------
void init_tb0(void)
{
    tb0mr = 0x42;       // XX0X XX00   //脉冲周期测定模式 f8 dhd
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
    ta3mr = 0xd1;                       /* 4tuple-phase event count mode,free_run type   二相脉冲信号处理模式 dhd */
    udf |= 0x40;                        // two-phase pulse signal processing function enable   ta3
    onsf |= 0x20;                       /* Z-phase input is valid                              使能Z相，TA0IN 实际硬件没接 */
    trgsr &= 0xcf;                      /* input on TA3IN is selected                          TA3IN输入 */
    ta3 = 0;                            // m_counter clear 
    ta3ic = 0x00;                       // TA3 interrupt disable 
    ifsr0 = 0;                          // rising/falling edge select 
    int0ic = 0;			                // falling edge select 
    ta3s = 1;                           // count enable 
        
    //PWM_initialization
    ictb2 = 1;                          /* one TB2 underflow interrupt  1次下溢一次中断 */
    prcr  = 0x02;						/* 保护寄存器，允许写PM0、PM1、PM2、TB2SC、INVC0、INVC1 */
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
    tb2mr = 0x00;                       /* timer mode    时钟f1 */
    trgsr |= 0x45;                      // trigger select register TB2 trigger 
    tb2 = CARR_CNT - 1;                 /* carrier cycle   2400分频 100us */
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
    StopCounter	= 0;
	
	
	
	sum_spd_s = 0;
	
    //backford_frq
    index_flag = 0;
    
    ta3z_elec = 0;
    ta3z_mach = 0;

    m_pos_ref = 0;
    m_spd_ref_n = 0;
	m_spd_n = 0;
	m_pos_n = 0;
	a_m_spd = 0;
	count_spd = 0;
	
    sum_err_s = 0; 
    sum_err_p = 0;
	sum_err_p_stop = 0;
    TaccTmp = 0;
	spd_tmp = 0;
	spd_tmp_good = 0;
    alpha = 0;
    vol = 0;
	
	
	init_tb0();
	stop_status = 0;
    
    start_flg = 0;
    start_status = 0;
    start_count = 0;
	ta3_start = ta3;


	temp32 = ((INT32)M_CODER * 17895)>>(14-(POS_CONST_Q-4)); 
	pos_const = temp32/motor.dec;


    idb0 = 0x2a;                      
    idb1 = 0x15;                      	    
    tabsr |= 0x96;
	m_pos_ref_add = 0;
	
	s_count = 0;
	
	Y_k = 0;
    for(i=0;i<(1<<SAMPLES_BIT);i++)
        ta3_buf[i] = 0;
    ta3_ind = 0;
	
	didt_enable = 0;
	m_spd_n_add = 0;
	m_status = INIT;
	l_ism = 0;
}

//--------------------------------------------------------------------------------------
//  Name:	pwm_int
//  Parameters:	None
//  Returns:	None
//  Description: pwm interrupt routine
//--------------------------------------------------------------------------------------
UINT16 m_counter = 50;
UINT8 ISM_flag = 0;
UINT8 index_flag1 = 0;
extern INT16 test_speed;

void pwm_int(void)
{   

    INT16 temp16,spd,dir,zx_temp;
    INT16 l_theta_elec;                  //Q15,Motor Electrical Angle
	INT16 delta_p;
	INT16 m_spd_n_temp;
	INT16 spd_last_value;
	
	INT16 spd_verify;
	static INT32 m_spd_n_32 = 0;

    //save TA3 register value
	ta3n = (INT16)ta3;
	
//	temp16 = ISM;
//  da1 = motor.spd_obj>>4;
//da0 = m_spd_n >> 6;//3
//da1 = motor.spd_ref >> 4;
//da1 = motor.iq >> 6 ;
//da1 = ipm_ovl * 100;
//da1 = vol >> 3;
//da1 = sys.uzk_val >> 2;
//da1 = Uq_r_test >> 3;
//da0 = ((INT32)K14*4)*18141>>14;
//da1 = ((INT32)K14*4)*16816>>14;

//da0 = (((INT32)m_spd_n+1) >> 5)*18141>>14;
//da0 = (((INT32)abs(motor.iq)) >> 3)*3628>>14;
//da1 = ((INT32)motor.spd_ref >> 4)*16816>>14;
//da1 = ((INT32)TaccTmp + 128)*16816>>14;
//da0 = ((INT32)motor.angle - 53     )*18141>>14;
//da1 = ((INT32)motor.stop_angle )*16816>>14;

//da1 = ((INT32)m_spd_n )*16816>>14;
//da1 = (stop_status * 25) + good * 150;//

//da0 = (((INT32)motor.angle - motor.stop_angle) * 8 + 128 )*18141>>14;
//da1 = (((INT32)motor.stop_angle1 - motor.stop_angle) * 8 + 128 )*16816>>14;
//da1 = (((INT32)motor.angle - motor.stop_angle + 64) >> 3) *16816>>10;
//da1 = motor.angle >> 3;
//da1 = (alpha * 10)+128 ;
//da0 = motor.spd_ref >> 2;
//da0 = motor.iq >> 5;
//da0 =  motor.spd_obj >> 3;
//da0 = Ud_r;
//da1 = Uq_r >> 2;


	zx_temp = (INT16)(abs(m_spd_n) >> 1);
	//calculate speed
    delta_p = ta3n - ta3_buf[ta3_ind];  /* 取四个周期前的数相减 //使能了Z相，在编码器值清零前后，会不会有问题？  dhd */
    ta3_buf[ta3_ind++] = ta3n;
    ta3_ind &= SAMPLES;
	
    spd = ((INT32)delta_p * RPM_K)>>RPM_Q;
	
	spd_verify = spd; 	
	
	if(abs(spd) < 300)    //< 300   
	{
		if(ir_tb0ic)
		{
			ir_tb0ic = 0;
			if(mr3_tb0mr == 0)
			{
				spd = ((INT32)SPD_K)/((INT16)(tb0>>1));
				if(delta_p < 0)
				    spd = -spd;
			}
			else
				mr3_tb0mr = 0;
		}
		 if(abs(spd) >= 300)    
	      spd  = spd_verify ;  
	}

   if(0 == SEL_11_10)
	{
		m_spd_n_32 = m_spd_n_32 +(INT32)(spd -  m_spd_n)* B0_0;
	}
	else
	{
		m_spd_n_32 = m_spd_n_32 +(INT32)(spd -  m_spd_n)* B0_1;
	}
	m_spd_n = m_spd_n_32 >> 15; 
  
    //motor status
	switch(m_status)
    {
		case INIT:	//dummy status
         	ta3_start = ta3n;
		 	ta3_buf[ta3_ind] = ta3n;
			break;
			
	    case OPEN_START:     //start status
			m_counter++;
			m_start(ta3n);
			if(m_counter >= 50)                 
			{                                   
				m_counter = 0;                       
				int0ic = INT0_IPL;
				temp16 = ISM;
				if(l_ism ^ temp16)
				{
					if(l_ism)
					{
						int0ic = INT0_IPL;  
						ta3z_elec = ta3;
						ta3z_mach = ta3;
						motor.spd_obj = 0;
						motor.stop_angle = 0;
						motor.dir = 0;      
						FindZeroFlag = 1;    
						m_status = CLOSE_RUN;       
						
					}
					else if(temp16)
					{
						int0ic = INT0_IPL;  // | 0x10;
						ta3z_elec = ta3;
						ta3z_mach = ta3;
						motor.spd_obj = 0;
						motor.stop_angle = 0;
						motor.dir = 0;
						FindZeroFlag = 1;
						m_status = CLOSE_RUN;
					}
				}
			
				l_ism = temp16;
			}
			break;	
        case CLOSE_RUN:     //run of closed loop status
			//calculate mechanical angle
			temp16 = ta3n - ta3z_mach;
			motor.angle = (temp16&0x3ff);
			
			sum_err_p_stop = 0;			   
			//calculate electronical angle
			temp16 = ta3n - ta3z_elec;
			if(0 == SEL_11_10)
			{
				l_theta_elec = (temp16<<1) + 597 + (((INT32)m_spd_n * 42L )>> 14) + 20 + alpha;  //dhd test
			}
			else
			{
				l_theta_elec = (temp16<<1) + 587 + alpha;
			}
			l_theta_elec = l_theta_elec &0x3ff;
			l_theta_elec = (((INT32)l_theta_elec * 23040)>>14);//(0~1439)
			
			//produce SVPWM signal
            cal_forward(l_theta_elec, vol);	
			
			
			if(motor.angle <= 515 && ISM == 1 && ISM_flag == 0)  //511    511   515
			{
				if(index_flag1 == 0)
				{
					motor.angle = motor.angle + 515;//510    512    515
				}
			}
			//===================================
			motor.angle_adjusted = motor.angle - AdjustAngle; 
			
			while(motor.angle_adjusted < 0)
			{
				motor.angle_adjusted = motor.angle_adjusted + ENCODER;
			}
			//===================================
	        break;	
	    case STOP:     //stopped status
            
		    sum_err_s = 0;		//clear 
            sum_err_p = 0;
            index_flag = 1;
            sum_spd_s = 0;
			//calculate mechanical angle
			temp16 = ta3n - ta3z_mach;
			motor.angle = (temp16&0x3ff);
			//===================================
			motor.angle_adjusted = motor.angle - AdjustAngle; 
			
			while(motor.angle_adjusted < 0)
			{
				motor.angle_adjusted = motor.angle_adjusted + ENCODER;
			}
			//===================================
	        break;
			
    }

	#ifdef LED_PWM_CON
	LED_control(led_flag);
    #endif
	
	if( FL_pwm_action_flag == 1)
	{
		if( FL_pwm_period > 0 )
			FL_pwm_period--;
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
    INT16 Ta,Tb,Tc;                     //duty ratio
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
			else
			{
				U=1;U_=1;V=1;V_=1;W=1;W_=1;
	   			prcr = 0x02;
			    inv03 = 0;
	   			prcr = 0x00;
				U_=0;V_=0;W_=0;
				m_status = STOP;
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
			else
			{
				U=1;U_=1;V=1;V_=1;W=1;W_=1;
	   			prcr = 0x02;
			    inv03 = 0;
	   			prcr = 0x00;
				U_=0;V_=0;W_=0;
				m_status = STOP;
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
              
//--------------------------------------------------------------------------------------
//  Name:	cal_pid_s
//  Parameters:	None
//  Returns:	None
//  Description: calculate speed loop PID
//--------------------------------------------------------------------------------------

INT16 dif = 0;
INT16 difFed = 0;
INT16 cal_pid_s(INT16 pos_spd_ref, INT16 sp, INT16 si)
{	
    INT32 preSatOut;
    INT16 err;                          //Q15,Error
	INT32 up;
	

	#if(FACTORY_NUM == 200)
		err = m_spd_ref_n  - m_spd_n ;
		up = ((INT32)err)*((INT32)sp);
	#else
		err = m_spd_ref_n  - m_spd_n + pos_spd_ref;  
		up = ((INT32)err)*sp;
	#endif
	
	if(0 == SEL_11_10)
	{
    	sum_err_s = sum_err_s + ((((INT32)si)*up)>>16);
		if((sum_err_s) > 6000)
			sum_err_s = 6000;
		else if((sum_err_s) < -6000)	
			sum_err_s = -6000;	
	}
	else
	{
		sum_err_s = sum_err_s + ((((INT32)si)*up)>>10);
		if((sum_err_s) > 7000)
			sum_err_s = 7000;
		else if((sum_err_s) < -7000)	
			sum_err_s = -7000;
	}
		
	preSatOut = up + sum_err_s;

	if(preSatOut > PID_SPD_MAX_P)
    { 
		preSatOut = PID_SPD_MAX_P;
	
	}

	if(preSatOut < -PID_SPD_MAX_N)
	{
	   preSatOut = -PID_SPD_MAX_N;	
	
	}
    PI_out = preSatOut;
	
	return (INT16)PI_out;	  
}

//--------------------------------------------------------------------------------------
//  Name:	cal_pid_p
//  Parameters:	None
//  Returns:	None
//  Description: calculate postion loop PID
//--------------------------------------------------------------------------------------
INT16 FEDFORWARD;
INT16 forFed = 0;
INT16 m_pos_ref_last;
INT16 cal_pid_p(INT16 pp, INT16 pi)
{	
    INT16 preSatOut;
    INT16 err;                          //Q15,Error
	
	err = m_pos_ref - motor.angle;              //Q15-Q15=Q15
	
	if(err > 512)
		err -= 1024;
	else if(err < -512)
		err += 1024;
		
	limit(err,100);
	 
   preSatOut = (((INT32)err * pp)>>2) + (((INT32)sum_err_p*pi)>>4);
   limit(preSatOut,PID_POS_MAX);
    
	if((sum_err_p+err) > 400)
		sum_err_p = 400;
	else if((sum_err_p+err) < -400)	
		sum_err_p = -400;
	else
		sum_err_p += (INT32)err;                       //Q15*Q15->Q30,32bit
    return (INT16)preSatOut;                                                                      
}

INT16 cal_pid_p_stop(INT16 pp, INT16 ps, INT16 is)
{
	INT16 preSatOut;
	INT16 err_p;
	INT16 spd_err;
	
	err_p =  motor.stop_angle1- motor.angle;
	
	if(err_p > 512)           //deleted before!
		err_p -= 1024;
	else if(err_p < -512)
	err_p += 1024;
	
	spd_err = err_p * pp;
	if(0 == SEL_11_10)
	{
		if(good == 1)    
		{
			limit(spd_err, 84 );
		}
		else
		{
			limit(spd_err, 800 );
		}
	}

	spd_err = spd_err - m_spd_n;
	sum_spd_s = sum_spd_s + spd_err;
	sum_spd_s = (ps * is * sum_spd_s)>>10;
	limit(sum_spd_s, 500 );
	preSatOut = spd_err * ps + sum_spd_s;
		
    limit(preSatOut,PID_SPD_MAX_P);
	
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
	
	m_count++;
   	delta_ta3 = ta3_t - ta3_start;
	if(motor.dir)
	{
	    if(m_count >= COUNT_S)                       //10ms
	    {
	        alpha -= 5;
			if(alpha < 0)
				alpha += 1440;	
	       	motor.iq += DELTA_I;                      //1A,Q12,1365
			m_count = 0;
		}
	    switch(start_status)
	    {
	        case 0:
	            if(delta_ta3 < 0)
				{
                	start_status = 1;
				}
				else if(delta_ta3 > 0) 
				{
			        alpha -= 720;
					if(alpha < 0)
						alpha += 1440;	
			        motor.iq = motor.iq<<1;
			        start_status = 1;
					ta3_start = ta3_t;
				}
	   			break;
	        case 1:
	            if(delta_ta3 < 0)
				{
					ta3_start = ta3_t;
					start_count++;
					if(start_count > 10)
	                	start_status = 2;
				}
				else if(delta_ta3 > 0) 
				{
			        alpha -= 720;
					if(alpha < 0)
						alpha += 1440;	
		            motor.iq = motor.iq<<1;
		            start_status = 2;
				}
	            break;
	    }
	}
	else
	{
	    if(m_count >= COUNT_S)                       //10ms
	    {
	        alpha += 5;
			if(alpha >= 1440)
				alpha -= 1440;	
	       	motor.iq += DELTA_I;                      //1A,Q12,1365
			m_count = 0;
		}
	    switch(start_status)
	    {
	        case 0:
	            if(delta_ta3 > 0)
				{
                	start_status = 1;
				}
				else if(delta_ta3 < 0) 
				{
					alpha += 720;
					if(alpha >= 1440)
						alpha -= 1440;	
			        motor.iq = motor.iq<<1;
			        start_status = 1;
					ta3_start = ta3_t;
				}
	   			break;
	        case 1:
	            if(delta_ta3 > 0)
				{
					ta3_start = ta3_t;
					start_count++;
					if(start_count > 10)
	                	start_status = 2;
				}
				else if(delta_ta3 < 0) 
				{
					alpha += 720;
					if(alpha >= 1440)
						alpha -= 1440;	
		            motor.iq = motor.iq<<1;
		            start_status = 2;
				}
	            break;
	    }
	}
  
    limit(motor.iq,I_MAX_O);                 //10A,5412
	temp16 = ((INT32)motor.iq*R)>>CURRENT_Q;
#ifdef VOL_ADJUST
	//voltage adjust
	temp16 = ((INT32)temp16*DC300) / sys.uzk_val;   //Q12
#endif
	temp16 += 72;
	if(temp16 > U_MAX)
		vol = U_MAX;
	else
		vol = temp16; 
 	cal_forward(alpha, vol);
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
	static UINT16 spd_tmp_test;
	static UINT16 Dec;

	static UINT16 plus;
			
	spd_tmp_test = abs(spd_tmp);
	
	if(spd_tmp < 0)
	{
	    if(speed == STOP_SPD)
	    {
			Dec = 3; 
	        Wmax = 100;
	    }
		else if(speed <= 400 && speed != STOP_SPD)
	    {
            if(spd_tmp_test <= 400)
	        Dec = 2;
            if(spd_tmp_test > 400 && spd_tmp_test <= 700)
	        Dec = 2;
	        else if(spd_tmp_test > 700 && spd_tmp_test <= 1200)
	        Dec = 2;
	        else if(spd_tmp_test > 1200 && spd_tmp_test <= 1700)
	        Dec = 2;
	        else if(spd_tmp_test > 1700 && spd_tmp_test <= 1900)
	        Dec = 2;
			else if(spd_tmp_test > 1900 && spd_tmp_test <= 2200)
			Dec = 2;
			else
			Dec = 6;

			Wmax = 1200;
			if(0 == SEL_11_10)
			{
				if((400 == speed) && (motor.angle < 550) && (0 == stop_status))
				{
					Dec = 1;
					TaccTmp = 0;
				}
				else
				{
					Dec = 6;	
				}
			}
		
	        
	    }
		else
		{
			Wmax = 1800;
			Dec = 6;
		}
	}	
	else
	{
		  if(speed <=2000)
		     plus = 1;
		  else
		     plus = 2;
		     Wmax = 800;
	 }
	
	if(spd_tmp_test > (Wmax<<1))
		W1max = spd_tmp_test - (Wmax<<1);
	else
	{
		W1max = 0;
		Wmax = spd_tmp_test>>1;
	}
 
	
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
	
			
	if(spd_tmp >= 10 || spd_tmp <= -10)
	{
		if(speed - motor.spd_ref != 0)
		{		
		    if(speed - motor.spd_ref > 2)
			{
				if(speed - motor.spd_ref >= W1max + Wmax)
				{
					TaccTmp = TaccTmp + plus; //2
					motor.spd_ref = motor.spd_ref + TaccTmp;
				}
				else if(speed - motor.spd_ref < W1max + Wmax && speed - motor.spd_ref >= Wmax )
				{
					motor.spd_ref = motor.spd_ref + TaccTmp;
				}
				else if(speed - motor.spd_ref < Wmax)
				{
					TaccTmp = TaccTmp - 1;
					motor.spd_ref = motor.spd_ref + TaccTmp;
					if(TaccTmp <= 2)
					TaccTmp = 2;
					if(abs(speed - motor.spd_ref) <= 10)
					{
						motor.spd_ref = speed;
						spd_tmp = 0;
						TaccTmp = 0;
					}
				}
			}
			else if(speed - motor.spd_ref < -2)
			{
				if(speed - motor.spd_ref <= -(W1max + Wmax))
				{
					TaccTmp = TaccTmp - Dec;
					if(TaccTmp < -70 && 0 == SEL_11_10)//dhd
					{
						TaccTmp = -70;
					}
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
					TaccTmp = -2*Dec;
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

		if(speed - motor.spd_ref >= 200 || speed - motor.spd_ref <= -200)
		spd_tmp = speed - motor.spd_ref;
	}
		
	
    if(motor.dir)
        m_spd_ref_n = -motor.spd_ref<<1;
    else
        m_spd_ref_n = motor.spd_ref<<1;
		
		

}


//--------------------------------------------------------------------------------------
//  Name:	int0_int
//  Parameters:	None
//  Returns:	None
//  Description: INT0 interrupt function
//--------------------------------------------------------------------------------------
UINT16 pause_counter = 0;
UINT16 speed_ref = 3000;
UINT16 speed_low = 400;
UINT8 motorstart_flag = 0;
#define AUTO_MODE 1
void int0_int(void)
{
	if(motorconfig_flag == 1 && motorstart_flag == 0)    
	{                                                   
		index_flag1 = 1;                           
		ISM_flag = 1;
		ta3z_mach = ta3;
		motorstart_flag = 1;
	}
	ta3z_elec = ta3;
	
	if(RUN == sys.status || WIND == sys.status)	
	{
		if((motor.angle < 80) || (motor.angle > 996))
		{	
			ta3z_mach = ta3z_elec;					
	    	zpl_pass = 1;                // ZPL have passed
	    	if(sys.status == RUN)
	    	{  	
	    		//stitch_m_counter++; 
	    		//stitch_m_counter_all++; 	  	
	    	}	
		}
		
		else
		{
			#ifdef AUTO_MODE
			ta3z_mach = ta3z_elec;
			zpl_pass = 1; 
			#else
			sys.status = ERROR;
		    motor.stop_angle = 150 + sv_offset_angle;   // 53 degree	
	  		motor.spd_obj = 0;   	  			
		    sys.error = ERROR_39;	  //	no ZPL passed  
			#endif 	
		}
		
	}
  	index_flag = 1; 	

}

//--------------------------------------------------------------------------------------
//  Name:	motor_start
//    
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
		{
			motor.dir = 0; 
		}
		else
		{
		 	motor.dir = 1;			
		}
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
	if(motor.spd_obj == 0)
	temp16 = 1;
	else
	temp16 = 0;
	return temp16;

} 

//--------------------------------------------------------------------------------------
//  Name:	motor_control
//  Parameters:	None
//  Returns:	None
//  Description: motor control function
//--------------------------------------------------------------------------------------
UINT16 top = 255;
UINT16 bottom = 0;
UINT16 error = 1;
UINT16 mid = 0;
UINT16 Flag = 0;
UINT16 I1,I2;
void motor_control(void)
{
    INT32 temp16,dir;
    INT16 i;
    INT32 Ud_r;
    INT32 Uq_r;
	INT16 iq_r;
	INT16 temp16test;   
	UINT16 spd_tmp_test;
	INT16 temp_didt;
	UINT32 iqtemp;
	UINT16 spdtemp;
	UINT8 iqstep;
	UINT8 spdstep;
	INT16 nn,nn1;
	static INT32 iqf_32 = 0;
    top = 255;
	bottom = 0;
	
	if(motor.spd_obj - spd_last_value != 0)  
	{
		spd_tmp = motor.spd_obj - motor.spd_ref;
		if(motor.spd_obj <= 120)
		{
			spd_tmp = 120 - motor.spd_ref;
		}			
		spd_tmp_good = spd_tmp;
		spd_lasttime = spd_last_value;
		
	}	
	spd_last_value = motor.spd_obj;
	
	if(spd_tmp_good < 0 && spd_tmp == 0)
	    motor_run_status = 1;
	else if (spd_tmp > 0 )
	    motor_run_status = 0;
	

	if(spd_tmp <= -1000)     
	    choose_flag = 1; 
	else if (spd_tmp > 0)
	    choose_flag = 0;	//
    if(motor.spd_obj == 0 && spd_tmp <= -1200)
	   choose_flag = 0;	   
	
    a_m_spd += abs(m_spd_n);
	count_spd ++;
    if(count_spd >= 512)
    {
         count_spd = 0;
         motor.spd = (a_m_spd+512)>>10;
         a_m_spd = 0;
    }
	
	//--------------------------------------------------------------------------------------
	// watch main motor run status,if motor is stucked
	//--------------------------------------------------------------------------------------
	if(spd_tmp == 0 && motor.spd_obj >= 100*MINSPEED)
	{
		temp16 = m_spd_ref_n - m_spd_n;
		temp16 = abs(temp16);
		if(temp16 > (motor.spd_ref>>2))
		{
			motor_stuck_counter++;
			if(motor_stuck_counter > MOTORSTUCKTIME)
			{
				motor_stuck_flag = 1;
			}
		}
		else
		{
			motor_stuck_counter = 0;
			if(motor_stuck_flag != 0)  
			{
				motor_stuck_flag = 0;
			}
		}
	}
	
	if(m_status > OPEN_START)
	{
		if(motor.spd_obj > 0)
		{
			motor.stop_flag = 0;
			stop_status = 0;
			didt_enable = 1;
			
			good = 0; 
			
			if(m_status == CLOSE_RUN)
		    {
				/* 计算规划转速、计算iq目标值 dhd */
				if(0 == SEL_11_10)
				{
					if(m_spd_n == 0)         
					{
				        m_pos_ref_add = motor.angle<<6;
						m_pos_ref = m_pos_ref_add >> 6;
						if(motor.spd_ref > 50)
						{
							if(motor.acc_curve == 2)
								cal_spd_ref(motor.spd_obj, acc_spd_tab_const[(motor.spd_ref>>9)], motor.dec);
							else
								cal_spd_ref(motor.spd_obj, motor.acc, motor.dec);
						}
						else
							cal_spd_ref(motor.spd_obj, 2, 2);
						motor.iq = cal_pid_s(0, (KPs_L0), (KIs_L0));
					}
					else
					{
						if((motor.spd_obj != motor.spd_ref))
						{
							m_pos_ref_add += (((INT32)35791*m_spd_ref_n) >> 16);
							m_pos_ref = m_pos_ref_add >> 6;
							if(motor.spd_ref > 50)
							{
								if(motor.acc_curve)
									cal_spd_ref(motor.spd_obj, acc_spd_tab_const[(motor.spd_ref>>9)], motor.dec);
								else
									cal_spd_ref(motor.spd_obj, motor.acc, motor.dec);
							}
							else
								cal_spd_ref(motor.spd_obj, 2, 2);
							s_count = 0;
							 if (spd_tmp > 0)
							 {
					      		  if (m_spd_n < 4100) 
						   		  {
								   motor.iq = cal_pid_s(0, (KPs_up_L0), KIs_up_L0);  
						          }
						    	  else
						    	  {
							       motor.iq = cal_pid_s(0, (KPs_up_H0), KIs_up_H0);  //1 - 2
								   }
						    }
							else if (spd_tmp < 0)
							{  
								if (m_spd_n < 2100 ) //4000
							 	 {
									motor.iq = cal_pid_s(0, (KPs_down_L0), KIs_down_L0);  //KIp 
									didt_enable = 0;
						       	 }
						   		 else
						       	 {
						 			motor.iq = cal_pid_s(0, (KPs_down_H0), KIs_down_H0);  //KIp 
							      }
			 				}
						}
						else
						{
						    if(index_flag)
						    {
						        index_flag = 0;
						        m_pos_ref_add = motor.angle<<6;
							}
							else
								m_pos_ref_add += (((INT32)35791*m_spd_ref_n) >> 16);
							m_pos_ref = m_pos_ref_add >> 6;
							if(motor.spd_ref > 50)
							{
								if(motor.acc_curve)
									cal_spd_ref(motor.spd_obj, acc_spd_tab_const[(motor.spd_ref>>9)], motor.dec);
								else
									cal_spd_ref(motor.spd_obj, motor.acc, motor.dec);
							}
							else
								cal_spd_ref(motor.spd_obj, 2, 2);
							if(s_count >= 50)
							{ 
								if (m_spd_n < 2100)
								  {
								   motor.iq = cal_pid_s(0, (KPs_L0), KIs_L0);
								  }
							    else if(m_spd_n <= 4100) 
								  {
								   motor.iq = cal_pid_s(0, (KPs_M0), KIs_M0);
								  }	  
								else
								  {
								   motor.iq = cal_pid_s(0, (KPs_H0), KIs_H0);
								   }	
							}
							else
							{
								s_count++;
                             
								 if (m_spd_n < 2100) //5000
								 {
									motor.iq = cal_pid_s(0, (KPs_trans_L0), (KIs_trans_L0));  //0  reach objtect speed for 50ms
							 	 }
								else if (m_spd_n < 4100)
								 {
									motor.iq = cal_pid_s(0, (KPs_trans_M0), (KIs_trans_M0));  //4  7 reach objtect speed for 50ms	
							 	 }							
							     else
								 {													
					   		        motor.iq = cal_pid_s(0, (KPs_trans_H0), (KIs_trans_H0));    //0   reach objtect speed for 50ms 
								 }

							}
						}
					}
				}
				else
				{
					if(m_spd_n == 0)         
					{
				        m_pos_ref_add = motor.angle<<6;
						m_pos_ref = m_pos_ref_add >> 6;
						if(motor.spd_ref > 50)
						{
							if(motor.acc_curve == 2)
								cal_spd_ref(motor.spd_obj, acc_spd_tab_const[(motor.spd_ref>>9)], motor.dec);
							else
								cal_spd_ref(motor.spd_obj, motor.acc, motor.dec);
						}
						else
							cal_spd_ref(motor.spd_obj, 2, 2);
						motor.iq = cal_pid_s(0, (KPs_L), (KIs_L));
					}
					else
					{
						if((motor.spd_obj != motor.spd_ref))
						{
							m_pos_ref_add += (((INT32)35791*m_spd_ref_n) >> 16);
							m_pos_ref = m_pos_ref_add >> 6;
							if(motor.spd_ref > 50)
							{
								if(motor.acc_curve)
									cal_spd_ref(motor.spd_obj, acc_spd_tab_const[(motor.spd_ref>>9)], motor.dec);
								else
									cal_spd_ref(motor.spd_obj, motor.acc, motor.dec);
							}
							else
								cal_spd_ref(motor.spd_obj, 2, 2);
							s_count = 0;
							 if (spd_tmp > 0)
							 {
	                        if(motor.spd_ref <= 1000 && m_spd_n < 4000)       //wait for test!  20091013
							    motor.iq = cal_pid_s(cal_pid_p((KPp),0), (KPs_up_L), KIs_up_L);  
							else if(motor.spd_ref < 2000 && m_spd_n < 4000)  //wait for test!  20091013
							    motor.iq = cal_pid_s(cal_pid_p((KPp<<1),0), (KPs_up_L), KIs_up_L);  
							else
							   {
					      		  if (m_spd_n < 4000) 
						   		  {
								   motor.iq = cal_pid_s(cal_pid_p((KPp<<2),0), (KPs_up_L), KIs_up_L);  
						          }
						    	  else
						    	  {
							       motor.iq = cal_pid_s(cal_pid_p((5),0), (KPs_up_H), KIs_up_H);  //1 - 2
								   }
							    }
							 }
							else if (spd_tmp < 0)
							{  
								if (m_spd_n < 7000 ) //4000
							 	 {
									motor.iq = cal_pid_s(0, (KPs_down_L), KIs_down_L);  //KIp 
									didt_enable = 0;
						       	 }
						   		 else
						       	 {
						 			motor.iq = cal_pid_s(0, (KPs_down_H), KIs_down_H);  //KIp 
							      }
			 				}
						}
						else
						{
						    if(index_flag)
						    {
						        index_flag = 0;
						        m_pos_ref_add = motor.angle<<6;
							}
							else
								m_pos_ref_add += (((INT32)35791*m_spd_ref_n) >> 16);
							m_pos_ref = m_pos_ref_add >> 6;
							if(motor.spd_ref > 50)
							{
								if(motor.acc_curve)
									cal_spd_ref(motor.spd_obj, acc_spd_tab_const[(motor.spd_ref>>9)], motor.dec);
								else
									cal_spd_ref(motor.spd_obj, motor.acc, motor.dec);
							}
							else
								cal_spd_ref(motor.spd_obj, 2, 2);
							if(s_count >= 50)
							{ 
								if (motor.spd_ref < 1100)
								  {
								   motor.iq = cal_pid_s(0, (KPs_L), KIs_L);
								  }
							    else if(motor.spd_ref <= 2000) 
								  {
								   motor.iq = cal_pid_s(0, (KPs_L), KIs_L);
								  }
							    else if(motor.spd_ref <= 3000) //3  5   wait for test !   2009 10 13
								  {
								   motor.iq = cal_pid_s(0, (KPs_L), KIs_L);
								  }							  
								else
								  {
								   motor.iq = cal_pid_s(0, (3), 5);	//3   5
								   }	
							}
							else
							{
								s_count++;
	                         if(motor.spd_obj == 400 && spd_tmp == 0 && motor_run_status == 1)
							  {
								    motor.iq = cal_pid_s( cal_pid_p((10),0) , (5), (5)); //cal_pid_p((5),0)  //5   5
							  } 						    
							 else
							  {

								if (motor.spd_ref < 2500) 
								 {
									motor.iq = cal_pid_s(cal_pid_p((5),0), (KPs_trans_L), (KIs_trans_L)); //0  reach objtect speed for 50ms	  6 5							
							 	 }
								else if (motor.spd_ref >= 2500 && motor.spd_ref <= 3500)
								 {
									motor.iq = cal_pid_s(cal_pid_p((5),0), (7), (7));  //4  7 reach objtect speed for 50ms	  7 7							
							 	 }							
							     else
								 {													
					   		      motor.iq = cal_pid_s(cal_pid_p((5),0), (KPs_trans_H), (KIs_trans_H));   //0   reach objtect speed for 50ms   4 7  									
								 }
							 
							   }
							}
						}
					}
				}	
		    }
			else
			{
		        m_pos_ref_add = motor.angle<<6;
				m_pos_ref = m_pos_ref_add >> 6;
				U=1;U_=1;V=1;V_=1;W=1;W_=1;
	   			prcr = 0x02;
			    inv03 = 1;
	   			prcr = 0x00;
	            m_status = CLOSE_RUN;
				//da1 = 200;
			}		 
		}
	else /* 停车控制 */
		{
			didt_enable = 0;
			if(m_status == CLOSE_RUN)  
			{
				m_pos_ref_add += (((INT32)35791*m_spd_ref_n) >> 16);  
				m_pos_ref = m_pos_ref_add >> 6;

				switch(stop_status)
				{
			        case 0:

						if( motor.spd_ref <= DEC_SPD )
			        	{
							stop_status = 2;
							spd_tmp = STOP_SPD - motor.spd_ref;
				        	cal_spd_ref(STOP_SPD, motor.acc, motor.dec);
			        	}
			        	else
			        	{
			        		stop_status = 1;
							spd_tmp = DEC_SPD - motor.spd_ref;
							cal_spd_ref(DEC_SPD, motor.acc, motor.dec);
						}
					  if(0 == SEL_11_10)
					  {
					  	motor.iq = cal_pid_s(0, KPs_down_L0, KIs_down_L0); 
					  }
					  else
					  {
						 motor.iq = cal_pid_s(cal_pid_p((KPp<<2),KIp), 6, KIs_L); 
					   }
						break;
					case 1:

						if( motor.spd_ref <= DEC_SPD )
						{
							if(motor.angle >= 950)
							{
								spd_tmp = STOP_SPD - motor.spd_ref;
							    cal_spd_ref(STOP_SPD, motor.acc, motor.dec);
								if(0 == SEL_11_10)
								{
									motor.iq = cal_pid_s(0, KPs_down_L0, KIs_down_L0);
								}
							    stop_status = 2;
							}
							else
							{
								cal_spd_ref(DEC_SPD, motor.acc, motor.dec);

								if(0 == SEL_11_10)
								{
									motor.iq = cal_pid_s(0, KPs_M0,KIs_M0); 
								}
								else
								{
									motor.iq = cal_pid_s(cal_pid_p((KPp>>1),0), (KPs_down_L),0);
								}
							}
			        	}
						else
						{
							cal_spd_ref(DEC_SPD, motor.acc, motor.dec);
							if(0 == SEL_11_10)
							{						
                            	motor.iq = cal_pid_s(0, KPs_down_L0, KIs_down_L0);
							}
							else
							{
								motor.iq = cal_pid_s(cal_pid_p((KPp<<2),KIp), (KPs_L), KIs_L);
							}
						}
			        	break;	
					case 2:
						cal_spd_ref(STOP_SPD, motor.acc, motor.dec);
						
						if(motor.dir)
						{ 
							temp16 = motor.stop_angle - motor.angle;
							if(temp16 < -768)
							{
								temp16 = temp16 + 1024;
							}
						}
						else
						{
							temp16 = motor.angle - motor.stop_angle;
							if(temp16 < -768)
							{
								temp16 = temp16 + 1024;
							}
						}

						if(temp16 >= -20 && temp16 <= 20)//-30~25
					 	{ 
							 if((motor.stop_angle - motor.angle)>0)
							   	kk = 1;
							 else
							   	kk = -1;
							 if( temp16 >= -7 && temp16 <= 7 )
							 {
							 	motor.stop_angle1 = motor.stop_angle;
							 }
							 else
							 {
								 motor.stop_angle1 = motor.angle + 7*kk;
							  }

							 stop_status = 3;
						     m_spd_ref_n = 0;  
							 m_count = 0;
					 	}
						if(0 == SEL_11_10)
						{
							motor.iq = cal_pid_s(0, KPs_stop0, KIs_stop0);
						}
						else
						{
						   motor.stop_angle1 = motor.stop_angle;
						   if(spd_lasttime == 50 && motor.spd_obj == 0)
						   {

							  #if(FACTORY_NUM == 200)
							  	   motor.iq = cal_pid_s(cal_pid_p(KPp>>1,0), 10, 10); 
							  #elif(FACTORY_NUM == 210)
							       motor.iq = cal_pid_s(cal_pid_p(KPp>>1,0), 10, 10); 
							  #elif(FACTORY_NUM == 10)
							    	motor.iq = cal_pid_s(cal_pid_p(KPp>>1,0), 10, 10); 
							  #else
							  	motor.iq = cal_pid_s(cal_pid_p(KPp>>1,0), 9, 7);  
							  #endif
						   }
						   else
						   {
                              #if(FACTORY_NUM == 200)
							   motor.iq = cal_pid_s(cal_pid_p(KPp>>1,0), 30, 60); 
							  #elif(FACTORY_NUM == 210)
							   motor.iq = cal_pid_s(cal_pid_p(KPp>>1,0), 15, 25); //15 15
							  #elif(FACTORY_NUM == 10)
							    motor.iq = cal_pid_s(cal_pid_p(KPp>>1,0), 15, 25);  //15  25
							  #elif(FACTORY_NUM == 190 || FACTORY_NUM == 40 || FACTORY_NUM == 195)
							   motor.iq = cal_pid_s(cal_pid_p(KPp>>1,0), 15, 15); 
							  #else
							   motor.iq = cal_pid_s(cal_pid_p(KPp>>1,0), 5, 5); //perfect is 5 5
							  #endif
						   }
						}
						break;
					case 3:
                        m_count++;
						if(0 == SEL_11_10)
						{
							nn = motor.stop_angle-motor.angle;
							nn1 = motor.stop_angle-motor.stop_angle1;
						
							if (good==0)
							{
								if((abs(nn)>6) ||  (abs(nn1)>6))
								{
									motor.iq = cal_pid_p_stop(KPp_stop31,KPs_stop31,0);//20-11//18-4
								}
								else
								{							 	
									m_spd_ref_n = 0;
									good = 1;
									motor.iq = cal_pid_p_stop(KPp_stop32,KPs_stop32,0); ////2-17
								}
								
							}
							else
							{
								motor.iq = cal_pid_p_stop(KPp_stop32,KPs_stop32,0);//30-1-0

							}							
							if (motor.stop_angle1 != motor.stop_angle)
								motor.stop_angle1 += kk;
						}
						else
						{
							 #if(FACTORY_NUM == 200)
							 motor.iq = cal_pid_p_stop(20,30,0);   //20，25，0  /20160604--20 30 0
							 #else
						     if(m_count < 50)
								motor.iq = cal_pid_p_stop(4,25,0); 
		                     else
							    motor.iq = cal_pid_p_stop(2,10,0);
							 #endif
							good = 1;
						}

						if(m_count > 40)
						{
							motor.spd_ref = 0;	
						}
						if(m_count > 30) //if(m_count > 60)
						{
							motor.stop_flag = 1;
						}
						if(m_count > 100)
						{
							m_count = 101;
							U=1;U_=1;V=1;V_=1;W=1;W_=1;
			    			prcr = 0x02;
						    inv03 = 0;
							prcr = 0x00;		    			
							U_=0;V_=0;W_=0;
							TaccTmp = 0;   
							motor.spd_ref = 0;
							good = 0;
							m_status = STOP;
							motor.iq = 0;
							m_spd_n = 0;

						}
						break;
				}
			}
			else 
			{
		        m_pos_ref_add = motor.angle<<6;
				m_pos_ref = m_pos_ref_add >> 6;
				motor.stop_flag = 1;
				stop_status = 4;
			}
		}
		
	spd_tmp_test = abs(spd_tmp);
	iqf_32 = iqf_32 +(((INT32)motor.iq* 239>>14) -  (iqf_32 >> 15))* 4096;
	if(m_status != CLOSE_RUN)iqf_32 = 0;
	if(iqf_32 > 8355840L)
		iqf_32 = 8355840L;
	//motor.iqf = iqf_32 >> 15; 
	//motor.iqf = abs(motor.iqf);

	if(0 == SEL_11_10)
	{
    	r=R0;
 	    iqtemp = abs(motor.iq);		
		spdtemp = abs(m_spd_n);	
		etable_iq =  ((INT32)etable_iq * (32768 - 8192) + (INT32)motor.iq * 8192)>>15;
		if(etable_iq >= 0)
		{
				iqstep = iqtemp / 138;
				if(iqstep > 51)
					iqstep = 51;
				
				spdstep = spdtemp / 300;
				if(spdstep > 20)
					spdstep = 20;
				e = etablenew1[iqstep][spdstep];		
		}
		else//Iq<0
		{
				iqstep = iqtemp / 689;
				if(iqstep > 9)
					iqstep = 9;
				
				spdstep = spdtemp / 200;
				if(spdstep > 28)
					spdstep = 28;

				e = etablenew2[iqstep][spdstep];
		}
	}
	else
	{
    	r=20;
		if (abs(m_spd_n) < 600)
		 { 
			if (motor.iq>=0)  
		    {
		    	if (abs(motor.iq) < 548)	
			      e=13;
			    else if (abs(motor.iq) < 913)
			      e=14;
			    else if (abs(motor.iq) < 2372)
			      e=19;
			    else if (abs(motor.iq) < 6388)
			      e=20;
			    else if (abs(motor.iq) < 9600)
			      e=21;
			    else 
			      e=21;
			 }
		     else
			 {
				e=10;		  			  
			 }
		   }	
		
		
		else if (abs(m_spd_n) < 1200)
		  {
		     if (motor.iq>=0)  
			 {
			   	if (abs(motor.iq) < 548)	
			      	e=14;
			   	else if (abs(motor.iq) < 913)
			      	e=15;
				else if (abs(motor.iq) < 2372)
			   		e=19;
				else if (abs(motor.iq) < 9600)
			   		e=20;
				else 
			   		e=21;
			  }
			  else
			   {	
				e=10; 
			   }
			}
		 
		else if (abs(m_spd_n) < 2800)
	      {  
			 if (motor.iq>=0)  
			 {
				if (abs(motor.iq) < 548)
				   e=15;
				else if (abs(motor.iq) < 1460)
				   e=16;  
				else if (abs(motor.iq) < 6388)
				   e=22;
				else
				   e=23;
			  }
			 else
			    {
				e=10;  //9 before			
				}
		   }
	    else if (abs(m_spd_n) < 3600)
		  {
			if (motor.iq>=0) 
			{
				if (abs(motor.iq) < 1460)
				   e=16;
				else if (abs(motor.iq) < 6388)
				   e=21;
				else
				   e=22;
			 }
			else
			 {
				e=12;//9 before
			 }
			}
		else if (abs(m_spd_n) < 5600)
		  {  
			if (motor.iq>=0) 
			{
			   if (abs(motor.iq) < 913)
			     e=18;
			   else if (abs(motor.iq) < 3650)
			     e=18;
			   else if (abs(motor.iq) < 6388)
			     e=22;
			   else
			     e=22;
			 e = 16; /////
			 }
			 else
			 {
			     e=14;//10 before
			 }
		   }
		else
		   { 
			   if (motor.iq>=0) 
			   {
				if (abs(motor.iq) < 1460)
				   e=16;//18
				else if (abs(motor.iq) < 2372)
				   e=16;//19
				else if (abs(motor.iq) < 4927)
				   e=16;//20
		        else
				   e=22;
				e = 16;/////16
				}
			    else
			    {
				e=14;
				}
			}
	}

		       
			   
if(0 == SEL_11_10)
{		
	Ud_r = (((INT32)(motor.iq) * L0*m_spd_n)>>(LQ_Q+CURRENT_Q)); 
}
else
{
	Ud_r = (((INT32)(motor.iq) * L*m_spd_n)>>(LQ_Q+CURRENT_Q)); 
}

temp_didt = motor.iq-motor.iq_last;   //20100126
if(temp_didt > 5000)                  //20100126
   temp_didt = 5000;                  //20100126
else if (temp_didt < -5000)           //20100126
   temp_didt = -5000;                 //20100126

//Uq_r = ((INT32)(motor.iq) * r>>CURRENT_Q) + (((INT32)(m_spd_n) * e)>>(KE_Q))+((((INT32)(motor.iq-motor.iq_last))*L*5)>>9);
if(0 == SEL_11_10)
{
	Uq_r = ((INT32)(motor.iq) * r>>CURRENT_Q) + (((INT32)(m_spd_n) * e)>>(KE_Q)); //dhd test
}
else
{
	#if(FACTORY_NUM == 200)
	Uq_r = ((INT32)(motor.iq) * r>>CURRENT_Q) + (((INT32)(m_spd_n) * e)>>(KE_Q))+((((INT32)(temp_didt))*L*5)>>9)*didt_enable; 
	#else
	Uq_r = ((INT32)(motor.iq) * r>>CURRENT_Q) + (((INT32)(m_spd_n) * e)>>(KE_Q))+((((INT32)(temp_didt))*L*5)>>9); 
	#endif	
}
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
					       	
			   if(good == 0)  
      	
			     temp16 += 95;
			
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
		if(motor.spd_obj > 0)
		{	
			U=1;U_=1;V=1;V_=1;W=1;W_=1;
			prcr = 0x02;
			inv03 = 1;
			prcr = 0x00;
			m_status = OPEN_START;
		}
        m_pos_ref_add = motor.angle<<6;
		m_pos_ref = m_pos_ref_add >> 6;
	}
	else
	{
        m_pos_ref_add = motor.angle<<6;
		m_pos_ref = m_pos_ref_add >> 6;
	}
	

}

#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------