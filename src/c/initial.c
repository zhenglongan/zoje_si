//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : initial.c
//  Description: Core program to control UART communication and protocol
//  Version    Date     Author    Description
//  0.01     03/07/07   pbb        created
//  0.02     14/07/07   lm         modify for AMS-210E
//  ...
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"           // M16C/62P special function register definitions
#include "..\..\include\typedef.h"          // data type define
#include "..\..\include\common.h"           // External variables declaration
#include "..\..\include\variables.h"        // External variables declaration
#include "..\..\include\initial.h"          // sine table     
#include "..\..\include\stepmotor.h"        // stepmotor driver
//--------------------------------------------------------------------------------------
//  Name:		init_ta0 
//  Parameters:	None
//  Returns:	None
//  Description: initial timer A0
//--------------------------------------------------------------------------------------
void init_ta0(void)
{
  	ta0mr = 0x00;         // 0000 0000 
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
  
  	ta0 = (FX*1/1000-1);	// Set up Timer A0 Reload Register for 1 ms interval interrupt
  	ta0s = 1;  			      // start timer A0 
	ta0ic = 0;				    // set timer A0 interrupt priority
}
//--------------------------------------------------------------------------------------
//  Name:		init_tb3 
//  Parameters:	None
//  Returns:	None
//  Description: initial timer B3
//--------------------------------------------------------------------------------------
void init_tb3(void)
{
  	tb3mr = 0x00;         // 0000 0000 
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
  
  	tb3 = (FX*1/1000-1);	// Set up Timer B3 Reload Register for 1 ms interval interrupt
	tb3ic = TB3_IPL;			// set timer B3 interrupt priority
	tb3s = 0;     			  // stop timer B3 
}
//--------------------------------------------------------------------------------------
//  Name:		init_tb4 
//  Parameters:	None
//  Returns:	None
//  Description: initial timer B4
//--------------------------------------------------------------------------------------
void init_tb4(void)
{
  	tb4mr = 0x00;         // 0000 0000 
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
  
  	tb4 = (FX*1/10000-1);	// Set up Timer B4 Reload Register for 100 us interval interrupt
	tb4ic = TB4_IPL;		// set timer B4 interrupt priority
	tb4s = 0;  			    // stop timer B4 
}
//--------------------------------------------------------------------------------------
//  Name:		init_ad 
//  Parameters:	None
//  Returns:	None
//  Description: initial ADC
//--------------------------------------------------------------------------------------
void init_ad(void)
{
	INT16 i;
  	// A-D conversion initial setting 
  	adcon0 = 0x98;                      // sweep repetition mode 0 
  	adcon1 = 0x29;                      // AN0 to AN3,10 bit mode ,sweep 0 
  	adcon2 = 0x04;                      // P0 group is selected
  	for(i = 0 ; i < 24 ; i++);          // wait for more than 1us 
  		adcon0 |= 0x40;                     // A-D conversion start 
}
//--------------------------------------------------------------------------------------
//  Name:		init_da 
//  Parameters:	None
//  Returns:	None
//  Description: initial DAC
//--------------------------------------------------------------------------------------
void init_da(void)
{
	prcr = 0x04;                        // Protect mode reg    
	da1e = 1;
	da1 = 0;
	da0e = 1;
	da0 = 0;
  	prcr = 0x00;                        // Protect mode reg       
}
//--------------------------------------------------------------------------------------
//  Name:		init_clk 
//  Parameters:	None
//  Returns:	None
//  Description: initial cpu main clock
//--------------------------------------------------------------------------------------
void init_clk(void)
{
  	INT32 i;
  //24MHz (12M*2 PLL)
	prcr=0x07;                            // Protect mode reg    
	pm0=0x00;                             // Processor mode reg0 
	pm1=0x08;                             // Processor mode reg1 
	pm2=0x00;                             // Processor mode reg2 
  	cm0=0x00;                             // System clock control register 0 
  	cm1=0x20;                             // System clock control register 1 
  	cm2=0x00;                             // System clock control register 2 
  	plc0=0x11;                            // 12MHz X 2
  	plc0|=0x80;
	for(i=0;i<480000;i++)				          // delay 20ms
	{
	}
  	cm1 |= 0x02;
  	pclkr=0x03;                           // f1,Peripheral Clock Select Register 
  	prcr=0x00;                            // Protect mode reg    
}
//--------------------------------------------------------------------------------------
//  Name:		init_io 
//  Parameters:	None
//  Returns:	None
//  Description: initial IO direction    pdX=0---input  pdX=1---output
//--------------------------------------------------------------------------------------
void init_io(void)
{
	
#if FIFTH_SC013K_PLATFORM	

	#if MULTIPULE_IO_ENABLE == 1
	p0  = 0x10; 
  	pd0 = 0x10;
	#else
	p0  = 0x00; 
  	pd0 = 0x00;     // set p0.0-p0.7 to input
    #endif
	
  	p1  = 0x00;     // SNT_ON=1  SNT_H=0
  	pd1 = 0x43;     // set p1.0  p1.1 p1.6 to output   set p1.2-p1.7 to intput 
    POWER_OFF = 0;
	
  	p2  = 0xff; 
  	pd2 = 0x00;     // set p2.0-p2.7 to input
    
	#if MULTIPULE_IO_ENABLE == 1
	p3  = 0x40;   
  	pd3 = 0xF8;
	#else
  	p3  = 0x00;     // FW=0  FL_ON=0  LM_AIR=0  R_AIR=0  L_AIR=0
  	pd3 = 0xF8;     // set p3.0 p3.1 p3.2 to inupt   set p3.3-p3.7 to output
    #endif
	
  	p4  = 0x01;//0x21;     // OUTPUT_ON=1  FL=0  FA=0  T_CLK=0  T_DIR=0  T_HALF=0//1  BACKUP1=0  BACKUP2=0
  	pd4 = 0xFF;     // set p4.0-p4.7 to output 
    
  	p5_1  = 0;      // ALARM_LED=0
  	pd5_1 = 1;      // set p5.1 to output 
  	p5_2  = 1;      // PWR_LED=1
  	pd5_2 = 1;      // set p5.2 to output 
  	p5_3  = 0;      // SUM=0
  	pd5_3 = 1;      // set p5.3 to output   
	p5_4  =0 ;      //FK_OFF=0 
    pd5_4 =1 ;      // set p5.4 to output
    
    p6_0 = 0;		// RDSET = 0            //09.5.14 wr add for 485
  	pd6_0 = 1;      // set p6.0 to output   //09.5.14 wr add for 485
  	
  	p6_1  = 1;      // RST_PN=1
  	pd6_1 = 1;      // set p6.1 to output

  	p7  = 0x3c;     // V=1  \V\=1  W=1  \W\=1
  	pd7 = 0x3e;		  // set p7.7 p7.6 p7.0 to input   set p7.1-p7.5 to output
    
  	p8 = 0x43;      // U=1  \U\=1
  	pd8 = 0x43;     // set p8.0,p8.1,p8.6 to output   set p8.2-p8.7 to input
    
  	p9 = 0x40;      // DA0=0  DA1=0  SPI_CLK=0  SPI_OUT=0 
  	prcr = 0x04;    // protect disable
  	pd9 = 0x60;     // set p9.0-p9.4 and p9.7 to input   set p9.5 and p9.6 to output
  	prcr = 0x00;    // protect enable
	
	p10 = 0x03;     // SPI_CS1=1  SPI_CS2=1
	pd10 = 0x43;    // set p10.0 and p10.1 to output   set p10.2-p10.7 to input
#else
 //bit7   bit6     bit5    bit4    bit3  bit2  bit1  bit0
 //ADTCSM SENSOR8  SNT_OVL FL1_ON  FL1   U24V  DV    UZKIN    
 //IN     IN       IN      IN      OUT   IN    IN    IN 	
 	p0  = 0x00; 
  	pd0 = 0x08;     
    
 //bit7   bit6     bit5    bit4    bit3  bit2  bit1  bit0
 //DVA    EXTEND   EXTSM   ADTC    DVSM  DVB   SNT_H SNT_ON  
 //IN     OUT      IN      IN      IN    IN    OUT   OUT 		
 #if SECOND_GENERATION_PLATFORM ==1
    p1  = 0x00;     
  	pd1 = 0x40;
 #else 
  	p1  = 0x01;     
  	pd1 = 0x43;     
 #endif
	
 //bit7   bit6     bit5    bit4    bit3  bit2  bit1  bit0
 //SFSW   IORG     CSENS   CORG    PSENS PORG  YORG  XORG 
 //IN     IN       IN      IN      IN    IN    IN    IN    
  	p2  = 0x00; 
  	pd2 = 0x00;   
    
 //bit7   bit6     bit5    bit4    bit3  bit2  bit1  bit0
 //FW     FL_ON    LM_AIR  R_AIR   L_AIR T_OC  PAUSE TH_BRK
 //OUT    OUT      OUT     OUT     OUT   IN    IN    IN    	
  	p3  = 0x00;     
  	pd3 = 0xF8;     
	
 //bit7   bit6     bit5    bit4    bit3  bit2  bit1  bit0
 //FR_ON  BACKUP1  T_HALF  T_DIR   T_CLK FA    FL    OUTPUT_ON
 //OUT    OUT      OUT     OUT     OUT   OUT   OUT   OUT    	    
  	p4  = 0x01;
  	pd4 = 0xFF;

 //bit7   bit6     bit5    bit4    bit3  bit2  bit1  bit0
 //GND    ONE_WIRE EPM     FK_OFF  SUM   PWR   ALARM CE
 //IN     IN       IN      OUT     OUT   OUT   OUT   OUT    
 #if SECOND_GENERATION_PLATFORM == 1
	p5  = 0x04;
  	pd5 = 0x1F;
#else	
  	p5_1  = 0;      // ALARM_LED=0
  	pd5_1 = 1;      // set p5.1 to output 
  	p5_2  = 1;      // PWR_LED=1
  	pd5_2 = 1;      // set p5.2 to output 
  	p5_3  = 0;      // SUM=0
  	pd5_3 = 1;      // set p5.3 to output   
	p5_4  =0 ;      //FK_OFF=0 
    pd5_4 =1 ;      // set p5.4 to output
#endif
 //bit7   bit6     bit5    bit4    bit3  bit2  bit1   bit0
 //TXD1   RXD1     SCLK    BUSY    TXD0  RXD0  RST_PN RDSET
 //OUT    IN       OUT     OUT     OUT   IN    OUT    OUT    
#if SECOND_GENERATION_PLATFORM == 1 
    p6 = 0X02;	
  	pd6 = 0XBB;
#else
    p6_0 = 0;		// RDSET = 0           
  	pd6_0 = 1;      // set p6.0 to output    	
  	p6_1  = 1;      // RST_PN=1
  	pd6_1 = 1;      // set p6.1 to output
#endif	
  	
 //bit7   bit6     bit5    bit4    bit3  bit2  bit1   bit0
 //IA     IB       W_      W       V_    V     SCL    SDA
 //IN     IN       OUT     OUT     OUT   OUT   OUT    OUT   
  	p7  = 0x3c;     
  	pd7 = 0x3e;		  

 //bit7    bit6    bit5   bit4     bit3    bit2  bit1   bit0
 //AC_OVDT P_ORG   VCC    PWR_ON   BLDC_ON ISM   U_     U
 //IN      IN      IN     IN       IN      IN    OUT    OUT   
  	p8 = 0x03;     
  	pd8 = 0x03;    
	
 //bit7    bit6    bit5    bit4   bit3    bit2  bit1   bit0
 //SPI_IN  SPI_OUT SPI_CLK DA1    DA0     GND   GND    IA
 //IN      OUT     OUT     OUT    OUT     IN    IN     IN    
  	p9 = 0x40;    
  	prcr = 0x04;    
  	pd9 = 0x60;     
  	prcr = 0x00;   
	
 //bit7    bit6    bit5    bit4    bit3    bit2    bit1    bit0
 //DV2     P_TSENS SENSOR7 SENSOR6 SPISTE4 SPISTE3 SPISTE2 SPISTE1 
 //IN      IN      IN      IN      OUT     OUT     OUT     OUT  	
	p10 = 0x0f;     
	pd10 = 0x0f;   
#endif	

}
//--------------------------------------------------------------------------------------
//  Name:		init_var 
//  Parameters:	None
//  Returns:	None
//  Description: initial external variables
//--------------------------------------------------------------------------------------
void init_var(void)
{
	UINT8 i;
	//--------------------------------------------------------------------------------------
  	// system variable
  	//--------------------------------------------------------------------------------------
  	sys.status = FREE;        // free status
  	sys.error = 0;            // clear error
  	sys.uzk_val = 0;          // clear voltage
  	sys.u24_val = 0;          // clear voltage
  	//--------------------------------------------------------------------------------------
  	// motor variable
  	//--------------------------------------------------------------------------------------  
	motor.iq = 0;
	motor.max_spd = MAX_SPEED_LIMIT;     // max speed limit
	motor.min_spd = 30;       // min speed limit
	motor.acc = 30;           // 30 rpm/ms
	motor.dec = 30;           // 30 rpm/ms
	motor.acc_curve = 0;
  	motor.spd = 0;            // speed 
  	motor.angle = 0;
	motor.dir = 0;            // clockwise
  	motor.spd_obj = 0;
  	motor.spd_ref = 0;
	motor.stop_angle = 0;
	motor.stop_flag	= 1;      // motor sotp	
	wipe_time = 0;	
	StitchStartFlag = 0;	
	//--------------------------------------------------------------------------------------
  	// sewing parameter
  	//-------------------------------------------------------------------------------------- 
  	sew_speed = 800;          // sewing speed
  	sew_stitch = 0;           // sewing stitch counter
  	u15 = 0;  		            // first stitch tension
  	findorigin_flag = 1;  	  // x and y find origin flag when sewing end
  	u71 = 1;                  // thread breakage detection select
  	u51 = 1;                  // wiper enable
  	u49 = 16;                 // wind speed
  	u02 = 15;                 // 1 speed catch
  	u03 = 27;                 // 2 speed catch
  	u04 = 27;                 // 3 speed catch
  	u05 = 27;                 // 4 speed catch
  	u06 = 27;                 // 5 speed catch
  	u07 = 200;                // first stitch tension clamp thread  
  	u08 = 0;                  // cut thread tension 
  	u09 = 0;                  // cut thread tension time
  	u10 = 2;                  // 1 speed no catch
  	u11 = 6;                  // 2 speed no catch
  	u12 = 10;                 // 3 speed no catch
  	u13 = 15;                 // 4 speed no catch
  	u14 = 20;                 // 5 speed no catch
  	u16 = -5;                 // thread tension changeover timing at the time of sewing start
  	u26 = 70;                 // high of presser at 2 step
  	u33 = 2;                  // number of stitchs of thread clamp release
  	u34 = 0;                  // clamping timing of thread clamp   
  	clamp_com = 0;            // thread clamp command without
  	u36 = 0;                  // feed motion timing 
  	u37 = 0;                  // state of the presser after end of sewing 
  	u38 = 0;                  // presser lifting motion at end of sewing  
  	u41 = 0;                  // state of the presser when emergency stop 
  	u42 = 0;                  // up position or upper dead point
  	u46 = 0;                  // thread trimming disable
	u48 = 1;                  // setting the find origin style 
  	u68 = 0;                  // thread tension output time
  	u69 = 0;                  // bend position of thread clamp
  	u70 = 0;                  // thread clamp and thread clamp position
	u94 = 1;
  	u97 = 1;                  // emergency and thread trimming operation
  	u101 = 0;                 // main motor and X/Y feed synchronized control
  	u103 = 1;                 // inpresser with or without control
  	u104 = 0;                 // inpresser lowering timing
  	u105 = 0;                 // inpresser and wiper position
  	u112 = 35;                // inpresser down limit        
  	u89 = 2;                  // jog move funtion mode    
  	aging_flag = 0;           // aging flag
  	aging_delay = 20;         // aging delay time   
  	u72 = 8;                  // number of invalid stitches at start of sewing of thread breakage detection 
  	u73 = 3;                  // number of invalid stitches during sewing of thread breakage detection 
  	u35 = 0;                  // have not clamp thread motor
	k43 = 4;				  // cut speed
	u235 = 8;				  // inpresser current setting
//	u236 = 150;				  // motor stop angle setting //10.02.01 wr modify from 170 to 150
    k61 = 155;
	u224 = 0;				  // pedal style choose,
	AdjustAngle = 0;
	OutOfRange_flag = 0;
	x_bios = 0;
	y_bios = 0;
	single_flag = 0;
	laststitch_flag = 0;
	cut_flag = 0;
	stop_flag = 0;
	move_flag = 0;
	end_flag = 0;
	comx_step = 0;
	comy_step = 0;
	//--------------------------------------------------------------------------------------
  	// global variable
  	//--------------------------------------------------------------------------------------
	pedal_style = 2;
	pedal_last_state = 1; 
  	sound_count = 0;               // clear sound counter
	power_off_count = 0;           // clear power off counter
	pause_count = 0;               // clear pause counter
  	alarmled_flag = 0;             // alarm led is off
  	alarmbuz_flag = 0;             // alarm buzzer is off
  	pause_flag = 0;                // pause flag clear
  	stay_flag = 0;                 // emergency break flag clear
  	connect_flag = 0;              // panel connect flag
  	motorconfig_flag = 0;          // main motor config flag
  	stepconfig_flag = 0;           // stepping motor config flag
	InpresserIni_flag = 0;

  	inpress_position = 0;  			// inpresser position clear
  	coor_com = 0;                  // coordinate command
	cooradjust_com = 0;			   // coordinate adjust command	
	cooradjustx_step = 0;
	cooradjusty_step = 0;
  	thbrk_count = 0;               // clear thread breakage counter
  	thbrk_flag = 0;                // clear thread breakage flag
  	brkdt_flag = 0;                // clear thread breakage detection flag
  	adtc_count = 0;                // clear adtc counter
  	adtc_flag = 0;                 // clear adtc detection flag
  	sfsw_count = 0;                // clear sfsw counter
  	sfsw_flag = 0;                 // clear sfsw detection flag
	aging_com = 0;                 // aging command   
  	first_stitch_flag = 0;
   	foot_flag = 1;                 // foot up
	foot_half_flag = 1;            
	foot_com = 1;                  // foot command

	foot_count = 0;                //ou shu 
	
	inpress_flag = 0;              // foot up
	inpress_com = 0;               // foot command
	FindZeroFlag = 0;			   // find motor encoder z phase
	EncoderZ_flag = 1;
	motor_stuck_flag = 0;
	tension_start_angle = 880;

	MotorPositionSet = 0;
	machine_stop_flag = 0;

	CancelAllCoorFlag = 0;
	StitchUpFlag = 0;
	PointShiftFlag = 0;
	SewTestStitchCounter = 0;
	LastNopMoveFlag = 0;
	SewStitchCounterFlag = 0;
	SingleManualFlag = 0;
	ShapePointFlag = 0;
	SewDataFlag = 0;
	ElementPointFlag = 0;
	ElementIndex = 0;
	ElementIndexLatch = 0;
	TestStatusChangeFlag = 0;
	MoveToCurrentCoorFlag = 0;
	EditEntranceFlag = 0;
	SewingTestEndFlag = 0;
	StatusChangeLatch = FREE;
	CurrentPointShiftFlag = 0;
	CurrentPointShiftPosition = 0;
	EditElementCommand = 0;
	CancelOverlabPointFlag = 0;
	FootUpCom = 0;
	FootRotateFlag = 0;
	InpressHighRelativeFlag = 0;  //2011-3-18 songyang modify
	k110 = 1;
	k111 = 17;
	k60 = 0;
	MotiongSetFlag = 0;
	StopStatusFlag = 0;
	MotorStuckCounter = 0;
	IniFlag = 0;
	CourseBackStartFlag = 0;
	
	//210E
	cut_count = 0;                 //cut counter biao shi di ji ci jian xian 08.12.29 wr
	
  	tension = 50;             // tension
  	tension_hole = 50;        //tension ji zhun zhi 09.06.19 wr add 
  	temp_tension = 0;        //at_solenoid shi ji zhi      09.06.19 wr add
  	temp_tension_last = 0;   //at_solenoid shang yi ci zhi 09.06.19 wr add
	allx_edit_step = 0;
  	test_tension = 0;       //test tesion for ce shi mo shi 09.07.16 wr add
  	inpress_high = 0;        // inpresser high
  	inpress_high_hole = 0;   // inpresser high hole hua yang 09.06.09 wr add
  	inpress_real_delta_runing = 0;   // inpress real act delta  09.06.23 wr add
	course_thread_run_stop_flag = 0;
	
	k74 = 0;
	
	u202 = 0;
	u205 = 1;
	u208 = 0;
	
	fk_status = OUT;	//song xian dian ci tie zhuang tai :OUT 0 wei xi he;//IN 1 xi he 09.09.10 wr add
    fk_cut = 0;       //song xian dian ci tie ji shi shi jian dao; 0:wei dao;1:dao 09.09.11 wr add
    fk_count = 1;	   //song xian dian ci tie ji shu :20s hou duan dian 09.09.10 wr add
	
	k03 = 0; //2011-2-17 songyang added 0: Mechanical Type; 1: Electrical Type
	k02 = 1;    
	u102 = 0;              // step-motor move time compensation //2011-3-10 songyang add
	foot_2step_flag = 2;  //2011-4-12 songyang add; 2-step stroke: 2--highest; 1--middle; 0--lowest
	foot_2step_start_flag = 0;  //2011-4-12 songyang add;
	
	//2011-7-30
	running_overflow1 = 0;
	running_overflow2 = 0;
	
	trim_origin_pos = 0; 
	single_comm = 0x00;   //2012-4-18 add
	single_inpress_flag = 0;      //2012-4-18 add
	stretch_foot_flag = 1;  //2012-6-7
	
	cut_control_flag = 0;
	k115 = 0;
	
	find_communication_start_flag = 0; //2013-9-17 add
	ready_go_setout_com = 0;
	pattern_number = 0;
	last_pattern_number = 0;
	pattern_delay = 0;
	pattern_change_flag = 0;
	pattern_change_finish = 0; 

	PORG_action_flag = 0;
	return_origin_flag = 0;
	course_next_flag = 0;
	new_pattern_done = 0;
	first_select_flag = 0;
	marking_speed = 0;
	marking_flag = 0;
	marking_finish_flag = 0;
	x_bios_offset = 0;
	y_bios_offset = 0;
	auto_select_flag= 0;
	auto_function_flag = 0;
	
	return_from_setout = 0;
	
	opl_origin_flag = 0;
    allx_edit_step = 0;
	x_motor_dir = 0;
	y_motor_dir = 0;
    led_light_adjust = 0;
	go_original_flag = 0;
	one_step_delay = 1;
	serail_config_flag = 0;
	
	go_origin_speed = 0;
    go_setout_speed = 0;
    nop_move_speed = 0;
	pattern_change_counter = 0;
	
	speed_down_stitchs = 0;
	start_to_speed_down= 0;
	speed_down_counter = 0;
	
	k04 =  0;
	test_flag = 0;
	check_data_speed =0;
	
	xy_move_mode = 0;
	inpress_act_flag = 0; 
	HighSpeedStitchLength = 80;
	
	speed_up_counter = 0;
	release_tension_current = 0;
	nop_move_pause_flag = 0;

	
	read_step_x = 0;
	read_step_y = 0;
    nop_move_remainx = 0;
	nop_move_remainy = 0;
	
	last_pattern_point = (PATTERN_DATA *)pat_buf;
    last_allx_step = 0;
	last_ally_step = 0;
	
	sewingcontrol_flag = 0;
	sewingcontrol_stitchs =0;
	need_backward_sewing = 0;
	sewingcontrol_tail_flag = 0;
	need_action_once = 0;
	
	movexy_delay_counter = 0;
	movexy_delay_flag = 0;

	#if SUPPORT_UNIFY_DRIVER
		#if SECOND_GENERATION_PLATFORM
		z_motor_dir = 0;
		#else
		z_motor_dir = 1;
		#endif
	#else
	
		#if NEW_STRUCTURE_800_INPRESS 
			z_motor_dir = 1;
		#else
			z_motor_dir = 0;
		#endif
    #endif
	
	#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER33 
		if( z_motor_dir == 1 )
			z_motor_dir = 0;
		else
			z_motor_dir = 1;
	#endif
	DVA_scan_flag = 0;
	DVA_scan_counter = 0;
	DVA_action_done = 0;
	super_pattern_flag = 0;
	pat_buff_write_offset = 0;
	pat_buff_total_counter = 0;
	bakeup_total_counter = 0;

	

	cool_air_action_flag = 0;
    cool_air_counter = 0;
	cool_air_action_counter = 0;
	cool_air_1_sec = 0;
	
	cool_air_close_time = 10;
	cool_air_open_time = 5;
	cut_test_flag =0;
	
	check_bottom_thread_switch = 0;
	bottom_thread_remain = 0; 
	set_default_length =0 ;
	open_or_close_loop = 0;  //1-open 0-close
	alarm_thread_length = 200;
	bottom_thread_alarm_type = 0;
	thread_break_detect_level = 0;
	
	stepmotor_need_double_time = 0;
	movestep_x_flag = 0;
	movestep_y_flag = 0;
	movestepx_delay_counter = 0;
	movestepy_delay_counter = 0;
	
	second_start_switch = 0;
	second_start_counter = 1;
	power_on_ready = 0;
	return_from_preddit = 0;


	need_action_two = 0;
	
	origin_com_after_pattern_done = 0;
	alarm_output_enable = 0;
	cutter_output_test = 0;
	
	enable_stop_in_nopmove = 1;
	special_go_allmotor_flag = 0;
	already_in_origin = 0;
	enable_thread_broken_alarm = 0;
	finish_nopmove_pause_flag =0;
	
	target_pat_point = 0;
    target_allx_step = 0;
	target_ally_step = 0;
	bakeup_total_counter = 0;
	
	inpress_port = 1; //0-1,2,4  1-1,2,3
	ct_holding_steps = -26;
	baseline_alarm_flag = 0;
    baseline_alarm_stitchs =0;
	
	release_poweron_ready = 0;
	
	id_alarm_flag = 0;
    id_alarm_counter = 0;
	
	k05 = 0;
	k21 = 0;
	k22 = 0;
	k23 = 0;
	k24 = 0;
	k25 = 0;
	k26 = 0;
	k27 = 0;
	k28 = 0;
	x_sensor_pos = 0;//left
	id_pattern = 0;
	
	pattern_id_delay = 0;
	front2stitchs_tension_off = 1;
	
	inpress_type = MOTOR_INPRESS;
	//inpress_type = AIR_INPRESS;
	y_compensation = 0;
	x_compensation = 0;
    last_direction = 0;
	last_x_direction = 0;
	y_gear_compensation = 0;
	x_gear_compensation = 0;
	Corner_deceleration_speed = 10;
	
	release_tension_time = 10;
	release_tension_before_nopmove = 1;
	da0_release_flag = 0; 
	da0_release_conter = 0;
	
	blow_air_counter = 0;
	blow_air_flag = 0;
	
	identify_mode = 0;
	auto_origin_mode = 0;
	y_curver_number = 1;
	
	milling_cutter_action_flag = 0;
	milling_first_move = 0;
	milling_cutter_stop_flag = 0;
	rotated_cutter_position = 0;
	
	stepversion3 = 0;
	stepversion4 = 0;
	sum_rotated_angle = 0;
	waitting_for_pattern_done = 0;
	
	bobbin_case_switch_counter = 0;
	bobbin_case_switch_flag = 0;
	
	bobbin_case_arm_offset = -2;
	bobbin_case_platform_offset = 0;
	bobbin_case_arm_position = 0;
	bobbin_case_platform_position = 0;
	bobbin_case_dump_position = 190;
	bobbin_case_workmode = 0;
	already_auto_find_start_point = 0;
	bobbin_case_enable = 0;
	rotated_cutter_enable = 0;
	rotated_cutter_running_delay = 0;
	rotated_cutter_up_delay = 3000;
    rotated_cutter_speed = 3;
	rotated_cutter_current_level = 4;
	bobbin_case_current_level = 5;
    bobbin_case_inout_delay = 2000;
	bobbin_case_scrath_delay = 500;
	
	checki11_item = 0;
	checki11_test_flag= 0;
	checki11_action = 0;
	for( i=0; i<5; i++)
	     checki11_output_status[i] = 0;
	bobbin_case_once_done_flag = 0;
	bobbin_case_stop_position = 0;
	bobbin_case_alarm_mode = 0;
	bobbin_case_restart_mode = 0;
	
	synchronization_flag = 0;
	k169 = 0;
	ZJ_8060 = 0;
	X_AXIS_ORGIN_ENABLE = 1;       //X轴找原点
	AUTO_SEARCH_START_POINT = 0;
	SEND_SERVO_PARA = 0;           //下发调试参数
	ROTATED_CUTTER = 0;            //旋转切刀
	#if (CURRENT_MACHINE == MACHINE_BOBBIN_CUTTER) ||(CURRENT_MACHINE == MACHINE_900_BOBBIN)
		SUPPORT_CS3_FUN = 1;           //支持扩展SC074A
	#else
		SUPPORT_CS3_FUN = 0;
	#endif
	ENABLE_BOBBIN_CASE_FUN = 0;    //换梭功能
	nop_move_control = 0;

	start_sewcontrol_mode = 0;     
	
	step_movetype = 0;
	DelayCounter = 0;
	DRV_DATA_LEN=0;
	drv_satus = 0;
	svpara_trans_flag = 0;
	
	cutter_function_flag = 0;
	cutter_rotated_abs_angle = 0;
	drill_motor_run_enable = 0;
	drill_motor_pwm = 750;
	drill_motor_counter = 0;

	
	check_valve4_flag = 0;
	check_valve5_flag = 0;
	cutter_motoer_remain_angle = 0;
	last_rotated_dir = 0;
	dsp3_input_value = 0;
	
//#if FOLLOW_INPRESS_FUN_ENABLE
	inpress_follow_down_angle = 54;    //54d

	inpress_follow_up_angle = 270;     //270d

	inpress_follow_down_speed = 8;
	inpress_follow_up_speed = 8;
    inpress_follow_range = 40;
	movezx_delay_flag = 0;
	movezx_delay_counter = 0;
//#endif
    indraft_control_counter = 0;
	indraft_control_flag = 0;
	cutter_syn_counter = 0;
	
	ct_bump_workingtime = 0;
    last1_speed = 18;
	last2_speed = 12;
	last3_speed = 8;
	last4_speed = 4;;
	cutter_syn_delay =0;
	ct_bump_action_flag = 0;
	ct_bump_workingtime =0;
	cutter_motor_initial_angle = 0;
	
	clamp_stepflag = 0;

	stepper_cutter_enable = 0;
	stepper_cutter_position = 0;
    stepper_cutter_shake_rage =20;
	
	debug_dsp_flag = 0;
	serail_number = 0;
	pattern_chage_recover_counter = 0;
	stepper_cutter_delay = 0;
	x_origin_offset = 0;
	
	x_origin_offset_effect = 0;
	confirm_barcode = 10000;
	special_pause_flag = 0;
	
	cutter_test_cnt = 0;
	stepper_cutter_origin_offset = 0;
	
	dsp3_moto1_direction = 0;
	counter_1ms = 0;
	
	inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
	
	debug_para1 = 0;
	debug_para2 = 0;
	
	release_tension_value_flag =0;
	find_x_origin_counter = 0;
	
	autosewing_control_flag = 0;
	autosewing_allset_flag = 0;
	autosewing_allow_working = 0;
	autosewing_switch_last_status = 0;
	
	inpress_follow_down_angle_pc =0;
	inpress_follow_up_angle_pc = 0;
	inpress_follow_down_speed_pc =0 ;
	inpress_follow_up_speed_pc = 0;
    inpress_follow_range_pc =0;
	

	bobbin_plateform_org_offset = 0;
	check_valve1_flag = 0;
	check_valve2_flag = 0;
	check_valve3_flag = 0;
	check_valve6_flag = 0;
	
	second_point_passby_flag = 0;
	DEADPOINT = 10;
	already_find_startpoint_flag = 0;
	
	laser_fun_delay_off_flag = 0;
    laser_fun_delay_counter = 0;
	
	monitor_pat_point = (PATTERN_DATA *)(pat_buf);
	monitor_allx_step = 0;
	monitor_ally_step = 0; 

	
	over_counter = 0;
	cutter_speed_done_flag = 0;
	
	laser_power_on_flag = 0;
	laser_power_error_flag = 0;
    laser_power_on_counter = 0;
	
	monitor_refresh_flag = 0;
	
	double_xy_time_flag = 0;
	drill_motor_updown_flag = 0;
	
	debug_read_point = 0;
	debug_write_point = 0;
	debug_counter = 0;
	
	step_curve_write_counter = 0;
	pause_active_level = 0;
	
	inflection_poin_flag = 0;
	rec1_total_counter = 0;
	laser_cutter_aciton_flag = 0;
	
	wirte_stepermotor_para_flag = 0;
	write_eeprom_para_flag = 0;
	write_stepmotor_curve_flag = 0;
	
	crc_value = 0;
    readback_crc_value = 0;
	
	enable_footer_up = 0;
	bar_coder_refresh_flag = 0;
	oil_empty_alarm_enable = 0;
	bar_coder_refresh_enable = 0;
	laser_power_on_enable = 0;
	laser_offset_x = 0;
	laser_offset_y = 0;
	for(i=0;i<5;i++)
		laser_test_flag[i] = 0;
	K227 = 0;//二代主轴
	sv_offset_angle = 0;
	m_spd_n_last = 0;
	
	cutter_protect_flag = 0;
    cutter_protect_counter = 0;
	
	FL_pwm_action_flag = 0;
	FL_pwm_period = 0;
	FL_pwm_counter = 0;
	
	power_on_ask_for_framework = 0;

	
	power_on_first_find_x = 0;
	autosewing_offset_moveing_done = 0;
	
	
#if CHANGE_DOUBLE_FRAMEWORK 
	left_start_action_flag = 0;
	left_start_counter = 0;
	left_start_lock_flag = 0;
	right_start_action_flag = 0;
	right_start_counter = 0;
	right_start_lock_flag = 0;
	left_quest_running = 0;
	right_quest_running = 0;
	current_running_flag = 0;
	para_x_take_offset_left =100;
    para_x_take_offset_right =100;
	para_y_backward_dis = 100;
	para_left_barcode_position = 200;
	para_right_barcode_position = 200;
	para_catch_delay_time = 200;
#else	
	para_catch_delay_time =0;
	para_y_backward_dis = 0;
#endif

	#if ENABLE_RFID_FUNCTION
	rc522_write_falg = 0;
	rc522_write_ret_falg = 1;
	rc522_control_falg = 0;
	rfid_config_flag = 0;
	ms_scan_counter = 0;
	#endif

	com_refresh_flag = 0;
	delay_before_inpress_down = 0;
	
	inpress_not_working_stitchs = 0;
	inpress_follow_mode = 0;
	
	frame_start_counter = 0;
	frame_start_flag = 0;
	pen_x_bios_offset = 0;
	pen_y_bios_offset = 0;
	
	monitor_x_step = 0;
	monitor_y_step = 0;
	monitor_x_time = 0;
	monitor_y_time = 0;
	monitor_point = 0;
	chang_parameter_flag = 0;
	powr_on_zx_flag = 0;
	
	inpress_litter_footer_action_flag = 0;
	inpress_litter_footer_action_value = 0;
    inpress_litter_footer_action_counter = 0;
	
	#if ENABLE_CONFIG_PARA
	yj_motor_dir = para.yj_org_direction;
	#else
	yj_motor_dir = 0;
	#endif
	first_start_flag = 0;

	
	special_encoder_mode = 0;
	crc1 = 0;
	crc2 = 0;
	crc3 = 0;
	for( i=0;i<32;i++)
		spi_out_status[i]=0;
	laser_emergency_stop = 0;
	emergency_restart_flag = 0;
	valve_3_open_counter = 0;
	find_laser_start_stop_flag = 0;
	laser_already_begin_flag = 0;
	
	rotated_cutter_running_flag =  0;
	rotated_cutter_running_counter = 0;
	power_on_laser_cool_flag = 0;
	para_x_take_offset_left = 0;
	para_right_barcode_position = 0;
	main_control_lock_setup = 0;
	making_pen_actoin_flag = 0;
	making_pen_status = 0;
	rfid_debug_flag = 0;
	uart1_sending_flag = 0;
	
	cutter_angle_adjust = 0;
	monitor_cutter_angle = 0;
	checki10_follow_flag = 0;
	checki10_action0 = 0;
	checki10_action1 = 0;
	one_step_run_flag = 0;
}			
//--------------------------------------------------------------------------------------
//  Name:		initial 
//  Parameters:	None
//  Returns:	None
//  Description: system initialization 
//--------------------------------------------------------------------------------------
void initial(void)
{
	INT16  count;
	//--------------------------------------------------------------------------------------
  	// disable interrupts
  	//--------------------------------------------------------------------------------------
	asm("fclr I");   
	//--------------------------------------------------------------------------------------
  	// call intial cpu clock function
  	//--------------------------------------------------------------------------------------
	init_clk();	

#if ENABLE_CONFIG_PARA == 1 || INSTALLMENT == 1	
	init_eeprom();	
#endif
	
#if ENABLE_CONFIG_PARA == 1	
	for(count =0 ;count< 30000;count++);
	init_para_variables();
#endif	
	//--------------------------------------------------------------------------------------
  	// call initial IO direction fucntion
  	//--------------------------------------------------------------------------------------
  	init_io();	
	//--------------------------------------------------------------------------------------
  	// call initial timer A0 function
  	//--------------------------------------------------------------------------------------
  	init_ta0();
  	//--------------------------------------------------------------------------------------
  	// call initial timer B3 function
  	//--------------------------------------------------------------------------------------
  	init_tb3();
  	//--------------------------------------------------------------------------------------
  	// call initial timer B4 function
  	//--------------------------------------------------------------------------------------
  	init_tb4();
  	//--------------------------------------------------------------------------------------
  	// call initial clamp thread function
  	//--------------------------------------------------------------------------------------
  	init_ct();
  	//--------------------------------------------------------------------------------------
  	// call initial ADC function
  	//--------------------------------------------------------------------------------------
  	init_ad();
  	//--------------------------------------------------------------------------------------
  	// call initial DAC function
  	//--------------------------------------------------------------------------------------
  	init_da();
  	//--------------------------------------------------------------------------------------
  	// call initial communication function
  	//--------------------------------------------------------------------------------------
	init_comm();	
	//--------------------------------------------------------------------------------------
  	// call initial stepmotor function
  	//--------------------------------------------------------------------------------------
	init_stepmotor_drv();	
	//--------------------------------------------------------------------------------------
  	// call initial external variables function
  	//--------------------------------------------------------------------------------------
  	init_var();
	//--------------------------------------------------------------------------------------
  	// delay 1000ms  wait power on
  	//--------------------------------------------------------------------------------------
	count = 0;
	while(1)
	{
    	if(ir_ta0ic)        //test 1ms timer
    	{
			ir_ta0ic = 0;
		  	count++;
			if(count >= 1000)
			break;
		}
		#if ( (USE_SC011N_PLATFORM == 0) && (FIFTH_SC013K_PLATFORM == 0) )
    	if((PWR_ON == 1) && (BLDC_ON == 0))   // if 120V < AC input < 290V and  BLDC ON 
	  		break;
		#endif
  	}
	//--------------------------------------------------------------------------------------
  	// initial watch and motor
  	//--------------------------------------------------------------------------------------
  #if SECOND_GENERATION_PLATFORM == 1 || USE_SC011N_PLATFORM ==1 || FIFTH_SC013K_PLATFORM == 1
  
  #else
 	if((PWR_ON == 0) && (AC_OVDT == 1))     // if 300V is overvoltage 
		sys.error = ERROR_05;
	else
  #endif	
	{
		//call initial system watch module function     function is in the watch.c
	  	init_watch();
	
	  	//set interrupt priority
		int1ic = INT1_IPL | 0x10; 			// rising  edge select           
	  	int2ic = INT2_IPL;              	// falling edge select 
	
	  	// enable interrupts
		asm("fset I");  					

		//set motor PWM port output intial value
		U=1;U_=1;V=1;V_=1;W=1;W_=1;
		prcr = 0x02;
		inv03 = 0;
		prcr = 0x00;
	
		//output enable  
		OUTPUT_ON = 0;

		//delay 10ms
		count = 0;
		while(1)
		{
	    	if(ir_ta0ic)        //test 1ms timer
	    	{
				ir_ta0ic = 0;
			  	count++;
				if(count >= 10)
			  		break;
			}
		}

		//call system watch of initialize function 
		if(sys.error == 0)
			sys.error = watch_initialization();	 
	
		//disable interrupts
		asm("fclr I");   

		//call initial motor control module function
		init_motor();
		//set interrupt priority
		tb2ic = TB2_IPL;			      // falling edge select 
	  	int0ic = INT0_IPL;          // falling edge select 
		s0tic = UART_TRANSMIT_IPL;	// falling edge select 
		s0ric = UART_RECEIVE_IPL;	  // falling edge select 
		s1tic = UART1_TRANSMIT_IPL_7;  // UART0 TX: TXR transmit
        s1ric = UART1_RECEIVE_IPL_7;   // UART0 RX: TXR receive 

	}
	//--------------------------------------------------------------------------------------
  	// set interrupt priority
  	//--------------------------------------------------------------------------------------
	//	ta0ic = TA0_IPL;		// set ta0 interrupt priority to 3 
	PWR_LED = 0;	      	// enable power led
  	ALARM_LED = 0;	    	// disable alarm led
	//--------------------------------------------------------------------------------------
  	// enable interrupts
  	//-------------------------------------------------------------------------------------- 
  	asm("fset I"); 

}


//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
