//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : variables.c
//  Description: external variables define
//  Version    Date     Author    Description
//  0.01     03/07/07   pbb        created
//  ...
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\typedef.h"      //Data type define
#include "..\..\include\common.h"       //External variables declaration
//--------------------------------------------------------------------------------------
//  External variables define
//--------------------------------------------------------------------------------------
SYS_STRU sys; 		         // systerm variable structure
MOTOR_STRU motor;          // motor variable structure
PATTERN_DATA *pat_point;   // pattern point variable structure
PATTERN_DATA *sta_point;   // pattern point variable structure

PATTERN_DATA PatStart = {0xC0,0x00,0x00};
PATTERN_DATA PatEnd   = {0xC0,0x01,0x00};

STEPVERSION Step1Version = {3,0,1,0,1};
STEPVERSION Step2Version = {3,0,1,0,1};
UINT16 MotorStuckCounter;
UINT8 IniFlag;
UINT8 CourseBackStartFlag;

UINT8 flag_1ms;            // 1ms flag
UINT16 ms_counter;         // 1ms delay counter
UINT16 wipe_time;
UINT8 inpress_time;
UINT16 us_counter;         // 100us delay counter

#if LITTLE_RAM_CPU
	UINT8 pat_buf[12003]; 	
#else
	UINT8 pat_buf[24020];      //25000 
#endif


UINT8 *recpat_point;       // receive pattern point
UINT8 pedal_state;
UINT8 pedal_last_state;
UINT16 cut_start_angle;
UINT16 cut_end_angle;
UINT16 tension_start_angle;
UINT16 tension_end_angle;
UINT8 wiper_start_time;
UINT8 k52;
UINT8 pedal_style;
UINT8 foot_flag;           // foot flag  0=down  1=up
UINT8 foot_half_flag;
UINT8 inpress_flag;        // inpress flag  0=down  1=up
UINT8 clamp_flag;          // clamp out flag   0=in  1=out
UINT8 OutOfRange_flag;
UINT8 DVSMLastState;

UINT8 zpl_pass;            // ZPL pass flag
UINT8 stitch_counter_tmp;

UINT8 origin_com;          // all motor find origin command
UINT8 commandpoint_com;    // go to the commamd point in edit status
UINT8 wind_com;            // wind command
UINT8 repeat_com;          // repeat command

UINT8 foot_com;            // foot command    0=down  1=up  2=no move

UINT8 inpress_com;         // inpress command 0=down  1=up  2=no move
UINT8 coor_com;            // coordinate command    
UINT8 cooradjust_com;      // coordinate compensation command 

UINT16 stitch_num;         // sewing stitch number
UINT16 stitch_counter;     // sewing stitch counter


INT32 allx_step,ally_step; // all step counter
INT16 allx_edit_step,ally_edit_step;
INT16 xstep_cou,ystep_cou; // x and y move step counter
INT32 sox_step,soy_step;   // start point step counter
INT16 manx_step,many_step; // manual move step counter
INT32 comx_step,comy_step; // command move step counter
INT16 stacoorx_step,stacoory_step; // next pattern start point coordinate *20(step) 09.2.9 wr add
INT16 origin2x_step,origin2y_step; // next pattern origin2 point *20(step) 09.2.13 wr add
INT16 currentpyx_step,currentpyy_step; // current pattern pian yi *20(step) 09.2.13 wr add
INT16 comx_polar_step,comy_polar_step; // command move step counter
INT16 cooradjustx_step,cooradjusty_step;   // x and y shift compensation 
INT16 cooradjustxtotal_step,cooradjustytotal_step;
INT16 coordisxtotal_step,coordisytotal_step;   // for display
INT16 xcurrent_coordinates,ycurrent_coordinates;

INT32 allx_step_temp,ally_step_temp;
UINT8 predit_shift = 1;
UINT8 curpat_have_origin2;
UINT32 DelayCounter = 0;
UINT32 DelayTime = 0;


UINT8 MotorPositionSet;
UINT8 InpressHighControlFlag;
UINT8 InpressAbsoluteHigh;
UINT8 OutputFlag;
UINT8 OutputNumber;
UINT8 InputFlag;
UINT8 InputNumber;
UINT8 PatternDelayFlag;
UINT16 PattenrDelayTime;
UINT8 RotateFlag;
UINT8 Tension3thFlag;
UINT8 ElectronicClampFlag;
UINT8 ElectronicClampValue;
UINT8 AreaDivisionFlag;
UINT8 KnifeDriveFlag;
UINT8 KnifeStyle;
UINT8 SewingSpeedFlag;
UINT8 SewingSpeedValue;
UINT8 NopmoveFuncFlag;
UINT8 NopmoveSpeed;
UINT8 InpressHighRelativeFlag;
UINT8 InpressHighRelativeValue;
UINT8 SewingStopFlag;
UINT8 SewingStopValue;
UINT8 MotorSpeedRigister;
INT16 CancelAllXCoor;
INT16 CancelAllYCoor;
UINT8 CancelAllCoorFlag;
UINT8 StitchUpFlag;
UINT8 PointShiftFlag;
UINT8 PointShiftDirection;
UINT16 PointShiftNumber;
UINT8 EndStyleChooseFlag;
UINT16 NopMoveElement;
UINT16 SewingElement;
UINT16 ElementNumber;
UINT16 SewTestStitchCounter;
UINT16 PatternShiftValue;
UINT8 LastNopMoveFlag;
UINT8 SewStitchCounterFlag;
UINT8 SingleManualFlag;
UINT8 ShapePointFlag;
UINT8 SewDataFlag;
UINT8 ElementPointFlag;
UINT8 ElementIndex;
UINT8 ElementIndexLatch;
UINT8 TestStatusChangeFlag;
UINT8 MoveToCurrentCoorFlag;
INT16 CurrentXCoor;
INT16 CurrentYCoor;
UINT8 EditEntranceFlag;
UINT8 SewingTestEndFlag;
INT16 StepDrive1Version;
INT16 StepDrive2Version;
UINT8 StatusChangeLatch;
UINT8 CurrentPointShiftFlag;
INT16 CurrentPointShiftPosition;
UINT8 EditElementCommand;
UINT8 CancelOverlabPointFlag;
UINT8 FootUpCom;
UINT8 DVBLastState;
UINT8 DVALastState;
UINT8 FootRotateFlag;




INT16 DestinationPointShiftPosition;
UINT8 MotionSet;
UINT8 MotiongSetFlag;
UINT8 StopStatusFlag;







UINT8 move_flag;           // move stepper motor flag
UINT8 movex_flag;		   // move x flag in pattern process
UINT8 movey_flag;		   // move y flag in pattern process
UINT8 movestep_x_flag;	   // move x step motor flag
UINT8 movestep_y_flag;     // move y step motor flag
UINT8 lastmove_flag;       // move stepper motor flag
UINT8 origin2_lastmove_flag;
UINT8 NopMoveSpd_flag;
UINT8 nopmove_flag;        // nop move stepper motor flag
UINT8 cut_flag;            // cut thread flag
UINT8 laststitch_flag;	   // the last stitch flag in line/circle sewing without cut code
UINT8 cut_start_flag;
UINT8 cut_end_flag;
UINT8 wipe_start_flag;
UINT8 wipe_end_flag;
UINT8 inpress_start_flag;
UINT8 inpress_end_flag;
UINT8 tension_start_flag;
UINT8 tension_end_flag;
UINT8 wipe_start_time;
UINT8 wipe_end_time;
UINT8 stop_flag;           // stop flag
UINT8 stop_number;
UINT8 end_flag;            // pattern end flag
UINT8 machine_stop_flag;

UINT8 first_stitch_flag;   // the first stitch in the pattern
UINT8 slow_flag;           // slow flag
UINT8 check_flag;          // check data flag
UINT8 process_flag;        // process data flag
UINT8 calculate_flag;      // calculat angle flag
UINT8 start_flag;          // pattern start flag
UINT8 connect_flag;        // panel connect flag
UINT8 motorconfig_flag;    // main motor config flag
UINT8 stepconfig_flag;     // stepping motor config flag
UINT8 InpresserIni_flag = 0;
UINT16 DelayMsCount;
UINT8 StitchStartFlag;

INT16 movestep_angle;      // move step angle



INT16 movestepx_angle;      // move x step angle
INT16 movestepy_angle;      // move y step angle
INT16 movect_angle;        // move ct angle
INT16 last_speed;          // sewing speed

INT16 allyj_step;          // all yj step counter
INT16 allin_step;          // all in step counter
INT16 allct_step;          // all ct step counter


UINT8 clamp_stepflag;      // clamp step flag
UINT8 tb1_flag;            // timer b1 flag

 
UINT8 status_now;
UINT8 status_15;
UINT8 FindZeroFlag;
UINT8 EncoderZ_flag;
UINT8 motor_stuck_flag;
//--------------------------------------------------------------------------------------
// sewing parameter
//-------------------------------------------------------------------------------------- 
INT16 sew_speed;           // sewing speed                               
UINT8 u15;                 // first stitch tension                      
UINT16 sew_stitch;         // sewing stitch counter                    
UINT8 findorigin_flag;     // x and y find origin flag when sewing end
UINT8 u71;                 // thread breakage detection select                
                          
UINT8 u49;                 // wind speed                              
UINT8 u02;                 // 1 speed catch                           
UINT8 u03;                 // 2 speed catch                           
UINT8 u04;                 // 3 speed catch                           
UINT8 u05;                 // 4 speed catch                           
UINT8 u06;                 // 5 speed catch                           
UINT8 u07;                 // first stitch tension clamp thread                         
UINT8 u08;                 // cut thread tension                      
INT8 u09;                 // cut thread tension time                 
UINT8 u10;                 // 1 speed no catch                        
UINT8 u11;                 // 2 speed no catch                        
UINT8 u12;                 // 3 speed no catch                        
UINT8 u13;                 // 4 speed no catch                        
UINT8 u14;                 // 5 speed no catch  
UINT8 u201;                // go original when enenergy  
UINT8 u202;  			   // sewing when pedal is up
UINT8 u203;				   // go original forbidden when pedal is up
UINT8 u204;				   // go original forbidden when taken-up is down
UINT8 u205;				   // wind switch
UINT8 u51;				   // wiper switch
UINT8 u207;				   // taken-up position when pause buttern is on
UINT8 u208;				   // pedal action when pause buttern is on
UINT8 u209;				   // puase switch style
INT16 k43; 			       // cut speed
UINT8 k01;				   // cancel sewing sera potection	
INT32 k56;				   // x left limit
INT32 k57;				   // x right limit	
INT32 k58;				   // y up limit
INT32 k59;                 // y down limit
UINT8 u217;				   // set the high speed
UINT8 u218;				   // set the low speed
UINT8 u219;				   // set the mid-high speed
UINT8 u220;				   // set the mid-low speed	
UINT8 u221;				   // single pedal
UINT8 u222;				   // trim on delay 
UINT8 u223;				   // tension start delay
UINT8 u224; 			   // presser style
UINT8 u225;				   // presser wright
UINT8 u226;			       // light presser
UINT8 u227;				   // mid presser
UINT8 u228;				   // heavy presser
UINT8 u229;				   // sewing material style
UINT8 u230;				   // thin material
UINT8 u231;				   // mid material
UINT8 u232;				   // thick material
UINT8 u233;				   // check origin style when aging
UINT8 u234;				   // pedal action times when aging	
UINT8 u235;				   // inpresser current setting 
UINT8 u236;				   // stop angle setting   
UINT8 k02;				      // sewing machine type
UINT8 k110;				   // rotate device control
UINT16 k111;				   // origin check with rotate device
UINT8 k92;				   // origin check without rotate device	
UINT8 k93;				   // Y axis minimum distance 
UINT8 k60;				   // two or three pedal 
INT16 AdjustAngleSet;      
INT8 u16;                  // thread tension changeover timing at the time of sewing start           
UINT8 u26;                 // high of presser at 2 step
UINT8 u33;                 // number of stitchs of thread clamp release
INT8 u34;                 // clamping timing of thread clamp                                                    
UINT8 clamp_com;           // thread clamp command                    
INT8 u36;                 // feed motion timing 
UINT8 u37;                 // state of the presser after end of sewing                  
UINT8 u38;                 // presser lifting motion at end of sewing  
UINT8 u39;				   // check origin when sewing end(normal)	
UINT8 u40;                 // check origin when sewing end(c pattern)                                  
UINT8 u41;                 // state of the presser when emergency stop
UINT8 u42;                 // up position or upper dead point  
UINT8 u46;                 // thread trimming disable 
UINT8 u48;				   // setting the find origin style        
UINT8 u68;                 // thread tension output time
// UINT8 u69;                 // bend position of thread clamp
INT8 u69;
UINT8 u70;                 // thread clamp and thread clamp position
UINT8 u94;				   // find dead center when find origin or reset 
UINT8 u97;                 // emergency and thread trimming operation
UINT8 u101;                // main motor and X/Y feed synchronized control 
UINT8 u103;                // inpresser with or without control
UINT8 u104;                // inpresser lowering timing
UINT8 u105;                // inpresser and wiper position
UINT8 u112;                // inpresser down limit                
UINT8 u89;                 // jog move funtion mode    
UINT8 aging_flag;          // aging flag 
UINT8 aging_delay;         // aging delay time 
UINT8 u72;                 // number of invalid stitches at start of sewing of thread breakage detection 
UINT8 u73;                 // number of invalid stitches during sewing of thread breakage detection 
UINT8 u35;                 // have clamp thread motor
INT16 x_bios;
INT16 y_bios;
INT16 DisMotorAngle;	   // motor mechanical angle for display
INT16 AdjustAngle;		   // motor set adjust angle

//--------------------------------------------------------------------------------------
// 210E add 10.02.01 wr add
//--------------------------------------------------------------------------------------
UINT8 u81;
UINT8 u82;
UINT8 u84;
UINT8 u85;
UINT8 u86;
UINT8 u87;
UINT8 u91;
UINT8 u108;
UINT8 u129;
UINT8 u245;
UINT8 k31;
UINT8 k53;
UINT8 k54;
UINT8 k60; //2011-2-28
UINT16 k61;
UINT8 k63;
UINT8 k67;
UINT8 k74;
UINT8 k75;
INT8 k95;
UINT8 k98;
UINT8 k100;
UINT8 k131;
UINT8 k150;
             
			 
UINT8 k03,k04;                   //thread_clamp_type		 
INT16 tension;             // tension 09.06.19 wr modify UINT8 to INT16
//shi ji zhi -200~400
INT16 tension_hole;        //09.06.19 wr //09.06.24 wr tong yi INT16
//ji zhun zhi 0~200
INT16 tension_delta;       //09.06.19 wr 
//xiang dui zhi -200~200  
INT16 temp_tension;        // temp tension 09.06.19 wr //09.06.24 wr tong yi INT16
//shi ji zhi ji suan hou de ying yong zhi 0~200    
INT16 temp_tension_last;   // temp tension last 09.06.19 wr add //09.06.24 wr tong yi INT16
//shang yi ci save zhi 
INT16 test_tension;        //test tension for ce shi mo shi //09.07.16 wr add
                    
INT16 inpress_high;        // inpresser high //09.06.16 wr modify UINT8 to INT8; 09.06.18 wr modify INT8 to INT16;
//shi ji zhi -70~140                                  //09.06.23 wr tong yi INT16
INT16 inpress_high_hole;   // zheng ge hua yang de inpresser high     //09.06.09 wr add
//ji zhun zhi 0~70                                    //09.06.23 wr tong yi INT16
INT16 inpress_delta;        //09.06.05 wr add  
//xiang dui zhi -70~70                                //09.06.23 wr tong yi INT16
INT16 inpress_position;    // inpresser position      //09.06.23 wr tong yi INT16 
INT16 emermove_high;       // inpresser high          //09.06.23 wr tong yi INT16
INT16 inpress_real_delta_runing;  // inpress real act delta  09.06.23 wr add

UINT8 foot_count; //0 1 position
UINT16 cut_count;           //cut counter biao shi di ji ci jian xian 08.12.29 wr 09.3.3 wr modify UINT8 to UINT16

UINT8 course_run_stop_flag;
UINT8 course_thread_run_stop_flag;

//ji xie jia xian add
UINT8 fk_status;	//song xian dian ci tie zhuang tai :OUT 0 wei xi he;//IN 1 xi he 09.09.10 wr add
UINT8 fk_cut;       //song xian dian ci tie ji shi shi jian dao; 0:wei dao;1:dao 09.09.11 wr add
UINT16 fk_count;	//song xian dian ci tie ji shu :20s hou duan dian 09.09.10 wr add

//--------------------------------------------------------------------------------------
// self test
//--------------------------------------------------------------------------------------    
UINT8 smotor_speed;        // smotor speed

UINT8 output_com;          // output command 1---do   0---stop      

UINT8 stepmotor_comm;      // stepping motor command
UINT8 stepmotor_state;     // stepping motor state
UINT8 stepmotor_single;    // stepping motor single act of cw or ccw 09.3.26 wr add
//--------------------------------------------------------------------------------------
// 
//-------------------------------------------------------------------------------------- 
UINT8 single_reply;        // single move step reply
UINT8 single_flag;         // single move step flag

UINT8 shift_reply;         // manual shift step reply
UINT8 shift_flag;          // manual shift step flag

UINT8 alarmled_flag;       // alarm led flag
UINT8 alarmbuz_flag;       // alarm buzzer flag
UINT16 sound_count;        // sound counter
INT16 power_off_count;     // power off counter
UINT16 pause_count;         // pause counter
UINT8 pause_flag;          // pause flag
UINT8 stay_flag;           // stay flag
UINT8 thbrk_count;         // thread breakage counter
UINT8 thbrk_flag;          // thread breakage flag
UINT8 brkdt_flag;          // breakage detection flag
UINT8 adtc_count;          // adtc counter
UINT8 adtc_flag;           // adtc flag
UINT8 sfsw_count;          // sfsw counter
UINT8 sfsw_flag;           // sfsw flag

UINT8 aging_com;           // aging command 
//--------------------------------------------------------------------------------------
//  1900a 
//--------------------------------------------------------------------------------------
UINT8 spi_flag;
UINT8 timer_x;
UINT8 timer_y;
UINT8 timer_yj;
UINT8 timer_zx;

//*******
INT16 tactic_flag_last = 0;
INT16 tactic_flag = 0;
INT16 iq_max_tester = 0;
INT16 speed_min_tester = 0;
INT16 door_ac = 1;
INT16 door_dt = 1;
INT16 spider_man;
INT16 spider_lim;
UINT8 m_status;
UINT8 pwm_forbid_flag;
UINT8 over_spd_flag;
//********

//2011-3-10
UINT16 last_max_length;
INT16 last_temp_speed;
UINT8 u102;  // step-motor move time compensation

//2011-4-12
UINT8 foot_2step_flag; //foot 2step-stroke
UINT8 foot_2step_start_flag; //foot 2step-stroke flag: allowing DVA to start 

//2011-4-15
UINT16 stepstatus1;                              
UINT16 stepstatus2;
UINT8  find_deadpoint_flag;//0 for do not find deadpoint or already found deadpoint,1 for finding
UINT8  x_step_current_level;
UINT8  y_step_current_level;
UINT8  inpress_step_current_level;
UINT8  foot_step_current_level;
UINT8  motor_para[10];

UINT8 allow;//0 for not allow ,1 for allow
//2011-7-26
UINT8 DAActionFlag;
UINT16 DAActionCounter;
INT16 m_spd_ref_n;
INT16 m_spd_n;
INT16 m_spd_n_last;

//2011-7-30
UINT8 running_overflow1,running_overflow2;

//2012-3-14  
INT8 trim_origin_pos;
//2012-4-18 
UINT8 single_comm;
UINT8 single_inpress_flag;
//2012-5-18 add
UINT8 FootNumber;
UINT8 k06;
//2012-6-7 add
UINT8 k112;
UINT8 k113;
UINT8 k114;
UINT8 stretch_foot_flag;
//2012-6-8 add
UINT16 stepversion1;                              
UINT16 stepversion2;
UINT16 stepversion3;
UINT16 stepversion4;
//2012-7-6 add
UINT8 movexy_delay_counter,movexy_delay_flag;
//2012-10-13 add
INT16 before_down_speed;
UINT8 k07;

UINT8 cut_control_flag;
UINT8 inpress_act_flag;
UINT8 k115;

UINT8 find_communication_start_flag;
UINT8 ready_go_setout_com,return_from_setout;
UINT8 auto_select_flag,auto_function_flag;
UINT8 pattern_delay,pattern_change_flag,pattern_change_finish;
UINT16 pattern_number,last_pattern_number; 
UINT8 new_pattern_done,PORG_action_flag;
UINT8 return_origin_flag,course_next_flag;
UINT8 new_pattern_done,first_select_flag;
UINT8 marking_speed,marking_flag,marking_finish_flag;
UINT8 formwork_identify_device; //0-sensor 1-scan coder
INT16 x_bios_offset,y_bios_offset;

//2013-11-12
UINT8 opl_origin_flag,x_motor_dir,y_motor_dir,led_light_adjust,go_original_flag;
INT16 allx_edit_step;
UINT16 one_step_delay;
UINT16 serail_number;   /* 条码识别号 */
UINT8  serail_module_sleep;
UINT8 serail_config_flag;

UINT8 go_origin_speed;
UINT8 go_setout_speed;
UINT8 nop_move_speed;
UINT16  pattern_change_counter;

UINT8 speed_down_stitchs,start_to_speed_down,speed_down_counter;
UINT16 ratio_array[17];

UINT8 test_flag;
UINT16 check_data_speed;

UINT8 xy_move_mode;
UINT8 HighSpeedStitchLength;
UINT8 speed_up_counter;
UINT8 release_tension_current,release_tension_time;
UINT8 nop_move_pause_flag;
INT32 read_step_x,read_step_y;
INT32 nop_move_remainx,nop_move_remainy;

PATTERN_DATA *last_pattern_point;
INT32 last_allx_step,last_ally_step;

INT8 sewingcontrol_flag,sewingcontrol_stitchs,need_backward_sewing;
UINT8 sewingcontrol_tail_flag,need_action_once,need_action_two;

UINT8 z_motor_dir;

UINT8 DVA_scan_flag,DVA_scan_counter,DVA_action_done;
UINT8 super_pattern_flag;
UINT16 pat_buff_write_offset;
UINT16 pat_buff_total_counter,bakeup_total_counter;
UINT16 pat_offset;

UINT8 cool_air_action_flag;
UINT16 cool_air_counter,cool_air_action_counter,cool_air_1_sec;
UINT16 cool_air_close_time,cool_air_open_time;

UINT8 cut_test_flag;

UINT8  check_bottom_thread_switch;//158
UINT32 bottom_thread_remain,set_default_length;      //156,157  *100   5000*100

UINT8 open_or_close_loop;
UINT16 alarm_thread_length;
UINT8  bottom_thread_alarm_type,thread_break_detect_level;//0-distance 1-stitchs

UINT8 stepmotor_need_double_time;
UINT8 movestepx_delay_counter,movestepy_delay_counter;

UINT8 second_start_switch,second_start_counter,power_on_ready;

UINT8 return_from_preddit;
UINT8 origin_com_after_pattern_done;

UINT8 alarm_output_enable;
UINT8 cutter_output_test;

UINT8 enable_stop_in_nopmove;
UINT8 special_go_allmotor_flag;

UINT8 already_in_origin;
UINT8 enable_thread_broken_alarm;
UINT8 finish_nopmove_pause_flag;    //在回起缝点的空送过程中，按急停了

PATTERN_DATA *target_pat_point;
INT32 target_allx_step,target_ally_step; 
UINT16 target_total_counter;

UINT8 XorgLast,YorgLast;
UINT8 inpress_port;
INT16 ct_holding_steps;
UINT8 baseline_alarm_flag;
UINT16 baseline_alarm_stitchs;
UINT8 release_poweron_ready;

UINT8 id_alarm_flag;
UINT16 id_alarm_counter,id_pattern;

INT8 k05;
INT8 k21,k22,k23,k24,k25,k26,k27,k28;
UINT8 x_sensor_pos;

UINT8 pattern_id_delay;
UINT8 front2stitchs_tension_off;
UINT8 inpress_type;

INT16 y_compensation;
INT16 x_compensation;
UINT8 last_direction,last_x_direction;
INT16 y_gear_compensation,x_gear_compensation;
UINT16 Corner_deceleration_speed;

UINT8 release_tension_before_nopmove,da0_release_flag;
UINT16 da0_release_conter;


UINT8 identify_mode,auto_origin_mode,y_curver_number;

UINT8 milling_cutter_action_flag, milling_first_move,milling_cutter_stop_flag;
UINT16 rotated_cutter_position,sum_rotated_angle;
UINT8 waitting_for_pattern_done;

UINT8  bobbin_case_switch_counter,bobbin_case_switch_flag,bobbin_case_workmode;
INT8   bobbin_case_arm_offset,bobbin_case_platform_offset;
UINT8  bobbin_case_arm_position,bobbin_case_platform_position;
UINT16 bobbin_case_dump_position;

UINT8 already_auto_find_start_point;

UINT8 bobbin_case_enable,rotated_cutter_enable;
UINT16 rotated_cutter_running_delay,rotated_cutter_up_delay;
UINT8 rotated_cutter_speed,rotated_cutter_current_level,bobbin_case_current_level;
UINT16 bobbin_case_inout_delay,bobbin_case_scrath_delay;

UINT8 checki11_item,checki11_test_flag,checki11_action,checki11_output_status[5];
UINT8 bobbin_case_once_done_flag;

UINT8 bobbin_case_stop_position,bobbin_case_alarm_mode,bobbin_case_restart_mode;
//#if SEND_SERVO_PARA
UINT8 svpara_buf[100];
UINT8 svpara_trans_flag;
UINT8 svpara_disp_buf[210];	
//#endif

UINT8 synchronization_flag;

/**********************************************
 * 机型判断参数
 *********************************************/
UINT8  ZJ_8060;
UINT8  X_AXIS_ORGIN_ENABLE;     //X轴找原点
UINT8  AUTO_SEARCH_START_POINT;
UINT8  SEND_SERVO_PARA;         //下发调试参数
UINT8  ROTATED_CUTTER;          //旋转切刀
UINT8  SUPPORT_CS3_FUN;         //支持扩展SC074A
UINT8  ENABLE_BOBBIN_CASE_FUN;  //换梭功能


UINT8  k167;                    //压脚动作
UINT8  k168;                    //加固速度
UINT8  k169;                    //机型选择
UINT8  k165;                    //吹气使能
UINT16 k166;                    //吹气时间

UINT8 k170;                     //X原点传感器类型：A(0)-老传感器；B(1)-新传感器，安装于台面
UINT8 k171;                     //CZ137功能选择：0-记号笔；1-吹气

UINT8 blow_air_enable;          //吹气功能使能；
UINT8 blow_air_flag;
UINT16 blow_air_counter;

UINT16 nop_move_control;

UINT8 flag_start_waitcom;      //开始启用通信等待标记
UINT8 counter_wait_com;        //100ms计数防止面板和主控通信等待时间过长
UINT8 flag_wait_com;           //100ms标记防止面板和主控通信等待时间过长
/*起缝原地加固模式
 * 0:不原地加固
 * 1:1针
 * 2:2针
 */
UINT8 start_sewcontrol_mode;      
UINT8 step_movetype;//2单步，其他正常

UINT8 download_drv_flag;
UINT8 recieve_flag;
UINT8 data_length_drv;
UINT8 erase_falg;
UINT8 DRV_DATA_LEN;
UINT16 drv_satus ;
FAULT_DIAG de_bug;//当前正在运行程序ID

UINT16 cutter_rotated_abs_angle;
UINT8 cutter_function_flag;
UINT16 drill_motor_run_enable,drill_motor_pwm,drill_motor_counter;

UINT8 check_valve4_flag,check_valve5_flag,last_rotated_dir;
INT8  cutter_motoer_remain_angle;

UINT16 dsp3_input_value;

//#if FOLLOW_INPRESS_FUN_ENABLE

UINT16 inpress_follow_down_angle,inpress_follow_up_angle;
//UINT16 inpress_follow_down_end_angle,inpress_follow_up_end_angle;
UINT8 inpress_follow_down_speed,inpress_follow_up_speed,enable_inpress_follow;
INT16 inpress_follow_range;
UINT8 movezx_delay_flag,movezx_delay_counter;

//#endif

UINT16 indraft_control_counter;
UINT8 indraft_control_flag;
UINT8 cutter_syn_counter;

UINT8  ct_bump_action_flag;
UINT16 ct_bump_counter,ct_bump_workingtime;

UINT8 last1_speed,last2_speed,last3_speed,last4_speed;
UINT8 cutter_syn_delay;

INT16 cutter_motor_initial_angle;

UINT8 stepper_cutter_enable;
UINT16 stepper_cutter_position;
UINT8 stepper_cutter_shake_rage,stepper_cutter_delay;

UINT8 debug_dsp_flag;
UINT16 pattern_chage_recover_counter;

INT32 x_origin_offset;
UINT8 x_origin_offset_effect;
UINT16 confirm_barcode;
UINT8 special_pause_flag;
UINT8 cutter_test_cnt;
INT8 stepper_cutter_origin_offset;
UINT8  dsp3_moto1_direction;

UINT32 counter_1ms;

UINT8 inpress_follow_high_flag;

INT32 debug_para1,debug_para2;

UINT8 release_tension_value_flag;

UINT16 find_x_origin_counter;
UINT8 autosewing_control_flag;
UINT8 autosewing_allset_flag;
UINT8 autosewing_allow_working,autosewing_switch_last_status;

UINT16 inpress_follow_down_angle_pc,inpress_follow_up_angle_pc;
UINT8 inpress_follow_down_speed_pc,inpress_follow_up_speed_pc;
INT16 inpress_follow_range_pc;



INT8 bobbin_plateform_org_offset;

UINT8 check_valve1_flag,check_valve2_flag,check_valve3_flag,check_valve6_flag;

UINT8 second_point_passby_flag;
UINT16 DEADPOINT;
UINT8 already_find_startpoint_flag;

UINT8 laser_fun_delay_off_flag;
UINT16 laser_fun_delay_counter;

PATTERN_DATA *monitor_pat_point; 
INT32 monitor_allx_step,monitor_ally_step; 

UINT16 over_counter;
UINT8 cutter_speed_done_flag;

UINT8  laser_power_on_flag,laser_power_error_flag;
UINT32 laser_power_on_counter;

UINT8 monitor_refresh_flag;

UINT16 rec1_datalength,rec1_package_length;
UINT8  rec1_status_machine;
UINT8 wirte_stepermotor_para_flag,write_eeprom_para_flag;
UINT8 write_stepmotor_curve_flag;

UINT8 double_xy_time_flag;
UINT8 drill_motor_updown_flag;

FAULT_DIAG sys_de_bug[32];//当前正在运行程序ID
UINT8 debug_read_point,debug_write_point,debug_counter;

#if SECOND_GENERATION_PLATFORM == 1
UINT8 SNT_H,SNT_ON,SUM,ALARM_LED,PWR_LED;
#endif

UINT16 step_curve_write_counter;
UINT8  pause_active_level;

UINT8 inflection_poin_flag;

UINT8  tra1_buf[255];      
UINT16 tra1_ind_r; 
UINT16 tra1_ind_w; 
UINT16 rec1_total_counter;
UINT8  laser_cutter_aciton_flag;

UINT16 crc_value,readback_crc_value;

UINT8 enable_footer_up, bar_coder_refresh_flag,oil_empty_alarm_enable,bar_coder_refresh_enable;
UINT8 laser_power_on_enable;
INT16 laser_offset_x,laser_offset_y;
UINT8 laser_test_flag[5];

UINT8 K227;
UINT16 sv_offset_angle;

UINT8 cutter_protect_flag;
UINT16 cutter_protect_counter;

UINT8 FL_pwm_action_flag,FL_pwm_counter;
UINT32 FL_pwm_period;

UINT8 power_on_ask_for_framework,auto_enable_catch_framework;
UINT16 remain_stitchs;

UINT8 power_on_first_find_x;

UINT8 autosewing_offset_moveing_done,testpin;

#if CHANGE_DOUBLE_FRAMEWORK 
	UINT8  left_start_action_flag,left_start_counter,left_start_lock_flag;
	UINT8  right_start_action_flag,right_start_counter,right_start_lock_flag;
	UINT8  left_quest_running;
	UINT8  right_quest_running;
	UINT8  current_running_flag;
	
	UINT16 para_x_take_offset_right,para_y_backward_dis,para_catch_delay_time;
	UINT16 para_left_barcode_position,para_right_barcode_position;
#else
	UINT16 para_x_take_offset_left,para_right_barcode_position,para_catch_delay_time,para_y_backward_dis,para_x_take_offset_right;		
#endif 

UINT8 com_refresh_flag,com_monitor_coder,x_monitor_steps,y_monitor_steps;

#if ENABLE_RFID_FUNCTION
UINT16  Rfid_Nom;
UINT8  rc522_write_falg;
UINT8  rc522_write_ret_falg;
UINT8  rc522_control_falg,rfid_config_flag;
#endif

UINT16 delay_before_inpress_down,ms_scan_counter;
UINT8  inpress_not_working_stitchs,inpress_follow_mode;

UINT8 frame_start_counter,frame_start_flag;

INT16 pen_x_bios_offset,pen_y_bios_offset;

UINT8 monitor_x_step,monitor_y_step,monitor_x_time,monitor_y_time,monitor_tabx_time,monitor_taby_time;
UINT16 monitor_point;
UINT8 chang_parameter_flag;
UINT8 powr_on_zx_flag;

UINT8  inpress_litter_footer_action_flag, inpress_litter_footer_action_value;
UINT16 inpress_litter_footer_action_counter;

UINT8 yj_motor_dir;
UINT8 first_start_flag;


UINT8 special_encoder_mode;
UINT16 crc1,crc2,crc3;

SYSTEM_PARA para;
UINT8 stm32_input[4],stm32_output[4],spi_out_status[32];

UINT8 laser_emergency_stop,emergency_restart_flag;
UINT32 valve_3_open_counter;
UINT8 find_laser_start_stop_flag;
UINT8 laser_already_begin_flag;

UINT8 rotated_cutter_running_flag;
UINT32 rotated_cutter_running_counter;
UINT8 power_on_laser_cool_flag;
UINT8 main_control_lock_flag,main_control_lock_setup;

UINT8 making_pen_actoin_flag,making_pen_status;

UINT8 rfid_debug_flag,uart1_sending_flag;

UINT16 cutter_angle_adjust,monitor_cutter_angle;

UINT8 checki10_follow_flag, checki10_action0,checki10_action1;
UINT8 one_step_run_flag;

SYSTEM_PARA5 para5;
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
