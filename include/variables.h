//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : variables.h
//  Description: external variables declaration
//  Version    Date     Author    Description
//  0.01     03/07/07   pbb        created
//  ... 
//  ...
//--------------------------------------------------------------------------------------
#ifndef VARIABLES_H
#define VARIABLES_H
//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "typedef.h"      //Data type define
#include "common.h"
//--------------------------------------------------------------------------------------
//  External variables declaration
//--------------------------------------------------------------------------------------
extern SYS_STRU sys;
extern MOTOR_STRU motor;
extern PATTERN_DATA *pat_point;
extern PATTERN_DATA *sta_point;
extern PATTERN_DATA *TempEnd_point;
extern PATTERN_DATA *TempStart_point;
extern PATTERN_DATA *TempPat_point;
extern PATTERN_DATA PatStart;
extern PATTERN_DATA PatEnd;

extern STEPVERSION Step1Version;
extern STEPVERSION Step2Version;
extern UINT16 MotorStuckCounter;
extern UINT8 IniFlag;
extern UINT8 CourseBackStartFlag;

extern UINT8 flag_1ms;
extern UINT16 ms_counter; 
extern UINT16 wipe_time; 
extern UINT8 inpress_time;
extern UINT16 us_counter;  
extern UINT8 pat_buf[]; 

extern UINT8 *recpat_point;         

extern UINT8 pedal_state;
extern UINT8 pedal_last_state;
extern UINT16 cut_start_angle;
extern UINT16 cut_end_angle;
extern UINT16 tension_start_angle;
extern UINT16 tension_end_angle;
extern UINT8 wiper_start_time;
extern UINT8 k52;
extern UINT8 pedal_style;
extern UINT8 foot_flag;     
extern UINT8 foot_half_flag;  
extern UINT8 inpress_flag;   
extern UINT8 clamp_flag;
extern UINT8 OutOfRange_flag;
extern UINT8 DVSMLastState;
 
extern UINT8 zpl_pass;   
extern UINT8 stitch_counter_tmp;
  
extern UINT8 origin_com; 
extern UINT8 commandpoint_com;
extern UINT8 wind_com;    
extern UINT8 repeat_com;       

extern UINT8 foot_com; 
extern UINT8 foot_half_com;
extern UINT8 inpress_com; 
extern UINT8 coor_com;   
extern UINT8 cooradjust_com;      // coordinate compensation command 
   
extern UINT16 stitch_num; 
extern UINT16 stitch_counter;
extern INT32 allx_step,ally_step;
extern INT16 allx_edit_step,ally_edit_step;
extern INT16 xstep_cou,ystep_cou;
extern INT32 sox_step,soy_step;   
extern INT16 manx_step,many_step;
extern INT32 comx_step,comy_step; 
extern INT16 stacoorx_step,stacoory_step; 
extern INT16 origin2x_step,origin2y_step; 
extern INT16 currentpyx_step,currentpyy_step; 
extern INT16 comx_polar_step,comy_polar_step;
extern INT16 cooradjustx_step,cooradjusty_step;   // x and y shift compensation 
extern INT16 cooradjustxtotal_step,cooradjustytotal_step;
extern INT16 coordisxtotal_step,coordisytotal_step;
extern INT16 xcurrent_coordinates,ycurrent_coordinates;
extern INT32 allx_coor_temp,ally_coor_temp;
extern INT32 allx_step_temp,ally_step_temp;
extern UINT8 predit_shift;
extern UINT32 DelayCounter;
extern UINT32 DelayTime;
extern UINT8 curpat_have_origin2;


extern UINT8 MotorPositionSet;
extern UINT8 InpressHighControlFlag;
extern UINT8 InpressAbsoluteHigh;
extern UINT8 OutputFlag;
extern UINT8 OutputNumber;
extern UINT8 InputFlag;
extern UINT8 InputNumber;
extern UINT8 PatternDelayFlag;
extern UINT16 PattenrDelayTime;
extern UINT8 RotateFlag;
extern UINT8 Tension3thFlag;
extern UINT8 ElectronicClampFlag;
extern UINT8 ElectronicClampValue;
extern UINT8 AreaDivisionFlag;
extern UINT8 KnifeDriveFlag;
extern UINT8 KnifeStyle;
extern UINT8 SewingSpeedFlag;
extern UINT8 SewingSpeedValue;
extern UINT8 NopmoveFuncFlag;
extern UINT8 NopmoveSpeed;
extern UINT8 InpressHighRelativeFlag;
extern UINT8 InpressHighRelativeValue;
extern UINT8 SewingStopFlag;
extern UINT8 SewingStopValue;
extern UINT8 MotorSpeedRigister;
extern INT16 CancelAllXCoor;
extern INT16 CancelAllYCoor;
extern UINT8 CancelAllCoorFlag;
extern UINT8 StitchUpFlag;
extern UINT8 PointShiftFlag;
extern UINT8 PointShiftDirection;
extern UINT16 PointShiftNumber;
extern UINT8 EndStyleChooseFlag;
extern UINT16 NopMoveElement;
extern UINT16 SewingElement;
extern UINT16 ElementNumber;
extern UINT16 SewTestStitchCounter;
extern UINT16 PatternShiftValue;
extern UINT8 LastNopMoveFlag;
extern UINT8 SewStitchCounterFlag;
extern UINT8 SingleManualFlag;
extern UINT8 ShapePointFlag;
extern UINT8 SewDataFlag;
extern UINT8 ElementPointFlag;
extern UINT8 ElementIndex;
extern UINT8 ElementIndexLatch;
extern UINT8 TestStatusChangeFlag;
extern UINT8 MoveToCurrentCoorFlag;
extern INT16 CurrentXCoor;
extern INT16 CurrentYCoor;
extern UINT8 EditEntranceFlag;
extern UINT8 SewingTestEndFlag;
extern INT16 StepDrive1Version;
extern INT16 StepDrive2Version;
extern UINT8 StatusChangeLatch;
extern UINT8 CurrentPointShiftFlag;
extern INT16 CurrentPointShiftPosition;
extern UINT8 EditElementCommand;
extern UINT8 CancelOverlabPointFlag;
extern UINT8 FootUpCom;
extern UINT8 DVBLastState;
extern UINT8 DVALastState;
extern UINT8 FootRotateFlag;

extern INT16 DestinationPointShiftPosition;
extern UINT8 MotionSet;
extern UINT8 MotiongSetFlag;
extern UINT8 StopStatusFlag;

extern UINT8 move_flag;   
extern UINT8 movex_flag;		   // move x flag in pattern process
extern UINT8 movey_flag;		   // move y flag in pattern process
extern UINT8 movestep_x_flag;	   // move x step motor flag
extern UINT8 movestep_y_flag;     // move y step motor flag        
extern UINT8 lastmove_flag; 
extern UINT8 origin2_lastmove_flag; 
extern UINT8 NopMoveSpd_flag;   
extern UINT8 nopmove_flag;
extern UINT8 cut_flag;
extern UINT8 laststitch_flag;   
extern UINT8 cut_start_flag;
extern UINT8 cut_end_flag;
extern UINT8 tension_start_flag;
extern UINT8 tension_end_flag;
extern UINT8 wipe_start_flag;
extern UINT8 wipe_end_flag;
extern UINT8 inpress_start_flag;
extern UINT8 inpress_end_flag;
extern UINT8 tension_start_flag;
extern UINT8 tension_end_flag;
extern UINT8 wipe_start_time;
extern UINT8 wipe_end_time; 
extern UINT8 stop_flag;   
extern UINT8 stop_number;
extern UINT8 first_stitch_flag;      
extern UINT8 end_flag;
extern UINT8 machine_stop_flag;
extern UINT8 slow_flag;
extern UINT8 check_flag;   
extern UINT8 process_flag;               
extern UINT8 calculate_flag; 
extern UINT8 start_flag; 
extern UINT8 connect_flag; 
extern UINT8 motorconfig_flag;    
extern UINT8 stepconfig_flag; 
extern UINT8 InpresserIni_flag;    
extern UINT16 DelayMsCount;
extern UINT8 StitchStartFlag;
     
extern INT16 movestep_angle; 
extern INT16 movestep_time;  


extern INT16 movestepx_angle;      // move x step angle
extern INT16 movestepy_angle;      // move y step angle
extern INT16 movect_angle;   
extern INT16 last_speed;

extern INT16 allyj_step;          
extern INT16 allin_step;          
extern INT16 allct_step;     

extern UINT8 clamp_stepflag;  
extern UINT8 tb1_flag;     


extern UINT8 status_now;
extern UINT8 status_15;
extern UINT8 FindZeroFlag;
extern UINT8 EncoderZ_flag;
extern UINT8 motor_stuck_flag;
//--------------------------------------------------------------------------------------
// sewing parameter
//-------------------------------------------------------------------------------------- 
extern INT16 sew_speed;        
extern UINT8 u15;         
extern UINT16 sew_stitch; 
extern UINT8 findorigin_flag;
extern UINT8 u71; 
extern UINT8 u51; 
extern UINT8 u49; 
extern UINT8 u02;                  
extern UINT8 u03;                  
extern UINT8 u04;                  
extern UINT8 u05;                  
extern UINT8 u06;                  
extern UINT8 u07;                  
extern UINT8 u08;                  
extern INT8 u09;                  
extern UINT8 u10;                  
extern UINT8 u11;                  
extern UINT8 u12;                  
extern UINT8 u13;                  
extern UINT8 u14;  
extern UINT8 u201;
extern UINT8 u202;  
extern UINT8 u203;
extern UINT8 u204;
extern INT8 u16; 
extern UINT8 u26; 
extern UINT8 u33; 
extern INT8 u34;               
extern UINT8 clamp_com;
extern INT8 u36; 
extern UINT8 u37;  
extern UINT8 u38; 
extern UINT8 u39;
extern UINT8 u40;
extern UINT8 u205;
extern UINT8 u51;
extern UINT8 u207;
extern UINT8 u208;
extern UINT8 u209;
extern UINT8 u46;
extern INT16 k43;
extern UINT8 k01;
extern INT32 k56;
extern INT32 k57;
extern INT32 k58;
extern INT32 k59;
extern UINT8 u217;
extern UINT8 u218;
extern UINT8 u219;
extern UINT8 u220;
extern UINT8 u221;
extern UINT8 u222;
extern UINT8 u223;
extern UINT8 u224;
extern UINT8 u225;
extern UINT8 u226;
extern UINT8 u227;
extern UINT8 u228;
extern UINT8 u229;
extern UINT8 u230;
extern UINT8 u231;
extern UINT8 u232;
extern UINT8 u233;
extern UINT8 u234;
extern UINT8 u235;
extern UINT8 u236;
extern UINT8 k02;
extern UINT8 k110;
extern UINT16 k111;
extern UINT8 k92;
extern UINT8 k93;
extern UINT8 k60;
extern INT16 AdjustAngleSet;
extern UINT8 u41; 
extern UINT8 u42;   
extern UINT8 u46;   
extern UINT8 u48;              
extern UINT8 u68;                 
extern UINT8 u69;                 
extern UINT8 u70;
extern UINT8 u94;
extern UINT8 u97;                 
extern UINT8 u101; 
extern UINT8 u103;                
extern UINT8 u104;                
extern UINT8 u105;                
extern UINT8 u112; 
extern UINT8 u89; 
extern UINT8 aging_flag; 
extern UINT8 aging_delay; 
extern UINT8 u72; 
extern UINT8 u73;
extern UINT8 u35;
extern INT16 x_bios;
extern INT16 y_bios;
extern INT16 DisMotorAngle;
extern INT16 AdjustAngle;

//--------------------------------------------------------------------------------------
// 210E add 10.02.01 wr add
//--------------------------------------------------------------------------------------
extern UINT8 u81;
extern UINT8 u82;
extern UINT8 u84;
extern UINT8 u85;
extern UINT8 u86;
extern UINT8 u87;
extern UINT8 u91;
extern UINT8 u108;
extern UINT8 u129;
extern UINT8 u245;
extern UINT8 k31;
extern UINT8 k53;
extern UINT8 k54;
extern UINT8 k60; //2011-2-28
extern UINT16 k61;
extern UINT8 k63;
extern UINT8 k67;
extern UINT8 k74;
extern UINT8 k75;
extern INT8 k95;
extern UINT8 k98;
extern UINT8 k100;
extern UINT8 k131;
extern UINT8 k150;


extern UINT8 k03,k04;       //2011-2-17 songyang added
extern INT16 tension;             // tension 09.06.19 wr modify UINT8 to INT16
//shi ji zhi -200~400
extern INT16 tension_hole;        //09.06.19 wr //09.06.24 wr tong yi INT16
//ji zhun zhi 0~200
extern INT16 tension_delta;       //09.06.19 wr 
//xiang dui zhi -200~200  
extern INT16 temp_tension;        // temp tension 09.06.19 wr //09.06.24 wr tong yi INT16
//shi ji zhi ji suan hou de ying yong zhi 0~200    
extern INT16 temp_tension_last;   // temp tension last 09.06.19 wr add //09.06.24 wr tong yi INT16
//shang yi ci save zhi 
extern INT16 test_tension;        //test tension for ce shi mo shi //09.07.16 wr add
                    
extern INT16 inpress_high;        // inpresser high //09.06.16 wr modify UINT8 to INT8; 09.06.18 wr modify INT8 to INT16;
//shi ji zhi -70~140                                  //09.06.23 wr tong yi INT16
extern INT16 inpress_high_hole;   // zheng ge hua yang de inpresser high     //09.06.09 wr add
//ji zhun zhi 0~70                                    //09.06.23 wr tong yi INT16
extern INT16 inpress_delta;        //09.06.05 wr add  
//xiang dui zhi -70~70                                //09.06.23 wr tong yi INT16
extern INT16 inpress_position;    // inpresser position      //09.06.23 wr tong yi INT16 
extern INT16 emermove_high;       // inpresser high          //09.06.23 wr tong yi INT16
extern INT16 inpress_real_delta_runing;  // inpress real act delta  09.06.23 wr add

extern UINT8 foot_count; //0 1 position
extern UINT16 cut_count;           //cut counter biao shi di ji ci jian xian 08.12.29 wr 09.3.3 wr modify UINT8 to UINT16
extern UINT8 course_run_stop_flag;
extern UINT8 course_thread_run_stop_flag;

//ji xie jia xian add
extern UINT8 fk_status;	//song xian dian ci tie zhuang tai :OUT 0 wei xi he;//IN 1 xi he 09.09.10 wr add
extern UINT8 fk_cut;       //song xian dian ci tie ji shi shi jian dao; 0:wei dao;1:dao 09.09.11 wr add
extern UINT16 fk_count;	//song xian dian ci tie ji shu :20s hou duan dian 09.09.10 wr add

//--------------------------------------------------------------------------------------
// self test
//-------------------------------------------------------------------------------------- 
extern UINT8 smotor_speed;

extern UINT8 output_com; 
 
extern UINT8 stepmotor_comm; 
extern UINT8 stepmotor_state;  
extern UINT8 stepmotor_single;    // stepping motor single act of cw or ccw 09.3.26 wr add
//--------------------------------------------------------------------------------------
// 
//-------------------------------------------------------------------------------------- 
extern UINT8 single_reply;   
extern UINT8 single_flag;     

extern UINT8 shift_reply;         
extern UINT8 shift_flag;          

extern UINT8 alarmled_flag;         
extern UINT8 alarmbuz_flag;  
extern UINT16 sound_count;
extern INT16 power_off_count;
extern UINT16 pause_count;   
extern UINT8 pause_flag;      
extern UINT8 stay_flag;      
extern UINT8 thbrk_count;         
extern UINT8 thbrk_flag; 
extern UINT8 brkdt_flag; 
extern UINT8 adtc_count;   
extern UINT8 adtc_flag;                        
extern UINT8 sfsw_count;  
extern UINT8 sfsw_flag;      

extern UINT8 aging_com;                         
//--------------------------------------------------------------------------------------
//  1900a 
//--------------------------------------------------------------------------------------
extern UINT8 spi_flag;
extern UINT8 timer_x;
extern UINT8 timer_y;
extern UINT8 timer_yj;
extern UINT8 timer_zx;


//******
extern INT16 tactic_flag_last ;
extern INT16 tactic_flag ;
extern INT16 iq_max_tester;
extern INT16 speed_min_tester;
extern INT16 door_ac;
extern INT16 door_dt;
extern INT16 spider_man;
extern INT16 spider_lim;
extern UINT8 m_status;
extern UINT8 pwm_forbid_flag;
extern UINT8 over_spd_flag;
//*********

//2011-3-10
extern UINT16 last_max_length;
extern INT16 last_temp_speed;
extern UINT8 u102;  // step-motor move time compensation

//2011-4-12
extern UINT8 foot_2step_flag; //foot 2step-stroke
extern UINT8 foot_2step_start_flag; //foot 2step 

//2011-4-15
extern UINT8 find_deadpoint_flag;
extern UINT16 stepstatus1;                              
extern UINT16 stepstatus2;
extern UINT8  x_step_current_level;
extern UINT8  y_step_current_level;
extern UINT8 allow;
extern UINT8  inpress_step_current_level;
extern UINT8  foot_step_current_level;
extern UINT8  motor_para[10];

//2011-7-26
extern UINT8 DAActionFlag;
extern INT16 m_spd_ref_n;
extern INT16 m_spd_n;
extern INT16 m_spd_n_last;
extern UINT16 DAActionCounter;
//2011-7-30
extern UINT8 running_overflow1,running_overflow2;
//2012-3-14  
extern INT8 trim_origin_pos;
//2012-4-18 
extern UINT8 single_comm;
extern UINT8 single_inpress_flag;
//2012-5-18 add
extern UINT8 FootNumber;
extern UINT8 k06;
//2012-6-7 add
extern UINT8 k112;
extern UINT8 k113;
extern UINT8 k114;
extern UINT8 stretch_foot_flag;//2012-6-8 add
extern UINT16 stepversion1;                              
extern UINT16 stepversion2;
extern UINT16 stepversion3;
extern UINT16 stepversion4;
//2012-7-6 add
extern UINT8 movexy_delay_counter,movexy_delay_flag;
//2012-10-13 add
extern INT16 before_down_speed;
extern UINT8 k07;

extern UINT8 cut_control_flag;
extern UINT8 inpress_act_flag;
extern UINT8 k115;

extern UINT8 find_communication_start_flag; 
extern UINT8 ready_go_setout_com,return_from_setout;
extern UINT8 auto_select_flag,auto_function_flag;
extern UINT8 pattern_delay,pattern_change_flag,pattern_change_finish;
extern UINT16 pattern_number,last_pattern_number;  /* 2013.11.11修改为16位  */
extern UINT8 new_pattern_done,PORG_action_flag;
extern UINT8 return_origin_flag,course_next_flag;
extern UINT8 new_pattern_done,first_select_flag;
extern UINT8 marking_speed,marking_flag,marking_finish_flag;
extern UINT8 formwork_identify_device;
extern INT16 x_bios_offset,y_bios_offset;

//2013-11-12

extern UINT8 opl_origin_flag,x_motor_dir,y_motor_dir,led_light_adjust,go_original_flag;
extern INT16 allx_edit_step;
extern UINT16 one_step_delay;
extern UINT16 serail_number;
extern UINT8  serail_module_sleep;
extern UINT8 serail_config_flag;

extern UINT8 go_origin_speed;
extern UINT8 go_setout_speed;
extern UINT8 nop_move_speed;

extern UINT16  pattern_change_counter;

extern UINT8  speed_down_stitchs,start_to_speed_down,speed_down_counter;
extern UINT16 ratio_array[];
extern UINT8 test_flag;
extern UINT16 check_data_speed;

extern UINT8 xy_move_mode;
extern UINT8 HighSpeedStitchLength;
extern UINT8 speed_up_counter;
extern UINT8 release_tension_current,release_tension_time;
extern UINT8 nop_move_pause_flag,nop_move_pause_action;
extern UINT8 course_running_flag;
extern INT32 read_step_x,read_step_y;
extern INT32 nop_move_remainx,nop_move_remainy;

extern PATTERN_DATA *last_pattern_point;
extern INT32 last_allx_step,last_ally_step;
extern INT8 sewingcontrol_flag,sewingcontrol_stitchs,need_backward_sewing;
extern UINT8 sewingcontrol_tail_flag,need_action_once,need_action_two;

extern UINT8 z_motor_dir;
extern UINT8 repeat_same_pattern_flag;
extern UINT8 DVA_scan_flag,DVA_scan_counter,DVA_action_done;
extern UINT8 super_pattern_flag;
extern UINT16 pat_buff_write_offset;
extern UINT16 pat_buff_total_counter,bakeup_total_counter;
extern UINT16 pat_offset;



extern UINT8 cool_air_action_flag;
extern UINT16 cool_air_counter,cool_air_action_counter,cool_air_1_sec;
extern UINT16 cool_air_close_time,cool_air_open_time;
extern UINT8 cut_test_flag;

extern UINT8  check_bottom_thread_switch;
extern UINT32 bottom_thread_remain,set_default_length; 
extern UINT8 open_or_close_loop;
extern UINT16 alarm_thread_length;
extern UINT8  bottom_thread_alarm_type,thread_break_detect_level;

extern UINT8 stepmotor_need_double_time;
extern UINT8 movestepx_delay_counter,movestepy_delay_counter;

extern UINT8 second_start_switch,second_start_counter,power_on_ready;
extern UINT8 return_from_preddit,x_find_org,y_find_org;

extern UINT8 origin_com_after_pattern_done;
extern UINT8 alarm_output_enable;
extern UINT8 cutter_output_test;
extern UINT8 enable_stop_in_nopmove;

extern UINT8 enable_stop_in_nopmove;
extern UINT8 special_go_allmotor_flag;

extern UINT8 already_in_origin;
extern UINT8 enable_thread_broken_alarm;
extern UINT8 finish_nopmove_pause_flag;

extern PATTERN_DATA *target_pat_point;
extern INT32 target_allx_step,target_ally_step; 
extern UINT16 target_total_counter;

extern UINT8 XorgLast,YorgLast;
extern UINT8 inpress_port;
extern INT16 ct_holding_steps;
extern UINT8 baseline_alarm_flag;
extern UINT16 baseline_alarm_stitchs;

extern UINT8 release_poweron_ready;

extern UINT8 id_alarm_flag;
extern UINT16 id_alarm_counter,id_pattern;

extern INT8 k05;
extern INT8 k21,k22,k23,k24,k25,k26,k27,k28;
extern UINT8 x_sensor_pos;

extern UINT8 pattern_id_delay;
extern UINT8 front2stitchs_tension_off;
extern UINT8 inpress_type;

extern INT16 y_compensation;
extern INT16 x_compensation;
extern UINT8 last_direction,last_x_direction;
extern INT16 y_gear_compensation,x_gear_compensation;
extern UINT16 Corner_deceleration_speed;

extern UINT8 release_tension_before_nopmove,da0_release_flag;
extern UINT16 da0_release_conter;

extern UINT8 blow_air_flag;
extern UINT16 blow_air_counter;

extern UINT8 identify_mode,auto_origin_mode,y_curver_number;

extern UINT8 milling_cutter_action_flag, milling_first_move,milling_cutter_stop_flag;
extern UINT16 rotated_cutter_position,sum_rotated_angle;
extern UINT8 waitting_for_pattern_done;

extern UINT8  bobbin_case_switch_counter,bobbin_case_switch_flag,bobbin_case_workmode;
extern INT8   bobbin_case_arm_offset,bobbin_case_platform_offset;
extern UINT8  bobbin_case_arm_position,bobbin_case_platform_position;
extern UINT16 bobbin_case_dump_position;

extern UINT8 already_auto_find_start_point;

extern UINT8 bobbin_case_enable,rotated_cutter_enable;
extern UINT16 rotated_cutter_running_delay,rotated_cutter_up_delay;
extern UINT8 rotated_cutter_speed,rotated_cutter_current_level,bobbin_case_current_level;
extern UINT16 bobbin_case_inout_delay,bobbin_case_scrath_delay;

extern UINT8 checki11_item,checki11_test_flag,checki11_action,checki11_output_status[];
extern UINT8 bobbin_case_once_done_flag;

extern UINT8 bobbin_case_stop_position,bobbin_case_alarm_mode,bobbin_case_restart_mode;

extern UINT8 svpara_buf[];
extern UINT8 svpara_trans_flag;
extern UINT8 svpara_disp_buf[];

extern UINT8 synchronization_flag;
extern const UINT8 MoveTime1_Speed[];
extern const UINT16 spdlimit1_tab[];
/**********************************************
 * 机型判断参数
 *********************************************/

extern UINT8 ZJ_8060;
extern UINT8 X_AXIS_ORGIN_ENABLE;     //X轴找原点
extern UINT8 AUTO_SEARCH_START_POINT;
extern UINT8 SEND_SERVO_PARA;         //下发调试参数
extern UINT8 ROTATED_CUTTER;          //旋转切刀
extern UINT8 SUPPORT_CS3_FUN;         //支持扩展SC074A
extern UINT8 ENABLE_BOBBIN_CASE_FUN;  //换梭功能
extern UINT8  IS_SC0413;               //是否是0413的机头板
 
extern UINT8 k167;                    //压脚动作
extern UINT8 k168;                    //加固速度
extern UINT8 k169;                    //机型选择

extern UINT8 k170;                    //X原点传感器类型：A(0)-老传感器；B(1)-新传感器，安装于台面
extern UINT8 k171;                    //CZ137功能选择：0-记号笔；1-吹气

extern UINT8  k165;                   //吹气使能
extern UINT16 k166;                   //吹气时间

extern UINT8 blow_air_enable;         //吹气功能使能；
extern UINT8 blow_air_flag;
extern UINT16 blow_air_counter;

extern UINT16 nop_move_control;


extern UINT8 flag_start_waitcom;      //开始启用通信等待标记
extern UINT8 counter_wait_com;        //100ms计数防止面板和主控通信等待时间过长
extern UINT8 flag_wait_com;           //100ms标记防止面板和主控通信等待时间过长

extern UINT8 start_sewcontrol_mode;
extern UINT8 step_movetype;

extern UINT8 data_length_drv;
extern UINT8 erase_falg;
extern UINT8 DRV_DATA_LEN;
extern UINT16 drv_satus ;
extern UINT8 download_drv_flag;
extern UINT16 read_stepmotor_up_drv(void);
extern void jump_to_begin(void);
extern FAULT_DIAG de_bug;//当前正在运行程序ID

extern UINT16 cutter_rotated_abs_angle;
extern UINT8  cutter_function_flag;
extern UINT16 drill_motor_run_enable,drill_motor_pwm,drill_motor_counter;

extern UINT8 check_valve4_flag,check_valve5_flag,last_rotated_dir;
extern INT8  cutter_motoer_remain_angle;
extern UINT16 dsp3_input_value;

//#if FOLLOW_INPRESS_FUN_ENABLE 
extern UINT16 inpress_follow_down_angle,inpress_follow_up_angle;

extern UINT8 inpress_follow_down_speed,inpress_follow_up_speed,enable_inpress_follow;
extern INT16 inpress_follow_range;
extern UINT8 movezx_delay_flag,movezx_delay_counter;
//#endif

extern UINT16 indraft_control_counter;
extern UINT8 indraft_control_flag;
extern UINT8 cutter_syn_counter;

extern UINT8 last1_speed,last2_speed,last3_speed,last4_speed;
extern UINT8 cutter_syn_delay;
extern UINT8  ct_bump_action_flag;
extern UINT16 ct_bump_counter,ct_bump_workingtime;
extern INT16 cutter_motor_initial_angle;

extern UINT8 stepper_cutter_enable;
extern UINT16 stepper_cutter_position;
extern UINT8 stepper_cutter_shake_rage,stepper_cutter_delay;
extern UINT8 debug_dsp_flag;

extern UINT16 pattern_chage_recover_counter;

extern INT32 x_origin_offset;
extern UINT8 x_origin_offset_effect;
extern UINT16 confirm_barcode;
extern UINT8 special_pause_flag;
extern UINT8 cutter_test_cnt;
extern INT8 stepper_cutter_origin_offset;
extern UINT8  dsp3_moto1_direction;
extern UINT32 counter_1ms;

extern UINT8 inpress_follow_high_flag;

extern union TRANS trans_x;
extern union TRANS trans_y;
extern union TRANS trans_z;
extern union RECV recieve_x;
extern union RECV recieve_y;
extern union RECV recieve_z;

extern INT32 debug_para1,debug_para2;
extern UINT8 release_tension_value_flag;

extern UINT16 find_x_origin_counter;

extern UINT8 autosewing_control_flag;
extern UINT8 autosewing_allset_flag;
extern UINT8 autosewing_allow_working,autosewing_switch_last_status;


extern UINT16 inpress_follow_down_angle_pc,inpress_follow_up_angle_pc;
extern UINT8 inpress_follow_down_speed_pc,inpress_follow_up_speed_pc;
extern INT16 inpress_follow_range_pc;

extern UINT8 offset_working_on;
extern INT8 bobbin_plateform_org_offset;
extern UINT8 check_valve1_flag,check_valve2_flag,check_valve3_flag,check_valve6_flag;

extern UINT8 second_point_passby_flag;
extern UINT16 DEADPOINT;
extern UINT8 already_find_startpoint_flag;

extern UINT8 laser_fun_delay_off_flag;
extern UINT16 laser_fun_delay_counter;

extern PATTERN_DATA *monitor_pat_point; 
extern INT32 monitor_allx_step,monitor_ally_step; 

extern UINT16 over_counter;
extern UINT8 cutter_speed_done_flag;

extern UINT8  laser_power_on_flag,laser_power_error_flag;
extern UINT32 laser_power_on_counter;

extern UINT8 monitor_refresh_flag;

extern UINT16 rec1_datalength,rec1_package_length;
extern UINT8  rec1_status_machine;
extern UINT8 wirte_stepermotor_para_flag,write_eeprom_para_flag;
extern UINT8 write_stepmotor_curve_flag;

extern UINT8 double_xy_time_flag;
extern UINT8 drill_motor_updown_flag;

extern FAULT_DIAG sys_de_bug[];//当前正在运行程序ID
extern UINT8 debug_read_point,debug_write_point,debug_counter;

#if SECOND_GENERATION_PLATFORM == 1
extern UINT8 SNT_H,SNT_ON,SUM,ALARM_LED,PWR_LED;
#endif

extern UINT16 step_curve_write_counter;
extern UINT8  pause_active_level;
extern UINT8 inflection_poin_flag;

extern UINT8 tra1_buf[]; 
extern UINT16 tra1_ind_r; 
extern UINT16 tra1_ind_w; 
extern UINT16 rec1_total_counter;
extern UINT8  laser_cutter_aciton_flag;

extern UINT16 crc_value,readback_crc_value;
extern UINT8 enable_footer_up, bar_coder_refresh_flag,oil_empty_alarm_enable,bar_coder_refresh_enable;
extern UINT8 laser_power_on_enable;
extern INT16 laser_offset_x,laser_offset_y;
extern UINT8 laser_test_flag[];

extern UINT8 K227;
extern UINT16 sv_offset_angle;

extern UINT8 cutter_protect_flag;
extern UINT16 cutter_protect_counter;
extern UINT8 FL_pwm_action_flag,FL_pwm_counter;
extern UINT32 FL_pwm_period;

extern UINT8 power_on_ask_for_framework,auto_enable_catch_framework;
extern UINT16 remain_stitchs;

extern UINT8 power_on_first_find_x;
extern UINT8 autosewing_offset_moveing_done,testpin;


#if CHANGE_DOUBLE_FRAMEWORK 
	extern UINT8  left_start_action_flag,left_start_counter,left_start_lock_flag;
	extern UINT8  right_start_action_flag,right_start_counter,right_start_lock_flag;
	extern UINT8  left_quest_running;
	extern UINT8  right_quest_running;
	extern UINT8  current_running_flag;
	extern UINT16 para_x_take_offset_left,para_x_take_offset_right,para_y_backward_dis,para_catch_delay_time;
	extern UINT16 para_left_barcode_position,para_right_barcode_position;
#else
	extern UINT16 para_right_barcode_position,para_x_take_offset_left,para_catch_delay_time,para_y_backward_dis,para_x_take_offset_right;
#endif

extern UINT8 com_refresh_flag,com_monitor_coder,x_monitor_steps,y_monitor_steps;

#if ENABLE_RFID_FUNCTION
extern UINT16  Rfid_Nom;
extern UINT8  rc522_write_falg;
extern UINT8  rc522_write_ret_falg;
extern UINT8  rc522_control_falg,rfid_config_flag;
#endif

extern UINT16 delay_before_inpress_down,ms_scan_counter;
extern UINT8  inpress_not_working_stitchs,inpress_follow_mode;

extern UINT8 frame_start_counter,frame_start_flag;
extern INT16 pen_x_bios_offset,pen_y_bios_offset;

extern UINT8 monitor_x_step,monitor_y_step,monitor_x_time,monitor_y_time,monitor_tabx_time,monitor_taby_time;
extern UINT16 monitor_point;
extern UINT8 chang_parameter_flag;
extern UINT8 powr_on_zx_flag;

extern UINT8  inpress_litter_footer_action_flag, inpress_litter_footer_action_value;
extern UINT16 inpress_litter_footer_action_counter;

extern UINT8 yj_motor_dir;
extern UINT8 first_start_flag;



extern UINT8 special_encoder_mode;

extern UINT16 crc1,crc2,crc3;

extern SYSTEM_PARA para;
extern UINT8 stm32_input[],stm32_output[],spi_out_status[];
extern UINT8 laser_emergency_stop,emergency_restart_flag;
extern UINT32 valve_3_open_counter;
extern UINT8 find_laser_start_stop_flag;
extern UINT8 laser_already_begin_flag;
extern UINT8 rotated_cutter_running_flag;
extern UINT32 rotated_cutter_running_counter;
extern UINT8 power_on_laser_cool_flag;
extern UINT8 main_control_lock_flag,main_control_lock_setup;

extern UINT8 making_pen_actoin_flag,making_pen_status;
extern UINT8 rfid_debug_flag,uart1_sending_flag;

extern UINT16 cutter_angle_adjust,monitor_cutter_angle;
extern UINT8 checki10_follow_flag,checki10_action0,checki10_action1;
extern UINT8 one_step_run_flag;

extern SYSTEM_PARA5 para5;
#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
