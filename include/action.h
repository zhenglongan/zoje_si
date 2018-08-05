//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : 
//  Description: 
//  Version    Date     Author    Description
//  0.01     09/08/08   lm        created
//  ... 
//  ...
//--------------------------------------------------------------------------------------
#ifndef ACTION_H
#define ACTION_H

extern const INT16 angle_tab[361];
extern void go_origin_yj(void);
extern void go_origin_zx(void);
extern void go_origin_x(void);
extern void go_origin_ct(void);
extern void go_origin_y(void);
extern void go_yj(INT16 step,INT16 time);
extern void x_converse_check(void);
extern void foot_up(void);
extern void element_calaulate(void);
extern void element_data_latch(UINT8 ElementIndex);
extern void foot_down(void);
extern void inpress_up(void);
extern void inpress_down(INT16 low_position);
extern void inpress_to(INT16 a);
extern void inpress_to_forsingle(INT16 a);//09.06.05 wr add
extern void cut_thread(void);
extern void go_origin_allmotor(void);
extern void go_origin_xy(void);
extern void half_current(void);
extern void xy_test(void);
extern void inpress_to(INT16 a);
extern void find_dead_center(void);
extern void go_startpoint(void);
extern void check_data(UINT8 control_flag);
extern void calculate_angle(void);
extern void process_data(void);
extern void go_beginpoint(UINT8 FirstNopmoveFlag);
extern void conprocess_data(void);
extern void process_sewingtest_data(void);
extern void conprocess_sewingtest_data(void);
extern void go_sewingtest_beginpoint(void);
extern void back_endpoint(void);
extern void single_next(void);
extern void single_inpresser_next(void);
extern void single_thread_next(void);
extern void single_back(void);
extern void single_inpresser_back(void);
extern void single_thread_back(void);
extern void back_startpoint(void);
extern void single_end(void);
extern void single_inpresser_end(void);
extern void single_thread_end(void);
extern void single_start(void);
extern void single_inpresser_start(void);
extern void single_thread_start(void);
extern void single_stop(void);
extern void move_next(void);
extern void move_back(void);
extern void move_startpoint(void);
extern void course_next(void);
extern void course_inpresser_next(void);
extern void course_back(void);
extern void course_inpresser_back(void);
extern void course_stop(void);
extern void shift_12(void);
extern void shift_01(void);
extern void shift_03(void);
extern void shift_04(void);
extern void shift_06(void);
extern void shift_07(void);
extern void shift_09(void);
extern void shift_10(void);
extern void remove_12(void);
extern void remove_01(void);
extern void remove_03(void);
extern void remove_04(void);
extern void remove_06(void);
extern void remove_07(void);
extern void remove_09(void);
extern void remove_10(void);
extern void remove_stop(void);
extern void go_setoutpoint(void);
extern void turnoff_led(void);
extern void turnoff_buz(void);
extern void turnoff_ledbuz(void);
extern void turnon_led(void);
extern void turnon_buz(void);
extern void flash_led(void);
extern void flash_buz(void);
extern void emergency(void);
extern void para_confirm(void);
extern void sewing_stop(void);
extern void sewing_stop_of_savetension(void);
extern UINT8 detect_position(void);
extern void repeat_pattern(void);
extern void clamp_out(void);
extern void clamp_in(void);
extern void clamp_backstep1(void);
extern void clamp_backstep2(void);
extern void clamp_backstep3(void);
extern void go_manualpoint(void);
extern void reset_panel(void);
extern void initial_mainmotor(void);
extern void initial_stepmotor(void);
extern void move_xy(void);
extern void move_ct(void);
extern void zpl_process(void);
extern void go_commandpoint(INT32 commandpointcoorx,INT32 commandpointcoory);
extern void course_edit_continue_next(void); //10.09.27 wr add
extern void course_edit_continue_back(void); //10.09.27 wr add

extern void footer_procedure(void);

extern void footer_both_up(void);
extern void footer_both_down(void);

extern UINT8 check_footer_status(void);
extern void stretch_foot_out(void);
extern void stretch_foot_in(void);

//2012-5-18 add
extern void footer_procedure(void);  
extern void footer_both_up(void);
extern void footer_both_down(void);
//2012-6-7 add
extern void stretch_foot_out(void);
extern void stretch_foot_in(void);
//2013-7-29 add
extern void stapoint_calc(void);
extern void delay_process(void);

extern UINT32 Calculate_QuickMove_Time(UINT32 temp16_x,UINT32 temp16_y);
extern UINT32 Calculate_QuickMove_Time_x(UINT32 temp16_max);
extern void SewingReverse(void);
extern void special_sewing(UINT8 flag,UINT8 cnt,INT16 offset);
extern void zoom_in_one_stitch(UINT8 stitchs,UINT8 cflag);
extern void x_quick_way(INT16 quick_time, INT16 tempx_step);
extern void do_pat_point_add_one(void);
extern void do_pat_point_sub_one(void);

extern void set_openorclose_loop(UINT8 set);
extern const INT16 angle_tab[];
extern void movestep_ct(int ct_data,UINT16 time);

#if AUTO_DEBUG_MODE
extern void auto_debug_program(void);
extern void test_quickmove_program(void);
#endif 

extern UINT8 check_sewing_range(void);

extern void jump_to_begin(void);
extern const UINT16 inpress_tab2[];

extern void PBP_Line(UINT8 cnt);
extern void quickmove_z_process(INT32 quick_time, INT32 tempx_step);

extern void take_frame_from_one_side(UINT8 side);
extern void return_frame_back(UINT8 side);
extern const UINT8 MoveTime_Speed_750_x[];

extern void go_origin_yj(void);
extern UINT16 get_IORG_status(void);

extern void check_output_pattern_done(void);

extern void calculate_inpress_angle(INT16 speed);

#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------