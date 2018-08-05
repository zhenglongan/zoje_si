//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : 
//  Description: 
//  Version    Date     Author    Description
//  0.01     21/02/06   liwenz    created
//  0.02     05/03/08   lm        modify
//  ...
//--------------------------------------------------------------------------------------

#ifndef STEPMOTOR_H
#define STEPMOTOR_H

extern void init_stepmotor_drv(void);
extern void config_stepmotor_drv(void);
extern void movestep_x(int x_data);
extern void movestep_y(int y_data);
extern void movestep_yj(int yj_data,unsigned int time);
extern void movestep_zx(int zx_data,unsigned int time);
extern void ready_quick(void);
extern void quickmove_y(int y_data);
extern void quickmove_x(INT32 x_data);
extern const UINT8 time_tab0[256];
extern const UINT8 time_tab1[256];
extern void ready_time(void);

extern void version_check(void); 
extern void nop_move_emergency(UINT16 x, UINT16 y);
extern UINT16 get_x_distance(void);
extern UINT16 get_y_distance(void);

extern void go_origin_rotated_cutter(void);
extern void rotated_cutter_single_next(void);
extern void rotated_cutter_single_stop(void);

extern void start_change_process(void);
extern void movestep_cs3(UINT16 command,INT16 x_data,UINT8 timer_need);
extern UINT8 bobbin_case_workflow1(void);
extern UINT8 find_a_bobbin_case(UINT8 full);
extern void go_origin_bobbin_case_arm(UINT8 pos);
extern void stepmotor_para(void);    //2012-5-22 add
extern void read_stepmotor_para(void);
extern UINT8 check_motion_done(void);
extern void send_stepmotor_end_drv(void) ;
extern void send_stepmotor_up_drv(void) ;
extern UINT16 check_DSP3_input(void);
extern void output_cs3(UINT8 x_data,UINT8 timer_need);
extern void setup_stepper_moter(void);
extern void process_stepper_cutter(void);
extern void go_origin_stepper_cutter(void);
extern void movestep_lct(int ct_data,UINT16 time);

extern void quickmove_x_process(INT32 time, INT32 data);
extern void quickmove_y_process(INT32 time, INT32 data);

extern void send_dsp_command(UINT8 port,UINT16 command);
extern void process_laser_cutter(void);

extern void movestep_x_y_both(int x_data,int y_data);

extern UINT8 get_bobbin_case_arm_org_status(void);
extern UINT8 check_autosewing_status(void);
//#if SUPPORT_UNIFY_DRIVER
extern void write_stepmotor_config_para(UINT8 port,UINT8 *pdata);
extern void read_stepmotor_config_para(UINT8 port);
extern UINT16 crc_calcu(UINT16 far *crc_in, UINT16 length, UINT16 init);
extern UINT16 read_stepmotor_curve_crc(UINT8 port);
extern UINT8 write_stepmotor_curve(UINT8 port,UINT8 *pdata);
//#endif

extern UINT8 requery_dsp1_status(void);
extern void SEND_DSP1_ONE_CODE( INT8 xx,INT8 yy);

extern void send_servo_parameter(UINT16 xspeed);
extern void send_dsp1_command2(UINT16 command);
extern void close_servo_parameter_function(void);

//多功能IO升级程序
extern void multipule_program_beginning(UINT8 port);
extern void send_multipule_program_data(UINT8 port);    
extern UINT16 read_multipule_program_status(UINT8 port);
extern void multipule_program_end(UINT8 port);  

extern void process_laser_offset_move(UINT8 dir);  
extern UINT16 get_YJORG_status(void);
extern UINT8 check_dsp2a_motion_done(void);
extern void set_rotated_cutter_speed(UINT16 speed);
#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------