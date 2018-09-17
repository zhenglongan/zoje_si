//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : typedef.h
//  Description: data type define
//  Version    Date     Author    Description
//  0.01     03/07/07   pbb        created
//  0.02     02/08/07   lm         modify
//  ... 
//  ...
//--------------------------------------------------------------------------------------
#ifndef TYPEDEF_H
#define TYPEDEF_H
//--------------------------------------------------------------------------------------      
//  type define
//--------------------------------------------------------------------------------------
typedef long INT32;
typedef unsigned long UINT32;
typedef short INT16;
typedef unsigned short UINT16;
typedef signed char INT8;
typedef unsigned char UINT8;

typedef struct  
{
  UINT8 status;      // system status
  UINT8 u24_val;     // vol of U24
  UINT16 error;      // system error
  INT16 uzk_val;     // vol of UZK
} SYS_STRU;

typedef struct  
{
  	UINT8 dir;         			// motor direction
  	UINT8 stop_flag;
	INT16 iq;          			// motor current
	INT16 iq_last;
	INT16 max_spd;     			// motor max speed
	INT16 min_spd;     			// motor max speed
	INT16 acc;         			// motor accelerate
	INT16 dec;         			// motor decelerate
	INT16 acc_curve;   			// motor accelerate cure type
  	INT16 spd;         			// motor feedback speed
  	INT16 angle;       			// motor angle
	INT16 angle_adjusted;
	INT16 l_angle;     			// motor angle history value
  	INT16 spd_obj;     			// motor object speed
  	INT16 spd_ref;     			// motor referense speed
	INT16 stop_angle;  			// stop posion
	INT16 stop_angle1;
	INT16 angle_hold;
} MOTOR_STRU;

typedef struct 
{
	UINT8 func;        // fuction code
	INT8 xstep;        // X 
	INT8 ystep;        // Y
} PATTERN_DATA;

typedef struct 
{
	UINT8 MachineType;
	UINT8 FatherVersion;
	UINT8 ChildVersion;
	UINT8 SVNVersion;
	UINT8 DSPNumber;
}STEPVERSION;

typedef struct
{
	UINT16 fun_ID;
	UINT8  code_ID;
	UINT8 test1;
	UINT8 test2;
	UINT8 test3;
	UINT8 test4;
}FAULT_DIAG;
//--------------------------------------------------------------------------------------      
//  1900a type define
//--------------------------------------------------------------------------------------
struct two_byte
{
	UINT8	byte2;
	UINT8	byte1;	
};

union TRANS
{
	struct	two_byte	byte;
	UINT16 	word;
};

union RECV
{
	struct	two_byte	byte;
	UINT16 	word;
};


typedef struct
{
	UINT16 DSP1_para_1F;		//1,2
	UINT16 DSP1_para_20;		//3,4
	UINT16 DSP1_para_21;		//5,6
	UINT16 DSP1_para_22;		//7,8
	UINT16 DSP1_para_23;		//9,10
	UINT16 DSP1_para_27;		//11,12
	UINT16 DSP1_para_28H;		//13,14
	UINT16 DSP1_para_28M1;		//15,16
	UINT16 DSP1_para_28M2;		//17,18
	UINT16 DSP1_para_28L;		//19,20
	
	UINT16 DSP2_para_1F;		//21,22
	UINT16 DSP2_para_20;		//23,24
	UINT16 DSP2_para_21;		//25,26
	UINT16 DSP2_para_22;		//27,28
	UINT16 DSP2_para_23;		//29,30

	UINT16 DSP2_para_27;		//31,32
	UINT16 DSP2_para_28H;		//33,34
	UINT16 DSP2_para_28M1;		//35,36
	UINT16 DSP2_para_28M2;		//37,38
	UINT16 DSP2_para_28L;		//39,40
	
	UINT8  dsp1A_half_current;	//41
	UINT8  dsp1B_half_current;	//42
	UINT8  dsp2A_half_current;	//43
	UINT8  dsp2B_half_current;	//44
		
	UINT8  platform_type;		//45,��������ʵ��ʹ��
	UINT8  mainmotor_type;      //46,��������ʵ��ʹ��
	UINT8  x_origin_mode;		//47
	UINT8  yj_org_direction;    //48,��������ʵ��ʹ��
	UINT8  Corner_deceleration_speed;//49����������ʵ��ʹ��
    UINT8  wipper_type;			//50,��������ʵ��ʹ��
	UINT8  x_sensor_open_level; //51����������ʵ��ʹ��
	UINT8  y_sensor_open_level;	//52����������ʵ��ʹ��
	UINT8  laser_function_enable;//53����������ʵ��ʹ��
	UINT8  dvb_open_level;		//54����������ʵ��ʹ��
	UINT8  last_8_speed;		//55����������ʵ��ʹ��
	UINT8  last_7_speed;		//56����������ʵ��ʹ��
	UINT8  last_6_speed;		//57����������ʵ��ʹ��
	UINT8  last_5_speed;		//58����������ʵ��ʹ��
	UINT8  last_4_speed;		//59����������ʵ��ʹ��
	UINT8  last_3_speed;		//60����������ʵ��ʹ��
	UINT8  last_2_speed;		//61����������ʵ��ʹ��
	UINT8  last_1_speed;		//62����������ʵ��ʹ��
	UINT8  dva_open_level;		//63����������ʵ��ʹ��
	
	UINT16  dsp1_step_crc;		//64,65����������ʵ��ʹ��
	UINT16  dsp2_step_crc;		//66,67����������ʵ��ʹ��
	
	UINT16 y_backward_dis;		//68,69����������ʵ��ʹ��
    UINT16 x_take_offset;		//70,71����������ʵ��ʹ��
	UINT16 x_take_offset2;		//72,73����������ʵ��ʹ��
	UINT16 left_barcode_position;	//74,75
	UINT16 right_barcode_position;	//76,77����������ʵ��ʹ��
	UINT16 catch_delay_time;	//78,79����������ʵ��ʹ��
	UINT16 y_barcode_position;	//80,81����������ʵ��ʹ��
	UINT16 blow_air_counter;	//82,83����������ʵ��ʹ��
	UINT16 cut_air_counter;		//84,85����������ʵ��ʹ��
	UINT8  yj_go_origin_enable;			//86����������ʵ��ʹ��
	UINT8  second_origin_footer_status; //87����������ʵ��ʹ��
	UINT8  go_special_position_mode;	//88
	
	UINT16  dsp3_step_crc;		//89,90
	UINT16  dsp4_step_crc;		//91,92
	UINT8   zx_org_direction;	//93
	
	UINT16 DSP3_para_1F;		//94,95
	UINT16 DSP3_para_20;		//96,97
	UINT16 DSP3_para_21;		//98,99
	UINT16 DSP3_para_22;		//100,101
	UINT16 DSP3_para_23;		//102,103
	UINT16 DSP3_para_27;		//104,105
	UINT16 DSP3_para_28H;		//106,107
	UINT16 DSP3_para_28M1;		//108,109
	UINT16 DSP3_para_28M2;		//110,111
	UINT16 DSP3_para_28L;		//112,113
	
	UINT8  dsp3A_half_current;	//114
	UINT8  dsp3B_half_current;	//115
	UINT8  dsp3A_current;		//116
	UINT8  dsp3B_current;		//117
	UINT8  dsp3_enable;			//118����������ʵ��ʹ��
	
	UINT8  dsp3a_motor_dir;		//119
	UINT8  dsp3b_motor_dir;		//120
	UINT8  dsp3a_sensor_dir;	//121����������ʵ��ʹ��	
	UINT8  dsp3b_sensor_dir;	//122����������ʵ��ʹ��
	
	UINT8  bobbin_platform_speed;//123
	 INT8  bobbin_shake_distance;//124
	UINT8  bobbin_shake_time;	 //125
	
	UINT8  rotate_cutter_delaytime1;//126,��ת�е��Ƕȵ������ǰ��ʱ
	UINT8  rotate_cutter_delaytime2;//127,��ת�е�����XY����ǰ��ʱ
	UINT8  rotate_cutter_movetime;//128,��ת�е�����XY����ʱ���޶�
	
	UINT8  rotate_cutter_working_mode;//129,��ת�е��Ĺ���ģʽ
	UINT8  rotate_cutter_detect_valve_down;//130,��ת�е������Ƿ�ʹ���½���ȫ������
	UINT8  x_motor_dir;			//131
	UINT8  y_motor_dir;			//132
	UINT8  zx_motor_dir;		//133
	
	UINT16 cutter_speed;		//134,135,��ת�е�����ٶ�
	UINT8  speed_limit_switch;  //136
	UINT8  speed_percent;		//137
	UINT8  one_key_run_function;//138
	UINT8  thread_break_backward_switch;//139
	UINT8  thread_break_backward_stitchs;//140
	UINT8  slow_start_mode;		//141
	UINT8  stitch1_speed;		//142
	UINT8  stitch2_speed;		//143
	UINT8  stitch3_speed;		//144
	UINT8  stitch4_speed;		//145
	UINT8  stitch5_speed;		//146
	UINT8  first_5_adjust;		//147
	
	UINT8  Corner_deceleration_speed1; //148 �յ�������
	UINT8  Corner_deceleration_speed2; //149 �յ�������
	UINT8  Corner_deceleration_speed3; //150 �յ��ڶ���
	UINT8  Corner_deceleration_speed4; //151 �յ���һ��
	
	UINT8  slow_start_stitchs;	//152
	UINT8  slow_start_speed[15];//153-167
	
	UINT8  adjust_mode_enable;  //168

	//2018-8-4
	//���ƿ�ʼʱ�Ƿ�����ǰ����ı���ѹ�������߶ȣ����ڶ����ȣ���׼�߶Ȳ��ģ�,�����������Ժ��Ϸ��ƣ�
	//�����ƺ���ʱ�����ǰһ�ξ��벼�ϱȽϱ�����������������ʱ��Ӧ����ѹ�Ÿ߶Ȳ���Ҫ̫�ߣ���ͻ�׼�߶Ȳ��䣩
	UINT8 start_sew_change_inpress_high_enable;//169  ʹ�ܿ��أ�55��ʾ����
	UINT8 start_sew_change_inpress_high_stitchs;//170 ���õ�����
	UINT8 start_sew_change_inpress_high_range;//171    ָ������ѹ���涯�߶�
	
}SYSTEM_PARA;

typedef struct
{
	UINT8 x_time_adjust[40];
	UINT8 x_angle_adjust[40];
	UINT8 y_time_adjust[40];
	UINT8 y_angle_adjust[40];
}SYSTEM_PARA5;

#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
