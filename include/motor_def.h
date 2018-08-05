//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : eeprom.h
//  Description: eeprom driver external function define
//  Version    Date     Author    Description
//  0.01     03/07/07   pbb        created
//  ...
//  ... 
//  ...
//--------------------------------------------------------------------------------------

#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "typedef.h" //data type define
#include "common.h"  //External variables declaration
//--------------------------------------------------------------------------------------
// 	Constants definition
//--------------------------------------------------------------------------------------

//motor status
#define INIT		0
#define OPEN_START	1
#define CLOSE_RUN	2
#define STOP		3
#define WAITING		4

#define DC300 620

#define STOP_SPD  k43*10       
#define DEC_SPD   800
#define DEC1_SPD  k43*10   

#define PP_NUM 2
#define M_CODER 256
#define ENCODER (M_CODER<<2)

#define DEGREE_165	165*ENCODER/360
#define DEGREE_202	202*ENCODER/360
#define DEGREE_20	20*ENCODER/360

/*QEP*/

#define SPD_K (60*FX/M_CODER/8)

#define L 9
#define E 36   
#define R 35   

#define THETA_INI 597
#define LQ_Q 10   
#define CURRENT_Q 10   
#define KE_Q 8 
                                            
/*SPEED_FRQ*/
#define SAMPLES_BIT 3
#define SAMPLES ((1<<SAMPLES_BIT)-1)
#define RPM_K  ((60*1000*10*32/((M_CODER<<2)))*PWM_FREQ/10000)//(360*PWM_FREQ/10000)////  
#define RPM_Q  (SAMPLES_BIT+4)  
                                            
#define DTT_CNT (FX/1000000*3)
#define CARR_CNT (FX/(2*PWM_FREQ))

//******************   
#define BEGIN 1
#define END 0
#define FB 3
#define DT 2
#define AC 1
#define NC 0    
#define H_time 7
#define L_time 25   
#define LIMIT_TIME_AC 80    //the least time between 2 acc_tactic   
#define LIMIT_TIME_DT 30    //the least time between DT and the other two status 
#define AC_HOLD_TIME 4		//the time when switch status from ac to normal        
//****************** 

#define U_MAX 1385     		//300v

/*PI*/
#if DEBUG_MAIN_MOTOR
#define KPs_H motor_para[8]
#define KIs_H motor_para[9]	
		
#else
#define KPs_H 3//6//3 
#define KIs_H 4//9//4	
#endif

#define KPs_M 6        
#define KIs_M 8//10//8			
#define KPs_L 6 //8//6			
#define KIs_L 10			

#define KPs_up_L 6    		
#define KIs_up_L 4//8//4
#define KPs_up_H 6		
#define KIs_up_H 4

#define KPs_down_L 10    
#define KIs_down_L 8		
#define KPs_down_H 5    
#define KIs_down_H 5

#define KPs_trans_L 5    	
#define KIs_trans_L 10		
#define KPs_trans_H 8
#define KIs_trans_H 10

#define KPs_stop_0	8
#define KIs_stop_0	8

#define KPs_stop_1	8
#define KIs_stop_1	8
#define KPs_stop_2	8
#define KIs_stop_2	8
//case2 used for finding deadpoint 
#define KPs_deadpoint	10//no more big than this
#define KIs_deadpoint	20
#define KPs_stop motor_para[0]//15			
#define KIs_stop motor_para[1]//10
//case3,step1
#define STOP_KPp_1 motor_para[2]//15
#define STOP_KPs_1 motor_para[3]//5
//case3,step2
#define STOP_KPp_2 motor_para[4]//8
#define STOP_KPs_2 motor_para[5]//30

#define PID_SPD_MAX_P 9500 	//9200
#define PID_SPD_MAX_N 9000     //9000
//didt rangee
#define DERV 1000*motor_para[6] 	//9000

//chose the condition of using didt:,0 for use under 1000rpm,1 for never use
#define DIDT_SPEED_RANGE motor_para[7]	

//select the porportion of didt when cutting thread,the range is 0-4,as default is 4
#if DEBUG_MAIN_MOTOR
#define	DIDT_CUT_THREAD  2
#else
#define DIDT_CUT_THREAD motor_para[8]	
#endif
//last step for motor.spd_obj == 0
#define ANGLE_P 6*ENCODER/360
#define ANGLE_N -(6*ENCODER/360)


#define abs(z) ((z > 0)?(z):(-(z))) 
#define limit(a,b) a=(a>(b))?(b):a;a=(a<(-(b)))?(-(b)):a 

#endif

//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
