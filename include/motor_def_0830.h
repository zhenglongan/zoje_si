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
#define SEL_11_10 K227
//motor status
#define INIT		0
#define OPEN_START	1
#define CLOSE_RUN	2
#define STOP		3
#define WAITING		4

#define DC300 620

#define START_SPEED 90            //442--90rpm
#define STOP_SPD 120//200
#define DEC_SPD  800

#define PP_NUM 2
#define M_CODER 256
#define POS_CONST_Q 10
#define ENCODER (M_CODER<<2)


#define POS_K ((M_CODER<<2)*32768/60/1000)
/*QEP*/
#define CAL_ANGLE 100//128                 //Encoder offset calibration

#define THETA_180DEG (M_CODER<<1)         
#define ADD_THETA (M_CODER*4*PP_NUM/60*10000/PWM_FREQ)   //Q15,6 Degree,1092,546
#define COUNT_S (10*PWM_FREQ/10000)

#define POS_CONST ((M_CODER<<2)*4096*8192/(60*1000*DECELERATION)+1)/2 
//#define STOP_OFFSET 80//(M_CODER>>2) 
#define STOP_OFFSET 50//(M_CODER>>2) 

#define POSITION_Q 10
#define POSITION (60*1000*(1<<(POSITION_Q-1))/(M_CODER<<2))
//#define POS_CONST_Q 10


#define ANGLE_CONST (1024*1024/360)

#define SPD_K (60*FX/M_CODER/8)

#define SECTOR0  0
#define SECTOR1  171 
#define SECTOR2  342 
#define SECTOR3  512
#define SECTOR4  683
#define SECTOR5  854



  
#define L0 18//13   //5    //电感Lq
#define E0 37//43   //36   //磁链系数   
#define R0 34//35   //20   //电阻

#define L 5
#define E 36   
#define R 20 


#define LQ_Q 10   
#define CURRENT_Q 10   
#define KE_Q 8    

                                            
/*SPEED_FRQ*/
#define SAMPLES_BIT 3
#define SAMPLES ((1<<SAMPLES_BIT)-1)
#define RPM_K  ((60*1000*10*32/((M_CODER<<2)))*PWM_FREQ/10000)//(360*PWM_FREQ/10000)////  
#define RPM_Q  (SAMPLES_BIT+4)  
#define RPM_T (RPM_K/(1<<(RPM_Q-1)))
                                             
                                             
                                            
#define DTT_CNT (FX/1000000*3)
#define CARR_CNT (FX/(2*PWM_FREQ))


#define START_I 500        //1000
#define DELTA_I 500        //1000
#define I_MAX_O 5000       //8000 3000

#define U_MAX 1385       //300v

/*PI*/
#define KPs_L0 5//10 //6	    // m_spd_n < 1100
#define KIs_L0 80//5 //5
#define KPs_M0 5 //6     // 1100 =< m_spd_n < 1500
#define KIs_M0 96 //8
#define KPs_H0 5//6 //3     // m_spd_n >= 1500
#define KIs_H0 80 //5

#define KPs_up_L0 40//5//5  //6    // m_spd_n < 1500
#define KIs_up_L0 64//3  //5
#define KPs_up_H0 40//4  //4	   // m_spd_n >= 1500
#define KIs_up_H0 64//2  //5    3500-- 7   5

#define KPs_down_L0 24//5 //10    // 800 < m_spd_n < 4000       8    10
#define KIs_down_L0 60//K14//16//4  //5     //5-7                        5     5  
#define KPs_down_H0 40//24//7  //8     // m_spd_n >= 4000           
#define KIs_down_H0 1//16//5  //7

#define KPs_trans_L0 6    // m_spd_n < 1300
#define KIs_trans_L0 40//128  //5
#define KPs_trans_M0 6    // m_spd_n < 1300
#define KIs_trans_M0 128  //5		 
#if (FACTORY_NUM == 200)
#define KPs_trans_H0 4	//7 // m_spd_n >= 1300
#else
#define KPs_trans_H0 4	//7 // m_spd_n >= 1300
#endif
#define KIs_trans_H0 128  //7  7 3500-- 7

#define KPs_L 6 //6	    // m_spd_n < 1100
#define KIs_L 5 //5
#define KPs_M 6 //6     // 1100 =< m_spd_n < 1500
#define KIs_M 8 //8
#define KPs_H 3 //3     // m_spd_n >= 1500
#define KIs_H 5 //5

#define KPs_up_L 6  //6    // m_spd_n < 1250
#define KIs_up_L 5  //5
#define KPs_up_H 4  //4	   // m_spd_n >= 1250
#define KIs_up_H 5  //5    3500-- 7   5

#define KPs_down_L 8 //10    // 800 < m_spd_n < 4000       8    10
#define KIs_down_L 5  //5     //5-7                        5     5  
#define KPs_down_H 8  //8     // m_spd_n >= 4000           
#define KIs_down_H 7  //7

#define KPs_trans_L 6  //6    // m_spd_n < 1300
#define KIs_trans_L 5  //5		 
#if (FACTORY_NUM == 200)
#define KPs_trans_H 6  //4	7 // m_spd_n >= 1300
#else
#define KPs_trans_H 4  //4	7 // m_spd_n >= 1300
#endif
#define KIs_trans_H 7  //7  7 3500-- 7

#define PID_SPD_MAX_P 9200  
#define PID_SPD_MAX_N 9200       

#define KPp 5  //5
#define KIp 5  //5
#define PID_POS_MAX   1000   //1000  

#define SPEED_MAX_P   10000
#define SPEED_MAX_N   3000


#define KIs_stop0 2//16//2    //15
#if (FACTORY_NUM == 160)
	#define KPs_stop0 20//40//20   //7     //5  ZOJE(SunSir)=40  JACK=20	
	#define KPs_stop31 5//20//5    //15    ZOJE(SunSir)=20   JAKE=5
#else
	#define KPs_stop0 40//40//20   //7     //5  ZOJE(SunSir)=40  JACK=20	
	#define KPs_stop31 20//20//5    //15    ZOJE(SunSir)=20   JAKE=5
#endif
#define KPp_stop31 11     //5	

#define KPp_stop32 28     //5	
#define KPs_stop32 35 //11    //15
 
#define abs(z) ((z > 0)?(z):(-(z))) 
#define limit(a,b) a=(a>(b))?(b):a;a=(a<(-(b)))?(-(b)):a 

#endif

//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
