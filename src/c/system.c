//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : main.c
//  Description: Core program to control the sewing machine
//  Version    Date     Author    Description
//  ...
//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"         // M16C/62P special function register definitions
#include "..\..\include\typedef.h"        // Data type define
#include "..\..\include\common.h"         // Common constants definition
#include "..\..\include\variables.h"      // External variables declaration
#include "..\..\include\system.h"         // External function declaration
#include "..\..\include\motor.h"          // constants definition
#include "..\..\include\delay.h"          // delay time definition
#include "..\..\include\solenoid.h"       // solenoid driver definition
#include "..\..\include\communication.h"  // Communication function
#include "..\..\include\stepmotor.h"      // stepper motor function
#include "..\..\include\action.h"         // action function
#include "..\..\include\math.h"           // math library definition
#include "..\..\include\ctmotor.h"        // thread clamp motor function
#include "..\..\include\M_debug.h" 
#include "..\..\include\iic_bus_eeprom.h"

#define double_pedal 2
#define single_pedal 1

#if MOVING_DEBUG
void test_stepper_moving(void);
#endif

/**
  * @�������� �Ǻűʴ���
  
  ����+����+����+��ƫ�ƣ�+�Ǻű�+�Ǻű�+����λ��+���� 
  ����+����+����+��ƫ�ƣ�+�Ǻű�+����+�Ǻű� +����λ��+����
  ����+����+����+��ƫ�ƣ�+�Ǻű�+����+�Ǻű� +����λ��+����
  
  �����Ǻű���Ϊһ�飬��һ����ƫ�ƣ��ڶ�������ƫ�ƣ�����������֮�⣬��ҪӰ�쵽���ͼ�ͣ
*/
void marking_pen_stop(void)
{
	while( rec1_total_counter > 0 )
		   rec_com();
	tb4s = 0;
	MARKING_PEN = 0;
	laser_cutter_aciton_flag = 0;	
}
void process_marking_pen(UINT8 flag)
{
	INT32 allx_step_tmp,ally_step_tmp;
	UINT16 dly;
	UINT8 slow_flag ,action_flag ,i ,protect_flag,stop_status,first_flag,stitch_cnt;
	
#if SUPPORT_SEWING_MARKING_PEN	
  if( sys.status == RUN)
  {	
	//�ȴ���ƫ��
	if( (milling_first_move == 0)&&(pen_x_bios_offset!=0)||(pen_y_bios_offset!=0) )//ƫ�Ʋ�Ϊ0
	{
		allx_step_tmp = allx_step;
		ally_step_tmp = ally_step;
		go_commandpoint(pen_x_bios_offset + allx_step,pen_y_bios_offset + ally_step);
		allx_step = allx_step_tmp;
		ally_step = ally_step_tmp;
		milling_first_move = 1;
		MARKING_PEN = 1;
	}
	delay_ms(50);
	rec1_total_counter = 0;
	first_flag = 1;
	/*
	ʱ����24MHz ���Ǽ���24000000����ʱ1�� 24000��Ӧ1ms 100us => 2400
	1.5ms = 36000
	1-9: 1����� 500us  9��--4.5ms	
	
	*/
	
	dly = marking_speed;
	dly = dly*2400+9600;
	tb4 = dly;
	laser_cutter_aciton_flag = 0;
	//����ɨ�账��Ǻű�
	while(1)
	{
		if( sys.status == ERROR )
		    break;
		//��ͣ�Ĵ���	
		if( PAUSE == pause_active_level)//press sotp button when stepper motor moving
	  	{
			delay_ms(20);
			if( PAUSE == pause_active_level)
			{
				while( rec1_total_counter > 0 )
					rec_com();
				stitch_cnt = 0;
				tb4s = 0;     
				sys.status = READY;
				stop_status = 1;
			}
		}
		//��ͣ�Ĵ���
		if( stop_status == 1) 		 
		{
			stitch_cnt = 0;
			while( 1 )
			{
				 //ɨ��������ť
				  if( DVA == 0) 
				  {
					  delay_ms(20);
					  if( DVA == 0)
						{
						   sys.status = RUN;
						   stop_status = 0;
						   predit_shift = 0; 
						   break;
						}
				  }			
				  delay_ms(1);
				  //����ԭ�㴦��
				  if( origin_com == 1 )
				  {
						predit_shift = 0; 
						single_flag = 0;
						break;
				  }
				  //��ת
				  if(coor_com  == 1)
				  {
						predit_shift = 0;
						coor_com = 0;
				  }
				  if( PointShiftFlag == 1)
				  {
						PointShiftFlag = 0;
						predit_shift = 0 ;
				  }
				  //����	
  				  switch(single_flag)
			  	  {			
			   		case 1:							
						move_next();						  
						predit_shift = 0;  
						single_flag = 0;                  
						break;
			  		case 2:
						move_back(); 
						predit_shift = 0;
						single_flag = 0; 
						break;
			  		case 6:	 					    
						course_next();  						
						delay_ms(2);  
						if(PAUSE == pause_active_level)
						{
							delay_ms(10);
							if(PAUSE == pause_active_level)
							{				
								predit_shift = 0; 
								single_flag = 0;
						  	}	 
					  	}
						break;		
			  		case 7:	 	
					    course_back();
						delay_ms(2); 
						if(PAUSE == pause_active_level)
						{
							delay_ms(10);
							if(PAUSE == pause_active_level)
							{				
								predit_shift = 0; 
								single_flag = 0;
						  	}	 
					  	}
						break;			
			  		case 8:	 
						course_stop();       
						predit_shift = 0;   
						break;	
					}
				}//while 1
		}//stop_status
		
		//����������
		if(end_flag == 1)
		{
			if(inpress_flag == 0)  
			   inpress_up();
			break;
		}
		//����ԭ���
		if( origin_com == 1 )
			break;
			
		process_data();	//����һ������
		
		if( move_flag == 1  )	
		{
			move_flag = 0;
			nopmove_flag = 0;
			if( MARKING_PEN == 0)
			{
				MARKING_PEN = 1;
				delay_ms(200);
			}
			allx_step = allx_step + xstep_cou;                        
		  	ally_step = ally_step + ystep_cou;	
			if (first_flag == 1)//����
			{
				tb4 = dly; 
				tb4s = 1;
				laser_cutter_aciton_flag = 1;
				first_flag = 0;
			}			
			PBP_Line(stitch_cnt);
		}
		//���Լ���
		if( cut_flag == 1)
		    cut_flag = 0;
		//���ʽ���		
		if(	(making_pen_status == 4)&&(	FootRotateFlag == 1) )
		{
			 marking_pen_stop();
			 stitch_cnt = 0;
			 first_flag = 1;
			 if( (milling_first_move == 1)  &&( (pen_x_bios_offset!=0)||(pen_y_bios_offset!=0) ) )
			{ 
				 allx_step_tmp = allx_step;
				 ally_step_tmp = ally_step;			
				 go_commandpoint(allx_step-pen_x_bios_offset,ally_step- pen_y_bios_offset);	  
				 allx_step = allx_step_tmp;
				 ally_step = ally_step_tmp;	 
			}
			milling_first_move = 0;
			break;
		} 
		//���ʹ���
		if(nopmove_flag == 1)	
		{
				marking_pen_stop();
			 	stitch_cnt = 0;
			 	first_flag = 1;
				while( nopmove_flag == 1 )
				{
					do_pat_point_sub_one();
					sys.status = READY;
				    go_beginpoint(0); //�ֶο��ʹ���
					sys.status = RUN;
					process_data();	
					if(OutOfRange_flag == 1)
					{
					   end_flag = 1;
					   sys.status = ERROR;
					   sys.error = ERROR_15;
					   status_now = READY;
					   break;
					}
					if( nopmove_flag != 1)
					{
						do_pat_point_sub_one();
						break;
					}
				}
		 }	
		rec_com();	  
	}//while 1

	marking_pen_stop();
	stitch_cnt = 0;
	first_flag = 1;
	
	if( (milling_first_move == 1)  &&( (pen_x_bios_offset!=0)||(pen_y_bios_offset!=0) ) )
	{ 
		 allx_step_tmp = allx_step;
		 ally_step_tmp = ally_step;			
		 go_commandpoint(allx_step-pen_x_bios_offset,ally_step- pen_y_bios_offset);	  
		 allx_step = allx_step_tmp;
		 ally_step = ally_step_tmp;	 
	}
	milling_first_move = 0;
 }
 else
 #endif
 { 		
	if( marking_flag == 1)//�����Ǻűʹ�����
	{
	   if( marking_finish_flag ==0 )//��һ�������Ǻű���
	   {
		   if( (pen_x_bios_offset!=0)||(pen_y_bios_offset!=0) )//ƫ�Ʋ�Ϊ0
		   {
			   allx_step_tmp = allx_step;
		 	   ally_step_tmp = ally_step;
		       go_commandpoint(pen_x_bios_offset + allx_step,pen_y_bios_offset + ally_step);
			   allx_step = allx_step_tmp;
		 	   ally_step = ally_step_tmp;
		   }
		 #if SUPPORT_CS3_FUN
		   output_cs3(8,8); //x32.1
		 #else
		   MARKING_PEN ^= 1;
		 #endif
		   marking_flag = 0;
		   delay_ms(150 + k114);
		   marking_finish_flag = 1;
	   }
	   else//�ڶ��������Ǻű���
	   {
		 #if SUPPORT_CS3_FUN
		   output_cs3(8,0);
		 #else

		   MARKING_PEN ^= 1;
		 #endif
		   marking_flag = 0;
		   delay_ms(150+k114);
		   marking_finish_flag = 0;
		   if( (pen_x_bios_offset!=0)||(pen_y_bios_offset!=0) )
		   {
			  allx_step_tmp = allx_step;
		 	  ally_step_tmp = ally_step;
		      go_commandpoint(allx_step - pen_x_bios_offset ,ally_step - pen_y_bios_offset);
			  allx_step = allx_step_tmp;
		 	  ally_step = ally_step_tmp;
		   }
	   }
	}
  }
}
/**
  * @�������� ��ȡʶ���ģ���
  * @����      formwork_identify_device
  *     @arg 0: ��ʾ������ʶ��ʽ
  *     @arg 1: ��ʾ����ʶ��ʽ
  *
  * @����ֵ ʶ���ģ���
  *     @arg 0:     ��Чģ��
  *     @arg 1~999: ��Чģ��
  * 
  * 8 ���������汾
    INPUT 1 = P2.2   PORG
	INPUT 2 = P2.4   CORG
	INPUT 3 = P2.5   CSENS
	INPUT 4 = P2.3   PSENS
	INPUT 5 = P0.7   ADTCSM
	INPUT 6 = P10.4  SENSOR6
	INPUT 7 = P10.5  SENSOR7
	INPUT 8 = P0.6   SENSOR8
  */
UINT16 identify_pattern(void)
{
	UINT16 tmp_pattern;
	tmp_pattern =0;
	if(formwork_identify_device == 0) 
	{
		if(PORG == 1)
			  tmp_pattern |= 0x01;//����1
		if(PSENS == 1)
			  tmp_pattern |= 0x02;//����2
		if(CSENS == 1)
			  tmp_pattern |= 0x04;//����3
		if(CORG == 1)
			  tmp_pattern |= 0x08;//����4
		if(ADTCSM )
			  tmp_pattern |= 0x10;//����5
	}
	else if(formwork_identify_device == 1) 
	{
		if(PORG == 1)
			  tmp_pattern |= 0x01;//����1
		if(CORG == 1)
			  tmp_pattern |= 0x02;//����2
		if(CSENS == 1)
			  tmp_pattern |= 0x04;//����3
		if(PSENS == 1)
			  tmp_pattern |= 0x08;//����4
		if(ADTCSM )
	 		  tmp_pattern |= 0x10;//����5
		if(SENSOR6 == 1)
			  tmp_pattern |= 0x20;//����6
		if(SENSOR7 == 1)
			  tmp_pattern |= 0x40;//����7
		if(SENSOR8 )
	 		  tmp_pattern |= 0x80;//����8
	    		  
	}
	else if(formwork_identify_device >= 2)	
	{
	 	 tmp_pattern = serail_number;
	}
	return tmp_pattern;

}

/**
  * @��������  ���߶�������
  * @����      
  * @����ֵ 
  *
  * @����ʹ�õ�ȫ�ֱ��������
  *     @k43 --�����ٶ�
  *     @k95 --���߶����Ƕ�΢��
  *     @u09 --���߽Ƕ�΢��
  *     @u42 --���߹��ܿ��� 0-�� 1-�ر�
  *     @k03 --����������   0-��е 1-����
  *     @u42 --ͣ�����Ƿ�ͣ�������� 0-��λ�� 1-������
  *     @u46 --����������   0-�ر� 1-��
  *
  * @��Ӱ���ȫ�ֱ��� 
  */
  

#if SPECIAL_MOTOR_CUTTER

void trim_action(void)
{
		INT16 temp16; 
		//1.0  ����ת�ٽ��������ٶ���
		motor.dir = 0;
		motor.spd_obj = k43*10;  
		if( SUPPORT_CS3_FUN == 1)
			output_cs3(0x20,0x20);
		else
  			CUTTER_HOLDING_PART = 1;
		
		#if FOLLOW_INPRESS_FUN_ENABLE
		temp16 = motor.angle_adjusted;
        while( temp16 <= angle_tab[50] ) 
	    { 
        	rec_com();
			temp16 = motor.angle_adjusted;
		}
		if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
	    {
			movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
			inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
		}
		#endif
		
		//2.0  �ȴ����ᵽ����߽Ƕȣ����˿���
	    temp16 = motor.angle_adjusted;
        while( temp16 <= angle_tab[280-8*k95] ) 
	    { 
        	rec_com();
			if(sys.status == POWEROFF)   
			      return;
        	temp16 = motor.angle_adjusted;          
        }   	    
        	      		 		   
		process_flag = 0;
		if(u46 == 0)
		{
		    movestep_ct(44,k114);//4
		    SNT_H = 1;    
			L_AIR = 1;
		 }
		 //3.0 �򿪼����Ժ����ή�ٵ�100
		 motor.spd_obj = 100;
		 
		 //4.0 �ȴ����߽Ƕȣ���������
		 temp16 = motor.angle_adjusted;
         while( temp16 <= angle_tab[300-8*u09] )
		 { 
    	     rec_com();
			 if(sys.status == POWEROFF)   
			      return;
	         temp16 = motor.angle_adjusted;
         }
		 
		 if(k03 == 0)
         {
	         da0 = 255;
		     DAActionFlag=1;//����1��ǿ�ƹرձ�������
		     DAActionCounter =0;
         }

		 sewing_stop();	
		 if(u46 == 0)
		 {
		 	delay_ms(k112);//40 k112
		 	movestep_ct(-40,cutter_syn_delay);//3
			delay_ms(k113);
		 }
		 	 
		 //SNT_H = 1; 
	  	 //FW = 1;
		
		 while(motor.stop_flag == 0)
		 {
			rec_com();
			if(sys.status == POWEROFF) 
			      return;
		 }
		 //FW = 0;
	  	 //SNT_H = 0; 
		  //6.0  ͣ�Ⱥ󣬹رռ��ߡ������ź�			
		  if(u46 == 0)
		  {		   
			  da0 = 0;
			  L_AIR = 0;
			  SNT_H = 0;
		  }
		  //7.0 ���Ҫ��ͣ�������㣬������Ӧ�Ĵ���
      	  if(u42 == 1)
		  {
			  delay_ms(155);
		      find_dead_point();
		  }
		  //8.0  ִ�в��߶���

		  if(blow_air_enable == 1)
		  {
			  if( k171 ==1 )
			  	BLOW_AIR3 = 1;
			  else
			  	BLOW_AIR2 = 1;
			  
			  blow_air_counter = (UINT16)k166*100; 
			  blow_air_flag = 1;
		  }
		  else
		  {
			  blow_air_flag=0;
		  }
	     delay_ms(100);//35
		 if( SUPPORT_CS3_FUN == 1)
			 output_cs3(0x20,0x20);
		 else
 		 	 CUTTER_HOLDING_PART = 0;
		 delay_ms(32);
		 if(u46 == 0)
			go_origin_ct();

		  while(motor.stop_flag == 0)
		  {
			  rec_com();
			  if( sys.status == POWEROFF) 
			      return;
		  }    
}
#elif NEW_CUT_MODE

void trim_action(void)
{
		INT16 temp16; 
		UINT8 flag;
		//1.0  ����ת�ٽ��������ٶ���
		motor.dir = 0;
		motor.spd_obj = k43*10;  
		
		#if FOLLOW_INPRESS_FUN_ENABLE
		temp16 = motor.angle_adjusted;
        while( temp16 <= angle_tab[50] ) 
	    { 
        	rec_com();
			temp16 = motor.angle_adjusted;
		}
		if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
	    {
			movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
			inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
		}
		#endif
		
		//2.0  �ȴ����ᵽ����߽Ƕȣ����˿���
	    temp16 = motor.angle_adjusted;
        while( temp16 <= angle_tab[240-8*k95] ) 
	    { 
        	rec_com();
			if(sys.status == POWEROFF)   
			      return;
        	temp16 = motor.angle_adjusted;          
        }            	      		 		   
		process_flag = 0;
		if(u46 == 0)
		{
		   #if SECOND_GENERATION_PLATFORM 
			   FL = 1;
			   FL_pwm_counter = 0;
			   FL_pwm_period = 400;//(UINT16)k113*10;
			   FL_pwm_action_flag = 1;
		   #elif  CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
			   SNT_H = 1; 
			   FA = 1;		   
		   #else
			   SNT_H = 1;    
			   L_AIR = 1;
		   #endif		   
		   cutter_protect_counter = 0;
		   cutter_protect_flag = 1;
		 }
		 //3.0 �򿪼����Ժ����ή�ٵ�100
		 #if SECOND_GENERATION_PLATFORM || CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
		 #else
		 motor.spd_obj = 100;
		 #endif
		 //4.0 �ȴ����߽Ƕȣ���������
		 temp16 = motor.angle_adjusted;
         while( temp16 <= angle_tab[300-8*u09] )
		 { 
    	     rec_com();
			 if(sys.status == POWEROFF)   
			      return;
	         temp16 = motor.angle_adjusted;
			 
         }
		 if(k03 == 0)
         {
			 da0 = 255;
		     DAActionFlag=1;//����1��ǿ�ƹرձ�������
		     DAActionCounter =0;
         }
		 //5.0 ����ͣ���Ƕȣ����ȴ�����ͣ��
		 sewing_stop();	
		 
		 temp16 = motor.angle_adjusted;
		 while( temp16 > angle_tab[100] )//�ȴ��µ�һȦ
		 {
				temp16 = motor.angle_adjusted;
				if( motor.stop_flag == 1)
					    break;
		 }
		 flag = 1;
		 while( motor.stop_flag == 0)
		 {	
				temp16 = motor.angle_adjusted;				
				if( (temp16 >= 40 )&&(flag == 1) )
				{
					if( (u46 == 0)&&(u51 ==1) )
					{
						flag = 0;
						#if FW_TYPE_SOLENOID_VALVE || CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
						  	  #if SECOND_GENERATION_PLATFORM
								FK_OFF = 1;
							  #else
							    FR_ON = 1;
							  #endif		  
						#else
							SNT_H = 1;        // 24V to 33V
						  	FW = 1;
						#endif
					}					
				}
				if( temp16 >= k61 - 14 )//150=>53
					break;
		 }		 
		 		 
		 //6.0  ͣ�Ⱥ󣬹رռ��ߡ������ź�			
		  if(u46 == 0)
		  {		   
			  da0 = 0;
			  //delay_ms(55);
			  #if SECOND_GENERATION_PLATFORM 
			  FL = 0;
			  FL_pwm_action_flag = 0;
			  FL = 0;
			  #elif CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
			  FA = 0;
			  SNT_H = 0;
			  #else
			  L_AIR = 0;
			  SNT_H = 0;
			  #endif
		  }

		  //7.0 ���Ҫ��ͣ�������㣬������Ӧ�Ĵ���
      	  if(u42 == 1)
		  {
			  delay_ms(155);
		      find_dead_point();
		  }
		  //8.0  ִ�в��߶���

		  if(blow_air_enable == 1)
		  {
			  if( k171 ==1 )
			  	BLOW_AIR3 = 1;
			  else
			  	BLOW_AIR2 = 1;
			  
			  blow_air_counter = (UINT16)k166*100; 
			  blow_air_flag = 1;
		  }
		  else
		  {
			  blow_air_flag=0;
		  }
		  
		  if( u46 == 0 )       
		 	  fw_solenoid();
		  while( motor.stop_flag == 0)
		  	rec_com();
		  start_to_speed_down = 0;  
		  cutter_speed_done_flag = 0;
		  
}

#else

void trim_action(void)
{
		INT16 temp16; 
		//1.0  ����ת�ٽ��������ٶ���
		motor.dir = 0;
		motor.spd_obj = k43*10;  
		
		#if FOLLOW_INPRESS_FUN_ENABLE
		if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
	    {
			movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
			inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
		}
		#endif
		
		//2.0  �ȴ����ᵽ����߽Ƕȣ����˿���
	    temp16 = motor.angle_adjusted;
        while( temp16 <= angle_tab[240-8*k95] ) 
	    { 
        	rec_com();
			if(sys.status == POWEROFF)   
			      return;
        	temp16 = motor.angle_adjusted;          
        }            	      		 		   
		process_flag = 0;
		if(u46 == 0)
		{
		   #if SECOND_GENERATION_PLATFORM 
		   FL = 1;
		   FL_pwm_counter = 0;
		   FL_pwm_period = 400;//(UINT16)k113*10;
		   FL_pwm_action_flag = 1;
		   #elif  CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
		   SNT_H = 1; 
		   FA = 1;		
		   #else 
		   SNT_H = 1;    
		   L_AIR = 1;
		   #endif		   
		   cutter_protect_counter = 0;
		   cutter_protect_flag = 1;
		 }
		 //3.0 �򿪼����Ժ����ή�ٵ�100
		 #if SECOND_GENERATION_PLATFORM || CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
		 #else
		 motor.spd_obj = 100;
		 #endif
		 //4.0 �ȴ����߽Ƕȣ���������
		 temp16 = motor.angle_adjusted;
         while( temp16 <= angle_tab[300-8*u09] )
		 { 
    	     rec_com();
			 if(sys.status == POWEROFF)   
			      return;
	         temp16 = motor.angle_adjusted;
         }
		 if(k03 == 0)
         {
			 da0 = 255;
		     DAActionFlag=1;//����1��ǿ�ƹرձ�������
		     DAActionCounter =0;
         }
		 //5.0 ����ͣ���Ƕȣ����ȴ�����ͣ��
		 sewing_stop();	
		 temp16 = motor.angle_adjusted;
		 #if SECOND_GENERATION_PLATFORM || CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
		 
		 while(motor.stop_flag == 0)
		 {
			rec_com();
			if(sys.status == POWEROFF) 
			      return;
		 }
		 #else
         while( temp16 > angle_tab[100] )
		 { 
    	     rec_com();
			 if(sys.status == POWEROFF)   
			      return;
	         temp16 = motor.angle_adjusted;
         }
   		 while( temp16 < k61-14 )
		 { 
    	     rec_com();
			 if(sys.status == POWEROFF)   
			      return;
	         temp16 = motor.angle_adjusted;
			 if(motor.stop_flag == 1)
			   break;
         }
   	    #endif
		 
		 //6.0  ͣ�Ⱥ󣬹رռ��ߡ������ź�			
		  if(u46 == 0)
		  {		   
			  da0 = 0;
			  //delay_ms(55);
			  #if SECOND_GENERATION_PLATFORM 
			  FL = 0;
			  FL_pwm_action_flag = 0;
			  delay_ms(1);
			  FL = 0;
			  #elif CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
			  FA = 0;
			  SNT_H = 0;
			  #else
			  L_AIR = 0;
			  SNT_H = 0;
			  #endif
		  }
		  /*
		  while(motor.stop_flag == 0)
		  {
			rec_com();
			if(sys.status == POWEROFF) 
			      return;
		  } 
		  */ 
		  //7.0 ���Ҫ��ͣ�������㣬������Ӧ�Ĵ���
      	  if(u42 == 1)
		  {
			  delay_ms(155);
		      find_dead_point();
		  }
		  //8.0  ִ�в��߶���

		  if(blow_air_enable == 1)
		  {
			  if( k171 ==1 )
			  	BLOW_AIR3 = 1;
			  else
			  	BLOW_AIR2 = 1;
			  
			  blow_air_counter = (UINT16)k166*100; 
			  blow_air_flag = 1;
		  }
		  else
		  {
			  blow_air_flag=0;
		  }
		  //#endif
		  if(u46 == 0)
        	  fw_solenoid();  
		  start_to_speed_down = 0;  
		  cutter_speed_done_flag = 0;
		  while(motor.stop_flag == 0)
		  {
			rec_com();
			if(sys.status == POWEROFF) 
			      return;
		  } 
}

#endif

void process_nop_move_pause(UINT8 direction)
{
	UINT32 i,temp16_x,temp16_y,temp16_max,quick_time,temp32;
	INT32 tempx_step,tempy_step;

	if( direction == 2)//��Ҫ�ӿ���ֹͣλ�÷��ص����͵�ԭ���
	{
		//1.0 ����Ҫ���˶��پ���
		temp16_y = fabsm(read_step_y);
		temp16_x = fabsm(read_step_x);
		tempx_step = read_step_x;
		tempy_step = read_step_y;
		//2.0 �ָ����Ļ���ָ�롢���ꡢ����������
		pat_point = last_pattern_point;
		allx_step = last_allx_step;
		ally_step = last_ally_step;
		pat_buff_total_counter = bakeup_total_counter;
	}
	else //�ӿ���ֹͣλ����ǰ�ƶ�
	{
		//1.0 ���㻹�ж��پ���û��
		temp16_y = fabsm(nop_move_remainy);
		temp16_x = fabsm(nop_move_remainx);
		tempx_step = -nop_move_remainx;
		tempy_step = -nop_move_remainy;
		
		//2.0 ������õ���ͷ�������ӹ̣��ָ������������ָ�롢���ꡢ������
			pat_point = target_pat_point;
    		allx_step = target_allx_step;
			ally_step = target_ally_step;
			pat_buff_total_counter = target_total_counter;
	}
	//3.0 ��Ҫ�߾����ǲ��ÿ��� ���� ��ͨ����Э��
	if( temp16_x > temp16_y)
  	{
  		temp16_max = temp16_x;
  	}
  	else
  	{
  		temp16_max = temp16_y;
  	}

   	if( temp16_max > 255)
	{
		//quick_time = Calculate_QuickMove_Time(temp16_max);
		quick_time = Calculate_QuickMove_Time(temp16_x,temp16_y);
	}
	else if(temp16_max <= 255 && temp16_max > 0)
	{
		quick_time = 62;
	}

	//4.0 ִ��������X��Y�ᶯ��
	if(temp16_y < 255)
	{
	  	timer_y = 62;
		movestep_y(tempy_step);
	}
	else
	{
		quickmove_y_process(quick_time,tempy_step);		
	}
	delay_ms(1);
 
	if(temp16_x < 255)
	{ 
	  	timer_x = 62;
		movestep_x(tempx_step);
	}
	else
	{
		quickmove_x_process(quick_time,tempx_step);		
	}
	//5.0  ��ʱ�ȴ���ܵĶ������ 
	if(temp16_max < 255)
 	    delay_ms(70);
	else
	{
		delay_ms(quick_time+68);
		#if STEPPER_WAITTING_PROTOCOL
        for(i=0;i<quick_time;i++)
		{
			delay_ms(1);
			if( check_motion_done() )
			   break;
		}
		#endif
	}
	nop_move_pause_flag = 0;
}




/**
  * @��������  ����״̬����
  * @����      
  * @����ֵ 
  *
  * @����ʹ�õ�ȫ�ֱ��������
  *     @inpress_com  --���������ѹ������ָ����˻������ģ�
  *     @inpress_flag --��ѹ��״̬ 0-���µ� 1-̧����
  *     @power_on_ready--k137 �����Ƿ�ֱ�ӽ���ɷ���״̬
  *     @
  *     @
  *
  * @��Ӱ���ȫ�ֱ��� 
  */
void free_status(void)
{	
	UINT8 temp8;
	
    need_backward_sewing = 0; 
	pattern_change_flag = 0;
	if( release_tension_value_flag == 1)
	{
		#if DSP3_CUTER_DRIVER
		output_cs3(1,0);//port0
		#endif
		release_tension_value_flag = 0;
	}
	#if INSTALLMENT	
	if( main_control_lock_setup == 1)
	{
		write_par(0,main_control_lock_flag);  		
		main_control_lock_setup = 0;
	}
	#endif	

	#if SUPPORT_UNIFY_DRIVER
	if( wirte_stepermotor_para_flag != 0)
	{
		write_stepmotor_config_para(debug_dsp_flag+1,svpara_disp_buf);
		wirte_stepermotor_para_flag = 0;		
	}
	#endif
	
	#if ENABLE_CONFIG_PARA == 1
	if( write_eeprom_para_flag >= 1)
    {
		  if( write_eeprom_para_flag == 1)
		  {
		    	write_para_group(100,svpara_disp_buf,205);
				delay_ms(300);
		    	restore_para_from_eeprom();
		  }
		  else if( write_eeprom_para_flag == 2)
		  {
				write_para_group(400,svpara_disp_buf,205);
				delay_ms(300);
				//app_GetProgramFromEeprom();
		  }
		  else
		  {
			  write_para_group(700,svpara_disp_buf,205);
			  delay_ms(300);
			  get_para_from_eeprom();
		  }
		  write_eeprom_para_flag = 0;	
		  SUM = 1;
		  delay_ms(100);
		  SUM = 0;
		  predit_shift =0;
    }
    #endif
	if((svpara_trans_flag == 1)&&(SEND_SERVO_PARA == 1))
	{
		stepmotor_para();
		svpara_trans_flag = 0;
		set_func_code_info(FREE,1,0,0);
	}
	
	#if ENABLE_RFID_FUNCTION
	
	if(auto_function_flag == 1)
	{
		if((formwork_identify_device == 3)&&(rc522_control_falg==1))//RFID
		{	
			SUM =1;
			temp8 = RFID_SCAN();
	        rfid_wr_ret();
			rc522_control_falg = 0;
		}
	}
	#endif
#if	MACHINE_900_BOBBIN_DEBUG_MODE

#else
	//1.0 ��Ӧ��ѹ�Ŷ���ָ��
  	switch(inpress_com)
  	{
  		case 0:
  		 	if(inpress_flag == 1)
  	     	{
  	       		inpress_down(inpress_high_hole); 
  	      	}
  	     	break;
  	         
    	case 1:                         
    	  	if(inpress_flag == 0)
  	      	{
				if( motorconfig_flag == 1 )
				{
					temp8 = detect_position();	
		    		if(temp8 == OUT)    
		      		{
						find_dead_center();
		      		} 
				}
 				if( inpress_position == 0)
					go_origin_zx();
				else
					inpress_up();
				if(foot_com == 1)
				{
					delay_ms(30);
				}
  	      	}
  	      	break;  
  		default: break;
  	}
    //2.0 ��Ӧ��ѹ�Ŷ���ָ��
  	switch(foot_com)
  	{
  		case 0:          
  			if(foot_flag == 1)     		
  	  		{
  	    		footer_both_down();  
  	    	}
  	    	break;  	         
    	case 1:                   
    	  	if(foot_flag == 0)     		
  	      	{
  	       		footer_both_up();       
  	      	}
  	      	break;		
   		default: break;                   	         	
  	}
	//3.0 ɨ���̤�壬ִ��ѹ����������
	if(DVB == 0)           				
	{
		delay_ms(10);
		if(DVB == 0 && DVBLastState == 1)  
		{	
			 if(foot_flag == 0)
			 {
				foot_up();
			 }
			 else
			 {
		 	    foot_down();
			 }
		}
	}
	
	DVBLastState = DVB;
	DVALastState = DVA;
	
	if(pedal_state == 1)
	{
		delay_ms(10);
		if(pedal_state == 1)
		{
			pedal_last_state = 1;
		}
	}
	//4.0 ��ͣ����״̬ɨ��
  	if( (PAUSE == pause_active_level)&&(power_on_ready ==0) )
	{
		delay_ms(10);                           
		if(PAUSE == pause_active_level)
		{				
			status_now = sys.status;
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
      		sys.error = ERROR_19;  //��ͣ���ز�������λ��
	  	}
	 
  	}
#endif 
    predit_shift = 0;
 	if(StatusChangeLatch != FREE) 
	{
		sys.status = StatusChangeLatch;
		set_func_code_info(FREE,2,0,0);
	}
}

/**
  * @��������  ׼��״̬������
  * @����      
  * @����ֵ 
  *
  * @����ʹ�õ�ȫ�ֱ��������
  *     @
  *     @
  *
  * @��Ӱ���ȫ�ֱ��� 
  */
void ready_status(void)
{   
	UINT8 temp8;
	UINT16 temp16;	
	INT32 allx_step_tmp,ally_step_tmp;
	#if INSTALLMENT	
	if( main_control_lock_setup == 1)
	{
		write_par(0,main_control_lock_flag);  		
		main_control_lock_setup = 0;
	}
	#endif	
	//����ʶ����õ����Э�飬����֪ͨ���ؿ���ʶ���»���
	check_output_pattern_done();
	
	#if CHANGE_DOUBLE_FRAMEWORK
	if( current_running_flag == 0)//ɨ�赽�����룬�����Ѿ�������
	{
			if( left_quest_running == 1)//�������
			{			
				current_running_flag = 1;
				delay_ms(100);
				take_frame_from_one_side(1);//�������ģ�����		
				SUM = 0;
				autosewing_allset_flag = 1;		
			}
			else if(right_quest_running == 1)
			{				
				current_running_flag = 2;
				delay_ms(100);
				take_frame_from_one_side(2);//���ұ���ģ�����	
				SUM = 0;	
				autosewing_allset_flag = 1;			
			}	
	}
	else
	{
	
		if( autosewing_allow_working == 1  )
		{
			if( nop_move_pause_flag ==1)
		    	process_nop_move_pause(1);
			if(nop_move_pause_flag ==0)
			{
				go_setoutpoint();
				pre_run_conditioning();
			}
			if( sys.status ==RUN )
			{
			    autosewing_allow_working = 0;
				new_pattern_done = 0;
			}
			return;				
		}	
	}
	#endif
	
	#if AUTO_CHANGE_PATTERN_FUNCTION == 0

		#if DOUBLE_X_60MOTOR	
			if( oil_empty_alarm_enable == 1)
			{
				if(OIL_EMPTY_DETECT == 1)
				{
					status_now = READY;
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
			    	sys.error = ERROR_80;
				}
			}
		#endif
	#endif
	
	#if DSP3_CUTER_DRIVER
	if( (indraft_control_counter==0) &&( indraft_control_flag == 1) )
	{
		indraft_control_flag = 0;
		output_cs3(4,0);
	}
	#endif
	
	if ( alarm_output_enable >= 2)
	{
		#if DISABLE_FUN 
		YELLOW_ALARM_LED = 1;
		#endif
		RED_ALARM_LED = 0;
		GREEN_ALALRM_LED = 0;
	}
	if( motorconfig_flag == 0 )//��������λ
  	{
  		initial_mainmotor();		
	}	
  	if(sys.status == ERROR)
  	{
	  	predit_shift = 0;
  	  	return;
  	}
	
	#if FUNCTION_AUTOSEWING && AUTOSEWING_DETECTOR
	/*
	����������
	*/
	if( autosewing_control_flag == 1)
	{
		if( check_autosewing_status())
		{
			if(autosewing_switch_last_status == 0)
			{
				delay_ms(50);
				if( check_autosewing_status())
				{					
					autosewing_switch_last_status = 1;
					delay_ms(100);
					foot_down();
					delay_ms(100);					
					if( (ADTCSM)&&(PSENS ==1) )
					{
						autosewing_allset_flag = 1;
					}
					else
					{
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
			    		sys.error = ERROR_93;						
					}
					#if AUTO_CHANGE_PATTERN_FUNCTION
					AUTO_FIRST_ASK_FRAMEWORK = 0;//��е���Ѿ���λ�����������Ϣȡ��
					#endif
				}
			}
		}
		else 
		{
			autosewing_switch_last_status = 0;
			
		}
	}
	#endif
	
	//2.0 ִ���Ƿ��л���������ģ�����ͣ��ԭ��ģ�����ͣ������
	if ( ready_go_setout_com == 1)
	{
		ready_go_setout_com = 0;
		predit_shift = 0;
	}
	//3.0 ��Ӧ�����ϵļ��߰�ť���ܣ�ԭ�ض������
	if(cut_test_flag == 1)
	{
			if( (u35==0)&&(clamp_flag== 1) )  //��ץ�߲������
			{
				go_origin_ct();	
			}
			cut_test_flag = 0;
			/*
			���߶���֮ǰ��Ҫȷ��һ���Ƿ���й����ͼ�ͣ����ֹͣ��ģ���м䣬���߻�����������롣
			*/
			if( (nop_move_pause_flag ==0 )&&(finish_nopmove_pause_flag ==0) )
			{
				if((foot_half_flag == 1) || (foot_flag == 1))
				{
			    	footer_both_down();
				}
				delay_ms(100);
				
				inpress_down(inpress_high);
		 		delay_ms(50);
				
			 	trim_action();
				delay_ms(100);
				
				inpress_up();
				delay_ms(100);
			}
			predit_shift = 0;
			/*
			�����ץ�߹��ܣ�������ɺ�ץ�߻�����Ҫ�����ȥ
			*/
			if( (clamp_flag == 0)&&(u35==0))
    		{
    			go_origin_ct();
				delay_ms(20);
    			clamp_out();
    			clamp_stepflag = 1;
          		movect_angle = 800;
    		}
	}
    //4.0 ɨ�輱ͣ���ص�״̬����ֹ���ż�ͣ��������״̬
	if (PAUSE == pause_active_level)
	{
		delay_ms(10);
		if(PAUSE == pause_active_level)
		{
			status_now = sys.status;
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
      		sys.error = ERROR_19;
			#if AUTO_CHANGE_PATTERN_FUNCTION
			AUTO_FIRST_ASK_FRAMEWORK = 0;
			#endif
			if(ROTATED_CUTTER == 1)
			{
				if ( (milling_cutter_action_flag ==1)&&(rotated_cutter_enable ==1) )
					 rotated_cutter_single_stop();
			}
	  	}
  	}
	//5.0 �Է��������Ӧ
	if( single_flag !=0 )
	{
		return_from_setout = 1;				//ֻҪ�Է�����ͱ�ʾ��������Ѿ���ʼ�����ˣ�����������в���ɨ��ģ��
		need_action_once = 0;
		if( second_start_switch == 1)		//�Է��Ժ󣬶��������Ĺ�����Ȼ��Ч
		    second_start_counter = 0;
		already_in_origin = 0;				//�Է춯���󣬾Ͳ���ԭ����
		if(AUTO_SEARCH_START_POINT == 1)	//û�ҹ�ԭ�㣬��ִ���������ԭ�㴦��
		{
			if (already_auto_find_start_point == 0)
			{
				find_start_point();
				already_auto_find_start_point = 1;
			}
		}
		set_func_code_info(READY,2,single_flag,0);
	}
	
  	switch(single_flag)
  	{
   		case 1:	 //5.1  ���쵥����ǰ�Ĵ���
			 
			SewingTestEndFlag = 1;  
			//5.1.1 �ƶ�ǰ�������
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			
			allx_step = allx_step + xstep_cou;//5.1.2 �ж������������Ƿ���ֳ������Ʒ�Χ�����
  			ally_step = ally_step + ystep_cou;
			
			if( check_sewing_range())//5.1.2.1 ����Χ�Ĵ���
			{
				allx_step = allx_step - xstep_cou;
  				ally_step = ally_step - ystep_cou;
				do_pat_point_sub_one();
				single_flag = 0;
				move_flag = 0;
				nopmove_flag = 0;
				sys.error = ERROR_15;
				StatusChangeLatch = ERROR;
				set_func_code_info(READY,3,0,0);
			}
			else
			{	
				//5.1.2.1 û����Χ���,�Ȼָ�����
				allx_step = allx_step - xstep_cou;
  				ally_step = ally_step - ystep_cou;
				//
				if(nop_move_pause_flag ==1)
				{
		    		//5.1.2.2 ���֮ǰ�����˿��͹����а���ͣ���������ʱ����ǰ��Ҫ�Ȱѿ���
					process_nop_move_pause(1);
					set_func_code_info(READY,4,0,0);
					single_flag = 0;
				}
				else
				{			
					if(nopmove_flag == 1)
					{	
						if(u103 != 0 && inpress_flag == 0)   
						{
							inpress_up();        	
						}
					}
			
					if(move_flag == 1)                                 
					{
						if(u103 == 2)
					    {
						     if( inpress_flag == 1)
						     {
						         inpress_down(inpress_high);
						     }
						     else
						     {
							     if(single_inpress_flag == 1 )     
						         {
							        inpress_to(inpress_high);  
							        inpress_flag = 0;  	   
							     }    		       
						     }
							 single_inpress_flag = 0;
						}
						
						if(ROTATED_CUTTER == 1)
						{
							if( rotated_cutter_enable == 1)
							{
								if ( milling_cutter_action_flag ==1)
								{
									rotated_cutter_single_next();		
								}
								else if(milling_cutter_action_flag ==2 )
								{
									rotated_cutter_single_stop();
									milling_cutter_action_flag = 0;
								}
								
							}
						}  
						
					}
					
					#if ENABLE_LOOPER_CUTTER
						if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
						{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 0) )//ƫ�Ʋ�Ϊ0
							{
							     allx_step_tmp = allx_step;
		 	 					 ally_step_tmp = ally_step;
								 go_commandpoint(x_bios_offset + allx_step, y_bios_offset + ally_step);	  
								 allx_step = allx_step_tmp;
		 	 					 ally_step = ally_step_tmp;
								 delay_ms(300);
								 milling_first_move = 1;				
							}
						}
					#endif
					
					#if ENABLE_LASER_CUTTER
					process_laser_offset_move(0);
					#endif
					
					SewTestStitchCounter++;
					if( (FootRotateFlag == 1) &&(marking_flag == 1) )
					{
						process_marking_pen(0);
						marking_flag = 0;
						FootRotateFlag = 0;
						delay_ms(50);
						set_func_code_info(READY,5,0,0);
					}
					else 
						move_next();
				}
			}   
			SewingTestEndFlag = 0;   
			predit_shift = 0;                    
			break;
			
  		case 2:
		       
			SewingTestEndFlag = 1;  
			SewTestStitchCounter--;		
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		} 
			
		    if(nopmove_flag == 1)
			 {
					if(inpress_flag == 0)  
					{
						inpress_up();        	
					}
			}
			if(move_flag == 1)
			{
				if(u103 == 2)              
			    {	
					if(inpress_flag == 1)  
					{
						inpress_down(inpress_high);        	
					}
					else
					{
						 if(single_inpress_flag == 1)     
					      {
						        inpress_to(inpress_high);  
						        inpress_flag = 0;           	
					       }
					}
					single_inpress_flag = 0;
				}
			}
			if( (nop_move_pause_flag ==1) ||(finish_nopmove_pause_flag ==1) )
			{
		    	process_nop_move_pause(2);
				finish_nopmove_pause_flag = 0;
				set_func_code_info(READY,6,0,0);
			}
			else
			{
	            if(ROTATED_CUTTER == 1)
				 {
					 if( rotated_cutter_enable ==1 )
					 {
						if ( milling_cutter_action_flag ==1)
						{
							rotated_cutter_single_back();		
						}
						else if(milling_cutter_action_flag ==2 )
						{
							rotated_cutter_single_stop();
							milling_cutter_action_flag = 0;
						}
					 }
				 }
				#if ENABLE_LOOPER_CUTTER
						if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
						{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 1) )//ƫ�Ʋ�Ϊ0
							{
							     allx_step_tmp = allx_step;
		 	 					 ally_step_tmp = ally_step;
								 go_commandpoint(allx_step-x_bios_offset ,ally_step-y_bios_offset );	
								 allx_step = allx_step_tmp;
		 	 				     ally_step = ally_step_tmp;  
								 delay_ms(300);
								 milling_first_move = 0;
							
							}
						}
				#endif
				
				#if ENABLE_LASER_CUTTER
				process_laser_offset_move(1);
				#endif
				
				if( (FootRotateFlag == 1) &&(marking_flag == 1) )
				{
						process_marking_pen(1);
						marking_flag = 0;
						FootRotateFlag = 0;
						delay_ms(50);
				}		
				else
				move_back(); 
			}
			SewingTestEndFlag = 0; 
			predit_shift = 0;
			single_flag = 0; 
			break;
  		case 3:	 
			inpress_high = inpress_high_hole;
			tension = tension_hole;

			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			if(inpress_flag == 0)  		
			{
				inpress_up();        	
			}
			R_AIR = 0;
			marking_finish_flag = 0;
		#if ENABLE_LOOPER_CUTTER
			if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
			{
				if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 1) )//ƫ�Ʋ�Ϊ0
				{
				     allx_step_tmp = allx_step;
		 	 		 ally_step_tmp = ally_step;
					 go_commandpoint(allx_step-x_bios_offset ,ally_step-y_bios_offset );	  
					 allx_step = allx_step_tmp;
		 	 		 ally_step = ally_step_tmp;
					 delay_ms(300);
					 milling_first_move = 0;
	
				}
			}
        #endif		
		if(ROTATED_CUTTER == 1)
		{
		  if( rotated_cutter_enable ==1 )
		   {
		   	  //DRILL_MOTOR_UPDOWN = 0;//ȷ�ϵ���ȥ��
			  rotated_cutter_down_positon();
			   if ( milling_first_move ==1)
			   {		
					if( (x_bios_offset!=0)||(y_bios_offset!=0) )//ƫ�Ʋ�Ϊ0
					{
					     allx_step_tmp = allx_step;
		 	 			 ally_step_tmp = ally_step;
						 go_commandpoint(allx_step-x_bios_offset  ,ally_step -y_bios_offset );	
						 allx_step = allx_step_tmp;
		 	 			 ally_step = ally_step_tmp;  
						 delay_ms(300);
						 milling_first_move = 0;//2017-6-6
					}
			   }
		   }	
		}  
		#if ENABLE_LASER_CUTTER
					if( milling_cutter_action_flag != 0)
					{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 1) )//ƫ�Ʋ�Ϊ0
							{
							     allx_step_tmp = allx_step;
		 	 					 ally_step_tmp = ally_step;
								 go_commandpoint(allx_step - x_bios_offset,ally_step - y_bios_offset);	  
								 allx_step = allx_step_tmp;
		 	 					 ally_step = ally_step_tmp;
								 delay_ms(300);
								 milling_first_move = 0;
						
							}
					}
		#endif
			if(u39 == 1)
			    go_origin_allmotor();
			else
				move_startpoint();
			set_func_code_info(READY,7,0,0);
			move_flag = 0;
			nopmove_flag = 0;
			single_flag = 0; 
			TestStatusChangeFlag = 0;
			StopStatusFlag = 0;
			stop_flag = 0;
			if(u38 == 0)       
				foot_com = 1;  
			predit_shift = 0;            
			break;		
  		case 4:	 
			SewingTestEndFlag = 1;  
			if(ROTATED_CUTTER == 1)
			{
				if ( (milling_cutter_action_flag ==1)&&(rotated_cutter_enable ==1) )
				     rotated_cutter_single_stop();
			}

			if(nop_move_pause_flag ==1)
		    	process_nop_move_pause(1);
			else
				go_sewingtest_beginpoint();  
			
			SewingTestEndFlag = 0;   
			predit_shift = 0;
			single_flag = 0;
			break;			
  		case 5:	 
	
			SewingTestEndFlag = 1;  
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			inpress_up(); 
			if(ROTATED_CUTTER == 1)
			{
				if ( (milling_cutter_action_flag ==1)&&(rotated_cutter_enable ==1) )
				     rotated_cutter_single_stop();
			}
			
			if(nop_move_pause_flag ==1)
		    	process_nop_move_pause(2);
			else
			{
			 	back_endpoint();
			}
			if( (FootRotateFlag == 1) &&(marking_flag == 1) )
			{
				process_marking_pen(1);
				marking_flag = 0;
				FootRotateFlag = 0;
				delay_ms(50);
			}
			SewingTestEndFlag = 0;  
			predit_shift = 0;
			single_flag = 0;  
			break;
  		case 6:	 

			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			course_next();  
			if( (FootRotateFlag == 1) &&(marking_flag == 1) )
			{
				 process_marking_pen(0);
				 marking_flag = 0;
				 FootRotateFlag = 0;
				 delay_ms(50);
			}
			#if ENABLE_LOOPER_CUTTER
						if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
						{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 0) )//ƫ�Ʋ�Ϊ0
							{
							     allx_step_tmp = allx_step;
		 	 					 ally_step_tmp = ally_step;
								 go_commandpoint(x_bios_offset + allx_step,y_bios_offset + ally_step);	  
								 allx_step = allx_step_tmp;
		 	 					 ally_step = ally_step_tmp;
								 delay_ms(300);
								 milling_first_move = 1;
								
							}
						}
			#endif
			
			#if ENABLE_LASER_CUTTER
				process_laser_offset_move(0);
			#endif
			delay_ms(2);  
			set_func_code_info(READY,8,0,0);                  
			if(PAUSE == pause_active_level)
			{
				delay_ms(10);
				if(PAUSE == pause_active_level)
				{				
					predit_shift = 0; 
					single_flag = 0;
			  	}
	 
		  	}
			break;		
  		case 7:	 	

			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			course_back();
			if( (FootRotateFlag == 1) &&(marking_flag == 1) )
			{
				process_marking_pen(1);
				marking_flag = 0;
				FootRotateFlag = 0;
				delay_ms(50);
			}
			delay_ms(2); 
			#if ENABLE_LOOPER_CUTTER
						if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
						{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 1) )//ƫ�Ʋ�Ϊ0
							{
							     allx_step_tmp = allx_step;
		 	 				     ally_step_tmp = ally_step;
								 go_commandpoint(allx_step-x_bios_offset ,ally_step-y_bios_offset );
								 allx_step = allx_step_tmp;
		 	 					 ally_step = ally_step_tmp;	  
								 delay_ms(300);
								 milling_first_move = 0;
								
							}
						}
			#endif
			#if ENABLE_LASER_CUTTER
			process_laser_offset_move(1);
			#endif
			set_func_code_info(READY,9,0,0);               
			if(PAUSE == pause_active_level)
			{
				delay_ms(10);
				if(PAUSE == pause_active_level)
				{				
					predit_shift = 0; 
					single_flag = 0;
			  	}
	 
		  	}
			break;			
  		case 8:	 

			course_stop();       
			predit_shift = 0;   
			set_func_code_info(READY,10,0,0);	
			break;	
		default:                                     
			break;	
  	}
	
	if( (return_from_setout == 1)&&(clamp_com == 1)&&(clamp_stepflag == 2)&&(stitch_counter < u33) )
	{
		clamp_backstep2(); 
	}
	
	if(ENABLE_BOBBIN_CASE_FUN == 1)
	{
		if( bobbin_case_once_done_flag == 1)
		{
			if( bobbin_case_enable == 1)
				bobbin_case_workflow1();
			bobbin_case_once_done_flag = 0;
			temp8 = detect_position();	
	    	if(temp8 == OUT)    
	      	{
			   find_dead_center();
	      	}
			if( sys.status == ERROR)
				return;
		}
		if( bobbin_case_switch_flag ==1 )
		{
			bobbin_case_switch_flag = 0;
			if( bobbin_case_enable == 1)
			    find_a_bobbin_case(5);
		}
	}


	if(super_pattern_flag == 1)
	{
		if(coor_com  == 1)//0x77
	  	{
			if( (nop_move_pause_flag ==0)&&(foot_flag == 0) )
			{
				return_from_setout = 1;
				temp8 = detect_position();	
				if(temp8 == OUT)   
		  		{
					find_dead_center();
		  		}
				if(inpress_flag == 0)
				{
					inpress_up();
					delay_ms(20);
				}
				if(AUTO_SEARCH_START_POINT == 1)
				{
					if (already_auto_find_start_point == 0)
					{
						find_start_point();//error
						already_auto_find_start_point = 1;
					}
				}
				delay_ms(50);				
		     	go_commandpoint(comx_step,comy_step);				
				need_action_once = 0;//��ת�Ժ���ֱ�Ӽӹ̣�Ҫ�ж�λ��
				delay_ms(500);
				set_func_code_info(READY,11,0x77,0);
			}
			predit_shift = 0;
			coor_com = 0;			
			synchronization_flag = 1;
	  	}
		
		if( PointShiftFlag == 1)//0x64
	    {	
		    if( (nop_move_pause_flag ==0)&&(foot_flag == 0) )
			{
				pat_point = (PATTERN_DATA *)(pat_buf);
			    pat_point = pat_point + (PointShiftNumber%TOTAL_STITCH_COUNTER);
			    pat_buff_total_counter = PointShiftNumber;
			}
		    PointShiftFlag = 0;
			predit_shift = 0 ;
			synchronization_flag = 0;
			set_func_code_info(READY,12,PointShiftNumber>>8,PointShiftNumber);
            monitor_allx_step = allx_step;
			monitor_ally_step = ally_step;
			monitor_pat_point = pat_point - 1;
			monitor_refresh_flag = 1;
	    }
	}

	if(FootUpCom == 1)
	{
		if(u38 == 0)
		{
			if(DVB == 0)    
			{
				delay_ms(10);
				if(DVB == 0 && DVBLastState == 1)
					{
							footer_both_up();
							FootUpCom = 0;
			   	    	    DVBLastState = DVB;	
				  	}
			 }
			 else
			 {
					DVBLastState = DVB;
			 }
			rec_com();   

   		}
  		else
		{
			FootUpCom = 0;
		}
 		//--------------------------------------------------------------------------------------
	  	//  switch system status 
	  	//-------------------------------------------------------------------------------------- 
		if(StatusChangeLatch != READY)
		{
			predit_shift = 0;
			sys.status = READY;
			StatusChangeLatch = READY;
			return;
		}
	}
	else
	{
	  	if(aging_com == 1)
	  	{
	  		temp16 = (UINT16)aging_delay * 100;
	  		if(temp16 > 9900)
	  		{
	  			temp16 = 9900;
	  		}
	  		delay_ms(temp16); 
			foot_com = 0; 	  	
	  	}
		//ץ��λ�ô���
		if(u35==0)                            
  		{
	  		  if(clamp_com == 1)          
	  		  {
		  		  	if( return_from_setout == 0 )//ȷ�����Ƿ��ƹ����з��ص�READY�����
					{
						if(clamp_flag == 0)       
			  		  	{
			  		  		go_origin_ct();
							delay_ms(20);
							clamp_out();
			  		  	}
		  		  	    clamp_stepflag = 1;       // clamp thread step 1 
		  		  		movect_angle = 800;       // 800---281 degree    
		  		  		if(TSENS ==0  )          
					    {	
					    	delay_ms(100);
			  		      	if(TSENS ==0 )
			  		      	{
			  		    	  		sys.status = ERROR;
									StatusChangeLatch = ERROR;
									if( sys.error == 0)
			  		        		    sys.error = ERROR_23;	 		  
			  		        		return;
			  		      	}
		  		    	}	
					}	
	  		  }
	  		  else                         // check clamp flag
	  		  {
	  		  	if(clamp_flag == 1)        // clamp is out
	  		  	{
	  		  		clamp_in();
	  		  	}
	  		  	clamp_stepflag = 0;      
	  		  	if(TSENS == 0)            // sensor is not covered
				{	
				    	delay_ms(100);
		  		      	if(TSENS == 0)
		  		      	{
		  		    	 		sys.status = ERROR;
								StatusChangeLatch = ERROR;
								if( sys.error == 0)
		  		        		sys.error = ERROR_23;		  
		  		        		return;
		  		      	}
	  		    }
	  	   }
  		}
  		else if(u35 == 1)                   // have no clamp thread motor
  		{
  		    if(clamp_com == 0)              // check clamp flag
  		    {
			  	if(clamp_flag == 1)         // clamp is out
	  		  	{
	  		  		clamp_in();
	  		  	}
  		  		clamp_stepflag = 0;      
			}
		}
		//--------------------------------------------------------------------------------------
	  	//  find origin
	  	//-------------------------------------------------------------------------------------- 
		if(origin_com == 1)
		{						
			if(  (origin_com_after_pattern_done  ==1) && (already_in_origin == 1)&&(allx_step ==0)&&(ally_step==0) )
			{
				predit_shift = 0;	
				origin_com = 0;
			}
			else
			{
				temp8 = detect_position();	
	    		if(temp8 == OUT)    
	      		{
					find_dead_center();
	      		}
				if( marking_finish_flag == 1 )
				{
					marking_finish_flag = 0;
					if( (pen_x_bios_offset!=0)||(pen_y_bios_offset!=0) )
					{
						  allx_step_tmp = allx_step;
					 	  ally_step_tmp = ally_step;
					      go_commandpoint(allx_step - pen_x_bios_offset ,ally_step - pen_y_bios_offset);
						  allx_step = allx_step_tmp;
					 	  ally_step = ally_step_tmp;
					}
				}
				
				
				if( (milling_cutter_action_flag != 0)|| ( (ROTATED_CUTTER == 1)&&(rotated_cutter_enable ==1) ) )
				{
					if( (ROTATED_CUTTER == 1)&&(rotated_cutter_enable ==1) )
					{
						//DRILL_MOTOR_UPDOWN = 0;//ȷ��
						rotated_cutter_down_positon();
						DRILL_FOOTER = 0;
						drill_motor_run_enable = 0; //ϳ���ص�
						delay_ms(500);
					}
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 1) )//ƫ�Ʋ�Ϊ0
							{
							     allx_step_tmp = allx_step;
		 	 					 ally_step_tmp = ally_step;
								 go_commandpoint(allx_step - x_bios_offset,ally_step - y_bios_offset);	  
								 allx_step = allx_step_tmp;
		 	 					 ally_step = ally_step_tmp;
								 delay_ms(300);
								 milling_first_move = 0;
						
							}
				}
					
				go_origin_allmotor();	   
				#if CHANGE_DOUBLE_FRAMEWORK   //˫��ģ��
					AUTO_LEFT_FRAME_STANDBY = 0;
					AUTO_RIGHT_FRAME_STANDBY = 0;
					right_quest_running = 0;
					left_quest_running = 0;
					current_running_flag = 0;
				#endif
				delay_ms(20);   
				if( (sewingcontrol_flag == 2)&&(sewingcontrol_stitchs != 0) )
				   	need_backward_sewing = 1; 
				if( sewingcontrol_flag == 1)
				    need_action_two = 1;
				
				if( k115 == 1 )
				{
					if ( already_auto_find_start_point == 0)
					{
						find_start_point();
						already_auto_find_start_point = 1;
					}
				}
						
				if(u39 != 1)           
				   go_setoutpoint(); 
			
				if(OutOfRange_flag == 1)
				{
					sys.error = ERROR_15;
					StatusChangeLatch = ERROR;
				}
				FootRotateFlag = 0;
	            if(k110 == 1)  
		        {   
		            R_AIR = 0;	
					delay_ms(80);
		        }
				predit_shift = 0;	
		    	origin_com = 0;  
			
				if( (u38 == 0)&&(return_from_preddit==0 ) )
				{
				   footer_both_up();
				}   
				return_from_preddit = 0;  	
				if ( foot_flag ==0)
				  FootNumber = 1;
				
				#if SEND_CURVER
				delay_ms(50);
				write_stepmotor_para();
				#endif
				
				#if AUTO_CHANGE_PATTERN_FUNCTION ==1
				if( power_on_ask_for_framework == 0)
				{
					power_on_ask_for_framework = 1;
					AUTO_FIRST_ASK_FRAMEWORK = 1;   //�����Ͱ����
					foot_up();
				}
				#endif
			}
			already_in_origin = 1;	
			origin_com_after_pattern_done = 0;
			finish_nopmove_pause_flag = 0;
			autosewing_switch_last_status = 0;
	  	}
	 	//--------------------------------------------------------------------------------------
	  	// inpress down or up
	  	//-------------------------------------------------------------------------------------- 
	  	switch(inpress_com)
	  	{
	  		case 0:                           
	  		  	if(inpress_flag == 1)  
	  	        {
	  	        	 inpress_down(inpress_high);      
	  	        }
	  	        break;
  	         
	    	case 1:                         
	    	  	if(inpress_flag == 0) 
	  	        {
 	      			temp8 = detect_position();	
					if(temp8 == OUT)        
					{
						find_dead_center();
					}
    	        	 inpress_up();        
					 if(foot_com == 1)
					 {
						 delay_ms(30);
					 }
	  	        }
	  	      	break;		
  	
	  		case 2:  break;                
  		       
	  		default: break;                   	         	
	  	}
	  	//--------------------------------------------------------------------------------------
	  	// foot down or up
	  	//-------------------------------------------------------------------------------------- 
	  	switch(foot_com)
	  	{
	  		case 0:                        
	  			if(foot_flag == 1)     		
	  	        {
					footer_both_down();          	
	  	        }
	  	        break;
  	         
	    	case 1:                         
	    	  	if(foot_flag == 0)     		
	  	        {
	  	        	 footer_both_up();    
					 if( k115 == 0) 
					 	 already_auto_find_start_point = 0;
	  	        }
	  	        break;		
  	
	  		case 2:  break;               
  		       
	  		default: break;                   	         	
	  	}  
		
		if(aging_com == 1)
		{
			if(k110 == 2)
			{
				delay_ms(200);  	  	
				stretch_foot_out();
			}
		}
  	
	  	if(aging_com == 1)
		 {
		    	if( (foot_flag == 1 && u202 == 1) || (foot_flag == 0))
			      {
				      temp8 = detect_position();	
		    	      if(temp8 == OUT)    
		      		  {
						   find_dead_center();
		      		  }
					  if ( finish_nopmove_pause_flag == 1 )
					  {
						   origin_com =1;
						   special_go_allmotor_flag = 1;
					  }
					  else
					  {
						  if(nop_move_pause_flag ==1)
		    				process_nop_move_pause(1);
                       	    pre_run_conditioning();
					  }
			       }
						
		 }
		else
		 {	
			//--------------------------------------------------------------------------------------
		  	//  switch system status 
		  	//-------------------------------------------------------------------------------------- 
			if(StatusChangeLatch != READY)
			{
				if(stop_flag == 0)
				{
					predit_shift = 0;
					sys.status = StatusChangeLatch;
					return;
				}
				else if(stop_flag == 1)
				{
					if(StatusChangeLatch == NEEDLE || StatusChangeLatch == SINGLE)	
					{
						predit_shift = 0;
						sys.status = StatusChangeLatch;
						return;
					}	
					else
					{
						predit_shift = 0;
						StatusChangeLatch = READY;
						sys.status = READY;
					}	
				}
			} 	
			else
			{
			      if(k60 == 1)
				  {
				  		if(DVSM == 0)
						{
							 delay_ms(10);
							 if(DVSM == 0 && DVSMLastState == 1)
							 {				
										if(foot_half_flag == 1)
										{		 
								      	      if(foot_flag == 0)  
											  {
											        foot_half_down();
								      		        if(u104 == 1)
								      		         {	      	
								      			         delay_ms(50);
											      	     inpress_down(inpress_high); 
								      		          }	
											   }
								    	}
								  		else if(foot_half_flag == 0)
										{
							    			if(u104 == 1)
							      			{	      	
												inpress_up();
												delay_ms(50); 
							      			}
											foot_half_up();			    				
							    		}
		 					    }
						  	}
						 	DVSMLastState = DVSM;
					}//k60
					
					if( ( enable_footer_up ==1 )|| (((foot_flag ==1)||(return_from_setout == 0))&& (u38 == 0) && ((u41 == 1)||(stop_flag == 0)||((stop_flag == 1) && (stop_number%2 == 1)) )) )
					{
						  	if(DVB == 0)           				
							{
								delay_ms(10);
								if(DVB == 0 && DVBLastState == 1)  
								{	
								   	  if(k60 == 1)    
									  {
										 if(foot_flag == 0)
										 {
											  if(foot_half_flag == 1)
													foot_up();
										 }
										 else
										 {
											 	 foot_down();
										 }
									  }
									  else
									  {
									       footer_procedure(); 
									  }
							  	}
						  	}
							DVBLastState = DVB;
					} 
					
					
				  //==============================================================================================
				  
				   if( (k60 == 0 || k60 == 1 && DVSM == 1) && DVA == 0 && DVB == 1 )   
                   {
	 	                  delay_ms(10);
		                 if ( (k60 == 0 || k60 == 1 && DVSM == 1) && DVA == 0 && DVALastState == 1 && DVB == 1 )		
		                  {   
							  if( one_step_run_flag == 1) //�Զ�����
							  {
								  if( return_from_setout == 0 )//һ������
								  {
								  	 footer_both_down();
								  	 //
									 if( bar_coder_refresh_enable == 0)
									 {
										 pattern_change_flag = 0;
										 if( nop_move_pause_flag ==1)
			    						     process_nop_move_pause(1);
									  	 pre_run_conditioning();
									 }
									 else
									 	 autosewing_allset_flag = 1;
								  }
								  else
								  {
									  if( nop_move_pause_flag ==1)
			    						  process_nop_move_pause(1);
									  pre_run_conditioning();
								  }
							  }
							  else
							  {			  
							  
							  	if((k74 == 0 &&  foot_flag == 0) || (k74 == 1 && (u81 <= 1 || u81 >= 4) && foot_flag == 0) || (k74 == 1 && (u81 == 2 || u81 == 3) && foot_flag == 0 && foot_half_flag == 0) )
								{
									DVA_scan_flag = 0;
									DVA_action_done = 0;
									
									temp8 = detect_position();	
							    	if( temp8 == OUT)     
							      	{
										find_dead_center();
							      	}
									if ( finish_nopmove_pause_flag == 1 )
									{
										origin_com =1;
										special_go_allmotor_flag = 1;
									}
									else
									{
										  if( nop_move_pause_flag ==1)
			    							  process_nop_move_pause(1);
										  #if AUTO_CHANGE_PATTERN_FUNCTION	  
										  if( return_from_setout == 0)
											  go_commandpoint(0,0); 
										  autosewing_offset_moveing_done = 0;
										  #endif
										  pre_run_conditioning();										 
									}
						    	 }
							  }
						      DVALastState = DVA;
						   }
						  else
						   {
						      DVALastState = DVA;
						   }
				}
				else    
				{
					  DVALastState = DVA;
				}
					  
				 #if FUNCTION_AUTOSEWING
				 
				 #if AUTO_CHANGE_PATTERN_FUNCTION == 1
				 if(autosewing_allset_flag == 1) //���͵�λ��
				 {
					 if( autosewing_offset_moveing_done == 0)
					 {
						 //SUM = 1;
						 AUTO_ALLOW_FEEDARM_RELEASE = 1;//���Էſ��ˡ�
						 while( AUTO_ALLOW_SEWING_NOW == 0)//�Ѿ��ƿ���
						 {
							 if(PAUSE == pause_active_level)
							    break;
							 rec_com();
						 }
						 AUTO_ALLOW_FEEDARM_RELEASE = 0;
						 
						 autosewing_offset_moveing_done = 1;
						 delay_ms(300);
						 go_commandpoint(x_bios_offset*10 + allx_step,ally_step);
						 //SUM = 0;
						 AUTO_ALLOW_TAKE_OUT_FRAMEWORK = 0;//�����е��ȡ�ź�
						 AUTO_FIRST_ASK_FRAMEWORK = 0;//�����ź����
					 }
				 }
				 if( (autosewing_allow_working == 1)&&(autosewing_allset_flag == 1) )
				 {
						delay_ms(100);
						SUM = 0;
						temp8 = detect_position();	
						if( temp8 == OUT)     
						 	find_dead_center();
						
						if ( finish_nopmove_pause_flag == 1 )
						{
							 origin_com =1;
							 special_go_allmotor_flag = 1;
						}
						else
						{
							if( nop_move_pause_flag == 1 )
				    			process_nop_move_pause(1);
							go_commandpoint(0,0);							
							delay_ms(100);
							do{
								
							pre_run_conditioning();	
							rec_com();
							}while( sys.status != RUN);
							
							autosewing_allow_working = 0;
							autosewing_allset_flag = 0;	
							autosewing_offset_moveing_done = 0;															 
						}
						
				 }
				 #else
				 if( (autosewing_allow_working == 1)&&(autosewing_allset_flag == 1) )
				 {
					  temp8 = detect_position();	
					  if( temp8 == OUT)     
					  	  find_dead_center();
					
					  if ( finish_nopmove_pause_flag == 1 )
					  {
							origin_com =1;
							special_go_allmotor_flag = 1;
					  }
					  else
					  {
							if( nop_move_pause_flag == 1 )
		    					process_nop_move_pause(1);
							pre_run_conditioning();										 
					  }
					  autosewing_allow_working = 0;
					  autosewing_allset_flag = 0;	
				 }
				 #endif
				#endif
				
					
			}
			
		}//���ϻ�ģʽ
		
		if( (single_flag ==0 )&&(origin_com == 0)&&(PointShiftFlag==0)&&(coor_com==0) )
		 	predit_shift = 0;
	}//ѹ��̧
}

/**
  * @�������� ����״̬����
  * @���� ��
  * @����ֵ ��
  */
void run_status(void)
{			
	UINT8 temp8,i,j,flag,last_stitch_down,flag1,sewingcontrol_stitchs_abs;
	INT8 tmp_stitchs;
	INT16 temp16;
	UINT16 tmp16_stitchs,tmpspd;		
	PATTERN_DATA *tmp_point;
	UINT32  tmp32_spd1,tmp32_spd2;
	#if FOLLOW_INPRESS_FUN_ENABLE
	INT16 inpress_up_angle,inpress_down_angle,max_angle,inpress_position_tmp;
	UINT8 action_flag0,action_flag1,action_flag2,action_flag3,action_flag4;
	#endif	
	UINT8 nop_stop_flag;	
  	zpl_pass = 1;
	stop_flag = 0;
  	cut_flag = 0;
	move_flag = 0;
	nopmove_flag = 0;
	PatternDelayFlag = 0;
  	process_flag = 1;	
  	motor.dir = 0;
  	stitch_counter = 1;  
  	laststitch_flag = 0;
  	pause_count = 0;               // clear pause counter
  	stay_flag = 0;                 // emergency break flag clear
  	thbrk_count = 0;               // clear thread breakage counter
  	thbrk_flag = 0;                // clear thread breakage flag
  	brkdt_flag = 0;                // clear thread breakage detection flag  
	FootRotateFlag = 0;
	last_max_length = 0;           //
	single_flag = 0;               
	#if DEBUG_RFID_DA1_FUN	
	da1 = 0;
	#endif                               
  	//--------------------------------------------------------------------------------------
  	//  parameter confirm
  	//--------------------------------------------------------------------------------------
	return_from_setout = 1;     //��ʾ���з���״̬���Ժ�ص�READYʱ��Ҳ֪������û������������
	para_confirm();	

	DVA_scan_flag = 0;
	DVA_scan_counter = 0;
	DVA_action_done = 0;

	SPISTE3 = 1;
	
  	//--------------------------------------------------------------------------------------
  	//  process first pattern data
  	//-------------------------------------------------------------------------------------- 
  	while(1)
  	{	    
    	if ( ready_go_setout_com == 1)//��Ӧ�����������ֹ׼��״̬û����Ӧ���������״̬����
		{
			ready_go_setout_com = 0;
			predit_shift = 0;
		}
		
    	if(end_flag == 1 || stay_flag == 1 || stop_flag == 1)
    	{   	
			break;
    	}
    	else
    	{
			while(1)
			{
				rec_com();
				process_data();
				nop_stop_flag = 0;
	    		if(nopmove_flag == 1)//����ǰ������Ĵ���
	    		{	
					do_pat_point_sub_one();
					
					if( release_tension_before_nopmove ==1)
					{
						da0_release_flag = 0;
						da0 = 255;  
						da0_release_conter = 0;
						da0_release_flag = 1;
					}
					else
						da0 = 0;
					
					tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
				    set_func_code_info(RUN,0,tmp16_stitchs>>8,tmp16_stitchs);
					
					go_beginpoint(0); //�ֶο��ʹ���
					
					if( stop_flag ==1)
					    nop_stop_flag = 1;
					tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
					set_func_code_info(RUN,20,tmp16_stitchs>>8,tmp16_stitchs);
				
					if( release_tension_before_nopmove ==1 )
					{
						da0_release_flag = 0;
						da0 = 0;  
						da0_release_conter = 0;							
					}
						
					if( nop_move_pause_flag == 1)//���͹����з�����ͣ��ֱ���л�������״̬
					{
						status_now = READY;
			      		sys.status = ERROR;
			      		StatusChangeLatch = ERROR;
						if( sys.error == 0)
      		      			sys.error = ERROR_02;
						return;
					}
					if( OutOfRange_flag == 1)
					{
						sys.error = ERROR_15;
						StatusChangeLatch = ERROR;
					}
					if( sys.status == ERROR)
					{
						status_now = READY;
						return;
					}
					if( move_flag == 1 || RotateFlag == 1)
					{
					    do_pat_point_add_one();//���ͺ���ӳ��죬�����м�1�����ȼ�1
						tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
						set_func_code_info(RUN,23,tmp16_stitchs>>8,tmp16_stitchs);
					}
					if( FootRotateFlag ==1 )
					    FootRotateFlag = 0;
				}
				
				if( FootRotateFlag ==1 )//����ǰ�Ǻűʹ���
				{
					if( (k110 == 0) )
					{
						marking_flag = 1;
						process_marking_pen(0);
					}
					if(k110 == 1)
					{
						R_AIR = R_AIR^1;
					}					
					else if(k110 == 2)      
					{
						if( stretch_foot_flag == 0)
							stretch_foot_in();
						else
							stretch_foot_out();
					}
					FootRotateFlag = 0;
				}
				if ( PatternDelayFlag == 1 )//����ǰ������ʱ
				{
					delay_ms(PattenrDelayTime);
					PatternDelayFlag = 0;
				}
				if ( stop_flag == 1 )	//����ǰ������;ֹͣ��
				{
					if( nop_stop_flag == 0)
						do_pat_point_sub_one();
					if((stop_number%2 == 1)&&(u41 == 0))   
					{
					    footer_both_up();
					}
					sys.status = READY;
				    StatusChangeLatch = READY;
		            StopStatusFlag = 1;
					return;
				}
				if ( end_flag == 1 ) //����ǰ���ֽ����룬��FINISH״̬����
				{
					inpress_up();
					process_flag = 0;
					sys.status = FINISH;
					StatusChangeLatch = FINISH;
					TestStatusChangeFlag = 0;
					ready_go_setout_com = 0;
					predit_shift = 0;
					return;
				}
				
				if ( (milling_cutter_action_flag ==1)&&(rotated_cutter_enable==1)&&(ROTATED_CUTTER == 1) )//������������ת�е���������
				{
					process_rotated_cutter();
					milling_cutter_action_flag = 0;
					if(  (end_flag == 1)||(sys.status == ERROR))
					    return;
				}
				#if ENABLE_LASER_CUTTER
			   	if( (milling_cutter_action_flag >=1 )||(laser_already_begin_flag == 1) )	//�����е�����
				{
					set_func_code_info(RUN,23,0,0);
					laser_already_begin_flag = 0;
					process_laser_cutter();
					milling_cutter_action_flag = 0;
					if(  (end_flag == 1)||(sys.status == ERROR) )
					    return;
					second_start_counter = 0;//���ö�����������
				}			  
				#endif
				#if ENABLE_LOOPER_CUTTER
				if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
				{
					process_stepper_cutter();
					milling_cutter_action_flag = 0;
					if(  (end_flag == 1)||(sys.status == ERROR) )
					    return;
				}
				#endif
				
				if(move_flag == 1 || RotateFlag == 1)
				{
					move_flag = 0;
					do_pat_point_sub_one();
					zpl_pass = 1;
		  	    	end_flag = 0;	
		  	    	process_flag = 1;	
		  	    	motor.dir = 0;
					cut_flag = 0;
					stop_flag = 0;
		  	    	stitch_counter = 1;		  	    	
        			if(u35==0)        //����ǰ��ץ�߶���׼��            
        			{
          				if(clamp_com == 1)          
          				{
          					if(clamp_flag == 0)       
    						{
    							go_origin_ct();
								delay_ms(20);
    							clamp_out();
    							clamp_stepflag = 1;      
          			  			movect_angle = 800;      // 800---281 degree
    						}
    						if(TSENS == 0)             // sensor is covered
	    					{	
	    						delay_ms(10);
        						if(TSENS == 0)
        						{
      	  							sys.status = ERROR;
									StatusChangeLatch = ERROR;
									if( sys.error == 0)
          							sys.error = ERROR_23;	 // clamp is not in normal position  		  
          							return;
        						}
      						}
          				}
          				else
          				{
          					if(clamp_flag == 1)        // clamp is out
            				{
            		  			sys.status = ERROR;
								StatusChangeLatch = ERROR;
								if( sys.error == 0)
            		  			sys.error = ERROR_23;	   // clamp is not in normal position	  
            		  			return;
            				}
            				else
            				{
          			  			clamp_stepflag = 0;      // clamp thread step 0
          					}
          				}
        			} 
					break;
				}//move_flag
				
			}//end while		
			
		}//end else			    	
		if( pause_flag == 1)//press sotp button when stepper motor moving
	  	{
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
	        sys.error = ERROR_02; 
			return;
		}
		 
		if(sys.status == ERROR)    
		  return;		 

		if( (second_start_switch == 1)&&(aging_com == 0) )//������������
		{
		   if( second_start_counter > 0)
		   {
			   second_start_counter --;
			   status_now = READY;
			   sys.status = READY;
			   StatusChangeLatch = READY;
			   inpress_down(inpress_high_hole);		
			   inpress_high = inpress_high_hole;
			   delay_ms(20);
			   return;
		   }
		}

		if( k03 == 0)
		{
			if ( front2stitchs_tension_off == 0)
			     da0 = 0;
			else
			{
				da0_release_flag = 0;
				da0 = 255;
				da0_release_conter = 0;
				da0_release_flag = 1;
			}
			SNT_H = 0;
		}
		
		if( baseline_alarm_flag == 1)
		{
			 if( pat_buff_total_counter >= baseline_alarm_stitchs)
			 {
				 if( ENABLE_BOBBIN_CASE_FUN == ENABLE_FUN)
				 {
				    if( bobbin_case_enable == 1)
					{
						temp8 = 0;
						tmp16_stitchs = baseline_alarm_stitchs;
						if( bobbin_case_alarm_mode == 1)//�Ȼ����ٱ���
						{
							sys.status =  ERROR;
							sys.error = ERROR_45;
							if( bobbin_case_restart_mode == 1)
							{
								delay_ms(300);
								sys.status =  RUN;
								sys.error = 0;
								StatusChangeLatch = READY;
							}
							temp8 = bobbin_case_workflow1();
							bobbin_case_once_done_flag = 0;
						}
						if( (bobbin_case_restart_mode == 1)&&(temp8 ==1) )
						{
						//	while( tmp16_stitchs == baseline_alarm_stitchs)
							{
								delay_ms(20);
							}
						}
						else
						{
							sys.status =  ERROR;
							sys.error = ERROR_45;
							StatusChangeLatch = ERROR;
						    return;
						}
					}
					else
						{
							sys.status =  ERROR;
							sys.error = ERROR_45;
							StatusChangeLatch = ERROR;
						    return;
						}
				}
				else
				{
					 sys.status =  ERROR;
					 sys.error = ERROR_45;
					 StatusChangeLatch = ERROR;
				     return;
				}
				
			 }
		}

		if ( alarm_output_enable >= 2)
		{
			#if DISABLE_FUN //2016-01-22�봵����ͻ
			YELLOW_ALARM_LED = 0;
			#endif
			RED_ALARM_LED = 0;
			GREEN_ALALRM_LED = 1;
		}
		if( blow_air_flag == 1 )
		{
			blow_air_flag = 0;
		    if( k171 ==1 )
			  	BLOW_AIR3 = 0;
			else
			  	BLOW_AIR2 = 0;
		}
    	//--------------------------------------------------------------------------------------
    	//  motor run
    	//--------------------------------------------------------------------------------------			
	  	flag1 = 0;//��ѹ�Ÿı��־�����ڱ�֤���ص����ظ�	
		
		//INPRESSER_LOWER_FUN����Ϊ1��ʾǰ�����½���΢��һЩ��sewingcontrol_stitchs_abs�����ӹ������ľ���ֵ
		#if INPRESSER_LOWER_FUN && FOLLOW_INPRESS_FUN_ENABLE
		
		if( sewingcontrol_stitchs >= 0)
			sewingcontrol_stitchs_abs = sewingcontrol_stitchs;
		else
			sewingcontrol_stitchs_abs = -sewingcontrol_stitchs;
			
		if( sewingcontrol_stitchs_abs >0 )//��ʾ��Ҫǰ���뽵�͵ĵ�һЩ
		{
			 if( inpress_high_hole > 0)//�����ѹ�Ÿ߶Ȼ�׼ֵ����0����׼Խ����͵�Խ�ߣ�
			 {
				delay_ms(50);
				inpress_down(inpress_high/2);//����׼���һ��λ�ã��൱�ڽ���һ��ĸ߶ȣ��������壩
				//�ݴ��Ž��͵�һ��߶ȶ�Ӧmovestep_zx()������ʹ�õĲ��������ں����Ļָ�
				inpress_position_tmp = inpress_tab2[inpress_high/2] - inpress_tab2[inpress_high];
				flag1 = 1;
			
			 }
			 else//�����ѹ�Ÿ߶Ȼ�׼ֵ����0����ʱ�Ѿ�û�����½��Ŀռ��ˣ�ֻ�ܰ��涯��Χ����
			 {
				inpress_down(inpress_high);	
				inpress_position_tmp = inpress_follow_range;//�ݴ��������涯��Χ���������ڻָ������涯��Χ
				inpress_follow_range = inpress_follow_range/2;	
				flag1 = 1;
			 }
		}
		else
			inpress_down(inpress_high);	
		#else//������ǰ�����½���΢��һЩ�Ĺ���
		inpress_flag = 1;
		inpress_down(inpress_high);	
			#if FOLLOW_INPRESS_FUN_ENABLE
			//2018-8-5
	    	//���������ǰ����ı���ѹ�������߶ȵĹ���,��ʱ�����涯�߶�
	    	//ע�⣺ֻ����û��������ѹ�����λ�ý��Ͳ��ԣ�INPRESSER_LOWER_FUN=0��ʱ����Ч
			if(para.start_sew_change_inpress_high_enable == 0x55
				&&para.start_sew_change_inpress_high_stitchs!=0)
			{
				inpress_position_tmp = inpress_follow_range;//�ݴ��������涯��Χ���������ڻָ������涯��Χ
				inpress_follow_range = para.start_sew_change_inpress_high_range;	
				flag1 = 1;//���ں���ָ��涯��Χʱ��ִֻ֤��һ�λָ��涯��Χ����
			}
			#endif
			
		#endif
		
		inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
		
		delay_ms(50);
		
		if( MotorSpeedRigister >= 2 && MotorSpeedRigister <= MAXSPEED0)
		{
			sew_speed = MotorSpeedRigister*100;
		}
		else if(MotorSpeedRigister == 0)
		{
			MotorSpeedRigister = sew_speed/100;
		}
		
		if( para.speed_limit_switch == 1)
		{
				if( para.speed_percent >100)
					para.speed_percent = 100;
				tmp32_spd1 = para.speed_percent;
				tmp32_spd2 = sew_speed;				
				tmp32_spd2 = tmp32_spd2 * tmp32_spd1 /100; //ת�ٰٷֱ�
				tmp32_spd2 = ( tmp32_spd2 + 50 )/100*100;
				if( tmp32_spd2 < 200)
					tmp32_spd2 = 200;
				sew_speed = tmp32_spd2;
		}
		/*
		   sewingcontrol_flag: k18 ������ʼ��ӹ̷�ʽ���� 0-���ӹ� 1-��һ��ӹ� *2-��ǰ����ӹ� 3-����
		   need_action_once     �����Ƿ����Ҫִ��ͷ���ӹ̲���
		   need_backward_sewing �Ƿ���Ҫ�����ƶ��ļӹ�ǰ����
		*/
		temp8 = 0;

		if( sewingcontrol_flag >= 2 )
		{
			if( need_action_once == 1)//�ڻ�����ʼλ�ã�Ҫ���мӹ̴���
			{
				if( sewingcontrol_flag == 2 )
			        temp8 = 1;
				else
				    temp8 = 4;
			}
			else 
			{
				/*
				ִ�����Է������ת,Ҫ������ָ������ʲôλ�����䣬������ڼӹ̷�Χ�ڣ����Կ��ǽ��мӹ̡�
				�������ǣ����ǰһ����Ч�����ǿ��ͣ�������߻����������룩����ô����Ϳ���ִ�мӹ̣�
				W�ͼӹ���Ϊû�е��ӹ�������㣬���Զ���V�ͼӹ�������
				���ǵ����������Ӧ���ǿ��ͽ���+�����룬����Լ��ǰ���뷶Χ���ǿ��ͣ���ô�������V�ͼӹ�
				*/
				tmp_point = pat_point;	
				for( i=0; i<2; i++)
				{
					if( ( (tmp_point->func)&0xf0 == 0x80 ) &&(tmp_point->xstep == 0x0e)  )
					{
						temp8 = 0 ;
						break;
					}
					if( tmp_point > (PATTERN_DATA *)(pat_buf) )
					    tmp_point--;
					
					if( (tmp_point->func == 0x00)||((--tmp_point)->func != 0x20)||((--tmp_point)->func != 0x20) )
					{
						if( sewingcontrol_flag == 2 )
						{
							temp8 = 3;
						}
						else
						    temp8 = 4;
						break;
					}
				}
				/*
				//Ĭ�϶���W�ͣ����ڿ�ʼλ��
				tmp_point = pat_point;	
				if( tmp_point > (PATTERN_DATA *)(pat_buf) )
					tmp_point--;
				if( ((tmp_point->func &0xf0)==0x20)||((tmp_point->func &0xf0)==0x30)||((tmp_point->func &0xf0)==0x60)||((tmp_point->func &0xf0)==0x70) )
				    temp8 = 0;
			    else if( sewingcontrol_flag == 2 )
				{
					temp8 = 3;
				}
				else
					temp8 = 4;
				*/
			}
		}
		else if( sewingcontrol_flag ==1 )
		{
			if( need_action_two ==1)
			    temp8 = 2;
			else
			{
				tmp_point = pat_point;	//������������			
				for( i=0; i<2; i++)
				{
					if( tmp_point > (PATTERN_DATA *)(pat_buf) )
					    tmp_point--;
					if( (tmp_point->func == 0x00)||(tmp_point->func == 0x10)||(tmp_point->func == 0x50) )
					{
						temp8 = 2;
						break;
					}
				}
			}
		}
		
        set_func_code_info(RUN,1,temp8,0);
	
		if( (temp8 == 1 )||(temp8 == 3) )
		{
			   if( temp8 ==3)//��ԭ����V�� ���� W�� ���мӹ�
			   {
				   tmp_stitchs = sewingcontrol_stitchs;
				   if ( sewingcontrol_stitchs >0 )
				     sewingcontrol_stitchs = -sewingcontrol_stitchs;
			   }
			   SewingReverse();
			   if( temp8 ==3)
			       sewingcontrol_stitchs = tmp_stitchs;
			   nopmove_flag = 0;
			   move_flag = 0;	
			   cut_flag = 0; 	  
			   need_backward_sewing = 0;
			   need_action_once =0;
			   StitchStartFlag = 0;
		}
		else if(temp8 == 2 )
		{
			   if(clamp_com == 0)           					
	 	  			motor.spd_obj  = u10 * 100;  
	    	   else                         					
	 	    	    motor.spd_obj = u02 * 100;   
			   process_data();
			   if(move_flag == 1)
			   {
				   check_data_speed = motor.spd_obj;
				   calculate_angle();
			   	   zoom_in_one_stitch(3,1);
		       	   need_action_two =0;
				   StitchStartFlag = 0;
			   }
			   else
			      do_pat_point_sub_one();
		}
		else if(temp8 == 4 )//�¼ӹ̷�ʽ
		{
			motor.spd_obj  = u10 * 100;
			temp8 = fabs(sewingcontrol_stitchs);
			if( temp8 != 0)
			{
				special_sewing( 0 ,temp8,0);//�����ȷ�2��
				while( motor.angle_adjusted >= 16)
		        	   rec_com(); 
		        special_sewing( 1 ,temp8,10);//����ƫ��2��
				while( motor.angle_adjusted >= 16)
		        	   rec_com(); 
				special_sewing( 0 ,temp8,0);//��λ��������2��
				while( motor.angle_adjusted >= 16)
		        	   rec_com(); 
		        special_sewing( 1 ,temp8,-10);//����ƫ��2��
			}
		}
		else
		{
			if(RotateFlag == 1)
			{
				motor.spd_obj = 600;
			}
			else
			{
			   if(clamp_com == 0)           					
	 	  			motor.spd_obj  = u10 * 100;  
	    	   else                         					
	 	    	    motor.spd_obj = u02 * 100;   
			}
		    StitchStartFlag = 1;	    
		}
		
		if( (u51 == 0)&&(sewingcontrol_flag ==0) )
		{
			temp16 = motor.angle_adjusted;
	      	while(temp16 <= 512)    //180d
	        { 
	        	rec_com();
				temp16 = motor.angle_adjusted;
			}
			FW = 1;
		}
	
		
		movexy_delay_counter = 0;
	    movexy_delay_flag = 0;
		movestep_x_flag = 0;
		movestep_y_flag = 0;
		movestepx_delay_counter = 0;
		movestepy_delay_counter = 0;
        last_stitch_down = 0;
		nopmove_flag = 0;
    	lastmove_flag = 0;	

    	//--------------------------------------------------------------------------------------
    	//  sewing cycle 
    	//--------------------------------------------------------------------------------------                
  		while(1)
    	{	  	  	  		     
			//tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
			//set_func_code_info(RUN,2,tmp16_stitchs>>8,tmp16_stitchs);
			double_xy_time_flag = 0;
			
			if( StitchStartFlag == 1)
			{
				StitchStartFlag = 0;
			}
			else if(StitchStartFlag == 0)
			{

					while(motor.angle_adjusted >= 220)//�ȴ��Ƕ�С��220/4=55�㣬���ȴ��µ�һȦ����
		    		{
		    			rec_com();    						                
						if( (sys.status == ERROR)&&(stay_flag==0) )
						{
						   sys.status = ERROR;
						   StatusChangeLatch = ERROR;
							return;
						}
						if(sys.status == POWEROFF)  
					       return;						                
		    		}  	

				stitch_counter++;      				               
			}
		    #if AUTO_CHANGE_PATTERN_FUNCTION
			if( pat_buff_total_counter >= remain_stitchs )
			{
				AUTO_NOTIFY_READY_FOR_CHANGE = 1;//���ҽӽ���ɣ�֪ͨPLC��ǰ׼���ø���ģ��
			}
			#endif
			special_pause_flag = 0;
			
				if( u71 == 1)
				{
					  temp16 = motor.angle_adjusted;
					  while(temp16 < 151)//53d = 
		    		  {
						  temp16 = motor.angle_adjusted;
						  rec_com();
					  }
					  if(TH_BRK == thread_break_detect_level)//???
				      {
				  	  		thbrk_count = thbrk_count + 1;
				      }
				}
			
			if( (front2stitchs_tension_off ==  1) && ( stitch_counter > 1) )
			{
				if( k03 == 0 )
				{
					da0_release_flag = 0;
					da0 = 0;
					SNT_H = 0;
				}
			}
			if( (u51 == 0)&&(stitch_counter>1) )
			{
				FW = 0;
			}
			test_flag = 0;
			//-----------------------------------------------------------------------------------
			//adjust speed
			//---------------------------------------------------------------------------------
			if(MotorSpeedRigister >= 2 && MotorSpeedRigister <= MAXSPEED0)
			{
				sew_speed = MotorSpeedRigister*100;
			}
			else if(MotorSpeedRigister == 0)
			{
				MotorSpeedRigister = sew_speed/100;
			}
			
			if( para.speed_limit_switch == 1)
			{
				if( para.speed_percent >100)
					para.speed_percent = 100;
				tmp32_spd1 = para.speed_percent;
				tmp32_spd2 = sew_speed;				
				tmp32_spd2 = tmp32_spd2 * tmp32_spd1 /100; //ת�ٰٷֱ�
				tmp32_spd2 = ( tmp32_spd2 + 50 )/100*100;
				if( tmp32_spd2 < 200)
					tmp32_spd2 = 200;
				sew_speed = tmp32_spd2;
			}
			
			if((stay_flag == 0)&&(brkdt_flag == 0))
			{
				zpl_process();  						
				//set_func_code_info(RUN,3,stitch_counter>>8,stitch_counter);
				//--------------------------------------------------------------------------------------
		      	//  move stepper motor
		      	//--------------------------------------------------------------------------------------   	              	                   	    
		      	if(move_flag == 1)
		      	{
					already_find_startpoint_flag = 1;
					allx_step = allx_step + xstep_cou;                        
	  				ally_step = ally_step + ystep_cou;
				
					if( check_sewing_range())
					{
						allx_step = allx_step - xstep_cou;
			    		ally_step = ally_step - ystep_cou;
						do_pat_point_sub_one();
						move_flag = 0;
						stay_flag = 1;
						set_func_code_info(RUN,4,0,0);
						OutOfRange_flag = 1;
					}
					else
					{
						allx_step = allx_step - xstep_cou;                        
	  					ally_step = ally_step - ystep_cou;
						//
							if( sewingcontrol_tail_flag ==1 )
							{
								if( (pat_point->func ==0x80)||(pat_point->func ==0xc0) )
								{
									if(pat_point->xstep == 0x04)//next code =cut
									{
										motor.spd_obj = k43*10;//????2017-3-3
										zoom_in_one_stitch(3,0);
										move_flag = 0;
										continue;
									}
								}
							}
					}
				}
				
			}
			
			//---------------------------------------------------------------------------------
			// foot rotate function
			//-----------------------------------------------------------------------------------
			if(FootRotateFlag == 1)
			{
					   if( (k110 == 0) )
					   {
							marking_flag = 1;
							process_marking_pen(0);
					   }
					   if(k110 == 1)
					   {
						     R_AIR = R_AIR^1;
					   }
					   else if(k110 == 2)      
					   {
						    if(stretch_foot_flag == 0)
							    stretch_foot_in();
						    else
							    stretch_foot_out();
					   }
						FootRotateFlag = 0;
			 }
	
			//--------------------------------------------------------------------------------------
      		//  pause
      		//-------------------------------------------------------------------------------------- 
	      	if(stay_flag == 1)
	      	{  
								
				#if FOLLOW_INPRESS_FUN_ENABLE
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
					movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
				#endif
				pause_stop();
				if(motor.stop_flag == 1)
				{
					return;
				}
				tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
				set_func_code_info(RUN,22,tmp16_stitchs>>8,tmp16_stitchs);
				
				while(motor.angle_adjusted <= 512)
				{
					rec_com();
					if( (sys.status == ERROR)&&(stay_flag==0) )
					{
					   sys.status = ERROR;
					   StatusChangeLatch = ERROR;
					   
					   sewing_stop();
					   while(motor.stop_flag == 0)
				       {
							rec_com();
						}
						return;
					}
					if(sys.status == POWEROFF)  
				      return;
			    }
	      	}
			
				
			if(RotateFlag == 1)
			{
				set_func_code_info(RUN,5,0,0);
				temp16 = motor.angle_adjusted;
	      		while(temp16 <= 1010)    // 816---287 degree     597---210 degree 
	        	{ 
	        		rec_com();    						                                               
					temp16 = motor.angle_adjusted;        
					if( (sys.status == ERROR)&&(stay_flag==0) )
					{
					   sys.status = ERROR;
					   StatusChangeLatch = ERROR;
						return;
					}   
					if(sys.status == POWEROFF)   
				       return; 
					
				} 
				sewing_stop();
				while(motor.stop_flag == 0)
				{
					rec_com();
					if(sys.status == POWEROFF)  
				       return;
				}
				delay_ms(80);
				
				if( k03 == 0 )  
			    {	                     
				 	da0 = 0;	            
				 	SNT_H = 0;              
			    }
				RotateFlag = 0;
				break;
			}
	      	//--------------------------------------------------------------------------------------
	      	//  thread breakage 
	      	//-------------------------------------------------------------------------------------- 
	      	if(brkdt_flag == 1)
	      	{
	      		#if FOLLOW_INPRESS_FUN_ENABLE
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
			    	movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
				#endif
				sewing_stop();
	      		while(motor.stop_flag == 0)
				{
					rec_com();
					if(sys.status == POWEROFF)   
				      return;
				}
				tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
				set_func_code_info(RUN,6,tmp16_stitchs>>8,tmp16_stitchs);
				
				delay_ms(180);
				
				if(u42 == 1)
				{
					find_dead_point();
				}
				temp8 = detect_position();	
				if(temp8 == OUT)    								
				{
					find_dead_center();
				}
				delay_ms(80);
			    if(k03 == 0)   
			    {	                 
			    	da0 = 0;
			    	SNT_H = 0;  
				}
			    else
			    {
				   temp_tension = 0;     
      			   at_solenoid();         
				}
				if(k167 == 1)
				{
				}
				else
				{
					inpress_up();
				}
				#if AUTO_BACKWARD_THREAD_BREAK && ENABLE_CONFIG_PARA
				if( para.thread_break_backward_switch == 1 )
				{
					single_flag = 2;
					for( i = 0;i< para.thread_break_backward_stitchs;i++ )
					{
						course_back();
						if( single_flag == 0)
							break;
					}
					single_flag = 0;
				}
				#endif
	        	sys.status = ERROR;
				StatusChangeLatch = ERROR;
	        	sys.error = ERROR_17;     			// thread breakage        
	      		return;
	      	}        
			//--------------------------------------------------------------------------------------
			// sewing stop
			//--------------------------------------------------------------------------------------
			if(SewingStopFlag == 1)
			{
				#if FOLLOW_INPRESS_FUN_ENABLE
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
			    	movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
				#endif
				switch(SewingStopValue)
				{
					case 1:
						motor.stop_angle = 	k61;
						break;
					case 2:
						motor.stop_angle = 	k61; 
						break;
					case 3:
						motor.stop_angle = 	512;
						break;	    
					default:
						break;
				}	
				set_func_code_info(RUN,7,0,0);
				if(motor.stop_angle >= 1024)
				{
					motor.stop_angle = motor.stop_angle - 1024;
				}
				
			  	motor.spd_obj = 0;
				while(motor.stop_flag == 0)
				{
					rec_com();
					if( (sys.status == ERROR)&&(stay_flag==0) )
					{
					   sys.status = ERROR;
					   StatusChangeLatch = ERROR;
						return;
					}
					if(sys.status == POWEROFF)   
				       return;
				}
				   	     	
	      		delay_ms(20);
				
				if(SewingStopValue == 2)  
				{
				    temp16 = motor.angle_adjusted;
							
     				if( (temp16 > DEADPOINT)&&(temp16 < DEGREE_180) )
					   motor.dir = 1;
	   				else
	  	 			   motor.dir = 0;
					
	    			motor.spd_obj = 120;	
	   				while(1)
	    			{
	    				rec_com(); 
		 			  	if(motor.spd_ref == motor.spd_obj)
		  			 	{
		  			 	 	break;
		  			 	}
	    			}  
				
					motor.stop_angle = DEADPOINT;
		 			if(motor.stop_angle >= 1024)
		   			   motor.stop_angle = motor.stop_angle - 1024;
					 	  
	   				motor.spd_obj = 0;  
	   				while(motor.stop_flag == 0)    
	   				{
	    				rec_com(); 
	   				}	
				}
				
				if(SewingStopValue == 1 || SewingStopValue == 2)
				{       
	        		inpress_up();
				}
				
				SewingStopFlag = 0;
				break;
			}
	    	//--------------------------------------------------------------------------------------
	      	//  cut and stop
	      	//-------------------------------------------------------------------------------------- 
	    	if(cut_flag == 1) 
	      	{
				 tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
			 	 set_func_code_info(RUN,8,tmp16_stitchs>>8,tmp16_stitchs);
				 
				 if( sewingcontrol_tail_flag >=2)
				 {
					  tmp_point = pat_point;
					  if( sewingcontrol_tail_flag == 2)
					  {
						  special_sewing(2,2,0);
						  while(motor.angle_adjusted >= 16)
		    		  		rec_com(); 
						  special_sewing(0,2,0);	
						  while(motor.angle_adjusted >= 16)
		    		  	  	rec_com();
					  }
					  else if( sewingcontrol_tail_flag == 3)
					  {
						  special_sewing(2,3,0);
						  while(motor.angle_adjusted >= 16)
		    		  		rec_com(); 
						  special_sewing(0,3,0);	
						  while(motor.angle_adjusted >= 16)
		    		  	  	rec_com();
					  }
					  else if( sewingcontrol_tail_flag == 4)
					  {
						  special_sewing(2,4,0);
						  while(motor.angle_adjusted >= 16)
		    		  		rec_com(); 
						  special_sewing(0,4,0);	
						   while(motor.angle_adjusted >= 16)
		    		  		rec_com();
					  }
					  pat_point = tmp_point;
				 }
				if( (sewingcontrol_flag >=2)&&(sewingcontrol_stitchs !=0) )
				     need_backward_sewing = 1;
			    if( sewingcontrol_flag == 1)
				    need_action_two = 1;
				trim_action();				
			    inpress_up();	
			    cut_flag = 0;
				if( inpress_type == AIR_INPRESS)
			   		delay_ms(50);
			
	           break;
	      	}
	      	//--------------------------------------------------------------------------------------
	      	//  no cut and stop
	      	//-------------------------------------------------------------------------------------- 
	    	if(stop_flag == 1) 
	      	{           
				#if FOLLOW_INPRESS_FUN_ENABLE
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
			    	//2018-8-4
			    	//���������ǰ����ı���ѹ�������߶ȵĹ��ܣ��ҵ�ǰ��������ָ���������ڣ���ôʹ��ָ�����涯��Χ
					movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
				#endif
				set_func_code_info(RUN,9,0,0);
				switch(stop_number)
				{
					case 1:
					case 2:
						motor.stop_angle = 	k61;
						break;
					case 3:
					case 4:
					    motor.stop_angle = 	k61;
						break;
					case 5:
					case 6:
						motor.stop_angle = 	512;
						break;
				 	default:
						break;
				}
				if(motor.stop_angle >= 1024)
				{
					motor.stop_angle = motor.stop_angle - 1024;
				}
			  	motor.spd_obj = 0; 
				
				 
				while(motor.stop_flag == 0)
				{
					rec_com();
					if(sys.status == POWEROFF)  
				       return;
				}
				delay_ms(20);
				if(stop_number == 3 || stop_number == 4) //2013-7-29 add
				{
				   temp16 = motor.angle_adjusted;			
     			   if( (temp16 > DEADPOINT)&&(temp16 < DEGREE_180) )
					 motor.dir = 1;
	   			   else
	  	 			 motor.dir = 0;
					
	    		   motor.spd_obj = 120;//DEADPOINT_SPD;	
	   			   while(1)
	    		   {
	    				rec_com(); 
		 			  	if(motor.spd_ref == motor.spd_obj)
		  			 	{
		  			 	 	break;
		  			 	}
	    		   }  
				   motor.stop_angle = DEADPOINT;
		 		   if(motor.stop_angle >= 1024)
		   			  motor.stop_angle = motor.stop_angle - 1024;
				
				   motor.spd_obj = 0;  
	   			   while(motor.stop_flag == 0)    
	   			   {
	    		   		rec_com(); 
	   			   }	
				}
				if(k03 == 1)    
				{
					temp_tension = 0;       
      				at_solenoid();         
			    }
      			
				if(stop_number == 1 || stop_number == 2 || stop_number == 3 || stop_number == 4)
				{       
	        		inpress_up();     
				}  
				
				if((stop_number%2 == 1)&&(u41 == 0)) 
				{
					footer_both_up();   
				}
				do_pat_point_sub_one();     
	        	break;
	      	}    
			
			if(PatternDelayFlag == 1)
			{
			    #if FOLLOW_INPRESS_FUN_ENABLE
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
			    	movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
				#endif
				sewing_stop(); 
				while(motor.stop_flag == 0)
				{
					rec_com();
					if( (sys.status == ERROR)&&(stay_flag==0) )
					{
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
						return;
					}
					if(sys.status == POWEROFF) 
				       return;
				}
				set_func_code_info(RUN,10,0,0);    
				delay_ms(20); 
				if(u42 == 1)  
				{
					find_dead_point();
				}
				inpress_up();    
				delay_ms(PattenrDelayTime);
				PatternDelayFlag = 0;
				break;
			}
			// ------------------------------------------------------------------------------------------
			// check the last stitch in normal line/circle sewing without cut code or stop code
			// ------------------------------------------------------------------------------------------ 
			if(end_flag == 1)
			{  
				#if FOLLOW_INPRESS_FUN_ENABLE
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
			    	movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
				#endif
				temp16 = motor.angle_adjusted;
      			while(temp16 <= 853) 
				{ 
        			rec_com();                                // communication with panel                                               
        			temp16 = motor.angle_adjusted;          
        		}            
        		process_flag = 0;	                    
 	      		sewing_stop();  
				 
				while(motor.stop_flag == 0)
				{
					rec_com();
				}
				set_func_code_info(RUN,11,0,0);
			
				if(u42 == 1)
				{
					delay_ms(100);
					find_dead_point();
				}
				
				if(k03 == 0) 
				{
					da0 = 0;	            
					SNT_H = 0;             
				}			  
				else                       
				{ 
					temp_tension = 0;     
      				at_solenoid();         
				}
      			
				laststitch_flag = 0;
				do_pat_point_sub_one();
				delay_ms(50);       
	        	inpress_up();              
	        	break;
			}
	
			//if( (milling_cutter_action_flag == 1 )&&(rotated_cutter_enable==1)&&(ROTATED_CUTTER == 1) )
			if( milling_cutter_action_flag == 1 )
			{
				sewing_stop();
				while(motor.stop_flag == 0)
				{
					rec_com();
				}
				delay_ms(50);    
	        	inpress_up(); 
				delay_ms(50);
				break;
			}
			
			// ------------------------------------------------------------------------------------------
			// check the last stitch in normal line/circle sewing without cut code or stop code
			// ------------------------------------------------------------------------------------------ 
			if(nopmove_flag == 1)
			{	                       	
				#if FOLLOW_INPRESS_FUN_ENABLE
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
					movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
				#endif
				sewing_stop();  
				 
				while(motor.stop_flag == 0)
				{
					rec_com();
				}
				tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
				set_func_code_info(RUN,12,tmp16_stitchs>>8,tmp16_stitchs);
	    
			    start_to_speed_down = 0;
				cutter_speed_done_flag = 0;
				if(u42 == 1)
				{
					find_dead_point();
				}
				
			 	if(k03 == 0)  
			    {
				   da0 = 0;
				   SNT_H = 0; 
				}
			  	else                         
			    {                         
				   temp_tension = 0;     
      			   at_solenoid();         
				 }      			
				laststitch_flag = 0;
				nopmove_flag = 0;
				do_pat_point_sub_one();
	     
	        	inpress_up();  
				delay_ms(50);    
			     
	        	break;//����������ѭ���ˣ����뻨����ѭ��
			}
			
			#if FOLLOW_INPRESS_FUN_ENABLE
			if( (inpress_act_flag == 1) && (inpress_high != inpress_position ) )
			{
				inpress_to_forsingle(inpress_high);
				inpress_act_flag = 0;
			}
			#endif
	      	//--------------------------------------------------------------------------------------
	      	//  move stepper motor
	      	//--------------------------------------------------------------------------------------   	              	                   	    
	      	if(move_flag == 1)
	      	{
				#if SEND_SERVO_PARA_ONLINE
				if( k114 != 0)
				{
					//SUM = 1;
					tmpspd = motor.spd_obj /100;
					send_servo_parameter(tmpspd);
				}
				else if( chang_parameter_flag ==0)
				{
					chang_parameter_flag = 1;
					close_servo_parameter_function();
				}
				#endif 
				
				
				#if FOLLOW_INPRESS_FUN_ENABLE				
				inpress_up_angle   = angle_tab[inpress_follow_up_angle];
				inpress_down_angle = angle_tab[inpress_follow_down_angle];
				
				action_flag0 = 1;
				action_flag1 = 1;
				action_flag2 = 1;
				action_flag3 = 1;
				action_flag4 = 1;
				
				max_angle = movestepy_angle;
				if( movestepx_angle > movestepy_angle )
					max_angle = movestepx_angle;
				if( inpress_up_angle > max_angle )
				    max_angle = inpress_up_angle;
				if( movect_angle  > max_angle )
				    max_angle = movect_angle;
				/*
				  ��������½��Ƕȱ������Ƕȴ󣬲�����һ���Ѿ������½��ˣ���ʾ��ǰ������½���ʵ�Ѿ�ִ�й���
				*/	
				
                if( (inpress_down_angle > inpress_up_angle )&&(last_stitch_down ==1) )
				     action_flag0 = 0;	//������Ϊ��Щʱ����ѹ��������һȦ��û�н����Ϳ�ʼ�½���
				else if( (inpress_down_angle > inpress_up_angle )&&(last_stitch_down ==0))//֮ǰ10�ȣ������340�ȣ��Ǿ�����ִ��
				{
					inpress_down_angle = 5;
				}
				
				/*
				  ���ǰһ����½���ǰִ���ˣ�340�ȵ��������������10�ȵ������
				*/
				
				temp16 = motor.angle_adjusted;
				//max_angle
				while( temp16 < 1000 )//350d
				{
					temp16 = motor.angle_adjusted;
					if( temp16 >=1000)
					    break;				    
						
					if( (temp16 > inpress_down_angle ) &&( action_flag0 == 1) )//��ѹ���½��Ƕ�
					{
						action_flag0 = 0;
					
						if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
						{
							//test_flag =1;							
							//if( movezx_delay_flag == 1)
							  //while( movezx_delay_counter > 0);
							movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
							inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
							//movezx_delay_counter = inpress_follow_down_speed ;
							//movezx_delay_flag =1;
							//set_func_code_info(RUN,13,0,0);
							
						}						
					}
					
					if( (temp16 > inpress_up_angle ) &&( action_flag1 == 1) )
					{
						action_flag1 = 0;
						//���ǰ���벻�������ԣ����ﲻ��Ҫ������ѹ�Ž��£���ΪֻҪ������ѹ�Ų�̧����ô�����併��û�б�Ҫ�����˳���
						#if FIRST_STITCH_NOT_ACTION 
						if( (inpress_follow_high_flag == FOLLOW_INPRESS_LOW )&&(stitch_counter > inpress_not_working_stitchs+1) )
						#else
						if( inpress_follow_high_flag == FOLLOW_INPRESS_LOW )
						#endif
						{
							//test_flag =21;							
							//if( movezx_delay_flag == 1)
							   //while( movezx_delay_counter > 0);		
							#if INPRESSER_LOWER_FUN //ǰ�����½���΢��һЩ��sewingcontrol_stitchs_abs�����ӹ������ľ���ֵ
						    if( (stitch_counter > sewingcontrol_stitchs_abs )&&(flag1 == 1) )
							{
								if( inpress_high_hole > 0)//֮ǰ������һ�����͸߶ȣ����ڻָ�����
			 					{
									flag1 = 0;//��ֻ֤�����һ��
									//�ظ������������λ�ã�֮ǰ��Ϊ���λ���½���һ�룬�����涯�߶�û�䣬�������λ��Ҳ������һЩ
								 	movestep_zx(inpress_follow_range + inpress_position_tmp,inpress_follow_up_speed);									
								}
								else//֮ǰ������һ����涯�߶ȣ����ڻָ�����
								{
									flag1 = 0;
									inpress_follow_range = inpress_position_tmp;
									movestep_zx(inpress_follow_range,inpress_follow_up_speed);									
								}
								//turnoff_buz();				 
							}
							else
							{
							    movestep_zx(inpress_follow_range,inpress_follow_up_speed);								
							}
							#else//û��ʹ��ǰ�����½���΢��һЩ�Ĳ���
							
							//2018-8-4
					    	//���������ǰ����ı���ѹ�������߶ȵĹ��ܣ��ҵ�ǰ��������ָ���������ڣ���ôʹ��ָ�����涯��Χ
					    	//�˴��������ڽ���ѹ���涯��Χ�ָ�������ֵ
							if(para.start_sew_change_inpress_high_enable == 0x55 
								&& stitch_counter > para.start_sew_change_inpress_high_stitchs
								&& flag1 == 1)
							{
								flag1 = 0;//��ֻ֤����һ��
								inpress_follow_range = inpress_position_tmp;//�ָ�����������ѹ���涯��Χ
							}

							movestep_zx(inpress_follow_range,inpress_follow_up_speed);
							#endif
							inpress_follow_high_flag = FOLLOW_INPRESS_HIGH;
							//movezx_delay_counter = inpress_follow_up_speed ;
							//movezx_delay_flag =1;
									
							//========================================
							stitch_counter++;
							pat_point++;
							check_data(0);//������������ת�٣������ı�ʵ������ת��
								inpress_down_angle = angle_tab[inpress_follow_down_angle];
								pat_point--;
								stitch_counter--;
								if( inpress_down_angle > inpress_up_angle)//Ĭ����������½�С����ʼ��
								{
									action_flag0 = 1;//�����ٴ��½�����һ����½�������һȦ�Ϳ���ִ����									
									last_stitch_down = 1;//��ʾ��һ���Ѿ�����һ��ѹ�����µĶ��������
								}
								else
									last_stitch_down = 0;
							//========================================
						}
						
					}
					
					if( (xstep_cou != 0) &&(temp16 > movestepx_angle ) &&( action_flag2 == 1) )
					{
						if( movestepx_delay_counter ==0 )
						{							
							movestep_x(-xstep_cou);					
							movestepx_delay_counter = timer_x + 1;
							movestep_x_flag = 1;
							//test_flag = 1;
							action_flag2 = 0;
							allx_step = allx_step + xstep_cou;  
							//set_func_code_info(RUN,15,0,0);
						}
					}							
					if( (ystep_cou != 0) &&(temp16 > movestepy_angle ) &&( action_flag3 == 1) )
					{
						if( movestepy_delay_counter ==0 )
						{						
							movestep_y(-ystep_cou);					
							movestepy_delay_counter = timer_y + 1;
							movestep_y_flag = 1;
							//test_flag = 1;
							action_flag3 = 0;
							ally_step = ally_step + ystep_cou;
							//set_func_code_info(RUN,16,0,0);
						}
					}
					if( (clamp_com == 1) && (stitch_counter <= u33) )     // with clamp thread
        			{
						if( (temp16 > movect_angle)&&(action_flag4 == 1) )
      			  		{		
      			    		if( sewingcontrol_flag !=1 )
				        		move_ct();
							action_flag4 = 0;
						}
					}
					
				}
				//action_flag1==1��ʾ��һȦ�涯��ѹ��û��̧��
				//��ʱӦ��ǿ��̧����ѹ��
				if( (inpress_up_angle > 1000 ) &&( action_flag1 == 1) )
				{
					inpress_follow_high_flag = FOLLOW_INPRESS_HIGH;
					movestep_zx(inpress_follow_range,inpress_follow_up_speed);
				}
				move_flag = 0;
				rec_com();

				
				#else//�涯��ѹ�Ź���δ��ʱ��������ĳ��򣬴�ʱ��ѹ��������������
				
				if( (clamp_com == 1) && (stitch_counter <= u33) )     // with clamp thread
        		{
      				//set_func_code_info(RUN,17,0,0);
					if(movestep_angle <= movect_angle)
      			  	{		
      			    	move_xy(); 
						if( sewingcontrol_flag !=1 )
				        	move_ct();                
				    }
				    else
				    {        	        
				    	if( sewingcontrol_flag !=1 )
							move_ct();							 
				        move_xy();
				    } 
        		}
	      		else                      // without clamp thread
	      	    {
					 //set_func_code_info(RUN,18,0,0);
					 if( (movestep_angle < 768)||(xy_move_mode >= 1) )  
					 {
	        			       //1��move x&y
	        			       move_xy();
							   
	        					//2��move inpress   10d ensure minest 1235us(2700rpm) transmit x done(400us)
					    		temp16 = motor.angle_adjusted; //2011-3-1
	        	     			while((temp16 <= 798) || (temp16 >= 1000))    //  798---280 degree
	        	     			{
	        	     				rec_com();    // communication with panel 
									 
	        	     	           	if(sys.status == POWEROFF)   //2012-11-1 add 
				                       return;
									   
						 		   	temp16 = motor.angle_adjusted;      	                                                    
	        	     			}
	        	     			if(u103 != 0)
	        	     			{
	        						inpress_to_forsingle(inpress_high);
	        					}
					  }
					  else    
					  { 
						  temp16 = motor.angle_adjusted;                
	        	     	  while((temp16 <= 735) || (temp16 >= 1000))    //  735---258 degree
	        	     	  {
	        	     	  	rec_com();     
							 
	        	     	   	if(sys.status == POWEROFF)    
				               return;
									   
						    temp16 = motor.angle_adjusted;       	                                                    
	        	     	  }
						  if(u103 != 0)
	        	     	  {
	        			   	  inpress_to_forsingle(inpress_high);
	        			  }	 
						  move_xy();
					  }	        
	      		}     	        
				if(sys.error == ERROR_15) 
				{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
					while(motor.stop_flag == 0)
					{
						rec_com();
					}
					delay_ms(50);
					inpress_up();
					return;
				} 
				#endif
	      	}
	      	rec_com();     
			/*                              
	      	if(super_pattern_flag == 1)  // in super pattern mode need check receive data 
	      		{
	      		   if(pat_buff_total_counter % 8000 <= 4000 && pat_buff_total_counter % 8000 >= 3900 && \
		      	      pat_buff_write_offset == 0 || \
		      	      pat_buff_total_counter % 8000 <= 8000 && pat_buff_total_counter % 8000 > 7900 && \
		      	      pat_buff_write_offset == HALF_WRITE_OFFSET )  
		      	   {	      	   	    
	      	   	    	sys.status = ERROR;
		      	   		StatusChangeLatch = ERROR;
		      	   		sys.error = ERROR_81;   //data receive error		      	   		
		      	   }
	      		}
			*/	
			if(sys.status == ERROR)
		    {	   
				StatusChangeLatch = ERROR; 
				sewing_stop();
				while(motor.stop_flag == 0)
					rec_com();
					
				delay_ms(50);
				inpress_up();
				break;                               
		    }
			if(sys.status == POWEROFF)  
			{ 
				break;
			}
			
			//set_func_code_info(RUN,21,pat_buff_total_counter>>8,pat_buff_total_counter);

    	}
		if(stop_flag == 1)
		{
			StopStatusFlag = 1;
				break;
		}
		if(sys.status == ERROR)
		{ 
			StatusChangeLatch = ERROR; 
			break;
		}
	    if(sys.status == POWEROFF)  
		{ 
			break;
		}
  	}
	//--------------------------------------------------------------------------------------
	//  switch system status 
	//--------------------------------------------------------------------------------------  	    			        
	TestStatusChangeFlag = 0;
	if ( SUPPORT_CS3_FUN == 1)
		output_cs3(0x10,0x00); //x32.2
	else
		BLOW_AIR = 0;
	if(stop_flag == 1)
	{	
		sys.status = READY;
		StatusChangeLatch = READY;
		StopStatusFlag = 1;
	} 
	else
	{
		if(StatusChangeLatch == ERROR)
		{
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
			TestStatusChangeFlag = 0;
			
		}
		else if(sys.status == POWEROFF)  
		{
			set_func_code_info(RUN,19,0,0);
			return;
		}
		else
		{
			sys.status = FINISH;
			StatusChangeLatch = FINISH;
			TestStatusChangeFlag = 0;
			ready_go_setout_com = 0;
			predit_shift = 0;
			
		}
	}
}
//--------------------------------------------------------------------------------------
//  Name:		error_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of error stauts
//--------------------------------------------------------------------------------------
void error_status(void)
{		
	//-------------------------------------------------------------------------------------- 
  	// sound count                                                                  
  	//-------------------------------------------------------------------------------------- 
	sound_count++;
  	if(sound_count >= 50000)
  	{
  		sound_count = 0;
  	}
	if(motor.stop_flag == 0 || motor.spd_obj != 0)
	{
		sewing_stop();      	        	  
	    while(motor.stop_flag == 0)
		{
			rec_com();
		}
	}
	if ( alarm_output_enable >= 1)
	{
		RED_ALARM_LED = 1;
		if ( alarm_output_enable >= 2)
		{
			#if 0 
			YELLOW_ALARM_LED = 0;
			#endif
			GREEN_ALALRM_LED = 0;
		}
	}
	if ( SUPPORT_CS3_FUN == 1)
		output_cs3(0x10,0x0); //x32.2
	else
		BLOW_AIR = 0;
	FW = 0;
  	//-------------------------------------------------------------------------------------- 
  	// switch error number                                                                  
  	//--------------------------------------------------------------------------------------     		 
  	switch(sys.error)
  	{
  		//--------------------------------------------------------------------------------------
    	// emergency break
    	//--------------------------------------------------------------------------------------	 	         
    	case 2:  			// during the sewing process emergency break,next is emermove
			pause_flag = 0;
    	    stay_flag = 0;		
			//	flash_buz();	
            break; 
		case 45://���߱���
				flash_led();   		
            	flash_buz();
		break;
		case 51:
			flash_led();   		// flash alarm led                                                   
            flash_buz();	  	// flash alarm buzzer 
			if(PAUSE == pause_active_level)              
	        {			
	           	delay_ms(10);
				if(PAUSE == pause_active_level)
				{
					turnoff_ledbuz();					// turn off alarm led and buzzer 
	           		sys.status = READY;
					StatusChangeLatch = READY;
                	sys.error = OK; 
					pause_flag = 0;	
                	return;  
				}	    	    	      	    
             }
		break; 
		case 21:			// during the find origin process,next is free
			//--------------------------------------------------------------------------------------
            // turn on alarm led
            //-------------------------------------------------------------------------------------- 	    	           	       	  
    	  	flash_led();   		// flash alarm led                                                   
            flash_buz();	  	// flash alarm buzzer 
			pause_flag = 0;
            break;
		case 22:				// during time delay in aging process,next is ready
			pause_flag = 0;
            break;
		case 15: 				// out of sewing range
		    delay_ms(1000);
			sys.error = OK;	
			StatusChangeLatch = READY;
			sys.status = ERROR;
			break;	
    	//--------------------------------------------------------------------------------------
    	// thread breakage
    	//--------------------------------------------------------------------------------------	 	         
    	case 17: // thread breakage
		    flash_buz();	  	// flash alarm buzzer 
    	  	thbrk_flag = 0;    	       
    	    brkdt_flag = 0; 
			if( enable_thread_broken_alarm == 1)
			{  
				delay_ms(1000);
				turnoff_ledbuz();					
	           	sys.status = READY;
				StatusChangeLatch = READY;
                sys.error = OK; 
				pause_flag = 0;	
				stay_flag = 0;
                return;  
			}
            break;          
		//--------------------------------------------------------------------------------------
    	// dangerous error
    	//--------------------------------------------------------------------------------------	      
  		//case 3:
    	case 4:  // 300V undervoltage
  		//case 6:
    	case 7:  // IPM overcurrent or overvoltage
    	case 8:  // 24V overvoltage 
    	case 9:  // 24V undervoltage
  		//case 10:
  		//case 11:
  		//case 12:
    	case 13: // no motor encoder connect
    	case 14: // motor is not normal  
    		    
    	case 18: // cut knife is not in normal position	    	
    	case 20: // step motor software version problem 	 	     
    		//--------------------------------------------------------------------------------------
            // turn on alarm led
            //-------------------------------------------------------------------------------------- 	    	           	       	  
    	  	flash_led();   // flash alarm led                                                   
            flash_buz();	  // flash alarm buzzer  	      
     	    break;    	      
    	case 23: // catcher is not in normal position  	 	           
    	case 24: // panel is not matching   	 	           
    	case 25: // X origin sensor is not normal  	 	           
    	case 26: // Y origin sensor is not normal  	 	           
    	case 27: // press origin sensor is not normal  	 	             	 	           	
    	case 28: // catch thread origin sensor is not normal  	 	             	 	  
    	case 29: // inpress origin sensor is not normal  
    	case 30: // stepping motor driver communication is not normal 
    	case 31: // stepping motor overcurrent                        
    	case 32: // stepping motor driver power is not normal         	             	 	           	        	
    	case 33: // length is more than 12.7mm  
		case 34: // BLDC_ON alawys ON,abnormal current
		case 35: // IPM over cuurrent frequently
		case 36: // over cuurrent more than 10 times in 100ms
		case 37: // motor is stucked(condition 1) 
		case 38: // motor is stucked(condition 2) 
		case 39: // motor over speed
		case 40: // over current in stop status
		case 41: // motor over load
		case 42: // DC line abnormal
		case 11: // overspeed               2012-8-22
		case 12: // overdiffurence             
		case 74:
		case 75:
    					         
        	SNT_ON = 1;      // 24V disable                                  	           
           	emergency();     // emergency measure
           	//--------------------------------------------------------------------------------------
            // turn on alarm led and flash alarm buzzer 
            //-------------------------------------------------------------------------------------- 	          	 	  
    	    turnon_led();    // turn on alarm led	     	                                                      
            flash_buz();	    // flash alarm buzzer    		          		 
            break;
    	//--------------------------------------------------------------------------------------
    	// normal error
    	//--------------------------------------------------------------------------------------                                                        
    	case 5:  // 300V overvoltage
			#if SECOND_GENERATION_PLATFORM ==0 || USE_SC011N_PLATFORM == 0
			if(AC_OVDT)
    		{    					 	          	  
    	    	flash_led();   // flash alarm led                                                   
               	flash_buz();	  // flash alarm buzzer
    		}
    		else
			#endif
    		{
    			turnoff_ledbuz();	 // turn off alarm led and buzzer
    		}
			break;   
		case 10:					// fan error,valve error
			SNT_ON = 1;      // 24V disable
			//--------------------------------------------------------------------------------------
            // turn on alarm led
            //-------------------------------------------------------------------------------------- 	    	           	       	  
    	  	flash_led();   // flash alarm led                                                   
            flash_buz();	  // flash alarm buzzer 
			//if(T_OC == 0)
			//{
			//	SNT_H == 0;
			//	sys.error = OK;
			//	turnoff_ledbuz();					// turn off alarm led and buzzer
			//}
			break;	 
    	case 19: 										// pause button is not in normal position	
    	 	if(PAUSE != pause_active_level)              
	        {			
	           	delay_ms(10);
				if(PAUSE != pause_active_level)
				{
					turnoff_ledbuz();					// turn off alarm led and buzzer 
	           		sys.status = status_now;
					if( status_now != ERROR)
					   StatusChangeLatch = status_now;
					else
					   StatusChangeLatch = READY;
                	sys.error = OK; 
					pause_flag = 0;	
					stay_flag = 0;
                	return;  
				}	    	    	      	    
             }
             flash_led();   							// flash alarm led                                                   
             flash_buz();	  							// flash alarm buzzer  	      
     	     break;      
		case 98:
			 flash_led();                                                     
             flash_buz();
		case 48:		
			if(SFSW == 1)              
	        {			
	           	delay_ms(10);
				if(SFSW == 1)
				{
					turnoff_ledbuz();				
	           		sys.status = READY;
					StatusChangeLatch = READY;
                	sys.error = OK; 
					sfsw_flag = 0;	
				}	    	    	      	    
            }
		break;	  
		case 52: 
			flash_led();   // flash alarm led                                                   
            flash_buz();	  // flash alarm buzzer 
			if(PAUSE == pause_active_level)              
	        {			
	           	delay_ms(10);
				if(PAUSE == pause_active_level)
				{
					turnoff_ledbuz();
					sys.error = OK;	
					StatusChangeLatch = FREE;
					sys.status = FREE;
				}
			}
		break;		        
    	case 16: 										// needle is not in normal position  
    	    //--------------------------------------------------------------------------------------
            // turn on alarm led
            //-------------------------------------------------------------------------------------- 	    	           	       	  
    	  	flash_led();   // flash alarm led                                                   
            flash_buz();	  // flash alarm buzzer  	      
     	    break;   
		case 0:
			SNT_ON = 0;
			turnoff_ledbuz();					// turn off alarm led and buzzer
			sys.error = OK;	
			StatusChangeLatch = READY;
			sys.status = READY;
			break;	
		case 80:
		    if( OIL_EMPTY_DETECT == 0)
			{
				delay_ms(1000);
				if( OIL_EMPTY_DETECT == 0)
				{
					turnoff_ledbuz();
					sys.error = OK;	
					StatusChangeLatch = READY;
					sys.status = READY;
				}
			}
		break;
		case 82:
			turnon_led();    // turn on alarm led	     	                                                      
            flash_buz();	 // flash alarm buzzer 
		break;
		case 93:
			turnon_led();       	                                                      
            flash_buz();
			if( (ADTCSM)&&(PSENS ==1) )
			{
				autosewing_allset_flag = 1;
				turnoff_ledbuz();
				sys.error = OK;	
				StatusChangeLatch = READY;
				sys.status = READY;
			}
		break;
    	default: // no number error
    	   	SNT_ON = 1;      // 24V disable                                  	           
           	emergency();     // emergency measure
           	//--------------------------------------------------------------------------------------
            // turn on alarm led and flash alarm buzzer 
            //-------------------------------------------------------------------------------------- 	          	 	  
    	    turnon_led();    // turn on alarm led	     	                                                      
            flash_buz();	 // flash alarm buzzer    		          		 
            break;    	    	   	            	         
  	}
	predit_shift = 0;
	if(StatusChangeLatch != ERROR) 
	{
		turnoff_ledbuz();	// turn off alarm led and buzzer
		sys.error = OK;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		prewind_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of prewind stauts
//--------------------------------------------------------------------------------------
void prewind_status(void)
{	
	UINT16 temp16;
	UINT16 spd_tmp,i;
	while(motor.stop_flag == 0)    
  	{
    	rec_com();    // communication with panel                  	
  	}
  
	if(wind_com == 1)
	{	    
    //	go_origin_allmotor();	   // all step motor go origin  //10.08.30 wr zhushi
        footer_both_down();      //foot_down();    2012-5-18 modify
    	wind_com = 0;
  	}
 
 	if(inpress_flag == 0)
	{
		inpress_up();
	}
 	if(pedal_style == single_pedal)
	{
		if(StatusChangeLatch != PREWIND) 
		{
			predit_shift = 0;
			sys.status = StatusChangeLatch;
		}
		else
		{
			//--------------------------------------------------------------------------------------
		  	//  foot sensor
		  	//--------------------------------------------------------------------------------------
			if(pedal_state == 0 && pedal_last_state != 0)
			{
				delay_ms(10);
				if(pedal_state == 0)
				{
					pedal_last_state = 0;
				}
			}
			//--------------------------------------------------------------------------------------
		  	//  start sensor
		  	//--------------------------------------------------------------------------------------  
		  	if(pedal_state == 2)           // foot sensor is pushed
			{
				delay_ms(10);
				if(pedal_state == 2 && pedal_last_state == 0)
				{				
					if(foot_flag == 1)    // if foot is up
					{		 
	      
			    	}
			    	else                  // if foot is down
			    	{		
						pedal_last_state = 2;
					
			    		motor.dir = 0;	
			    		temp16 = u49 * 100;
						if(k02 ==1)
						{
							if(temp16 > MAXSPEED1*100 || temp16 < 200)
				    		{
				    			temp16 = 1600;
				    		}	
						}
						else if(k02 == 0)
						{
							if(temp16 > MAXSPEED0*100 || temp16 < 200)
				    		{
				    			temp16 = 1600;
				    		}
						}
						
						inpress_down(inpress_high_hole);
						delay_ms(20);
						motor.spd_obj = temp16; 
						sys.status = WIND;
						StatusChangeLatch = WIND;
						predit_shift = 0; 	   				    	
			    	}		    	    	      	    
			  	}
		  	} 
		} 
	}   
	else if(pedal_style == double_pedal)
	{	
		if(StatusChangeLatch != PREWIND) 
		{
			predit_shift = 0;
			sys.status = StatusChangeLatch;
		}
		else
		{
			//--------------------------------------------------------------------------------------
		  	//  start sensor
		  	//--------------------------------------------------------------------------------------  
		  	if(DVA == 0)           // foot sensor is pushed
			{
				delay_ms(10);
				if(DVA == 0 && DVALastState == 1)
				{				
					if(foot_flag == 1)    // if foot is up
					{		 
	      
			    	}
			    	else                  // if foot is down
			    	{		
			    		motor.dir = 0;
						temp16 = u49 * 100;
						if(k02 == 1)
						{
				    		if(temp16 > MAXSPEED1*100 || temp16 < 200)
				    		{
				    			temp16 = 1600;
				    		}	
						}
						else if(k02 == 0)
						{
							if(temp16 > MAXSPEED0*100 || temp16 < 200)
				    		{
				    			temp16 = 1600;
				    		}
						}
						
						inpress_down(inpress_high_hole);
						delay_ms(250);
						for(i=0;i<5;i++)
				    	{	  	  	  		     
							if(i==0)
							  spd_tmp = u10;	
							else if(i==1)  
							  spd_tmp = u11;	
							else if(i==2)
							  spd_tmp = u12 ;	
							else if(i==3)
							  spd_tmp = u13;	
							else if(i==4)
							  spd_tmp = u14;
							if(spd_tmp >= u49)
							   motor.spd_obj = u49*100;
							else
							   motor.spd_obj = spd_tmp*100;
							delay_ms(10);  
							while(motor.angle_adjusted >= 16)
					    		{
					    			rec_com();    						                
					    		}  	 
							while(motor.angle_adjusted < 16)
					    		{
					    			rec_com();    						                
					    		} 
						}
						motor.spd_obj = temp16;
						sys.status = WIND;
						StatusChangeLatch = WIND;
	        			predit_shift = 0;  
						DVALastState = DVA;   				    	
			    	}    	    	      	    
			  	}
		  	}
		 	else
			{
				delay_ms(10);
				if(DVA == 1)
				{
					DVALastState = DVA;
				}
			}
		}
	}
}
//--------------------------------------------------------------------------------------
//  Name:		wind_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of wind stauts
//--------------------------------------------------------------------------------------
void wind_status(void)
{	  
	if(pedal_style == single_pedal)
	{
		//--------------------------------------------------------------------------------------
	  	//  foot sensor
	  	//--------------------------------------------------------------------------------------
		if(pedal_state == 0 && pedal_last_state != 0)
		{
			delay_ms(10);
			if(pedal_state == 0)
			{
				pedal_last_state = 0;
			}
		}
	  	//--------------------------------------------------------------------------------------
	  	//  start sensor
	  	//--------------------------------------------------------------------------------------  
	  	if(pedal_state == 2)           // foot sensor is pushed
		{
			delay_ms(10);
			if(pedal_state == 2 && pedal_last_state == 0)
			{				
				if(foot_flag == 1)    // if foot is up
				{		 
	      
		    	}
		    	else                
		    	{	
					pedal_last_state = 2;
					sewing_stop();
				
					while(motor.stop_flag == 0)
					{
						rec_com();
					}
					delay_ms(80);
					
					if(k03 == 0) 
					  {
						  da0 = 0;	
					 	  SNT_H = 0;  
					  }
					
					inpress_up();	
		    		wind_com = 0;   
					sys.status = PREWIND;	
					StatusChangeLatch = PREWIND;
		    	}		    	    	      	    
		  	}
	  	} 
	}
	else if(pedal_style == double_pedal)
	{	
		//--------------------------------------------------------------------------------------
	  	//  start sensor
	  	//--------------------------------------------------------------------------------------  
	  	if(DVA == 0)           // foot sensor is pushed
		{
			delay_ms(10);
			if(DVA == 0 && DVALastState == 1)
			{				
				if(foot_flag == 1)    // if foot is up
				{		 
	      
		    	}
		    	else                  // if foot is down
		    	{	
					sewing_stop();
				
					while(motor.stop_flag == 0)
					{
						rec_com();
					}
					delay_ms(80);
					
				if(k03 == 0)
				{
					da0 = 0;
					SNT_H = 0; 
				}
					
					inpress_up();
		    		wind_com = 0;
					sys.status = PREWIND;
					StatusChangeLatch = PREWIND;
					DVALastState = DVA;
		    	}		    	    	      	    
		  	}
		 	DVALastState = DVA;
	  	}
	 	else
	 	{
			delay_ms(10);
			if(DVA == 1)
			{
				DVALastState = DVA;
			}
		}
	}	
		
	if(StatusChangeLatch != WIND) 
	{
		if(motor.spd_obj > 0)
		{
			sewing_stop();
			while(motor.stop_flag == 0)
			{
				rec_com();
			}
			delay_ms(80);
			
			if(k03 == 0) 
			 {		  
			       da0 = 0;	
				   SNT_H = 0;
			  }
			
			inpress_up();
    		wind_com = 0;
		}
		sys.status = StatusChangeLatch;
		predit_shift = 0;
	}        
}
//--------------------------------------------------------------------------------------
//  Name:		inpress_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of inpress stauts
//--------------------------------------------------------------------------------------
void inpress_status(void)
{	
	UINT8 temp8;
	//--------------------------------------------------------------------------------------
  	// presser down
  	//-------------------------------------------------------------------------------------- 	
	if( foot_flag == 1)
	{		 
	  	footer_both_down();     
	}

    if((k03 == 0) && (fk_status == OUT) && (fk_cut == 0))
	{
		da0 = 255;
		fk_count = 1;
		fk_status = IN;
		delay_ms(100);  
		da0 = release_tension_current;      
	}

	//--------------------------------------------------------------------------------------
  	// inpresser down
  	//-------------------------------------------------------------------------------------- 	
	if( inpress_act_flag == 1)
	{
		if( inpress_type != AIR_INPRESS )
		{
			#if SECOND_GENERATION_PLATFORM || CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
			 L_AIR = 1;
			 delay_ms(para_y_backward_dis);
			 //inpress_litter_footer_action_value = 1;
	    	 //inpress_litter_footer_action_counter = para_y_backward_dis;
			 //inpress_litter_footer_action_flag = 1;
			#endif
			inpress_to(inpress_high);
			inpress_flag = 0; 
	        delay_ms(120);
		 }
		else
	    {
   	        if( inpress_com == 0 ) 
			    inpress_down(0);             
		  	else  
		  	{
				temp8 = detect_position();	
	    		if(temp8 == OUT)    
	      		{
					find_dead_center();
	      		}
			    inpress_up();        
		  	}
	    }
		inpress_act_flag = 0;
		set_func_code_info(INPRESS,1,0,0);
	}
	
	predit_shift = 0;
	if(StatusChangeLatch != INPRESS)
	{
		predit_shift = 0;
		sys.status = StatusChangeLatch;
		
		if(StatusChangeLatch == READY && k03 == 0)
		{							
			da0 = 0;
			SNT_H = 0;  
			fk_cut = 0;
			fk_count = 1;
			fk_status = OUT;
		}
		
	}
}
//--------------------------------------------------------------------------------------
//  Name:		poweroff_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of power off stauts
//--------------------------------------------------------------------------------------
void poweroff_status(void)
{
  	#if SECOND_GENERATION_PLATFORM ==0
	SNT_ON = 1;       // u24 disable
  	PWR_LED = 1;      // power_on led disable
  	ALARM_LED = 0;    // alarm led disable
	SUM = 0;          // alarm buzzer disable
	#endif
	//--------------------------------------------------------------------------------------
  	// turn off all output
  	//-------------------------------------------------------------------------------------- 
	if(motor.spd > 10)
	{
		U=1;U_=1;V=1;V_=1;W=1;W_=1;
		prcr = 0x02;
		inv03 = 0;
		prcr = 0x00;
		U_=0;V_=0;W_=0;
	}
	else
	{
		U=1;U_=1;V=1;V_=1;W=1;W_=1;
		prcr = 0x02;
		inv03 = 0;
		prcr = 0x00;
		OUTPUT_ON = 1;
	}
	//--------------------------------------------------------------------------------------
  	// turn off all output
  	//-------------------------------------------------------------------------------------- 
	if(power_off_count < 10)
		power_off_count++;
	else
	{	
		#if ( SECOND_GENERATION_PLATFORM ==1 ) || (USE_SC011N_PLATFORM == 1) || (FIFTH_SC013K_PLATFORM == 1)
		
		#else
		if(AC_OVDT)
		{
			OUTPUT_ON = 1;
			sys.error = ERROR_05;
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
			return;
		}
		#endif
	}
	//--------------------------------------------------------------------------------------
  	// turn off all output
  	//-------------------------------------------------------------------------------------- 		
	#if (SECOND_GENERATION_PLATFORM ==1) || (USE_SC011N_PLATFORM == 1) || (FIFTH_SC013K_PLATFORM == 1)
	 
	#else
	if(PWR_ON)
  	{    
	    RST_PN = 1;								// reset control panel
	    prcr = 0x07;              				// Protect mode reg    
	    pm0 = 0x08;               				// Processor mode reg0 
	    prcr = 0x00;              				// Protect mode reg    
  	}  
	#endif
}
//--------------------------------------------------------------------------------------
//  Name:		single_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of single step stauts
//--------------------------------------------------------------------------------------
void single_status(void)
{	
	UINT8 temp8;
	INT32 tempx_step,tempy_step;
	UINT16 i;
	//--------------------------------------------------------------------------------------
  	//  inpresser up or down  
  	//-------------------------------------------------------------------------------------- 
	
	 need_action_once = 0;
	 need_backward_sewing = 0;
	 {
        	if(pedal_style == single_pedal)
	        {
	  	      //--------------------------------------------------------------------------------------
	  	      //  foot sensor
	  	      //-------------------------------------------------------------------------------------- 
		         if(pedal_state == 0 && pedal_last_state != 0)
		          {
			           delay_ms(10);
			         if(pedal_state == 0)
			           {
				           pedal_last_state = 0;
			           }
		           }
	  	         if(pedal_state == 1 && pedal_last_state == 0)           // foot sensor is pushed
		          {
			           delay_ms(10);
			          if(pedal_state == 1)
			           {			
				           pedal_last_state = 1;	
				           if(foot_flag == 1)
				           {		 
		      		          foot_down();
		    	            }
		  	             }
	  	            }
	           }
	        else if(pedal_style == double_pedal)
	           {
		             //if(TestStatusChangeFlag == 0)
		              {
		              	 if(DVB == 0)           
		 	              {
				              delay_ms(10);
				              if(DVB == 0 && DVBLastState == 1)
				               {				
					                if(foot_flag == 1)
						              {
						                    footer_both_down();  
						                    if(u104 == 1)
					                        {	      	
						                        delay_ms(50);
						                        inpress_down(inpress_high); 
					                         }							             	
						               }	    	    
						              else
						               {
							                  if(u104 == 1)
						                       {	      	
					                              inpress_up(); 
								                  delay_ms(150);
						                        }	
							                   footer_both_up(); 
						                }
			  		                   DVBLastState = DVB;		    	    	    	    
			  	                 }
		  	                 }
		  	              else
			                {
				                //delay_ms(10);
			                	//if(DVB == 1)
				                // {
					                 DVBLastState = DVB;
				                // }
			                }
		 	             if(k60 == 1)
			               {
			 	            if(DVSM == 0)           // foot sensor is pushed
				              {
					             delay_ms(10);
					             if(DVSM == 0 && DVSMLastState == 1)
					              {				
						              if(foot_half_flag == 1)
						               {		 
	              			      	       if(foot_flag == 0)     
											 foot_half_down();
				    	               }		    	    	    	    
				  	               }
			  	               }
			 	            else
				              {
					              delay_ms(10);
					             if(DVSM == 1)
					             {
					 	            DVSMLastState = DVSM;
					             }
				              }
			             }
		           }
	        }	
	       DVALastState = DVA;
    }
    if(super_pattern_flag == 0)
    {
       if ( PointShiftFlag == 1)
       {
			footer_both_down();
			inpress_up();
			delay_ms(120);
			tempx_step = 0;
			tempy_step = 0;
			pat_point = (PATTERN_DATA *)(pat_buf);
			SewTestStitchCounter = 0;
			for(i=0;i<PointShiftNumber;i++)
			{
					process_data();
					if( end_flag ==1 )
					{
						pat_point -- ;
						break;
					}
					tempx_step += xstep_cou;
					tempy_step += ystep_cou;
					if( (move_flag ==1)||(lastmove_flag == 1) )
					   SewTestStitchCounter++;
			}
			go_commandpoint(tempx_step,tempy_step);
			delay_ms(120);
			PointShiftFlag = 0;
			predit_shift =0 ;
	   }
	}
	else
	{
		if(coor_com  == 1)//0x77
	  	{
			temp8 = detect_position();	
			if(temp8 == OUT)   
	  		{
				find_dead_center();
	  		}
			if(inpress_flag == 0)
			{
				inpress_up();
				delay_ms(20);
			}
			delay_ms(50);
	     	go_commandpoint(comx_step,comy_step);
			predit_shift = 0;
			coor_com = 0;
	  	}
		if( PointShiftFlag == 1)//0x64
	    {	
		    pat_point = (PATTERN_DATA *)(pat_buf);
		    pat_point = pat_point + (PointShiftNumber%TOTAL_STITCH_COUNTER);
		    pat_buff_total_counter = PointShiftNumber;
		    PointShiftFlag = 0;
	    }
	}
    
  	//--------------------------------------------------------------------------------------
  	//  single step move 
  	//--------------------------------------------------------------------------------------
  	switch(single_flag)
  	{
  		case 0:	            
		     predit_shift = 0;    
			break;
  		case 1:	 
		

			SewingTestEndFlag = 1;  
			
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			
			allx_step = allx_step + xstep_cou;
  			ally_step = ally_step + ystep_cou;
			
			if( check_sewing_range())
			{
				allx_step = allx_step - xstep_cou;
  				ally_step = ally_step - ystep_cou;
				do_pat_point_sub_one();
				single_flag = 0;
				if(move_flag == 1 || nopmove_flag == 1)
				{
					move_flag = 0;
					nopmove_flag = 0;
				}
				sys.error = ERROR_15;
				StatusChangeLatch = ERROR;
			}
			else
			{	
				SewTestStitchCounter++;
				allx_step = allx_step - xstep_cou;
  				ally_step = ally_step - ystep_cou;
				if(nopmove_flag == 1)
				{	
					if(u103 != 0 && inpress_flag == 0)   
					{
						inpress_up();        	
					}
				}
				if(move_flag == 1)                                 
				{
					if(u103 == 2)
				     {
					     if(inpress_flag == 1)
					      {
					            inpress_down(inpress_high);
					      }
					     else
					      {
						       if(single_inpress_flag == 1 )     
					            {
						           inpress_to(inpress_high);  
						           inpress_flag = 0;  	   
						        }    		       
					       }
						  single_inpress_flag = 0;
					}  
				}
				move_next();
			}   
			SewingTestEndFlag = 0;   
			predit_shift = 0;                    
			break;
  		case 2:
			
      
			SewingTestEndFlag = 1;  
			SewTestStitchCounter--;	
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		} 
			
		    if(nopmove_flag == 1)
			 {
					if(inpress_flag == 0)  
					{
						inpress_up();        	
					}
			}
			if(move_flag == 1)
			{
				
				if(u103 == 2)              
			    {	
					if(inpress_flag == 1)  
					{
						inpress_down(inpress_high);        	
					}
					else
					{
						 if(single_inpress_flag == 1)     
					      {
						        inpress_to(inpress_high);  
						        inpress_flag = 0;           	
					       }
					}
					single_inpress_flag = 0;
				}
			}
			move_back(); 
			SewingTestEndFlag = 0; 
			predit_shift = 0;
			break;
  		case 3:	 
			inpress_high = inpress_high_hole;
			tension = tension_hole;
 
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			if(inpress_flag == 0)  		
			{
				inpress_up();        	
			}
			//if(u39 == 1)
			    go_origin_allmotor();
			//else
			//	move_startpoint();
		
			move_flag = 0;
			nopmove_flag = 0;
			predit_shift = 0;
			single_flag = 0; 
			TestStatusChangeFlag = 0;
			StopStatusFlag = 0;
			stop_flag = 0;
			if(u38 == 0)       
			foot_com = 1;  
			             
			break;		
  		case 4:	 
		
 
			SewingTestEndFlag = 1; 
			if(nop_move_pause_flag ==1)
		    	process_nop_move_pause(1);
			else
				go_sewingtest_beginpoint(); 
			SewingTestEndFlag = 0;   
			predit_shift = 0;
			single_flag = 0;
			break;			
  		case 5:	 
		
 
			SewingTestEndFlag = 1;  
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			inpress_up();  
				back_endpoint();
			SewingTestEndFlag = 0;  
			predit_shift = 0;
			single_flag = 0;  
			break;	
  		case 6:	 
 
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			course_next();  
			delay_ms(2);                    
			break;		
  		case 7:	 	
 
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			course_back();
			delay_ms(2);                
			break;			
  		case 8:	 
  
			course_stop();       
			predit_shift = 0;   
			            
			break;	
		case 9:
			
 
			temp8 = detect_position();	
    		if(temp8 == OUT)   
      		{
				find_dead_center();
      		}
			go_setoutpoint();
			predit_shift = 0;
			single_flag = 0; 
			break;
		case 10:
			SewTestStitchCounter = 0;
		
  
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			go_manualpoint();
			predit_shift = 0;
			single_flag = 0; 
			break;	
		case 11:	 
  
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			course_inpresser_next();  
			delay_ms(5);   
			if(course_run_stop_flag == 1)
			{	
				single_flag = 8;
				course_run_stop_flag = 0;
			}                 
			break;
		case 12:	 	

			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			course_inpresser_back();
			delay_ms(5);    
			if(course_run_stop_flag == 1)
			{	
				single_flag = 8;
				course_run_stop_flag = 0;
			}                  
			break;
		case 13:	 
 
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			course_thread_next();  
			delay_ms(5);   
			if(course_thread_run_stop_flag == 1)
			{	
				single_flag = 8;
				course_thread_run_stop_flag = 0;
			}                 
			break;
		case 14:	 	

			temp8 = detect_position();	
    		if(temp8 == OUT)   
      		{
				find_dead_center();
      		}
			course_thread_back();
			delay_ms(5);    
			if(course_thread_run_stop_flag == 1)
			{	
				single_flag = 8;
				course_thread_run_stop_flag = 0;
			}                  
			break;
  		default:                                     
			break;	
  	}
	if(CurrentPointShiftFlag == 1)
	{
		pat_point = (PATTERN_DATA *)(pat_buf);
		pat_point = pat_point + (CurrentPointShiftPosition%TOTAL_STITCH_COUNTER);
		pat_buff_total_counter = CurrentPointShiftPosition;
		CurrentPointShiftFlag = 0;
	}
  	//--------------------------------------------------------------------------------------
  	//  single step of inpress action
  	//--------------------------------------------------------------------------------------
    if((foot_flag == 0) && (single_comm == 0x03 || single_comm == 0x04 || single_comm == 0x83 || single_comm == 0x84))  //2012-9-7 modify
  	{
  		if(u103 != 0)
  		{
  			inpress_to(inpress_high);
  			inpress_flag = 0;              
  		}
  	}
	predit_shift = 0;//2014-8-15
	if(StatusChangeLatch != SINGLE) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
		
		if(StatusChangeLatch == READY && k03 == 0)
		{							
			da0 = 0;
			SNT_H = 0; 
			fk_cut = 0;
			fk_count = 1;
			fk_status = OUT;
		}
		
	}
}
//--------------------------------------------------------------------------------------
//  Name:		manual_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of manual move stepper motor stauts
//--------------------------------------------------------------------------------------
void manual_status(void)
{	
	if(pedal_style == single_pedal)
	{
	  	//--------------------------------------------------------------------------------------
	  	//  foot sensor
	  	//--------------------------------------------------------------------------------------
		if(pedal_state == 0 && pedal_last_state != 0)
		{
			delay_ms(10);
			if(pedal_state == 0)
			{
				pedal_last_state = 0;
			}
		}
	  	if(pedal_state == 1 && pedal_last_state == 0)          
		{
			delay_ms(10);
			if(pedal_state == 1)
			{				
				pedal_last_state = 1;
				if(foot_flag == 1)
				{		 
		      		foot_down();
		    	}
		    	else
		    	{
		    		footer_both_up(); 
		    	}		    	    	    	    
		  	}
	  	}
	}
	else if(pedal_style == double_pedal)
	{
		//--------------------------------------------------------------------------------------
	  	//  foot sensor
	  	//--------------------------------------------------------------------------------------
	  	if(DVB == 0)           
		{
			delay_ms(10);
			if(DVB == 0 && DVBLastState == 1)
			{				
			      if(k60 == 1)     
				  {
					   if(foot_flag == 0)
					    {
							 if(foot_half_flag == 1)
							   foot_up();
					    }
					 else
					   {
							  foot_down();
					  }
				 }
			   else
			     {
					 footer_procedure();
			     }
			     DVBLastState = DVB;	  	    	    	    
		  	}
	  	}
	    if(k60 == 1)   
		{
			if(DVSM == 0)
			{
				delay_ms(10);
				if(DVSM == 0 && DVSMLastState == 1)
				{				
					if(foot_half_flag == 1)
					{		 
			      		if(foot_flag==0) 
						  foot_half_down();	
			    	}
			  		else if(foot_half_flag == 0)
					{
		    			foot_half_up();	
		    		}
		   	    	DVSMLastState = DVSM;		    	    
			  	}
		  	}
		 	else
			{
				DVSMLastState = DVSM;
			}
		}
	}
	
	DVALastState = DVA;
	
  	//--------------------------------------------------------------------------------------
  	//  manual shift step  
 	 //--------------------------------------------------------------------------------------
  	switch(shift_flag)
  	{
	  	case 0x0C: shift_12();   speed_up_counter = 0; break;	
	    case 0x01: shift_01();   speed_up_counter = 0; break;    		            							
	    case 0x03: shift_03();   speed_up_counter = 0; break;  		
	    case 0x04: shift_04();   speed_up_counter = 0; break;		
	    case 0x06: shift_06();   speed_up_counter = 0; break;    		            							
	    case 0x07: shift_07();   speed_up_counter = 0; break;		
	    case 0x09: shift_09();   speed_up_counter = 0; break;	
	    case 0x0A: shift_10();   speed_up_counter = 0; break;    	 
		case 0xEA: remove_10();  delay_process();  break; 			            									  		            								
	    case 0xEC: remove_12();  delay_process();  break;	
	    case 0xE1: remove_01();  delay_process();  break; 		            							
	    case 0xE3: remove_03();  delay_process();  break;  	
	    case 0xE4: remove_04();  delay_process();  break;		
	    case 0xE6: remove_06();  delay_process();  break;   			            							
	    case 0xE7: remove_07();  delay_process();  break;		
	    case 0xE9: remove_09();  delay_process();  break;		
	    case 0x88: remove_stop(); speed_up_counter = 0;break; 								
	    default:                  break;	
  	}
	if(StatusChangeLatch != MANUAL) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		setout_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of setout stauts
//--------------------------------------------------------------------------------------
extern void reset_uart1(void);
void setout_status(void)
{
    return_from_setout = 0;
	bar_coder_refresh_flag = 0;
	over_counter = 0;
    inpress_high = inpress_high_hole;
    tension = tension_hole;

	find_x_origin_counter++;
	already_find_startpoint_flag = 0;
	if( k115 == 0)
		already_auto_find_start_point = 0;
		
	if(finish_nopmove_pause_flag ==1)
	{
		sys.status = ERROR;
		StatusChangeLatch = ERROR;
	    sys.error = ERROR_02; 
		status_now = READY;
		return;
	}
#if CHANGE_DOUBLE_FRAMEWORK 	
		  if( current_running_flag == 1)//���������
		   {
			   return_frame_back(1);//����ͻ�ģ��
			   left_quest_running = 0;
		   }
		   else if( current_running_flag == 2)
		   {
			   return_frame_back(2);//�ұ��ͻ�ģ��
			   right_quest_running = 0;
		   }
		   current_running_flag = 0;
		   autosewing_allset_flag = 0;
		   delay_ms(50);
		   go_origin_allmotor();
		   serail_number = 0;
		   pattern_change_flag = 0;
		   #if DMA_UART1_MODE
				reset_uart1();
		   #else
				rec1_ind_w = 0;
			#endif
		   
#else   
	end_flag = 0;
	if( (PointShiftFlag == 1)&&(super_pattern_flag ==1) )
	{	
		pat_point = (PATTERN_DATA *)(pat_buf);
		pat_point = pat_point + (PointShiftNumber%TOTAL_STITCH_COUNTER);
		pat_buff_total_counter = PointShiftNumber;
		PointShiftFlag = 0;
	}
	
	if(u39 == 1)//����ԭ��									
	{	
	  		switch(u37)  										
	  		{
	  			case 0: 
					    if( super_pattern_flag == 0)
						   go_origin_xy();	
	                	pat_point = (PATTERN_DATA *)(pat_buf);
				  		sta_point = pat_point;
						pat_buff_total_counter = 0;
						#if AUTO_CHANGE_PATTERN_FUNCTION ==0
					
						if(u38 == 0)
						{
	                	    footer_both_up();
						}
						#endif
					break;
	              
	  			case 1: 
				#if AUTO_CHANGE_PATTERN_FUNCTION ==0
						if(u38 == 0)
						{
	                	   footer_both_up(); 
						}
						#endif
						if(super_pattern_flag == 0)
						   go_origin_xy();	
						pat_point = (PATTERN_DATA *)(pat_buf);
				   		sta_point = pat_point;
	               	 	pat_buff_total_counter = 0;
						predit_shift = 0;	
		              	break;
		  			case 2:
						if(super_pattern_flag == 0)
							go_origin_xy();	
	                	pat_buff_total_counter = 0;
					pat_point = (PATTERN_DATA *)(pat_buf);
			   		sta_point = pat_point;
					predit_shift = 0;
					FootUpCom = 1;
                	break;
	  			
	  				default: break;		
	  			}
	 	}
	 	else	//���������ԭ��Ļ�
	 	{
	  			switch(u37)
	  			{
	  				case 0:
						go_setoutpoint();
						#if AUTO_CHANGE_PATTERN_FUNCTION ==0
						if(u38 == 0)
						{
	                	   footer_both_up(); 
						}
						#endif
	              		break;
	  				case 1: 
					#if AUTO_CHANGE_PATTERN_FUNCTION ==0
						if(u38 == 0)
						{
	                	    footer_both_up();
						}
						#endif
						delay_ms(10);
	              		go_setoutpoint();
						
	              		break; 			
	  				case 2: 
						go_setoutpoint();
						FootUpCom = 1;
                		break;
	  			
	  				default: break;		
	  		}
	  }
 
  		
	if(k110 == 1)         
	{
		if(R_AIR == 1)
		{
		   R_AIR = 0;
		   delay_ms(80);  
		}
	}
#endif
		
	speed_down_stitchs = 0;
	start_to_speed_down= 0;
	cutter_speed_done_flag = 0;
	speed_down_counter = 0;	
	nop_move_pause_flag = 0;
	predit_shift = 0;
	if( second_start_switch == 1)
	{
		second_start_counter = 1;
	}
	if( sewingcontrol_flag == 1)
	    need_action_two = 1;
	//--------------------------------------------------------------------------------------
  	//  switch system status 
  	//--------------------------------------------------------------------------------------  	    			        
	if ( ready_go_setout_com == 1)
	{
		ready_go_setout_com = 0;
		predit_shift = 0;
	}
	monitor_allx_step = 0;
	monitor_ally_step = 0;
	monitor_pat_point = (PATTERN_DATA *)(pat_buf);
	monitor_refresh_flag = 0;
	
	
	
	#if AUTO_CHANGE_PATTERN_FUNCTION

	AUTO_NOTIFY_READY_FOR_CHANGE = 0;//�������ƽ����ˣ�Ԥ�����0
	AUTO_NEED_CHANGE_NOW = 1;		 //������ɣ�����ȡ��ģ��
	while( AUTO_ARM_HOLING_WELL == 0)//��е���Ѿ�ץ����
	{
		if(PAUSE == pause_active_level)
		   break;
		rec_com();
	}
	foot_up();
	delay_ms(100);
	AUTO_ALLOW_TAKE_OUT_FRAMEWORK = 1;//�ſ�ѹ�������е��ȡ��ģ��
	delay_ms(300);
	AUTO_NEED_CHANGE_NOW = 0;
  
	AUTO_FIRST_ASK_FRAMEWORK = 1;//���¿�ʼ����
	SUM = 0;
	autosewing_allow_working = 0;
	autosewing_offset_moveing_done = 0;
    autosewing_switch_last_status =0;
	autosewing_allset_flag = 0;
	predit_shift = 0;
	StatusChangeLatch = READY;
	sys.status = READY;   
	
	#else
	
	if(sys.status == ERROR)
	{
		
	}	
	else
	{
        if(u35==0)                    
        {
        	if(clamp_com == 1)           // check clamp flag
        	{
				go_origin_ct();
        		if(clamp_flag == 0)        // clamp is in
    			{
    				delay_ms(20);
					clamp_out();
    				clamp_stepflag = 1;      // clamp thread step 1
          			movect_angle = 800;      // 800---281 degree
    			}
    			if(TSENS == 0)             // sensor is covered
	    		{	
	    			delay_ms(100);
        			if(TSENS == 0)
        			{
      					sys.status = ERROR;
						StatusChangeLatch = ERROR;
						if( sys.error == 0)
        				sys.error = ERROR_23;	 // clamp is not in normal position  		  
        				return;
        			}
      			}
        	}
        	else
        	{
        		if(clamp_flag == 1)        // clamp is out
        		{
          			sys.status = ERROR;
					StatusChangeLatch = ERROR;
					if( sys.error == 0)
          			sys.error = ERROR_23;	   // clamp is not in normal position	  
          			return;
        		}
        		else
        		{
          			clamp_stepflag = 0;      // clamp thread step 0
        		}
        	}
        } 
	
		StatusChangeLatch = READY;
	  	sys.status = READY;   
	}
	#endif
}
//--------------------------------------------------------------------------------------
//  Name:		emerstop_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of emergency stop stauts
//--------------------------------------------------------------------------------------
void emerstop_status(void)
{	
	INT16 temp16;
	INT8 temp8;
	
	if(OutOfRange_flag == 0)
	{
		if(u97 == 1)
		{
			if(pedal_style == single_pedal)
			{
				//--------------------------------------------------------------------------------------
		  		//  start sensor
		  		//-------------------------------------------------------------------------------------- 
				if(pedal_state == 0 && pedal_last_state != 0)
				{
					delay_ms(10);
					if(pedal_state == 0)
					{
						pedal_last_state = 0;
					}
				} 
	            
			}
			else if(pedal_style == double_pedal)
			{
				
			} 
		}
		else if(u97 == 0)
		{
			sys.status = EMERMOVE;  
			StatusChangeLatch = EMERMOVE;   
			emermove_high = IN_ORIGIN;	 
		}
	}
	else if(OutOfRange_flag == 1)
	{
		sys.status = EMERMOVE;  
		StatusChangeLatch = EMERMOVE;   
		emermove_high = IN_ORIGIN;
	}
	
	
	if(StatusChangeLatch != EMERSTOP)
	{
		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		preedit_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of preedit stauts
//--------------------------------------------------------------------------------------
void preedit_status(void)
{
	INT16 temp16;
	INT16 i;
	UINT8 temp8;
	UINT16 cnt;
	INT32 allx_step_tmp,ally_step_tmp;
	PATTERN_DATA *TempStart_pointTemp;
	need_backward_sewing = 0; 
	//--------------------------------------------------------------------------------------
  	// config main motor driver add for 210E
  	//-------------------------------------------------------------------------------------- 
  	initial_mainmotor();
  	if(sys.status == ERROR)
  	{
  	  return;
  	}
	//--------------------------------------------------------------------------------------
  	//  find origin
  	//-------------------------------------------------------------------------------------- 
	if(origin_com == 1)
	{		
		pat_point = (PATTERN_DATA *)(pat_buf);
		temp8 = detect_position();	
		if(temp8 == OUT)    
  		{
			find_dead_center();
  		}
		if(inpress_flag == 0)
		{
			inpress_up();
			delay_ms(20);
		}
		if( (milling_first_move == 1)  &&( (x_bios_offset!=0)||(y_bios_offset!=0) ) )
			{ 
				milling_first_move = 0;
				allx_step_tmp = allx_step;
				ally_step_tmp = ally_step;
	      	    go_commandpoint(allx_step - x_bios_offset,ally_step - y_bios_offset);	
				allx_step = allx_step_tmp;
				ally_step = ally_step_tmp;
			}	
	  	go_origin_allmotor();	     	// all step motor go origin  
		
	  	if(sys.status == ERROR)
    	{
      		return;
    	}
		if( ready_go_setout_com == 0)
			predit_shift = 0;
		origin_com = 0;
  	}
    if ( ready_go_setout_com == 1)
	{
			footer_both_down();
			inpress_up();
			delay_ms(120);
			if(AUTO_SEARCH_START_POINT == 1)
			{
				if(already_auto_find_start_point == 0)//��һ�ο�ʼ����
			  	{
					find_start_point();
					already_auto_find_start_point = 1;
					/*ԭΪ120ms����ֹ��û�б���ͽ�����һ�׶Σ����Ժ���Ż��Ĺ����У�
					 *Ӧ������ô�ڹ��ϴ����б�������⡣
					 */
					delay_ms(200);
			  	}
			}
			if( (allx_step != 0 )||(ally_step != 0) )
			{
				pat_point = (PATTERN_DATA *)(pat_buf);
				pat_buff_total_counter = 0;//2017-8-16
				nopmove_flag = 0;
				move_flag = 0;
				allx_step_temp = 0;
				ally_step_temp = 0;
				for( cnt=0; cnt<TOTAL_STITCH_COUNTER;cnt++)
				{
					process_data(); 
				    if( nopmove_flag == 1)
					{
						allx_step_temp = allx_step_temp + xstep_cou;
						ally_step_temp = ally_step_temp + ystep_cou;
						nopmove_flag = 0;
					}
					else if( (second_point_passby_flag == 0)&&(origin2_lastmove_flag == 1) )
							origin2_lastmove_flag = 0;
					else
					{
						if( move_flag == 1)
							already_find_startpoint_flag = 1;
						do_pat_point_sub_one();  	    	
    					break; 
					}
				}
				if( (allx_step_temp != allx_step)||(ally_step_temp != ally_step) )
					go_commandpoint(allx_step_temp,ally_step_temp);
		    }
			else
				go_startpoint();
			PointShiftFlag = 0;
			predit_shift =0 ;
		    ready_go_setout_com = 0;
	}
 
 	if(MotiongSetFlag == 1)
	{
		temp8 = detect_position();	
		if(temp8 == OUT)    // out of range 
  		{
			find_dead_center();
  		}
		if(inpress_flag == 0)
		{
			inpress_up();
			delay_ms(20);
		}
		switch(MotionSet)   // PATTERNPOINTSHIFT  0x6B -- confirm and run the editing pattern points: 1- follow all the points; 2- return back and follow all the points
		{
			case 0:
				pat_point = (PATTERN_DATA *)(pat_buf);
				pat_point = pat_point + (DestinationPointShiftPosition%TOTAL_STITCH_COUNTER);
				pat_buff_total_counter = DestinationPointShiftPosition;
				go_commandpoint(comx_step,comy_step);
				predit_shift = 0;
				break;
			case 1:
				temp16 = DestinationPointShiftPosition - CurrentPointShiftPosition;
				if(temp16 >= 0)
				{
					pat_point = (PATTERN_DATA *)(pat_buf);
					pat_point = pat_point + (CurrentPointShiftPosition%TOTAL_STITCH_COUNTER);
					pat_buff_total_counter = CurrentPointShiftPosition;
				
					for(i=0;i<temp16;i++)
					{
						course_edit_next();
						if(OutOfRange_flag == 1)
						{
							break;
						}
					}
					if(OutOfRange_flag == 1)
					{
						predit_shift = 1;
					}
					else
					{
						predit_shift = 0;
					}
				}
				else
				{
					pat_point = (PATTERN_DATA *)(pat_buf);
					pat_point = pat_point + (DestinationPointShiftPosition%TOTAL_STITCH_COUNTER);
					pat_buff_total_counter = DestinationPointShiftPosition;
				
					for(i=0;i<-temp16;i++)
					{
						course_edit_back();
					}
					predit_shift = 0;
				}
				MotionSet = 0;
				break;
			case 2:
				
				if( check_sewing_range())
				{
					TempStart_pointTemp = pat_point;
					pat_point = (PATTERN_DATA *)(pat_buf);
					allx_step_temp = 0;
					ally_step_temp = 0;
					
					//
					for(pat_point = (PATTERN_DATA *)(pat_buf);pat_point <= TempStart_pointTemp;pat_point++)
					{
						switch(pat_point->func & 0xF0)
						{
							case 0x00:
							case 0x10:
							case 0x50:
							case 0x20:
							case 0x30:
							case 0x60:
							case 0x70:
								allx_step_temp = allx_step_temp + ((INT16)pat_point->xstep)*2;
								ally_step_temp = ally_step_temp + ((INT16)pat_point->ystep)*2;
      	         	  			switch(pat_point->func & 0x0F)
                 	  			{	
    	         	  			    case 0x00: break;
    	         	  			    case 0x08: allx_step_temp = allx_step_temp+1; break;
    	         	  			    case 0x02: ally_step_temp = ally_step_temp+1; break;
    	         	  			    case 0x0A: allx_step_temp = allx_step_temp+1; ally_step_temp = ally_step_temp+1; break;
    	         	  			    default:   break;
                 	  			} 
								break;
							default:
								break;
						}
					}
					pat_point = TempStart_pointTemp;
					go_commandpoint(allx_step_temp,ally_step_temp);
					
					sys.error = ERROR_15;
					StatusChangeLatch = ERROR;
				}
				else
				{
					pat_point = (PATTERN_DATA *)(pat_buf);
					pat_point = pat_point + (DestinationPointShiftPosition%TOTAL_STITCH_COUNTER);
					pat_buff_total_counter = DestinationPointShiftPosition;
			     	go_commandpoint(comx_step,comy_step);
				}
				predit_shift = 0;
				MotionSet = 0;
				break;
		}
		monitor_allx_step = allx_step;
		monitor_ally_step = ally_step;
		monitor_pat_point = pat_point -1 ;
		monitor_refresh_flag = 1;
		MotiongSetFlag = 0;
	}
	
	//------------------------------------------------------------------------------------
	//stitch up command
	//------------------------------------------------------------------------------------	
 	if(StitchUpFlag == 1)
	{
		temp8 = detect_position();	
		if(temp8 == OUT)    									// out of range 
		{
		   find_dead_center();
		}
		StitchUpFlag = 0;
	 }
	//-------------------------------------------------------------------------------------
	// inpress up or down command
	//-------------------------------------------------------------------------------------
	if( inpress_act_flag == 1)
	{
		if( inpress_type == AIR_INPRESS)
		{
			if(inpress_flag == 1)
				inpress_down(0);
			else
			    inpress_up();
		}
		else
		{
			if(inpress_high == 0)
			{
				if( inpress_flag == 1)
				    inpress_down(inpress_high_hole);
		    }
			else if(inpress_high >= 71)//==
			{
				if(inpress_flag == 0)
				{
					temp8 = detect_position();	
					if(temp8 == OUT) 
			  		{
						find_dead_center();
			  		}
					inpress_up();
				} 
		   }
			else                          
			{	
				inpress_to(inpress_high);
			}
		}
		inpress_act_flag = 0;       
		predit_shift = 0;    
	}
	//-----------------------------------------------------------------------------------------
	// point shift command  //  POINTSHIFT	0x64 -- stitch check and move 
	//--------------------------------------------------------------------------------------------
	if(super_pattern_flag == 0)
	{
		if(PointShiftFlag == 1)
		{
			temp8 = detect_position();	
			if(temp8 == OUT)    
	  		{
				find_dead_center();
	  		} 
		    if(inpress_flag == 0)
			{
				inpress_up();
				delay_ms(20);
			}             
			if(PointShiftDirection == 1)
			{
				for(i=0;i<PointShiftNumber;i++)
				{
					course_edit_back();
				}
			}
			else if(PointShiftDirection == 0)
			{
				for(i=0;i<PointShiftNumber;i++)
				{
					course_edit_next();
				}
			}
			predit_shift = 0;
			PointShiftFlag = 0;
		}
	}
	else
	{
	   if(coor_com  == 1)//0x77
  	   {
			temp8 = detect_position();	
			if(temp8 == OUT)   
	  		{
				find_dead_center();
	  		}
			if(inpress_flag == 0)
			{
				inpress_up();
				delay_ms(20);
			}
			delay_ms(50);
			if(AUTO_SEARCH_START_POINT == 1)	//û�ҹ�ԭ�㣬��ִ���������ԭ�㴦��
			{
				if (already_auto_find_start_point == 0)
				{
					find_start_point();
					already_auto_find_start_point = 1;
				}
			}
			go_commandpoint(comx_step,comy_step);
			
			predit_shift = 0;
			coor_com = 0;
  	   }
	   if( PointShiftFlag == 1)//0x64
	   {	
			temp8 = detect_position();	
			if(temp8 == OUT)    
	  		{
				find_dead_center();
	  		} 
		    if(inpress_flag == 0)
			{
				inpress_up();
				delay_ms(20);
			}
		    pat_point = (PATTERN_DATA *)(pat_buf);
		    pat_point = pat_point + (PointShiftNumber%TOTAL_STITCH_COUNTER);
		    pat_buff_total_counter = PointShiftNumber;
			delay_ms(120);	    
			predit_shift = 0;
			PointShiftFlag = 0;
	    }
	}
	// pedal command
	if(pedal_style == single_pedal)
	{
	  	//--------------------------------------------------------------------------------------
	  	//  foot sensor
	  	//-------------------------------------------------------------------------------------- 
	  	if(pedal_state == 1)           					// foot sensor is pushed,
		{
			delay_ms(10);
			if(pedal_state == 1)
			{				
				if(foot_flag == 1 && pedal_last_state == 0)
				{		 
		      		foot_down();	
		    	}	    	    
		  	}
	  	} 
	 	//--------------------------------------------------------------------------------------
	  	//  foot sensor
	  	//-------------------------------------------------------------------------------------- 
		if(pedal_state == 0)           				// foot sensor is pushed,
		{
			delay_ms(10);
			if(pedal_state == 0)
			{				
		    	if(foot_flag == 0)
		    	{	
				   footer_both_up();  //	foot_up();	  //2012-9-4 modify
		   	 	}	    	    
		  	}
		 	pedal_last_state = 0;
	  	}
	}
	else if(pedal_style == double_pedal)
	{
		//--------------------------------------------------------------------------------------
	  	//  foot sensor
	  	//-------------------------------------------------------------------------------------- 
	  	if(DVB == 0)           					// foot sensor is pushed,
		{
			delay_ms(10);
			if(DVB == 0 && DVBLastState == 1)
			{	
		
		       if(k60 == 1)     
				 {
					   if(foot_flag == 0)
					    {
							 if(foot_half_flag == 1)
							   foot_up();
					    }
					 else
					   {
							  foot_down();
					   }    	    
		  	     }
			   else
			    {
					 footer_procedure();
			    }
	   	       DVBLastState = DVB;	
	  	    }
	 	   else
		    {
		 	    DVBLastState = DVB;
		    } 
		}   
		if(k60 == 1)
		{
			 if(DVSM == 0)
			  {
				delay_ms(10);
				if(DVSM == 0 && DVSMLastState == 1)
				 {				
					if(foot_half_flag == 1)
					{	
						if(foot_flag==0) //2012-9-4 add: after footer already down	 
			      		 foot_half_down();	
			    	}
			  		else if(foot_half_flag == 0)
					{
		    			foot_half_up();	
		    		}
		   	    	DVSMLastState = DVSM;		    	    
			  	 }
		  	 }
		 	 else
			 {
				 DVSMLastState = DVSM;
			 }
		 }	
	}
	// step forward or backward command
	switch(single_flag)
  	{
  		case 0:	                                     
			break;
  		case 1:	 
  			
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			move_next();
			predit_shift = 0;                        
			break;
  		case 2:	
  			
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			move_back(); 
			predit_shift = 0;              
			break;
  		case 3:	 
  			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			//move_edit_startpoint();  
			go_origin_allmotor();
			if(AUTO_SEARCH_START_POINT == 1)//#if AUTO_SEARCH_START_POINT  
			{
				//if( already_auto_find_start_point == 0 )
				{
					delay_ms(100);
					find_start_point();	
					already_auto_find_start_point = 1;
				}
			}
			//#endif
			pat_point = (PATTERN_DATA *)(pat_buf);
			if(k110 == 1)
			{
				R_AIR = 0;
				delay_ms(80);  
			}
		
			predit_shift = 0;     
			break;
		case 4:	 
			

			SewingTestEndFlag = 1;  

			if(nop_move_pause_flag ==1)
		    	process_nop_move_pause(1);
			else
				go_sewingtest_beginpoint();  
			
			SewingTestEndFlag = 0;   
			predit_shift = 0;
			single_flag = 0;
			break;			
  		case 5:	 
			SewingTestEndFlag = 1;  
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			inpress_up(); 
			if(nop_move_pause_flag ==1)
		    	process_nop_move_pause(2);
			else
			{
			 	back_endpoint();
			}
			if( (FootRotateFlag == 1) &&(marking_flag == 1) )
			{
				process_marking_pen(1);
				marking_flag = 0;
				FootRotateFlag = 0;
				delay_ms(50);
			}
			SewingTestEndFlag = 0;  
			predit_shift = 0;
			single_flag = 0;  
			break;
		case 6:
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			//course_edit_continue_next();  
			course_next();
			delay_ms(1);      

			break;
		case 7:
		    temp8 = detect_position();	
    		if(temp8 == OUT)  
      		{
				find_dead_center();
      		}
			//course_edit_continue_back();  
			course_back();
			delay_ms(1);    
			break;
		case 8:
			course_stop(); 
			break;	
					
  		default:                                     
			break;	
  	}
	// status change command		
	if(StatusChangeLatch != PREEDIT)
	{
		predit_shift = 0;
		sys.status = StatusChangeLatch;
		if( StatusChangeLatch == FREE)
		{
		  temp8 = detect_position();	
		  if(temp8 == OUT)    
		 	   find_dead_center(); 
		}	   
	}
	if( (origin_com == 0)&&(ready_go_setout_com==0) &&(MotiongSetFlag==0)&&(PointShiftFlag==0)&&(coor_com==0)&&(single_flag==0) )
	   predit_shift = 0;
}
//--------------------------------------------------------------------------------------
//  Name:		edit_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of edit stauts
//--------------------------------------------------------------------------------------
void edit_status(void)
{	
	UINT16 i;
	UINT8 temp8;
	//--------------------------------------------------------------------------------------
  	//  manual shift step  
  	//--------------------------------------------------------------------------------------
  	switch(shift_flag)
  	{
  		case 0x0C:  shift_12();      break;  
    	case 0x01:  shift_01();      break;    		            							
    	case 0x03:  shift_03();      break;  
    	case 0x04:  shift_04();      break;
    	case 0x06:  shift_06();      break;    		            							
    	case 0x07:  shift_07();      break;
    	case 0x09:  shift_09();      break;
    	case 0x0A:  shift_10();      break;    		            									  		            								
    	case 0xEC:  remove_12();     break;
    	case 0xE1:  remove_01();     break;    		            							
    	case 0xE3:  remove_03();     break;  
    	case 0xE4:  remove_04();     break;
    	case 0xE6:  remove_06();     break;    		            							
    	case 0xE7:  remove_07();     break;
    	case 0xE9:  remove_09();     break;
    	case 0xEA:  remove_10();     break; 	
    	case 0x88:  remove_stop();   break; 		
    	case 0x5C:  remove_12();   delay_process();  break;
    	case 0x51:  remove_01();   delay_process();  break;    		            							
    	case 0x53:  remove_03();   delay_process();  break;  
    	case 0x54:  remove_04();   delay_process();  break;
    	case 0x56:  remove_06();   delay_process();  break;    		            							
    	case 0x57:  remove_07();   delay_process();  break;
    	case 0x59:  remove_09();   delay_process();  break;
    	case 0x5A:  remove_10();   delay_process();  break;    		
    	default:   				                                 break;	
  	}
	//----------------------------------------------------------------------------------------------
  	//  coordinate command  // 0x77-- cancel pattern editing points or return to edit starting point 
  	//----------------------------------------------------------------------------------------------
  	if(coor_com  == 1)
  	{
		temp8 = detect_position();	
		if(temp8 == OUT)   
  		{
			find_dead_center();
  		}
		if(inpress_flag == 0)
		{
			inpress_up();
			delay_ms(20);
		}
		delay_ms(50);
		/*
		if(k169 == MACHINE_MID)
		{
			go_commandpoint(allx_step,comy_step);
			delay_ms(100);
			go_commandpoint(comx_step,ally_step);		
		}
		else
		{
		*/
     	go_commandpoint(comx_step,comy_step);
		//}
		
		predit_shift = 0;
		coor_com = 0;
  	}
	if ( ready_go_setout_com == 1)
	{
		ready_go_setout_com = 0;
	}
	//-------------------------------------------------------------------------------------
	// move to current coordinator command
	//--------------------------------------------------------------------------------------
	/*
	if(MoveToCurrentCoorFlag == 1)
	{
		temp8 = detect_position();	
		if(temp8 == OUT)  
  		{
			find_dead_center();
  		}
		if(inpress_flag == 0)
		{
			inpress_up();
			delay_ms(20);
		}
		go_commandpoint(CurrentXCoor,CurrentYCoor);
		MoveToCurrentCoorFlag = 0;
	}
	*/
	//------------------------------------------------------------------------------------
	//stitch up command
	//------------------------------------------------------------------------------------	
 	if(StitchUpFlag == 1)
	{
		temp8 = detect_position();	
		if(temp8 == OUT)    									
		{
		   find_dead_center();
		}
		StitchUpFlag = 0;
	}
	//-------------------------------------------------------------------------------------
	// inpress up or down command
	//---------------------------------------------------------------------------------------
	if( inpress_act_flag == 1)
	{
		if( inpress_type == AIR_INPRESS)
		{
			if(inpress_flag == 1)
				inpress_down(0);
			else
			    inpress_up();
		}
		else
		{	
			if(inpress_high == 0)
			{
				if(inpress_flag == 1)
				{
					inpress_down(inpress_high_hole);
				}
			}
			else if(inpress_high == 71)
			{
				if(inpress_flag == 0)
				{
					temp8 = detect_position();	
					if(temp8 == OUT) 
			  		{
						find_dead_center();
			  		}
					inpress_up();
				}
			}
			else                          
			{
				inpress_to(inpress_high);
			}
		}
		predit_shift = 0;
		inpress_act_flag = 0;  
	}
	//--------------------------------------------------------------------------------------
  	//  single step move 
  	//--------------------------------------------------------------------------------------
  	switch(single_flag)
  	{
  		case 0:	                                     
			break;
  		case 1:	 
  			;
			temp8 = detect_position();	
    		if(temp8 == OUT)   
      		{
				find_dead_center();
      		}
		
			move_next();

			predit_shift = 0;                
			break;
  		case 2:	 
  			 
			temp8 = detect_position();	
    		if(temp8 == OUT)   
      		{
				find_dead_center();
      		}
			move_back(); 

			predit_shift = 0;            
			break;				
  		default:                                     
			break;	
  	}
  	
	//-----------------------------------------------------------------------------------------
	// point shift command
	//--------------------------------------------------------------------------------------------
	if(super_pattern_flag == 0)
	{
	   if(PointShiftFlag == 1)
	   {
			temp8 = detect_position();	
			if(temp8 == OUT)    
	  		{
				find_dead_center();
	  		} 
		if(inpress_flag == 0)
		{
			inpress_up();
			delay_ms(20);
		}   
		if(PointShiftDirection == 1)
		{
			for(i=0;i<PointShiftNumber;i++)
			{
				course_edit_back();
			}
		}
		else if(PointShiftDirection == 0)
		{
			for(i=0;i<PointShiftNumber;i++)
			{
				course_edit_next();
			}
		}
		predit_shift = 0;
		PointShiftFlag = 0;
	    }
	}
	else
	{
	   if( PointShiftFlag == 1)//0x64
	   {	
			temp8 = detect_position();	
			if(temp8 == OUT)    
	  		{
				find_dead_center();
	  		} 
		    if(inpress_flag == 0)
			{
				inpress_up();
				delay_ms(20);
			}
		    pat_point = (PATTERN_DATA *)(pat_buf);
		    pat_point = pat_point + (PointShiftNumber%TOTAL_STITCH_COUNTER);
		    pat_buff_total_counter = PointShiftNumber;	    
			predit_shift = 0;
			PointShiftFlag = 0;
		
	    }
	}
	predit_shift = 0;                             
	if(StatusChangeLatch != EDIT) 
	{
 		if(StatusChangeLatch == PREEDIT)
		{
			sys.status = StatusChangeLatch;
			predit_shift = 1;
		}
		else
		{
			predit_shift = 0;
			sys.status = StatusChangeLatch;
		}
		
	}
}
//--------------------------------------------------------------------------------------
//  Name:		noedit_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of noedit stauts
//--------------------------------------------------------------------------------------
void noedit_status(void)
{	
	if(foot_flag == 1)
	{
		 footer_both_down();      
	}
	
	if(StatusChangeLatch != NOEDIT) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		finish_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of finish stauts
//--------------------------------------------------------------------------------------
void finish_status(void)
{	
	#if CHANGE_DOUBLE_FRAMEWORK 
	if( coor_com  == 1)
  	    coor_com = 0;
	if( PointShiftFlag == 1)
	    PointShiftFlag = 0;
	predit_shift = 0;
	#else
	#if SEND_SERVO_PARA_ONLINE
	SUM = 0;
	#endif
	if(super_pattern_flag == 1)	
	{
		if(coor_com ==1)
		{
		#if AUTO_CHANGE_PATTERN_FUNCTION ==0
			if( (u37 == 1)&&(u38 == 0) )
			{
		        footer_both_up(); 
			}
		#endif
			if(u39 == 1)											
		  	{
				go_origin_xy();				
			}
			if(finish_nopmove_pause_flag ==0)
			{ 
				if( u39 !=1)
				{
					if(inpress_flag == 0)
					{
						inpress_up();
						delay_ms(20);
					}
		     		go_commandpoint(comx_step,comy_step);
				}
				#if AUTO_CHANGE_PATTERN_FUNCTION ==0
				if( (u37 == 0)&&(u38 == 0) )
				{
		            footer_both_up(); 
				}
				else if (u37 == 2)
				   FootUpCom = 1;
				#endif  
				if(u39 == 1)											
		  	    {
				   go_origin_zx();			
			    }
				 
				pat_buff_total_counter = 0;
			}
			predit_shift = 0;
			coor_com = 0;
		}
		if( PointShiftFlag == 1)//0x64
	    {	
		    if(finish_nopmove_pause_flag ==0)
			{
				pat_point = (PATTERN_DATA *)(pat_buf);
			    pat_point = pat_point + (PointShiftNumber%TOTAL_STITCH_COUNTER);
			    pat_buff_total_counter = PointShiftNumber;
			}
		    PointShiftFlag = 0;
	    }
	}
	#endif
	
	if(StatusChangeLatch != FINISH) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
	if ( ready_go_setout_com == 1)
	{
		ready_go_setout_com = 0;
		predit_shift = 0;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		needle_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of needle stauts
//--------------------------------------------------------------------------------------
void needle_status(void)
{	
	//--------------------------------------------------------------------------------------
  	// presser down and inpresser to lowest point
  	//-------------------------------------------------------------------------------------- 	
	if(foot_flag == 1)
	{		 
	    foot_down();         	
		delay_ms(50);
    	
	}
	
	if(inpress_flag == 1)
	{
		inpress_down(inpress_high_hole);
	}
	
	DVALastState = DVA;
	
	if(StatusChangeLatch != NEEDLE) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		waitoff_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of waitoff stauts
//--------------------------------------------------------------------------------------
void waitoff_status(void)
{	
	//--------------------------------------------------------------------------------------
  // foot down or up
  //-------------------------------------------------------------------------------------- 
  switch(foot_com)
  {
  	case 0:                          
  		     if(foot_flag == 1)     
  	         {
  	         	 footer_both_down();     
  	         }
  	         break;
  	         
    case 1:                        
    	     if(foot_flag == 0)     
  	         {
  	         	 footer_both_up();   
  	         }
  	         break;		
  	
  	case 2:  break;                 
  		       
  	default: break;                   	         	
  }
}
//--------------------------------------------------------------------------------------
//  Name:		trim_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of trim stauts
//--------------------------------------------------------------------------------------
void trim_status(void)
{	
	UINT8 temp8;
	INT16 temp16;
	//--------------------------------------------------------------------------------------
  	//  inpresser down
  	//--------------------------------------------------------------------------------------
	inpress_down(inpress_high);                                
	 trim_action();      
	 inpress_up();	
     emermove_high = IN_ORIGIN;
 	 delay_ms(250);
  	//--------------------------------------------------------------------------------------
  	//  switch system status 
  	//--------------------------------------------------------------------------------------  	   
			        
  	sys.status = EMERMOVE;   
	StatusChangeLatch = EMERMOVE; 	
}
//--------------------------------------------------------------------------------------
//  Name:		slack_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of slack stauts
//--------------------------------------------------------------------------------------	
void slack_status(void)
{	

	if(StatusChangeLatch != SLACK) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		checki03_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of checki03 stauts---check input signal
//--------------------------------------------------------------------------------------	
void checki03_status(void)
{	
	//--------------------------------------------------------------------------------------
  	// config main motor driver
  	//-------------------------------------------------------------------------------------- 
  	initial_mainmotor();
	if(inpress_flag == 1)
	{
	   inpress_down(inpress_high_hole);
	}
	if ( SUPPORT_CS3_FUN == 1)
	{
		dsp3_input_value = check_DSP3_input();

		if( dsp3_input_value &0x01)
			output_cs3(1,1);
		else	
			output_cs3(1,0);
		
		if( dsp3_input_value &0x02)
			output_cs3(2,2);
		else	
			output_cs3(2,0);
		
		if( dsp3_input_value &0x04)
			output_cs3(4,4);
		else	
			output_cs3(4,0);
		
		if( dsp3_input_value &0x08)
			output_cs3(8,8);
		else	
			output_cs3(8,0);
		
		if( dsp3_input_value &0x10)
			output_cs3(0x10,0x10);
		else	
			output_cs3(0x10,0);
		
		if( dsp3_input_value &0x20)
			output_cs3(0x20,0x20);
		else	
			output_cs3(0x20,0);
		
	}
  	if(sys.status == ERROR)
  	{
  	  return;
  	}
	if(StatusChangeLatch != CHECKI03) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		checki04_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of checki04 stauts---check servo motor
//--------------------------------------------------------------------------------------	
void checki04_status(void)
{	
	UINT8 temp8;
	UINT16 spd_tmp,i;
  	if(sys.status == ERROR)
  	{
    	return;
  	}
	motor.dir = 0; 
  	//--------------------------------------------------------------------------------------
  	// config main motor driver add for 210E
  	//-------------------------------------------------------------------------------------- 
  	initial_mainmotor();
  	if(sys.status == ERROR)
  	{
  	  return;
  	}
	//--------------------------------------------------------------------------------------
  	//  find origin
  	//-------------------------------------------------------------------------------------- 
	if(origin_com == 1)
	{		
		/*	
		temp8 = detect_position();	
		if(temp8 == OUT)   
  		{
			find_dead_center();
  		}	
	  	go_origin_allmotor();	   // all step motor go origin    
	  	if(sys.status == ERROR)
    	{
      		return;
    	}	  
		*/
    	origin_com = 0;        		
  	}
	//--------------------------------------------------------------------------------------
  	// servo motor run and stop
  	//-------------------------------------------------------------------------------------- 	
	if(foot_flag == 1)
	{		 
	  	 footer_both_down();      	
	}
	else
	{
		if(smotor_speed != 0)
		{
			if(motor.stop_flag == 1)
			{
				inpress_down(inpress_high_hole);
				if(k02 ==1)
				{
					if(smotor_speed > MAXSPEED1 || smotor_speed < 2)
					{
						smotor_speed = 2; 
					}			
				}	
				else if(k02 == 0)
				{
					if(smotor_speed > MAXSPEED0 || smotor_speed < 2)
					{
						smotor_speed = 2; 
					}
				}
				for(i=0;i<5;i++)
		    	{	  	  	  		     
					if(i==0)
					  spd_tmp = u10;	
					else if(i==1)  
					  spd_tmp = u11;	
					else if(i==2)
					  spd_tmp = u12 ;	
					else if(i==3)
					  spd_tmp = u13;	
					else if(i==4)
					  spd_tmp = u14;
					if(spd_tmp >= smotor_speed)
					   motor.spd_obj = smotor_speed*100;
					else
					   motor.spd_obj = spd_tmp*100;
					delay_ms(10);  
					while(motor.angle_adjusted >= 16)
			    		{
			    			rec_com();    						                
			    		}  	 
					while(motor.angle_adjusted < 16)
			    		{
			    			rec_com();    						                
			    		} 
				}
				motor.spd_obj = smotor_speed * 100;				
			}	
		}	
		else
		{
			if(motor.stop_flag == 0) 
			{
				sewing_stop();		
				while(motor.stop_flag == 0)
				{
					rec_com();
				}
				delay_ms(80);
			  if(k03 == 0)   
				{   
				    da0 = 0;	           
				    SNT_H = 0;             
				 }
				inpress_up();
			}	
		}	
	}	
	
	if(StatusChangeLatch != CHECKI04) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		checki05_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of checki05 stauts---check output signal
//--------------------------------------------------------------------------------------	
void checki05_status(void)
{	
  	UINT8 fir_trim_flag = 0,temp8; 
	UINT16 temp16;
	initial_mainmotor();
	if(sys.status == ERROR)
  	{
    	return;
  	}
	#if SUPPORT_UNIFY_DRIVER

	if( write_stepmotor_curve_flag != 0)
	{
	    if( write_stepmotor_curve(write_stepmotor_curve_flag, pat_buf) )
		{
			SUM = 1;
			delay_ms(100);
			SUM = 0;
			write_stepmotor_curve_flag = 0;
		}	
	}

	#endif
  	//--------------------------------------------------------------------------------------
  	// output move
  	//--------------------------------------------------------------------------------------
	switch(output_com)
	{
		//--------------------------------------------------------------------------------------      
    	//  no move
    	//--------------------------------------------------------------------------------------	
		case 0: break;	
		//--------------------------------------------------------------------------------------      
    	//  wiper move
    	//--------------------------------------------------------------------------------------
    	case 1: 								//����
			if( powr_on_zx_flag == 0)
			{
				go_origin_zx();
				powr_on_zx_flag = 1;
			}
			if(inpress_flag == 1)  
    	    {
    	    	inpress_down(inpress_high_hole);
				delay_ms(50);
    	   	}
			SNT_H = 1;        	
			#if SECOND_GENERATION_PLATFORM || CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
			FK_OFF = 1;
			delay_ms(150);
			FK_OFF = 0;
			#else
		    FW = 1;
			da1 = 200;
	        delay_ms(150);
		    FW = 0;
			da1 = 0;
			#endif
		    SNT_H = 0;        	
	        delay_ms(200);
	        output_com = 0;
	        break;
    	//--------------------------------------------------------------------------------------      
    	//  trimmer move
    	//--------------------------------------------------------------------------------------    	
    	case 2: 					//����
		   	SNT_H = 1;      
			#if SECOND_GENERATION_PLATFORM 
			if( cutter_output_test == 0)
			{
				FL = 1;
			    FL_pwm_counter = 0;
			    FL_pwm_period = 400;
			    FL_pwm_action_flag = 1;
				cutter_output_test = 1;
			}
			else
			{
				FL_pwm_action_flag = 0;
				FL = 0;
				cutter_output_test = 0;
			}
			#elif  CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
			FA = 1;
			delay_ms(200);
			FA = 0;
			#else 	
			if( cutter_output_test == 0)
			{
				L_AIR = 1;
				cutter_output_test = 1;
			}
			else
			{
	        	L_AIR = 0;
				cutter_output_test = 0;
			}
			#endif
		    SNT_H = 0;        	// 33V to 24V
	        output_com = 0;	
			
			#if DOUBLE_X_60MOTOR
		 	if(u46 == 0)
			{
				#if SPECIAL_MOTOR_CUTTER 
				go_origin_ct();
				delay_ms(50);
				movestep_ct(39,4);
				delay_ms(50);
				movestep_ct(-29,4);
				delay_ms(50);
				#endif
			}
			#endif 
			
	        break;
	    
	  	//--------------------------------------------------------------------------------------      
    	//  foot move
    	//--------------------------------------------------------------------------------------
    	case 3: 								//��ѹ��
			if(foot_flag == 1) 
    	    {
    	    	 foot_down();  
				 #if SECOND_GENERATION_PLATFORM || CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800 || COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER33 || COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER35 || COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER38
				 #else
				 FL = 1; 
				 #endif
				 //movestep_yj(-100,60);
				 delay_ms(80);
    	   	}
    	    else                    
    	    {
    	    	 foot_up();
				 #if SECOND_GENERATION_PLATFORM || CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800 || COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER33 ||COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER35|| COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER38
				 #else
				 FL = 0; 
				 #endif
				 //movestep_yj(100,60);
				 delay_ms(80);
    	    }
			//movestep_cs3(0x2000,-1,1);   //��ʱ��
		    delay_ms(20);
	        output_com = 0;
	        break;
	  	//--------------------------------------------------------------------------------------      
    	//  infoot move
    	//--------------------------------------------------------------------------------------
   	 	case 4: 								//��ѹ��
			if(inpress_flag == 1) 
    	    {
    	    	inpress_down(0);
				#if SECOND_GENERATION_PLATFORM || CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
				L_AIR = 1;
				#endif
    	    }
    	    else                  
    	    {
    	    	inpress_up();
				#if SECOND_GENERATION_PLATFORM || CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
				L_AIR = 0;
				#endif

    	    }    	      
		    delay_ms(200);
	        output_com = 0;
	        break;
		case 5:	
			da0 = 255;
            delay_ms(50);
			da0 = 0;
	        SNT_H = 0;        // 33V to 24V
	        delay_ms(200);
	        output_com = 0;
	        break;
		case 6:
		   if( (k110 == 1)||(auto_function_flag == 1) )
		    {
			   R_AIR =1;
               delay_ms(500);
			   R_AIR = 0;
	           delay_ms(200);
			}
			else if(k110 == 2)
			{
				stretch_foot_out();
			    delay_ms(500);
				stretch_foot_in();
			 }
			output_com = 0;
			
	        break;
	  	//--------------------------------------------------------------------------------------   
		case 7:                     //fuzhuqifa 1
			if( check_valve1_flag == 0)
			{
		     	EXTEND = 1;
			 	delay_ms(200);
			 	check_valve1_flag = 1;
				#if DEBUG_CS3_INPUT
				 	output_cs3(1,1);
		        #endif
			}
			else
			{ 
			 	EXTEND = 0;
			 	check_valve1_flag = 0;
			    #if DEBUG_CS3_INPUT
				 	output_cs3(1,0);
		        #endif
			}
			 delay_ms(200);
			 output_com = 0;
		
		break;   
		case 8:                     //fuzhuqifa 2
			if( check_valve2_flag == 0)
			{
		     	FR_ON = 1;
				check_valve2_flag = 1;
				#if DEBUG_CS3_INPUT
			 		output_cs3(2,2);
	    		#endif	
			}
			else
			{
			    FR_ON = 0;
			    check_valve2_flag = 0;
				#if DEBUG_CS3_INPUT
			 		output_cs3(2,0);
	    		#endif
			}
			 delay_ms(200);
			 output_com = 0;
		break;
		case 9:						//fuzhuqifa 3
			if( check_valve3_flag == 0)
			{
		     	FK_OFF = 1;
				check_valve3_flag = 1;
				#if DEBUG_CS3_INPUT
			 		output_cs3(4,4);
	    		#endif
				#if TEST_MAIN_MOTOR
				temp16 = k61;
				k61 = (k61+512)%1024;
				find_dead_center();
				k61 = temp16;	
				#endif
			}
			else
			{
			 	FK_OFF = 0;
				check_valve3_flag = 0;
				#if DEBUG_CS3_INPUT
			 		output_cs3(4,0);
	    		#endif
				#if TEST_MAIN_MOTOR
				temp8 = detect_position();	
	    		if(temp8 == OUT)    // out of range 
	      		{
					find_dead_center();
	      		}
				#endif
			}
			 delay_ms(200);
			 output_com = 0;
		
		break;
		case 10:                     //fuzhuqifa 4
			if( check_valve4_flag == 0)
			{
		       T_HALF = 1;
			   delay_ms(200);
			   check_valve4_flag = 1;
			   #if DEBUG_CS3_INPUT
				   output_cs3(8,8);
			   #endif			   
			}
			else
			{
			 	T_HALF = 0;
				delay_ms(200);
				check_valve4_flag = 0;
				#if DEBUG_CS3_INPUT
					output_cs3(8,0);
			    #endif
			}			  
			output_com = 0;
		    #if MACHINE_900_LASER_CUTTER2
			if( laser_power_on_enable == 1)
				LASER_POWER_PIN = 1;
			delay_ms(500);
			LASER_POWER_PIN = 0;
			#endif
		break;
		case 11:				    //fuzhuqifa 5
			if( check_valve5_flag == 0)
			{
				#if AUTO_CHANGE_PATTERN_FUNCTION
				T_DIR_2 =1;
				#else
		     	T_DIR = 1;
				#endif
			 	delay_ms(200);
				check_valve5_flag = 1;
				#if DEBUG_CS3_INPUT
					output_cs3(0x10,0x10);
			    #endif
			}
			else
			{
				#if AUTO_CHANGE_PATTERN_FUNCTION
				T_DIR_2 = 0;
				#else
		     	T_DIR = 0;
				#endif
				 delay_ms(200);
				 check_valve5_flag = 0;
				 #if DEBUG_CS3_INPUT
					 output_cs3(0x10,0);
			     #endif
			}
			 output_com = 0;
		
		break;
		case 12:					//fuzhuqifa 6
			 if( check_valve6_flag == 0)
			 {
		     	  T_CLK = 1;			 	  
				  check_valve6_flag = 1;
				  #if DEBUG_CS3_INPUT
					  output_cs3(0x20,0x20);
			      #endif
			 }
			 else
			 { 
			 	  T_CLK = 0;
				  check_valve6_flag = 0;
			 	  #if DEBUG_CS3_INPUT
					  output_cs3(0x20,0x0);
			      #endif
			 }
			 FA = 1;
			 delay_ms(200);
			 FA = 0;
			 output_com = 0;
			 #if  MOVING_DEBUG
				 test_stepper_moving();
				 delay_ms(500);
				 go_origin_allmotor();
			 #endif
		break;
    	//  no move
    	//--------------------------------------------------------------------------------------
	  	default: 
		output_com = 0;
		break;          
	}	
	
	if(StatusChangeLatch != CHECKI05) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		checki06_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of checki06 stauts---check X and Y sensor
//--------------------------------------------------------------------------------------	
void checki06_status(void)
{	
	UINT8 temp8;
	INT32 backup;
  	//--------------------------------------------------------------------------------------
  	// config main motor driver add for 210E
  	//-------------------------------------------------------------------------------------- 
  	initial_mainmotor();
  	if(sys.status == ERROR)
  	{
  	  return;
  	}
	//--------------------------------------------------------------------------------------
  	//  find origin
  	//-------------------------------------------------------------------------------------- 
	if(origin_com == 1) //check i04
	{		
		temp8 = detect_position();	
		if(temp8 == OUT)    
  		{
			find_dead_center();
  		}	
	  	 if(sys.status == ERROR)
    	  {      	
			return;
    	  }
    	 footer_both_down();      
		
		if(k60 == 1)
		{
			foot_half_down();
		}
			  
    	origin_com = 0;        		
  	}
 	if(pedal_style == single_pedal)
	{ 
		//--------------------------------------------------------------------------------------
	  	//  start sensor
	  	//-------------------------------------------------------------------------------------- 
		if(pedal_state == 0 && pedal_last_state != 0)
		{
			delay_ms(10);
			if(pedal_state == 0)
			{
				pedal_last_state = 0;
			}
		}
	  	if(pedal_state == 2)           						// start sensor is pushed
		{
			delay_ms(10);
			if(pedal_state == 2 && pedal_last_state == 0)
			{				
				pedal_last_state = 2;
				go_origin_xy();    							//x and y go origin
		      	rec_com();       						      	    
		  	}
	  	}
	}
	else if(pedal_style == double_pedal)
	{
		//--------------------------------------------------------------------------------------
	  	//  start sensor
	  	//--------------------------------------------------------------------------------------  
	  	if(DVA == 0)           								// start sensor is pushed
		{
			delay_ms(10);
			if(DVA == 0)
			{				
				go_origin_xy();    							//x and y go origin
				backup = x_origin_offset;
				x_origin_offset = 0;
				find_start_point1();
				x_origin_offset = backup;
				while(DVA == 0)
		    	{
		      		rec_com();       						// communication with panel	
		    	}	 	      	    
		  	}
	  	}
		
		if(DVB == 0)           								// start sensor is pushed
		{
			delay_ms(10);
			if(DVB == 0)
			{				
				if(foot_flag == 1) 
	    	    {
	    	    	 foot_down();   
	    	   	}
	    	    else                    
	    	    {
	    	    	 foot_up();
	    	    }	 
				while(DVB == 0)
		    	{
		      		rec_com();       						// communication with panel	
		    	}
   	    
		  	}
	  	}
	 		
	 #if AUTO_DEBUG_MODE
	 	if (PAUSE == pause_active_level)
		{
			delay_ms(10);                           
			if(PAUSE == pause_active_level)
			{				
				if(DVA == 0)           							
				{
					delay_ms(10);
					if(DVA == 0)
					{				
						//auto_debug_program();  
						test_quickmove_program();  						
						while( (DVA == 0)||(PAUSE == pause_active_level) )
				    	{
				      		rec_com();       					
				    	}	 	      	    
				  	}
			  	}	  			  	
		  	}
	  	}
	  #endif 
	}
  	//--------------------------------------------------------------------------------------
  	//  manual shift step  
  	//--------------------------------------------------------------------------------------
  	switch(shift_flag)
  	{		
	  	case 0x0C: 
			shift_12();      		// up 
			break;    		            							
	    case 0x03: 
			shift_03();           	// right 
			break;      
	    case 0x06: 
			shift_06();            	// down   
			break;    		            							    
	    case 0x09: 
			shift_09();           	// left 
			break;         	
	    case 0x88: 	
			remove_stop();                 
			break;   	 
	    case 0x5C: 	
			remove_12();  
			delay_process();							// down 
			break;          							
	    case 0x53: 
			remove_03();   							// left 
			delay_process();
			break;    
			  
	    case 0x56: 									// up
			remove_06();     
			delay_process();
			break;         							    
	    case 0x59: 
			remove_09();   		
			delay_process();					// right  
			break;    		            									  		            								    
	    default:                                  
			break;	
  	}
	if(StatusChangeLatch != CHECKI06) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		checki07_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of checki07 stauts---check press motor and sensor
//--------------------------------------------------------------------------------------	
void checki07_status(void)
{	
	UINT8 temp8;
	static UINT8 yj_counter;

  	//--------------------------------------------------------------------------------------
  	// config main motor driver add for 210E
  	//-------------------------------------------------------------------------------------- 
  	initial_mainmotor();
  	if(sys.status == ERROR)
  	{
  	  return;
  	}
	//--------------------------------------------------------------------------------------
  	//  find origin
  	//-------------------------------------------------------------------------------------- 
	if(origin_com == 1) //check i04
	{		
		temp8 = detect_position();	
		if(temp8 == OUT)   
  		{
			find_dead_center();
  		}	
		
	  	go_origin_allmotor();	   // all step motor go origin    
	  	if(sys.status == ERROR)
    	{
      		return;
    	} 
    	origin_com = 0;        		
  	}
 	if(pedal_style == single_pedal)
	{ 
		//--------------------------------------------------------------------------------------
	  	//  start sensor
	  	//-------------------------------------------------------------------------------------- 
		if(pedal_state == 0 && pedal_last_state != 0)
		{
			delay_ms(10);
			if(pedal_state == 0)
			{
				pedal_last_state = 0;
			}
		}
	  	if(pedal_state == 2)           						// start sensor is pushed
		{
			delay_ms(10);
			if(pedal_state == 2 && pedal_last_state == 0)
			{				
				pedal_last_state = 2;
				delay_ms(50);      
				if(k74 == 1)
  	  			  { 
				    go_yj(-35+trim_origin_pos,80); //2012-3-14 add
					delay_ms(80); 
					 LM_AIR = 0; 
				   }
	  			yj_counter = 1;
      			stepmotor_state = 0x00;
      			stepmotor_comm = 0xff;
		      	rec_com();       						// communication with panel	 	      	    
		  	}
	  	}
	}
	else if(pedal_style == double_pedal)
	{
		//--------------------------------------------------------------------------------------
	  	//  start sensor
	  	//--------------------------------------------------------------------------------------  
	  	if(DVA == 0)           								// start sensor is pushed
		{
			delay_ms(10);
			if(DVA == 0)
			{				
				delay_ms(50);
	            go_yj(-35+trim_origin_pos,80); //2012-3-14 add
				delay_ms(80);
				if(k74 == 1)
  	  			  { 
	                 LM_AIR = 0;
				   }
				yj_counter = 1;
      			stepmotor_state = 0x00;
      			stepmotor_comm = 0xff;
				while(DVA == 0)
		    	{
		      		rec_com();       						// communication with panel	
		    	}	 	      	    
		  	}
	  	}
	}
  	//--------------------------------------------------------------------------------------
  	//  manual shift step  
  	//--------------------------------------------------------------------------------------
  	switch(stepmotor_comm)
  	{
  		case 0x01: 
		          if(k74 == 1)    // foot up 
	                LM_AIR = 1;  
				  
  				  stepmotor_state = 1;  
  				  stepmotor_comm = 0xff;
  			break;      		            							
  	  case 0x02: 
  	  		if(k74 == 1)
	           LM_AIR = 0;     // foot down 
			stepmotor_state = 2;  
			stepmotor_comm = 0xff;  
			break;       
  	  case 0x03:
	  			if (yj_counter == 1) 
				   go_yj(-65,80); 
				else
				   go_yj(65,80);
		
			delay_ms(100); 
			stepmotor_state = 3;  
			stepmotor_comm = 0xff;  
			break;   // cut thread	                       		            							    
  	  case 0x04: 
	          if (yj_counter == 1)
			     go_yj(-45,60);  
			  else
			     go_yj(45,60);
				if (yj_counter == 1)
				   yj_counter = 2;
				else 
				   yj_counter = 1 ;
			
			delay_ms(80); 
			stepmotor_state = 4;  
			stepmotor_comm = 0xff;  
			break;   // cut                            	     		            									  		            								    
  	  default:                                                                  
			break;	
  	}
  	//--------------------------------------------------------------------------------------
  	//  single shift step 09.3.26 wr add 
  	//--------------------------------------------------------------------------------------
  	switch(stepmotor_single)
  	{
  		case 0x00://cw
			#if SUPPORT_UNIFY_DRIVER 
  			movestep_yj(-255,0);
			#else
			movestep_yj(-1,1);
			#endif
  		  	delay_ms(1);
  			stepmotor_single = 0xff;
  	  		break; 
  		case 0x01://ccw 
			#if SUPPORT_UNIFY_DRIVER 
  			movestep_yj(255,0);
			#else
			movestep_yj(1,1);
			#endif
  		  	delay_ms(1);
  			stepmotor_single = 0xff;
  			break;              	     		            									  		            								    
  	  default:
  	  	break;	
  	}
	
	if(StatusChangeLatch != CHECKI07) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}	
}
//--------------------------------------------------------------------------------------
//  Name:		checki08_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of checki08 stauts---check clamp thread motor and sensor
//--------------------------------------------------------------------------------------	
void checki08_status(void)
{	
	UINT8 temp8;
 	//--------------------------------------------------------------------------------------
  	// config main motor driver add for 210E
  	//-------------------------------------------------------------------------------------- 
  	initial_mainmotor();
  	if(sys.status == ERROR)
  	{
  	  return;
  	}
	//--------------------------------------------------------------------------------------
  	//  find origin
  	//-------------------------------------------------------------------------------------- 
	if(origin_com == 1) 
	{		
		temp8 = detect_position();	
		if(temp8 == OUT)    // out of range 
  		{
			find_dead_center();
  		}	
		 
    	origin_com = 0;        		
  	}
 	if(pedal_style == single_pedal)
	{ 
		//--------------------------------------------------------------------------------------
	  	//  start sensor
	  	//-------------------------------------------------------------------------------------- 
		if(pedal_state == 0 && pedal_last_state != 0)
		{
			delay_ms(10);
			if(pedal_state == 0)
			{
				pedal_last_state = 0;
			}
		}
	  	if(pedal_state == 2)           						// start sensor is pushed
		{
			delay_ms(10);
			if(pedal_state == 2 && pedal_last_state == 0)
			{				
				pedal_last_state = 2;
				
				go_origin_ct();    					    //ct go origin
      			stepmotor_state = 0x00;
      			stepmotor_comm = 0xff;
		      	rec_com();       						// communication with panel	 	      	    
		  	}
	  	}
	}
	else if(pedal_style == double_pedal)
	{
		//--------------------------------------------------------------------------------------
	  	//  start sensor
	  	//--------------------------------------------------------------------------------------  
	  	if(DVA == 0)           								// start sensor is pushed
		{
			delay_ms(10);
			if(DVA == 0)
			{				
				go_origin_ct();    					    //ct go origin
      			stepmotor_state = 0x00;
      			stepmotor_comm = 0xff;
				while(DVA == 0)
		    	{
		      		rec_com();       						// communication with panel	
		    	}	 	      	    
		  	}
	  	}
	}
  	switch(stepmotor_comm)
  	{
  	  case 0x01: 	
	  			go_origin_ct();
	  			delay_ms(300);
	  			clamp_out();               // clamp out                   
  			    stepmotor_state = 1;  
  			    stepmotor_comm = 0xff;  
  			    break;     
  			         		            							
  	  case 0x02: clamp_backstep1();         // clamp back step 1 
  	  	         delay_ms(30); 
  	  	         stepmotor_state = 2;  
  	  	         stepmotor_comm = 0xff;
				 if ( SUPPORT_CS3_FUN == 1)
				 	  output_cs3(0x10,0x10); //x32.2  
				 else
				 	  BLOW_AIR = 1;
   	  	         break;
  	  	                
  	  case 0x03: clamp_backstep2();         // clamp back step 2                       		            							   
  	  	         delay_ms(30); 
  	  	         stepmotor_state = 3;  
  	  	         stepmotor_comm = 0xff; 
  	  	         break; 
  	  	           
  	  case 0x04: //clamp_backstep3();         // clamp back step 3
  	  	         delay_ms(30); 
  	  	         stepmotor_state = 4;  
  	  	         stepmotor_comm = 0xff;  
				 if ( SUPPORT_CS3_FUN == 1)
				      output_cs3(0x10,0x10); //x32.2
				 else
				      BLOW_AIR = 0;
  	  	         break;                                	     		            									 
  	  	          		            								    
  	  default:   break;	
  	}
  	//--------------------------------------------------------------------------------------
  	//  single shift step 
  	//--------------------------------------------------------------------------------------
  	switch(stepmotor_single)
  	{
  		case 0x00://ccw   --out    //��ʱ��
				#if SUPPORT_NEW_DRIVER
				movestep_ct(-1,1);
				#else
	  			movestep_ct(-511,0);
				#endif
  			delay_ms(1);
  			stepmotor_single = 0xff;
  	  		break; 
  		case 0x01://cw    ++in  	//˳ʱ��	
				#if SUPPORT_NEW_DRIVER
				movestep_ct(1,1);
				#else
				movestep_ct(511,0);
				#endif
  			delay_ms(1);  		
  			stepmotor_single = 0xff;
  			break;              	     		            									  		            								    
  	  default:
  	  	break;	
  	}

	predit_shift = 0;
	if(StatusChangeLatch != CHECKI08) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		checki10_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of checki10 stauts---check inpress motor and sensor
//--------------------------------------------------------------------------------------	
void checki10_status(void)
{	
	UINT8 temp8;
	INT16 temp16,temp_speed;
	INT16 inpress_up_angle,inpress_down_angle;
	
  	initial_mainmotor();
  	if(sys.status == ERROR)
  	{
  	  return;
  	}
	//--------------------------------------------------------------------------------------
  	//  find origin
  	//-------------------------------------------------------------------------------------- 
	if(origin_com == 1) 
	{		
		temp8 = detect_position();	
		if(temp8 == OUT)   
  		{
			find_dead_center();
  		}			
	  	if(sys.status == ERROR)
    	{
      		return;
    	}
    	footer_both_down();    
		#if SECOND_GENERATION_PLATFORM || CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
		L_AIR = 1;
		#else	
		FA = 1;
		#endif
		if(k60 == 1)
		{
			foot_half_down();
		}			  
    	origin_com = 0;         		
  	}
 
 
 	#if FOLLOW_INPRESS_FUN_ENABLE && INPRESS_FOLLOW_MOTOR_ACTION
 	if( ( checki10_follow_flag == 1) && (inpress_type == NEW_MOTOR_INPRESS) )
	{
		temp16 = motor.angle_adjusted;
		temp_speed = (INT16)MotorSpeedRigister * 100;
		calculate_inpress_angle( temp_speed );
		inpress_up_angle   = angle_tab[inpress_follow_up_angle];
		inpress_down_angle = angle_tab[inpress_follow_down_angle];	    
		
		if( (temp16 > 0 )&&( temp16 < 150) )
		{
			checki10_action0 = 0;
			checki10_action1 = 0;
		}
						
		if( (temp16 > inpress_down_angle) && (checki10_action0 == 0)   )//��ѹ���½��Ƕ�
		{
  		   inpress_down(inpress_high_hole);
		   checki10_action0 = 1;
		   if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
		   {
				movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
				inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
		   }						
		}
					
		if( (temp16 > inpress_up_angle ) && (checki10_action1 == 0) )
		{
			checki10_action1 = 1;
			if( inpress_follow_high_flag == FOLLOW_INPRESS_LOW )
			{
				movestep_zx(inpress_follow_range,inpress_follow_up_speed);
				inpress_follow_high_flag = FOLLOW_INPRESS_HIGH;
			 }
		}
	}
	
	#endif
 	if(pedal_style == single_pedal)
	{ 
		//--------------------------------------------------------------------------------------
	  	//  start sensor
	  	//-------------------------------------------------------------------------------------- 
		if(pedal_state == 0 && pedal_last_state != 0)
		{
			delay_ms(10);
			if(pedal_state == 0)
			{
				pedal_last_state = 0;
			}
		}
	  	if(pedal_state == 2)           					
		{
			delay_ms(10);
			if(pedal_state == 2 && pedal_last_state == 0)
			{				
				pedal_last_state = 2;
				go_origin_zx();                        
      			stepmotor_state = 0x00;
      			stepmotor_comm = 0xff;
		      	rec_com();    
				   						      	    
		  	}
	  	}
	}
	else if(pedal_style == double_pedal)
	{
		//--------------------------------------------------------------------------------------
	  	//  start sensor
	  	//--------------------------------------------------------------------------------------  
	  	if(DVA == 0)           								// start sensor is pushed
		{
			delay_ms(10);
			if(DVA == 0)
			{				
				go_origin_zx();                        
      			stepmotor_state = 0x00;
      			stepmotor_comm = 0xff;
				checki10_follow_flag = 1;
				turnoff_buz();
				while(DVA == 0)
		    	{
		      		rec_com();       					
		    	}	 	      	    
		  	}
	  	}
	}
  	//--------------------------------------------------------------------------------------
  	//  manual shift step  
  	//--------------------------------------------------------------------------------------
  	switch(stepmotor_comm)
  	{
  		case 0x00: 
				go_origin_zx();
				inpress_to(71);  
				stepmotor_state = 0;  
				stepmotor_comm = 0xff;  
		break;   // origin 
  		case 0x01: 
				inpress_to(70);  
				stepmotor_state = 1;  
				stepmotor_comm = 0xff; 
		break;   // 7.0mm   		            							
        case 0x02: 					
			    inpress_to(48);  
				stepmotor_state = 2;  
				stepmotor_comm = 0xff;  
		break;   
  	  	case 0x03: 
		        inpress_to(0);   
				stepmotor_state = 3;  
				stepmotor_comm = 0xff;  
		break;                        		            							   
  	  	case 0x04: ;
				inpress_to(inpress_high_hole);  
				stepmotor_state = 4;  
				stepmotor_comm = 0xff;  
		break;   // -X.Xmm                             	     		            									  		            								    
  	  	default:   break;	
  	}
  	//--------------------------------------------------------------------------------------
  	//--------------------------------------------------------------------------------------
  	switch(stepmotor_single)
  	{
  		case 0x00://-
			#if SUPPORT_UNIFY_DRIVER
			movestep_zx(-1,12);
			#elif SUPPORT_NEW_DRIVER
			movestep_zx(-1,0);
			#else
  			movestep_zx(-511,0);
			#endif
  		  	delay_ms(1);
  			stepmotor_single = 0xff;

  	  		break; 
  		case 0x01://+ 
			#if SUPPORT_UNIFY_DRIVER
			movestep_zx(1,12);
			#elif SUPPORT_NEW_DRIVER
  			movestep_zx(1,0);
			#else
			movestep_zx(511,0);
			#endif
  		  	delay_ms(1);
  			stepmotor_single = 0xff;
			
  			break;              	     		            									  		            								    
  	  	default:
  	  	    break;	
  	}
	
	predit_shift = 0;
	if(StatusChangeLatch != CHECKI10) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
;void checki11_status_BOBBIN(void)
{
	UINT16 i,j;
	INT16 tmp;
	INT32 delay;
	UINT8 ret;
#if MACHINE_900_BOBBIN_DEBUG_MODE

#else	
	initial_mainmotor();
  	if(sys.status == ERROR)
  	{
  	  return;
  	}
	if( ally_step != -5670)//?5670�ĺ���
	{
		go_origin_allmotor();
		delay_ms(500);
		if( (sys.error != 0)&&(sys.status == ERROR) )
		return;
		go_commandpoint(0,-5670);//
	}
#endif
	if( (bobbin_case_switch_flag ==1 )&&(bobbin_case_enable == 1) )
	{
		bobbin_case_switch_flag = 0;
		if( bobbin_case_enable == 1)
			find_a_bobbin_case(5);
	}
	
	if( DVA == 0 )    
	{
		delay_ms(10);
		if( DVA == 0 )
		{				
			if( bobbin_case_enable == 1)
				bobbin_case_workflow1(); 	
		}
	}
	
	if( DVB == 0 )       
	{
		delay_ms(10);
		if( DVB == 0 )
		{				
			if( bobbin_case_enable == 1)
				go_origin_bobbin_case_arm(1); 	    
		 }
	}
	
	if (PAUSE == pause_active_level)
	{
			delay_ms(10);                           
			if(PAUSE == pause_active_level)
			{				
				if(DVA == 0)           							
				{
					delay_ms(10);
					if(DVA == 0)
					{				
						while( DVB == 1 )
				    	{
				      	   if( bobbin_case_enable == 1)
						   	   bobbin_case_workflow1();  
						   if( sys.status == ERROR)
						       break;  					
				    	}	 	      	    
				  	}
			  	}	  			  	
		  	}
	}
	
	//if( (para.rotate_cutter_working_mode == 55)	&& (rotated_cutter_running_flag == 1) )
	//{
	//	monitor_cutter_angle = get_cutter_motor_angle();
	//	da1 = monitor_cutter_angle >>1;
	//}
	
	if( checki11_test_flag == 1)
	{
		switch(checki11_item)
	  	{
	  		case 1://��ת����е�
				 if(rotated_cutter_enable == 1)
				 {
				    if( checki11_action == 1)//+��
					{
		  				movestep_cs3(0x4000,-1,5);
						sum_rotated_angle = sum_rotated_angle + 1;
						checki11_test_flag=0;
					}
					else 
					{
					    movestep_cs3(0x4000,1,5);
						sum_rotated_angle = sum_rotated_angle - 1;
						checki11_test_flag=0;
					}
				}
	  		  	delay_ms(6);				
	  		break; 
	  		case 2://��������
	  			if( checki11_output_status[0] == 0)
				{
					DRILL_MOTOR_UPDOWN = 1;
					checki11_output_status[0] = 1;
					//output_cs3(2,2);//port1
				}
				else
				{
					DRILL_MOTOR_UPDOWN = 0;
					checki11_output_status[0] = 0;
					//output_cs3(2,0);//port1
				}
				checki11_test_flag = 0;
			break;  
			case 3://ѹ������
				if( checki11_output_status[1] == 0)
				{
					DRILL_FOOTER = 1;
					checki11_output_status[1] = 1;
					#if DSP3_CUTER_DRIVER
					delay_ms(350);
					output_cs3(1,1);//port0
					#endif
				}
				else
				{
					DRILL_FOOTER = 0;
					checki11_output_status[1] = 0;
					#if DSP3_CUTER_DRIVER
					output_cs3(1,0);
					#endif
				}
				checki11_test_flag = 0;
			break;
			case 4://�е�����
				if( para.rotate_cutter_working_mode == 55)	
				{		
					if( checki11_output_status[2] == 0)
					{
						checki11_output_status[2] = 1;
						
						if( para.cutter_speed >2000)
							para.cutter_speed = 2000;
						set_rotated_cutter_speed( para.cutter_speed );
						//SUM = 1;
						/*
						rotated_cutter_running_flag = 1;
						da1 = 0;
						counter_1ms = 0;
						while(1)
						{
							monitor_cutter_angle = get_cutter_motor_angle();
							da1 = monitor_cutter_angle >>1;
							ret = get_YJORG_status();
							while(ret == 0)	//�ȴ�һȦ��ͬ���ź�
							{
								ret = get_YJORG_status();								
								rec_com();
								monitor_cutter_angle = get_cutter_motor_angle();
								da1 = monitor_cutter_angle >>1;
							}
							ret = get_YJORG_status();	
							da1 = monitor_cutter_angle >>1;
							if (PAUSE == pause_active_level)
							{
									delay_ms(10);                           
									if(PAUSE == pause_active_level)
									{
										set_rotated_cutter_speed( 0 );
										delay_ms(500);
										go_origin_yj();
										break;
									}	
							}
							if( counter_1ms >= 10000)//30s
							{
								set_rotated_cutter_speed( 0 );
								delay_ms(2500);
								//go_origin_yj();
								delay_ms(2500);
								set_rotated_cutter_speed( para.cutter_speed );
								counter_1ms = 0;
							}
						}
						*/
					}
					else
					{
						checki11_output_status[2] = 0;
						set_rotated_cutter_speed( 0 );
						//delay_ms(100);
						//go_origin_yj();
						//SUM = 0;
						rotated_cutter_running_flag = 0;
					}
				}
				else
				{
					if( checki11_output_status[2] == 0)
					{
						if( DRILL_MOTOR_UPDOWN ==1)
							drill_motor_run_enable = 1;
						checki11_output_status[2] = 1;
					}
					else
					{
						drill_motor_run_enable = 0;
						if( DRILL_MOTOR_UPDOWN ==1)
						{
						#if DSP3_CUTER_DRIVER
							output_cs3(2,2);//port1
							delay_ms(200);
							output_cs3(2,0);
						#else
						
						#endif		
						}				
						checki11_output_status[2] = 0;
					}
				}
				checki11_test_flag = 0;
			break; 
			case 5://������
			#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER40		
				if( checki11_action == 1)//+��
		  			movestep_yj(1,1);
				else 
					movestep_yj(-1,1);
		  		delay_ms(10);	
				checki11_test_flag = 0;
			#else
				if( bobbin_case_enable == 1)
				{
					if( checki11_action == 1)//+��
		  				movestep_cs3(0xa000,-10,15);
					else 
					    movestep_cs3(0xa000,10,15);
		  		  	delay_ms(20);
				}
			#endif
			break; 
			case 6://ץ�۵��
				if( bobbin_case_enable == 1)
				{
					if( checki11_action == 1)//+��
		  				movestep_cs3(0x2000,-1,5);
					else 
					    movestep_cs3(0x2000,1,5);
		  		  	delay_ms(10);
				}
			break; 
			case 7://�н�����
				if( checki11_output_status[3] == 0)
				{
					BOBBIN_CASE_ARM_SCRATH = 1;
					checki11_output_status[3] = 1;
				}
				else
				{
					BOBBIN_CASE_ARM_SCRATH = 0;
					checki11_output_status[3] = 0;
				}
				checki11_test_flag = 0;
			break; 
			case 8://ץ��ǰ������
				if( checki11_output_status[4] == 0)
				{
					BOBBIN_CASE_ARM_OUT = 1;
					checki11_output_status[4] = 1;
				}
				else
				{
					BOBBIN_CASE_ARM_OUT = 0;
					checki11_output_status[4] = 0;
				}
				checki11_test_flag = 0;
			break; 
			case 9://���󵥲�����
				if( bobbin_case_enable == 1)
				{
					find_a_bobbin_case(5);
					checki11_test_flag = 0;
				}
			break; 
			case 10://��������λ
				if( bobbin_case_enable == 1)
				{
					go_origin_bobbin_case_arm(0); 
				}
				checki11_test_flag = 0;
			break; 
			case 11://��ת�е���λ
				if( rotated_cutter_enable == 1)
					go_origin_rotated_cutter();
				#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER40	
				go_origin_yj();	
				#endif
				checki11_test_flag = 0;
			break;    
			
			case 12://ֱ���е�����
				#if ENABLE_LOOPER_CUTTER
				if( stepper_cutter_enable == 1)
			 	{
					 if( checki11_action == 1)//+��
					 {
						 movestep_lct(1,0);
					 }
					 else
					 {
						 movestep_lct(-1,0);
					 }
				 }
				#endif
				checki11_test_flag = 0;
			 break;     
			case 13://ֱ���е���ԭ��
			#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER40
			//if( para.rotate_cutter_working_mode == 55)
			{
				switch(cutter_test_cnt)
				{
					case 0:
						//go_origin_rotated_cutter();	
						cutter_rotated_abs_angle = 45;
						rotated_cutter_by_data();
						cutter_test_cnt++;					
					break;
					case 1:
						cutter_rotated_abs_angle = 90;
						rotated_cutter_by_data();
						cutter_test_cnt++;
					break;
					case 2:
						cutter_rotated_abs_angle = 180;
						rotated_cutter_by_data();
						cutter_test_cnt++;
					break;
					case 3:
						cutter_rotated_abs_angle = 270;
						rotated_cutter_by_data();
						cutter_test_cnt =0;
					break;
				}				
				
			}
			#endif
			#if ENABLE_LOOPER_CUTTER
				if( cutter_test_cnt == 0)
				{
					go_origin_stepper_cutter();
					cutter_test_cnt = 1;
				}
				else if( cutter_test_cnt == 1)
				{
					for( i=0;i< 200;i++)
					{
						movestep_lct(1,0);
						delay_us(600);
					}
					cutter_test_cnt = 2;
				}
				else if( cutter_test_cnt == 2)
				{
						go_origin_stepper_cutter();
						delay_ms(100);
						for( i=0;i< stepper_cutter_position;i++)
						{
							movestep_lct(1,0);
							delay_us(600);
						}
						cutter_test_cnt = 3;					
				}
				else
				{
					tmp = stepper_cutter_shake_rage; 
					movestep_lct(-tmp,63);
					delay_ms(63);
					cutter_test_cnt = 0;
				}
				
			#endif					
				checki11_test_flag = 0;
			break;   
			case 14://�����Դ
				if( laser_power_on_enable == 1)
				{
					if( laser_test_flag[0] == 0)
					{
						LASET_POWER_SWITCH = 1;
						laser_test_flag[0] = 1;
					}
					else
					{
						LASET_POWER_SWITCH = 0;
						laser_test_flag[0] = 0;
					}
				}
				checki11_test_flag = 0;
			break;
			case 15://����ѹ������
					if( laser_test_flag[1] == 0)
					{
						LASET_MIRROR_COVER = 1;
						laser_test_flag[1] = 1;
					}
					else
					{
						LASET_MIRROR_COVER = 0;
						laser_test_flag[1] = 0;
					}
					checki11_test_flag = 0;
			break;
			case 16://�����и��ź�
					LASER_POWER_PIN = 1;
					delay_ms(200);
					LASER_POWER_PIN = 0;
					checki11_test_flag = 0;
			break;
			case 17://�������
					if( laser_test_flag[3] == 0)
					{
						LASER_FUN_PIN = 1;
						laser_test_flag[3] = 1;
					}
					else
					{
						LASER_FUN_PIN = 0;
						laser_test_flag[3] = 0;
					}
					checki11_test_flag = 0;
			break;
			case 18://����ָʾ��
					if( laser_test_flag[4] == 0)
					{
						LASER_INDICATIOR_LED = LASER_LED_ON;
						laser_test_flag[4] = 1;
					}
					else
					{
						LASER_INDICATIOR_LED = 0;
						laser_test_flag[4] = 0;
					}
					checki11_test_flag = 0;
			break; 	     		            									  		            								    
	  	}
		//delay_ms(100);
	}
}
/*

*/
void checki11_status(void)
{	
	UINT8 temp8;
	#if ENABLE_LASER_CUTTER
	#else
	if( (ENABLE_BOBBIN_CASE_FUN == 1)||(stepper_cutter_enable==1)||ROTATED_CUTTER==1 )
	#endif
	{
		checki11_status_BOBBIN();	
	}
	
	predit_shift = 0;
	if(StatusChangeLatch != CHECKI11) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		emermove_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of emermove stauts
//--------------------------------------------------------------------------------------	
void emermove_status(void)
{	
	UINT8 temp8;
	if(OutOfRange_flag == 0)
	{
		if(emermove_high == 0) 
		{
			predit_shift = 1;
			if(inpress_flag == 1)
			{
				if(pause_flag == 1)              			
				{	
					sys.error = ERROR_19;	  			// pause button is not in normal position  		
					status_now = sys.status;
					emermove_high = 71;
					predit_shift = 0;
					sys.status = ERROR;
					StatusChangeLatch = ERROR;  	
					return;    	      	    
				}
				else if(pause_flag == 0)
				{
					inpress_down(emermove_high);

					if((k03 == 0) && (fk_status == OUT) && (fk_cut == 0))  
					{
						da0 = 255;
						fk_count = 1;
						fk_status = IN;
						delay_ms(100); 
						da0 = release_tension_current;    
				    }
				}
			}                              			
			predit_shift = 0;   	
		}	
		else if(emermove_high == 71)
		{
			predit_shift = 1;
			if(inpress_flag == 0)
			{
				if(pause_flag == 1)              			
				{			
					sys.error = ERROR_19;	  			// pause button is not in normal position 
					status_now = sys.status;
					emermove_high = 0;
					predit_shift = 0;
					sys.status = ERROR;
					StatusChangeLatch = ERROR;   	
					return;    	      	    
				}
				else if(pause_flag == 0)
				{
					inpress_up();
					
					if(k03 == 0)       
					  {
					     da0 = 0;
					     SNT_H = 0;  
					     fk_cut = 0;
					     fk_count = 1;
					     fk_status = OUT;
					  }
					
				}
			}
			predit_shift = 0;
		}
	
		if(pedal_style == single_pedal)
		{
			if(pedal_state == 0 && pedal_last_state != 0)
			{
				delay_ms(10);
				if(pedal_state == 0)
				{
					pedal_last_state = 0;
				}
			} 
	
		  	if(pedal_state == 2)           								// start sensor is pushed
			{
				delay_ms(10);
				if(pedal_state == 2 && pedal_last_state == 0)
				{				
					if( (foot_flag == 0) || ( (foot_flag == 1)&&(u202 == 1) ) )						
					{	
						pedal_last_state = 2;			 
	      				if(pause_flag == 1)              			
						{			
								sys.error = ERROR_19;	  			// pause button is not in normal position
								status_now = sys.status;
								sys.status = ERROR;
							    StatusChangeLatch = ERROR;      	
							      return;    	      	    
						}
						else if(pause_flag == 0)
						{		  
								temp8 = detect_position();	
					    		if(temp8 == OUT)   
				      			{
									find_dead_center();
				      			}
								if(nop_move_pause_flag ==1)
								     process_nop_move_pause(1);
							
								if(pat_point == sta_point)
								{
									
									if(origin2_lastmove_flag == 1)
									{
										process_data();
										if(nopmove_flag == 1)
									    {	  
											while(nopmove_flag == 1)
											{
												do_pat_point_sub_one();
									    	  	go_beginpoint(0); 
												process_data();
												if(nopmove_flag == 1)
												{

												}
												else 
												{
													do_pat_point_sub_one();
													move_flag = 0;
													break;
												}
											}
										}
										origin2_lastmove_flag = 0;
									}		   	
								}
								
							    if(k03 == 0)       
								{
									da0 = 0;           
									SNT_H = 0;  
									fk_cut = 0;
									fk_count = 1;
									fk_status = OUT;
								}
								 	
					        	sys.status = RUN;	
								StatusChangeLatch = RUN; 	    			  	
							}
						}
			    	}
		  	}//=3
		}
		else if(pedal_style == double_pedal)
		{
			if(DVB == 0)           					// foot sensor is pushed,
			{
					delay_ms(10);
					if(DVB == 0 && DVBLastState == 1)
					{
						if(foot_flag == 1)
						{		 
				      		 footer_both_down();     
				    	}
				  		else if(foot_flag == 0)
						{
							 footer_both_up();     
							
			    		}
					}
			}
			DVBLastState = DVB;
				
			//--------------------------------------------------------------------------------------
		  	//  start sensor
		  	//-------------------------------------------------------------------------------------- 
		  	if(DVA == 0)           				// start sensor is pushed
			{
				delay_ms(10);
				if(DVA == 0 && DVALastState == 1)
				{
			    	if(foot_flag == 0)
					{				   	
			        	//--------------------------------------------------------------------------------------
						//  pause sensor
						//--------------------------------------------------------------------------------------   
						if(pause_flag == 1)              				// pause sensor is pushed
						{			
							sys.error = ERROR_19;	  			// pause button is not in normal position 
							status_now = sys.status;
							sys.status = ERROR;
						    StatusChangeLatch = ERROR;     	
							predit_shift = 0; 
						    return;    	      	    
						}
						else if(pause_flag == 0)
						{		
							temp8 = detect_position();	
				    		if(temp8 == OUT)    // out of range 
			      			{
								find_dead_center();
			      			}
							
							if(nop_move_pause_flag ==1)
								process_nop_move_pause(1);
								
							if(pat_point == sta_point)
							{
								
								if(origin2_lastmove_flag == 1)
								{
									process_data();
									if(nopmove_flag == 1)
								    {	  
										while(nopmove_flag == 1)
										{
											do_pat_point_sub_one();
								    	  	go_beginpoint(0); 
											process_data();
											if(nopmove_flag == 1)
											{

											}
											else 
											{
												do_pat_point_sub_one();
												move_flag = 0;
												break;
											}
										}
									}
									origin2_lastmove_flag = 0;
								}		   	
							}
							
							if(k03 == 0)       
							  {
								da0 = 0;
								SNT_H = 0;  
								fk_cut = 0;
								fk_count = 1;
								fk_status = OUT;
							  }
							   	
				        	sys.status = RUN;
							StatusChangeLatch = RUN; 	 	    			  	
						}	 	    			  	
			    	}
			  	}
		  	}
		 	DVALastState = DVA;
		}
	  	//--------------------------------------------------------------------------------------
	  	//  single step move 
	  	//--------------------------------------------------------------------------------------
	  	switch(single_flag)
	  	{
	  		case 0:	                                     
			      predit_shift = 0;                               
				break;
	  		case 1:
				if(pause_flag == 1)              			
				{			
					sys.error = ERROR_19;	  					// pause button is not in normal position 
					status_now = sys.status;
					single_flag = 0;
					do_pat_point_sub_one();
					predit_shift = 0;
					sys.status = ERROR;
					StatusChangeLatch = ERROR;  	
					return;    	      	    
				}
				else if(pause_flag == 0)
				{	
					predit_shift = 1; 
					temp8 = detect_position();	
		    		if(temp8 == OUT)    // out of range 
		      		{
						find_dead_center();
		      		}
					if(nop_move_pause_flag ==1)
					{
						 process_nop_move_pause(1);
					}
					allx_step = allx_step + xstep_cou;
		  			ally_step = ally_step + ystep_cou;
			
					if( check_sewing_range())
					{
						allx_step = allx_step - xstep_cou;
		  				ally_step = ally_step - ystep_cou;
						do_pat_point_sub_one();
						single_flag = 0;
						if(move_flag == 1 || nopmove_flag == 1)
						{
							move_flag = 0;
							nopmove_flag = 0;
						}
						sys.error = ERROR_15;
						StatusChangeLatch = ERROR;
					}
					else
					{
						allx_step = allx_step - xstep_cou; 
  						ally_step = ally_step - ystep_cou; 
						move_next();
					}
					predit_shift = 0;
				}                       
				break;
	  		case 2:
				if(pause_flag == 1)              				
				{			
					sys.error = ERROR_19;	  			// pause button is not in normal position 
					status_now = sys.status;
					single_flag = 0;
					do_pat_point_add_one();
					predit_shift = 0;
					sys.status = ERROR;
					StatusChangeLatch = ERROR;   	
					return;    	      	    
				}
				else if(pause_flag == 0)	
				{ 
					predit_shift = 1;
					temp8 = detect_position();	
		    		if(temp8 == OUT)    // out of range 
		      		{
						find_dead_center();
		      		}
					if(nop_move_pause_flag ==1)
					{
						 process_nop_move_pause(2);
					}
					move_back();  
					predit_shift = 0;
				}                    
				break;  		
	  		case 3:
				if(pause_flag == 1)              				
				{			
					sys.error = ERROR_19;	  			// pause button is not in normal position  
					status_now = EMERMOVE;
					single_flag = 0;
					predit_shift = 0;
					sys.status = ERROR;
					StatusChangeLatch = ERROR;  	
					return;    	      	    
				}
				else if(pause_flag == 0)	
				{ 
					predit_shift = 1;
					inpress_high = inpress_high_hole;
					tension = tension_hole;
					if(u42 == 0)
					{
						if(u94 == 1)
						{  
							find_dead_point();
						}
					}
					//if(u39 == 1)
					    go_origin_allmotor();
					//else
					//	move_startpoint();
					delay_ms(20);
					nop_move_pause_flag = 0;
					if(k110 == 1)
					{
					   R_AIR = 0;
					   delay_ms(80); 
					}
					if(u38 == 0)
					{
			  		  	footer_both_up();  
					}
					delay_ms(1);
					inpress_up(); 
			 		single_flag = 0; 
					if(u42 == 0)
					{
						if(u94 == 1)
						{
							find_up_position();
						}
					}    					   
					if(aging_com == 1)
					{
						aging_com = 0;
					}
					predit_shift = 0;
					TestStatusChangeFlag = 0;
					
					if(u35==0)                     // have clamp thread motor //08.12.31 wr add this action
        			{
          				if(clamp_com == 1)           // check clamp flag
          				{
          					if(clamp_flag == 0)        // clamp is in
    						{
    							clamp_out();
    							clamp_stepflag = 1;      // clamp thread step 1
          			  			movect_angle = 800;      // 800---281 degree
    						}
    						if(TSENS == 0)             // sensor is covered
	    					{	
	    						delay_ms(100);
        						if(TSENS == 0)
        						{
      	  							sys.status = ERROR;
									StatusChangeLatch = ERROR;
									if( sys.error == 0)
          							sys.error = ERROR_23;	 // clamp is not in normal position  		  
          							return;
        						}
      						}
          				}
          				else
          				{
          					if(clamp_flag == 1)        // clamp is out
            				{
            		  			sys.status = ERROR;
								StatusChangeLatch = ERROR;
								if( sys.error == 0)
            		  			sys.error = ERROR_23;	   // clamp is not in normal position	  
            		  			return;
            				}
            				else
            				{
          			  			clamp_stepflag = 0;      // clamp thread step 0
          					}
          				}
        			} 
					FootRotateFlag = 0;
			  		sys.status = READY;
					StatusChangeLatch = READY; 	 
				}
	  		    break;	
			case 4:	 
				if(pause_flag == 1)              			
				{			
					sys.error = ERROR_19;	  					// pause button is not in normal position 
					status_now = sys.status;
					single_flag = 0;
					do_pat_point_sub_one();
					predit_shift = 0;
					sys.status = ERROR;
					StatusChangeLatch = ERROR;  	
					return;    	      	    
				}
				else if(pause_flag == 0)
				{	 
					predit_shift = 1;
					temp8 = detect_position();	
		    		if(temp8 == OUT)    // out of range 
		      		{
						find_dead_center();
		      		}
					go_beginpoint(0);
					single_flag = 0;
					predit_shift = 0;
				}
				break;			
  			case 5:
				if(pause_flag == 1)              				// pause sensor is pushed
				{			
					sys.error = ERROR_19;	  					// pause button is not in normal position 
					status_now = sys.status;
					single_flag = 0;
					do_pat_point_add_one();                                //2011-6-15 songyang modify
					predit_shift = 0;
					sys.status = ERROR;
					StatusChangeLatch = ERROR;  	
					return;    	      	    
				}
				else if(pause_flag == 0)
				{	 
					predit_shift = 1;
					temp8 = detect_position();	
		    		if(temp8 == OUT)    // out of range 
		      		{
						find_dead_center();
		      		}
					if(nop_move_pause_flag ==1)
					    process_nop_move_pause(2);
					else
						back_endpoint();
					single_flag = 0;
					predit_shift = 0;
				}
				break;	
		case 6:	 
			temp8 = detect_position();	
    		if(temp8 == OUT)    // out of range 
      		{
				find_dead_center();
      		}
			course_next();  
			delay_ms(5);                    
			break;		
  		case 7:	 	
			temp8 = detect_position();	
    		if(temp8 == OUT)    // out of range 
      		{
				find_dead_center();
      		}
			course_back();
			delay_ms(5);                      
			break;			
  		case 8:	 
			course_stop();                      
			break;	
		case 9:
				if(pause_flag == 1)              				// pause sensor is pushed
				{			
					sys.error = ERROR_19;	  					// pause button is not in normal position 
					status_now = sys.status;
					single_flag = 0;
					predit_shift = 0;
					sys.status = ERROR;
					StatusChangeLatch = ERROR;  	
					return;    	      	    
				}
				else if(pause_flag == 0)
				{	 
					predit_shift = 1;
					temp8 = detect_position();	
		    		if(temp8 == OUT)    // out of range 
		      		{
						find_dead_center();
		      		}
					go_setoutpoint();
					predit_shift = 0;
					single_flag = 0; 
				}
				break;
			case 10:
				if(pause_flag == 1)              				// pause sensor is pushed
				{			
					sys.error = ERROR_19;	  					// pause button is not in normal position 
					status_now = sys.status;
					single_flag = 0;
					predit_shift = 0;
					sys.status = ERROR;
					StatusChangeLatch = ERROR;  	
					return;    	      	    
				}
				else if(pause_flag == 0)
				{	 
					predit_shift = 1;
					temp8 = detect_position();	
		    		if(temp8 == OUT)    // out of range 
		      		{
						find_dead_center();
		      		}
					go_manualpoint();
					single_flag = 0;
					predit_shift = 0;
				}
				break;	  		                                           		
	  		default:                                     
				break;	
	  	}
	}
	else if(OutOfRange_flag == 1)
	{
		//--------------------------------------------------------------------------------------
	  	//  single step move 
	  	//--------------------------------------------------------------------------------------
	  	switch(single_flag)
	  	{
	  		case 0:	                                     
				break;
	  		case 1:                
				break;
	  		case 2:        
				break;  		
	  		case 3:
				predit_shift = 1;
				inpress_high = inpress_high_hole;
				tension = tension_hole;
					
				if(u42 == 0)
				{
					if(u94 == 1)
					{  
						find_dead_point();
					}
				}
				if(u39 == 1)
				    go_origin_allmotor();
				else
					move_startpoint();
					
				OutOfRange_flag = 0;
				single_flag = 0; 
				delay_ms(20);
				if(k110 == 1)
				{
					R_AIR = 0;
					delay_ms(80);  
				}
				if(u38 == 0)
				{
		  		     footer_both_up();   
				}
				delay_ms(1);
				inpress_up(); 
		 
				if(u42 == 0)
				{
					if(u94 == 1)
					{
						find_up_position();
					}
				}    
				   
				if(aging_com == 1)
				{
					aging_com = 0;
				}
				predit_shift = 0;
				TestStatusChangeFlag = 0;
				OutOfRange_flag = 0;	
				FootRotateFlag = 0;
		  		sys.status = READY;
				StatusChangeLatch = READY; 	 
	  		    break;		  		                                           		
	  		default:                                     
				break;	
	  	}
	}
	
}

//--------------------------------------------------------------------------------------
//  Name:		pre_run_conditioning 
//  Parameters:	None
//  Returns:	None
//  Description: pre-process the function code before start Run Status.
//               nopmove, origin2_last_move, stop_flag, PatternDelayFlag are usually defined 
//               as special function code that needed to be pre-processed before running
//--------------------------------------------------------------------------------------
void pre_run_conditioning(void)
{	
	#if DEBUG_RFID_DA1_FUN	
	da1 = 250;
	#endif
	if(synchronization_flag == 1)
	   return;
	#if AUTO_CHANGE_PATTERN_FUNCTION == 0	 
 	if( (bar_coder_refresh_enable == 1)&&(bar_coder_refresh_flag == 0) )//����ǰȷ�ϣ�������ûˢ��������
		return;
	//if(pattern_change_flag == 1 )
	//   return;
	while( pattern_change_flag == 1 )
	{
		 rec_com();
		 check_output_pattern_done();
	}
	#endif
	
	if( waitting_for_pattern_done == 1)
	{
		if ( identify_mode == 1)
		     delay_ms(500);
		//else 
		//     delay_ms(200);
		waitting_for_pattern_done = 0;
	}
	
	if(AUTO_SEARCH_START_POINT == 1)
	{
		if( already_auto_find_start_point == 0 )
		{
			find_start_point();
			already_auto_find_start_point = 1;
		}
	}

    if( (sewingcontrol_flag == 2)&&(sewingcontrol_stitchs != 0) )
		 need_backward_sewing = 1;	 
	if(origin2_lastmove_flag == 1)
	{
			  origin2_lastmove_flag = 0;
			  stop_flag = 0;
			  process_data();
		      if(origin2_lastmove_flag == 1)
			  {
				process_data();
			  }
			 
		      if(move_flag ==1 )
		      {
			     do_pat_point_sub_one();
		 	  }		   
  	          //==============================
	    	  if(nopmove_flag == 1)
			  {	  
				  while(nopmove_flag == 1 || PatternDelayFlag == 1)
				  {
						do_pat_point_sub_one();						
					    if(PatternDelayFlag == 1)
						{
							PatternDelayFlag = 0;
						}
						else
					    {
						    go_beginpoint(0);
						} 
						process_data();
						
						if(nopmove_flag == 1)
						{
											
						}
						else if(PatternDelayFlag == 1)
						{
					        delay_ms(PattenrDelayTime);
					    	PatternDelayFlag = 0;
					    }								
						else if(nopmove_flag == 0 || move_flag == 1 || stop_flag == 1)
						{
							do_pat_point_sub_one();
							move_flag = 0;
							if(stop_flag == 1)
							{
								 StopStatusFlag = 1;
								 if((stop_number%2 == 1)&&(u41 == 0)) 
							     {
									 footer_both_up();  
								 }
							}
							
							break;
						}
					}
			   }
			   origin2_lastmove_flag = 0;
	  }			  
      else if(StopStatusFlag == 1)	
	 {
		 		if(origin2_lastmove_flag == 1 || stop_flag == 1)
				{
					 process_data();
					 origin2_lastmove_flag = 0;
					 stop_flag = 0;
					 process_data(); 
					 
				     if(origin2_lastmove_flag == 1 || stop_flag == 1 || move_flag == 1 || end_flag == 1) //2013-7-29 modify
					 {
						 do_pat_point_sub_one();
				     }					
					 else if(PatternDelayFlag == 1)
					 {
						 delay_ms(PattenrDelayTime);
						 PatternDelayFlag = 0;
					 }								
					 if(nopmove_flag == 1)
					 {	  
							while(nopmove_flag == 1 || PatternDelayFlag == 1)						
							{
								do_pat_point_sub_one();
						    	if(PatternDelayFlag == 1)						
								{
								   PatternDelayFlag = 0;
							    }						
								else
								{
									go_beginpoint(0);
								} 			
								process_data();
								if(nopmove_flag == 1)
								{
										
								}						
								else if(PatternDelayFlag == 1)
								{
								   delay_ms(PattenrDelayTime);
								   PatternDelayFlag = 0;
							    }		
								else if(nopmove_flag == 0 || move_flag == 1 || stop_flag == 1)
								{
								   do_pat_point_sub_one();
								   move_flag = 0;
								   break;
								}					
							}											
				   	 }
				     origin2_lastmove_flag = 0;
			    }	
		   
		} 	  
		
	    if(stop_flag == 1)
		{
			sys.status = READY;
			StatusChangeLatch = READY;
			StopStatusFlag = 1; 
			if((stop_number%2 == 1)&&(u41 == 0))
			{
			    footer_both_up();  
			}				
		}	 
		else
		{
			 sys.status = RUN;
			 StatusChangeLatch = RUN;
			 StopStatusFlag = 0;	
			 
			 if( aging_flag == 1)
			 {
			   	aging_com = 1;
				if(u233 == 2)
				{ 
					u39 = 1;
				}
			 }	   
	   }		 
}


void delay_process(void)
{
   switch(one_step_delay)
   {
	   case 1:
	        delay_ms(2);
	   break;
	   case 2:
	   	   if( speed_up_counter < 5)
		   {
			   delay_us(300);
			   speed_up_counter ++;
		   }
	   	   delay_us(800);
	   break;
	   case 3:
	   		if( speed_up_counter < 5)
		   {
			   delay_us(500);//1100
			   speed_up_counter ++;
		   }
		   else if( speed_up_counter < 10)
	       {
			   delay_us(350);//950
			   speed_up_counter ++;
		   }
		   else if( speed_up_counter < 15)
		   {
			   delay_us(100);//700
			   speed_up_counter ++;
		   }
	   		delay_us(600);
	   break;
   }
}

typedef unsigned char ( *pt2FunctionErase)(unsigned long, unsigned short * );
#define BOOTLOADER_ADDR 0xfb000L
void download_status(void)
{
	pt2FunctionErase fp;
	UINT32 far* ptr;
	UINT32 entry;
	
	ptr = (unsigned long far *)( BOOTLOADER_ADDR );
	if(*ptr != 0xffffffff)
	{
		predit_shift = 0;
		//==================protect
		U=1;V=1;W=1;
	  	prcr = 0x02;
		inv03 = 0;;
		U_=0;V_=0;W_=0;
	  	prcr = 0x00;
		OUTPUT_ON = 1;
		//=================
		entry = BOOTLOADER_ADDR ;
		fp = (pt2FunctionErase)entry;
		fp( 0, 0);
	}
}


void downloaddsp_status(void)
{
	UINT16 delay,delay_t;
	switch(sys.status)
	{
		case DOWNLOAD_DSP1:
			 download_drv_flag = 1;
	 	break;
		case DOWNLOAD_DSP2:
			 download_drv_flag = 2; 
		break;
		case DOWNLOAD_DSP3:
			 download_drv_flag = 3; 
		break;
		case DOWNLOAD_DSP4:
			 download_drv_flag = 4; 
		break;
	}
	delay_t = 15000;
	if(DVB == 0)  
	{
			delay_ms(10);
			if(DVB == 0)
			{	
				delay_t = 25000;			
		  	}
	}
	if(DVA == 0)           								// start sensor is pushed
	{
			delay_ms(10);
			if(DVA == 0)
			{	
				delay_t = 35000;			
		  	}
	}
	if(5 == predit_shift) 		//1���յ�һ������
	{  
		if(1 == erase_falg) 	//�������Ϊ0,��֪ͨ������������״̬
		{
		    jump_to_begin();
		}
		send_stepmotor_up_drv();//���յ�һ������ͨ��SPI���͸�����
		if(1 == erase_falg)     //��ʱ�ȴ��������
		     delay_ms(delay_t); 
		else  
		     delay_ms(40);         
		drv_satus = read_stepmotor_up_drv();
		//0XA0    ���������
		//0XA1    �ļ�У�����
		//0XA2    ���ݰ���У�����
		//0XA3    SPIͨ��У�����
		//0XA4    flash��������
		//0XA5    flash��д����
		//0XA6    flashУ�����
		//0XA7    ���ݰ�crcУ�����
		//0XA8    ��������
		//0xa9    ��ʱ
		delay = 0;
		while(0x00 != drv_satus)
		{
			rec_com();
			delay_ms(5);
			drv_satus = read_stepmotor_up_drv();
			delay++;
			if( delay >1000)//��ʱ
			{
			    sys.status = ERROR;
				sys.error = 0x34;
				de_bug.test1 = 0x00;
				de_bug.test2 = 0xa9;
				break;	
			}
			if( (drv_satus >=0xa0) && (drv_satus <=0xaf) )
			{
				sys.status = ERROR;
				sys.error = 0x34;
				de_bug.test1 = 0x00;
				de_bug.test2 = drv_satus;
				break;
			}
		}
		predit_shift = 0;

	}
	if(6 == predit_shift ) //2��������У��֤
	{  
		send_stepmotor_end_drv();
        predit_shift = 0;
	}
    rec_com();		
}
void bobbin_change_status(void)
{
	UINT8 temp8;
	temp8 = bobbin_case_workflow1();
	bobbin_case_once_done_flag = 0;
	sys.status = READY;
	sys.error = 0;
}

void download_dsp_curve_status(void)
{
	#if SUPPORT_UNIFY_DRIVER
	UINT8 i;
	if( write_stepmotor_curve_flag != 0)
	{
	    if( write_stepmotor_curve(write_stepmotor_curve_flag, pat_buf) )
		{
			for(i=0;i<write_stepmotor_curve_flag;i++)
			{
				SUM = 1;
				delay_ms(1000);
				SUM = 0;
				delay_ms(1000);				
			}
			write_stepmotor_curve_flag = 0;			
		}	
		else
		{
			sys.status = ERROR;
			sys.error =  ERROR_52;
			de_bug.test1 = 0x00;
			de_bug.test2 = 0xa7;
		}
		predit_shift = 0;
	}
	#endif
	if(StatusChangeLatch != DOWNLOAD_DSP_CURVE) 
	{
		sys.status = StatusChangeLatch;
	}
}


void multipule_io_status(void)
{	
    #if MULTIPULE_IO_ENABLE  ==1
	read_all_io_output();
	delay_ms(25);
	read_all_io_input();	
	delay_ms(25);
	#endif
	if( StatusChangeLatch != MULTIPULE_IO )
	{
		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}

void download_multipul_program_status(void)
{
  #if MULTIPULE_IO_ENABLE  ==1
  UINT16 delay;
  if(5 == predit_shift) 
  {  
	    if(1 == erase_falg)
	       multipule_program_beginning(4);
	   
	    send_multipule_program_data(4);//����������������
	    if(1 == erase_falg)
	        delay_ms(2500); 
	    else  
	    	delay_ms(60);
	
	    drv_satus = read_multipule_program_status(4);//��ȡ����״̬
	    delay = 0;
	    while(0x00 != drv_satus)
	    {
	        rec_com();
	        delay_ms(5);
	        drv_satus = read_multipule_program_status(4);//��ȡ����״̬
	        delay++;
	        if( delay >1000)//��ʱ
	        {
	          sys.status = ERROR;
	          sys.error = ERROR_86;
	          de_bug.test1 = 0x00;
	          de_bug.test2 = 0xa9;
	          break;	
	        }
	        if( (drv_satus >=0xa0) && (drv_satus <=0xaf) )
	        {
	          sys.status = ERROR;
	          sys.error = ERROR_86; //50 �Ŵ��󣿣�������������������
	          de_bug.test1 = 0x00;
	          de_bug.test2 = drv_satus;
	          break;
	        }
	     }
      	 predit_shift = 0;
  }
  
  if(6 == predit_shift ) //2��������У��֤
  {  
      multipule_program_end(4);
      predit_shift = 0;
  }
  rec_com();
  #endif		
}


void rfid_read_write(void)
{

	#if ENABLE_RFID_FUNCTION	
	if(auto_function_flag == 1)
	{
		if((formwork_identify_device == 3)&&(rc522_control_falg==1))//RFID
		{	
			SUM =1;
			RFID_SCAN();
	        rfid_wr_ret();
			rc522_control_falg = 0;
		}
	}
	#endif
	predit_shift = 0;
 	if(StatusChangeLatch != RFID_WR) 
	{
		sys.status = StatusChangeLatch;
	}
}

//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
