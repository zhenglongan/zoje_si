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
#include "..\..\include\pattern_check.h"  //2018-9-17,花样码检查相关
#include "..\..\include\inpress_follow_mainmotor.h"//2018-11-12,手轮转动主轴中压脚随动
#include "..\..\include\automatic_feeding.h"//2019-1-11,自动送料装置相关
#include "..\..\include\program_watch.h"       // program watching definition
#include "..\..\include\machine_id.h"       // program watching definition

extern void send_dsp1_command(UINT16 command,UINT16 data);    


#define double_pedal 2
#define single_pedal 1

/**
  * @函数功能 记号笔处理
  
  车缝+剪线+空送+（偏移）+记号笔+记号笔+（回位）+空送 
  车缝+剪线+空送+（偏移）+记号笔+空送+记号笔 +（回位）+空送
  车缝+剪线+空送+（偏移）+记号笔+空送+记号笔 +（回位）+空送
  
  两个记号笔码为一组，第一个走偏移，第二个返回偏移，处理发到空送之外，不要影响到空送急停
*/
void marking_pen_stop(void)
{
	while( rec1_total_counter > 0 )//？？？？串口1有数据调用UART0处理？
	{
		//2019-3-24 新增面板响应函数rec_com()的防重入机制
		if(ui_borad_reentrant_protection == 0)
		{
			ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
			rec_com();       				// communication with panel 
			ui_borad_reentrant_protection = 0;//其他地方又可以使用了
		}  
	}
	tb4s = 0;//关闭TB4定时器，TB4用于delay_us()计时，同时用于画笔的插补运动
	MARKING_PEN = 0;
	laser_cutter_aciton_flag = 0;	
}
void process_marking_pen(UINT8 flag)
{
	INT32 allx_step_tmp,ally_step_tmp;
	UINT16 dly;
	UINT8 slow_flag ,action_flag ,i ,protect_flag,stop_status,first_flag,stitch_cnt;

	PW_SET_FUNC_ID(39);//2020-6-6 zla 记录当前所在函数位置
	
#if SUPPORT_SEWING_MARKING_PEN//支持记号笔用车缝数据表示形状
  if( sys.status == RUN)
  {	
	//先处理偏移
	if( (milling_first_move == 0)&&(pen_x_bios_offset!=0)||(pen_y_bios_offset!=0) )//偏移不为0
	{
		//2019-4-3 禁止在偏移时出现急停，此时不响应急停
		shift_no_pause=1;
		
		allx_step_tmp = allx_step;
		ally_step_tmp = ally_step;
		go_commandpoint(pen_x_bios_offset + allx_step,pen_y_bios_offset + ally_step);//移动到画笔偏移地址
		allx_step = allx_step_tmp;//函数go_commandpoint会重置allx_step、ally_step，这里不把偏移量计算在内
		ally_step = ally_step_tmp;
		milling_first_move = 1;//保证只偏移一次
		
		shift_no_pause=0;//恢复对急停的响应
		
		MARKING_PEN = 1;//打开画笔
	}
	delay_ms(200);
	rec1_total_counter = 0;
	first_flag = 1;
	/*
	时钟是24MHz 就是计数24000000个用时1秒 24000对应1ms 100us => 2400
	1.5ms = 36000
	1-9: 1档最快 500us  9档--4.5ms	
	
	*/
	
	dly = marking_speed;
	dly = dly*2400+9600;//(4+marking_speed)*100us
	tb4 = dly;//100us中断定时器TB4暂时用于画笔插补算法，后续要恢复计数初值
	laser_cutter_aciton_flag = 0;
	//逐针扫描处理记号笔
	while(1)
	{
		//如果是除了急停之外的其他错误，那么跳出循环
		if( (sys.status == ERROR)&&(sys.error != ERROR_02) )
		     break;
		
		//急停的处理	
		if( PAUSE == pause_active_level)//press sotp button when stepper motor moving
	  	{
			delay_ms(10);//防抖动
			if( PAUSE == pause_active_level)
			{
				//2019-4-2 已经写入缓冲区的插补数据需要走完才行
				marking_pen_stop();
				stitch_cnt = 0;
				//tb4s = 0;     
				//sys.status = READY;
//				sys.status = ERROR;
//				sys.error = ERROR_02;
//				StatusChangeLatch = ERROR;
				SET_SYS_ERROR(ERROR_02);//急停
				stop_status = 1;
			}
		}
		//暂停的处理
		if( stop_status == 1) 		 
		{
			stitch_cnt = 0;
			while( 1 )
			{
				 //扫描启动按钮
				  if( DVA == 0 && sys.status == READY && PAUSE!=pause_active_level) 
				  {
					  delay_ms(20);
					  if( DVA == 0 && sys.status == READY && PAUSE!=pause_active_level) 
						{
						   sys.status = RUN;
						   stop_status = 0;
						   predit_shift = 0;
						   first_flag = 1;
						   break;
						}
				  }	

		  		if( (sys.status == ERROR)&&(StatusChangeLatch == READY))//报错后恢复
				{
					sys.status = READY;
					predit_shift = 0;
					sys.error = 0;
				}
		
				  delay_ms(1);
				  //按找原点处理
				  if( origin_com == 1 )
				  {
				  		//end_flag=1;//退出小while(1)后将退出函数，在run函数里直接退出，避免后续当车缝处理了
						predit_shift = 0; 
						single_flag = 0;
						//opl_origin_flag = 0;//传感器找原点处理							
						stop_status = 0;//用于退出while小循环
						break;
				  }
				  //跳转
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
				  //单步	
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
		
		//遇到结束符
		if(end_flag == 1)
		{
			if(inpress_flag == 0)  
			   inpress_up();
			break;
		}
		//急停后按回原点键
		if( origin_com == 1 )
			break;
			
		process_data();	//处理一针数据
		
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
			if (first_flag == 1)//启动
			{
				tb4 = dly; //这一步已经做过了
				tb4s = 1;
				laser_cutter_aciton_flag = 1;//设置为1，TB4中断函数中才能执行插补算法
				first_flag = 0;
			}			
			PBP_Line(stitch_cnt);//参数未使用，逐点比较法直线插补算法
		}
		//忽略剪线
		if( cut_flag == 1)
		    cut_flag = 0;
		//画笔结束		
		if(	(making_pen_status == 4)&&(	FootRotateFlag == 1) )
		{
			 marking_pen_stop();
			 stitch_cnt = 0;
			 first_flag = 1;
			 if( (milling_first_move == 1)  &&( (pen_x_bios_offset!=0)||(pen_y_bios_offset!=0) ) )
			{ 
				//2019-4-3 禁止在偏移时出现急停，此时不响应急停
				shift_no_pause=1;
				 allx_step_tmp = allx_step;
				 ally_step_tmp = ally_step;			
				 go_commandpoint(allx_step-pen_x_bios_offset,ally_step- pen_y_bios_offset);	  
				 allx_step = allx_step_tmp;
				 ally_step = ally_step_tmp;	
				 shift_no_pause=0;//恢复对急停的响应
			}
			milling_first_move = 0;
			break;
		} 
		//空送处理
		if(nopmove_flag == 1)	
		{
				marking_pen_stop();
			 	stitch_cnt = 0;
			 	first_flag = 1;
				while( nopmove_flag == 1 )
				{
					do_pat_point_sub_one();
					sys.status = READY;
				    go_beginpoint(0); //分段空送处理
					sys.status = RUN;
					process_data();	
					if(OutOfRange_flag == 1)
					{
					   end_flag = 1;
					   sys.status = ERROR;
//					   sys.error = ERROR_15;
					   SET_SYS_ERROR_1(ERROR_15);//超出缝制范围
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
		//2019-3-24 新增面板响应函数rec_com()的防重入机制
		if(ui_borad_reentrant_protection == 0)
		{
			ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
			rec_com();       				// communication with panel 
			ui_borad_reentrant_protection = 0;//其他地方又可以使用了
		}    
	}//while 1

	marking_pen_stop();
	stitch_cnt = 0;
	first_flag = 1;
	//画笔处理完成，回到偏移前的地址
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
	if( marking_flag == 1)//遇到记号笔功能码
	{
	   if( marking_finish_flag ==0 )//第一次遇到记号笔码
	   {
		   if( (pen_x_bios_offset!=0)||(pen_y_bios_offset!=0) )//偏移不为0
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
		   //delay_ms(150 + k114);
		   delay_ms(150);//2019-7-26 zla
		   marking_finish_flag = 1;
	   }
	   else//第二次遇到记号笔码
	   {
		 #if SUPPORT_CS3_FUN
		   output_cs3(8,0);
		 #else

		   MARKING_PEN ^= 1;
		 #endif
		   marking_flag = 0;
		   //delay_ms(150+k114);
		   delay_ms(150);//2019-7-26 zla
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
  * @函数功能 获取识别的模板号
  * @参数      formwork_identify_device
  *     @arg 0: 表示传感器识别方式
  *     @arg 1: 表示条码识别方式
  *
  * @返回值 识别的模板号
  *     @arg 0:     无效模板
  *     @arg 1~999: 有效模板
  * 
  * 8 个传感器版本
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
	#if 0
	if(formwork_identify_device == 0) 
	{
		if(PORG == 1)
			  tmp_pattern |= 0x01;//输入1
		if(PSENS == 1)
			  tmp_pattern |= 0x02;//输入2
		if(CSENS == 1)
			  tmp_pattern |= 0x04;//输入3
		if(CORG == 1)
			  tmp_pattern |= 0x08;//输入4
		if(ADTCSM )
			  tmp_pattern |= 0x10;//输入5
	}
	else if(formwork_identify_device == 1) 
	{
		if(PORG == 1)
			  tmp_pattern |= 0x01;//输入1
		if(CORG == 1)
			  tmp_pattern |= 0x02;//输入2
		if(CSENS == 1)
			  tmp_pattern |= 0x04;//输入3
		if(PSENS == 1)
			  tmp_pattern |= 0x08;//输入4
		if(ADTCSM )
	 		  tmp_pattern |= 0x10;//输入5
		if(SENSOR6 == 1)
			  tmp_pattern |= 0x20;//输入6
		if(SENSOR7 == 1)
			  tmp_pattern |= 0x40;//输入7
		if(SENSOR8 )
	 		  tmp_pattern |= 0x80;//输入8
	    		  
	}
	else if(formwork_identify_device >= 2)	
	#endif
	{
	 	 tmp_pattern = serail_number;
	}
	return tmp_pattern;

}

/**
  * @函数功能  剪线动作函数
  * @参数      
  * @返回值 
  *
  * @控制使用的全局变量或参数
  *     @k43 --剪线速度
  *     @k95 --剪线动作角度微调
  *     @u09 --松线角度微调
  *     @u42 --剪线功能开关 0-打开 1-关闭
  *     @k03 --夹线器类型   0-机械 1-电子
  *     @u42 --停车后是否停在上死点 0-上位置 1-上死点
  *     @u46 --拨线器开关   0-关闭 1-打开
  *
  * @受影响的全局变量 
  */
  
//机型18、58
#if NEW_CUT_MODE
void trim_action(void)
{
		INT16 temp16; 
		UINT8 flag;

		#if SEWING_TENSION_USER_DEFINED_ENABLE
		if( para.sewing_tension_user_defined_enable == 55 )
		{
			da0=0;//剪线时必须关闭
		}
		#endif

		//1.0  主轴转速降到剪线速度上
		motor.dir = 0;
		motor.spd_obj = k43*10;  
		
		temp16 = motor.angle_adjusted;
        while( temp16 <= angle_tab[50] ) 
	    { 
        	//2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();       				// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}  
			temp16 = motor.angle_adjusted;
		}
		if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
	    {
			movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
			inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
		}
		
		//2.0  等待主轴到达剪线角度，到了开刀
	    temp16 = motor.angle_adjusted;
        //while( temp16 <= angle_tab[240-8*k95] ) 
         while( temp16 <= angle_tab[240+k95] ) 
	    { 
        	//2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();       				// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}  
			if(sys.status == POWEROFF)
			      return;
        	temp16 = motor.angle_adjusted;          
        }            	      		 		   
		process_flag = 0;
		//if(u46 == 0)//剪线开关打开
		if(u46 == TRIM_SWITCH_OPEN)//剪线开关打开
		{
		   #if SECOND_GENERATION_PLATFORM 
			   FL = 1;//二代一体机剪线电磁铁动作
			   FL_pwm_counter = 0;
			   FL_pwm_period = 400;//(UINT16)k113*10;
			   FL_pwm_action_flag = 1;	   
		   #else
			   SNT_H = 1;    
			   L_AIR = 1;
		   #endif		   
		   cutter_protect_counter = 0;
		   cutter_protect_flag = 1;
		 }
		 //3.0 打开剪刀以后，主轴降速到100
		 #if SECOND_GENERATION_PLATFORM
		 #else
		 motor.spd_obj = 100;
		 #endif
		 //4.0 等待松线角度，到了松线
		 temp16 = motor.angle_adjusted;
		 //while( temp16 <= angle_tab[300-8*u09] )
         while( temp16 <= angle_tab[300+u09] )
		 { 
    	     //2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();       				// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}  
			 if(sys.status == POWEROFF)   
			      return;
	         temp16 = motor.angle_adjusted;
			 
         }
		 //if(k03 == 0)
		 if(k03 == TENSION_TYPE_MECHANICAL)
         {
			 da0 = 255;
		     DAActionFlag=1;//启动1秒强制关闭保护策略
		     DAActionCounter =0;
         }
		 //5.0 给出停车角度，并等待主轴停稳
		 PW_SET_STOP_CODE(35);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
		 sewing_stop();	
		 
		 temp16 = motor.angle_adjusted;
		 while( temp16 > angle_tab[100] )//等待新的一圈
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
					//if( (u46 == 0)&&(u51 ==1) )//U51等于1就表示FW是拨线电磁铁功能了，而不是小夹线器
					if( /*(u46 == 0)*/(u46 == TRIM_SWITCH_OPEN)
						&&(U9_35_wipper_type == WIPER_TYPE_SOLENOID) )//U51等于1就表示FW是拨线电磁铁功能了，而不是小夹线器
					{
						flag = 0;
						#if FW_TYPE_SOLENOID_VALVE
						  	  #if SECOND_GENERATION_PLATFORM
								FK_OFF = 1;//小模板机，拨线电磁铁
							  #else
							    FR_ON = 1;
							  #endif		  
						#else
							SNT_H = 1;        // 24V to 33V
						  	FW = 1;//拨线电磁铁
						#endif
					}					
				}
				if( temp16 >= k61 - 14 )//150=>53
					break;
		 }		 
		 		 
		 //6.0  停稳后，关闭剪线、拨线信号			
		  //if(u46 == 0)
		  if(u46 == TRIM_SWITCH_OPEN)//剪线开关打开
		  {		   
			  da0 = 0;
			  //delay_ms(55);
			  #if SECOND_GENERATION_PLATFORM 
			  FL = 0;
			  FL_pwm_action_flag = 0;
			  FL = 0;
			  #else
			  L_AIR = 0;
			  SNT_H = 0;
			  #endif
		  }

		  //7.0 如果要求停到上死点，进行相应的处理
      	  //if(u42 == 1)
      	  if(u42 == NEEDLE_STOP_POSITION_UP_DEAD_POINT)
		  {
			  delay_ms(155);
		      find_dead_point();
		  }
		  //8.0  执行拨线动作
		//气阀拨线，或者或吹气拨线
		  //if(blow_air_enable == 1)
		  if(blow_air_enable == BLOW_AIR_STITCH_OPEN_AFTER_TRIM)
		  {
		  	  #if 0
			  if( k171 ==1 )
			  	BLOW_AIR3 = 1;
			  else
			  	BLOW_AIR2 = 1;
			  
			  blow_air_counter = (UINT16)k166*100; 
			  blow_air_flag = 1;
			  #else
				blow_air_need_action_flag = 1;
				
			  #endif
		  }
		  else
		  {
			  blow_air_flag=0;
		  }
		  //u46=0，打开剪线功能
		  //if( u46 == 0 )//小夹线器或者拨线功能
		  if(u46 == TRIM_SWITCH_OPEN)//剪线开关打开
		 	  fw_solenoid();//电磁铁拨线
		  while( motor.stop_flag == 0)
		  {
		  	//2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();       				// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}  
		  }
		  start_to_speed_down = 0;  
		  cutter_speed_done_flag = 0;
		  
}

#else

//除了机型18、55以外的竖屏机型
void trim_action(void)
{
		INT16 temp16; 
		UINT32 sys_time_old;
		
		PW_SET_CODE_ID(4500);//2020-6-8 zla 记录当前所在代码段位置
		
		//1.0  主轴转速降到剪线速度上
		motor.dir = 0;
		motor.spd_obj = k43*10;  

		//降下中压脚
		if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
	    {
			movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
			inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
		}
		
		PW_SET_CODE_ID(4501);//2020-6-8 zla 记录当前所在代码段位置
		g_timeout_counter = 3000;
		//2.0  等待主轴到达剪线角度，到了开刀
	    temp16 = motor.angle_adjusted;
		//2019-1-5 为三丝杠模板机（机型：47）新增圆刀电机剪线功能
		#if ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
        while( temp16 <= angle_tab[(trim_para.trim_motor_branch_angle)%360] ) 
		#else
		//while( temp16 <= angle_tab[240-8*k95] ) 
		while( temp16 <= angle_tab[240+k95] )
		#endif
	    { 
	    	PW_SET_CODE_ID(4502);//2020-6-8 zla 记录当前所在代码段位置
        	//2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();       				// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}  
			if(sys.status == POWEROFF)
			{
				PW_SET_CODE_ID(4503);//2020-6-8 zla 记录当前所在代码段位置
				return;
			}
        	temp16 = motor.angle_adjusted;
        	
			if( g_timeout_counter == 0 )
			{
//				sys.status = ERROR;
//				if( sys.error == OK ) sys.error = ERROR_205;
				SET_SYS_ERROR(ERROR_205);//延时报错退出机制
				PW_SET_CODE_ID(4530);//2020-6-6 zla 记录当前所在代码段位置
				return;
			}
        }
        PW_SET_CODE_ID(4504);//2020-6-8 zla 记录当前所在代码段位置
		process_flag = 0;
		//if(u46 == 0)//u46=0表示允许剪线
		if(u46 == TRIM_SWITCH_OPEN)//剪线开关打开
		{
		   //2019-1-5 为三丝杠模板机（机型：47）新增圆刀电机剪线功能
		   #if ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
				if(0)//一步到位
				{
					//电机剪线直接一步到位剪断
					movestep_trim(-(int)trim_para.trim_motor_range, trim_para.trim_motor_break_time);
			   		cutter_delay_counter = trim_para.trim_motor_break_time+40;//多等40ms
					cutter_delay_flag = 1;//开始计时
				}
				else
				{
					//电机剪线第1步：到达分线位置，分线尖到达针尖位置
					movestep_trim(-(int)trim_para.trim_motor_branch_position, trim_para.trim_motor_branch_time);
		   			cutter_delay_counter = trim_para.trim_motor_branch_time;
					cutter_delay_flag = 1;//开始计时
				}
				
		   #else

			
			   #if SECOND_GENERATION_PLATFORM 
			   FL = 1;//二代一体中使用的剪线电磁铁
			   FL_pwm_counter = 0;
			   FL_pwm_period = 400;//(UINT16)k113*10;
			   FL_pwm_action_flag = 1;
			   #else 
			   SNT_H = 1;    
//			   L_AIR = 1;
			   #endif		   
			   cutter_protect_counter = 0;
			   cutter_protect_flag = 1;
		   #endif
		 }
		 PW_SET_CODE_ID(4505);//2020-6-8 zla 记录当前所在代码段位置
		 //3.0 打开剪刀以后，主轴降速到100
		 #if SECOND_GENERATION_PLATFORM
		 #else
		 motor.spd_obj = 100;
		 #endif
		 g_timeout_counter = 3000;
		 //4.0 等待松线角度，到了松线
		 temp16 = motor.angle_adjusted;
         //while( temp16 <= angle_tab[300-8*u09] )
         while( temp16 <= angle_tab[300+u09] )
		 { 
    	     //2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();       				// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}  
			 if(sys.status == POWEROFF)
			 {
			 	PW_SET_CODE_ID(4506);//2020-6-8 zla 记录当前所在代码段位置
			      return;
			 }
	         temp16 = motor.angle_adjusted;
	         if( g_timeout_counter == 0 )
			{
//				sys.status = ERROR;
//				if( sys.error == OK ) sys.error = ERROR_205;
				SET_SYS_ERROR(ERROR_205);//延时报错退出机制
				PW_SET_CODE_ID(4531);//2020-6-6 zla 记录当前所在代码段位置
				return;
			}
         }
		 //if(k03 == 0)
		 if(k03 == TENSION_TYPE_MECHANICAL)
         {
			 da0 = 255;
		     DAActionFlag=1;//启动1秒强制关闭保护策略
		     DAActionCounter =0;
         }
         PW_SET_CODE_ID(4507);//2020-6-8 zla 记录当前所在代码段位置
		 //5.0 给出停车角度，并等待主轴停稳
		 PW_SET_STOP_CODE(36);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
		 sewing_stop();	
		 PW_SET_CODE_ID(4508);//2020-6-8 zla 记录当前所在代码段位置
		 
		 temp16 = motor.angle_adjusted;
		
	//2019-1-5 为三丝杠模板机（机型：47）新增圆刀电机剪线功能
	#if ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
		if(0)//一步到位
		{
			
		}
		else//先分线再剪线
		{
			//if(u46 == 0)//u46=0表示允许剪线
			if(u46 == TRIM_SWITCH_OPEN)//剪线开关打开
			{
				PW_SET_CODE_ID(4509);//2020-6-8 zla 记录当前所在代码段位置
				g_timeout_counter = 3000;
				while( cutter_delay_flag == 1)//等待分线完成
				{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();       				// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(4532);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
				}
				PW_SET_CODE_ID(4510);//2020-6-8 zla 记录当前所在代码段位置
				//电机剪线第2步：到达断线位置，将线剪断
				movestep_trim(-(int)(trim_para.trim_motor_range-trim_para.trim_motor_branch_position), trim_para.trim_motor_break_time);
				delay_ms(trim_para.trim_motor_break_time+40);//多延时了40ms，保证能够将线剪断
				//2019-6-11 衔接效率提升开关（剪线+空送+起缝
				#if BRIDGING_EFFICIENCY_IMPROVEMENT
				#else
				cutter_delay_counter = trim_para.trim_motor_break_time+40;//多等40ms
				cutter_delay_flag = 1;//开始计时
				#endif
				PW_SET_CODE_ID(4511);//2020-6-8 zla 记录当前所在代码段位置
			}
		}
		
		
	#else
	
		 #if SECOND_GENERATION_PLATFORM
		 PW_SET_CODE_ID(4512);//2020-6-8 zla 记录当前所在代码段位置
		 g_timeout_counter = 3000;
		 while(motor.stop_flag == 0)
		 {
			//2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();       				// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}  
			if(sys.status == POWEROFF)
			{
				PW_SET_CODE_ID(4513);//2020-6-8 zla 记录当前所在代码段位置
			      return;
			}
			if( g_timeout_counter == 0 )
			{
//				sys.status = ERROR;
//				if( sys.error == OK ) sys.error = ERROR_205;
				SET_SYS_ERROR(ERROR_205);//延时报错退出机制
				PW_SET_CODE_ID(4533);//2020-6-6 zla 记录当前所在代码段位置
				return;
			}
		 }
		 #else
		 PW_SET_CODE_ID(4514);//2020-6-8 zla 记录当前所在代码段位置
		 g_timeout_counter = 3000;
         while( temp16 > angle_tab[100] )
		 { 
    	     //2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();       				// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}  
			 if(sys.status == POWEROFF)
			 {
			 	PW_SET_CODE_ID(4515);//2020-6-8 zla 记录当前所在代码段位置
			      return;
			 }
	         temp16 = motor.angle_adjusted;
	         if( g_timeout_counter == 0 )
			{
//				sys.status = ERROR;
//				if( sys.error == OK ) sys.error = ERROR_205;
				SET_SYS_ERROR(ERROR_205);//延时报错退出机制
				PW_SET_CODE_ID(4534);//2020-6-6 zla 记录当前所在代码段位置
				return;
			}
         }
         PW_SET_CODE_ID(4516);//2020-6-8 zla 记录当前所在代码段位置
         g_timeout_counter = 3000;
   		 while( temp16 < k61-14 )
		 { 
    	     //2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();       				// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}  
			 if(sys.status == POWEROFF)
			 {
			 	PW_SET_CODE_ID(4517);//2020-6-8 zla 记录当前所在代码段位置
			      return;
			 }
	         temp16 = motor.angle_adjusted;
			 if(motor.stop_flag == 1)
			   break;
			if( g_timeout_counter == 0 )
			{
//				sys.status = ERROR;
//				if( sys.error == OK ) sys.error = ERROR_205;
				SET_SYS_ERROR(ERROR_205);//延时报错退出机制
				PW_SET_CODE_ID(4535);//2020-6-6 zla 记录当前所在代码段位置
				return;
			}
         }
         PW_SET_CODE_ID(4518);//2020-6-8 zla 记录当前所在代码段位置
   	    #endif
		
	#endif


		PW_SET_CODE_ID(4519);//2020-6-8 zla 记录当前所在代码段位置
		 //6.0  停稳后，关闭剪线、拨线信号			
		  //if(u46 == 0)
		  if(u46 == TRIM_SWITCH_OPEN)//剪线开关打开
		  {		   
			  da0 = 0;
			  //delay_ms(55);
			  #if SECOND_GENERATION_PLATFORM 
			  FL = 0;
			  FL_pwm_action_flag = 0;
			  delay_ms(1);
			  FL = 0;
			  #else
//			  L_AIR = 0;
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
		  //7.0 如果要求停到上死点，进行相应的处理
      	  //if(u42 == 1)
      	   if(u42 == NEEDLE_STOP_POSITION_UP_DEAD_POINT)
		  {
			  delay_ms(155);
			  PW_SET_CODE_ID(4520);//2020-6-8 zla 记录当前所在代码段位置
		      find_dead_point();
		      PW_SET_CODE_ID(4521);//2020-6-8 zla 记录当前所在代码段位置
		      if( sys.status == ERROR )
					return;
		  }

		  //2019-7-8 应胡龙的要求，暂时增加一套参数用于剪线后抓线吸气打开
		  if(para.after_trim_scratch_air_start_time != 255 && para.after_trim_scratch_air_time != 0)
		  {
		  		scratch_blow_air_need_action_flag = 1;
		  		scratch_blow_air_need_action_counter = 0;
		  }
		  
		  //8.0  执行拨线动作,如果使能了起缝吹气，剪线就不吹气了
		  //if(blow_air_enable == 1)
		  if(blow_air_enable == BLOW_AIR_STITCH_OPEN_AFTER_TRIM)
		  {
			  #if 0
			  if( k171 ==1 )
			  	BLOW_AIR3 = 1;
			  else
			  	BLOW_AIR2 = 1;
			  
			  blow_air_counter = (UINT16)k166*100; 
			  blow_air_flag = 1;
			  #else
				blow_air_need_action_flag = 1;
			  #endif
				
		  }
		  else
		  {
			  blow_air_flag=0;
		  }
		  //#endif
		  //if(u46 == 0)
		  if(u46 == TRIM_SWITCH_OPEN)//剪线开关打开
        	  fw_solenoid();  
		  start_to_speed_down = 0;  
		  cutter_speed_done_flag = 0;
		  PW_SET_CODE_ID(4522);//2020-6-8 zla 记录当前所在代码段位置
		  g_timeout_counter = 3000;
		  while(motor.stop_flag == 0)
		  {
			//2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();       				// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}  
			if(sys.status == POWEROFF)
			{
				PW_SET_CODE_ID(4523);//2020-6-8 zla 记录当前所在代码段位置
			      return;
			      
			}
			if( g_timeout_counter == 0 )
			{
//				sys.status = ERROR;
//				if( sys.error == OK ) sys.error = ERROR_205;
				SET_SYS_ERROR(ERROR_205);//延时报错退出机制
				PW_SET_CODE_ID(4536);//2020-6-6 zla 记录当前所在代码段位置
				return;
			}
		  } 
		PW_SET_CODE_ID(4524);//2020-6-8 zla 记录当前所在代码段位置

		//2019-1-5 为三丝杠模板机（机型：47）新增圆刀电机剪线功能
		#if ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
		//if(u46 == 0)//u46=0表示允许剪线
		if(u46 == TRIM_SWITCH_OPEN)//剪线开关打开
		{
			PW_SET_CODE_ID(4525);//2020-6-8 zla 记录当前所在代码段位置
			g_timeout_counter = 3000;
			while( cutter_delay_flag == 1)
			{
				//2019-3-24 新增面板响应函数rec_com()的防重入机制
				if(ui_borad_reentrant_protection == 0)
				{
					ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
					rec_com();       				// communication with panel 
					ui_borad_reentrant_protection = 0;//其他地方又可以使用了
				}
				if( g_timeout_counter == 0 )
				{
//					sys.status = ERROR;
//					if( sys.error == OK ) sys.error = ERROR_205;
					SET_SYS_ERROR(ERROR_205);//延时报错退出机制
					PW_SET_CODE_ID(4537);//2020-6-6 zla 记录当前所在代码段位置
					return;
				}
			}
			PW_SET_CODE_ID(4526);//2020-6-8 zla 记录当前所在代码段位置
			//2019-6-11 衔接效率提升开关(剪线+空送+起缝)
			#if BRIDGING_EFFICIENCY_IMPROVEMENT
			movestep_trim((int)(trim_para.trim_motor_range), 63);
			cutter_delay_counter = 100;//等个100ms
			cutter_delay_flag = 1;//开始计时
			trim_motor_need_find_origin_flag = 1;//表示剪线后需要在后续的空送中找剪线电机原点
			#else
			//2019-1-17 中捷胡龙认为回原点速度过慢，这里先往回走到接近原点位置，然后再找原点，速度会快一些
			if(trim_para.trim_motor_range > 20)
			{
				movestep_trim((int)(trim_para.trim_motor_range-10), 100);
				delay_ms(100);
			}
			PW_SET_CODE_ID(4527);//2020-6-8 zla 记录当前所在代码段位置
			//电机剪线第2步：到达断线位置，将线剪断
			go_origin_trim();	
			PW_SET_CODE_ID(4528);//2020-6-8 zla 记录当前所在代码段位置
			#endif
		}
		#endif
		
		PW_SET_CODE_ID(4529);//2020-6-8 zla 记录当前所在代码段位置

}

#endif


//2018-11-14
//新增函数用于让主轴转动一圈，然后再回退指定针数
void backwords_after_thread_break(UINT8 mode, UINT8 stitchs)
{
	UINT8 i;
	INT16 temp16; 
	UINT8 temp8;

	/*-------------------------首先主轴转动一圈-------------------------		*/
	motor.dir = 0;
	motor.spd_obj = k43*10;//按照剪线速度先动起来 
	temp16 = motor.angle_adjusted;
	//while( temp16 <= angle_tab[240-8*k95] ) 
	while( temp16 <= angle_tab[240+k95] ) 
	{ 
		//2019-3-24 新增面板响应函数rec_com()的防重入机制
  		if(ui_borad_reentrant_protection == 0)
  		{
  			ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
  			rec_com();       				// communication with panel 
  			ui_borad_reentrant_protection = 0;//其他地方又可以使用了
  		}
  		
		if(sys.status == POWEROFF)	 
			  return;
		temp16 = motor.angle_adjusted;			
	}
	PW_SET_STOP_CODE(22);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
	sewing_stop();//停车
	while(motor.stop_flag == 0)
	{
		//2019-3-24 新增面板响应函数rec_com()的防重入机制
  		if(ui_borad_reentrant_protection == 0)
  		{
  			ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
  			rec_com();       				// communication with panel 
  			ui_borad_reentrant_protection = 0;//其他地方又可以使用了
  		}
		if(sys.status == POWEROFF) 
		      return;
	}
	
	temp8 = detect_position();	
	if(temp8 == OUT)    
  	{
	   find_dead_center();
  	}
	
	footer_both_down();
	inpress_up();
	delay_ms(120);

	
	/*-----------------------------逐针回退-----------------------------	*/
	single_flag = 2;
	for( i = 0;i<stitchs ;i++ )
	{
		if(mode == 1)//可以跨越空送段
		{
			course_back();
		}
		else if(mode == 0)//不可以跨越空送
		{
			course_back_stop_before_nopmove();
		}
		if( single_flag == 0)
			break;
	}
	single_flag = 0;
}



void process_nop_move_pause(UINT8 direction)
{
	UINT32 i,temp16_x,temp16_y,temp16_max,quick_time,temp32;
	INT32 tempx_step,tempy_step;
	UINT8 fast_flag,_check_flag;//2019-2-20 新增空送具体判断功能码
	
	if( direction == 2)//需要从空送停止位置返回到空送的原起点
	{
		//1.0 计算要回退多少距离
		temp16_y = fabsm(read_step_y);
		temp16_x = fabsm(read_step_x);
		tempx_step = read_step_x;
		tempy_step = read_step_y;
		//2.0 恢复起点的花样指针、坐标、处理总针数
		pat_point = last_pattern_point;
		allx_step = last_allx_step;//再回复，就不会回复到V新尾部加固开始空送的位置了
		ally_step = last_ally_step;
		pat_buff_total_counter = bakeup_total_counter;
	}
	else //从空送停止位置向前移动
	{
		//1.0 计算还有多少距离没走
		temp16_y = fabsm(nop_move_remainy);
		temp16_x = fabsm(nop_move_remainx);
		tempx_step = -nop_move_remainx;
		tempy_step = -nop_move_remainy;
		
		//2.0 如果采用的是头部正反加固，恢复真正结束点的指针、坐标、总针数
		pat_point = target_pat_point;
		allx_step = target_allx_step;
		ally_step = target_ally_step;
		pat_buff_total_counter = target_total_counter;
		SewTestStitchCounter = target_SewTestStitchCounter;
	}
	//3.0 看要走距离是采用快走 还是 普通车缝协议
	if( temp16_x > temp16_y)
  	{
  		temp16_max = temp16_x;
  	}
  	else
  	{
  		temp16_max = temp16_y;
  	}

  	//2019-2-20 新增空送具体电机完成判断功能，避免空送死等 
  	_check_flag = 0;
	if( temp16_x > 127)
		_check_flag = 1;
	if( temp16_y > 127)
		_check_flag += 2;

#if (COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER47) || (COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER48)
	quick_time = Calculate_QuickMove_Time(temp16_x,temp16_y);
	//4.0 执行真正的X、Y轴动作
	if(temp16_y != 0 )
	{
		quickmove_y_process(quick_time,tempy_step);
		delay_ms(1);
	}
	if(temp16_x != 0 )
	{
		quickmove_x_process(quick_time,tempx_step);		
	}
	//5.0  延时等待框架的动作完成 
	if(temp16_max < 255)
 	    delay_ms(quick_time);
#else
   	if( temp16_max > 255)
	{
		//quick_time = Calculate_QuickMove_Time(temp16_max);
		quick_time = Calculate_QuickMove_Time(temp16_x,temp16_y);
	}
	else if(temp16_max <= 255 && temp16_max > 0)
	{
		quick_time = 62;
	}

	//4.0 执行真正的X、Y轴动作
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
	//5.0  延时等待框架的动作完成 
	if(temp16_max < 255)
 	    delay_ms(70);
#endif
	else
	{
		delay_ms(quick_time+68);
		#if STEPPER_WAITTING_PROTOCOL
        for(i=0;i<quick_time;i++)
		{
			delay_ms(1);
			if( check_motion_done(_check_flag) )
			   break;
		}
		#endif
	}
	nop_move_pause_flag = 0;
}




/**
  * @函数功能  自由状态函数
  * @参数      
  * @返回值 
  *
  * @控制使用的全局变量或参数
  *     @inpress_com  --面板或软件中压脚升降指令（非人机交互的）
  *     @inpress_flag --中压脚状态 0-降下的 1-抬起着
  *     @power_on_ready--k137 开机是否直接进入可缝制状态
  *     @
  *     @
  *
  * @受影响的全局变量 
  */
void free_status(void)
{	
	UINT8 temp8;
	INT8 ret;

#if 0
	read_all_io_output();
	delay_ms(25);
	read_all_io_input();	
	delay_ms(25);	
	
	 _speed_array[0] = 0xFF;
	 _speed_array[1] = check_origin(TEMPLATE_FEEDING_MOTOR_PORT);
	 _speed_array[2] = check_origin(TEMPLATE_EJECTING_MOTOR_PORT);
	 //_speed_array[3] = get_input_from_io(SPI_STM32_PORT, TEMPLATE_LEFT_POSITION_DETECTION_PORT-SC0714_JC_1);
	 //_speed_array[4] = get_input_from_io(SPI_STM32_PORT, TEMPLATE_RIGHT_POSITION_DETECTION_PORT-SC0714_JC_1);
	 _speed_array[5] = af_position_detection();
#endif	
	PW_SET_FUNC_ID(1);//2020-6-6 zla 记录当前所在函数位置
			 
    need_backward_sewing = 0; 
	pattern_change_flag = 0;

	//2020-8-21 zla 避免空送急停后切换到原点位置出现跑位
	//先确认是否需要空送到坐标(0,0)
	//1.0 如果说出现空送急停了，不论是平时空送还是回起缝点空送，都要从发生停车的位置返回到（0,0）
	if( (nop_move_pause_flag ==1)||(finish_nopmove_pause_flag==1) )
	{
		 allx_step = last_allx_step + read_step_x;//坐标值更新到与当前位置对应起来，否则Y向就出现撞出台板
		 ally_step = last_ally_step + read_step_y;
		 nop_move_pause_flag = 0;
		 finish_nopmove_pause_flag = 0;
	}


	//-------------------------------------------------------------------------------------
	//2019-7-18 zla 增加非准备界面下移框按钮的支持
	//-------------------------------------------------------------------------------------
	//2019-7-29 zla 移框坐标限制开关，用于保证FREE状态下的移框能够自由进行
	shift_disable_range_limit = 1;
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
  	shift_disable_range_limit = 0;
  	
  	//-------------------------------------------------------------------------------------
	//2019-7-18 zla 增加非准备界面下升降中压脚按钮操作
	//-------------------------------------------------------------------------------------
	if( inpress_act_flag == 1)
	{
		//气动压脚
		if( inpress_type == AIR_INPRESS)
		{
			if(inpress_flag == 1)
				inpress_down(0);
			else
			    inpress_up();
		}
		else//电机压脚
		{
			//2019-7-19 zla 新增中压脚第一次动作标志，开机时为1，如果没找原点就需要
			//抬起或者降下中压脚，就需要先找一下原点才行
			if( inpress_first_move_flag == 1 )//此标志将在go_origin_zx()中清0
			{
				go_origin_zx();
			}
			initial_mainmotor();
			
			if(inpress_high == 0)
			{
				if( inpress_flag == 1)
				{
				    inpress_down(inpress_high_hole);
				    delay_ms(100);
				}
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
		ret = 0;
		//2020-12-19 zla 增加机身ID录入功能
		if( machine_id_write_flag == 1 )
		{
			machine_id_write_flag = 0;
			ret = machine_id_write( svpara_disp_buf );
			//写入成功
			if( ret == 1 )
			{
				
			}
			//写入失败
			else
			{
				ret = 100;
			}
		}
		main_control_lock_setup = 0;
		write_par(0,main_control_lock_flag);
		write_par(2,remote_control_lock_flag);
		//机身ID写入失败，直接返回失败
		if( ret == 100 )
		{
			ret = 0;
			set_control_lock_return(1);

		}
		else//如果机身ID写入成功，再判断其他的
		{
			if( (read_par(0) == main_control_lock_flag)&&(read_par(2) == remote_control_lock_flag) )
				set_control_lock_return(0);
			else
				set_control_lock_return(1); 
		}
	}
	#endif	

	if( wirte_stepermotor_para_flag != 0)
	{
		write_stepmotor_config_para(debug_dsp_flag+1,svpara_disp_buf);
		wirte_stepermotor_para_flag = 0;		
	}
	
	#if (ENABLE_CONFIG_PARA == 1) || (TASC_PLATFORM_CONFIG_PARA_ENABLE==1)
	if( write_eeprom_para_flag >= 1)
    {
		  if( write_eeprom_para_flag == 1)
		  {
		    	write_para_group(100,svpara_disp_buf,205);
				delay_ms(100);
		    	restore_para_from_eeprom();
				//2018-11-5
				//降低中压脚到指定的基准高度
				if(cfg_inpress_hole_down == 1)
				{
					cfg_inpress_hole_down = 0;
					//先降低到基准位置
					SUM = 1;
					go_origin_zx();//先找原点
					delay_ms(100);
					inpress_to(inpress_high);
					inpress_flag = 0; 
					//delay_ms(5000);
					//SUM=0;
					delay_ms(1000);
					//SUM=1;
					//再往下多降低一些
					movestep_zx(-(INT16)(para.start_sew_inpress_hole),para.start_sew_inpress_hole+10);
					//SUM=1;
					//delay_ms(5000);
					//SUM=0;
				}
			//2020-08-10 zla 支持使用第1组参数指定距离和时间空送
			#if ENABLE_DEBUG_NOPMOVE == 1
				//2020-6-30 zla 用于指定时间和长度进行框架空送s
				if( para.dva_open_level == 1 && para.dsp1_step_crc < RESOLUTION*k56 )//X空送
				{
					if( fabsm(allx_step) == para.dsp1_step_crc )
					{
						go_commandpoint_test(0, ally_step, para.dsp2_step_crc);
					}
					else
					{
						go_commandpoint_test((INT32)para.dsp1_step_crc, ally_step, para.dsp2_step_crc);
					}
				}
				else if( para.dva_open_level == 2 && para.dsp1_step_crc < RESOLUTION*k59 )//Y空送
				{
					if( fabsm(ally_step) == para.dsp1_step_crc )
					{
						go_commandpoint_test(allx_step, 0, para.dsp2_step_crc);
					}
					else
					{
						go_commandpoint_test(allx_step, -(INT32)para.dsp1_step_crc, para.dsp2_step_crc);
					}
				}
			#endif
				delay_ms(200);
				//SUM = 0;

			//2019-1-5 为三丝杠模板机（机型：47）新增圆刀电机剪线功能
			#if ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
				#if 0
				//2019-1-7
				//控制剪线电机动作
				if(trim_motor_action_flag  == 1)
				{
					trim_motor_action_flag = 0;
					go_origin_trim();//先回原点
					delay_ms(400);
					
					switch(para.trim_motor_debug_action)
					{
						case 0://回原点
							//啥也不干
							break;
						case 1://移动到分线位置
							movestep_trim(-(int)para.trim_motor_branch_position, para.trim_motor_branch_time);
							delay_ms(para.trim_motor_branch_time);
							break;
						case 2://移动到断线位置
							movestep_trim(-(int)para.trim_motor_range, para.trim_motor_break_time+para.trim_motor_branch_time);
							delay_ms(para.trim_motor_break_time);
							break;
						case 3://先移动到分线位置，然后再移动到断线位置
							movestep_trim(-(int)para.trim_motor_branch_position, para.trim_motor_branch_time);
							delay_ms(para.trim_motor_branch_time);
							movestep_trim(-(int)(para.trim_motor_range-para.trim_motor_branch_position), para.trim_motor_break_time);
							delay_ms(para.trim_motor_break_time);
							break;
						default:
							break;
					}
					delay_ms(400);
				}
				#endif
			#endif
		  }
		  else if( write_eeprom_para_flag == 2)
		  {
				write_para_group(400,svpara_disp_buf,205);
				delay_ms(300);
				//app_GetProgramFromEeprom();
		  }
		  else if( write_eeprom_para_flag == 3)
		  {
			  write_para_group(700,svpara_disp_buf,205);
			  delay_ms(300);
			  get_para_from_eeprom();
		  }
		  else if( write_eeprom_para_flag == 4)//第7组
		  {
		  	  write_para_group(1000,svpara_disp_buf,205);
			  delay_ms(300);
			  get_para_from_eeprom();
		  }
		  else if( write_eeprom_para_flag == 5)//第8组
		  {
		  	  write_para_group(1300,svpara_disp_buf,205);
			  delay_ms(300);
			  get_para_from_eeprom();
			  
			//只在使能了自动送料功能和多功能IO才有效
			#if MULTIPULE_IO_ENABLE && AUTOMATIC_FEEDING_ENABLE
			//2019-1-11将参数从para8中提取到自送送料参数结构体
			af_get_configuration(&af_info);
			if(para8.af_enable_switch == 55)
			{
				if(af_action_flag == 1)
				{
					af_action_flag = 0;

					/*
					//听个响先
					SUM = 1;
					delay_ms(3000);
					SUM = 0;
					delay_ms(3000);
					*/
					//送料电机回原点
					if(para8.debug_action_flag == 0 || para8.debug_action_flag == 1)
					{
						go_origin(TEMPLATE_FEEDING_MOTOR_PORT);
					}
					//出料电机回原点
					if(para8.debug_action_flag == 2 || para8.debug_action_flag == 3)
					{
						go_origin(TEMPLATE_EJECTING_MOTOR_PORT);
					}
					//全部回原点
					if(para8.debug_action_flag == 4 || para8.debug_action_flag == 5)
					{
						//af_init();
						go_origin_allmotor();//里面自然会调用af_init()让送出料电机回原点
					}
					//step1:送料动作
					else if(para8.debug_action_flag == 6)
					{
						af_procedure_contorl(FEEDING_ACTION);
					}
					//step2:主轴到取料位置取模板
					else if(para8.debug_action_flag == 7)
					{
						af_procedure_contorl(FRAME_GET_TEMPLATE);
					}
					//step3:送料完成返回动作
					else if(para8.debug_action_flag == 8)
					{
						af_procedure_contorl(FEEDING_BACK_ACTION);
					}
					//step4:XY框架到停车位置(假设这已经开始缝制了)
					else if(para8.debug_action_flag == 9)
					{
						af_procedure_contorl(FRAME_TO_PARKING_POSITION);
					}
					//step5:缝制后XY框架到取料位置(假设这已经结束缝制了)
					else if(para8.debug_action_flag == 10)
					{
						af_procedure_contorl(FRAME_TO_HANDOVER_POSITION);
					}
					//step6:出料动作
					else if(para8.debug_action_flag == 11)
					{
						af_procedure_contorl(EJECTING_ACTION);
					}
					//step7:全套测试
					else if(para8.debug_action_flag == 12)
					{
						af_procedure_test();
					}
					//送料动作：1000步，即500mm
					else if(para8.debug_action_flag == 20)
					{
						motor_move(TEMPLATE_FEEDING_MOTOR_PORT, 5000, 3000);
					}
					//送料动作：-1000步，即-500mm
					else if(para8.debug_action_flag == 21)
					{
						motor_move(TEMPLATE_FEEDING_MOTOR_PORT, -5000, 3000);
					}
					//出料动作：1000步，即500mm
					else if(para8.debug_action_flag == 30)
					{
						motor_move(TEMPLATE_EJECTING_MOTOR_PORT, 5000, 3000);
					}
					//出料动作：-1000步，即-500mm
					else if(para8.debug_action_flag == 31)
					{
						motor_move(TEMPLATE_EJECTING_MOTOR_PORT, -5000, 3000);
					}
					//XY框架到停车位置
					else if(para8.debug_action_flag == 40)
					{
						go_commandpoint(af_info.x_parking_position, af_info.y_parking_position);
					}
					//XY框架到接料位置
					else if(para8.debug_action_flag == 41)
					{
						go_commandpoint(af_info.x_handover_position, af_info.y_handover_position);
					}
					//送料气缸顶起
					else if(para8.debug_action_flag == 100)
					{
						//送料气缸顶起
						af_jacking_control(TEMPLATE_FEEDING_JACKING_PORT, 1);
					}
					//送料气缸降下
					else if(para8.debug_action_flag == 101)
					{
						//送料气缸降下
						af_jacking_control(TEMPLATE_FEEDING_JACKING_PORT, 0);
					}
					//出料气缸顶起
					else if(para8.debug_action_flag == 102)
					{
						//出料气缸顶起
						af_jacking_control(TEMPLATE_EJECTING_JACKING_PORT, 1);
					}
					//出料气缸降下
					else if(para8.debug_action_flag == 103)
					{
						//出料气缸降下
						af_jacking_control(TEMPLATE_EJECTING_JACKING_PORT, 0);
					}
					//查询送料原点状态
					else if(para8.debug_action_flag == 104)
					{
						if(check_origin(TEMPLATE_FEEDING_MOTOR_PORT)==1)
						{
							//响4声
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
						}
					}
					//查询出料原点状态
					else if(para8.debug_action_flag == 105)
					{
						if(check_origin(TEMPLATE_EJECTING_MOTOR_PORT)==1)
						{
							//响4声
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
						}
					}
					//查询左边微动开关状态
					else if(para8.debug_action_flag == 106)
					{
						if( (af_position_detection()==0) ||	 (af_position_detection()==-2) )
						{
							//响4声
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
						}
					}
					//查询右边微动开关状态
					else if(para8.debug_action_flag == 107)
					{
						if( (af_position_detection()==0) ||	 (af_position_detection()==-1) )
						{
							//响4声
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
							SUM=1;
							delay_ms(100);
							SUM = 0;
							delay_ms(100);
						}
					}
				}

			}
			
			//2019-4-10 增加RFID存储模板长宽信息等的调试，暂时放到这里，后续放到面板中实现
			if(para8.rfid_debug_action == 1 || para8.rfid_debug_action == 0)
			{
				//如果是读RFID，包括编号和长宽
				if(para8.rfid_debug_action == 0)
				{
					rc522_write_falg = 0;//RFID_SCAN()中将读卡，宽高读到af_info中
					RFID_SCAN();//如果读出来的去是0，就表示失败了
					para8.rfid_serial_number = serail_number;//读出的编号
					para8.rfid_template_width = af_info.width;//读出的宽度
					para8.rfid_template_height = af_info.height;//读出的高度
					if(serail_number != 0)//读取成功
					{
						//这里好像也不用做什么？
						para8.rfid_debug_action = 255;//恢复成无意义的数
					}
					else
					{
						//para8.rfid_debug_action = 254;//操作失败
					}
				}
				//写RFID，包括编号和长宽
				else if(para8.rfid_debug_action == 1)
				{
					rc522_write_falg = 1;//RFID_SCAN()中将写卡，宽高从af_info中获取
					Rfid_Nom = para8.rfid_serial_number;//要写入的编号
					af_info.width = para8.rfid_template_width;//要写入的宽度
					af_info.height = para8.rfid_template_height;//要写入的高度
					RFID_SCAN();
					rc522_write_falg = 0;//恢复成读卡控制
					if(rc522_write_ret_falg == 0)//写卡成功
					{
						//这里好像也不用做什么？
						para8.rfid_debug_action = 255;//恢复成无意义的数
					}
					else
					{
						//para8.rfid_debug_action = 254;//操作失败
					}
				}
				
				
				//接下来将第8组再次存入EEPROM中
				cpy_para8_buff();//从para8转成
				write_para_group(1300,svpara_disp_buf,205);
				
			}
			#endif
			
		  }
		  else if( write_eeprom_para_flag == 6)//第9组
		  {
		  	  write_para_group(1600,svpara_disp_buf,205);
			  delay_ms(300);
			  get_para_from_eeprom();
		  }
		  else if( write_eeprom_para_flag == 7)//第10组
		  {
		  	  write_para_group(1900,svpara_disp_buf,205);
			  delay_ms(300);
			  get_para_from_eeprom();
		  }
		  else//第11组
		  {
		  	  write_para_group(2200,svpara_disp_buf,205);
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

    #if 0
	if((svpara_trans_flag == 1)&&(SEND_SERVO_PARA == 1))
	{
		stepmotor_para();
		svpara_trans_flag = 0;
		set_func_code_info(FREE,1,0,0);
	}
	#endif
	
	#if ENABLE_RFID_FUNCTION
	
	if(auto_function_flag == TEMPLATE_IDENTIFY_SWITCH_OPEN)
	{
		if((formwork_identify_device == TEMPLATE_IDENTIFY_DEVICE_RFID)&&(rc522_control_falg==1))//RFID
		{	
			SUM =1;
			temp8 = RFID_SCAN();
	        rfid_wr_ret(temp8);
			rc522_control_falg = 0;
		}
	}
	#endif
#if	MACHINE_900_BOBBIN_DEBUG_MODE

#else
	//1.0 响应中压脚动作指令
  	/*switch(inpress_com)
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
    //2.0 响应外压脚动作指令
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
  	}*/

  	//2019-4-17 新增
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
  	        	 #if 0
				 if( k115 == 0) 
				 	 already_auto_find_start_point = 0;
				 #endif
  	        }
  	        break;		
	
  		case 2:  break;               
		       
  		default: break;                   	         	
  	}  
  	
	//3.0 扫描脚踏板，执行压框升降动作
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
	
	//2019-9-18 zla
	//虚拟按键功能,支持面板通过专用指令实现和按下启动、压框、急停按钮一个作用
	#if VIRTUAL_BOTTON_FUNCTION_ENABLE	
	//面板控制压框动作
	if(additional_command == ADDITIONAL_COMMAND_CLAMP_UP)//外压框抬起
	{
		foot_up();
		additional_command = ADDITIONAL_COMMAND_NONE;
	}
	else if(additional_command == ADDITIONAL_COMMAND_CLAMP_DOWN)//外压框降下
	{
		foot_down();
		additional_command = ADDITIONAL_COMMAND_NONE;
	}
	#endif
	
	if(pedal_state == 1)
	{
		delay_ms(10);
		if(pedal_state == 1)
		{
			pedal_last_state = 1;
		}
	}
	//4.0 急停开关状态扫描
  	if( (PAUSE == pause_active_level)&&(power_on_ready ==0) )
	{
		delay_ms(10);                           
		if(PAUSE == pause_active_level)
		{				
			status_now = sys.status;
//			sys.status = ERROR;
//			StatusChangeLatch = ERROR;
//      		sys.error = ERROR_19;  //急停开关不在正常位置
		SET_SYS_ERROR(ERROR_19);//急停开关不在正常位置
	  	}
	 
  	}
  	//2019-9-18 zla
	//虚拟按键功能,支持面板通过专用指令实现和按下启动、压框、急停按钮一个作用
	#if VIRTUAL_BOTTON_FUNCTION_ENABLE
	//面板控制急停
	if(additional_command == ADDITIONAL_COMMAND_START_STOP)
	{
		status_now = sys.status;
//		sys.status = ERROR;
//		StatusChangeLatch = ERROR;
//  		sys.error = ERROR_19;
		SET_SYS_ERROR(ERROR_19);//急停开关不在正常位置
  		additional_command = ADDITIONAL_COMMAND_NONE;
	}
  	#endif
  	
#endif 
    predit_shift = 0;
 	if(StatusChangeLatch != FREE) 
	{
		sys.status = StatusChangeLatch;
		set_func_code_info(FREE,2,0,0);
	}
}

/**
  * @函数功能  准备状态处理函数
  * @参数      
  * @返回值 
  *
  * @控制使用的全局变量或参数
  *     @
  *     @
  *
  * @受影响的全局变量 
  */
void ready_status(void)
{
	UINT8 temp8;
	UINT16 temp16;	
	INT32 allx_step_tmp,ally_step_tmp;
	INT8 ret;

	PW_SET_FUNC_ID(2);//2020-6-6 zla 记录当前所在函数位置
	PW_SET_CODE_ID(2000);//2020-6-6 zla 记录当前所在代码段位置
	
	#if INSTALLMENT
	if( main_control_lock_setup == 1)
	{
		main_control_lock_setup = 0;
		write_par(0,main_control_lock_flag);  		
		write_par(2,remote_control_lock_flag); 
		if( (read_par(0) == main_control_lock_flag)&&(read_par(2) == remote_control_lock_flag) )
			set_control_lock_return(0);
		else
			set_control_lock_return(1); 
	}
	#endif
	
	//花样识别借用的输出协议，用于通知主控可以识读新花样
	//下发花样后，面板发送OUTPUT指令（借用）,这里会清除pattern_change_flag
	check_output_pattern_done();
	PW_SET_CODE_ID(2001);//2020-6-6 zla 记录当前所在代码段位置
//	#if CHANGE_DOUBLE_FRAMEWORK
#if 0
	if( current_running_flag == 0)//扫描到新条码，并且已经下载了
	{
			if( left_quest_running == 1)//先左后右
			{			
				current_running_flag = 1;
				delay_ms(100);
				take_frame_from_one_side(1);//从左边拿模板过来		
				SUM = 0;
				autosewing_allset_flag = 1;		
			}
			else if(right_quest_running == 1)
			{				
				current_running_flag = 2;
				delay_ms(100);
				take_frame_from_one_side(2);//从右边拿模板过来	
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
	

	#if (COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER58)
		if( oil_empty_alarm_enable == 1)//k199，油量报警开关
		{
			if(OIL_EMPTY_DETECT == 1)
			{
				status_now = READY;
//				sys.status = ERROR;
//				StatusChangeLatch = ERROR;
//		    	sys.error = ERROR_80;//油量警报
				SET_SYS_ERROR(ERROR_80);//油量警报
			}
		}
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
	if( motorconfig_flag == 0 )//主轴找零位
  	{
  		PW_SET_CODE_ID(2002);//2020-6-6 zla 记录当前所在代码段位置
  		initial_mainmotor();		
  		PW_SET_CODE_ID(2003);//2020-6-6 zla 记录当前所在代码段位置
	}	
  	if(sys.status == ERROR)
  	{
	  	predit_shift = 0;
  	  	return;
  	}
	
	//2.0 执行是否有回起缝点的命令，模板机是停在原点的，不会停在起缝点
	//实际上，当RFID切换时，新面板完成花样下发后，此变量将被置1
	if ( ready_go_setout_com == 1)
	{
		ready_go_setout_com = 0;
		predit_shift = 0;
	}
	//3.0 响应界面上的剪线按钮功能，原地动轴剪线
	if(cut_test_flag == 1)
	{
			#if DISABLE_THREAD_CLAMP_FUNCTION
			#else
			//if( (u35==0)&&(clamp_flag== 1) )  //打开抓线并且伸出
			if( (u35==THREAD_SCRATCH_SWITCH_OPEN)&&(clamp_flag== 1) )  //打开抓线并且伸出
			{
				PW_SET_CODE_ID(2004);//2020-6-6 zla 记录当前所在代码段位置
				go_origin_ct();	
				PW_SET_CODE_ID(2005);//2020-6-6 zla 记录当前所在代码段位置
			}
			#endif
			cut_test_flag = 0;
			/*
			剪线动作之前，要确认一下是否进行过空送急停，防止停在模板中间，剪线机针下扎会断针。
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

				//2019-1-21 首先检查针杆位置，框架动作之前把针先提起来，防止撞针
				temp8 = detect_position();	
				if(temp8 == OUT)    								
				{
					PW_SET_CODE_ID(2006);//2020-6-6 zla 记录当前所在代码段位置
					find_dead_center();
					PW_SET_CODE_ID(2007);//2020-6-6 zla 记录当前所在代码段位置
				}
				PW_SET_CODE_ID(2008);//2020-6-6 zla 记录当前所在代码段位置
			 	trim_action();
			 	PW_SET_CODE_ID(2009);//2020-6-6 zla 记录当前所在代码段位置
			 	//2019-6-11 衔接效率提升开关(剪线+空送+起缝)
				#if BRIDGING_EFFICIENCY_IMPROVEMENT && ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
				if(trim_motor_need_find_origin_flag == 1)
				{
					PW_SET_CODE_ID(2010);//2020-6-6 zla 记录当前所在代码段位置
					while( cutter_delay_flag == 1)
					{
						delay_ms(1);
					}
					PW_SET_CODE_ID(2011);//2020-6-6 zla 记录当前所在代码段位置
					go_origin_trim();//等待标缝指令到原点后再找原点
					PW_SET_CODE_ID(2012);//2020-6-6 zla 记录当前所在代码段位置
				}
				#endif
				delay_ms(100);
				
				inpress_up();
				delay_ms(100);
			}
			predit_shift = 0;
			/*
			如果开抓线功能，剪线完成后抓线机构还要先伸出去
			*/
			#if DISABLE_THREAD_CLAMP_FUNCTION
			#else
			//if( (clamp_flag == 0)&&(u35==0))
			if( (clamp_flag == 0)&&(u35==THREAD_SCRATCH_SWITCH_OPEN))
    		{
    			PW_SET_CODE_ID(2013);//2020-6-6 zla 记录当前所在代码段位置
    			go_origin_ct();
				delay_ms(20);
    			clamp_out();
    			clamp_stepflag = 1;
          		movect_angle = 800;
          		PW_SET_CODE_ID(2014);//2020-6-6 zla 记录当前所在代码段位置
    		}
    		#endif
	}
	
	PW_SET_CODE_ID(2015);//2020-6-6 zla 记录当前所在代码段位置
	//2019-9-18 zla
	//虚拟按键功能,支持面板通过专用指令实现和按下启动、压框、急停按钮一个作用
	#if VIRTUAL_BOTTON_FUNCTION_ENABLE	
	//-------------------------------------------------------------------------------------
	// 2019-9-18 zla 添加中压脚动作控制 inpress up or down command
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
			  			PW_SET_CODE_ID(2016);//2020-6-6 zla 记录当前所在代码段位置
						find_dead_center();
						PW_SET_CODE_ID(2017);//2020-6-6 zla 记录当前所在代码段位置
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
	#endif

    //4.0 扫描急停开关的状态，防止按着急停进行运行状态
	//if (PAUSE == pause_active_level && pause_last_flag == 0)
	if (PAUSE == pause_active_level)
	{
		delay_ms(10);
		//if(PAUSE == pause_active_level && pause_last_flag == 0)
		if (PAUSE == pause_active_level)
		{
			PW_SET_CODE_ID(2018);//2020-6-6 zla 记录当前所在代码段位置
			status_now = sys.status;
//			sys.status = ERROR;
//			StatusChangeLatch = ERROR;
//      		sys.error = ERROR_19;
			SET_SYS_ERROR(ERROR_19);//急停开关不在正常位置
      		//sys.error = ERROR_02;

			//2018-11-19 新增旋转切刀宏定义，用于减少代码量
			#if ENABLE_ROTATED_CUTTER==1
			if(ROTATED_CUTTER == 1)
			{
				if ( (milling_cutter_action_flag ==1)&&(rotated_cutter_enable ==1) )
					 rotated_cutter_single_stop();
			}
			#endif
			
			//2019-10-23 zla 避免进入连续试缝还动作一次，这样因为已经进入ERROR状态，会导致跑位情况的出现
			predit_shift = 0;
			single_flag = 0;
	  	}
  	}
  	#if 0
  	//更新上次PAUSE状态
	if(PAUSE == pause_active_level)
	{
		pause_last_flag = 1;
	}
	else
	{
		pause_last_flag = 0;
	}
	//急停按钮按下，但是没报错的时候，有了试缝操作了，这个时候报错吧
	if(single_flag !=0 && PAUSE == pause_active_level)
	{
		status_now = sys.status;
		sys.status = ERROR;
		StatusChangeLatch = ERROR;
  		//sys.error = ERROR_19;
  		sys.error = ERROR_02;

  		single_flag = 0;
  		predit_shift = 0; 
	}
	#endif
	
	//2019-9-18 zla
	//虚拟按键功能,支持面板通过专用指令实现和按下启动、压框、急停按钮一个作用
	#if VIRTUAL_BOTTON_FUNCTION_ENABLE	
	//面板控制急停
	if(additional_command == ADDITIONAL_COMMAND_START_STOP)
	{
		PW_SET_CODE_ID(2019);//2020-6-6 zla 记录当前所在代码段位置
		status_now = sys.status;
//		sys.status = ERROR;
//		StatusChangeLatch = ERROR;
//  		sys.error = ERROR_19;
		SET_SYS_ERROR(ERROR_19);//急停开关不在正常位置
  		additional_command = ADDITIONAL_COMMAND_NONE;
  		//2019-10-23 zla 避免进入连续试缝还动作一次，这样因为已经进入ERROR状态，会导致跑位情况的出现
		predit_shift = 0; 
		single_flag = 0;
	}
  	#endif
	//2020-12-10 zla 在前台程序中报，气压报警，不在中断里报
	if( u108 == AIR_PRESSURE_ALARM_SWITCH_OPEN )
	{
		if( (pause_inpress_flag == 1) && sys.error == OK)
		{
			SET_SYS_ERROR(ERROR_48);//气压异常
			sfsw_power_on_delay = 2000;//2020-12-10 zla 2000ms内不能恢复
			//2019-10-23 zla 避免进入连续试缝还动作一次，这样因为已经进入ERROR状态，会导致跑位情况的出现
			predit_shift = 0; 
			single_flag = 0;
		}
	}
	
	//5.0 试缝操作的响应
	if( single_flag !=0 )
	{
		return_from_setout = 1;				//只要试缝过，就表示这个花样已经开始处理了，在这个过程中不再扫描模板
		need_action_once = 0;
		if( second_start_switch == 1)		//试缝以后，二次启动的功能仍然有效
		    second_start_counter = 0;
		already_in_origin = 0;				//试缝动作后，就不在原点了
		if(AUTO_SEARCH_START_POINT == 1)	//没找过原点，则执行特殊的找原点处理
		{
			if (already_auto_find_start_point == 0)
			{
				//find_start_point();
				already_auto_find_start_point = 1;
			}
		}
		set_func_code_info(READY,2,single_flag,0);

		//2020-12-7 zla 如果之前在暂停码位置，那么此时必须先走到实际花样位置才能响应段跳转
		if( cs.enable >= 1 )
		{
			cs_goto_real_positon();
		}
	}
	
  	switch(single_flag)
  	{
   		case 1:	 //5.1  车缝单步向前的处理
			 
			SewingTestEndFlag = 1;  
			//5.1.1 移动前提升针杆
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
      			PW_SET_CODE_ID(2020);//2020-6-6 zla 记录当前所在代码段位置
				find_dead_center();
				PW_SET_CODE_ID(2021);//2020-6-6 zla 记录当前所在代码段位置
      		}
			
			allx_step = allx_step + xstep_cou;//5.1.2 判定单步操作后是否出现超出缝制范围的情况
  			ally_step = ally_step + ystep_cou;
			
			if( check_sewing_range())//5.1.2.1 超范围的处理
			{
				allx_step = allx_step - xstep_cou;
  				ally_step = ally_step - ystep_cou;
				do_pat_point_sub_one();
				single_flag = 0;
				move_flag = 0;
				nopmove_flag = 0;
//				sys.error = ERROR_15;
				SET_SYS_ERROR_1(ERROR_15);//超出缝制范围
				StatusChangeLatch = ERROR;
				set_func_code_info(READY,3,0,0);
			}
			else
			{	
				//5.1.2.1 没超范围情况,先恢复坐标
				allx_step = allx_step - xstep_cou;
  				ally_step = ally_step - ystep_cou;
				//
				if(nop_move_pause_flag ==1)
				{
		    		//5.1.2.2 如果之前发生了空送过程中按急停的情况，这时再向前就要先把空送走完
					process_nop_move_pause(1);
					set_func_code_info(READY,4,0,0);
					single_flag = 0;
				}
				else
				{			
					if(nopmove_flag == 1)
					{	
						//if(u103 != 0 && inpress_flag == 0)   
						if(u103 != SEWING_TEST_FOOTER_STATUS_UP && inpress_flag == 0)
						{
							inpress_up();        	
						}
					}
			
					if(move_flag == 1)
					{
						//if(u103 == 2)
						if(u103 == SEWING_TEST_FOOTER_STATUS_DOWN)
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
						//2018-11-19 新增旋转切刀宏定义，用于减少代码量
						#if ENABLE_ROTATED_CUTTER==1
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
						#endif
						
					}
					
					#if ENABLE_LOOPER_CUTTER
						if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
						{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 0) )//偏移不为0
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
					
					//SewTestStitchCounter++;
					if( (FootRotateFlag == 1) &&(marking_flag == 1) )
					{
						process_marking_pen(0);
						PW_SET_FUNC_ID(2);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
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
			
  		case 2://单步后退试缝
		       
			SewingTestEndFlag = 1;  
			//SewTestStitchCounter--;		
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
				//if(u103 == 2) 
				if(u103 == SEWING_TEST_FOOTER_STATUS_DOWN)
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
				//2018-11-19 新增旋转切刀宏定义，用于减少代码量
				#if ENABLE_ROTATED_CUTTER==1
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
				#endif
				#if ENABLE_LOOPER_CUTTER
						if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
						{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 1) )//偏移不为0
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
						PW_SET_FUNC_ID(2);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
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
      			PW_SET_CODE_ID(2022);//2020-6-6 zla 记录当前所在代码段位置
				find_dead_center();
				PW_SET_CODE_ID(2023);//2020-6-6 zla 记录当前所在代码段位置
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
				if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 1) )//偏移不为0
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
		//2018-11-19 新增旋转切刀宏定义，用于减少代码量
		#if ENABLE_ROTATED_CUTTER==1
		if(ROTATED_CUTTER == 1)
		{
		  if( rotated_cutter_enable ==1 )
		   {
		   	  //DRILL_MOTOR_UPDOWN = 0;//确认刀下去了
			  rotated_cutter_down_positon();
			   if ( milling_first_move ==1)
			   {		
					if( (x_bios_offset!=0)||(y_bios_offset!=0) )//偏移不为0
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
		#endif
		#if ENABLE_LASER_CUTTER
					if( milling_cutter_action_flag != 0)
					{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 1) )//偏移不为0
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
			//if(u39 == 1)
			PW_SET_CODE_ID(2024);//2020-6-6 zla 记录当前所在代码段位置
			if(u39 == AFTER_SEWING_FINISH_GO_ORIGIN_OPEN)
			    go_origin_allmotor();
			else
				move_startpoint();
			PW_SET_CODE_ID(2025);//2020-6-6 zla 记录当前所在代码段位置
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
  		case 4:	//单步前进遇到空送码
			SewingTestEndFlag = 1;  
			//2018-11-19 新增旋转切刀宏定义，用于减少代码量
			#if ENABLE_ROTATED_CUTTER==1
			if(ROTATED_CUTTER == 1)
			{
				if ( (milling_cutter_action_flag ==1)&&(rotated_cutter_enable ==1) )
				     rotated_cutter_single_stop();
			}
			#endif

			//只在使能了自动送料功能和多功能IO才有效
			#if MULTIPULE_IO_ENABLE && AUTOMATIC_FEEDING_ENABLE
			//2019-1-15 
			if(af_info.frame_position_flag == 1)//如果XY框架在停车位置
			{
				//禁止在偏移时出现急停，此时不响应急停
				shift_no_pause=1;
				go_commandpoint(0, 0);//先回0位
				shift_no_pause=0;//恢复对急停的响应
				af_info.frame_position_flag = 0;
			}
			#endif
					
			if(nop_move_pause_flag ==1)
		    	process_nop_move_pause(1);
			else
				go_sewingtest_beginpoint();  
			PW_SET_CODE_ID(2026);//2020-6-6 zla 记录当前所在代码段位置
			
			//2020-4-11 zla 当下一段起缝是激光开始码时，会导致这个码被一起分析了
			//所以这里干脆把偏移值走出来
			#if ENABLE_LASER_CUTTER
			process_laser_offset_move(0);
			#endif
			
			SewingTestEndFlag = 0;   
			predit_shift = 0;
			single_flag = 0;
			break;			
  		case 5:	 //单步后退遇到空送码
	
			SewingTestEndFlag = 1;  
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
      			PW_SET_CODE_ID(2027);//2020-6-6 zla 记录当前所在代码段位置
				find_dead_center();
				PW_SET_CODE_ID(2028);//2020-6-6 zla 记录当前所在代码段位置
      		}
			inpress_up(); 
			//2018-11-19 新增旋转切刀宏定义，用于减少代码量
			#if ENABLE_ROTATED_CUTTER==1
			if(ROTATED_CUTTER == 1)
			{
				if ( (milling_cutter_action_flag ==1)&&(rotated_cutter_enable ==1) )
				     rotated_cutter_single_stop();
			}
			#endif
			if(nop_move_pause_flag ==1)
		    	process_nop_move_pause(2);
			else
			{
			 	back_endpoint();
			}
			if( (FootRotateFlag == 1) &&(marking_flag == 1) )
			{
				process_marking_pen(1);
				PW_SET_FUNC_ID(2);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
				marking_flag = 0;
				FootRotateFlag = 0;
				delay_ms(50);
			}
			SewingTestEndFlag = 0;  
			predit_shift = 0;
			single_flag = 0;  
			PW_SET_CODE_ID(2029);//2020-6-6 zla 记录当前所在代码段位置
			break;
  		case 6:	 

			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
      			PW_SET_CODE_ID(2030);//2020-6-6 zla 记录当前所在代码段位置
				find_dead_center();
				PW_SET_CODE_ID(2031);//2020-6-6 zla 记录当前所在代码段位置
      		}
			course_next();  
			if( (FootRotateFlag == 1) &&(marking_flag == 1) )
			{
				 process_marking_pen(0);
				 PW_SET_FUNC_ID(2);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
				 marking_flag = 0;
				 FootRotateFlag = 0;
				 delay_ms(50);
			}
			#if ENABLE_LOOPER_CUTTER
						if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
						{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 0) )//偏移不为0
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
			delay_ms(1);
			set_func_code_info(READY,8,0,0);                  
			if(PAUSE == pause_active_level)
			{
				delay_ms(10);
				if(PAUSE == pause_active_level)
				{		
					PW_SET_CODE_ID(2032);//2020-6-6 zla 记录当前所在代码段位置
					predit_shift = 0; 
					single_flag = 0;
			  	}
	 
		  	}
			break;		
  		case 7:	 	

			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
      			PW_SET_CODE_ID(2033);//2020-6-6 zla 记录当前所在代码段位置
				find_dead_center();
				PW_SET_CODE_ID(2034);//2020-6-6 zla 记录当前所在代码段位置
      		}
			course_back();
			if( (FootRotateFlag == 1) &&(marking_flag == 1) )
			{
				process_marking_pen(1);
				PW_SET_FUNC_ID(2);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
				marking_flag = 0;
				FootRotateFlag = 0;
				delay_ms(50);
			}
			delay_ms(1); 
			#if ENABLE_LOOPER_CUTTER
						if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
						{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 1) )//偏移不为0
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
					PW_SET_CODE_ID(2035);//2020-6-6 zla 记录当前所在代码段位置
					predit_shift = 0; 
					single_flag = 0;
			  	}
	 
		  	}
			break;			
  		case 8:	 
			PW_SET_CODE_ID(2036);//2020-6-6 zla 记录当前所在代码段位置
			course_stop();       
			PW_SET_CODE_ID(2037);//2020-6-6 zla 记录当前所在代码段位置
			predit_shift = 0;   
			set_func_code_info(READY,10,0,0);	
			break;	
		default:                                     
			break;	
  	}
  	
#if DISABLE_THREAD_CLAMP_FUNCTION
#else
	if( (return_from_setout == 1)&&(clamp_com == 1)&&(clamp_stepflag == 2)&&(stitch_counter < u33) )
	{
		clamp_backstep2(); 
	}
#endif
	//2020-12-17 zla 新增宏定义作为换梭使能，用于减少代码量
	#if ENABLE_BOBBIN_CASE == 1
	if(ENABLE_BOBBIN_CASE_FUN == 1)
	{
		if( bobbin_case_once_done_flag == 1)
		{
			if( bobbin_case_enable == 1)
			{
				PW_SET_CODE_ID(2038);//2020-6-6 zla 记录当前所在代码段位置
				bobbin_case_workflow1();
				PW_SET_CODE_ID(2039);//2020-6-6 zla 记录当前所在代码段位置
			}
			bobbin_case_once_done_flag = 0;
			//换梭后，可能针杆位置出现了变化，这个时候回死点
			temp8 = detect_position();	
	    	if(temp8 == OUT)    
	      	{
	      		PW_SET_CODE_ID(2040);//2020-6-6 zla 记录当前所在代码段位置
			   find_dead_center();
			   PW_SET_CODE_ID(2041);//2020-6-6 zla 记录当前所在代码段位置
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
		PW_SET_CODE_ID(2042);//2020-6-6 zla 记录当前所在代码段位置
	}
	#endif

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
		  			PW_SET_CODE_ID(2043);//2020-6-6 zla 记录当前所在代码段位置
					find_dead_center();
					PW_SET_CODE_ID(2044);//2020-6-6 zla 记录当前所在代码段位置
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
						//find_start_point();//error
						already_auto_find_start_point = 1;
					}
				}
				delay_ms(50);	
				//2020-3-18 zla，按照马工在耐拓的方法修改，先把其置0，这样可以加快段跳转速度
				//从逻辑上看，其实早置0和晚置0，都不会影响主控的执行流程，但是却可以省下对坐标
				//跳转指令的查询时间，实现类似于两条指令合一的操作
				predit_shift = 0;
				
				//2020-12-7 zla 如果之前在暂停码位置，那么此时必须先走到实际花样位置才能响应段跳转
				if( cs.enable >= 1 )
				{
					cs_goto_real_positon();
				}
				
				//SUM=1;
				go_commandpoint(comx_step,comy_step);	
				//SUM=0;
				need_action_once = 0;//跳转以后不能直接加固，要判定位置
				//delay_ms(500);
				set_func_code_info(READY,11,0x77,0);
				PW_SET_CODE_ID(2045);//2020-6-6 zla 记录当前所在代码段位置
			}
			predit_shift = 0;
			coor_com = 0;			
			synchronization_flag = 1;
			//2020-11-27 zla 新增面板下发段跳转指令0x77后启动限制标志的最大有效时间限时，避免无法启动
			delay_start_after_coor_time = 2000;
	  	}
		
		if( PointShiftFlag == 1)//0x64
	    {	
		    if( (nop_move_pause_flag ==0)&&(foot_flag == 0) )
		    //2020-12-04 zla 如果是RFID切换支持跳转坐标
			//if( (nop_move_pause_flag ==0)&&( (foot_flag == 0) || (cs.enable >= 1)) )
			{
				pat_point = (PATTERN_DATA *)(pat_buf);
			    pat_point = pat_point + (PointShiftNumber%TOTAL_STITCH_COUNTER);
			    pat_buff_total_counter = PointShiftNumber;
			}
		    PointShiftFlag = 0;
			predit_shift = 0 ;
			synchronization_flag = 0;
			PW_SET_CODE_ID(2046);//2020-6-6 zla 记录当前所在代码段位置
			set_func_code_info(READY,12,PointShiftNumber>>8,PointShiftNumber);
            monitor_allx_step = allx_step;
			monitor_ally_step = ally_step;
			monitor_pat_point = pat_point - 1;
			monitor_refresh_flag = 1;

			//2020-4-2 zla 跳转的花样位置是激光切割段，那么走出偏移来
			if(laser_already_begin_flag == 1)
			{
				//之前未发生急停
				if( (nop_move_pause_flag ==0 )&&(finish_nopmove_pause_flag ==0) )
				{
					if( ((laser_offset_x!=0)||(laser_offset_y!=0))&& (milling_first_move == 0) )//偏移不为0
					{
						PW_SET_CODE_ID(2047);//2020-6-6 zla 记录当前所在代码段位置
						//1、走到偏差位置
						allx_step_tmp = allx_step;
						ally_step_tmp = ally_step;
						//2018-10-26 禁止在偏移时出现急停，此时不响应急停
						shift_no_pause=1;
						go_commandpoint(laser_offset_x + allx_step,laser_offset_y + ally_step);//执行偏移地址
						milling_first_move = 1;//避免重复偏移
						shift_no_pause=0;//恢复对急停的响应
						allx_step = allx_step_tmp;//为显示的光标正确，绝对坐标不考虑偏移值，和画笔偏移是一样的
						ally_step = ally_step_tmp;
						PW_SET_CODE_ID(2048);//2020-6-6 zla 记录当前所在代码段位置
					}
				}
				laser_already_begin_flag = 0;
			}
			//从激光段跳转到了车缝段，需要走回激光偏移
			else
			{
				//之前未发生急停
				if( (nop_move_pause_flag ==0 )&&(finish_nopmove_pause_flag ==0) )
				{
					if( ((laser_offset_x!=0)||(laser_offset_y!=0))&& (milling_first_move == 1) )//偏移不为0
					{
						PW_SET_CODE_ID(2049);//2020-6-6 zla 记录当前所在代码段位置
						//1、走到偏差位置
						allx_step_tmp = allx_step;
						ally_step_tmp = ally_step;
						//2018-10-26 禁止在偏移时出现急停，此时不响应急停
						shift_no_pause=1;
						go_commandpoint(allx_step -laser_offset_x, ally_step - laser_offset_y);//走回偏移地址
						milling_first_move = 0;//避免重复偏移
						shift_no_pause=0;//恢复对急停的响应
						allx_step = allx_step_tmp;//为显示的光标正确，绝对坐标不考虑偏移值，和画笔偏移是一样的
						ally_step = ally_step_tmp;
						PW_SET_CODE_ID(2050);//2020-6-6 zla 记录当前所在代码段位置
					}
				}
			}
			
		}
	}

	if(FootUpCom == 1)//如果u37设置为2，即缝制结束后压框不抬起，那么必须等待抬起后才能继续缝制
	{
		if(u38 == 0)
		{
			if(DVB == 0)    
			{
				delay_ms(10);
				if(DVB == 0 && DVBLastState == 1)
					{
						PW_SET_CODE_ID(2051);//2020-6-6 zla 记录当前所在代码段位置
							footer_both_up();
							FootUpCom = 0;
			   	    	    DVBLastState = DVB;	
				  	}
			 }
			 else
			 {
					DVBLastState = DVB;
			 }
			//2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();						// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}  

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
			PW_SET_CODE_ID(2052);//2020-6-6 zla 记录当前所在代码段位置
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
	  		PW_SET_CODE_ID(2053);//2020-6-6 zla 记录当前所在代码段位置
	  		temp16 = (UINT16)aging_delay * 100;
	  		if(temp16 > 9900)
	  		{
	  			temp16 = 9900;
	  		}
	  		delay_ms(temp16); 
			foot_com = 0; 	  	
	  	}
	  	
	  	#if DISABLE_THREAD_CLAMP_FUNCTION
		#else
		//抓线位置处理
		//if(u35==0)
		if(u35 == THREAD_SCRATCH_SWITCH_OPEN)
  		{
	  		  if(clamp_com == 1)          
	  		  {
		  		  	if( return_from_setout == 0 )//确保不是缝制过程中返回到READY的情况
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
			  		      		PW_SET_CODE_ID(2054);//2020-6-6 zla 记录当前所在代码段位置
//			  		    	  		sys.status = ERROR;
//									StatusChangeLatch = ERROR;
//									if( sys.error == 0)
//			  		        		    sys.error = ERROR_23;	 		  
									SET_SYS_ERROR(ERROR_23);//抓线位置异常
									//_speed_array[0] = 5;
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
		  		      		PW_SET_CODE_ID(2055);//2020-6-6 zla 记录当前所在代码段位置
//		  		    	 		sys.status = ERROR;
//								StatusChangeLatch = ERROR;
//								if( sys.error == 0)
//		  		        		sys.error = ERROR_23;	
								SET_SYS_ERROR(ERROR_23);//抓线位置异常
								//_speed_array[0] = 6;
		  		        		return;
		  		      	}
	  		    }
	  	   }
  		}
  		//else if(u35 == 1)                   // have no clamp thread motor
  		else if(u35 == THREAD_SCRATCH_SWITCH_CLOSE)// have no clamp thread motor
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
		#endif
		//--------------------------------------------------------------------------------------
	  	//  find origin
	  	//-------------------------------------------------------------------------------------- 
		if(origin_com == 1)
		{			
			if(pause_inpress_flag == 1)//2020-12-10 zla 如果已经气压不足了，不允许找原点
			{
				origin_com = 0;
				return;//退出，下一次进入ready函数就会报气压不足了
			}
			
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
	      			PW_SET_CODE_ID(2056);//2020-6-6 zla 记录当前所在代码段位置
					find_dead_center();
					PW_SET_CODE_ID(2057);//2020-6-6 zla 记录当前所在代码段位置
	      		}
				if( marking_finish_flag == 1 )
				{
					marking_finish_flag = 0;
					if( (pen_x_bios_offset!=0)||(pen_y_bios_offset!=0) )
					{
						
						  //2020-11-14 zla 回原点时关闭画笔
						  MARKING_PEN = 0;//关闭画笔
						  allx_step_tmp = allx_step;
					 	  ally_step_tmp = ally_step;
					      go_commandpoint(allx_step - pen_x_bios_offset ,ally_step - pen_y_bios_offset);
						  allx_step = allx_step_tmp;
					 	  ally_step = ally_step_tmp;
					}
				}
				PW_SET_CODE_ID(2058);//2020-6-6 zla 记录当前所在代码段位置
				//2018-11-19 新增旋转切刀宏定义，用于减少代码量
				#if ENABLE_ROTATED_CUTTER==1
				if( (milling_cutter_action_flag != 0)|| ( (ROTATED_CUTTER == 1)&&(rotated_cutter_enable ==1) ) )
				{
					if( (ROTATED_CUTTER == 1)&&(rotated_cutter_enable ==1) )
					{
						//DRILL_MOTOR_UPDOWN = 0;//确认
						rotated_cutter_down_positon();
						DRILL_FOOTER = 0;
						drill_motor_run_enable = 0; //铣刀关电
						delay_ms(500);
					}
					if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 1) )//偏移不为0
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

				//SUM = 1;
				PW_SET_CODE_ID(2059);//2020-6-6 zla 记录当前所在代码段位置
				go_origin_allmotor();
				PW_SET_CODE_ID(2060);//2020-6-6 zla 记录当前所在代码段位置
				//SUM = 0;
				
				//#if CHANGE_DOUBLE_FRAMEWORK   //双换模板
				#if 0
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
				
//				if( k115 == 1 )
				{
					if ( already_auto_find_start_point == 0)
					{						
						//find_start_point();
						already_auto_find_start_point = 1;
					}
				}
						
				//if(u39 != 1)   
				if(u39 != AFTER_SEWING_FINISH_GO_ORIGIN_OPEN)
				{
					PW_SET_CODE_ID(2061);//2020-6-6 zla 记录当前所在代码段位置
				   go_setoutpoint();
				   PW_SET_CODE_ID(2062);//2020-6-6 zla 记录当前所在代码段位置
				}
			
				if(OutOfRange_flag == 1)
				{
					PW_SET_CODE_ID(2063);//2020-6-6 zla 记录当前所在代码段位置
//					sys.error = ERROR_15;
					SET_SYS_ERROR_1(ERROR_15);//超出缝制范围
					StatusChangeLatch = ERROR;
				}
				FootRotateFlag = 0;
//	            if(k110 == 1)  
//		        {   
//		            R_AIR = 0;	
//					delay_ms(80);
//		        }
				predit_shift = 0;	
		    	origin_com = 0;  
			
				if( (u38 == 0)&&(return_from_preddit==0 ) )
				{
				   footer_both_up();
				}   
				return_from_preddit = 0;  	
				if ( foot_flag ==0)
				  FootNumber = 1;
				//2002-8-29 zla 找完原点后，不能响应状态切换，不响应从READY切换到FREE
				//避免开机连续按下两次机头键后主控切换到FREE，但是面板还留在蓝色READY界面
				//无法启动缝制
				if(StatusChangeLatch == FREE)
				{
					StatusChangeLatch = READY;
				}
				
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
						PW_SET_CODE_ID(2064);//2020-6-6 zla 记录当前所在代码段位置
						find_dead_center();
						PW_SET_CODE_ID(2065);//2020-6-6 zla 记录当前所在代码段位置
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
	  	        	 #if 0
					 if( k115 == 0) 
					 	 already_auto_find_start_point = 0;
					 #endif
	  	        }
	  	        break;		
  	
	  		case 2:  break;               
  		       
	  		default: break;                   	         	
	  	}  
		
		if(aging_com == 1)
		{
//			if(k110 == 2)
//			{
//				delay_ms(200);  	  	
//				stretch_foot_out();
//			}
		}
  	
	  	if(aging_com == 1)
		 {
		    	if( (foot_flag == 1 && u202 == 1) || (foot_flag == 0))
			      {
				      temp8 = detect_position();	
		    	      if(temp8 == OUT)    
		      		  {
		      		  	PW_SET_CODE_ID(2066);//2020-6-6 zla 记录当前所在代码段位置
						   find_dead_center();
						   PW_SET_CODE_ID(2067);//2020-6-6 zla 记录当前所在代码段位置
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
		    				PW_SET_CODE_ID(2068);//2020-6-6 zla 记录当前所在代码段位置
                       	    pre_run_conditioning();
                       	    PW_SET_CODE_ID(2069);//2020-6-6 zla 记录当前所在代码段位置
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
					PW_SET_CODE_ID(2070);//2020-6-6 zla 记录当前所在代码段位置
					predit_shift = 0;
					sys.status = StatusChangeLatch;
					return;
				}
				else if(stop_flag == 1)
				{
					if(StatusChangeLatch == NEEDLE || StatusChangeLatch == SINGLE)	
					{
						PW_SET_CODE_ID(2071);//2020-6-6 zla 记录当前所在代码段位置
						predit_shift = 0;
						sys.status = StatusChangeLatch;
						return;
					}	
					else
					{
						PW_SET_CODE_ID(2072);//2020-6-6 zla 记录当前所在代码段位置
						predit_shift = 0;
						StatusChangeLatch = READY;
						sys.status = READY;
					}	
				}
			} 	
			else
			{
			      if(k60 == 1)//使用三联踏板
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
											   }
								    	}
								  		else if(foot_half_flag == 0)
										{
											foot_half_up();			    				
							    		}
		 					    }
						  	}
						 	DVSMLastState = DVSM;
					}//k60
					//enable_footer_up ==1表示急停后允许压框抬起
					//if( ( enable_footer_up ==1 )|| (((foot_flag ==1)||(return_from_setout == 0))&& (u38 == 0) && ((u41 == 1)||(stop_flag == 0)||((stop_flag == 1) && (stop_number%2 == 1)) )) )
					if( ( enable_footer_up == ENABLE_FOOTER_UP_AFTER_PAUSE_SWITCH_OPEN )||\
						( ((foot_flag ==1)||(return_from_setout == 0)) && (u38 == 0) && ((stop_flag == 0)||((stop_flag == 1) && (stop_number%2 == 1)) )) )
					{

							//2019-9-18 zla
							//虚拟按键功能,支持面板通过专用指令实现和按下启动、压框、急停按钮一个作用
							#if VIRTUAL_BOTTON_FUNCTION_ENABLE	
							//面板控制压框动作
							if(additional_command == ADDITIONAL_COMMAND_CLAMP_UP)//外压框抬起
							{
								foot_up();
								//FootNumber = 2;//保证footer_procedure()里的流程正确
								additional_command = ADDITIONAL_COMMAND_NONE;
							}
							else if(additional_command == ADDITIONAL_COMMAND_CLAMP_DOWN)//外压框降下
							{
								foot_down();
								FootNumber = 1;//保证footer_procedure()里的流程正确
								additional_command = ADDITIONAL_COMMAND_NONE;
							}
						  	#endif
						  	
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
					else
					{
						//2019-9-18 zla
						//虚拟按键功能,支持面板通过专用指令实现和按下启动、压框、急停按钮一个作用
						#if VIRTUAL_BOTTON_FUNCTION_ENABLE	
						//面板控制压框动作
						if(additional_command == ADDITIONAL_COMMAND_CLAMP_UP || additional_command == ADDITIONAL_COMMAND_CLAMP_DOWN)//外压框控制
						{
							additional_command = ADDITIONAL_COMMAND_NONE;
						}
						#endif
					}
					
				  //==============================================================================================
				  //满足启动条件了
				//2019-1-17 只在使能了自动送料功能和多功能IO才有效
				#if MULTIPULE_IO_ENABLE && AUTOMATIC_FEEDING_ENABLE
				   if( ( (k60 == 0 || k60 == 1 && DVSM == 1) && DVA == 0 && DVB == 1 ) || (af_info.continue_sewing_flag == 1)  )  
                   {
	 	                  delay_ms(10);
		                 if ( ( (k60 == 0 || k60 == 1 && DVSM == 1) && DVA == 0 && DVALastState == 1 && DVB == 1 ) || (af_info.continue_sewing_flag == 1) )
		                  {
				#else
					//2019-9-18 zla
					//虚拟按键功能,支持面板通过专用指令实现和按下启动、压框、急停按钮一个作用
					#if VIRTUAL_BOTTON_FUNCTION_ENABLE	
				   if( ((k60 == 0 || k60 == 1 && DVSM == 1) && DVA == 0 && DVB == 1) || (additional_command == ADDITIONAL_COMMAND_START_RUN) )   
                   {
	 	                  delay_ms(10);
		                 if ( ((k60 == 0 || k60 == 1 && DVSM == 1) && DVA == 0 && DVALastState == 1 && DVB == 1)  || (additional_command == ADDITIONAL_COMMAND_START_RUN))		
		                  {
		                 	 additional_command = ADDITIONAL_COMMAND_NONE;
					#else
					if( (k60 == 0 || k60 == 1 && DVSM == 1) && DVA == 0 && DVB == 1 )   
                    {
	 	                  delay_ms(10);
		                 if ( (k60 == 0 || k60 == 1 && DVSM == 1) && DVA == 0 && DVALastState == 1 && DVB == 1 )		
		                  {
		            #endif
		        #endif     
		        		//2019-4-27 急停按下后，如果此时要启动，那么报错急停
		        		if(PAUSE == pause_active_level)
		        		{
		        			PW_SET_CODE_ID(2073);//2020-6-6 zla 记录当前所在代码段位置
//		        			sys.status = ERROR;
//							StatusChangeLatch = ERROR;
//					        sys.error = ERROR_02; 
							SET_SYS_ERROR(ERROR_02);//急停
							return;
		        		}

		            	//2019-1-17 只在使能了自动送料功能和多功能IO才有效
						#if MULTIPULE_IO_ENABLE && AUTOMATIC_FEEDING_ENABLE
							if(para8.af_enable_switch == 55 && return_from_setout==0)//2019-4-16 避免急停后重启时错误更新送出料距离
							{
								rc522_write_falg = 0;//表示读取RFID
								RFID_SCAN();//再次读取RFID编号
								if(serail_number == 0)
								{
									delay_ms(100);
									RFID_SCAN();//再次读取RFID编号
									if(serail_number == 0)
									{
										delay_ms(100);
										RFID_SCAN();//再次读取RFID编号
									}
								}
								//rec1_com();
								if(serail_number != 0)
								{
									/*SUM = 1;
									delay_ms(2000);
									SUM = 0;*/
									#if 0
									if(serail_number == 1)//500*800模板
									{
										af_info.width = 8000;
										af_info.height = 5000;
										//计算出当前模板应该走的送出料距离
										af_info.feeding_step = af_info.feeding_step_0 - (af_info.width-af_info.width_0)/2;
										af_info.ejecting_step = af_info.ejecting_step_0 - (af_info.height-af_info.height_0);
									}
									else if(serail_number == 2)//700*1300模板
									{
										af_info.width = 10200;
										af_info.height = 7500;
										//计算出当前模板应该走的送出料距离
										af_info.feeding_step = af_info.feeding_step_0 - (af_info.width-af_info.width_0)/2;
										af_info.ejecting_step = af_info.ejecting_step_0 - (af_info.height-af_info.height_0);
									}
									else//其他编号暂时使用最小模板的长宽
									{
										af_info.width = 8000;
										af_info.height = 5000;
										//计算出当前模板应该走的送出料距离
										af_info.feeding_step = af_info.feeding_step_0 - (af_info.width-af_info.width_0)/2;
										af_info.ejecting_step = af_info.ejecting_step_0 - (af_info.height-af_info.height_0);
									}
									#else
									//计算出当前模板应该走的送出料距离
									//af_info.feeding_step = af_info.feeding_step_0 - (af_info.width-af_info.width_0)/2;
									//af_info.ejecting_step = af_info.ejecting_step_0 - (af_info.height-af_info.height_0);
									#endif
								
									//如果重新启动读出来的是0，那么直接返回
									if(af_info.continue_sewing_flag == 1 && serail_number == 0)
									{
										af_info.continue_sewing_flag = 0;//清除标志位
										return;//不启动，直接开始下一个READY函数
									}
									else
									{
										af_info.continue_sewing_flag = 0;//清除标志位
									}
								}
							}
							
							if( one_step_run_flag == 1) //一键启动，K138=3时此变量置1
							  {
							  	
								  if( return_from_setout == 0 )//全新的一次启动
								  {
								  	 if(para8.af_enable_switch == 55)
								  	 {
								  	 	//如果框架在停车位置，且框架没感应到，说明这个时候需要自动取料
										if(af_info.frame_position_flag == 1 && af_position_detection() != 0 && serail_number != 0 )
										{
											if(serail_number != 0)//只有放料位置有模板才行
											{
												ret = af_feeding();
												if(ret != 0)//取料失败，报错
												{
//													sys.status = ERROR;
//													StatusChangeLatch = ERROR;
//													if( sys.error == 0)
//											  		sys.error = ERROR_110;//缝制前取料失败
													SET_SYS_ERROR(ERROR_110);//缝制前取料失败
											  		return;//直接返回
												}
											}
											else//没有模板就直接原地启动了
											{
											}
										}
										else
										{
										}
										
								  	 }
								  	 else
								  	 {
								  	 	
								  	 }
								  	 footer_both_down();//和二次启动不同，这里是直接将压框降下
								  	 //
								  	 #if 0
									 if( bar_coder_refresh_enable == 0)
									 {
										 pattern_change_flag = 0;
										 if( nop_move_pause_flag ==1)
			    						     process_nop_move_pause(1);
									  	 pre_run_conditioning();
									 }
									 else
									 	 autosewing_allset_flag = 1;
									 #else
										pattern_change_flag = 0;
										if( nop_move_pause_flag ==1)
			    						    process_nop_move_pause(1);
			    						PW_SET_CODE_ID(2074);//2020-6-6 zla 记录当前所在代码段位置
									  	pre_run_conditioning();
									  	PW_SET_CODE_ID(2075);//2020-6-6 zla 记录当前所在代码段位置
									 #endif
								  }
								  else//上次运行未完成，此处继续
								  {
									  if( nop_move_pause_flag ==1)
			    						  process_nop_move_pause(1);
			    					PW_SET_CODE_ID(2076);//2020-6-6 zla 记录当前所在代码段位置
									  pre_run_conditioning();
									  PW_SET_CODE_ID(2077);//2020-6-6 zla 记录当前所在代码段位置
								  }
							  }
							  else//二次启动，第一次启动后到起缝点，第二次开始缝制
							  {			  
							  	//k74=0，电机外压框，K74=1，气动压框，现在模板机都是气动压框
							  	//这段话的意思是保证压框已经降下
							  	//if((k74 == 0 &&  foot_flag == 0) || (k74 == 1 && (u81 <= 1 || u81 >= 4) && foot_flag == 0) || (k74 == 1 && (u81 == 2 || u81 == 3) && foot_flag == 0 && foot_half_flag == 0) )
							  	if( ( (u81 <= 1 || u81 >= 4) && foot_flag == 0) || ( (u81 == 2 || u81 == 3) && foot_flag == 0 && foot_half_flag == 0) )
								{
									DVA_scan_flag = 0;
									DVA_action_done = 0;

									//主轴也回了一下原点
									temp8 = detect_position();	
							    	if( temp8 == OUT)     
							      	{
							      		PW_SET_CODE_ID(2078);//2020-6-6 zla 记录当前所在代码段位置
										find_dead_center();
										PW_SET_CODE_ID(2079);//2020-6-6 zla 记录当前所在代码段位置
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

										  	if(para8.af_enable_switch == 55)
										  	{
										  	 	//如果框架在停车位置，且框架没感应到，说明这个时候需要自动取料
												if(af_info.frame_position_flag == 1 && af_position_detection() != 0 )
												{
													ret = af_feeding();
													if(ret != 0)//取料失败，报错
													{
//														sys.status = ERROR;
//														StatusChangeLatch = ERROR;
//														if( sys.error == 0)
//												  		sys.error = ERROR_110;//缝制前取料失败
														SET_SYS_ERROR(ERROR_110);//缝制前取料失败
												  		return;//直接返回
													}
												}
										  	}
								  	 		PW_SET_CODE_ID(2080);//2020-6-6 zla 记录当前所在代码段位置
										  pre_run_conditioning();										 
										  PW_SET_CODE_ID(2081);//2020-6-6 zla 记录当前所在代码段位置
									}
						    	 }
							  }
						      DVALastState = DVA;
						      
						#else//没有使用胡彬设计的自动送料装置

							  if( one_step_run_flag == 1) //一键启动，K138=3时此变量置1
							  {
								  if( return_from_setout == 0 )//全新的一次启动
								  {
								  	 footer_both_down();//和二次启动不同，这里是直接将压框降下
								  	 //
								  	 #if 0
									 if( bar_coder_refresh_enable == 0)
									 {
										 pattern_change_flag = 0;
										 if( nop_move_pause_flag ==1)
			    						     process_nop_move_pause(1);
									  	 pre_run_conditioning();
									 }
									 else
									 	 autosewing_allset_flag = 1;
									 #else
									 pattern_change_flag = 0;
									 if( nop_move_pause_flag ==1)
		    						     process_nop_move_pause(1);
		    						 PW_SET_CODE_ID(2082);//2020-6-6 zla 记录当前所在代码段位置
								  	 pre_run_conditioning();
								  	 PW_SET_CODE_ID(2083);//2020-6-6 zla 记录当前所在代码段位置
									 #endif
									 
								  }
								  else//上次运行未完成，此处继续
								  {
								    //2020-8-6 zla 避免出现暂停码停止后，如果此时是一键启动导致压框不降下就能启动
								  	footer_both_down();//和二次启动不同，这里是直接将压框降下
									  if( nop_move_pause_flag ==1)
			    						  process_nop_move_pause(1);
			    					PW_SET_CODE_ID(2084);//2020-6-6 zla 记录当前所在代码段位置
									  pre_run_conditioning();
									  PW_SET_CODE_ID(2085);//2020-6-6 zla 记录当前所在代码段位置
								  }
							  }
							  else//二次启动，第一次启动后到起缝点，第二次开始缝制
							  {		
							  	//k74=0，电机外压框，K74=1，气动压框，现在模板机都是气动压框
							  	//这段话的意思是保证压框已经降下
							  	//if((k74 == 0 &&  foot_flag == 0) || (k74 == 1 && (u81 <= 1 || u81 >= 4) && foot_flag == 0) || (k74 == 1 && (u81 == 2 || u81 == 3) && foot_flag == 0 && foot_half_flag == 0) )
							  	if( ((u81 <= 1 || u81 >= 4) && foot_flag == 0) || ((u81 == 2 || u81 == 3) && foot_flag == 0 && foot_half_flag == 0) )
								{
									DVA_scan_flag = 0;
									DVA_action_done = 0;

									//主轴也回了一下原点
									temp8 = detect_position();	
							    	if( temp8 == OUT)     
							      	{
							      		PW_SET_CODE_ID(2086);//2020-6-6 zla 记录当前所在代码段位置
										find_dead_center();
										PW_SET_CODE_ID(2087);//2020-6-6 zla 记录当前所在代码段位置
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
			    						PW_SET_CODE_ID(2088);//2020-6-6 zla 记录当前所在代码段位置
										  pre_run_conditioning();
										  PW_SET_CODE_ID(2089);//2020-6-6 zla 记录当前所在代码段位置
									}
						    	 }
							  }
						      DVALastState = DVA;
						#endif
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
				 if( (autosewing_allow_working == 1)&&(autosewing_allset_flag == 1) )
				 {
					  temp8 = detect_position();	
					  if( temp8 == OUT)
					  {
					  	PW_SET_CODE_ID(2090);//2020-6-6 zla 记录当前所在代码段位置
					  	  find_dead_center();
					  	  PW_SET_CODE_ID(2091);//2020-6-6 zla 记录当前所在代码段位置
					  }
					
					  if ( finish_nopmove_pause_flag == 1 )
					  {
							origin_com =1;
							special_go_allmotor_flag = 1;
					  }
					  else
					  {
							if( nop_move_pause_flag == 1 )
		    					process_nop_move_pause(1);
		    				PW_SET_CODE_ID(2092);//2020-6-6 zla 记录当前所在代码段位置
							pre_run_conditioning();
							PW_SET_CODE_ID(2093);//2020-6-6 zla 记录当前所在代码段位置
					  }
					  autosewing_allow_working = 0;
					  autosewing_allset_flag = 0;	
				 }
				#endif
				
					
			}
			
		}//非老化模式
		
		if( (single_flag ==0 )&&(origin_com == 0)&&(PointShiftFlag==0)&&(coor_com==0) )
		 	predit_shift = 0;
	}//压框不抬
}

/**
  * @函数功能 运行状态处理
  * @参数 无
  * @返回值 无
  */
void run_status(void)
{			
	UINT8 temp8,i,j,flag,last_stitch_down,sewingcontrol_stitchs_abs;
	INT8 tmp_stitchs;
	INT16 temp16;
	UINT16 tmp16_stitchs,tmpspd;		
	PATTERN_DATA *tmp_point;
	UINT32  tmp32_spd1,tmp32_spd2;
	INT16 inpress_up_angle,inpress_down_angle,max_angle;
	UINT8 action_flag0,action_flag1,action_flag2,action_flag3,action_flag4;
	UINT8 nop_stop_flag;	
	//UINT8 start_zoom_sew_count=0;//2018-10-31起缝缩缝加固标志
	INT8 segment;
	INT8 subsequent_stitches;//车缝后面还有多少针车缝
	UINT16 tmp_SewTestStitchCounter;
	UINT16 tmp_pat_buff_total_counter;
	
	UINT16 loop_time;
	UINT32 sys_time_old;
	
	PW_SET_FUNC_ID(3);//2020-6-6 zla 记录当前所在函数位置
	PW_SET_CODE_ID(7);//2020-6-6 zla 记录当前所在代码段位置
	
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
	//2019-4-17 只在使能了自动送料功能和多功能IO才有效
	//有时候连续缝制切换花样的时候，面板会下发一个找原点指令，这里直接清零
	//避免中途停止后，状态恢复会直接回原点
	#if MULTIPULE_IO_ENABLE && AUTOMATIC_FEEDING_ENABLE
		origin_com = 0;
	#endif
	inpress_act_flag = 0;
	
  	//--------------------------------------------------------------------------------------
  	//  parameter confirm
  	//--------------------------------------------------------------------------------------
	return_from_setout = 1;     //表示进行缝制状态，以后回到READY时，也知道缝制没有正常结束。
	para_confirm();	

	DVA_scan_flag = 0;
	DVA_scan_counter = 0;
	DVA_action_done = 0;

	SPISTE3 = 1;
	
	inpress_follow_range_recover_flag = 0;

	tail_reinforce_info.nopmove_remain_dis_x = 0;
	tail_reinforce_info.nopmove_remain_dis_y = 0;
	tail_reinforce_info.sharp_v_reinforce_counter = 0;
	tail_reinforce_info.sharp_v_reinforce_done_flag = 0;
	cut_code_check_done_flag = 0;

	//2019-11-06 zla 新增标志用于指示当前是否处于拐点降速阶段，用于实现动框微调的独立
	enter_corner_deceleration_flag=0;
	
	pause_inpress_flag = 0;
	sfsw_count = 0;
  	//--------------------------------------------------------------------------------------
  	//  process first pattern data
  	//-------------------------------------------------------------------------------------- 
  	while(1)
  	{	    
  		PW_SET_CODE_ID(8);//2020-6-6 zla 记录当前所在代码段位置
    	if ( ready_go_setout_com == 1)//响应回起缝点命令，防止准备状态没有响应，带到这个状态来。
		{
			ready_go_setout_com = 0;
			predit_shift = 0;
		}
		
    	if(end_flag == 1 || stay_flag == 1 || stop_flag == 1)
    	{   	
    		PW_SET_CODE_ID(151);//2020-6-7 zla 记录当前所在代码段位置
			break;
    	}
    	else
    	{
    		PW_SET_CODE_ID(152);//2020-6-7 zla 记录当前所在代码段位置
    		loop_time = 0;//2020-6-8 zla 增加超过最大重复次数报警处理
			while(1)
			{
				loop_time++;
				if( loop_time > 2000 )
				{
					//不可能出现连续2000条空送的情况
//					sys.status = ERROR;
//					if( sys.error == OK ) sys.error = ERROR_205;
					SET_SYS_ERROR(ERROR_205);//延时报错退出机制
					PW_SET_CODE_ID(156);//2020-6-7 zla 记录当前所在代码段位置
					return;
				}
				PW_SET_CODE_ID(9);//2020-6-6 zla 记录当前所在代码段位置
				//2019-3-24 新增面板响应函数rec_com()的防重入机制
				if(ui_borad_reentrant_protection == 0)
				{
					ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
					rec_com();						// communication with panel 
					ui_borad_reentrant_protection = 0;//其他地方又可以使用了
				}
				process_data();
				nop_stop_flag = 0;
				PW_SET_CODE_ID(144);//2020-6-6 zla 记录当前所在代码段位置
	    		if(nopmove_flag == 1)//启动前空送码的处理
	    		{	
					do_pat_point_sub_one();
					
					if( release_tension_before_nopmove ==1)//K30，空送前打开夹线器
					{
						da0_release_flag = 0;
						da0 = 255; //夹线器最松
						da0_release_conter = 0;
						da0_release_flag = 1;
					}
					else
						da0 = 0;
					
					tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
				    set_func_code_info(RUN,0,tmp16_stitchs>>8,tmp16_stitchs);

					#if DEBUG_NOPMOVE
					SUM=1;
					delay_ms(1000);
					SUM=0;
					delay_ms(1000);
					SUM=1;
					delay_ms(1000);
					SUM=0;
					delay_ms(1000);
					SUM=1;
					delay_ms(1000);
					SUM=0;
					delay_ms(1000);
					SUM=1;
					delay_ms(1000);
					SUM=0;
					#endif

					//只在使能了自动送料功能和多功能IO才有效
					#if MULTIPULE_IO_ENABLE && AUTOMATIC_FEEDING_ENABLE
					if(para8.af_enable_switch == 55)
					{
						//2019-1-15 
						if(af_info.frame_position_flag == 1)//如果XY框架在停车位置
						{
							//禁止在偏移时出现急停，此时不响应急停
							shift_no_pause=1;
							go_commandpoint(0, 0);//先回0位
							shift_no_pause=0;//恢复对急停的响应
							af_info.frame_position_flag = 0;
						}
					}
					#endif
					//2019-1-22
					//SUM = 1;
					go_beginpoint(0); //分段空送处理
					PW_SET_CODE_ID(10);//2020-6-6 zla 记录当前所在代码段位置
					//SUM = 0;
					
					if( stop_flag ==1)
					    nop_stop_flag = 1;
					tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
					set_func_code_info(RUN,20,tmp16_stitchs>>8,tmp16_stitchs);
				
					if( release_tension_before_nopmove ==1 )//K30，空送前打开夹线器
					{
						da0_release_flag = 0;
						da0 = 0;  //空送后关闭夹线器，此时最紧
						da0_release_conter = 0;							
					}
						
					if( nop_move_pause_flag == 1)//空送过程中发生急停，直接切换到错误状态
					{
						PW_SET_CODE_ID(145);//2020-6-6 zla 记录当前所在代码段位置
						status_now = READY;
//			      		sys.status = ERROR;
//			      		StatusChangeLatch = ERROR;
//						if( sys.error == 0)
//      		      			sys.error = ERROR_02;
						//2020-12-10 zla 借用急停方式实现气压不足报警
					    if( pause_inpress_flag == 1 )
					    {
					       SET_SYS_ERROR(ERROR_48);//气压不足
					       sfsw_power_on_delay = 2000;
					    }
					    else
					    {
							SET_SYS_ERROR(ERROR_02);//急停
						}
						return;
					}
					if( OutOfRange_flag == 1)
					{
						PW_SET_CODE_ID(146);//2020-6-6 zla 记录当前所在代码段位置
//						sys.error = ERROR_15;
						SET_SYS_ERROR_1(ERROR_15);//超出缝制范围
						StatusChangeLatch = ERROR;
					}
					if( sys.status == ERROR)
					{
						PW_SET_CODE_ID(155);//2020-6-6 zla 记录当前所在代码段位置
						status_now = READY;
						return;
					}
					if( move_flag == 1 || RotateFlag == 1)
					{
					    do_pat_point_add_one();//空送后面接车缝，下面有减1这里先加1
						tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
						set_func_code_info(RUN,23,tmp16_stitchs>>8,tmp16_stitchs);
					}
					if( FootRotateFlag ==1 )
					    FootRotateFlag = 0;
				}
				PW_SET_CODE_ID(147);//2020-6-6 zla 记录当前所在代码段位置
				if( FootRotateFlag ==1 )//启动前记号笔功能
				{
					//if( (k110 == 0) )//无翻转与伸缩压脚装置
					{
						marking_flag = 1;
						PW_SET_CODE_ID(148);//2020-6-6 zla 记录当前所在代码段位置
						process_marking_pen(0);
						PW_SET_FUNC_ID(3);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
						if(sys.status == ERROR)
						{
							return;
						}
						if(origin_com == 1)//2019-4-3 画笔急停后回原点
						{
							sys.status = READY;
							PW_SET_CODE_ID(151);//2020-6-6 zla 记录当前所在代码段位置
							return;
						}
					}
					#if 0
					if(k110 == 1)//有翻转装置
					{
						R_AIR = R_AIR^1;
					}					
					else if(k110 == 2)//有伸缩装置   
					{
						if( stretch_foot_flag == 0)
							stretch_foot_in();
						else
							stretch_foot_out();
					}
					#endif
					FootRotateFlag = 0;
				}
				if ( PatternDelayFlag == 1 )//启动前花样延时
				{
					delay_ms(PattenrDelayTime);
					PatternDelayFlag = 0;
				}
				if ( stop_flag == 1 )	//启动前发现中途停止码
				{
					if( nop_stop_flag == 0)
						do_pat_point_sub_one();

					//2020-12-7 zla 如果是急停上压框上功能码，回到虚拟原点
					if( cs.enable >= 1 )
					{
						if( IsSpecialStopCode(pat_point) == 1 )
						{
							cs_goto_image_positon();
						}
					}
					//if((stop_number%2 == 1)&&(u41 == 0)) 
					if((stop_number%2 == 1)) 
					{
					    footer_both_up();
					}
					sys.status = READY;
				    StatusChangeLatch = READY;
		            StopStatusFlag = 1;
					return;
				}
				if ( end_flag == 1 ) //启动前发现结束码，从FINISH状态返回
				{
					inpress_up();
					process_flag = 0;
					sys.status = FINISH;
					StatusChangeLatch = FINISH;
					//2020-9-27 zla 缝制完成后，清除结束V型加固相关变量和参数，避免出现跑位
					tail_reinforce_info.nopmove_remain_dis_x = 0;
					tail_reinforce_info.nopmove_remain_dis_y = 0;
					tail_reinforce_info.sharp_v_reinforce_counter = 0;
					tail_reinforce_info.sharp_v_reinforce_done_flag = 0;
					TestStatusChangeFlag = 0;
					ready_go_setout_com = 0;
					predit_shift = 0;
					PW_SET_CODE_ID(11);//2020-6-6 zla 记录当前所在代码段位置
					return;
				}
				//2018-11-19 新增旋转切刀宏定义，用于减少代码量
				#if ENABLE_ROTATED_CUTTER==1
				if ( (milling_cutter_action_flag ==1)&&(rotated_cutter_enable==1)&&(ROTATED_CUTTER == 1) )//空送里遇到旋转切刀启动命令
				{				
					PW_SET_CODE_ID(149);//2020-6-6 zla 记录当前所在代码段位置
					process_rotated_cutter();
					PW_SET_FUNC_ID(3);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
					PW_SET_CODE_ID(12);//2020-6-6 zla 记录当前所在代码段位置
					milling_cutter_action_flag = 0;
					if(  (end_flag == 1)||(sys.status == ERROR))
					    return;
				}
				#endif
				//机型7、10、11、12、18、22、23、33、34、38、55
				#if ENABLE_LASER_CUTTER
			   	//if( (milling_cutter_action_flag >=1 )||(laser_already_begin_flag == 1) )	//激光切刀功能
				if( (milling_cutter_action_flag >=1 )||(laser_already_begin_flag == 1)||(milling_first_move == 1) )	//激光切刀功能
				{
					set_func_code_info(RUN,23,0,0);
					//2020-4-2 zla 如果是试缝到激光切割位置再开始缝制，会错误的向前多分析一针，这里必须
					//退回去
					if(laser_already_begin_flag == 1 || milling_first_move == 1)
					{
						do_pat_point_sub_one();
					}
					laser_already_begin_flag = 0;
					
					#if SEWING_TENSION_USER_DEFINED_ENABLE
					if( para.sewing_tension_user_defined_enable == 55 )
					{
						da0=0;//必须关闭
					}
					#endif
					PW_SET_CODE_ID(143);//2020-6-6 zla 记录当前所在代码段位置
					enter_special_process_flag = 1;
					process_laser_cutter();
					enter_special_process_flag = 0;
					PW_SET_FUNC_ID(3);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
					PW_SET_CODE_ID(13);//2020-6-6 zla 记录当前所在代码段位置
					
					milling_cutter_action_flag = 0;
					if(  (end_flag == 1)||(sys.status == ERROR) )
					    return;
					second_start_counter = 0;//不用二次启动功能
				}			  
				#endif
				#if ENABLE_LOOPER_CUTTER
				if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
				{
					PW_SET_CODE_ID(150);//2020-6-6 zla 记录当前所在代码段位置
					process_stepper_cutter();
					PW_SET_FUNC_ID(3);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
					PW_SET_CODE_ID(14);//2020-6-6 zla 记录当前所在代码段位置
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
		  	   	#if DISABLE_THREAD_CLAMP_FUNCTION
				#else
        			//if(u35==0)        //车缝前的抓线动作准备 
        			if(u35 == THREAD_SCRATCH_SWITCH_OPEN)
        			{
          				if(clamp_com == 1)          
          				{
          					if(clamp_flag == 0)       
    						{
    							go_origin_ct();
								delay_ms(20);
    							clamp_out();//走到待抓位置
    							clamp_stepflag = 1;      
          			  			movect_angle = 800;      // 800---281 degree
    						}
    						if(TSENS == 0)             // sensor is covered
	    					{	
	    						delay_ms(10);
        						if(TSENS == 0)
        						{
//      	  							sys.status = ERROR;
//									StatusChangeLatch = ERROR;
//									if( sys.error == 0)
//          							sys.error = ERROR_23;	 // clamp is not in normal position  		  
									SET_SYS_ERROR(ERROR_23);//抓线位置异常
          							//_speed_array[0] = 7;
          							return;
        						}
      						}
          				}
          				else
          				{
          					if(clamp_flag == 1)        // clamp is out
            				{
//            		  			sys.status = ERROR;
//								StatusChangeLatch = ERROR;
//								if( sys.error == 0)
//            		  			sys.error = ERROR_23;	   // clamp is not in normal position	 
								SET_SYS_ERROR(ERROR_23);//抓线位置异常
            		  			//_speed_array[0] = 8;
            		  			return;
            				}
            				else
            				{
          			  			clamp_stepflag = 0;      // clamp thread step 0
          					}
          				}
        			} 
        		#endif
					break;
				}//move_flag
				
			}//end while		
			
		}//end else			    	
		if( pause_flag == 1)//press sotp button when stepper motor moving
	  	{
	  	#if SEWING_TENSION_USER_DEFINED_ENABLE
			if( para.sewing_tension_user_defined_enable == 55 )
			{
				da0=0;//出错时必须关闭
			}
		#endif
		
			PW_SET_CODE_ID(15);//2020-6-6 zla 记录当前所在代码段位置
//			sys.status = ERROR;
//			StatusChangeLatch = ERROR;
//	        sys.error = ERROR_02; 
			SET_SYS_ERROR(ERROR_02);//急停
			return;
		}
		 
		if(sys.status == ERROR)    
		  return;		 

		if( (second_start_switch == 1)&&(aging_com == 0) )//二次启动开关
		{
		   if( second_start_counter > 0)
		   {
			   second_start_counter --;
			   status_now = READY;
			   sys.status = READY;
			   StatusChangeLatch = READY;
			   //2020-9-28 zla 二次启动时不能重新赋值，且直接降到最新的基准高度上
			   inpress_down(inpress_high);
			   //inpress_down(inpress_high_hole);
			   //inpress_high = inpress_high_hole;
			   delay_ms(20);
			   PW_SET_CODE_ID(16);//2020-6-6 zla 记录当前所在代码段位置
			   return;
		   }
		}

		//if( k03 == 0)
		if(k03 == TENSION_TYPE_MECHANICAL)
		{
			if ( front2stitchs_tension_off == 0)//起缝前两针夹线器是否打开
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
		
		if( baseline_alarm_flag == 1)//如果底线不足警报开关打开（面板控制）
		{
			//baseline_alarm_stitchs面板实时发送到主控，表示当前底线还够缝制的针数
			//pat_buff_total_counter表示当前已经缝制的总花样点数，包括车缝和其他功能码
			 if( pat_buff_total_counter >= baseline_alarm_stitchs)
			 {
			 	//2020-12-17 zla 新增宏定义作为换梭使能，用于减少代码量
				#if ENABLE_BOBBIN_CASE == 1
				 if( ENABLE_BOBBIN_CASE_FUN == ENABLE_FUN)
				 {
				    if( bobbin_case_enable == 1)
					{
						temp8 = 0;
						tmp16_stitchs = baseline_alarm_stitchs;
						if( bobbin_case_alarm_mode == 1)//先换梭再报警
						{
							sys.status =  ERROR;
//							sys.error = ERROR_45;
							SET_SYS_ERROR_1(ERROR_45);//底线计数不足
							if( bobbin_case_restart_mode == 1)//换梭起缝方式0-手动启动，1-自动启动
							{
								delay_ms(300);//用于告知面板，底线不足
								sys.status =  RUN;//面板显示的底线不足自动消失，继续缝制
								sys.error = 0;
								StatusChangeLatch = READY;
							}
							temp8 = bobbin_case_workflow1();//换梭
							bobbin_case_once_done_flag = 0;
							//2018-9-17，防止换梭失败后错误被ERROR_45覆盖
							if(temp8==0)//换梭失败了
							{
								delay_ms(500);
								//换梭失败，返回，相关的错误状态已经在函数bobbin_case_workflow1()中被设置了
								return;
							}
						}
						if( (bobbin_case_restart_mode == 1)&&(temp8 ==1) )
						{
						//	while( tmp16_stitchs == baseline_alarm_stitchs)
							{
								delay_ms(20);
							}
						}
						else//如果手动换梭，直接跳转到这里，等待拿下面板确认，然后跳转到READY中换梭，然后等待DVA按下后重新启动
						{
//							sys.status =  ERROR;
//							sys.error = ERROR_45;
//							StatusChangeLatch = ERROR;
							SET_SYS_ERROR(ERROR_45);//底线计数不足
						    return;
						}
					}
					else
						{
//							sys.status =  ERROR;
//							sys.error = ERROR_45;
//							StatusChangeLatch = ERROR;
							SET_SYS_ERROR(ERROR_45);//底线计数不足
						    return;
						}
				}
				else
				#endif
				{
//					 sys.status =  ERROR;
//					 sys.error = ERROR_45;
//					 StatusChangeLatch = ERROR;
					 SET_SYS_ERROR(ERROR_45);//底线计数不足
				     return;
				}
				
			 }
		}
		PW_SET_CODE_ID(17);//2020-6-6 zla 记录当前所在代码段位置

		if ( alarm_output_enable >= 2)
		{
			#if DISABLE_FUN //2016-01-22与吹气冲突
			YELLOW_ALARM_LED = 0;
			#endif
			RED_ALARM_LED = 0;
			GREEN_ALALRM_LED = 1;
		}
		//中压脚动作前关闭吹气了
		if( blow_air_flag == 1 )
		{
			blow_air_flag = 0;
			#if 0
		    if( k171 ==1 )
			  	BLOW_AIR3 = 0;
			else
			  	BLOW_AIR2 = 0;
			#else
				BLOW_AIR3 = 0;
			#endif
		}

		//if(blow_air_enable == 2)//起缝前吹气
		if(blow_air_enable == BLOW_AIR_STITCH_OPEN_BEFORE_SEWING)//起缝前吹气
		{
			#if 0
			if( k171 ==1 )
			  	BLOW_AIR3 = 1;
		    else
		  		BLOW_AIR2 = 1;
			#else
				BLOW_AIR3 = 1;
			#endif
			  
			blow_air_counter = (UINT16)k166*10;//2018-11-16 修改单位到10ms,剪线后吹气单位是100ms
			blow_air_flag = 1;
			PW_SET_CODE_ID(18);//2020-6-6 zla 记录当前所在代码段位置
			g_timeout_counter = 5000;
			while(blow_air_flag)//等待吹气完成
			{
				delay_ms(1);
				if( g_timeout_counter == 0 )
				{
//					sys.status = ERROR;
//					if( sys.error == OK ) sys.error = ERROR_205;
					SET_SYS_ERROR(ERROR_205);//延时报错退出机制
					PW_SET_CODE_ID(157);//2020-6-6 zla 记录当前所在代码段位置
					return;
				}
			}
			PW_SET_CODE_ID(19);//2020-6-6 zla 记录当前所在代码段位置
			
		}
		/*#if ENABLE_CONFIG_PARA
		if(para.start_sewing_blow_air_enable == 55)
		{
			if( k171 ==1 )
			  	BLOW_AIR3 = 1;
			  else
			  	BLOW_AIR2 = 1;
			  
			blow_air_counter = (UINT16)k166*100; 
			blow_air_flag = 1;
			while(blow_air_flag);//等待吹气完成
		}
		#endif*/

		//2018-11-19 增加缝制中夹线器力度可调的功能
	#if SEWING_TENSION_USER_DEFINED_ENABLE
		if( para.sewing_tension_user_defined_enable == 55 )
		{
			//取值不能大于50，否则容易烧坏
			if(para.sewing_tension_user_defined_tension <= 100)
			{
				da0 = para.sewing_tension_user_defined_tension;
				//da0_sewing_delay_flag = 1;
				//da0_sewing_delay_count = 2000;//2s后延时关闭，避免烧坏
			}
		}
	#endif


		
    	//--------------------------------------------------------------------------------------
    	//  motor run
    	//--------------------------------------------------------------------------------------			
		//2018-11-17修改,增加非随动压脚的处理，也启用了系统参数，不过仍然使用之前写死的一些参数
		//#if (FOLLOW_INPRESS_FUN_ENABLE && ENABLE_CONFIG_PARA) || (FOURTH_PLATFORM_CONFIG_PARA_ENABLE == 1) || (TASC_PLATFORM_CONFIG_PARA_ENABLE == 1)
		#if ( ENABLE_CONFIG_PARA) || (TASC_PLATFORM_CONFIG_PARA_ENABLE)

		#if 1
		//如果使能了起缝指定针数中压脚基准高度自定义功能
		if(( (start_sew_inpress_hole_lower_enable == START_SEW_INPRESS_HOLE_LOWER_OPEN_HEAD) || (start_sew_inpress_hole_lower_enable == START_SEW_INPRESS_HOLE_LOWER_OPEN_MIDDLE) || (start_sew_inpress_hole_lower_enable == START_SEW_INPRESS_HOLE_LOWER_OPEN_ALL) )
			&& start_sew_inpress_hole_lower_stitchs>0
			&& start_sew_inpress_hole_lower_stitchs<=15
		)
		{
			//2018-11-21 查看当前段位，是在车缝开始还是车缝中间
			segment = check_move_code_position(pat_point);
			/*_speed_array[8] = segment;
			SUM = 1;
			delay_ms(5000);
			SUM = 0;*/
			if( (start_sew_inpress_hole_lower_enable == START_SEW_INPRESS_HOLE_LOWER_OPEN_ALL)
				||((start_sew_inpress_hole_lower_enable == START_SEW_INPRESS_HOLE_LOWER_OPEN_HEAD) && (segment == 1))
				||((start_sew_inpress_hole_lower_enable == START_SEW_INPRESS_HOLE_LOWER_OPEN_MIDDLE) && (segment == 0))
			 )
			 {
				if(flag1 == 1)
				{
					movestep_zx((INT16)start_sew_inpress_hole, fabsm(start_sew_inpress_hole)+9);
				}
				
				//起缝不启用小夹线器
				/*if(u51==0)
				{
					u51=2;
					u51_pc = 0;
				}*/
				
				//2018-11-5 按步数往下多走吧
				inpress_flag = 1;
				inpress_down(inpress_high);//2020-9-28 避免中压脚在高位启动出现撞机针问题，修改为降低到最新的基准高度上
				//inpress_down(inpress_high_hole);//2019-8-28 避免中压脚在高位启动出现撞机针问题
				movestep_zx(-(INT16)start_sew_inpress_hole, fabsm(start_sew_inpress_hole)+9);
				delay_ms(50);
				flag1 = 1;
			
			 }
		#else
		//如果使能了起缝指定针数中压脚基准高度自定义功能
		if(( (para.start_sew_inpress_hole_lower_enable == 1) || (para.start_sew_inpress_hole_lower_enable == 2) || (para.start_sew_inpress_hole_lower_enable == 3) )
			&& para.start_sew_inpress_hole_lower_stitchs>0
			&& para.start_sew_inpress_hole_lower_stitchs<=15
		)
		{
			//2018-11-21 查看当前段位，是在车缝开始还是车缝中间
			segment = check_move_code_position(pat_point);
			/*_speed_array[8] = segment;
			SUM = 1;
			delay_ms(5000);
			SUM = 0;*/
			if( (para.start_sew_inpress_hole_lower_enable == 3)
				||((para.start_sew_inpress_hole_lower_enable == 1) && (segment == 1))
				||((para.start_sew_inpress_hole_lower_enable == 2) && (segment == 0))
			 )
			 {
				if(flag1 == 1)
				{
					movestep_zx((INT16)para.start_sew_inpress_hole,para.start_sew_inpress_hole+9);
				}
				
				//起缝不启用小夹线器
				/*if(u51==0)
				{
					u51=2;
					u51_pc = 0;
				}*/
				
				//2018-11-5 按步数往下多走吧
				inpress_flag = 1;
				inpress_down(inpress_high);	
				movestep_zx(-(INT16)para.start_sew_inpress_hole,para.start_sew_inpress_hole+9);
				delay_ms(50);
				flag1 = 1;
			
			 }
		#endif
			 else
			 {
			 	/*if(u51_pc == 0)
			 	{
			 		u51 = 0;//恢复FW小夹线器的功能
			 		u51_pc = 0;
					
			 		SUM = 1;
					delay_ms(1000);
					SUM = 0;
			 	}*/
			 	flag1 = 0;
				inpress_flag = 1;
				inpress_down(inpress_high);//2020-9-28 避免中压脚在高位启动出现撞机针问题，修改为降低到最新的基准高度上
//				inpress_down(inpress_high_hole);	
			 }
		}
		else
		#endif
		{
			/*u51 = u51_pc;*///恢复FW小夹线器的功能
			inpress_flag = 1;
			inpress_down(inpress_high);//2020-9-29 避免中压脚在高位启动出现撞机针问题，修改为降低到最新的基准高度上
//			inpress_down(inpress_high_hole);
			flag1 = 0;
		}

		/*SUM = 1;
		delay_ms(5000);
		SUM = 0;*/
		
		inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
		
		//2019-6-11 衔接效率提升开关(剪线+空送+起缝),这里避免特殊情况下剪线电机没有回原点就起缝了，例如
		//出现了剪线码后紧跟车缝码的情况
		#if BRIDGING_EFFICIENCY_IMPROVEMENT && ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
		if(trim_motor_need_find_origin_flag == 1)
		{
			PW_SET_CODE_ID(20);//2020-6-6 zla 记录当前所在代码段位置
			g_timeout_counter = 5000;
			while( cutter_delay_flag == 1)
			{
				delay_ms(1);
				if( g_timeout_counter == 0 )
				{
//					sys.status = ERROR;
//					if( sys.error == OK ) sys.error = ERROR_205;
					SET_SYS_ERROR(ERROR_205);//延时报错退出机制
					PW_SET_CODE_ID(158);//2020-6-6 zla 记录当前所在代码段位置
					return;
				}
			}
			PW_SET_CODE_ID(21);//2020-6-6 zla 记录当前所在代码段位置
			go_origin_trim();//等待标缝指令到原点后再找原点
		}
		#endif
		//delay_ms(50);
		
		if( MotorSpeedRigister >= 2 && MotorSpeedRigister <= MAXSPEED0)
		{
			sew_speed = MotorSpeedRigister*100;
		}
		else if(MotorSpeedRigister == 0)
		{
			MotorSpeedRigister = sew_speed/100;
		}
		#if ENABLE_CONFIG_PARA
		if( para.speed_limit_switch == 1)
		{
				if( para.speed_percent >100)
					para.speed_percent = 100;
				tmp32_spd1 = para.speed_percent;
				tmp32_spd2 = sew_speed;				
				tmp32_spd2 = tmp32_spd2 * tmp32_spd1 /100; //转速百分比
				tmp32_spd2 = ( tmp32_spd2 + 50 )/100*100;//四舍五入
				if( tmp32_spd2 < 200)
					tmp32_spd2 = 200;
				sew_speed = tmp32_spd2;
		}
		#endif
		/*
		   sewingcontrol_flag: k18 缝制起始针加固方式设置 0-不加固 1-第一针加固 *2-在前几针加固 3-防鸟巢
		   need_action_once     决定是否这次要执行头部加固操作
		   need_backward_sewing 是否需要反向移动的加固前处理
		*/
		temp8 = 0;

		//k18,=2前几针加固，加固针数由sewingcontrol_stitchs指定
		if( sewingcontrol_flag >= 2 )
		{
			if( need_action_once == 1)//在花样开始位置，要进行加固处理
			{
				if( sewingcontrol_flag == 2 )
			        temp8 = 1;//需要加固，指定针数
				else
				    temp8 = 4;
			}
			else 
			{
				/*
				执行了试缝或是跳转,要看花样指针落在什么位置区间，如果还在加固范围内，可以考虑进行加固。
				条件就是：如果前一针有效代码是空送（不算剪线或其它功能码），那么这针就可以执行加固，
				W型加固因为没有到加固真正起点，所以都按V型加固来处理
				考虑到正常情况下应该是空送结束+剪线码，把以约定前两针范围内是空送，那么起针进行V型加固
				*/
				tmp_point = pat_point;	
				for( i=0; i<2; i++)
				{
					//旋转切刀角度设置指令
					if( ( (tmp_point->func)&0xf0 == 0x80 ) &&(tmp_point->xstep == 0x0e)  )
					{
						temp8 = 0 ;
						break;
					}
					//移到上一针数据
					if( tmp_point > (PATTERN_DATA *)(pat_buf) )
					    tmp_point--;
					//是空送||再上一针不是车缝||再再上一针不是车缝
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
				//默认都是W型，都在开始位置
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
				tmp_point = pat_point;	//还是起针缩缝			
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
        
		PW_SET_CODE_ID(22);//2020-6-6 zla 记录当前所在代码段位置
		//start_zoom_sew_count=0;
		if( (temp8 == 1 )||(temp8 == 3) )//V型或者W型加固
		{
			   if( temp8 ==3)//将原来的V型 换成 W型 进行加固
			   {
				   tmp_stitchs = sewingcontrol_stitchs;
				   if ( sewingcontrol_stitchs >0 )
				     sewingcontrol_stitchs = -sewingcontrol_stitchs;
			   }
			   PW_SET_CODE_ID(23);//2020-6-6 zla 记录当前所在代码段位置
			   SewingReverse();
			   PW_SET_CODE_ID(24);//2020-6-6 zla 记录当前所在代码段位置
			   if( temp8 ==3)
			       sewingcontrol_stitchs = tmp_stitchs;
			   nopmove_flag = 0;
			   move_flag = 0;	
			   cut_flag = 0; 	  
			   need_backward_sewing = 0;
			   need_action_once =0;
			   StitchStartFlag = 0;
		}
		else if(temp8 == 2 )//缩缝密集加固
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
				   PW_SET_CODE_ID(25);//2020-6-6 zla 记录当前所在代码段位置
			   	   zoom_in_one_stitch(3,1);
			   	   PW_SET_CODE_ID(26);//2020-6-6 zla 记录当前所在代码段位置
		       	   need_action_two =0;
				   StitchStartFlag = 0;
				   
					
				   //start_zoom_sew_count=3;//用于指示后面计算起缝中压脚不动作时需要加上这个值
			   }
			   else
			      do_pat_point_sub_one();
		}
		else if(temp8 == 4 )//新加固方式
		{
			motor.spd_obj  = u10 * 100;
			temp8 = fabs(sewingcontrol_stitchs);
			if( temp8 != 0)
			{
				PW_SET_CODE_ID(27);//2020-6-6 zla 记录当前所在代码段位置
				special_sewing( 0 ,temp8,0);//正向先缝2针
				PW_SET_CODE_ID(28);//2020-6-6 zla 记录当前所在代码段位置
				g_timeout_counter = 3000;
				while( motor.angle_adjusted >= 16)
				{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();       				// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(159);//2020-6-8 zla 记录当前所在代码段位置
						return;
					}
				}
				PW_SET_CODE_ID(29);//2020-6-6 zla 记录当前所在代码段位置
		        special_sewing( 1 ,temp8,10);//反向偏移2针
		        PW_SET_CODE_ID(30);//2020-6-6 zla 记录当前所在代码段位置
		        g_timeout_counter = 3000;
				while( motor.angle_adjusted >= 16)
			   	{	
				   //2019-3-24 新增面板响应函数rec_com()的防重入机制
				   if(ui_borad_reentrant_protection == 0)
				   {
					   ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
					   rec_com();					   // communication with panel 
					   ui_borad_reentrant_protection = 0;//其他地方又可以使用了
				   }  
				   if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(160);//2020-6-8 zla 记录当前所在代码段位置
						return;
					}
			   	}

				PW_SET_CODE_ID(31);//2020-6-6 zla 记录当前所在代码段位置
				special_sewing( 0 ,temp8,0);//新位置再正向2针
				PW_SET_CODE_ID(32);//2020-6-6 zla 记录当前所在代码段位置
				g_timeout_counter = 3000;
				while( motor.angle_adjusted >= 16)
		        {
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();       				// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}  
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(161);//2020-6-8 zla 记录当前所在代码段位置
						return;
					}
				}
				
				PW_SET_CODE_ID(33);//2020-6-6 zla 记录当前所在代码段位置
		        special_sewing( 1 ,temp8,-10);//反向偏移2针
		        PW_SET_CODE_ID(34);//2020-6-6 zla 记录当前所在代码段位置
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
		PW_SET_CODE_ID(35);//2020-6-6 zla 记录当前所在代码段位置
		//u51=0表示打开起缝夹线功能，sewingcontrol_flag即K18，=0表示起缝不加固
		//if( (u51 == 0)&&(sewingcontrol_flag ==0) && (flag1==0) )
		if( (u51 == SMALL_YARN_TRAPPER_OPEN)&&(sewingcontrol_flag ==0) && (flag1==0) )
		{
			temp16 = motor.angle_adjusted;
			PW_SET_CODE_ID(36);//2020-6-6 zla 记录当前所在代码段位置
			g_timeout_counter = 3000;
	      	while(temp16 <= 512)    //180d
	        { 
	        	//2019-3-24 新增面板响应函数rec_com()的防重入机制
				if(ui_borad_reentrant_protection == 0)
				{
					ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
					rec_com();       				// communication with panel 
					ui_borad_reentrant_protection = 0;//其他地方又可以使用了
				}  
				temp16 = motor.angle_adjusted;
				if( g_timeout_counter == 0 )
				{
//					sys.status = ERROR;
//					if( sys.error == OK ) sys.error = ERROR_205;
					SET_SYS_ERROR(ERROR_205);//延时报错退出机制
					PW_SET_CODE_ID(162);//2020-6-8 zla 记录当前所在代码段位置
					return;
				}
			}
			PW_SET_CODE_ID(37);//2020-6-6 zla 记录当前所在代码段位置
			FW = 1;//起缝夹线电磁铁（小夹线器，非DA0夹线器）
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
		
		//_speed_array[8] = stitch_counter;
		PW_SET_CODE_ID(38);//2020-6-6 zla 记录当前所在代码段位置
    	//--------------------------------------------------------------------------------------
    	//  sewing cycle 
    	//--------------------------------------------------------------------------------------                
  		while(1)
    	{	
    		PW_SET_CODE_ID(39);//2020-6-6 zla 记录当前所在代码段位置
    		if(predit_shift != 0)
    		{
    			//2019-4-8 如果卡在试缝是否完成里，那么直接置为完成
    			predit_shift = 0;
    		}
			//tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
			//set_func_code_info(RUN,2,tmp16_stitchs>>8,tmp16_stitchs);
			double_xy_time_flag = 0;
			
			#if DSP3_CUTER_DRIVER
			if( (indraft_control_counter==0) &&( indraft_control_flag == 1) )
			{
				indraft_control_flag = 0;
				output_cs3(4,0);
			}
			#endif
			if( StitchStartFlag == 1)
			{
				StitchStartFlag = 0;
			}
			else if(StitchStartFlag == 0)
			{
				PW_SET_CODE_ID(40);//2020-6-6 zla 记录当前所在代码段位置
				g_timeout_counter = 3000;
				//while(motor.angle_adjusted >= 220 && (RotateFlag == 0))//等待角度小于220/4=55°，即等待新的一圈到来
				while(motor.angle_adjusted >= 220 )//等待角度小于220/4=55°，即等待新的一圈到来
				{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();						// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}
					if( (sys.status == ERROR)&&(stay_flag==0) )
					{
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
						PW_SET_CODE_ID(41);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
					if(sys.status == POWEROFF)  
					{
						PW_SET_CODE_ID(42);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(163);//2020-6-8 zla 记录当前所在代码段位置
						return;
					}
				}
				PW_SET_CODE_ID(43);//2020-6-6 zla 记录当前所在代码段位置

				stitch_counter++;
			}
			special_pause_flag = 0;
			
			//if( u71 == 1)
			if( u71 == THREAD_BREAK_DETECTIVE_SWITCH_OPEN
			 && motor.spd_obj > 0 
			 && stay_flag == 0 )//断线检测打开
			{
				  temp16 = motor.angle_adjusted;
				  
				  PW_SET_CODE_ID(44);//2020-6-6 zla 记录当前所在代码段位置
				  g_timeout_counter = 3000;
				  while(temp16 < 151)//53d = 
	    		  {
					  temp16 = motor.angle_adjusted;
					  //2019-3-24 新增面板响应函数rec_com()的防重入机制
						if(ui_borad_reentrant_protection == 0)
						{
							ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
							rec_com();       				// communication with panel 
							ui_borad_reentrant_protection = 0;//其他地方又可以使用了
						}
						if( g_timeout_counter == 0 )
						{
//							sys.status = ERROR;
//							if( sys.error == OK ) sys.error = ERROR_205;
							SET_SYS_ERROR(ERROR_205);//延时报错退出机制
							PW_SET_CODE_ID(164);//2020-6-8 zla 记录当前所在代码段位置
							return;
						}
						if( sys.error != OK || sys.status == ERROR )
							return;
				  }
				  PW_SET_CODE_ID(45);//2020-6-6 zla 记录当前所在代码段位置
				  
				  if(TH_BRK == thread_break_detect_level)//断线了
			      {
					thbrk_count = thbrk_count + 1;//断线次数+1
			      }
			}
			
			if( (front2stitchs_tension_off ==  1) && ( stitch_counter > 1) )
			{
				//if( k03 == 0 )//机械夹线器
				if(k03 == TENSION_TYPE_MECHANICAL)
				{
					da0_release_flag = 0;
					da0 = 0;//最紧
					SNT_H = 0;
				}
			}
			//u51=0表示打开起缝夹线功能,stitch_counter>1表示已经不是第一针了
			//if( (u51 == 0)&&(stitch_counter>1) )
			if( (u51 == SMALL_YARN_TRAPPER_OPEN)&&(stitch_counter>1) )
			{
				FW = 0;//关闭起缝夹线电磁铁（小夹线器，非DA0夹线器）
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
			#if ENABLE_CONFIG_PARA
			if( para.speed_limit_switch == 1)
			{
				if( para.speed_percent >100)
					para.speed_percent = 100;
				tmp32_spd1 = para.speed_percent;
				tmp32_spd2 = sew_speed;				
				tmp32_spd2 = tmp32_spd2 * tmp32_spd1 /100; //转速百分比
				tmp32_spd2 = ( tmp32_spd2 + 50 )/100*100;//四舍五入
				if( tmp32_spd2 < 200)
					tmp32_spd2 = 200;
				sew_speed = tmp32_spd2;
			}
			#endif
			PW_SET_CODE_ID(46);//2020-6-6 zla 记录当前所在代码段位置
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
				
					if( check_sewing_range())//如果超出缝制范围
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
							if( sewingcontrol_tail_flag ==1 )//K20=1，尾部使用密集加固
							{
								if( (pat_point->func ==0x80)||(pat_point->func ==0xc0) )
								{
									if(pat_point->xstep == 0x04)//next code =cut
									{
										//2018-10-31,速度由zoom_in_one_stitch()控制
										//motor.spd_obj = k43*10;//????2017-3-3
										PW_SET_CODE_ID(47);//2020-6-6 zla 记录当前所在代码段位置
										zoom_in_one_stitch(3,0);
										PW_SET_CODE_ID(48);//2020-6-6 zla 记录当前所在代码段位置
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
					   //if( (k110 == 0) )
					   {
							marking_flag = 1;
							process_marking_pen(0);
							PW_SET_FUNC_ID(3);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
							if(sys.status == ERROR)
							{
								PW_SET_CODE_ID(49);//2020-6-6 zla 记录当前所在代码段位置
								return;
							}
							if(origin_com == 1)//2019-4-3 画笔急停后回原点
							{
								sys.status = READY;
								PW_SET_CODE_ID(50);//2020-6-6 zla 记录当前所在代码段位置
								return;
							}
					   }
					   #if 0
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
					   #endif
						FootRotateFlag = 0;
			 }
	
			//--------------------------------------------------------------------------------------
      		//  pause
      		//-------------------------------------------------------------------------------------- 
	      	if(stay_flag == 1)//出现了急停
	      	{
								
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
					movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
				pause_stop();
				if( sys.status == ERROR )
					return;
				if(motor.stop_flag == 1)
				{
					PW_SET_CODE_ID(51);//2020-6-6 zla 记录当前所在代码段位置
					return;
				}
				tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
				set_func_code_info(RUN,22,tmp16_stitchs>>8,tmp16_stitchs);
				
				PW_SET_CODE_ID(52);//2020-6-6 zla 记录当前所在代码段位置
				g_timeout_counter = 3000;
				while(motor.angle_adjusted <= 512)
				{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();       				// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}  
					if( (sys.status == ERROR)&&(stay_flag==0) )
					{
					   sys.status = ERROR;
					   StatusChangeLatch = ERROR;
					   PW_SET_CODE_ID(53);//2020-6-6 zla 记录当前所在代码段位置
					   PW_SET_STOP_CODE(27);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
					   sewing_stop();
					   PW_SET_CODE_ID(54);//2020-6-6 zla 记录当前所在代码段位置
					   g_timeout_counter = 3000;
					   while(motor.stop_flag == 0)
				       {
							//2019-3-24 新增面板响应函数rec_com()的防重入机制
							if(ui_borad_reentrant_protection == 0)
							{
								ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
								rec_com();       				// communication with panel 
								ui_borad_reentrant_protection = 0;//其他地方又可以使用了
							}  
							if( g_timeout_counter == 0 )
							{
//								sys.status = ERROR;
//								if( sys.error == OK ) sys.error = ERROR_205;
								SET_SYS_ERROR(ERROR_205);//延时报错退出机制
								PW_SET_CODE_ID(194);//2020-6-8 zla 记录当前所在代码段位置
								return;
							}
						}
						PW_SET_CODE_ID(55);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
					if(sys.status == POWEROFF)
					{
						PW_SET_CODE_ID(56);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(165);//2020-6-8 zla 记录当前所在代码段位置
						return;
					}
			    }
	      	}
			
				
			if(RotateFlag == 1)
			{
				//SUM = 1;
				set_func_code_info(RUN,5,0,0);
				PW_SET_CODE_ID(57);//2020-6-6 zla 记录当前所在代码段位置
				temp16 = motor.angle_adjusted;
	      		//while(temp16 <= 1010)    // 816---287 degree     597---210 degree 
	      		g_timeout_counter = 3000;
	      		while(temp16 <= angle_tab[295])    // 816---287 degree     597---210 degree 
	        	{ 
	        		//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();       				// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}
					temp16 = motor.angle_adjusted;        
					if( (sys.status == ERROR)&&(stay_flag==0) )
					{
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
						PW_SET_CODE_ID(58);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
					if(sys.status == POWEROFF)
					{
						PW_SET_CODE_ID(59);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(166);//2020-6-8 zla 记录当前所在代码段位置
						return;
					}
					
				}
				PW_SET_CODE_ID(60);//2020-6-6 zla 记录当前所在代码段位置
				PW_SET_STOP_CODE(28);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
				sewing_stop();
				PW_SET_CODE_ID(61);//2020-6-6 zla 记录当前所在代码段位置
				g_timeout_counter = 3000;
				while(motor.stop_flag == 0)
				{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();						// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}  
					if(sys.status == POWEROFF)
					{
						PW_SET_CODE_ID(62);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(167);//2020-6-8 zla 记录当前所在代码段位置
						return;
					}
				}
				PW_SET_CODE_ID(63);//2020-6-6 zla 记录当前所在代码段位置
				delay_ms(80);
				inpress_up();
				//if( k03 == 0 )  
				if(k03 == TENSION_TYPE_MECHANICAL)
				{
					da0 = 0;
					SNT_H = 0;
				}
				if(u51 == SMALL_YARN_TRAPPER_OPEN)
				{
					FW=0;
				}
				RotateFlag = 0;
				//SUM = 0;
				PW_SET_CODE_ID(64);//2020-6-6 zla 记录当前所在代码段位置
				break;
			}
	      	//--------------------------------------------------------------------------------------
	      	//  thread breakage 
	      	//-------------------------------------------------------------------------------------- 
	      	if(brkdt_flag == 1)
	      	{
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
					movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
				PW_SET_CODE_ID(65);//2020-6-6 zla 记录当前所在代码段位置
				PW_SET_STOP_CODE(29);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
				sewing_stop();
				PW_SET_CODE_ID(66);//2020-6-6 zla 记录当前所在代码段位置
				g_timeout_counter = 3000;
				while(motor.stop_flag == 0)
				{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();       				// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}  
					if(sys.status == POWEROFF)
					{
						PW_SET_CODE_ID(67);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(168);//2020-6-8 zla 记录当前所在代码段位置
						return;
					}
				}
				PW_SET_CODE_ID(68);//2020-6-6 zla 记录当前所在代码段位置
				tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
				set_func_code_info(RUN,6,tmp16_stitchs>>8,tmp16_stitchs);
				
				delay_ms(180);
				
				//if(u42 == 1)
				 if(u42 == NEEDLE_STOP_POSITION_UP_DEAD_POINT)
				{	
					PW_SET_CODE_ID(69);//2020-6-6 zla 记录当前所在代码段位置
					find_dead_point();
					PW_SET_CODE_ID(70);//2020-6-6 zla 记录当前所在代码段位置
					if( sys.status == ERROR )
						return;
				}
				
				
				temp8 = detect_position();
				if(temp8 == OUT)
				{
					PW_SET_CODE_ID(153);//2020-6-6 zla 记录当前所在代码段位置
					find_dead_center();
					PW_SET_CODE_ID(154);//2020-6-6 zla 记录当前所在代码段位置
					if( sys.status == ERROR )
						return;
				}
				
				
				delay_ms(80);
			    //if(k03 == 0) 
			    if(k03 == TENSION_TYPE_MECHANICAL)
			    {	                 
			    	da0 = 0;
			    	SNT_H = 0;  
				}
			    else
			    {
				   temp_tension = 0;     
      			   at_solenoid();         
				}
				//if(k167 == 1)//断线后辅助压脚降下，已经降下了，这里不需要动作
				if(k167 == FOOTER_STATUS_AFTER_THREAD_BROKE_DOWN)//断线后辅助压脚降下，已经降下了，这里不需要动作
				{
				}
				else//断线后辅助压脚抬起
				{
					inpress_up();
				}
				/*#if AUTO_BACKWARD_THREAD_BREAK && ENABLE_CONFIG_PARA
				if( para.thread_break_backward_switch == 1 )//断线后回退开关
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
				#endif*/
//	        	sys.status = ERROR;
//				StatusChangeLatch = ERROR;
//	        	sys.error = ERROR_17;     			// thread breakage
	        	PW_SET_CODE_ID(71);//2020-6-6 zla 记录当前所在代码段位置
	        	SET_SYS_ERROR(ERROR_17);//断线
	      		return;
	      	}        
			//--------------------------------------------------------------------------------------
			// sewing stop
			//--------------------------------------------------------------------------------------
			if(SewingStopFlag == 1)
			{
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
					movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
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
				
				PW_SET_CODE_ID(72);//2020-6-6 zla 记录当前所在代码段位置
				PW_SET_STOP_CODE(15);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
			  	motor.spd_obj = 0;
			  	g_timeout_counter = 3000;
				while(motor.stop_flag == 0)
				{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();       				// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}
					if( (sys.status == ERROR)&&(stay_flag==0) )
					{
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
						PW_SET_CODE_ID(73);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
					if(sys.status == POWEROFF)
					{
						PW_SET_CODE_ID(74);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(169);//2020-6-8 zla 记录当前所在代码段位置
						return;
					}
				}
				PW_SET_CODE_ID(75);//2020-6-6 zla 记录当前所在代码段位置
	      		delay_ms(20);
				
				if(SewingStopValue == 2)  
				{
				    temp16 = motor.angle_adjusted;
							
     				if( (temp16 > DEADPOINT)&&(temp16 < DEGREE_180) )
					   motor.dir = 1;
	   				else
	  	 			   motor.dir = 0;
					
	    			motor.spd_obj = 120;
	    			PW_SET_CODE_ID(76);//2020-6-6 zla 记录当前所在代码段位置
	    			g_timeout_counter = 3000;
	   				while(1)
	    			{
						//2019-3-24 新增面板响应函数rec_com()的防重入机制
						if(ui_borad_reentrant_protection == 0)
						{
							ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
							rec_com();						// communication with panel 
							ui_borad_reentrant_protection = 0;//其他地方又可以使用了
						}  
						if(motor.spd_ref == motor.spd_obj)
						{
							break;
						}
						if( g_timeout_counter == 0 )
						{
//							sys.status = ERROR;
//							if( sys.error == OK ) sys.error = ERROR_205;
							SET_SYS_ERROR(ERROR_205);//延时报错退出机制
							PW_SET_CODE_ID(170);//2020-6-8 zla 记录当前所在代码段位置
							return;
						}
					}
					PW_SET_CODE_ID(77);//2020-6-6 zla 记录当前所在代码段位置
					
					motor.stop_angle = DEADPOINT;
					if(motor.stop_angle >= 1024)
						motor.stop_angle = motor.stop_angle - 1024;
						
					PW_SET_CODE_ID(78);//2020-6-6 zla 记录当前所在代码段位置
					
					PW_SET_STOP_CODE(16);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
					motor.spd_obj = 0;
					g_timeout_counter = 3000;
					while(motor.stop_flag == 0)
					{
						//2019-3-24 新增面板响应函数rec_com()的防重入机制
						if(ui_borad_reentrant_protection == 0)
						{
							ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
							rec_com();       				// communication with panel 
							ui_borad_reentrant_protection = 0;//其他地方又可以使用了
						}
						if( g_timeout_counter == 0 )
						{
//							sys.status = ERROR;
//							if( sys.error == OK ) sys.error = ERROR_205;
							SET_SYS_ERROR(ERROR_205);//延时报错退出机制
							PW_SET_CODE_ID(171);//2020-6-8 zla 记录当前所在代码段位置
							return;
						}
					}
					PW_SET_CODE_ID(79);//2020-6-6 zla 记录当前所在代码段位置
				}
				
				if(SewingStopValue == 1 || SewingStopValue == 2)
				{       
	        		inpress_up();
				}
				
				SewingStopFlag = 0;
				PW_SET_CODE_ID(80);//2020-6-6 zla 记录当前所在代码段位置
				break;
			}
	    	//--------------------------------------------------------------------------------------
	      	//  cut and stop
	      	//-------------------------------------------------------------------------------------- 
	    	if(cut_flag == 1) 
	      	{
	      		//剪线码时，计算了一次转速，然后进入特殊缝纫函数special_sewing()又计算了一次转速，导致之前计算的被覆盖了
	      		//因此结束加固时转速出现混乱
				 tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
				//指引加固时第一次不要再check_data()计算转速了
				cut_code_check_done_flag = 1;
				
			 	 set_func_code_info(RUN,8,tmp16_stitchs>>8,tmp16_stitchs);
				 //_speed_array[7] = motor.spd_obj/100;
				 //2020-8-21 zla 检测结尾加固是否满足要求，如果针数不够，就取消加固
				 if( sewingcontrol_tail_flag >=2 )
				 {
				 	subsequent_stitches = check_move_code_foregoing_sew_code( (PATTERN_DATA*)(pat_point-1) );
				 	if( sewingcontrol_tail_flag >= 5 )
				 	{
				 		temp8 = sewingcontrol_tail_flag - 3;
				 	}
				 	else
				 	{
				 		temp8 = sewingcontrol_tail_flag;
				 	}
				 	
				 }
				 if( ( sewingcontrol_tail_flag >=2 ) && ( subsequent_stitches > (temp8+1) ) )
				 {
					  tmp_point = pat_point;
					  tmp_SewTestStitchCounter = SewTestStitchCounter;
					  tmp_pat_buff_total_counter = pat_buff_total_counter;
					  
					  if( sewingcontrol_tail_flag == 2 || sewingcontrol_tail_flag == 5)
					  {
					  		PW_SET_CODE_ID(81);//2020-6-6 zla 记录当前所在代码段位置
						  special_sewing(2,2,0);
						  PW_SET_CODE_ID(82);//2020-6-6 zla 记录当前所在代码段位置
						  g_timeout_counter = 3000;
						  while(motor.angle_adjusted >= 16)
						  {
							  //2019-3-24 新增面板响应函数rec_com()的防重入机制
								if(ui_borad_reentrant_protection == 0)
								{
									ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
									rec_com();       				// communication with panel 
									ui_borad_reentrant_protection = 0;//其他地方又可以使用了
								}
								if( g_timeout_counter == 0 )
								{
//									sys.status = ERROR;
//									if( sys.error == OK ) sys.error = ERROR_205;
									SET_SYS_ERROR(ERROR_205);//延时报错退出机制
									PW_SET_CODE_ID(172);//2020-6-8 zla 记录当前所在代码段位置
									return;
								}
						  }
						  PW_SET_CODE_ID(83);//2020-6-6 zla 记录当前所在代码段位置
						  if(sewingcontrol_tail_flag == 5)//如果是V型加固那就不走后面那一段的
						  {
						  		cut_flag = 0;
								tail_reinforce_info.nopmove_remain_dis_x = 0;
								tail_reinforce_info.nopmove_remain_dis_y = 0;
								
								PW_SET_CODE_ID(84);//2020-6-6 zla 记录当前所在代码段位置
								loop_time = 0;
								while(1)
								{
									process_data();
									if(cut_flag==1)//找到当前这个车缝码以后，再退出，此时花样指针也回到了车缝尾部
									{
										break;
									}
									else if(move_flag == 1)//车缝数据，计算剩下的距离
									{
										move_flag = 0;
										tail_reinforce_info.nopmove_remain_dis_x += xstep_cou;
										tail_reinforce_info.nopmove_remain_dis_y += ystep_cou;
									}
									
									loop_time++;
									if( loop_time > 1000 )
									{
										//超过1000针退出
//										sys.status = ERROR;
//										if( sys.error == OK ) sys.error = ERROR_205;
										SET_SYS_ERROR(ERROR_205);//延时报错退出机制
										PW_SET_CODE_ID(180);//2020-6-7 zla 记录当前所在代码段位置
										return;
									}
								}
								PW_SET_CODE_ID(85);//2020-6-6 zla 记录当前所在代码段位置
								tail_reinforce_info.sharp_v_reinforce_done_flag = 1;
								tail_reinforce_info.sharp_v_reinforce_counter = 2;
						  }
						  else
						  {	  //N型加固再多走这一部分
						  	PW_SET_CODE_ID(86);//2020-6-6 zla 记录当前所在代码段位置
							  special_sewing(0,2,0);	
							  PW_SET_CODE_ID(87);//2020-6-6 zla 记录当前所在代码段位置
							  g_timeout_counter = 3000;
							  while(motor.angle_adjusted >= 16)
							  {
								  //2019-3-24 新增面板响应函数rec_com()的防重入机制
									if(ui_borad_reentrant_protection == 0)
									{
										ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
										rec_com();       				// communication with panel 
										ui_borad_reentrant_protection = 0;//其他地方又可以使用了
									}
									if( g_timeout_counter == 0 )
									{
//										sys.status = ERROR;
//										if( sys.error == OK ) sys.error = ERROR_205;
										SET_SYS_ERROR(ERROR_205);//延时报错退出机制
										PW_SET_CODE_ID(173);//2020-6-8 zla 记录当前所在代码段位置
										return;
									}
							  }
							  PW_SET_CODE_ID(88);//2020-6-6 zla 记录当前所在代码段位置
						  }
					  }
					  else if( sewingcontrol_tail_flag == 3 || sewingcontrol_tail_flag == 6)
					  {
					  		PW_SET_CODE_ID(89);//2020-6-6 zla 记录当前所在代码段位置
						  special_sewing(2,3,0);
						  PW_SET_CODE_ID(90);//2020-6-6 zla 记录当前所在代码段位置
						  g_timeout_counter = 3000;
						  while(motor.angle_adjusted >= 16)
						  {
						  //2019-3-24 新增面板响应函数rec_com()的防重入机制
							if(ui_borad_reentrant_protection == 0)
							{
								ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
								rec_com();       				// communication with panel 
								ui_borad_reentrant_protection = 0;//其他地方又可以使用了
							}
							if( g_timeout_counter == 0 )
							{
//								sys.status = ERROR;
//								if( sys.error == OK ) sys.error = ERROR_205;
								SET_SYS_ERROR(ERROR_205);//延时报错退出机制
								PW_SET_CODE_ID(174);//2020-6-8 zla 记录当前所在代码段位置
								return;
							}
						  }
						  PW_SET_CODE_ID(91);//2020-6-6 zla 记录当前所在代码段位置
						  if(sewingcontrol_tail_flag == 6)//如果是V型加固那就不走后面那一段的
						  {
						  		cut_flag = 0;
								tail_reinforce_info.nopmove_remain_dis_x = 0;
								tail_reinforce_info.nopmove_remain_dis_y = 0;
								PW_SET_CODE_ID(92);//2020-6-6 zla 记录当前所在代码段位置
								loop_time= 0;
								while(1)
								{
									process_data();
									if(cut_flag==1)//找到当前这个车缝码以后，再退出，此时花样指针也回到了车缝尾部
									{
										break;
									}
									else if(move_flag == 1)//车缝数据，计算剩下的距离
									{
										move_flag = 0;
										tail_reinforce_info.nopmove_remain_dis_x += xstep_cou;
										tail_reinforce_info.nopmove_remain_dis_y += ystep_cou;
									}
									loop_time++;
									if( loop_time > 1000 )
									{
										//超过1000针退出
//										sys.status = ERROR;
//										if( sys.error == OK ) sys.error = ERROR_205;
										SET_SYS_ERROR(ERROR_205);//延时报错退出机制
										PW_SET_CODE_ID(181);//2020-6-7 zla 记录当前所在代码段位置
										return;
									}
								}
								PW_SET_CODE_ID(93);//2020-6-6 zla 记录当前所在代码段位置
								tail_reinforce_info.sharp_v_reinforce_done_flag = 1;
								tail_reinforce_info.sharp_v_reinforce_counter = 3;
						  }
						  else
						  {	  //N型加固再多走这一部分
						 	 PW_SET_CODE_ID(94);//2020-6-6 zla 记录当前所在代码段位置
							  special_sewing(0,3,0);	
							  PW_SET_CODE_ID(95);//2020-6-6 zla 记录当前所在代码段位置
							  g_timeout_counter = 3000;
							  while(motor.angle_adjusted >= 16)
							  {
							 	 //2019-3-24 新增面板响应函数rec_com()的防重入机制
								if(ui_borad_reentrant_protection == 0)
								{
									ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
									rec_com();       				// communication with panel 
									ui_borad_reentrant_protection = 0;//其他地方又可以使用了
								}
								if( g_timeout_counter == 0 )
								{
//									sys.status = ERROR;
//									if( sys.error == OK ) sys.error = ERROR_205;
									SET_SYS_ERROR(ERROR_205);//延时报错退出机制
									PW_SET_CODE_ID(175);//2020-6-8 zla 记录当前所在代码段位置
									return;
								}
							  }
							  PW_SET_CODE_ID(96);//2020-6-6 zla 记录当前所在代码段位置
						  }
					  }
					  else if( sewingcontrol_tail_flag == 4 || sewingcontrol_tail_flag == 7)
					  {
					  	PW_SET_CODE_ID(97);//2020-6-6 zla 记录当前所在代码段位置
						  special_sewing(2,4,0);
						  PW_SET_CODE_ID(98);//2020-6-6 zla 记录当前所在代码段位置
						  g_timeout_counter = 3000;
						  while(motor.angle_adjusted >= 16)
						  {
						  //2019-3-24 新增面板响应函数rec_com()的防重入机制
							if(ui_borad_reentrant_protection == 0)
							{
								ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
								rec_com();       				// communication with panel 
								ui_borad_reentrant_protection = 0;//其他地方又可以使用了
							}
							if( g_timeout_counter == 0 )
							{
//								sys.status = ERROR;
//								if( sys.error == OK ) sys.error = ERROR_205;
								SET_SYS_ERROR(ERROR_205);//延时报错退出机制
								PW_SET_CODE_ID(176);//2020-6-8 zla 记录当前所在代码段位置
								return;
							}
						  }
						  PW_SET_CODE_ID(99);//2020-6-6 zla 记录当前所在代码段位置
						  if(sewingcontrol_tail_flag == 7)//如果是V型加固那就不走后面那一段的
						  {
						  		cut_flag = 0;
								tail_reinforce_info.nopmove_remain_dis_x = 0;
								tail_reinforce_info.nopmove_remain_dis_y = 0;
								PW_SET_CODE_ID(100);//2020-6-6 zla 记录当前所在代码段位置
								loop_time= 0;
								while(1)
								{
									process_data();
									if(cut_flag==1)//找到当前这个车缝码以后，再退出，此时花样指针也回到了车缝尾部
									{
										break;
									}
									else if(move_flag == 1)//车缝数据，计算剩下的距离
									{
										move_flag = 0;
										tail_reinforce_info.nopmove_remain_dis_x += xstep_cou;
										tail_reinforce_info.nopmove_remain_dis_y += ystep_cou;
									}
									
									loop_time++;
									if( loop_time > 1000 )
									{
										//超过1000针退出
//										sys.status = ERROR;
//										if( sys.error == OK ) sys.error = ERROR_205;
										SET_SYS_ERROR(ERROR_205);//延时报错退出机制
										PW_SET_CODE_ID(182);//2020-6-7 zla 记录当前所在代码段位置
										return;
									}
								}
								PW_SET_CODE_ID(101);//2020-6-6 zla 记录当前所在代码段位置
								tail_reinforce_info.sharp_v_reinforce_done_flag = 1;
								tail_reinforce_info.sharp_v_reinforce_counter = 4;
						  }
						  else
						  {	  //N型加固再多走这一部分
						  	PW_SET_CODE_ID(102);//2020-6-6 zla 记录当前所在代码段位置
							  special_sewing(0,4,0);	
							  PW_SET_CODE_ID(103);//2020-6-6 zla 记录当前所在代码段位置
							  g_timeout_counter = 3000;
							   while(motor.angle_adjusted >= 16)
							   {
							   		//2019-3-24 新增面板响应函数rec_com()的防重入机制
									if(ui_borad_reentrant_protection == 0)
									{
										ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
										rec_com();       				// communication with panel 
										ui_borad_reentrant_protection = 0;//其他地方又可以使用了
									}
									if( g_timeout_counter == 0 )
									{
//										sys.status = ERROR;
//										if( sys.error == OK ) sys.error = ERROR_205;
										SET_SYS_ERROR(ERROR_205);//延时报错退出机制
										PW_SET_CODE_ID(177);//2020-6-8 zla 记录当前所在代码段位置
										return;
									}
							   }
							   PW_SET_CODE_ID(104);//2020-6-6 zla 记录当前所在代码段位置
						  }
					  }
					  pat_point = tmp_point;
					  SewTestStitchCounter = tmp_SewTestStitchCounter;
					  pat_buff_total_counter = tmp_pat_buff_total_counter;
				 }
				if( (sewingcontrol_flag >=2)&&(sewingcontrol_stitchs !=0) )
				     need_backward_sewing = 1;
			    if( sewingcontrol_flag == 1)
				    need_action_two = 1;
				
			#if SEWING_TENSION_USER_DEFINED_ENABLE
				if( para.sewing_tension_user_defined_enable == 55 )
				{
					da0=0;//剪线是必须关闭
				}
			#endif
				PW_SET_CODE_ID(105);//2020-6-6 zla 记录当前所在代码段位置
				trim_action();				
				PW_SET_CODE_ID(106);//2020-6-6 zla 记录当前所在代码段位置
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
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
			    	//2018-8-4
			    	//如果启动了前几针改变中压脚上升高度的功能，且当前针数属于指定的针数内，那么使用指定的随动范围
					movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
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
				
				PW_SET_STOP_CODE(17);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
			  	motor.spd_obj = 0; 
			  	
				PW_SET_CODE_ID(107);//2020-6-6 zla 记录当前所在代码段位置
				g_timeout_counter = 3000;
				while(motor.stop_flag == 0)
				{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();						// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}  
					if(sys.status == POWEROFF)
					{
						PW_SET_CODE_ID(108);//2020-6-6 zla 记录当前所在代码段位置
				       return;
				    }
				    
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(183);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
				}
				PW_SET_CODE_ID(109);//2020-6-6 zla 记录当前所在代码段位置
				
			#if SEWING_TENSION_USER_DEFINED_ENABLE
				if( para.sewing_tension_user_defined_enable == 55 )
				{
					da0=0;//剪线时必须关闭
				}
			#endif
			
				delay_ms(20);
				if(stop_number == 3 || stop_number == 4) //2013-7-29 add
				{
				   temp16 = motor.angle_adjusted;			
     			   if( (temp16 > DEADPOINT)&&(temp16 < DEGREE_180) )
					 motor.dir = 1;
	   			   else
	  	 			 motor.dir = 0;
					
	    		   motor.spd_obj = 120;//DEADPOINT_SPD;	
	    		   
	    		   PW_SET_CODE_ID(110);//2020-6-6 zla 记录当前所在代码段位置
	    		   g_timeout_counter = 3000;
	   			   while(1)
	    		   {
	    				//2019-3-24 新增面板响应函数rec_com()的防重入机制
						if(ui_borad_reentrant_protection == 0)
						{
							ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
							rec_com();						// communication with panel 
							ui_borad_reentrant_protection = 0;//其他地方又可以使用了
						}   
		 			  	if(motor.spd_ref == motor.spd_obj)
		  			 	{
		  			 	 	break;
		  			 	}
		  			 	
				   		if( g_timeout_counter == 0 )
					   {
//						   sys.status = ERROR;
//						   if( sys.error == OK ) sys.error = ERROR_205;
						   SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						   PW_SET_CODE_ID(184);//2020-6-6 zla 记录当前所在代码段位置
						   return;
					   }
	    		   }
	    		   PW_SET_CODE_ID(111);//2020-6-6 zla 记录当前所在代码段位置
				   motor.stop_angle = DEADPOINT;
		 		   if(motor.stop_angle >= 1024)
		   			  motor.stop_angle = motor.stop_angle - 1024;
				
				   PW_SET_STOP_CODE(18);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
				   motor.spd_obj = 0;  
				   g_timeout_counter = 3000;
	   			   while(motor.stop_flag == 0)    
	   			   {
						//2019-3-24 新增面板响应函数rec_com()的防重入机制
						if(ui_borad_reentrant_protection == 0)
						{
							ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
							rec_com();						// communication with panel 
							ui_borad_reentrant_protection = 0;//其他地方又可以使用了
						}
						if( g_timeout_counter == 0 )
					   {
//						   sys.status = ERROR;
//						   if( sys.error == OK ) sys.error = ERROR_205;
						   SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						   PW_SET_CODE_ID(185);//2020-6-6 zla 记录当前所在代码段位置
						   return;
					   }
	   			   }	
	   			   PW_SET_CODE_ID(112);//2020-6-6 zla 记录当前所在代码段位置
				}
				//if(k03 == 1)    
				if(k03 == TENSION_TYPE_ELECTRIC)
				{
					temp_tension = 0;       
      				at_solenoid();         
			    }
      			
				if(stop_number == 1 || stop_number == 2 || stop_number == 3 || stop_number == 4)
				{       
	        		inpress_up();     
				}  
				
				do_pat_point_sub_one();
				
				//2020-12-7 zla 如果是急停上压框上功能码，回到虚拟原点
				if( cs.enable >= 1 )
				{
					if( IsSpecialStopCode(pat_point) == 1 )
					{
						cs_goto_image_positon();
					}
				}
				
				//if((stop_number%2 == 1)&&(u41 == 0)) 
				if((stop_number%2 == 1)) 
				{
					footer_both_up();   
				}
	        	break;
	      	}    
			
			if(PatternDelayFlag == 1)
			{
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
					movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
				PW_SET_CODE_ID(113);//2020-6-6 zla 记录当前所在代码段位置
				PW_SET_STOP_CODE(30);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
				sewing_stop(); 
				g_timeout_counter = 3000;
				while(motor.stop_flag == 0)
				{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();						// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}  
					if( (sys.status == ERROR)&&(stay_flag==0) )
					{
						PW_SET_CODE_ID(114);//2020-6-6 zla 记录当前所在代码段位置
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
						return;
					}
					if(sys.status == POWEROFF)
					{
						PW_SET_CODE_ID(115);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
					if( g_timeout_counter == 0 )
				   {
//					   sys.status = ERROR;
//					   if( sys.error == OK ) sys.error = ERROR_205;
					   SET_SYS_ERROR(ERROR_205);//延时报错退出机制
					   PW_SET_CODE_ID(186);//2020-6-6 zla 记录当前所在代码段位置
					   return;
				   }
				}
				PW_SET_CODE_ID(116);//2020-6-6 zla 记录当前所在代码段位置
				set_func_code_info(RUN,10,0,0);    
				delay_ms(20); 
				//if(u42 == 1)  
				if(u42 == NEEDLE_STOP_POSITION_UP_DEAD_POINT)
				{
					PW_SET_CODE_ID(117);//2020-6-6 zla 记录当前所在代码段位置
					find_dead_point();
					PW_SET_CODE_ID(118);//2020-6-6 zla 记录当前所在代码段位置
					if( sys.status == ERROR )
						return;
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
				#if SEWING_TENSION_USER_DEFINED_ENABLE
				if( para.sewing_tension_user_defined_enable == 55 )
				{
					da0=0;//剪线时必须关闭
				}
				#endif
		
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
					movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
				
				PW_SET_CODE_ID(119);//2020-6-6 zla 记录当前所在代码段位置
				
				temp16 = motor.angle_adjusted;
				g_timeout_counter = 3000;
      			while(temp16 <= 853) 
				{ 
        			//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();						// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}                                // communication with panel                                               
        			temp16 = motor.angle_adjusted;          
	        		if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(187);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
        		}            
        		PW_SET_CODE_ID(120);//2020-6-6 zla 记录当前所在代码段位置
        		process_flag = 0;	                    
        		PW_SET_STOP_CODE(31);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
 	      		sewing_stop();  
				PW_SET_CODE_ID(121);//2020-6-6 zla 记录当前所在代码段位置
				g_timeout_counter = 3000;
				while(motor.stop_flag == 0)
				{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();						// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(188);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
				}
				PW_SET_CODE_ID(122);//2020-6-6 zla 记录当前所在代码段位置
				set_func_code_info(RUN,11,0,0);
			
				//if(u42 == 1)
				if(u42 == NEEDLE_STOP_POSITION_UP_DEAD_POINT)
				{
					delay_ms(100);
					PW_SET_CODE_ID(123);//2020-6-6 zla 记录当前所在代码段位置
					find_dead_point();
					PW_SET_CODE_ID(124);//2020-6-6 zla 记录当前所在代码段位置
					if( sys.status == ERROR )
						return;
				}
				
				//if(k03 == 0) 
				if(k03 == TENSION_TYPE_MECHANICAL)
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
				
				#if SEWING_TENSION_USER_DEFINED_ENABLE
				if( para.sewing_tension_user_defined_enable == 55 )
				{
					da0=0;//剪线时必须关闭
				}
				#endif
				PW_SET_CODE_ID(125);//2020-6-6 zla 记录当前所在代码段位置
				PW_SET_STOP_CODE(32);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
				sewing_stop();
				g_timeout_counter = 3000;
				while(motor.stop_flag == 0)
				{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();						// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(189);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
				}
				PW_SET_CODE_ID(126);//2020-6-6 zla 记录当前所在代码段位置
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
			
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
					movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
				
				PW_SET_CODE_ID(127);//2020-6-6 zla 记录当前所在代码段位置
				PW_SET_STOP_CODE(33);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
				sewing_stop();  
				g_timeout_counter = 3000;
				while(motor.stop_flag == 0)
				{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();						// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(190);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
				}
				PW_SET_CODE_ID(128);//2020-6-6 zla 记录当前所在代码段位置
				tmp16_stitchs = pat_point - (PATTERN_DATA *)(pat_buf);
				set_func_code_info(RUN,12,tmp16_stitchs>>8,tmp16_stitchs);
	    
			    start_to_speed_down = 0;
				cutter_speed_done_flag = 0;
				//if(u42 == 1)
				if(u42 == NEEDLE_STOP_POSITION_UP_DEAD_POINT)
				{
					PW_SET_CODE_ID(129);//2020-6-6 zla 记录当前所在代码段位置
					find_dead_point();
					PW_SET_CODE_ID(130);//2020-6-6 zla 记录当前所在代码段位置
					if( sys.status == ERROR )
						return;
				}
				
			 	//if(k03 == 0)  
			 	if(k03 == TENSION_TYPE_MECHANICAL)
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
			     
	        	break;//跳出缝制内循环了，进入花样外循环
			}
			//缝制中修改中压脚基准高度码的处理
			if( (inpress_act_flag == 1) && (inpress_high != inpress_position ) )
			{
				inpress_to_forsingle(inpress_high);
				inpress_act_flag = 0;
			}
			else
			{
				//2020-11-02 zla 修复当中压脚高度码指定高度和之前一样时，标志未被清0，导致缝完中压脚
				//下降的问题
				inpress_act_flag = 0;
			}
			
	      	//--------------------------------------------------------------------------------------
	      	//  move stepper motor
	      	//--------------------------------------------------------------------------------------   	              	                   	    
	      	if(move_flag == 1)
	      	{

				//if(IsCutCode((PATTERN_DATA*)(pat_point)))//下一针是剪线
				//{
	      			//_speed_array[6] = motor.spd_obj/100;//剪线前一针转速
				//}
				//2019-8-8
				//为了方便缝制充绒料,增加结束后几针中压脚基准高度降低一些（摆动幅度不变）的功能
				//风格和起缝降低方法一致
				if(	(end_sew_inpress_hole_lower_enable == END_SEW_INPRESS_HOLE_LOWER_OPEN)
					&&( (end_sew_inpress_hole_lower_stitchs >= 1) && (end_sew_inpress_hole_lower_stitchs <= 15) )
					&&( (end_sew_inpress_hole >= 1) && (end_sew_inpress_hole <= 50) )	)
				{
					if(end_sew_inpress_hole_lower_activated_flag == 0)//如果当前没有把中压脚基准降低，那么需要查看一下
					{
						PW_SET_CODE_ID(131);//2020-6-6 zla 记录当前所在代码段位置
						subsequent_stitches = check_move_code_susequent_sew_code( (PATTERN_DATA*)(pat_point-1) );
						PW_SET_CODE_ID(132);//2020-6-6 zla 记录当前所在代码段位置
						//看下当前是否已经已经在需要下压的段里
						if( (subsequent_stitches > 0) && (subsequent_stitches <= end_sew_inpress_hole_lower_stitchs) )
						{
							end_sew_inpress_hole_lower_activated_flag = 1;//需要在降下中压脚的时候，把压脚多降低一些
							
						}						
					}
				}
				
				inpress_up_angle   = angle_tab[inpress_follow_up_angle];
				inpress_down_angle = angle_tab[inpress_follow_down_angle];
				
				action_flag0 = 1;
				action_flag1 = 1;
				action_flag2 = 1;
				action_flag3 = 1;
				action_flag4 = 1;

				//2020-7-9 zla 移植马工中断发送指令函数
				//2020-6-28 zla 增加对赵博士新动框算法的支持
				#if (NEW_COMMAND_SEND_MODE == 1) && (NEW_FRAME_MOVE_SUPPORT_DOC_ZHAO >= 1)//0-不使用，1-断续动框方式，2-连续动框方式
				//X轴动框	
				if( xstep_cou != 0 )
				{
					while( x_pwm_run_flag == 1)
						rec_com();
					movestep_x_doc_zhao(-xstep_cou);
					x_pwm_run_flag = 1;
					allx_step = allx_step + xstep_cou;
					action_flag2 = 0;
					
				}	

				//Y轴动框	
				if( ystep_cou != 0)
				{
					while( y_pwm_run_flag == 1)
						rec_com();
					movestep_y_doc_zhao(-ystep_cou);
					y_pwm_run_flag = 1;
					ally_step = ally_step + ystep_cou;
					action_flag3 = 0;
				}
				#endif



				
				max_angle = movestepy_angle;
				if( movestepx_angle > movestepy_angle )
					max_angle = movestepx_angle;
				if( inpress_up_angle > max_angle )
				    max_angle = inpress_up_angle;
				if( movect_angle  > max_angle )
				    max_angle = movect_angle;
				/*
				  如果发现下降角度比上升角度大，并且上一针已经允许下降了，表示当前这针的下降其实已经执行过了
				*/	
				
                if( (inpress_down_angle > inpress_up_angle )&&(last_stitch_down ==1) )
				     action_flag0 = 0;	//这是因为有些时候，中压脚是在上一圈还没有结束就开始下降的
				else if( (inpress_down_angle > inpress_up_angle )&&(last_stitch_down ==0))//之前10度，这次是340度，那就立即执行
				{
					inpress_down_angle = 5;
				}
				
				/*
				  如果前一针的下降提前执行了，340度的情况，但这针是10度的情况？
				*/
				PW_SET_CODE_ID(133);//2020-6-6 zla 记录当前所在代码段位置
				temp16 = motor.angle_adjusted;
				//max_angle
				g_timeout_counter = 3000;
				while( temp16 < 1000 )//350d
				{
					temp16 = motor.angle_adjusted;
					if( temp16 >=1000)
					    break;
					
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(191);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
						
					if( (temp16 > inpress_down_angle ) &&( action_flag0 == 1) )//中压脚下降角度
					{
						action_flag0 = 0;
					
						if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
						{
							//test_flag =41;							
							//if( movezx_delay_flag == 1)
							  //while( movezx_delay_counter > 0);

							/*
							//如果启动了前几针改变中压脚上升高度的功能，且当前针数属于指定的针数内，那么使用指定的随动范围
							//在新的一针下降时，必须按照之前上升的角度来处理，否则整个随动动作将出错，会出现位置偏移
							if(para.start_sew_change_inpress_high_enable==55
								&&para.start_sew_change_inpress_high_range<=inpress_follow_range
								&&stitch_counter==para.start_sew_change_inpress_high_stitchs)
							{
								//随动范围恢复正常时的下一针，需要按照之前设置的随动高度下降，才能正确回到基准点
								movestep_zx(-(int)para.start_sew_change_inpress_high_range,inpress_follow_down_speed);
							}
							else
							{
								movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
							}
							*/
							movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
							
							inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
							//movezx_delay_counter = inpress_follow_down_speed ;
							//movezx_delay_flag =1;
							//set_func_code_info(RUN,13,0,0);
							
						}						
					}
					PW_SET_CODE_ID(134);//2020-6-6 zla 记录当前所在代码段位置
					if( (temp16 > inpress_up_angle ) &&( action_flag1 == 1) )
					{
						action_flag1 = 0;
						//起缝前几针不动作策略，这里不需要控制中压脚降下，因为只要控制中压脚不抬起，那么控制其降下没有必要，简化了程序
						#if FIRST_STITCH_NOT_ACTION 
						if( (inpress_follow_high_flag == FOLLOW_INPRESS_LOW )&&((stitch_counter/*+start_zoom_sew_count*/) > inpress_not_working_stitchs/*+1*/) )
						#else
						if( inpress_follow_high_flag == FOLLOW_INPRESS_LOW )
						#endif
						{
							//test_flag =61;							
							//if( movezx_delay_flag == 1)
							   //while( movezx_delay_counter > 0);		
							//#if INPRESSER_LOWER_FUN //前几针下降稍微低一些，sewingcontrol_stitchs_abs是起缝加固针数的绝对值
						    //if( (stitch_counter > sewingcontrol_stitchs_abs )&&(flag1 == 1) )
							//{
								//if( inpress_high_hole > 0)//之前降低了一半的最低高度，现在恢复正常
			 					//{
								//	flag1 = 0;//保证只会进来一次
									//回复到正常的最高位置，之前因为最低位置下降了一半，但是随动高度没变，所以最高位置也降低了一些
								// 	movestep_zx(inpress_follow_range + inpress_position_tmp,inpress_follow_up_speed);									
								//}
								//else//之前降低了一半的随动高度，现在恢复正常
								//{
								//	flag1 = 0;
								//	inpress_follow_range = inpress_position_tmp;
								//	movestep_zx(inpress_follow_range,inpress_follow_up_speed);									
								//}
								//turnoff_buz();				 
							//}
							//else
							//{
							//    movestep_zx(inpress_follow_range,inpress_follow_up_speed);								
							//}
							//#else//没有使用前几针下降稍微低一些的策略

							#if ENABLE_CONFIG_PARA || (TASC_PLATFORM_CONFIG_PARA_ENABLE == 1)
							//#if ENABLE_CONFIG_PARA
							//2018-8-8
					    	//如果启动了前几针改变中压脚上升高度的功能，且当前针数属于指定的针数内，那么使用指定的随动范围
					    	//此处程序用于将中压脚随动范围恢复到正常值,最后一针的时候，需要把中压脚的高度恢复到正常高度
							#if 1
							if(start_sew_change_inpress_high_enable == 1 
								&& stitch_counter == (start_sew_change_inpress_high_stitchs+1)
								&& stitch_counter < 20
								/*&&para.start_sew_change_inpress_high_range<=inpress_follow_range*/ )
							#else
							if(para.start_sew_change_inpress_high_enable == 55 
								&& stitch_counter == (para.start_sew_change_inpress_high_stitchs+1)
								&& stitch_counter < 20
								/*&&para.start_sew_change_inpress_high_range<=inpress_follow_range*/ )
							#endif
							{
								//_speed_array[9] = stitch_counter;
								//计算正常的随动高度到变量inpress_follow_range中
								if( inpress_follow_range_pc == 0)//k173，中压脚随动高度，操作头下发
								{
									if( inpress_follow_mode == 5 )//模式5.随动范围是13，否则为8
									    inpress_follow_range = 13;
									else	
										inpress_follow_range = INPRESS_DELTA_RANGE;
								}
								else
									inpress_follow_range = inpress_follow_range_pc;
								inpress_follow_range_recover_flag=1;
							}
							//2019-7-26 zla
							//如果使能了起缝指定针数中压脚基准高度自定义功能
							if(( (start_sew_inpress_hole_lower_enable == START_SEW_INPRESS_HOLE_LOWER_OPEN_HEAD) || (start_sew_inpress_hole_lower_enable == START_SEW_INPRESS_HOLE_LOWER_OPEN_MIDDLE) || (start_sew_inpress_hole_lower_enable == START_SEW_INPRESS_HOLE_LOWER_OPEN_ALL) )
								&& start_sew_inpress_hole_lower_stitchs>0
								&& start_sew_inpress_hole_lower_stitchs<=15
								&& stitch_counter>start_sew_inpress_hole_lower_stitchs
								&& flag1 == 1
							)
							{
								flag1 = 0;//保证只会进来一次
								//回复到正常的最高位置
							 	//movestep_zx(inpress_follow_range + inpress_position_tmp,inpress_follow_up_speed);	
							 	movestep_zx(inpress_follow_range + (INT16)start_sew_inpress_hole, inpress_follow_up_speed);
								
							}
							else if(end_sew_inpress_hole_lower_activated_flag == 1)//2019-8-8 需要抬起
							{		
								if(flag1 == 1)//如果现在还在中压脚起缝基准降低的情况，需要考虑进去，避免出现问题
								{
									flag1 = 0;//避免重复处理
									//少升起指定步数，这样下次再降下的时候，就能保证降低位置是新的基准点位置（比正常基准低）
									//因为起缝降低基准已经向下降低了一些了，所以需要把这个补回来才行
									movestep_zx(inpress_follow_range + (INT16)start_sew_inpress_hole - (INT16)end_sew_inpress_hole, inpress_follow_up_speed);
								}
								else
								{
									//少升起指定步数，这样下次再降下的时候，就能保证降低位置是新的基准点位置（比正常基准低）
									movestep_zx(inpress_follow_range - (INT16)end_sew_inpress_hole, inpress_follow_up_speed);
								}
								end_sew_inpress_hole_lower_activated_flag = 2;//已经执行基准降低，后续剪线完成后，抬压脚时再清0
							}
							else
							{
								movestep_zx(inpress_follow_range,inpress_follow_up_speed);
							}							
							#else
								movestep_zx(inpress_follow_range,inpress_follow_up_speed);
							#endif
							
							//#endif
							inpress_follow_high_flag = FOLLOW_INPRESS_HIGH;
							//movezx_delay_counter = inpress_follow_up_speed ;
							//movezx_delay_flag =1;
									
							//========================================
							stitch_counter++;
							pat_point++;
							PW_SET_CODE_ID(135);//2020-6-6 zla 记录当前所在代码段位置
							check_data(0);//看看下针主轴转速，但不改变实际主轴转速,其实是获取下一针中压脚动框参数
							PW_SET_CODE_ID(136);//2020-6-6 zla 记录当前所在代码段位置
							inpress_down_angle = angle_tab[inpress_follow_down_angle];
							pat_point--;
							stitch_counter--;
							if( inpress_down_angle > inpress_up_angle)//默认情况下是下降小于起始的
							{
								action_flag0 = 1;//允许再次下降，下一针的下降，在这一圈就可以执行了									
								last_stitch_down = 1;//表示上一针已经把这一针压降降下的动作完成了
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
							test_flag = 1;
						//2020-6-28 zla 增加对赵博士新动框算法的支持
						#if NEW_FRAME_MOVE_SUPPORT_DOC_ZHAO >= 1//0-不使用，1-断续动框方式，2-连续动框方式
							movestep_x_doc_zhao( -xstep_cou );
							if(timer_x>1)
								movestepx_delay_counter = timer_x - 1;
						#else
							movestep_x(-xstep_cou);
							movestepx_delay_counter = timer_x + 1;
						#endif
							
							movestep_x_flag = 1;
							
							action_flag2 = 0;
							allx_step = allx_step + xstep_cou;  
							//set_func_code_info(RUN,15,0,0);
						}
					}							
					if( (ystep_cou != 0) &&(temp16 > movestepy_angle ) &&( action_flag3 == 1) )
					{
						if( movestepy_delay_counter ==0 )
						{
							test_flag = 21;
							//2020-6-28 zla 增加对赵博士新动框算法的支持
						#if NEW_FRAME_MOVE_SUPPORT_DOC_ZHAO >= 1//0-不使用，1-断续动框方式，2-连续动框方式
							movestep_y_doc_zhao( -ystep_cou );
							if(timer_y>1)
								movestepy_delay_counter = timer_y - 1;
						#else
							movestep_y(-ystep_cou);
							movestepy_delay_counter = timer_y + 1;
						#endif
							movestep_y_flag = 1;
							
							action_flag3 = 0;
							ally_step = ally_step + ystep_cou;
							//set_func_code_info(RUN,16,0,0);
						}
					}
				#if DISABLE_THREAD_CLAMP_FUNCTION
				#else
					if( (clamp_com == 1) && (stitch_counter <= u33) )     // with clamp thread
        			{
						if( (temp16 > movect_angle)&&(action_flag4 == 1) )
      			  		{		
      			    		if( sewingcontrol_flag !=1 )
				        		move_ct();
							action_flag4 = 0;
						}
					}
				#endif
					
				}
				//action_flag1==1表示上一圈随动中压脚没有抬起
				//此时应该强制抬起中压脚
				if( (inpress_up_angle > 1000 ) &&( action_flag1 == 1) )
				{
					inpress_follow_high_flag = FOLLOW_INPRESS_HIGH;
					movestep_zx(inpress_follow_range,inpress_follow_up_speed);
				}
				move_flag = 0;
				//2019-3-24 新增面板响应函数rec_com()的防重入机制
				if(ui_borad_reentrant_protection == 0)
				{
					ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
					rec_com();						// communication with panel 
					ui_borad_reentrant_protection = 0;//其他地方又可以使用了
				}  
	      	}
  			//2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();						// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}    
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
			if( sys.status == ERROR || sys.error != OK )
		    {
		    	sys.status = ERROR;
				StatusChangeLatch = ERROR; 
				PW_SET_STOP_CODE(34);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
				sewing_stop();
				g_timeout_counter = 3000;
				while(motor.stop_flag == 0)
				{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();						// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}
					if( g_timeout_counter == 0 )
					{
//						sys.status = ERROR;
//						if( sys.error == OK ) sys.error = ERROR_205;
						SET_SYS_ERROR(ERROR_205);//延时报错退出机制
						PW_SET_CODE_ID(192);//2020-6-6 zla 记录当前所在代码段位置
						return;
					}
				}
					
				delay_ms(50);
				inpress_up();
				break;
		    }
			if(sys.status == POWEROFF)  
			{ 
				PW_SET_CODE_ID(137);//2020-6-6 zla 记录当前所在代码段位置
				break;
			}
			//2020-6-9 zla 如果缝制内循环中主轴已经停车或者指令转速为0，那么报错返回吧
			if(motor.stop_flag == 1 || motor.spd_obj == 0)
			{
//				sys.status = ERROR;
//				if( sys.error == OK ) sys.error = ERROR_205;
				SET_SYS_ERROR(ERROR_205);//延时报错退出机制
				PW_SET_CODE_ID(4901);//2020-6-6 zla 记录当前所在代码段位置
				return;
			}
			//set_func_code_info(RUN,21,pat_buff_total_counter>>8,pat_buff_total_counter);

    	}
    	
    	PW_SET_CODE_ID(138);//2020-6-6 zla 记录当前所在代码段位置
    	
		if(stop_flag == 1)
		{
			StopStatusFlag = 1;
			PW_SET_CODE_ID(139);//2020-6-6 zla 记录当前所在代码段位置
				break;
		}
		if(sys.status == ERROR)
		{ 
			StatusChangeLatch = ERROR; 
			break;
		}
	    if(sys.status == POWEROFF)  
		{ 
			PW_SET_CODE_ID(140);//2020-6-6 zla 记录当前所在代码段位置
			break;
		}
  	}
  	PW_SET_CODE_ID(141);//2020-6-6 zla 记录当前所在代码段位置
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
			//2020-9-27 zla 缝制完成后，清除结束V型加固相关变量和参数，避免出现跑位
			tail_reinforce_info.nopmove_remain_dis_x = 0;
			tail_reinforce_info.nopmove_remain_dis_y = 0;
			tail_reinforce_info.sharp_v_reinforce_counter = 0;
			tail_reinforce_info.sharp_v_reinforce_done_flag = 0;
			TestStatusChangeFlag = 0;
			ready_go_setout_com = 0;
			predit_shift = 0;
			PW_SET_CODE_ID(142);//2020-6-6 zla 记录当前所在代码段位置
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

	PW_SET_FUNC_ID(42);//2020-6-6 zla 记录当前所在函数位置
	
#if SEWING_TENSION_USER_DEFINED_ENABLE
	if( para.sewing_tension_user_defined_enable == 55 )
	{
		da0=0;//出错时必须关闭
	}
#endif

	#if 0
	//更新上次PAUSE状态
	if(PAUSE == pause_active_level)
	{
		pause_last_flag = 1;
	}
	else
	{
		pause_last_flag = 0;
	}
	#endif

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
		
		//sewing_stop();
		//2020-6-9 zla 临时让这个时候的停车角度到100°
		PW_SET_STOP_CODE(19);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
		motor.stop_angle = angle_tab[100];				
		motor.spd_obj = 0;
	    while(motor.stop_flag == 0)
		{
			//2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();       				// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}  
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
		case 45://底线报警
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
			if(AC_OVDT)
    		{    					 	          	  
    	    	flash_led();   // flash alarm led                                                   
               	flash_buz();	  // flash alarm buzzer
    		}
    		else
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
		case 48://气压不足
			#if 0
			if(SFSW == 1 && sfsw_power_on_delay == 0)
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
            #else
            if( pause_inpress_flag == 0 )
		    {			
		       	delay_ms(10);
				if( pause_inpress_flag == 0 )
				{
					turnoff_ledbuz();
		       		sys.status = READY;
					StatusChangeLatch = READY;
		        	sys.error = OK; 
				}	    	    	      	    
		    }
			#endif
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
	PW_SET_FUNC_ID(4);//2020-6-6 zla 记录当前所在函数位置
	
	while(motor.stop_flag == 0)    
  	{
		//2019-3-24 新增面板响应函数rec_com()的防重入机制
		if(ui_borad_reentrant_protection == 0)
		{
			ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
			rec_com();						// communication with panel 
			ui_borad_reentrant_protection = 0;//其他地方又可以使用了
		}  
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
	#if 1
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
						if(temp16 > MAXSPEED1*100 || temp16 < 200)
			    		{
			    			temp16 = 1600;
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
			    		if(temp16 > MAXSPEED1*100 || temp16 < 200)
			    		{
			    			temp16 = 1600;
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
					    			//2019-3-24 新增面板响应函数rec_com()的防重入机制
									if(ui_borad_reentrant_protection == 0)
									{
										ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
										rec_com();       				// communication with panel 
										ui_borad_reentrant_protection = 0;//其他地方又可以使用了
									}     						                
					    		}  	 
							while(motor.angle_adjusted < 16)
					    		{
					    			//2019-3-24 新增面板响应函数rec_com()的防重入机制
									if(ui_borad_reentrant_protection == 0)
									{
										ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
										rec_com();       				// communication with panel 
										ui_borad_reentrant_protection = 0;//其他地方又可以使用了
									}    						                
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
	#endif
}
//--------------------------------------------------------------------------------------
//  Name:		wind_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of wind stauts
//--------------------------------------------------------------------------------------
void wind_status(void)
{	  
	UINT8 temp8;
	UINT16 spd_tmp,i;
	PW_SET_FUNC_ID(5);//2020-6-6 zla 记录当前所在函数位置
#if 1	
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
					PW_SET_STOP_CODE(37);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
					sewing_stop();
				
					while(motor.stop_flag == 0)
					{
						//2019-3-24 新增面板响应函数rec_com()的防重入机制
						if(ui_borad_reentrant_protection == 0)
						{
							ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
							rec_com();						// communication with panel 
							ui_borad_reentrant_protection = 0;//其他地方又可以使用了
						}  
					}
					delay_ms(80);
					
					//if(k03 == 0)
					if(k03 == TENSION_TYPE_MECHANICAL)
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
	  	if(DVA == 0 || PAUSE == pause_active_level)           // foot sensor is pushed
	  	//if(DVA == 0)           // foot sensor is pushed
		{
			delay_ms(10);
			if( (DVA == 0 && DVALastState == 1) || (PAUSE == pause_active_level ) )
			//if(DVA == 0 && DVALastState == 1)
			{				
				if(foot_flag == 1)    // if foot is up
				{		 
	      
		    	}
		    	else                  // if foot is down
		    	{	
		    		PW_SET_STOP_CODE(38);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
					sewing_stop();
				
					while(motor.stop_flag == 0)
					{
						//2019-3-24 新增面板响应函数rec_com()的防重入机制
						if(ui_borad_reentrant_protection == 0)
						{
							ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
							rec_com();       				// communication with panel 
							ui_borad_reentrant_protection = 0;//其他地方又可以使用了
						}  
					}
					delay_ms(80);
					
				//if(k03 == 0)
				if(k03 == TENSION_TYPE_MECHANICAL)
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
#endif

#if 0
	if(smotor_speed != 0)
	{
		if(motor.stop_flag == 1)
		{
			inpress_down(inpress_high_hole);
			if(smotor_speed > MAXSPEED1 || smotor_speed < 2)
			{
				smotor_speed = 2; 
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
		    		//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();       				// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}    						                
		    	}  	 
				while(motor.angle_adjusted < 16)
	    		{
	    			//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();       				// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}  						                
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
				//2019-3-24 新增面板响应函数rec_com()的防重入机制
				if(ui_borad_reentrant_protection == 0)
				{
					ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
					rec_com();       				// communication with panel 
					ui_borad_reentrant_protection = 0;//其他地方又可以使用了
				}  
			}
			delay_ms(80);
		  //if(k03 == 0)   
		  if(k03 == TENSION_TYPE_MECHANICAL)
			{   
			    da0 = 0;	           
			    SNT_H = 0;             
			 }
			 //2019-3-9 暂停后不抬起中压脚
			//inpress_up();
		}	
	}		
	#endif
	if(StatusChangeLatch != WIND) 
	{
		if(motor.spd_obj > 0)
		{
			PW_SET_STOP_CODE(39);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
			sewing_stop();
			while(motor.stop_flag == 0)
			{
				//2019-3-24 新增面板响应函数rec_com()的防重入机制
				if(ui_borad_reentrant_protection == 0)
				{
					ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
					rec_com();       				// communication with panel 
					ui_borad_reentrant_protection = 0;//其他地方又可以使用了
				}  
			}
			delay_ms(80);
			
			//if(k03 == 0)
			if(k03 == TENSION_TYPE_MECHANICAL)
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
	UINT8 i;
	UINT8 back_stitchs;
	INT16 temp16;
	static UINT8 flag=0;

	PW_SET_FUNC_ID(6);//2020-6-6 zla 记录当前所在函数位置
	//--------------------------------------------------------------------------------------
  	// presser down
  	//-------------------------------------------------------------------------------------- 	
	if( foot_flag == 1)
	{		 
	  	footer_both_down();     
	}

    //if((k03 == 0) && (fk_status == OUT) && (fk_cut == 0))
    if((k03 == TENSION_TYPE_MECHANICAL) && (fk_status == OUT) && (fk_cut == 0))
	{
		da0 = 255;
		fk_count = 1;
		fk_status = IN;
		delay_ms(100);  
		da0 = release_tension_current;      
	}


	//2018-11-14
	//针对中捷新增的断线后用户在中压脚降下界面穿线后，需要主轴转动一圈把线勾下去，然后
	//向前回退指定针数
	if(cut_test_flag == 1)
	{
	//如果是拥有系统参数的机型
	//#if ENABLE_CONFIG_PARA || (TASC_PLATFORM_CONFIG_PARA_ENABLE == 1)
	#if ENABLE_CONFIG_PARA || (TASC_PLATFORM_CONFIG_PARA_ENABLE == 1)
		#if 1
		if(   (thread_break_backward_switch==THREAD_BREAK_BACKWARD_SWITCH_OPEN_CANT_CROSS_NOPMOVE || thread_break_backward_switch==THREAD_BREAK_BACKWARD_SWITCH_OPEN_CAN_CROSS_NOPMOVE)
			&& thread_break_backward_stitchs>0
			&& thread_break_backward_stitchs<50
			&& check_move_code_position(pat_point)>=0//保证是在线迹位置
			&& nop_move_pause_flag == 0)//保证不在空送急停时
		{
			//主轴转一圈并回退
			if(thread_break_backward_switch==THREAD_BREAK_BACKWARD_SWITCH_OPEN_CANT_CROSS_NOPMOVE)
			{
				backwords_after_thread_break(0,thread_break_backward_stitchs);//回退不跨越空送
			}
			else
			{
				backwords_after_thread_break(1,thread_break_backward_stitchs);//回退跨越空送
			}
			flag=1;
		}
		else
		{
			//不响应
		}
		#else
		if(   (para.thread_break_backward_switch==55 || para.thread_break_backward_switch==56)
			&& para.thread_break_backward_stitchs>0
			&& para.thread_break_backward_stitchs<50
			&& check_move_code_position(pat_point)>=0//保证是在线迹位置
			&& nop_move_pause_flag == 0)//保证不在空送急停时
		{
			//主轴转一圈并回退
			if(para.thread_break_backward_switch==55)
			{
				backwords_after_thread_break(0,para.thread_break_backward_stitchs);//回退不跨越空送
			}
			else
			{
				backwords_after_thread_break(1,para.thread_break_backward_stitchs);//回退跨越空送
			}
			flag=1;
		}
		else
		{
			//不响应
		}
		#endif
	#else//其他机型，如小模板机(机型18)等等
		if(u73!=0)//断线检测针数大于0
		{
			backwords_after_thread_break(0,u73);//回退不跨越空送
			flag=1;
		}
		else
		{
			//不响应
		}
	#endif
		
		cut_test_flag = 0;
		predit_shift = 0;//告知面板已经回退完成
	}

	//2020-8-21 zla 中压脚界面支持报错，用于避免退出此界面时进入错误状态面板理解错误
	if (PAUSE == pause_active_level)
	{
		delay_ms(10);
		if (PAUSE == pause_active_level)
		{
			PW_SET_CODE_ID(2018);//2020-6-6 zla 记录当前所在代码段位置
			status_now = sys.status;
//			sys.status = ERROR;
//			StatusChangeLatch = ERROR;
//			sys.error = ERROR_19;
			SET_SYS_ERROR(ERROR_19);//急停开关未在正常位置
			//2019-10-23 zla 避免进入连续试缝还动作一次，这样因为已经进入ERROR状态，会导致跑位情况的出现
			predit_shift = 0;
			single_flag = 0;
		}
	}

	//--------------------------------------------------------------------------------------
  	// inpresser down
  	//-------------------------------------------------------------------------------------- 	
	if( inpress_act_flag == 1)
	{
		if( inpress_type != AIR_INPRESS )
		{
				//如果中压脚已经降下（断线后回退），就没必要再次降下了
				if(flag == 0)
				{
					#if SECOND_GENERATION_PLATFORM
					 L_AIR = 1;
					 //delay_ms(para_y_backward_dis);
					 delay_ms(200);//借用了k205	 
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
					flag = 0;
				}
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
	
	//predit_shift = 0;
	if(StatusChangeLatch != INPRESS)
	{
		predit_shift = 0;
		sys.status = StatusChangeLatch;
		
		//if(StatusChangeLatch == READY && k03 == 0)
		if(StatusChangeLatch == READY && k03 == TENSION_TYPE_MECHANICAL)
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
	PW_SET_FUNC_ID(7);//2020-6-6 zla 记录当前所在函数位置
	
  	#if SECOND_GENERATION_PLATFORM
  	#else
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
	PW_SET_FUNC_ID(8);//2020-6-6 zla 记录当前所在函数位置
	
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
						               }	    	    
						              else
						               {	
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
					{
					   //SewTestStitchCounter++;
					}
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
//				sys.error = ERROR_15;
				SET_SYS_ERROR_1(ERROR_15);//超出缝制范围
				StatusChangeLatch = ERROR;
			}
			else
			{	
				//SewTestStitchCounter++;
				allx_step = allx_step - xstep_cou;
  				ally_step = ally_step - ystep_cou;
				if(nopmove_flag == 1)
				{	
					//if(u103 != 0 && inpress_flag == 0)   
					if(u103 != SEWING_TEST_FOOTER_STATUS_UP && inpress_flag == 0) 
					{
						inpress_up();        	
					}
				}
				if(move_flag == 1)                                 
				{
					//if(u103 == 2)
					if(u103 == SEWING_TEST_FOOTER_STATUS_DOWN)
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
			//SewTestStitchCounter--;	
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
				
				//if(u103 == 2) 
				if(u103 == SEWING_TEST_FOOTER_STATUS_DOWN)
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
  		//if(u103 != 0)
  		if(u103 != SEWING_TEST_FOOTER_STATUS_UP)
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
		
		//if(StatusChangeLatch == READY && k03 == 0)
		if(StatusChangeLatch == READY && k03 == TENSION_TYPE_MECHANICAL)
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
	PW_SET_FUNC_ID(9);//2020-6-6 zla 记录当前所在函数位置
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
	PW_SET_FUNC_ID(10);//2020-6-6 zla 记录当前所在函数位置
    return_from_setout = 0;
	bar_coder_refresh_flag = 0;
	over_counter = 0;
    inpress_high = inpress_high_hole;
    tension = tension_hole;

	find_x_origin_counter++;
	already_find_startpoint_flag = 0;
	
	#if 0
	if( k115 == 0)
		already_auto_find_start_point = 0;
	#endif
		
	if(finish_nopmove_pause_flag ==1)
	{
//		sys.status = ERROR;
//		StatusChangeLatch = ERROR;
//	    sys.error = ERROR_02; 
		SET_SYS_ERROR(ERROR_02);//急停
		status_now = READY;
		return;
	}
//#if CHANGE_DOUBLE_FRAMEWORK 	
#if 0
		  if( current_running_flag == 1)//左边运行着
		   {
			   return_frame_back(1);//左边送回模板
			   left_quest_running = 0;
		   }
		   else if( current_running_flag == 2)
		   {
			   return_frame_back(2);//右边送回模板
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
	
	//if(u39 == 1)//查找原点			
	if(u39 == AFTER_SEWING_FINISH_GO_ORIGIN_OPEN)//查找原点		
	{	
	  		switch(u37)  										
	  		{
	  			//case 0: 
	  			case AFTER_SEWING_FINISH_GO_ORIGIN_THEN_RAISE_FOOTER:
					    if( super_pattern_flag == 0)
						   //go_origin_xy();	
						   go_origin_xy(k92);//2019-7-26 修改后的找原点方法
	                	pat_point = (PATTERN_DATA *)(pat_buf);
				  		sta_point = pat_point;
						pat_buff_total_counter = 0;
					break;
	              
	  			//case 1: 
	  			case AFTER_SEWING_FINISH_RAISE_FOOTER_THEN_GO_ORIGIN:
						if(super_pattern_flag == 0)
						   //go_origin_xy();	
						   go_origin_xy(k92);//2019-7-26 修改后的找原点方法
						pat_point = (PATTERN_DATA *)(pat_buf);
				   		sta_point = pat_point;
	               	 	pat_buff_total_counter = 0;
						predit_shift = 0;	
		              	break;
	  			//case 2:
	  			case AFTER_SEWING_FINISH_RAISE_FOOTER_BY_DVB: FootUpCom = 1;
	  			//2020-4-7 zla 新增一个模式：回原点后踩踏板抬起（不抬起可以再缝制）
	  			case AFTER_SEWING_FINISH_NOT_RAISE_FOOTER:
				if(super_pattern_flag == 0)
					//go_origin_xy();	
					go_origin_xy(k92);//2019-7-26 修改后的找原点方法
            	pat_buff_total_counter = 0;
				pat_point = (PATTERN_DATA *)(pat_buf);
		   		sta_point = pat_point;
				predit_shift = 0;
				//FootUpCom = 1;
                	break;
	  			
	  			default: break;		
	  			}
	 	}
	 	else	//如果不查找原点的话
	 	{
	  			switch(u37)
	  			{
	  				//case 0:
	  				case AFTER_SEWING_FINISH_GO_ORIGIN_THEN_RAISE_FOOTER:
						go_setoutpoint();
	              		break;
	  				//case 1:
	  				case AFTER_SEWING_FINISH_RAISE_FOOTER_THEN_GO_ORIGIN:
						delay_ms(10);
	              		go_setoutpoint();
						
	              		break; 			
	  				//case 2: 
	  				case AFTER_SEWING_FINISH_RAISE_FOOTER_BY_DVB: FootUpCom = 1;
	  				//2020-4-7 zla 新增一个模式：回原点后踩踏板抬起（不抬起可以再缝制）
	  				case AFTER_SEWING_FINISH_NOT_RAISE_FOOTER:
						go_setoutpoint();
						//FootUpCom = 1;
                		break;
	  			
	  				default: break;		
	  		}
	  }
 
  	#if 0	
	if(k110 == 1)         
	{
		if(R_AIR == 1)
		{
		   R_AIR = 0;
		   delay_ms(80);  
		}
	}
	#endif
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
	
	if(sys.status == ERROR)
	{
		
	}	
	else
	{
	#if DISABLE_THREAD_CLAMP_FUNCTION
	#else
        //if(u35==0)      
        if(u35 == THREAD_SCRATCH_SWITCH_OPEN)
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
//      					sys.status = ERROR;
//						StatusChangeLatch = ERROR;
//						if( sys.error == 0)
//        				sys.error = ERROR_23;	 // clamp is not in normal position  		  
						SET_SYS_ERROR(ERROR_23);//抓线位置异常
        				//_speed_array[0] = 9;
        				return;
        			}
      			}
        	}
        	else
        	{
        		if(clamp_flag == 1)        // clamp is out
        		{
//          			sys.status = ERROR;
//					StatusChangeLatch = ERROR;
//					if( sys.error == 0)
//          			sys.error = ERROR_23;	   // clamp is not in normal position	 
          			//_speed_array[0] = 10;
          			SET_SYS_ERROR(ERROR_23);//抓线位置异常
          			return;
        		}
        		else
        		{
          			clamp_stepflag = 0;      // clamp thread step 0
        		}
        	}
        } 
	#endif
		StatusChangeLatch = READY;
	  	sys.status = READY;   
	}
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
	
	PW_SET_FUNC_ID(11);//2020-6-6 zla 记录当前所在函数位置
	if(OutOfRange_flag == 0)
	{
		//if(u97 == 1)//急停时手动剪线
		if(u97 == TRIM_METHOD_AFTER_PAUSE_MANUAL)//急停后手动剪线
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
		//else if(u97 == 0)//急停时自动剪线
		else if(u97 == TRIM_METHOD_AFTER_PAUSE_AUTO)//急停后自动剪线
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
	UINT16 tmp_SewTestStitchCounter;
	UINT16 tmp_pat_buff_total_counter;

	PW_SET_FUNC_ID(12);//2020-6-6 zla 记录当前所在函数位置
	
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
	  	//只在使能了自动送料功能和多功能IO才有效
		#if MULTIPULE_IO_ENABLE && AUTOMATIC_FEEDING_ENABLE
		if(para8.af_enable_switch == 55)
		{
			if(af_info.frame_position_flag == 1)//XY框架在停车位置
			{
				go_commandpoint(0, 0);//从停车位置回到真正的原点
				af_info.frame_position_flag = 0;
			}
		}
		#endif
		
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
				if(already_auto_find_start_point == 0)//第一次开始缝制
			  	{
					//find_start_point();
					already_auto_find_start_point = 1;
					/*原为120ms，防止还没有报错就进入下一阶段，在以后的优化的过程中，
					 *应考虑怎么在故障处理中避免该问题。
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
					//else if( (second_point_passby_flag == 0)&&(origin2_lastmove_flag == 1) )
					else if( (second_point_passby_flag == GO_SECOND_POINT_METHOD_LINE)&&(origin2_lastmove_flag == 1) )
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
					tmp_SewTestStitchCounter = SewTestStitchCounter;
					tmp_pat_buff_total_counter = pat_buff_total_counter;
					
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
					SewTestStitchCounter = tmp_SewTestStitchCounter;
					pat_buff_total_counter = tmp_pat_buff_total_counter;
					go_commandpoint(allx_step_temp,ally_step_temp);
					
//					sys.error = ERROR_15;
					SET_SYS_ERROR_1(ERROR_15);//超出缝制范围
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

	//3.0 响应界面上的剪线按钮功能，原地动轴剪线
	if(cut_test_flag == 1)
	{
			#if DISABLE_THREAD_CLAMP_FUNCTION
			#else
			if( (u35 == THREAD_SCRATCH_SWITCH_OPEN)&&(clamp_flag== 1) )  //打开抓线并且伸出
			{
				go_origin_ct();	
			}
			#endif
			cut_test_flag = 0;
			/*
			剪线动作之前，要确认一下是否进行过空送急停，防止停在模板中间，剪线机针下扎会断针。
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

				//2019-1-21 首先检查针杆位置，框架动作之前把针先提起来，防止撞针
				temp8 = detect_position();	
				if(temp8 == OUT)    								
				{
					find_dead_center();
				}
	
			 	trim_action();
			 	//2019-6-11 衔接效率提升开关(剪线+空送+起缝)
				#if BRIDGING_EFFICIENCY_IMPROVEMENT && ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
				if(trim_motor_need_find_origin_flag == 1)
				{
					while( cutter_delay_flag == 1)
					{
						delay_ms(1);
					}
					go_origin_trim();//等待标缝指令到原点后再找原点
				}
				#endif
				
				delay_ms(100);
				
				inpress_up();
				delay_ms(100);
			}
			predit_shift = 0;
			/*
			如果开抓线功能，剪线完成后抓线机构还要先伸出去
			*/
			#if DISABLE_THREAD_CLAMP_FUNCTION
			#else
			if( (clamp_flag == 0)&&(u35 == THREAD_SCRATCH_SWITCH_OPEN))
    		{
    			go_origin_ct();
				delay_ms(20);
    			clamp_out();
    			clamp_stepflag = 1;
          		movect_angle = 800;
    		}
    		#endif
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
			if(AUTO_SEARCH_START_POINT == 1)	//没找过原点，则执行特殊的找原点处理
			{
				if (already_auto_find_start_point == 0)
				{
					//find_start_point();
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
			//2019-4-25 没必要找传感器了
			#if 0
			if(AUTO_SEARCH_START_POINT == 1)//#if AUTO_SEARCH_START_POINT  
			{
				//if( already_auto_find_start_point == 0 )
				{
					delay_ms(100);
					find_start_point();	
					already_auto_find_start_point = 1;
				}
			}
			#endif
			//#endif
			pat_point = (PATTERN_DATA *)(pat_buf);
			#if 0
			if(k110 == 1)
			{
				R_AIR = 0;
				delay_ms(80);  
			}
			#endif
		
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
				PW_SET_FUNC_ID(12);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
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
			//course_edit_continue_back();  
			course_back();
			delay_ms(1);    
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
	INT32 allx_step_tmp,ally_step_tmp;

	PW_SET_FUNC_ID(13);//2020-6-6 zla 记录当前所在函数位置
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
  	#if 1
  	switch(single_flag)
  	{
   		case 1:	 //5.1  车缝单步向前的处理
			 
			SewingTestEndFlag = 1;  
			//5.1.1 移动前提升针杆
			temp8 = detect_position();	
    		if(temp8 == OUT)    
      		{
				find_dead_center();
      		}
			
			allx_step = allx_step + xstep_cou;//5.1.2 判定单步操作后是否出现超出缝制范围的情况
  			ally_step = ally_step + ystep_cou;
			
			if( check_sewing_range())//5.1.2.1 超范围的处理
			{
				allx_step = allx_step - xstep_cou;
  				ally_step = ally_step - ystep_cou;
				do_pat_point_sub_one();
				single_flag = 0;
				move_flag = 0;
				nopmove_flag = 0;
//				sys.error = ERROR_15;
				SET_SYS_ERROR_1(ERROR_15);//超出缝制范围
				StatusChangeLatch = ERROR;
				set_func_code_info(READY,3,0,0);
			}
			else
			{	
				//5.1.2.1 没超范围情况,先恢复坐标
				allx_step = allx_step - xstep_cou;
  				ally_step = ally_step - ystep_cou;
				//
				if(nop_move_pause_flag ==1)
				{
		    		//5.1.2.2 如果之前发生了空送过程中按急停的情况，这时再向前就要先把空送走完
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
						//2018-11-19 新增旋转切刀宏定义，用于减少代码量
					#if ENABLE_ROTATED_CUTTER==1
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
					#endif
						
					}
					
				#if ENABLE_LOOPER_CUTTER
						if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
						{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 0) )//偏移不为0
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
					
					//SewTestStitchCounter++;
					if( (FootRotateFlag == 1) &&(marking_flag == 1) )
					{
						process_marking_pen(0);
						PW_SET_FUNC_ID(13);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
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
			
  		case 2://单步后退试缝
		       
			SewingTestEndFlag = 1;  
			//SewTestStitchCounter--;		
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
				//2018-11-19 新增旋转切刀宏定义，用于减少代码量
			#if ENABLE_ROTATED_CUTTER==1
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
			#endif
			#if ENABLE_LOOPER_CUTTER
						if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
						{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 1) )//偏移不为0
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
						PW_SET_FUNC_ID(13);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
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
				if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 1) )//偏移不为0
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
		//2018-11-19 新增旋转切刀宏定义，用于减少代码量
	#if ENABLE_ROTATED_CUTTER==1
		if(ROTATED_CUTTER == 1)
		{
		  if( rotated_cutter_enable ==1 )
		   {
		   	  //DRILL_MOTOR_UPDOWN = 0;//确认刀下去了
			  rotated_cutter_down_positon();
			   if ( milling_first_move ==1)
			   {		
					if( (x_bios_offset!=0)||(y_bios_offset!=0) )//偏移不为0
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
	#endif
	#if ENABLE_LASER_CUTTER
					if( milling_cutter_action_flag != 0)
					{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 1) )//偏移不为0
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
			//2018-11-19 新增旋转切刀宏定义，用于减少代码量
		#if ENABLE_ROTATED_CUTTER==1
			if(ROTATED_CUTTER == 1)
			{
				if ( (milling_cutter_action_flag ==1)&&(rotated_cutter_enable ==1) )
				     rotated_cutter_single_stop();
			}
		#endif

			//只在使能了自动送料功能和多功能IO才有效
		#if MULTIPULE_IO_ENABLE && AUTOMATIC_FEEDING_ENABLE
			//2019-1-15 
			if(af_info.frame_position_flag == 1)//如果XY框架在停车位置
			{
				//禁止在偏移时出现急停，此时不响应急停
				shift_no_pause=1;
				go_commandpoint(0, 0);//先回0位
				shift_no_pause=0;//恢复对急停的响应
				af_info.frame_position_flag = 0;
			}
		#endif
					
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
			//2018-11-19 新增旋转切刀宏定义，用于减少代码量
		#if ENABLE_ROTATED_CUTTER==1
			if(ROTATED_CUTTER == 1)
			{
				if ( (milling_cutter_action_flag ==1)&&(rotated_cutter_enable ==1) )
				     rotated_cutter_single_stop();
			}
		#endif
			if(nop_move_pause_flag ==1)
		    	process_nop_move_pause(2);
			else
			{
			 	back_endpoint();
			}
			if( (FootRotateFlag == 1) &&(marking_flag == 1) )
			{
				process_marking_pen(1);
				PW_SET_FUNC_ID(13);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
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
				 PW_SET_FUNC_ID(13);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
				 marking_flag = 0;
				 FootRotateFlag = 0;
				 delay_ms(50);
			}
		#if ENABLE_LOOPER_CUTTER
						if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
						{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 0) )//偏移不为0
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
				PW_SET_FUNC_ID(13);//2020-6-6 zla 记录当前所在函数位置，恢复当前位置，以为已经从另一个状态返回
				marking_flag = 0;
				FootRotateFlag = 0;
				delay_ms(50);
			}
			delay_ms(2); 
		#if ENABLE_LOOPER_CUTTER
						if ( (milling_cutter_action_flag ==1)&&(stepper_cutter_enable==1) )
						{
							if( ((x_bios_offset!=0)||(y_bios_offset!=0))&& (milling_first_move == 1) )//偏移不为0
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

  	#else
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
  	#endif
  	
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
	PW_SET_FUNC_ID(14);//2020-6-6 zla 记录当前所在函数位置
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
	INT8 ret;
	
	PW_SET_FUNC_ID(15);//2020-6-6 zla 记录当前所在函数位置
	PW_SET_CODE_ID(1000);//2020-6-6 zla 记录当前所在代码段位置
//	#if CHANGE_DOUBLE_FRAMEWORK 
#if 0

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
		if(coor_com ==1)//等待面板的位置跳转指令
		{
			//2020-12-15 zla 增加输出功能码
			#if ENABLE_OUTPUT_FUNC == 1
			//2020-12-1 zla 输出码清除
			set_output_code( 0, 0 );
			#endif
			
			//if( (u37 == 1)&&(u38 == 0) )
			if( (u37 == AFTER_SEWING_FINISH_RAISE_FOOTER_THEN_GO_ORIGIN)&&(u38 == 0) )
			{
				footer_both_up();
			}
			//if(u39 == 1)	
			if(u39 == AFTER_SEWING_FINISH_GO_ORIGIN_OPEN)
			{
				/*SUM = 1;
				_speed_array[0] = opl_origin_flag;
				delay_ms(2000);
				SUM = 0;*/
				//go_origin_xy();//缝制后框架回原点
				PW_SET_CODE_ID(1001);//2020-6-6 zla 记录当前所在代码段位置
				go_origin_xy(k92);//2019-7-26 修改后的找原点方法
				PW_SET_CODE_ID(1002);//2020-6-6 zla 记录当前所在代码段位置

			//只在使能了自动送料功能和多功能IO才有效
			#if MULTIPULE_IO_ENABLE && AUTOMATIC_FEEDING_ENABLE
				if(para8.af_enable_switch == 55)
				{
					ret = af_ejecting();
					if(ret != 0)//出料失败，报错
					{
//						sys.status = ERROR;
//						StatusChangeLatch = ERROR;
//						if( sys.error == 0)
//				  		sys.error = ERROR_111;//缝制后出料失败
						SET_SYS_ERROR(ERROR_111);//缝制后出料失败
				  		return;//直接返回
					}
					//送料完成后，直接判断送料位置是否有放置好的模板
					rc522_write_falg = 0;
					RFID_SCAN();//读取RFID编号
					af_info.continue_sewing_flag = 0;
					if(serail_number != 0)
					{
						//说明取料位置已经读到了模板，这个时候可以直接开启下一轮的自动缝制
						af_info.continue_sewing_flag = 1;
						
						
					}
				}
				
			#endif
			
			}
			if(finish_nopmove_pause_flag ==0)
			{ 
				//if( u39 !=1)
				if(u39 != AFTER_SEWING_FINISH_GO_ORIGIN_OPEN)
				{
					if(inpress_flag == 0)
					{
						inpress_up();
						delay_ms(20);
					}
					PW_SET_CODE_ID(1003);//2020-6-6 zla 记录当前所在代码段位置
		     		go_commandpoint(comx_step,comy_step);
		     		PW_SET_CODE_ID(1004);//2020-6-6 zla 记录当前所在代码段位置
				}
				//if( (u37 == 0)&&(u38 == 0) )
				if( (u37 == AFTER_SEWING_FINISH_GO_ORIGIN_THEN_RAISE_FOOTER)&&(u38 == 0) )
				{
		            footer_both_up(); 
				}
				//else if (u37 == 2)
				else if (u37 == AFTER_SEWING_FINISH_RAISE_FOOTER_BY_DVB)
				   FootUpCom = 1;
				//if(u39 == 1)		
				if(u39 == AFTER_SEWING_FINISH_GO_ORIGIN_OPEN)
		  	    {
		  	    	PW_SET_CODE_ID(1005);//2020-6-6 zla 记录当前所在代码段位置
				   go_origin_zx();			
				   PW_SET_CODE_ID(1006);//2020-6-6 zla 记录当前所在代码段位置
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
		PW_SET_CODE_ID(1007);//2020-6-6 zla 记录当前所在代码段位置
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
	if ( ready_go_setout_com == 1)
	{
		PW_SET_CODE_ID(1008);//2020-6-6 zla 记录当前所在代码段位置
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
	PW_SET_FUNC_ID(16);//2020-6-6 zla 记录当前所在函数位置
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

	PW_SET_FUNC_ID(17);//2020-6-6 zla 记录当前所在函数位置
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

	PW_SET_FUNC_ID(18);//2020-6-6 zla 记录当前所在函数位置
	//--------------------------------------------------------------------------------------
  	//  inpresser down
  	//--------------------------------------------------------------------------------------
	inpress_down(inpress_high);                                
	 trim_action();      
	 //2019-6-11 衔接效率提升开关(剪线+空送+起缝)
	#if BRIDGING_EFFICIENCY_IMPROVEMENT && ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
	if(trim_motor_need_find_origin_flag == 1)
	{
		while( cutter_delay_flag == 1)
		{
			delay_ms(1);
		}
		go_origin_trim();//等待标缝指令到原点后再找原点
	}
	#endif
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

	PW_SET_FUNC_ID(19);//2020-6-6 zla 记录当前所在函数位置
	
	#if INSTALLMENT	
	if( main_control_lock_setup == 1)
	{
		main_control_lock_setup = 0;
		write_par(0,main_control_lock_flag);  		
		write_par(2,remote_control_lock_flag); 
		if( (read_par(0) == main_control_lock_flag) && (read_par(2) == remote_control_lock_flag) )
			set_control_lock_return(0);
		else
			set_control_lock_return(1); 
	}
	#endif	
	
	//2020-8-21 zla 中压脚界面支持报错，用于避免退出此界面时进入错误状态面板理解错误
	if (PAUSE == pause_active_level)
	{
		delay_ms(10);
		if (PAUSE == pause_active_level)
		{
			PW_SET_CODE_ID(2018);//2020-6-6 zla 记录当前所在代码段位置
			status_now = sys.status;
//			sys.status = ERROR;
//			StatusChangeLatch = ERROR;
//			sys.error = ERROR_19;
			SET_SYS_ERROR(ERROR_19);//急停开关未在正常位置
			//2019-10-23 zla 避免进入连续试缝还动作一次，这样因为已经进入ERROR状态，会导致跑位情况的出现
			predit_shift = 0;
			single_flag = 0;
		}
	}
	
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
	UINT8 temp8;

	PW_SET_FUNC_ID(20);//2020-6-6 zla 记录当前所在函数位置
	//--------------------------------------------------------------------------------------
  	// config main motor driver
  	//-------------------------------------------------------------------------------------- 
  	initial_mainmotor();
	
//新增检测模式下手轮转动时中压脚随动功能，注意，仅电机随动中压脚有效
#if INPRESS_FOLLOW_MAIN_MOTOR_ENABLE

	//第一次进入的内容
	if(first_enter_checki03_flag == 0)
	{
		first_enter_checki03_flag = 1;
		//if(inpress_flag == 1)
		{
		   //SUM = 1;
		   footer_both_down();
		   go_origin_zx();
		   delay_ms(500);
		   inpress_down(inpress_high_hole);
		   delay_ms(500);
		   //SUM = 0;
		}
		refresh_para_flag = 1;//告知函数follow_inpress_task()需要根据中压脚时序参数计算位置信息
	}
	
	_speed_array[9] = follow_inpress_task();//处理中压脚随动主轴功能，返回负值表示参数不合适
	
#else//正常情况下，这里和之前是一样的

	if(inpress_flag == 1)
	{
	   inpress_down(inpress_high_hole);
	   delay_ms(500);
	}
	
#endif


	
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
		//2019-3-4 退出主轴矫正后，主轴电机不回最高点，尝试修改一下
		temp8 = detect_position();	
		if(temp8 == OUT)    								
		{
			find_dead_center();
		}
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
void checki1f_status(void)
{	
	UINT8 temp8;
	INT8 x_buf[50],y_buf[50];
	UINT8 speed_tab[50];
	UINT8 i,j;

	PW_SET_FUNC_ID(31);//2020-6-6 zla 记录当前所在函数位置
	
	//--------------------------------------------------------------------------------------
  	// config main motor driver
  	//-------------------------------------------------------------------------------------- 
  	initial_mainmotor();

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

//	if(DVB == 0)
//  	{
//  		delay_ms(10);
//  		if(DVB == 0)
//  		{
//  			go_commandpoint(0, 0);
//  			for(i=0; i<50; i++)
//  			{
//  				x_buf[i] = -100;
//  				y_buf[i] = 50;
//  				speed_tab[i]=250;
//  				allx_step -= x_buf[i];
//  				ally_step -= y_buf[i];
//  			}
//  			delay_ms(200);
//  			buzzer_control_time = 500;
//  			movestep_xy3(x_buf,y_buf,50,speed_tab);
//  			delay_ms(2000);
//  		}
//  		
//  		while(DVB==0);//死等DVB抬起
//  	}
//  	if(DVA == 0)
//  	{
//  		delay_ms(10);
//  		if(DVA == 0)
//  		{
//  			while(DVA==0);//死等DVA抬起
//			go_commandpoint(0, 0);
//			for(i=0; i<50; i++)
//			{
//				x_buf[i] = -100;
//				y_buf[i] = 50;
//				speed_tab[i]=250;
//				allx_step -= x_buf[i];
//  				ally_step -= y_buf[i];
//			}
//			delay_ms(200);
//			buzzer_control_time = 500;
//			movestep_xy3(x_buf,y_buf,50,speed_tab);
//			delay_ms(2000);
//			send_dsp_command(DSP1,0x0005);//X轴急停
//			for(j=0;j<200;j++)			  //等待动作结束 
//			{
//				delay_ms(1);
//				if( check_motion_done(3) )
//				   break;
//			}
//			while(DVA==1)//死等DVA按下
//			{
//				delay_ms(1);
//			}
//			send_dsp1_command(0x0014,0);//继续运行
//			delay_ms(2000);

//  		}
//  		
//  		while(DVA==0);//死等DVA抬起
//  	}
//  	
	if(StatusChangeLatch != CHECK_INPUT) 
	{
		//2019-3-4 退出主轴校正后，主轴电机不回最高点，尝试修改一下
		temp8 = detect_position();	
		if(temp8 == OUT)    								
		{
			find_dead_center();
		}
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

	PW_SET_FUNC_ID(21);//2020-6-6 zla 记录当前所在函数位置
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
		//2019-3-9 进入CHECKI04降下辅助压脚，然后降下中压脚
		//
		//2019-7-19 zla 新增中压脚第一次动作标志，开机时为1，如果没找原点就需要
		//抬起或者降下中压脚，就需要先找一下原点才行
		if( inpress_first_move_flag == 1 )//此标志将在go_origin_zx()中清0
		{
			go_origin_zx();
		}
		
		FA = 1;
		delay_ms(300);
		inpress_down(inpress_high_hole);//降下中压脚
		temp8 = detect_position();//主轴回原点
		if(temp8 == OUT)   
  		{
			find_dead_center();
  		}	
		if(sys.status == ERROR)
    	{
      		return;
    	}
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
				if(smotor_speed > MAXSPEED1 || smotor_speed < 2)
				{
					smotor_speed = 2; 
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

					//2020-8-21 zla 升速过程中，按下了急停，那么此时会给定速度为0，此时需要等待停车完成并退出
					//否则会卡死在后续的角度等待中
					if( motor.spd_obj == 0 )
					{
						while(motor.stop_flag == 0)
						{
							//2019-3-24 新增面板响应函数rec_com()的防重入机制
							if(ui_borad_reentrant_protection == 0)
							{
								ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
								rec_com();       				// communication with panel 
								ui_borad_reentrant_protection = 0;//其他地方又可以使用了
							}  
						}
						break;
					}
					
					while(motor.angle_adjusted >= 16)
			    	{
			    		//2019-3-24 新增面板响应函数rec_com()的防重入机制
						if(ui_borad_reentrant_protection == 0)
						{
							ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
							rec_com();       				// communication with panel 
							ui_borad_reentrant_protection = 0;//其他地方又可以使用了
						}    						                
			    	}  	 
					while(motor.angle_adjusted < 16)
		    		{
		    			//2019-3-24 新增面板响应函数rec_com()的防重入机制
						if(ui_borad_reentrant_protection == 0)
						{
							ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
							rec_com();       				// communication with panel 
							ui_borad_reentrant_protection = 0;//其他地方又可以使用了
						}  						                
		    		} 
				}
				motor.spd_obj = smotor_speed * 100;				
			}	
		}	
		else
		{
			if(motor.stop_flag == 0) 
			{
				PW_SET_STOP_CODE(23);//2020-6-9 zla 记录最新主轴转速从非0设置为0的位置
				sewing_stop();		
				while(motor.stop_flag == 0)
				{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();       				// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}  
				}
				delay_ms(80);
			  //if(k03 == 0)   
			  if(k03 == TENSION_TYPE_MECHANICAL)
				{   
				    da0 = 0;	           
				    SNT_H = 0;             
				 }
				 //2019-3-9 暂停后不抬起中压脚
				//inpress_up();
			}	
		}	
	}	

	if(StatusChangeLatch != CHECKI04) 
	{
		inpress_up();//退出时抬起中压脚
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
	
	PW_SET_FUNC_ID(22);//2020-6-6 zla 记录当前所在函数位置
	initial_mainmotor();
	if(sys.status == ERROR)
  	{
    	return;
  	}

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
    	case 1: 								//拨线
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
			#if SECOND_GENERATION_PLATFORM
				//2018-10-26 增加小夹线器
				#if USE_SMALL_YARN_TRAPPER
				FW = 1;//小夹线器
				#endif
				
				FK_OFF = 1;//拨线气缸p5_4
				delay_ms(150);
				FK_OFF = 0;
				
				//2018-10-26 增加小夹线器
				#if USE_SMALL_YARN_TRAPPER
				FW = 0;//小夹线器
				#endif
				
			#else
		    FW = 1;//小夹线器
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
    	case 2: 					//气剪线
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
			
	        break;

	    case 3: 					//电剪线
		   	SNT_H = 1;      
			#if SECOND_GENERATION_PLATFORM 
			if( cutter_output_test == 0)
			{
				cutter_output_test = 1;
			}
			else
			{
				cutter_output_test = 0;
			}
			#else 	
			if( cutter_output_test == 0)
			{
				FA = 1;
				cutter_output_test = 1;
			}
			else
			{
	        	FA = 0;
				cutter_output_test = 0;
			}
			#endif
		    SNT_H = 0;        	// 33V to 24V
	        output_com = 0;	
			
	        break;
	  	//--------------------------------------------------------------------------------------      
    	//  foot move
    	//--------------------------------------------------------------------------------------
    	case 4: 								//外压框
			if(foot_flag == 1) 
    	    {
    	    	 foot_down();  
				 #if SECOND_GENERATION_PLATFORM || COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER35 || COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER38
				 #else
				 FL = 1; 
				 #endif
				 //movestep_yj(-100,60);
				 delay_ms(80);
    	   	}
    	    else                    
    	    {
    	    	 foot_up();
				 #if SECOND_GENERATION_PLATFORM ||COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER35|| COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER38
				 #else
				 FL = 0; 
				 #endif
				 //movestep_yj(100,60);
				 delay_ms(80);
    	    }
			//movestep_cs3(0x2000,-1,1);   //逆时针
		    delay_ms(20);
	        output_com = 0;
	        break;
	  	//--------------------------------------------------------------------------------------      
    	//  infoot move
    	//--------------------------------------------------------------------------------------
   	 	case 5: 								//中压脚
			if(inpress_flag == 1) 
    	    {
    	    	//2020-9-28 zla
    	    	inpress_down(inpress_high_hole);//inpress_down(0);
    	    }
    	    else                  
    	    {
    	    	inpress_up();
    	    }    	      
		    delay_ms(200);
	        output_com = 0;
	        break;
		case 6:	//松线
			da0 = 255;
            delay_ms(50);
			da0 = 0;
	        SNT_H = 0;        // 33V to 24V
	        delay_ms(200);
	        output_com = 0;
	        break;
		case 7:					//辅助压脚
			#if SECOND_GENERATION_PLATFORM
			if(L_AIR == 0)
				L_AIR = 1;
			else
				L_AIR = 0;
			#else
			if(FA == 0)
				FA = 1;
			else
				FA = 0;
			#endif
			 output_com = 0;
		break;
	  	//--------------------------------------------------------------------------------------   
		case 8:                     //气阀 1接的外压框配合的小抓钩
			if( check_valve1_flag == 0)
			{
		     	EXTEND = 1;
			 	delay_ms(200);
			 	check_valve1_flag = 1;
				output_cs3(1,1);
			}
			else
			{ 
			 	EXTEND = 0;
			 	check_valve1_flag = 0;
				output_cs3(1,0);
			}
			 delay_ms(200);
			 output_com = 0;
		
		break;   
		case 9:                     //气阀 2
		//2019-3-19 为500模板机新增注油开关为p3_4
		#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER58	
		if( check_valve2_flag == 0)
			{
		     	COOL_AIR = 1;
				check_valve2_flag = 1;
			}
			else
			{
			    COOL_AIR = 0;
			    check_valve2_flag = 0;
			}
		#else
			if( check_valve2_flag == 0)
			{
		     	FR_ON = 1;
				check_valve2_flag = 1;
			}
			else
			{
			    FR_ON = 0;
			    check_valve2_flag = 0;
			}
		#endif
			 delay_ms(200);
			 output_com = 0;
		break;
		case 10:						//气阀 3
			if( check_valve3_flag == 0)
			{
		     	FK_OFF = 1;
				check_valve3_flag = 1;
			}
			else
			{
			 	FK_OFF = 0;
				check_valve3_flag = 0;
			}
			 delay_ms(200);
			 output_com = 0;
		
		break;
		case 11:                     //气阀 4
			if( check_valve4_flag == 0)
			{
		       T_HALF = 1;
			   delay_ms(200);
			   check_valve4_flag = 1;		   
			}
			else
			{
			 	T_HALF = 0;
				delay_ms(200);
				check_valve4_flag = 0;
			}			  
			output_com = 0;
		break;
		case 12:				    //气阀 5
			if( check_valve5_flag == 0)
			{
		     	T_DIR = 1;
			 	delay_ms(200);
				check_valve5_flag = 1;
			}
			else
			{
		     	T_DIR = 0;
				 delay_ms(200);
				 check_valve5_flag = 0;
			}
			 output_com = 0;
		
		break;
		case 13:					//气阀 6
			 if( check_valve6_flag == 0)
			 {
		     	  T_CLK = 1;			 	  
				  check_valve6_flag = 1;
			 }
			 else
			 { 
			 	  T_CLK = 0;
				  check_valve6_flag = 0;
			 }
			 output_com = 0;
		break;
		case 14://翻转压脚
		   //if( (k110 == 1)||(auto_function_flag == 1) )
		   if( (auto_function_flag == TEMPLATE_IDENTIFY_SWITCH_OPEN) )
		    {
			   R_AIR =1;
               delay_ms(500);
			   R_AIR = 0;
	           delay_ms(200);
			}
			#if 0
			else if(k110 == 2)
			{
				stretch_foot_out();
			    delay_ms(500);
				stretch_foot_in();
			 }
			 #endif
			output_com = 0;
			
	        break;

		case 15://辅助气阀
			#if SECOND_GENERATION_PLATFORM
			#else
			if(FL == 0)
				FL = 1;
			else
				FL = 0;
			#endif

	        output_com = 0;
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
	INT32 backup,backup1;

	PW_SET_FUNC_ID(23);//2020-6-6 zla 记录当前所在函数位置
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
		//2019-8-6 增加XY回原点选择
		//1.5 X、Y轴执行找原点
		opl_origin_flag = 0;//强制传感器找原点
		go_origin_xy(k92);
		
		predit_shift = 0;
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
				//go_origin_xy();    							//x and y go origin
				go_origin_xy(k92);//2019-7-26 修改后的找原点方法
		      	//2019-3-24 新增面板响应函数rec_com()的防重入机制
				if(ui_borad_reentrant_protection == 0)
				{
					ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
					rec_com();       				// communication with panel 
					ui_borad_reentrant_protection = 0;//其他地方又可以使用了
				}  
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
				#if 1
				go_origin_xy(k92);//2019-7-26 修改后的找原点方法
				#else
				go_origin_xy();    							//x and y go origin
				backup = x_origin_offset;
				backup1 = y_origin_offset;
				x_origin_offset = 0;
				y_origin_offset = 0;
				find_start_point1();//走到偏移地址
				x_origin_offset = backup;
				y_origin_offset = backup1;
				#endif
				
				while(DVA == 0)
		    	{
		      		//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();       				// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					} 
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
		      		//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();       				// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}  
		    	}
   	    
		  	}
	  	}
	}
  	//--------------------------------------------------------------------------------------
  	//  manual shift step  
  	//--------------------------------------------------------------------------------------
  	//2019-7-29 zla 移框坐标限制开关，用于保证FREE状态下的移框能够自由进行
	shift_disable_range_limit = 1;
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
  	shift_disable_range_limit = 0;
  	
  	#if 0
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
  	#endif
  	
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

	PW_SET_FUNC_ID(24);//2020-6-6 zla 记录当前所在函数位置
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
		
	  	//go_origin_allmotor();	   // all step motor go origin    
	  	if(sys.status == ERROR)
    	{
      		return;
    	} 
    	origin_com = 0;        		
  	}
 	if(pedal_style == double_pedal)
	{
		//--------------------------------------------------------------------------------------
	  	//  start sensor
	  	//--------------------------------------------------------------------------------------  
	  	if(DVA == 0)           								// start sensor is pushed
		{
			delay_ms(10);
			if(DVA == 0)
			{				
			//2019-1-5 为三丝杠模板机（机型：47）新增圆刀电机剪线功能
			#if ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
				go_origin_trim();//剪线电机回原点
				stepmotor_state = 0; 
				delay_ms(400);
			#endif
      			stepmotor_state = 0x00;
      			stepmotor_comm = 0xff;
				while(DVA == 0)
		    	{
		      		//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();       				// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}  
		    	}	 	      	    
		  	}
	  	}
	}
  	//--------------------------------------------------------------------------------------
  	//  manual shift step  
  	//--------------------------------------------------------------------------------------
  	switch(stepmotor_comm)
  	{
  		case 0x00://原点			
		//2019-1-5 为三丝杠模板机（机型：47）新增圆刀电机剪线功能
		#if ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
  			go_origin_trim();//剪线电机回原点
			delay_ms(400);
		#endif
			stepmotor_state = 0;  
			stepmotor_comm = 0xff;
  			break;      	
  			
  	  case 0x01://分线位置
  	  		if(stepmotor_state >= 2)
  	  		{
  	  			//2019-1-5 为三丝杠模板机（机型：47）新增圆刀电机剪线功能
				#if ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
		  			go_origin_trim();//剪线电机回原点
					delay_ms(200);
				#endif
  	  		}
  	  		else
  	  		{
  	  		}
  	  	//2019-1-5 为三丝杠模板机（机型：47）新增圆刀电机剪线功能
		#if ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
			//电机剪线第1步：到达分线位置，分线尖到达针尖位置
			movestep_trim(-(int)trim_para.trim_motor_branch_position, trim_para.trim_motor_branch_time);
			delay_ms(trim_para.trim_motor_branch_time);  
		#endif
			stepmotor_state = 1;  
			stepmotor_comm = 0xff;
			break;   
		
  	  case 0x02://断线位置
		//2019-1-5 为三丝杠模板机（机型：47）新增圆刀电机剪线功能
		#if ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
			//电机剪线第2步：到达断线位置，将线剪断
			if(stepmotor_state==1)
			{
				movestep_trim(-(int)(trim_para.trim_motor_range-trim_para.trim_motor_branch_position), trim_para.trim_motor_break_time);
				delay_ms(trim_para.trim_motor_break_time+40);//多延时了40ms，保证能够将线剪断
			}
			else if(stepmotor_state==0)
			{
				//电机剪线第1步：到达分线位置，分线尖到达针尖位置
				movestep_trim(-(int)trim_para.trim_motor_branch_position, trim_para.trim_motor_branch_time);
				delay_ms(trim_para.trim_motor_branch_time);  
				movestep_trim(-(int)(trim_para.trim_motor_range-trim_para.trim_motor_branch_position), trim_para.trim_motor_break_time);
				delay_ms(trim_para.trim_motor_break_time+40);//多延时了40ms，保证能够将线剪断
			}
			
		#endif
			stepmotor_state = 2;  
			stepmotor_comm = 0xff;
			break;   // cut thread	                       		            							    
/*  	  case 0x04: 
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
*/
  	  default: 
  	    //stepmotor_state = stepmotor_comm;
  	  	stepmotor_comm = 0xff;
			break;	
  	}
  	//--------------------------------------------------------------------------------------
  	//  single shift step 09.3.26 wr add 
  	//--------------------------------------------------------------------------------------
  	switch(stepmotor_single)
  	{
  		case 0x00://cw 反向，点击-号
  		//2019-1-5 为三丝杠模板机（机型：47）新增圆刀电机剪线功能
		#if ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
			movestep_trim(-1,1);
  		  	delay_ms(1);
  		#endif
  			stepmotor_single = 0xff;
  	  		break; 
  		case 0x01://ccw 正向，点击+号
  		//2019-1-5 为三丝杠模板机（机型：47）新增圆刀电机剪线功能
		#if ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
  			movestep_trim(1,1);
  			delay_ms(1);
  		#endif
  			stepmotor_single = 0xff;
  			break;              	     		            									  		            								    
  	  default:
  	  	break;	
  	}
  	
  	//2019-8-6 3.0 响应界面上的剪线按钮功能，原地动轴剪线
	if(cut_test_flag == 1)
	{
			#if ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
  			go_origin_trim();//剪线电机回原点
			delay_ms(200);
  			#endif
  			
			#if DISABLE_THREAD_CLAMP_FUNCTION
			#else
			//if( (u35==0)&&(clamp_flag== 1) )  //打开抓线并且伸出
			if( (u35==THREAD_SCRATCH_SWITCH_OPEN)&&(clamp_flag== 1) )  //打开抓线并且伸出
			{
				go_origin_ct();	
			}
			#endif
			cut_test_flag = 0;
			/*
			剪线动作之前，要确认一下是否进行过空送急停，防止停在模板中间，剪线机针下扎会断针。
			*/
			//if( (nop_move_pause_flag ==0 )&&(finish_nopmove_pause_flag ==0) )
			{
				if((foot_half_flag == 1) || (foot_flag == 1))
				{
			    	footer_both_down();
				}
				delay_ms(100);
				
				inpress_down(inpress_high_hole);
		 		delay_ms(50);

				//2019-1-21 首先检查针杆位置，框架动作之前把针先提起来，防止撞针
				temp8 = detect_position();	
				if(temp8 == OUT)    								
				{
					find_dead_center();
				}
	
			 	trim_action();
			 	//2019-6-11 衔接效率提升开关(剪线+空送+起缝)
				#if BRIDGING_EFFICIENCY_IMPROVEMENT && ENABLE_MOTOR_TRIM_CIRCULAR_KNIFE
				if(trim_motor_need_find_origin_flag == 1)
				{
					while( cutter_delay_flag == 1)
					{
						delay_ms(1);
					}
					go_origin_trim();//等待标缝指令到原点后再找原点
				}
				#endif
				delay_ms(100);
				//2019-8-6 这个检测见面就没必要把中压脚抬起了
				//inpress_up();
				delay_ms(100);
			}
			predit_shift = 0;
			/*
			如果开抓线功能，剪线完成后抓线机构还要先伸出去
			*/
			#if DISABLE_THREAD_CLAMP_FUNCTION
			#else
			//if( (clamp_flag == 0)&&(u35==0))
			if( (clamp_flag == 0)&&(u35==THREAD_SCRATCH_SWITCH_OPEN))
    		{
    			go_origin_ct();
				delay_ms(20);
    			clamp_out();
    			clamp_stepflag = 1;
          		movect_angle = 800;
    		}
    		#endif
	}
	
	predit_shift = 0;
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

	PW_SET_FUNC_ID(25);//2020-6-6 zla 记录当前所在函数位置
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
			#if DISABLE_THREAD_CLAMP_FUNCTION
			#else
				go_origin_ct();    					    //ct go origin
			#endif
      			stepmotor_state = 0x00;
      			stepmotor_comm = 0xff;
				//2019-3-24 新增面板响应函数rec_com()的防重入机制
				if(ui_borad_reentrant_protection == 0)
				{
					ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
					rec_com();						// communication with panel 
					ui_borad_reentrant_protection = 0;//其他地方又可以使用了
				}  
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
			#if DISABLE_THREAD_CLAMP_FUNCTION
			#else
				go_origin_ct();    					    //ct go origin
			#endif
      			stepmotor_state = 0x00;
      			stepmotor_comm = 0xff;
				while(DVA == 0)
		    	{
					//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();						// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}  
		    	}	 	      	    
		  	}
	  	}
	}
  	switch(stepmotor_comm)
  	{
  	  case 0x01: 	
	  			//go_origin_ct();
	  			//delay_ms(300);
	  			#if DISABLE_THREAD_CLAMP_FUNCTION
				#else
	  			clamp_out();               // clamp out    待抓位置 
	  			#endif
  			    stepmotor_state = 1;  
  			    stepmotor_comm = 0xff;  
  			    break;     
  			         		            							
  	  case 0x02: 
			#if DISABLE_THREAD_CLAMP_FUNCTION
			#else
  	  			clamp_backstep1();         // clamp back step 1 抓紧位置 
  	  		#endif
  	  	         delay_ms(30); 
  	  	         stepmotor_state = 2;  
  	  	         stepmotor_comm = 0xff;
				 if ( SUPPORT_CS3_FUN == 1)
				 	  output_cs3(0x10,0x10); //x32.2  
				 else
				 	  BLOW_AIR = 1;
   	  	         break;
  	  	                
  	  case 0x03: 
			#if DISABLE_THREAD_CLAMP_FUNCTION
			#else
  	  			clamp_backstep2();         // clamp back step 2  断线位置
  	  		#endif
  	  	         delay_ms(30); 
  	  	         stepmotor_state = 3;  
  	  	         stepmotor_comm = 0xff; 
  	  	         break; 
  	  	           
  	  case 0x04: //clamp_backstep3();         // clamp back step 3
			#if DISABLE_THREAD_CLAMP_FUNCTION
			#else
  	  			go_origin_ct();		//原点位置
  	  		#endif
  	  	         delay_ms(300); 
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
  		case 0x00://ccw   --out    //逆时针
  			#if DISABLE_THREAD_CLAMP_FUNCTION
			#else
				movestep_ct(-1,1);
			#endif
  			delay_ms(1);
  			stepmotor_single = 0xff;
  	  		break; 
  		case 0x01://cw    ++in  	//顺时针	
  			#if DISABLE_THREAD_CLAMP_FUNCTION
			#else
				movestep_ct(1,1);
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

	PW_SET_FUNC_ID(26);//2020-6-6 zla 记录当前所在函数位置
  	initial_mainmotor();
  	if(sys.status == ERROR)
  	{
  	  return;
  	}
	//2019-1-9 进入CHECKI10中压脚检测界面，降下辅助压脚，
	//避免调试中压脚时和辅助压脚干涉
	FA = 1;
	delay_ms(200);
			 
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
		#if SECOND_GENERATION_PLATFORM
		L_AIR = 1;
		#else	
		FA = 1;
		#endif
		if(k60 == 1)
		{
			foot_half_down();
		}			  
    	origin_com = 0;

    	//2019-8-6 zla 增加回原点处理
    	go_origin_zx();                        
		stepmotor_state = 0x00;
		stepmotor_comm = 0xff;
		
  	}

	//-------------------------------------------------------------------------------------
	//2019-7-18 zla 增加非准备界面下升降中压脚按钮操作
	//-------------------------------------------------------------------------------------
	if( inpress_act_flag == 1)
	{
		//气动压脚
		if( inpress_type == AIR_INPRESS)
		{
			if(inpress_flag == 1)
				inpress_down(0);
			else
				inpress_up();
		}
		else//电机压脚
		{
			//2019-7-19 zla 新增中压脚第一次动作标志，开机时为1，如果没找原点就需要
			//抬起或者降下中压脚，就需要先找一下原点才行
			if( inpress_first_move_flag == 1 )//此标志将在go_origin_zx()中清0
			{
				go_origin_zx();
			}
			initial_mainmotor();
			
			if(inpress_high == 0)
			{
				if( inpress_flag == 1)
				{
					inpress_down(inpress_high_hole);
					delay_ms(100);
				}
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
				//2019-3-24 新增面板响应函数rec_com()的防重入机制
				if(ui_borad_reentrant_protection == 0)
				{
					ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
					rec_com();						// communication with panel 
					ui_borad_reentrant_protection = 0;//其他地方又可以使用了
				}  

				   						      	    
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
		      		//2019-3-24 新增面板响应函数rec_com()的防重入机制
					if(ui_borad_reentrant_protection == 0)
					{
						ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
						rec_com();       				// communication with panel 
						ui_borad_reentrant_protection = 0;//其他地方又可以使用了
					}    					
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
			movestep_zx(-1,12);
  		  	delay_ms(1);
  			stepmotor_single = 0xff;

  	  		break; 
  		case 0x01://+ 
			movestep_zx(1,12);
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
	initial_mainmotor();//保证主轴位置正确
  	if(sys.status == ERROR)
  	{
  	  return;
  	}
	if( ally_step != -5670)//?5670的含义
	{
		//SUM = 1;
		go_origin_allmotor();
		//上面的函数XY并没有找原点，在下面的语句里才是真正找原点
//		if( k115 == 1 )
		{
			if ( already_auto_find_start_point == 0)
			{						
				//find_start_point();
				already_auto_find_start_point = 1;
			}
		}
		//SUM = 0;
		delay_ms(500);
		if( (sys.error != 0)&&(sys.status == ERROR) )
		return;
		go_commandpoint(0,-5670);//Y轴向后退，给调试留下空间
	}
#endif

//2020-12-17 zla 新增宏定义作为换梭使能，用于减少代码量
#if ENABLE_BOBBIN_CASE == 1
	if( (bobbin_case_switch_flag ==1 )&&(bobbin_case_enable == 1) )
	{
		bobbin_case_switch_flag = 0;
		if( bobbin_case_enable == 1)
			find_a_bobbin_case(5);//直接找下一个位置，不管有没有梭芯
	}
	
	if( DVA == 0 )    
	{
		delay_ms(10);
		if( DVA == 0 )
		{				
			if( bobbin_case_enable == 1)
			{
				if(DVB == 0)//2019-11-25 DVA和DVB同时按下，连续执行换梭测试
				{
					while(1)
					{
						bobbin_case_workflow1();
						if( sys.error != OK || PAUSE == pause_active_level)
						{
							break;//报错或者暂停按钮按下时退出
						}
						delay_ms(1000);
					}
				}
				else//仅DVA按下，执行一次换梭
				{
					bobbin_case_workflow1();
				}
			}
		}
	}
	else if( DVB == 0 )
	{
		delay_ms(10);
		if( DVB == 0 )
		{
			#if 0
			if( bobbin_case_enable == 1)
				go_origin_bobbin_case_arm(1);
			#else
			if( bobbin_case_enable == 1)
			{
				/*if(_flag == 0)
				{
					_flag = 1;
					go_origin_bobbin_case_arm(1);
				}
				else
				{	
					_flag = 0;
					go_origin_bobbin_case_arm(0);
				}*/
				go_origin_bobbin_case_arm(1);
			}
			#endif
			
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
#endif
	//if( (para.rotate_cutter_working_mode == 55)	&& (rotated_cutter_running_flag == 1) )
	//{
	//	monitor_cutter_angle = get_cutter_motor_angle();
	//	da1 = monitor_cutter_angle >>1;
	//}
	
	if( checki11_test_flag == 1)
	{
		switch(checki11_item)
	  	{
	  		case 1://旋转电机切刀
				 if(rotated_cutter_enable == 1)
				 {
				    if( checki11_action == 1)//+号
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
	  		case 2://提升气缸
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
			case 3://压料气缸
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
			case 4://切刀启动
			//2018-11-19 新增旋转切刀宏定义，用于减少代码量
			#if ENABLE_ROTATED_CUTTER==1
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
							while(ret == 0)	//等待一圈的同步信号
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
						delay_ms(300);
						go_origin_yj();
					//	SUM = 0;
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
				#endif
				checki11_test_flag = 0;
			break; 
			case 5://换梭电机
			#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER40		
				if( checki11_action == 1)//+号
		  			movestep_yj(1,1);
				else 
					movestep_yj(-1,1);
		  		delay_ms(10);	
				checki11_test_flag = 0;
			#else
				//2020-12-17 zla 新增宏定义作为换梭使能，用于减少代码量
				#if ENABLE_BOBBIN_CASE == 1
					if( bobbin_case_enable == 1)
					{
					#if BOBBIN_CASE_USE_33V_MOTOR_PORT
						if( checki11_action == 1)//+号
			  				movestep_cs3(0xa000,-1,5);
						else 
						    movestep_cs3(0xa000,1,5);
					#else
						if( checki11_action == 1)//+号
			  				movestep_cs3(0xa000,-10,15);
						else 
						    movestep_cs3(0xa000,10,15);
					#endif
			  		  	delay_ms(20);
					}
				#endif
				checki11_test_flag = 0;
			#endif
			break; 
			case 6://抓臂电机
				//2020-12-17 zla 新增宏定义作为换梭使能，用于减少代码量
				#if ENABLE_BOBBIN_CASE == 1
					if( bobbin_case_enable == 1)
					{
						if( checki11_action == 1)//+号
			  				movestep_cs3(0x2000,-1,5);
						else 
						    movestep_cs3(0x2000,1,5);
			  		  	delay_ms(10);
					}
				#endif
				checki11_test_flag = 0;
			break; 
			case 7://夹紧气缸
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
			case 8://抓臂前后气缸
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
			case 9://换梭单步测试
				//2020-12-17 zla 新增宏定义作为换梭使能，用于减少代码量
				#if ENABLE_BOBBIN_CASE == 1
					if( bobbin_case_enable == 1)
					{
						find_a_bobbin_case(5);
						checki11_test_flag = 0;
					}
				#endif
			break; 
			case 10://换梭动作复位
				//2020-12-17 zla 新增宏定义作为换梭使能，用于减少代码量
				#if ENABLE_BOBBIN_CASE == 1
				if( bobbin_case_enable == 1)
				{
					go_origin_bobbin_case_arm(0); 
				}
				#endif
				checki11_test_flag = 0;
			break; 
			case 11://旋转切刀复位
			//2018-11-19 新增旋转切刀宏定义，用于减少代码量
			#if ENABLE_ROTATED_CUTTER==1
				if( rotated_cutter_enable == 1)
					go_origin_rotated_cutter();
				#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER40	
				go_origin_yj();	
				#endif
			#endif
				checki11_test_flag = 0;
			break;    
			
			case 12://直线切刀动作
				#if ENABLE_LOOPER_CUTTER
				if( stepper_cutter_enable == 1)
			 	{
					 if( checki11_action == 1)//+号
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
			case 13://直线切刀找原点
			#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER40  && ENABLE_ROTATED_CUTTER == 1
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
			case 14://激光电源
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
			case 15://激光压料气阀
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
			case 16://激光切割信号
					LASER_POWER_PIN = 1;
					delay_ms(200);
					LASER_POWER_PIN = 0;
					checki11_test_flag = 0;
			break;
			case 17://激光废气
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
			case 18://激光指示灯
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

	PW_SET_FUNC_ID(29);//2020-6-6 zla 记录当前所在函数位置
	
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

	PW_SET_FUNC_ID(27);//2020-6-6 zla 记录当前所在函数位置
	if(OutOfRange_flag == 0)
	{
		if(emermove_high == 0) 
		{
			predit_shift = 1;
			if(inpress_flag == 1)
			{
				if(pause_flag == 1)              			
				{	
//					sys.error = ERROR_19;	  			// pause button is not in normal position  		
					status_now = sys.status;
					emermove_high = 71;
					predit_shift = 0;
//					sys.status = ERROR;
//					StatusChangeLatch = ERROR;  	
					SET_SYS_ERROR(ERROR_19);//急停开关未在正常位置
					return;    	      	    
				}
				else if(pause_flag == 0)
				{
					inpress_down(emermove_high);

					//if((k03 == 0) && (fk_status == OUT) && (fk_cut == 0))  
					if((k03 == TENSION_TYPE_MECHANICAL) && (fk_status == OUT) && (fk_cut == 0))  
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
//					sys.error = ERROR_19;	  			// pause button is not in normal position 
					status_now = sys.status;
					emermove_high = 0;
					predit_shift = 0;
//					sys.status = ERROR;
//					StatusChangeLatch = ERROR;   	
					SET_SYS_ERROR(ERROR_19);//急停开关未在正常位置
					return;    	      	    
				}
				else if(pause_flag == 0)
				{
					inpress_up();
					
					//if(k03 == 0)    
					if(k03 == TENSION_TYPE_MECHANICAL)
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
//								sys.error = ERROR_19;	  			// pause button is not in normal position
								status_now = sys.status;
//								sys.status = ERROR;
//							    StatusChangeLatch = ERROR;      	
								SET_SYS_ERROR(ERROR_19);//急停开关未在正常位置
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
								
							    //if(k03 == 0)    
							    if(k03 == TENSION_TYPE_MECHANICAL)
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
//							sys.error = ERROR_19;	  			// pause button is not in normal position 
							status_now = sys.status;
//							sys.status = ERROR;
//						    StatusChangeLatch = ERROR;     	
							SET_SYS_ERROR(ERROR_19);//急停开关未在正常位置
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
							
							//if(k03 == 0)     
							if(k03 == TENSION_TYPE_MECHANICAL)
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
//					sys.error = ERROR_19;	  					// pause button is not in normal position 
					status_now = sys.status;
					single_flag = 0;
					do_pat_point_sub_one();
					predit_shift = 0;
//					sys.status = ERROR;
//					StatusChangeLatch = ERROR;  	
					SET_SYS_ERROR(ERROR_19);//急停开关未在正常位置
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
//						sys.error = ERROR_15;
						SET_SYS_ERROR_1(ERROR_15);//超出缝制范围
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
//					sys.error = ERROR_19;	  			// pause button is not in normal position 
					status_now = sys.status;
					single_flag = 0;
					do_pat_point_add_one();
					predit_shift = 0;
//					sys.status = ERROR;
//					StatusChangeLatch = ERROR;   	
					SET_SYS_ERROR(ERROR_19);//急停开关未在正常位置
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
//					sys.error = ERROR_19;	  			// pause button is not in normal position  
					status_now = EMERMOVE;
					single_flag = 0;
					predit_shift = 0;
//					sys.status = ERROR;
//					StatusChangeLatch = ERROR;  	
					SET_SYS_ERROR(ERROR_19);//急停开关未在正常位置
					return;    	      	    
				}
				else if(pause_flag == 0)	
				{ 
					predit_shift = 1;
					inpress_high = inpress_high_hole;
					tension = tension_hole;
					//if(u42 == 0)
					if(u42 == NEEDLE_STOP_POSITION_UP_POSITION)
					{
						//if(u94 == 1)//停车到上死点（0度）
						if(u94 == NEEDLE_STOP_POSITION_AFTER_ORIGIN_UP_DEAD_POINT)//停车到上死点（0度）
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
					#if 0
					if(k110 == 1)
					{
					   R_AIR = 0;
					   delay_ms(80); 
					}
					#endif
					if(u38 == 0)
					{
			  		  	footer_both_up();  
					}
					delay_ms(1);
					inpress_up(); 
			 		single_flag = 0; 
					//if(u42 == 0)
					if(u42 == NEEDLE_STOP_POSITION_UP_POSITION)
					{
						//if(u94 == 1)
						if(u94 == NEEDLE_STOP_POSITION_AFTER_ORIGIN_UP_DEAD_POINT)
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
				#if DISABLE_THREAD_CLAMP_FUNCTION
				#else
					//if(u35==0)                     // have clamp thread motor //08.12.31 wr add this action
					if( u35 == THREAD_SCRATCH_SWITCH_OPEN ) 
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
//      	  							sys.status = ERROR;
//									StatusChangeLatch = ERROR;
//									if( sys.error == 0)
//          							sys.error = ERROR_23;	 // clamp is not in normal position  	
									SET_SYS_ERROR(ERROR_23);//抓线位置异常
          							//_speed_array[0] = 2;
          							return;
        						}
      						}
          				}
          				else
          				{
          					if(clamp_flag == 1)        // clamp is out
            				{
//            		  			sys.status = ERROR;
//								StatusChangeLatch = ERROR;
//								if( sys.error == 0)
//            		  			sys.error = ERROR_23;	   // clamp is not in normal position	
            		  			//_speed_array[0] = 3;
            		  			SET_SYS_ERROR(ERROR_23);//抓线位置异常
            		  			return;
            				}
            				else
            				{
          			  			clamp_stepflag = 0;      // clamp thread step 0
          					}
          				}
        			} 
        		#endif
					FootRotateFlag = 0;
			  		sys.status = READY;
					StatusChangeLatch = READY; 	 
				}
	  		    break;	
			case 4:	 
				if(pause_flag == 1)              			
				{			
//					sys.error = ERROR_19;	  					// pause button is not in normal position 
					status_now = sys.status;
					single_flag = 0;
					do_pat_point_sub_one();
					predit_shift = 0;
//					sys.status = ERROR;
//					StatusChangeLatch = ERROR;  	
					SET_SYS_ERROR(ERROR_19);//急停开关未在正常位置
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
//					sys.error = ERROR_19;	  					// pause button is not in normal position 
					status_now = sys.status;
					single_flag = 0;
					do_pat_point_add_one();                                //2011-6-15 songyang modify
					predit_shift = 0;
//					sys.status = ERROR;
//					StatusChangeLatch = ERROR;  	
					SET_SYS_ERROR(ERROR_19);//急停开关未在正常位置
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
//					sys.error = ERROR_19;	  					// pause button is not in normal position 
					status_now = sys.status;
					single_flag = 0;
					predit_shift = 0;
//					sys.status = ERROR;
//					StatusChangeLatch = ERROR;  	
					SET_SYS_ERROR(ERROR_19);//急停开关未在正常位置
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
//					sys.error = ERROR_19;	  					// pause button is not in normal position 
					status_now = sys.status;
					single_flag = 0;
					predit_shift = 0;
//					sys.status = ERROR;
//					StatusChangeLatch = ERROR;  	
					SET_SYS_ERROR(ERROR_19);//急停开关未在正常位置
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
					
				//if(u42 == 0)
				if(u42 == NEEDLE_STOP_POSITION_UP_POSITION)
				{
					//if(u94 == 1)
					if(u94 == NEEDLE_STOP_POSITION_AFTER_ORIGIN_UP_DEAD_POINT)
					{  
						find_dead_point();
					}
				}
				//if(u39 == 1)
				if(u39 == AFTER_SEWING_FINISH_GO_ORIGIN_OPEN)
				    go_origin_allmotor();
				else
					move_startpoint();
					
				OutOfRange_flag = 0;
				single_flag = 0; 
				delay_ms(20);
				#if 0
				if(k110 == 1)
				{
					R_AIR = 0;
					delay_ms(80);  
				}
				#endif
				if(u38 == 0)
				{
		  		     footer_both_up();   
				}
				delay_ms(1);
				inpress_up(); 
		 
				//if(u42 == 0)
				if(u42 == NEEDLE_STOP_POSITION_UP_POSITION)
				{
					//if(u94 == 1)
					if(u94 == NEEDLE_STOP_POSITION_AFTER_ORIGIN_UP_DEAD_POINT)
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
// 	if( (bar_coder_refresh_enable == 1)&&(bar_coder_refresh_flag == 0) )//启动前确认，但条码没刷新则不启动
//		return;
	//if(pattern_change_flag == 1 )
	//   return;
	while( pattern_change_flag == 1 )
	{
		 //2019-3-24 新增面板响应函数rec_com()的防重入机制
		 if(ui_borad_reentrant_protection == 0)
		 {
			 ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
			 rec_com(); 					 // communication with panel 
			 ui_borad_reentrant_protection = 0;//其他地方又可以使用了
		 }	
		 check_output_pattern_done();
	}
	
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
			//find_start_point();
			already_auto_find_start_point = 1;
		}
	}
	
	//2019-10-10 新增扫码后启动延时功能，避免RFID切换花样时立刻启动缝制引发的跑位问题
	#if ENABLE_DELAY_START_AFTER_RFID
	while(delay_start_after_rfid_time>0)//死等延时时间到达
	{
		delay_ms(1);
	}
	#endif
	
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
					   	 	if( cs.enable >= 1 )
							{
								//2020-12-7 zla 在启动前，如果发现当前是处于一个虚拟原点位置，那么
								//那么需要回到实际位置再执行后续的动作
//								buzzer_control_time = 500;
								cs_goto_real_positon();
								if(sys.status == ERROR)
									return;	
							}
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
								 //if((stop_number%2 == 1)&&(u41 == 0)) 
								//2020-12-7 zla 如果是急停上压框上功能码，回到虚拟原点
								if( cs.enable >= 1 )
								{
									if( IsSpecialStopCode(pat_point) == 1 )
									{
										cs_goto_image_positon();
									}
								}
								 if((stop_number%2 == 1)) 
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
					 origin2_lastmove_flag = 0;
					 stop_flag = 0;
					 process_data();
					 
					 //2019-3-26 当中途停止后，如果段跳转后再启动，这里会执行两次process_data()
           			 //导致花样向前多移动了一针，后续缝制就跑位一针了，这里进行判断，只有当前花样
            		 //确实是停止码才移动花样到下一针，这个问题在中缝和杰克模板机上都有，中捷暂未
            		 //报告，但是还是修复为好
					 if(stop_flag == 1 || origin2_lastmove_flag == 1)
					 {
						 origin2_lastmove_flag = 0;
						 stop_flag = 0;
						 process_data(); 
					 }
					 
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
									if( cs.enable >= 1 )
									{
										//2020-12-7 zla 在启动前，如果发现当前是处于一个虚拟原点位置，那么
										//那么需要回到实际位置再执行后续的动作
//										buzzer_control_time = 500;
										cs_goto_real_positon();
										if(sys.status == ERROR)
											return;	
									}
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
			//2020-12-7 zla 如果是急停上压框上功能码，回到虚拟原点
			if( cs.enable >= 1 )
			{
				if( IsSpecialStopCode(pat_point) == 1 )
				{
					cs_goto_image_positon();
				}
			}
			
			sys.status = READY;
			StatusChangeLatch = READY;
			StopStatusFlag = 1; 
			//if((stop_number%2 == 1)&&(u41 == 0))
			if((stop_number%2 == 1))
			{
			    footer_both_up();  
			}				
		}	 
		else
		{
			if( cs.enable >= 1 )
			{
				//2020-12-7 zla 在启动前，如果发现当前是处于一个虚拟原点位置，那么
				//那么需要回到实际位置再执行后续的动作
				cs_goto_real_positon();
				if(sys.status == ERROR)
					return;
			}
			
			 sys.status = RUN;
			 StatusChangeLatch = RUN;
			 StopStatusFlag = 0;	
			 
			 if( aging_flag == 1)
			 {
			   	aging_com = 1;
				if(u233 == 2)
				{ 
					//u39 = 1;
					u39 = AFTER_SEWING_FINISH_GO_ORIGIN_OPEN;
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

	PW_SET_FUNC_ID(28);//2020-6-6 zla 记录当前所在函数位置
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

	PW_SET_FUNC_ID(32);//2020-6-6 zla 记录当前所在函数位置
	
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
	if(5 == predit_shift) 		//1接收到一包数据
	{  
		if(1 == erase_falg) 	//如果包号为0,则通知步进进入升级状态
		{
		    jump_to_begin();
		}
		send_stepmotor_up_drv();//将收到一包数据通过SPI发送给驱动
		if(1 == erase_falg)     //延时等待操作完成
		     delay_ms(delay_t); 
		else  
		     delay_ms(40);         
		drv_satus = read_stepmotor_up_drv();
		//0XA0    功能码错误
		//0XA1    文件校验错误
		//0XA2    数据包自校验错误
		//0XA3    SPI通信校验错误
		//0XA4    flash擦除错误
		//0XA5    flash烧写错误
		//0XA6    flash校验错误
		//0XA7    数据包crc校验错误
		//0XA8    解锁错误
		//0xa9    超时
		delay = 0;
		while(0x00 != drv_satus)
		{
			//2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();						// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}  
			delay_ms(5);
			drv_satus = read_stepmotor_up_drv();
			delay++;
			if( delay >1000)//超时
			{
//			    sys.status = ERROR;
//				sys.error = 0x34;
				SET_SYS_ERROR(ERROR_52);//步进驱动升级失败
				de_bug.test1 = 0x00;
				de_bug.test2 = 0xa9;
				break;	
			}
			if( (drv_satus >=0xa0) && (drv_satus <=0xaf) )
			{
//				sys.status = ERROR;
//				sys.error = 0x34;
				SET_SYS_ERROR(ERROR_52);//步进驱动升级失败
				de_bug.test1 = 0x00;
				de_bug.test2 = drv_satus;
				break;
			}
		}
		predit_shift = 0;

	}
	if(6 == predit_shift ) //2升级结束校验证
	{  
		send_stepmotor_end_drv();
        predit_shift = 0;
	}
  	//2019-3-24 新增面板响应函数rec_com()的防重入机制
	if(ui_borad_reentrant_protection == 0)
	{
		ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
		rec_com();       				// communication with panel 
		ui_borad_reentrant_protection = 0;//其他地方又可以使用了
	}  	
}
//？？？？2018-9-12，为何有这个状态
void bobbin_change_status(void)
{
	UINT8 temp8;
	PW_SET_FUNC_ID(34);//2020-6-6 zla 记录当前所在函数位置

	//2020-12-17 zla 新增宏定义作为换梭使能，用于减少代码量
	#if ENABLE_BOBBIN_CASE == 1
	temp8 = bobbin_case_workflow1();
	bobbin_case_once_done_flag = 0;
	#endif
	
	sys.status = READY;
	sys.error = 0;
}

void download_dsp_curve_status(void)
{
	UINT8 i;
	
	PW_SET_FUNC_ID(33);//2020-6-6 zla 记录当前所在函数位置
	
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
//			sys.status = ERROR;
//			sys.error =  ERROR_52;
			PW_SET_CODE_ID(52000);//2020-6-6 zla 记录当前所在代码段位置
			SET_SYS_ERROR(ERROR_52);//步进驱动升级失败
			de_bug.test1 = 0x00;
			de_bug.test2 = 0xa7;
		}
		predit_shift = 0;
	}
	if(StatusChangeLatch != DOWNLOAD_DSP_CURVE) 
	{
		sys.status = StatusChangeLatch;
	}
}


void multipule_io_status(void)
{	
	PW_SET_FUNC_ID(35);//2020-6-6 zla 记录当前所在函数位置
	
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

  PW_SET_FUNC_ID(36);//2020-6-6 zla 记录当前所在函数位置
  
  if(5 == predit_shift) 
  {  
	    if(1 == erase_falg)
	       multipule_program_beginning(4);
	   
	    send_multipule_program_data(4);//发送升级程序内容
	    if(1 == erase_falg)
	        delay_ms(2500); 
	    else  
	    	delay_ms(60);
	
	    drv_satus = read_multipule_program_status(4);//读取升级状态
	    delay = 0;
	    while(0x00 != drv_satus)
	    {
			//2019-3-24 新增面板响应函数rec_com()的防重入机制
			if(ui_borad_reentrant_protection == 0)
			{
				ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
				rec_com();						// communication with panel 
				ui_borad_reentrant_protection = 0;//其他地方又可以使用了
			}  
	        delay_ms(5);
	        drv_satus = read_multipule_program_status(4);//读取升级状态
	        delay++;
	        if( delay >1000)//超时
	        {
//	          sys.status = ERROR;
//	          sys.error = ERROR_86;
			  SET_SYS_ERROR(ERROR_86);//步进驱动升级失败
	          de_bug.test1 = 0x00;
	          de_bug.test2 = 0xa9;
	          break;	
	        }
	        if( (drv_satus >=0xa0) && (drv_satus <=0xaf) )
	        {
//	          sys.status = ERROR;
//	          sys.error = ERROR_86; //50 号错误？？？？？？？？？？？
			  SET_SYS_ERROR(ERROR_86);//步进驱动升级失败
	          de_bug.test1 = 0x00;
	          de_bug.test2 = drv_satus;
	          break;
	        }
	     }
      	 predit_shift = 0;
  }
  
  if(6 == predit_shift ) //2升级结束校验证
  {  
      multipule_program_end(4);
      predit_shift = 0;
  }
	//2019-3-24 新增面板响应函数rec_com()的防重入机制
	if(ui_borad_reentrant_protection == 0)
	{
		ui_borad_reentrant_protection = 1;//设置为1，避免其他地方重入面板应答函数
		rec_com();       				// communication with panel 
		ui_borad_reentrant_protection = 0;//其他地方又可以使用了
	}  
  #endif		
}


void rfid_read_write(void)
{
	UINT8 ret;

	PW_SET_FUNC_ID(30);//2020-6-6 zla 记录当前所在函数位置
	
	#if ENABLE_RFID_FUNCTION	
	if(auto_function_flag == TEMPLATE_IDENTIFY_SWITCH_OPEN)
	{
		if((formwork_identify_device == TEMPLATE_IDENTIFY_DEVICE_RFID)&&(rc522_control_falg==1))//RFID
		{	
			SUM =1;
			ret = RFID_SCAN();
	        rfid_wr_ret(ret);
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
