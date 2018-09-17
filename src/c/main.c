//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : main.c
//  Description: Core program to control the sewing machine
//  Version    Date     Author    Description
//  0.01     09/08/08   lm        created
//  ...
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"         // M16C/62P special function register definitions
#include "..\..\include\typedef.h"        // Data type define
#include "..\..\include\common.h"         // Common constants definition
#include "..\..\include\variables.h"      // External variables declaration
#include "..\..\include\initial.h"        // External variables declaration
#include "..\..\include\system.h"         // External variables declaration
#include "..\..\include\motor.h"          // Motor function
#include "..\..\include\delay.h"          // delay time definition
#include "..\..\include\watch.h"          // System watch function
#include "..\..\include\communication.h"  // Communication function
#include "..\..\include\stepmotor.h"      // stepper motor function
#include "..\..\include\action.h"         // action function
#include "..\..\include\solenoid.h"       // solenoid driver definition
#include "..\..\include\iic_bus_eeprom.h"       //
//--------------------------------------------------------------------------------------
//  Internal functions and subroutines declaration
//--------------------------------------------------------------------------------------
#pragma	INTERRUPT/E ta0_int
void ta0_int(void);
#pragma interrupt tb3_int
void tb3_int(void);
#pragma interrupt tb4_int
void tb4_int(void);
/**
  * @函数功能 主程序，系统状态机入口
  * @参数 无
  * @返回值 无
  */
  
void main(void)
{
	INT16  count;
	UINT8  config_flag;
	//--------------------------------------------------------------------------------------
  	// call initial function
  	//--------------------------------------------------------------------------------------
	initial();			
	reset_panel();
	version_check();   
	config_flag = 0; 

//机型4、6、8、9、10、11、12、13、14、15、16
#if DOUBLE_X_60MOTOR
	if( DVB == 0)
	{
		setup_stepper_moter();
		config_flag = 1;
	}
#endif    

#if INSTALLMENT	
	main_control_lock_flag = read_par(0);
#endif
  	while(connect_flag == 0)//只有等到PARA指令将工作参数下发以后，主控才可以继续向下运行
  	{
 		delay_ms(50);
		#if DMA_UART1_MODE
			  if(ir_dm1ic)
			  {
				 ir_dm1ic = 0;
			  }
		#endif
		if( (sys.status == DOWNLOAD)||(sys.status == DOWNLOAD_DSP1)||(sys.status == DOWNLOAD_DSP2)||(sys.status == DOWNLOAD_DSP_CURVE))
			break;
  	} 
	//上电按急停后，解除电机故障直接找原点出现无法进READY改参数的问题
	if (PAUSE == pause_active_level)
	{
		delay_us(10000);                           
		if(PAUSE == pause_active_level)
		{
			release_poweron_ready = 1;
			if( DVB == 0)
			{
				delay_ms(30);
				if( DVB == 0)
				{
					special_encoder_mode = 1;//上电同时按急停和DVB
				}
			}
		}
	}
	SNT_H = 0;    
	SNT_ON = 0;     
	delay_ms(300);	        //延时给系统硬件环境一个建立时间
	
	#if SUPPORT_NEW_DRIVER == 0
	config_stepmotor_drv();
	#endif
	count = 0;
	while(1)
	{
	    if(ir_ta0ic)       
	    {
			ir_ta0ic = 0;
			count++;
			if(count >= 3)
			break;
		}
	}
	ta0ic = TA0_IPL;		//确保定时器工作正常，系统重要保护机制正常
	version_check();  
  	//--------------------------------------------------------------------------------------
  	// 系统背景状态机
  	//--------------------------------------------------------------------------------------
	if( stepversion1 >= 60000 )
    {
	    sys.status = DOWNLOAD_DSP1;  
		predit_shift = 0; 
    }
	else if( stepversion2 >= 60000 )
    {
	    sys.status = DOWNLOAD_DSP2;  
		predit_shift = 0; 
    }
	#if MULTIPULE_IO_ENABLE	 == 1
	else if( stepversion3 >= 60000 )
    {
	    sys.status = DOWNLOAD_DSP3;  
		predit_shift = 0; 
    }
	//else if( stepversion4 >= 60000 )
    //{
	//    sys.status = DOWNLOAD_DSP4;  
	//	predit_shift = 0; 
    //}
	#endif	
	
	//机型2、3、4、5、6、8、9、10、11、12、13、14、15、16、17、18、19、20、21、22、23、24、25、26、
	//27、28、29、30、31、33、34、35、36、37、38、39、40、41、42、44、55
	#if SUPPORT_NEW_DRIVER
	else
	{
	   if( (config_flag == 0)&&(PAUSE != pause_active_level) )
	   	   setup_stepper_moter();
	}
    #endif 
	
	#if ENABLE_RFID_FUNCTION
	if((auto_function_flag == 1)&&(formwork_identify_device ==3)&&(PAUSE != pause_active_level))
	{
		RFID_initial();//初始化MFRC522，同时更改MFRC522通信波特率为115200
		init_uart1_RC522();//将UART1的波特率改到115200，以便和MFRC522匹配，此前波特率默认9600
		rfid_config_flag = 1;
	}
	#endif
	
	#if FAN_POWER_ENABLE
		FAN_POWER = 1;
	#endif
	#if SUPPORT_UNIFY_DRIVER
	crc1 = read_stepmotor_curve_crc(1);
	crc2 = read_stepmotor_curve_crc(2);
	#if MULTIPULE_IO_ENABLE	== 1
	crc3 = read_stepmotor_curve_crc(3);
	#endif
	#endif
	#if DEBUG_RFID_DA1_FUN	
			da1 = 0;
	#endif
	while(1)
  	{
		#if UART1_DEBUG_OUTPUT_MODE ==1
		
		#else
		
	  		#if CODE_BAR
							
			  #if DMA_UART1_MODE
			  	if((formwork_identify_device >=2)&&(auto_function_flag == 1)&&((sys.status == READY)||(sys.status == CHECKI03)))
				{
				  	rec1_com();
				}
			  #else
				//条码扫描使用的是串口1,要使能相应的中断、设定中断优先级		
				if((sys.status == READY)&&(formwork_identify_device >=2)&&(auto_function_flag == 1))
				{
				  re_u1c1 =1;
				  rec1_com();
				}
				
			  #endif
			  
			#endif
		#endif
		/*
		下面是为响应参数K141 ，关闭错误状态下的输出
		EXTEND------机头板上“气阀1”
		*/
		if ( (sys.status != ERROR)&&(alarm_output_enable >= 1) )
		{
			RED_ALARM_LED = 0;
		}
		if( sys.status != DOWNLOAD)
		    rec_com();   
	
		/*
		对模板机而言比较重要的状态：
		READY,RUN,FINISH,SETOUT,PREEDIT,EDIT
		*/
	  	switch(sys.status)
    	{
	      	case FREE:      free_status();      break;//自由状态，无法启动缝制
	      	case READY:     ready_status();     break;//缝制状态状态	
	      	case RUN:       run_status();       break;//缝制运行过程，主轴开始转动
	      	case ERROR:     error_status();     break;//错误状态       	      
	      	case PREWIND:   prewind_status();   break;//准备绕线状态	
	      	case WIND:      wind_status();      break;//绕线进行状态，主轴开始转动		
	      	case INPRESS:   inpress_status();   break;//设置中压脚状态	
	      	case POWEROFF:  poweroff_status();  break;//掉电状态（非正常状态）                               
	      	case SINGLE:    single_status();    break;//单步试缝状态    	
	      	case MANUAL:    manual_status();    break;//手动移框状态    		
	      	case SETOUT:    setout_status();    break;//返回起缝点状态   	
	      	case EMERSTOP:  emerstop_status();  break;//紧急停车状态（按急停按钮后停车过程中）  
	      	case PREEDIT:   preedit_status();   break;//花样主编辑状态（非移框动作状态）   	
	      	case EDIT:      edit_status();      break;//花样编辑状态（移框产生坐标） 	 		
	      	case NOEDIT:    noedit_status();    break;//编辑切换过渡状态（无动作）	
	      	case FINISH:    finish_status();    break;//缝制结束状态		
	      	case NEEDLE:    needle_status();    break;//穿线状态		
	      	case WAITOFF:   waitoff_status();   break;//等待关机状态（按界面中压脚后提示关电）		
	      	case TRIM:      trim_status();      break;//剪线状态		
	      	case SLACK:     slack_status();     break;//空状态（设定参数，不产生动作）		
	      	case CHECKI03:  checki03_status();  break;//输入信号检测状态		
	      	case CHECKI04:  checki04_status();  break;//主轴电机检测状态		
	      	case CHECKI05:  checki05_status();  break;//输出信号检测状态		
	      	case CHECKI06:  checki06_status();  break;//XY移框检测状态		
	      	case CHECKI07:  checki07_status();  break;//压脚剪线电机检测状态		
	      	case CHECKI08:  checki08_status();  break;//抓线电机检测状态		
	      	case CHECKI10:  checki10_status();  break;//中压脚电机检测状态		
	      	case EMERMOVE:  emermove_status();  break;//急停响应状态（主轴停车后，可前后移动）	
			case DOWNLOAD:  download_status();  break;//主控程序下载状态
			case CHECKI11:  checki11_status();  break;  
			case RFID_WR :  rfid_read_write();  break;  
			case DOWNLOAD_DSP1:  
			case DOWNLOAD_DSP2:
			case DOWNLOAD_DSP3:
			case DOWNLOAD_DSP4:      downloaddsp_status();  		break;	       
			case DOWNLOAD_DSP_CURVE: download_dsp_curve_status();	break;
			case BOARDTEST:          bobbin_change_status();		break; 			    
			case MULTIPULE_IO:	     multipule_io_status();			break;
			case DOWNLOAD_SPFL:		 download_multipul_program_status();	break;
		
	      	default:    sys.status = READY;sys.error = 0;  			break;        	
    	} 
	}
}

/**
  * @函数功能 定时器0 1毫秒中断程序
  * @参数 无
  * @返回值 无
  */
void ta0_int(void)
{
	UINT16 tmp_pattern,tmp_pattern2;
	INT16 temp16; 
	flag_1ms = 1;//1毫秒计时时间到标志
	counter_1ms ++;
	#if ENABLE_RFID_FUNCTION
		ms_scan_counter++;
	#endif
	
	#if ENBALE_TURNON_LASER_FUN 
	if( valve_3_open_counter < 20000)
		valve_3_open_counter++;
	else if( (sys.status != CHECKI05)&&(sys.status != CHECKI11) )
		LASER_FUN_PIN = 1;
	#endif
	
	#if SECOND_GENERATION_PLATFORM || CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
		if( inpress_litter_footer_action_flag == 1)
		{
			if( inpress_litter_footer_action_counter > 0 )
			{
				inpress_litter_footer_action_counter--;
			}
			else
			{
				inpress_litter_footer_action_flag = 0;
				inpress_litter_footer_action_counter = 0;
				
				if( inpress_litter_footer_action_value == 0)
				{
					#if CURRENT_STEPPER_CONFIG_TYPE == MACHINE_CONFIG_NUMBER30
					FA = 0;
					#else
					L_AIR = 0;
					#endif
				}
				else	
				{
					#if CURRENT_STEPPER_CONFIG_TYPE == MACHINE_CONFIG_NUMBER30
					FA = 1;
					#else
					L_AIR = 1;
					#endif
				}
			}
				
		}
	#endif
	//--------------------------------------------------------------------------------------
  	//  call motor control function
  	//--------------------------------------------------------------------------------------
	#if ERROR_OUTPUT_DEBUG
	if( sys.status == ERROR  )
	{
		if( (sys.error ==ERROR_46)||(sys.error ==ERROR_29) )
		{
		 	OUTPUT_ON = 0;
			T_DIR =1; 
		}
	}
	#endif	
	
	motor_control();//主轴的控制函数
	
	if( laser_fun_delay_off_flag ==1)
	{
	    if( laser_fun_delay_counter < 6000)
		    laser_fun_delay_counter++;
		else
		{			
			laser_fun_delay_off_flag = 0;
			laser_fun_delay_counter = 0;
			LASER_FUN_PIN = 0;
		}
	}
	if(movexy_delay_flag ==1 )
	{
		if( movexy_delay_counter >0 )
		    movexy_delay_counter--;
		else
			movexy_delay_flag = 0;
	}
	
	if( rotated_cutter_running_flag == 1)
	{
		if( rotated_cutter_running_counter >0 )
		    rotated_cutter_running_counter--;
		else
			rotated_cutter_running_flag = 0;
	}
	if( drill_motor_run_enable == 1)
	{
		cutter_syn_counter ++;
		if( cutter_syn_counter >= k112)
		{
			cutter_syn_counter = 0;  
			//drill_motor_run_enable = 0;  
		}
	}
	if( ct_bump_action_flag == 1)//计时关闭抓线吸风时间
	{
		ct_bump_counter++;
		if( ct_bump_counter > ct_bump_workingtime)
		{
			ct_bump_action_flag = 0;
			if ( SUPPORT_CS3_FUN == 1)
				 output_cs3(0x10,0x0); //x32.2
			else
				 BLOW_AIR = 0;
		}
	}
	/*
	 同步两次动框，确保缝制过程中，相邻两次动框时间不能出现覆盖的情况，确保前一步时间走完，再发下一步指令；
	 movestep_x_flag ， movestep_y_flag 是XY两轴分开动作时的各自动作时间值；
	*/
	if(movestep_x_flag == 1)
	{
		if( movestepx_delay_counter > 0)
			movestepx_delay_counter--;
	}
	if(movestep_y_flag == 1)
	{
		if( movestepy_delay_counter > 0)
		    movestepy_delay_counter--;
	}
	#if FOLLOW_INPRESS_FUN_ENABLE
	if( movezx_delay_flag ==1)
	{
		if( movezx_delay_counter >0 )
		    movezx_delay_counter--;
		else
		    movezx_delay_flag = 0;
	}
	#endif
	#if DSP3_CUTER_DRIVER
	if( indraft_control_flag == 1)
	{
		if( indraft_control_counter > 0)
		    indraft_control_counter --;
	}
	#endif
	if( blow_air_flag == 1 )//吹气功能定时关闭
	{
		if(blow_air_counter > 0)
		  blow_air_counter--;
		else
		 {
			 blow_air_flag = 0;
			 if( k171 ==1 )
			  	BLOW_AIR3 = 0;
			  else
			  	BLOW_AIR2 = 0;
		 }
	}
	
	if( id_alarm_flag == 1)
	{
		if( id_alarm_counter < 250)
		   id_alarm_counter++; 
		else
		{
			id_alarm_flag = 0;
			id_alarm_counter = 0;
		}
	}
	
	/*
	中压脚升降时配合松线器产生松开动作
	*/ 
	if(fk_status == IN)
	{
		fk_count = fk_count + 1;
		if(fk_count >= 30000)
		{
			da0 = 0;
			SNT_H = 0;  // 33V to 24V 
			fk_count = 1;
			fk_cut = 1;
			fk_status = OUT;
		}
	}
	/*
	  在剪线过程中的松线器打开控制功能
	  启动松线器控制功能后，先按外面给定的DA0值运行100毫秒，然后按设定的保持电流来维持状态
	  1秒钟以后自动关闭DA0的输出
	*/	
	if(DAActionFlag==1)
	{
		DAActionCounter++;
		if(DAActionCounter >= 100)
		{
			//DAActionFlag = 0;
			da0 = release_tension_current;//夹线器松开
		}
		if(DAActionCounter >= 300)
		{
			DAActionFlag = 0;
			da0 = 0; //夹线器夹紧
		}
	}
	if( da0_release_flag == 1)
	{
		da0_release_conter++;
		if(da0_release_conter >= 100)
		{
			da0 = release_tension_current; 
		}
		if(da0_release_conter >= 10000)
		{
			da0_release_flag = 0;
			da0 = 0; 
		}
	}
	if( cutter_protect_flag == 1)
	{
		if( cutter_protect_counter < 2000)
		    cutter_protect_counter++;
		else
		{
			cutter_protect_counter = 0;
			cutter_protect_flag = 0;
			FL_pwm_action_flag = 0;
			#if SECOND_GENERATION_PLATFORM 
		    FL = 0;
			#elif CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
			FA = 0;
			SNT_H = 0;
		    #else
		    SNT_H = 0;    
		    L_AIR = 0;
			#endif
		}
				
	}
	#if SECOND_GENERATION_PLATFORM == 0 && CURRENT_STEPPER_CONFIG_TYPE != CONFIG_MACHINE_TYPE_6037_800
	
	/*
	  模板机的注油泵的控制，控制引脚 COOL_AIR-----“辅助气阀2”
	  注油机构工作描述：一个靠主轴带动的注油泵，在主轴运转时抽取油池里的润滑油开始向旋梭内注油。
	  旋梭另一侧通过一导管和常开的气阀，将注入旋梭的油排回油池，这样形成了一个供油循环。
	  从上面可以看到，气阀默认状态是打开，这时相当于排油状态，隔一段时间，气阀吸合关闭，这时油
	  就会溢满旋梭，相当于给旋梭注了一次油，工作一小段时间再关闭恢复排油状态。
	  cool_air_action_flag -----  气阀的状态 0-表示关闭，对应着排油状态；1--是注油状态；
	  cool_air_close_time  -----  注油的间隔时间，按秒来计数的
	  cool_air_open_time  ------  注油的时间，气阀动作关闭状态时间，按毫秒来计数
	*/	
	
	if( motor.spd_obj > 0)
	{
		cool_air_1_sec ++;		
		if( cool_air_action_flag == 0)
		{
			COOL_AIR = 0;
			if( cool_air_1_sec >= 1000)// 1 sec
			{
				cool_air_1_sec = 0;
				cool_air_counter++;
					if ( cool_air_counter >= cool_air_close_time)
				{
				   cool_air_counter = 0;
				   cool_air_action_flag = 1;
				   cool_air_1_sec = 0;	
				}
			}
		}
		else 
		{
			COOL_AIR = 1;
			cool_air_counter++;
			if(cool_air_counter > cool_air_open_time)
			{
				   cool_air_action_flag = 0;
				   cool_air_counter = 0;
				   cool_air_1_sec = 0;		
			}
		}
		
	}
	else if( sys.status != CHECKI05) //不能影响到测试状态的输出
	 COOL_AIR = 0;
	
	#endif
	//准备状态或是测试状态下，打开模板识读，并且没进过RUN
	if( (auto_function_flag == 1) && (return_from_setout==0) && ( (sys.status == READY)||(sys.status == CHECKI03)  )  )
	{
			tmp_pattern = identify_pattern();//对于RFID方式来说，只是返回了全局变量serail_number，其他什么都没做
			if( (formwork_identify_device <2)&&(tmp_pattern!=0) )
		    {
				if( (identify_mode == 0)||((identify_mode == 1)&&(foot_flag == 0)) )
			    {
					pattern_id_delay++;
					if( pattern_id_delay >= 50) 
					{
						pattern_id_delay = 0;
				      	tmp_pattern2 = identify_pattern(); 
						if(  (tmp_pattern2 !=0)&&(tmp_pattern == tmp_pattern2) )
					    {
					         if( last_pattern_number != tmp_pattern2)
					         {
								    pattern_change_counter = 0;
					   				pattern_change_flag = 1;//find new pattern
					   				pattern_number = tmp_pattern2;
					   				last_pattern_number = tmp_pattern2;
									id_alarm_flag = 1;
			                        //turnon_buz();
				              }
						}
					}
				}
			}
			else if( (formwork_identify_device <2)&&(tmp_pattern==0) )
			{
				if( pattern_change_flag == 0)
				{
					last_pattern_number = 0;
				}
			}
	
			if ((formwork_identify_device >= 2)&&(tmp_pattern!=0))
			{
				if( pattern_change_flag == 0)//not find new pattern
				{	
					if( (tmp_pattern != last_pattern_number)||(one_step_run_flag == 1) )
					{
						pattern_change_flag = 1;//find new pattern
						pattern_number = tmp_pattern;
						serail_number = 0;	
						pattern_chage_recover_counter = 0;
						last_pattern_number = tmp_pattern;
					}
				}				
				else
				{
					pattern_chage_recover_counter++;
					if( pattern_chage_recover_counter >= 3000)//3000ms后，才能重新识别模板
					{
						pattern_change_flag = 0;
						pattern_chage_recover_counter = 0;
					}
					
				}				
			}
	} 
	//--------------------------------------------------------------------------------------
  	//  call system watch function
  	//--------------------------------------------------------------------------------------
	
	if(sys.error == 0)
		sys.error = sys_watch();//系统保护功能
  	if(sys.error != 0)
	{
    	sys.status = ERROR;
		StatusChangeLatch = ERROR;		
	}
	
	#if NEW_LASER_DEVICE && ENABLE_LASER_CUTTER
	if( laser_power_on_flag == 1)
	{
		if(laser_power_on_counter < 120000)
		{
			laser_power_on_counter++;
		}
		else//超过120s即2min后关闭总点源
		{
			laser_power_on_flag = 0;
			laser_power_on_counter = 0;
			LASET_POWER_SWITCH = 0;       //关闭激光总电源
			first_start_flag = 0;
		}
		//检测激光电源信号，已经不使用了
		temp16 = (ad1 & 0x03ff);
		if(temp16 > 500)
		{
	  	   //laser_power_error_flag = 1;
		}
		else
			laser_power_error_flag = 0;
	}
	/*
	temp16 = (ad1 & 0x03ff);
	temp16 -= 512;
	if(temp16 < 0)
	   temp16 = 0;
	da1 = temp16 >>1;
	*/
	#endif
	
	if( (ENABLE_BOBBIN_CASE_FUN == 1)&&( bobbin_case_enable == 1) )//#if ENABLE_BOBBIN_CASE_FUN
	{
		if( BOBBIN_CASE_SWITCH == 1)
		{
			if( bobbin_case_switch_flag == 0)
			{
				bobbin_case_switch_counter++;
				if( bobbin_case_switch_counter > 100 )
				{
					bobbin_case_switch_flag = 1;
				}
			}
			else
			    bobbin_case_switch_counter = 0;
		}
		else
		{
			bobbin_case_switch_counter = 0;
			bobbin_case_switch_flag = 0;
		}
	}
	//#endif
	/*
	  数字单踏板的信号采集
	*/		
	if((DVA==0)&&(DVB==0))
	{
        pedal_state = 1;					// foot down
	}
	else if((DVA==0)&&(DVB==1))
	{
        pedal_state = 2;					// start
	}
	else if((DVA==1)&&(DVB==0))
	{
        pedal_state = 0;  					// pedal is in normal state 
	}		
	else
		pedal_state = 0;
		
		
/**************************************************
 * 功能：100ms计数，及时响应面板
 * 2015-11-04
 **************************************************/
 	if(flag_start_waitcom == 1)
 	{
 		if(++counter_wait_com > 30)
 		{
			flag_wait_com = 1;
			flag_start_waitcom = 0;
 		}
 	}
 	else
 	{
		counter_wait_com = 0;
		flag_wait_com = 0;
 	} 	
	
	#if CHANGE_DOUBLE_FRAMEWORK   //双换模板
	
	if( AUTO_LEFT_RUNNING_SWITCH == 1 )//左侧启动开关
	{
		if( left_start_lock_flag == 0)
		{
			left_start_counter++;
			if( left_start_counter>=50 ) //50ms
			{
				left_start_lock_flag = 1;
				left_start_counter = 0;		
				left_quest_running = 1;
				//SUM = 1;
			}
		}
	} 
	else 
	{
		left_start_lock_flag = 0;
		left_start_counter = 0;
	}

	if( AUTO_RIGHT_RUNNING_SWITCH == 1 )//右侧启动开关
	{
		if( right_start_lock_flag == 0)
		{
			right_start_counter++;
			if( right_start_counter >= 50 ) //50ms
			{
				right_start_lock_flag = 1;
				right_start_counter = 0;
		        right_quest_running = 1;
				//SUM = 1;
			}	
		}
	} 
	else 
	{
		right_start_lock_flag = 0;
		right_start_counter = 0;
	}

	
	#endif
		
}

/**
  * @函数功能 定时器B3中断程序，产生1毫秒时间基准
  * @参数 
  * @返回值 
  */
void tb3_int(void)
{	
	ms_counter++;	
	if(wipe_start_flag == 1)
	{
		wipe_time++;
	}
	else if(wipe_start_flag == 0)
	{
		wipe_time = 0;
	}
	inpress_time++;	    	  		
}
/**
  * @函数功能 定时器B4中断程序，产生微秒级时间基准
  			  同时用于将画笔和激光切刀等的插补算法计算出的运动序列通过此函数发给DSP驱动器
  * @参数 
  * @返回值 
  */
void tb4_int(void)
{
	
	UINT8 coder;
	us_counter++;
#if INSERPOINT_ENABLE//执行XY插补算法，从缓存区tra1_buf[]中取出动作序列
	if( laser_cutter_aciton_flag == 1 )
	{
		if( rec1_total_counter > 0 )//缓冲区不空
		{
			coder = tra1_buf[tra1_ind_r];
			tra1_ind_r = (tra1_ind_r+1 )%250;
			rec1_total_counter --;
			switch(coder)
			{
				case 1:
					  timer_x = 1;
					  movestep_x(-1);
				break;
				case 2:
					  timer_x = 1;
					  movestep_x(1);
				break;
				case 3:
					  timer_y = 1;
					  movestep_y(-1);
				break;
				case 4:
					  timer_y = 1;
					  movestep_y(1);
				break;
			}
		}
	}
	#endif				    	  		
}


//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
