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
  * @�������� ������ϵͳ״̬�����
  * @���� ��
  * @����ֵ ��
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

//����4��6��8��9��10��11��12��13��14��15��16
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
  	while(connect_flag == 0)//ֻ�еȵ�PARAָ����������·��Ժ����زſ��Լ�����������
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
	//�ϵ簴��ͣ�󣬽���������ֱ����ԭ������޷���READY�Ĳ���������
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
					special_encoder_mode = 1;//�ϵ�ͬʱ����ͣ��DVB
				}
			}
		}
	}
	SNT_H = 0;    
	SNT_ON = 0;     
	delay_ms(300);	        //��ʱ��ϵͳӲ������һ������ʱ��
	
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
	ta0ic = TA0_IPL;		//ȷ����ʱ������������ϵͳ��Ҫ������������
	version_check();  
  	//--------------------------------------------------------------------------------------
  	// ϵͳ����״̬��
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
	
	//����2��3��4��5��6��8��9��10��11��12��13��14��15��16��17��18��19��20��21��22��23��24��25��26��
	//27��28��29��30��31��33��34��35��36��37��38��39��40��41��42��44��55
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
		RFID_initial();//��ʼ��MFRC522��ͬʱ����MFRC522ͨ�Ų�����Ϊ115200
		init_uart1_RC522();//��UART1�Ĳ����ʸĵ�115200���Ա��MFRC522ƥ�䣬��ǰ������Ĭ��9600
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
				//����ɨ��ʹ�õ��Ǵ���1,Ҫʹ����Ӧ���жϡ��趨�ж����ȼ�		
				if((sys.status == READY)&&(formwork_identify_device >=2)&&(auto_function_flag == 1))
				{
				  re_u1c1 =1;
				  rec1_com();
				}
				
			  #endif
			  
			#endif
		#endif
		/*
		������Ϊ��Ӧ����K141 ���رմ���״̬�µ����
		EXTEND------��ͷ���ϡ�����1��
		*/
		if ( (sys.status != ERROR)&&(alarm_output_enable >= 1) )
		{
			RED_ALARM_LED = 0;
		}
		if( sys.status != DOWNLOAD)
		    rec_com();   
	
		/*
		��ģ������ԱȽ���Ҫ��״̬��
		READY,RUN,FINISH,SETOUT,PREEDIT,EDIT
		*/
	  	switch(sys.status)
    	{
	      	case FREE:      free_status();      break;//����״̬���޷���������
	      	case READY:     ready_status();     break;//����״̬״̬	
	      	case RUN:       run_status();       break;//�������й��̣����Ὺʼת��
	      	case ERROR:     error_status();     break;//����״̬       	      
	      	case PREWIND:   prewind_status();   break;//׼������״̬	
	      	case WIND:      wind_status();      break;//���߽���״̬�����Ὺʼת��		
	      	case INPRESS:   inpress_status();   break;//������ѹ��״̬	
	      	case POWEROFF:  poweroff_status();  break;//����״̬��������״̬��                               
	      	case SINGLE:    single_status();    break;//�����Է�״̬    	
	      	case MANUAL:    manual_status();    break;//�ֶ��ƿ�״̬    		
	      	case SETOUT:    setout_status();    break;//��������״̬   	
	      	case EMERSTOP:  emerstop_status();  break;//����ͣ��״̬������ͣ��ť��ͣ�������У�  
	      	case PREEDIT:   preedit_status();   break;//�������༭״̬�����ƿ���״̬��   	
	      	case EDIT:      edit_status();      break;//�����༭״̬���ƿ�������꣩ 	 		
	      	case NOEDIT:    noedit_status();    break;//�༭�л�����״̬���޶�����	
	      	case FINISH:    finish_status();    break;//���ƽ���״̬		
	      	case NEEDLE:    needle_status();    break;//����״̬		
	      	case WAITOFF:   waitoff_status();   break;//�ȴ��ػ�״̬����������ѹ�ź���ʾ�ص磩		
	      	case TRIM:      trim_status();      break;//����״̬		
	      	case SLACK:     slack_status();     break;//��״̬���趨������������������		
	      	case CHECKI03:  checki03_status();  break;//�����źż��״̬		
	      	case CHECKI04:  checki04_status();  break;//���������״̬		
	      	case CHECKI05:  checki05_status();  break;//����źż��״̬		
	      	case CHECKI06:  checki06_status();  break;//XY�ƿ���״̬		
	      	case CHECKI07:  checki07_status();  break;//ѹ�ż��ߵ�����״̬		
	      	case CHECKI08:  checki08_status();  break;//ץ�ߵ�����״̬		
	      	case CHECKI10:  checki10_status();  break;//��ѹ�ŵ�����״̬		
	      	case EMERMOVE:  emermove_status();  break;//��ͣ��Ӧ״̬������ͣ���󣬿�ǰ���ƶ���	
			case DOWNLOAD:  download_status();  break;//���س�������״̬
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
  * @�������� ��ʱ��0 1�����жϳ���
  * @���� ��
  * @����ֵ ��
  */
void ta0_int(void)
{
	UINT16 tmp_pattern,tmp_pattern2;
	INT16 temp16; 
	flag_1ms = 1;//1�����ʱʱ�䵽��־
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
	
	motor_control();//����Ŀ��ƺ���
	
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
	if( ct_bump_action_flag == 1)//��ʱ�ر�ץ������ʱ��
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
	 ͬ�����ζ���ȷ�����ƹ����У��������ζ���ʱ�䲻�ܳ��ָ��ǵ������ȷ��ǰһ��ʱ�����꣬�ٷ���һ��ָ�
	 movestep_x_flag �� movestep_y_flag ��XY����ֿ�����ʱ�ĸ��Զ���ʱ��ֵ��
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
	if( blow_air_flag == 1 )//�������ܶ�ʱ�ر�
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
	��ѹ������ʱ��������������ɿ�����
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
	  �ڼ��߹����е��������򿪿��ƹ���
	  �������������ƹ��ܺ��Ȱ����������DA0ֵ����100���룬Ȼ���趨�ı��ֵ�����ά��״̬
	  1�����Ժ��Զ��ر�DA0�����
	*/	
	if(DAActionFlag==1)
	{
		DAActionCounter++;
		if(DAActionCounter >= 100)
		{
			//DAActionFlag = 0;
			da0 = release_tension_current;//�������ɿ�
		}
		if(DAActionCounter >= 300)
		{
			DAActionFlag = 0;
			da0 = 0; //�������н�
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
	  ģ�����ע�ͱõĿ��ƣ��������� COOL_AIR-----����������2��
	  ע�ͻ�������������һ�������������ע�ͱã���������תʱ��ȡ�ͳ�������Ϳ�ʼ��������ע�͡�
	  ������һ��ͨ��һ���ܺͳ�������������ע����������Ż��ͳأ������γ���һ������ѭ����
	  ��������Կ���������Ĭ��״̬�Ǵ򿪣���ʱ�൱������״̬����һ��ʱ�䣬�������Ϲرգ���ʱ��
	  �ͻ����������൱�ڸ�����ע��һ���ͣ�����һС��ʱ���ٹرջָ�����״̬��
	  cool_air_action_flag -----  ������״̬ 0-��ʾ�رգ���Ӧ������״̬��1--��ע��״̬��
	  cool_air_close_time  -----  ע�͵ļ��ʱ�䣬������������
	  cool_air_open_time  ------  ע�͵�ʱ�䣬���������ر�״̬ʱ�䣬������������
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
	else if( sys.status != CHECKI05) //����Ӱ�쵽����״̬�����
	 COOL_AIR = 0;
	
	#endif
	//׼��״̬���ǲ���״̬�£���ģ��ʶ��������û����RUN
	if( (auto_function_flag == 1) && (return_from_setout==0) && ( (sys.status == READY)||(sys.status == CHECKI03)  )  )
	{
			tmp_pattern = identify_pattern();//����RFID��ʽ��˵��ֻ�Ƿ�����ȫ�ֱ���serail_number������ʲô��û��
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
					if( pattern_chage_recover_counter >= 3000)//3000ms�󣬲�������ʶ��ģ��
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
		sys.error = sys_watch();//ϵͳ��������
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
		else//����120s��2min��ر��ܵ�Դ
		{
			laser_power_on_flag = 0;
			laser_power_on_counter = 0;
			LASET_POWER_SWITCH = 0;       //�رռ����ܵ�Դ
			first_start_flag = 0;
		}
		//��⼤���Դ�źţ��Ѿ���ʹ����
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
	  ���ֵ�̤����źŲɼ�
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
 * ���ܣ�100ms��������ʱ��Ӧ���
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
	
	#if CHANGE_DOUBLE_FRAMEWORK   //˫��ģ��
	
	if( AUTO_LEFT_RUNNING_SWITCH == 1 )//�����������
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

	if( AUTO_RIGHT_RUNNING_SWITCH == 1 )//�Ҳ���������
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
  * @�������� ��ʱ��B3�жϳ��򣬲���1����ʱ���׼
  * @���� 
  * @����ֵ 
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
  * @�������� ��ʱ��B4�жϳ��򣬲���΢�뼶ʱ���׼
  			  ͬʱ���ڽ����ʺͼ����е��ȵĲ岹�㷨��������˶�����ͨ���˺�������DSP������
  * @���� 
  * @����ֵ 
  */
void tb4_int(void)
{
	
	UINT8 coder;
	us_counter++;
#if INSERPOINT_ENABLE//ִ��XY�岹�㷨���ӻ�����tra1_buf[]��ȡ����������
	if( laser_cutter_aciton_flag == 1 )
	{
		if( rec1_total_counter > 0 )//����������
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
