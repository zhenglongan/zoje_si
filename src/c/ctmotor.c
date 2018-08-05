//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: 210E_machine_controller 
//  File Name : ctmotor.c
//  Description: Core program to control the catch_thread motor
//  Version    Date     Author    Description
//  0.01     17/12/07     zx       created
//  0.02     18/01/08     lm       modify
//  0.03     27/03/08     zx       modify
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include   "..\..\include\sfr62p.h"        // M16C/62P special function register definitions  
#include   "..\..\include\typedef.h"       // Data type define                                
#include   "..\..\include\common.h"        // Common constants definition           
#include   "..\..\include\variables.h"     // External variables declaration         
#include   "..\..\include\delay.h"         // delay time definition  
#include   "..\..\include\action.h"
#include   "..\..\include\stepmotor.h"
/*
  ץ�߻���������
  ץ�ߵ�� ------60���� 0.9�����
  ԭ�㴫���� ----TORG
  λ�ô����� ----TSENS  0-��ʾû����
  T_DIR = 1 511   +�ż�  ������߷����� �����⣩
  T_DIR = 0 -511  -�ż�  ���ͷ������   �����
  ԭSLA7078�����1.8/8ϸ�֣�0.225�� 
  ��ԭ�㵽���λ��һ��22��
*/

void init_ct(void)
{
	
}

/**
  * @�������� ץ�ߵ����ԭ�㶯��
  * @���� ��     
  *
  
  */
void go_origin_ct(void)
{
	UINT8 i;
    UINT16 run_counter;
	if(TORG)															
	{
		run_counter = 0;
		while(TORG)						
		{
		    #if SUPPORT_NEW_DRIVER
			movestep_ct(1,1);
			#else
			movestep_ct(511,0);//1.0 ��������ת���˳�����
			#endif
			run_counter++;
			if(run_counter > 400)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				if( sys.error == 0)
      			    sys.error = ERROR_28;
      			return;
			}
			delay_ms(2);	
		}
		for(i=0;i<4;i++)//2.0 ���߼���
		{
			#if SUPPORT_NEW_DRIVER
			movestep_ct(1,1);
			#else
			movestep_ct(511,0);
			#endif
			delay_ms(2);
		}
		
	}
	
	run_counter = 0;
	while(!TORG)	//������û���ϣ�Ҫ��������ԭ��
	{
			#if SUPPORT_NEW_DRIVER
			movestep_ct(-1,1);
			#else
			movestep_ct(-511,0);	//������ת
			#endif
			run_counter++;
			if(run_counter > 400)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				if( sys.error == 0)
      		        sys.error = ERROR_28;     		
      			return;
			}
			delay_ms(2);	
	}
	
	delay_ms(20);
	clamp_flag = 0;                                
	allct_step = 0;
	tb1_flag = 0;      
}
//--------------------------------------------------------------------------------------
//  Name:		clamp_out
//  Parameters:	None
//  Returns:	None
//  Description: clamp out  
//--------------------------------------------------------------------------------------
void clamp_out(void)
{	 
    UINT8 i;
	UINT16 run_counter;
	run_counter = 0;
	for(i=0;i<2;i++)
	{
		while(TSENS == 0)												
		{
				#if SUPPORT_NEW_DRIVER
					movestep_ct(1,2);
				#else
					movestep_ct(511,0);	//������ת
				#endif
				run_counter++;
				if(run_counter > 80)
				{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
					if( sys.error == 0)
	      		        sys.error = ERROR_23;     		
	      			return;
				}
				delay_ms(3);	
		}  
		delay_ms(5);
	}             
	clamp_flag = 1;             // clamp out
}
//--------------------------------------------------------------------------------------
//  Name:		clamp_in
//  Parameters:	None
//  Returns:	None
//  Description: clamp in  
//--------------------------------------------------------------------------------------
void clamp_in(void)
{	   
    go_origin_ct();
	clamp_flag = 0;             // clamp in
}
/**
  * @�������� ץ�������ץ������
  * @���� ��     
  ��һ�룺800rpm ��Ȧ75ms ��281�ȿ�ʼ������
  �ڶ��룺       78�ȿ�ʼ����
  */
void clamp_backstep1(void)
{	 
	#if DOUBLE_X_60MOTOR
    movestep_ct(-ct_holding_steps,3);//3
	movect_angle = 597;  
	#else
	movestep_ct(-ct_holding_steps,4);
	movect_angle = 284;
	#endif
	clamp_stepflag = 2;      
}
//--------------------------------------------------------------------------------------
//  Name:		clamp_backstep2
//  Parameters:	None
//  Returns:	None
//  Description: clamp back step 2
//--------------------------------------------------------------------------------------
void clamp_backstep2(void)
{	     
	INT8  tmp;
#if DOUBLE_X_60MOTOR
    //tmp = 5;
	//movestep_ct(-tmp,4);
	go_origin_ct();
#else	
	tmp = 17;
	movestep_ct(-tmp,4);//-12
#endif    
	movect_angle = 71;       //71---25 degree 
	clamp_flag = 0;
	clamp_stepflag = 0;
}

//--------------------------------------------------------------------------------------
//  Name:		move_ct
//  Parameters:	None
//  Returns:	None
//  Description: move ct motor
//--------------------------------------------------------------------------------------
void move_ct(void)
{	
	INT16 temp16,tmp;	
	
	switch(clamp_stepflag)  
	{
		
		case 1:  //��һ�׶����ء�ץ��  0-53   u34:-10~0
				 
				 if ( SUPPORT_CS3_FUN == 1)
				 	output_cs3(0x10,0x10); //x32.2
				 else
				 	BLOW_AIR = 1;
				 ct_bump_counter = 0;
				 ct_bump_action_flag = 1;
		         tmp = u34;
				 tmp = 995 + tmp*57;	//995=>349
				 temp16 = motor.angle_adjusted;
	
	             while( temp16 <= tmp )     
	             {
					if(sys.status == POWEROFF)   
					   return;
	                temp16 = motor.angle_adjusted;
	             }
	             clamp_backstep1();    
				 
             break;            
		case 2:  //�ڶ��׶� �ſ�
				 
			 if( stitch_counter >= u33)
			 {
				 temp16 = motor.angle_adjusted;
	             while( temp16 <= 284 )    
	             {
	                temp16 = motor.angle_adjusted;
	             }
				 clamp_backstep2();  	
			 }
             break;
	    default: 
			 break;
	}  
}

//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
