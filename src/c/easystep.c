#include "..\..\include\sfr62p.h"         // M16C/62P special function register definitions
#include "..\..\include\typedef.h"        // Data type define
#include "..\..\include\common.h"         // Common constants definition
#include "..\..\include\stepmotor.h"      // stepper motor function
#include "..\..\include\variables.h"      // External variables declaration
#include "..\..\include\easystep.h"
#include "..\..\include\delay.h"          // delay time definition

#if MULTIPULE_IO_ENABLE == 1


UINT8                 LineCode[206];  //ϵͳ�Ĵ����л�����
InPut_TypeDef         SystemInput[Coden+1];	 //ϵͳ�����뻺����  ����+������
UINT8		  		  *g_ProgramLine;			 //ϵͳ����ִ��ָ��
UINT8				  Total_Program_Lines;		 //����������

void app_GetProgramFromEeprom(void);
UINT16 get_input_from_io(UINT8 port,UINT8 reg);
UINT16 get_input_from_register(UINT8 port,UINT8 reg);
void set_output_to_IO(UINT8 port,UINT8 reg,UINT8 data);
void set_output_to_register(UINT8 port,UINT8 reg,UINT8 data);
void send_motor_command(UINT8 port,int data,UINT16 time);

UINT8 input_is_high(UINT8 number)
{
	UINT8 state;
	if( number <Xn )//������
	{
		switch(number)
		{
			case xorg:
					state 	= XORG;
				break;
				case porg:
					state 	= PORG;
				break;
				case corg:
					state 	= CORG;
				break;
				case iorg:
					state 	= IORG;
				break;
				case th_brk:
					state 	= TH_BRK;
				break;
				case adtcsm:
					state 	= ADTCSM;
				break;
				case yorg:
					state 	= YORG;
				break;
				case psens:
					state 	= PSENS;
				break;
				case csens:
					state 	= CSENS;
				break;
				case sfsw:
					state 	= SFSW;
				break;
				case pause_sig:
					state 	= PAUSE;
				break;
				case dva:
					state 	= DVA;
				break;
				case dvb:
					state 	= DVB;
				break;
				case dvsm:
					state 	= DVSM;
				break;
				default:
					state = SystemInput[number].State;	
								
				break;
		}
	}	
	else
	{
		state = SystemInput[number].State;
	}
			
	if( SystemInput[number].State != state)//����ϵͳ״̬�ͱ���״̬
	{
		if( state == 1 )//��ɸߵ�ƽ��
			SystemInput[number].Operat = State_POS_EDG;
		else
			SystemInput[number].Operat = State_NEG_EDG;
		SystemInput[number].State = state;
	}
	return state;
}

void set_programio_level(UINT8 number,UINT8 out_data)
{
	UINT8 i;
	UINT16 ret;
	if( number < Yn)//������
	{
		switch(number)
		{
			case fr_on://1
					FR_ON 	= out_data;
			break;
			case lm_air:
					LM_AIR 	= out_data;
			break;
			case fk_off:
					FK_OFF 	= out_data;
			break;
			case r_air:
					R_AIR 	= out_data;
			break;
			case l_air:
					L_AIR 	= out_data;
			break;
			case t_clk:
					T_CLK 	= out_data;
			break;
			case t_dir:
					T_DIR 	= out_data;
			break;
			case fl:
					FL 		= out_data;
			break;
			case t_half:
					T_HALF 	= out_data;
			break;
			case t_dir_extend:
					EXTEND 	= out_data;
			break;
			case fa:
					FA 	= out_data;
			break;
			case fw:
					FW 	= out_data;
			break;
			case da1v:
					da1	= out_data;
			break;
		}
	}
	else if( number == Yn )
	{
		for( i = logic_input1;i< Coden;i++)//�����������Ĵ���������
		{
			 SystemInput[i].State = 0;
			 SystemInput[i].Operat = 0;
		}
	}
	else 
	{
		if(  number< Ylogic )
		{
			if( (out_data == 1 )&&(SystemInput[number].State == 0) )//��ɸߵ�ƽ��
				 SystemInput[number].Operat = State_POS_EDG;
			else if( (out_data == 0 )&&(SystemInput[number].State == 1) )
				 SystemInput[number].Operat = State_NEG_EDG;
			SystemInput[number].State = out_data;
			
			if( SystemInput[auto_run_fun].State == 1)
			{
			    autosewing_allow_working = 1;
				autosewing_allset_flag = 1;
				SystemInput[auto_run_fun].State = 0;
			}
		}	
		else
		{	
			if( (number >= sc0714_output1)&&( number <= sc0714_output30))//дIO�Ĵ���״̬	
				set_output_to_IO(SPI_STM32_PORT,number-sc0714_output1+1,out_data);
			
			else if( (number >= sc0714_wait_signal1)&&(number <= sc0714_wait_signal30) )//�ȴ�����״̬
			{
				ret = get_input_from_io(SPI_STM32_PORT,number-sc0714_wait_signal1);
				while( ret != out_data )
				{
					delay_ms(10);
					ret = get_input_from_io(SPI_STM32_PORT,number-sc0714_wait_signal1);
				}
			}
			else if( (number >= sc0714_register1)&&( number <= sc0714_register30))//дSC0714�ڲ��Ĵ���״̬
			{
				set_output_to_register(SPI_STM32_PORT,number-sc0714_register1,out_data);
			}
		}
	}
	
}

void set_system_status_condiction(UINT8 number,UINT8 out_data)
{
	if( (out_data == 1 )&&(SystemInput[number].State == 0) )//��ɸߵ�ƽ��
		 SystemInput[number].Operat = State_POS_EDG;
	else if( (out_data == 0 )&&(SystemInput[number].State == 1) )
		 SystemInput[number].Operat = State_NEG_EDG;
	SystemInput[number].State = out_data;
}


//ϵͳ"�δ�"�ж� ��������
void app_EasyStepTick(void)
{
    UINT8 i,state;    
	for(i=0;i<Xn;i++)
    {
   	   state = input_is_high(i);//����X����״̬���������������س�������µģ�������ɨ������õ���IO���ŵı仯
	}	
}

//InPutFunBoolUnit ��������
//�β��Ǳ���������һ������

UINT8 InPutFunBoolUnit(InPut_TypeDef *pInPut)
{
  UINT8 ret = DEF_FALSE;
  UINT16 dat,Number;

  Number = pInPut->Number;	//ϵͳ������  
  
  if( Number > Coden )//����SPI����ɨ��
  {
	  dat = get_input_from_register(SPI_STM32_PORT,pInPut->State);//��ȡSPI�ļĴ���
	  if( dat != 0 )
	  	  return DEF_TRUE;
	  else
	  	  return DEF_FALSE;
  }
  else
  {
	  ret = DEF_FALSE;
	  switch(pInPut->State)	    //��������Ҫʲô״̬
	  {
	  		case State_POS_LEV://�ߵ�ƽ
			  	 if( input_is_high(Number) )
				   {
				       ret = DEF_TRUE;
				   }
			break;
	  	    case State_POS_EDG://������			
			  	 if( input_is_high(Number) )//�Ѿ��Ǹߵ�ƽ��
				   {
			  			if( SystemInput[Number].Operat == State_POS_EDG)//ȷ����������
						{
						    ret = DEF_TRUE;
							//if( ( Number >= logic_input1 )&&(Number <= logic_input20) )
							//	SystemInput[Number].State = 0;
							//SystemInput[Number].Operat = 0;
						}
				   }
	    	break;
	  		case State_NEG_LEV://�͵�ƽ
	    		 if( input_is_high(Number)==0 )
				   {
				       ret = DEF_TRUE;
				   }	
	    	break;
	  		case State_NEG_EDG://�½���
	    		 if( input_is_high(Number)==0 )
				   {
			  			if( SystemInput[Number].Operat == State_NEG_EDG)
						{
						    ret = DEF_TRUE;
							//if( ( Number >= logic_input1 )&&(Number <= logic_input20) )
							//	SystemInput[Number].State = 1;
							//SystemInput[Number].Operat = 0;
						}
				   }
	    	break;
	  }  
  }
  return ret;
}

void get_one_input(InPut_TypeDef *in)
{
	in->Number = *g_ProgramLine++;
	in->State  = *g_ProgramLine++;
	in->Operat = *g_ProgramLine++;		
}
void get_one_output(OutPut_TypeDef *out)
{
	out->Number = *g_ProgramLine++;
	out->Logic  = *g_ProgramLine++;
}

void app_ProcessSingleLine(void)
{
	UINT8 ch,ret,flag;
	InPut_TypeDef input;
	OutPut_TypeDef output,output2;
	UINT16 dly;
	int data;
	
	get_one_input(&input);
	flag = 1;
	ret = InPutFunBoolUnit(&input);		//��һ������
	while( flag == 1 )
	{
		if( input.Operat == Operat_END )
		{
				if( ret == 1)//�������������㣬��ʼ�������
				{
					while(flag == 1)
					{
						get_one_output(&output);
						if( (output.Number == 0xff)&&(output.Logic == 0xff) )//�������������
							break;
						
						if( (output.Number >= motor_output1 ) && (output.Number <= motor_output8) )
						{
							  get_one_output(&output2);//���Ҫ�õ��ڶ�������ź�
							  dly  = output2.Number;
							  dly  = dly*256 + output2.Logic;		 
						      data = dly;
							  dly = output.Logic;	 //ʱ��ϵ��*8
							  dly = dly << 3;; 
							  send_motor_command(output.Number,data,dly);	
						}
						else if( output.Number == clear_input)
						{
							 set_system_status_condiction(output.Logic,0);
						}
						else
						{
							  set_programio_level(output.Number,output.Logic&0x01);
							  dly = (output.Logic>>1);
							  if( dly > 0)
							  {
							      delay_ms(dly*10);
							  }							  
						}	  
					}
				}
				else //���������㣬Ҫ��ת����һ�����뿪ʼ
				{
					while(flag == 1)
					{
						get_one_output(&output);
						if( (output.Number == 0xff)&&(output.Logic == 0xff) )//�������������
							break;
					}
				}
				flag = 0;
		}
		else
		{
			   if(	input.Operat == Operat_AND)
			   {
	           	  	get_one_input(&input);
				  	ret = ret && InPutFunBoolUnit(&input);
			   }
	           else if(input.Operat == Operat_OR)
			   {
				    get_one_input(&input);
					ret = ret || InPutFunBoolUnit(&input);	
			   }
		}
	}
	
}
//������ݰ��д���
void app_EasyStepLineProcess(void)
{	
	UINT8 total_cnt;
	if( Total_Program_Lines != 0)
	{
		g_ProgramLine = LineCode;		//ָ����뻺����
		total_cnt = Total_Program_Lines;//�û�ָ��������
		while(  total_cnt > 0)
		{
			app_ProcessSingleLine();
			total_cnt--;
		}
	}	
}

//��EEPROM����ȡ�û��ı�̴���
void app_GetProgramFromEeprom(void)
{
	UINT8 i	;
	//read_para_group(400,svpara_disp_buf,205);
	Total_Program_Lines = 0;
	
	if( (svpara_disp_buf[204]!=0x0 ) && (svpara_disp_buf[204]!=0xff ) )
	{
		Total_Program_Lines = svpara_disp_buf[204];
		for(i=0;i<205;i++)
		{
			LineCode[i] = svpara_disp_buf[i];
		}
	}
}
//��ʼ��
void app_EasyStepInit(void)
{
	UINT8 i;
	for(i=0;i<Coden;i++)
	{
	    SystemInput[i].Operat = 0;
	    SystemInput[i].State = 0;
	}
	app_GetProgramFromEeprom();
}

void active_multipule_program_register(UINT8 port,UINT8 reg)    
{
	 UINT8 k;
	 while(spi_flag > 0);	 
	 send_dsp_command(port,DSP_COMMADN_WRITE_REGISTER);//д�Ĵ���	    
	 send_dsp_command(port,0x0);//class
	 send_dsp_command(port,reg);//number
	 send_dsp_command(port,0x1);//data
} 
UINT16 get_input_from_io(UINT8 port,UINT8 reg)
{
	while(spi_flag > 0);	
	send_dsp_command(port,DSP_COMMADN_READ_REGISTER);//���Ĵ���	    
	send_dsp_command(port,0x0);//class
	send_dsp_command(port,reg);//number
	send_dsp_command(port,0x1);//data
	return (UINT16)recieve_x.word;
}
UINT16 get_input_from_register(UINT8 port,UINT8 reg)
{
	while(spi_flag > 0);	
	send_dsp_command(port,DSP_COMMADN_READ_REGISTER);//���Ĵ�������	    
	send_dsp_command(port,0x40);//class
	send_dsp_command(port,reg);//number
	send_dsp_command(port,0x1);//data
	return (UINT16)recieve_x.word; 
}
UINT16 get_output_from_IO(UINT8 port,UINT8 reg)
{
	while(spi_flag > 0);	
	send_dsp_command(port,DSP_COMMADN_READ_REGISTER);//�����IO��	    
	send_dsp_command(port,0x20);//class
	send_dsp_command(port,reg);//number
	send_dsp_command(port,0x1);//data
	return (UINT16)recieve_x.word; 
}

void set_output_to_IO(UINT8 port,UINT8 reg,UINT8 data)
{
	while(spi_flag > 0);	
	send_dsp_command(port,DSP_COMMADN_WRITE_REGISTER);//д���IO��	    
	send_dsp_command(port,0x20);//class
	send_dsp_command(port,reg);//number
	send_dsp_command(port,data);//data
}
void set_output_to_register(UINT8 port,UINT8 reg,UINT8 data)
{
	while(spi_flag > 0);	
	send_dsp_command(port,DSP_COMMADN_WRITE_REGISTER);//д���IO��	    
	send_dsp_command(port,0x40);//class
	send_dsp_command(port,reg);//number
	send_dsp_command(port,data);//data
}
void send_motor_command(UINT8 port,int data,UINT16 time)
{
			switch(port)
			{
				case motor_output1://DSP1A
				break;
				case motor_output2:
				break;
				case motor_output3://DSP1B
				break;
				case motor_output4:
				break;
				case motor_output5://DSP2A
					//yj_quickmove(time,data);
				break;
				case motor_output6:
					//yj_quickmove(time,-data);
				break;
				case motor_output7://DSP2B
					
				break;
				case motor_output8:
				break;
			}	
}

void read_all_io_input(void)
{
	UINT8 temp8,i;
	
	temp8 = 0;
	for(i = 0; i < 8; i++)
	{
		if(get_input_from_io(SPI_STM32_PORT,i + 24))
		   temp8 |= (UINT8)(1<<i);
	}
	stm32_input[0] = temp8;
	temp8 = 0;
	for(i=0; i<8; i++)
	{
		if(get_input_from_io(SPI_STM32_PORT,i + 16))
		   temp8 |= (UINT8)(1<<i);
	}
	stm32_input[1] = temp8;
	temp8 = 0;
	for(i=0; i<8; i++)
	{
		if(get_input_from_io(SPI_STM32_PORT,i + 8))
		   temp8 |= (UINT8)(1<<i);
	}
	stm32_input[2] = temp8;
	temp8 = 0;
	for(i=0; i<8; i++)
	{
		if(get_input_from_io(SPI_STM32_PORT,i))
		   temp8 |= (UINT8)(1<<i);
	}
	stm32_input[3] = temp8;
}

void read_all_io_output(void)
{
		UINT8 temp8,i,index,dat;
	index = 0;
	temp8 = 0;
	for(i = 0; i < 8; i++)
	{
		dat = get_output_from_IO(SPI_STM32_PORT,i + 24);
		spi_out_status[i + 24] = dat;
		if(dat == 1)
		   temp8 |= (UINT8)(1<<i);
	}
	stm32_output[0] = temp8;
	temp8 = 0;
	for(i=0; i<8; i++)
	{
		dat = get_output_from_IO(SPI_STM32_PORT,i + 16);
		spi_out_status[i + 16] = dat;
		if(dat == 1)
		   temp8 |= (UINT8)(1<<i);
	}
	stm32_output[1] = temp8;
	temp8 = 0;
	for(i=0; i<8; i++)
	{
		dat = get_output_from_IO(SPI_STM32_PORT,i + 8);
		spi_out_status[i + 8] = dat;
		if(dat == 1)
		   temp8 |= (UINT8)(1<<i);
	}
	stm32_output[2] = temp8;
	temp8 = 0;
	for(i=0; i<8; i++)
	{
		dat = get_output_from_IO(SPI_STM32_PORT,i );
		spi_out_status[i] = dat;
		if(dat == 1)
		   temp8 |= (UINT8)(1<<i);
	}
	stm32_output[3] = temp8;
}

#endif
