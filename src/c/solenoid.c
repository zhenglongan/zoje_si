//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : main.c
//  Description: Core program to control the sewing machine
//  Version    Date     Author    Description
//  0.01     24/07/07   lm        created
//  ...
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"       // M16C/62P special function register definitions
#include "..\..\include\typedef.h"      // Data type define
#include "..\..\include\common.h"       // Common constants definition
#include "..\..\include\variables.h"    // External variables declaration
#include "..\..\include\delay.h"        // delay time definition
#include "..\..\include\action.h"       // action function
//--------------------------------------------------------------------------------------
//  functions declaration
//--------------------------------------------------------------------------------------
void fw_solenoid(void);
void at_solenoid(void);
void fa_solenoid(void);
void io_test(void);
//--------------------------------------------------------------------------------------
//  Name:		fw_solenoid 
//  Parameters:	None
//  Returns:	None
//  Description: thread wiper solenoid driver
//--------------------------------------------------------------------------------------
void fw_solenoid(void)
{		
  	//--------------------------------------------------------------------------------------      
  	//  inpresser position 
  	//--------------------------------------------------------------------------------------
	#if FOLLOW_INPRESS_FUN_ENABLE
	#else
	if(u105 != 0)
	{
		inpress_to(0);	
	}	
	#endif
	//--------------------------------------------------------------------------------------      
  	//  wiper move
  	//--------------------------------------------------------------------------------------
	if(u51 == 1)        // wiper enable
	{	
		#if FW_TYPE_SOLENOID_VALVE || CURRENT_STEPPER_CONFIG_TYPE == CONFIG_MACHINE_TYPE_6037_800
	  	  #if SECOND_GENERATION_PLATFORM
			FK_OFF = 1;
			delay_ms((UINT16)k52*10);
			FK_OFF = 0;
			delay_ms((UINT16)k53*10);
		  #else
		    FR_ON = 1;
			delay_ms((UINT16)k52*10);
			FR_ON = 0;
			delay_ms((UINT16)k53*10);
		  #endif
		  
		#else
			SNT_H = 1;        // 24V to 33V
		  	FW = 1;
	    	delay_ms((UINT16)k52*10);
		  	FW = 0;
		  	SNT_H = 0;        // 33V to 24V
			delay_ms((UINT16)k53*10);
		#endif
	}
}
//--------------------------------------------------------------------------------------
//  Name:		at_solenoid 
//  Parameters:	None
//  Returns:	None
//  Description: thread wiper solenoid driver
//--------------------------------------------------------------------------------------
void at_solenoid(void)
{
	UINT8 temp8;
	
	if(temp_tension != temp_tension_last)
	{
		temp8 = (UINT8)temp_tension; 
	  if(temp8 == 0)
	  {	
	    da0 = 0;
	  }
	  else
	  {
	  	#if(AT_SOLENOID==1)
	  	da0 = temp8 + 45;
	  	#else
	  	da0 = temp8;
	  	#endif
	  }
  	}
  	temp_tension_last = temp_tension;
}

//--------------------------------------------------------------------------------------
//  Name:		fw_solenoid 
//  Parameters:	None
//  Returns:	None
//  Description: thread wiper solenoid driver
//--------------------------------------------------------------------------------------
void io_test(void)
{
	BACKUP1 = 1;
  	delay_us(400);
	BACKUP1 = 0;
	delay_us(400);
}
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
