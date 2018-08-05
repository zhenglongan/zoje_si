//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : watch.h
//  Description: common constants definition 
//  Version    Date     Author    Description
//  0.01     19/07/07   lm        created
//  0.02     27/12/07   zx        modify
//  0.03     17/01/08   lm        modify
//  ... 
//--------------------------------------------------------------------------------------
/***********************************/
/*          include file           */
/***********************************/
#include "..\..\include\sfr62p.h"         // M16C/62P special function register definitions
#include "..\..\include\typedef.h"        // Data type define
#include "..\..\include\variables.h"      // External variables declaration
#include "..\..\include\communication.h"  // Communication function
#include "..\..\include\variables.h"  // Communication function
#include "..\..\include\common.h"  // Communication function
/***********************************/
/*     function declaration        */
/***********************************/
void delay_ms(UINT16 time); 
void delay_us(UINT16 time); 
//--------------------------------------------------------------------------------------
//  Name:		delay_ms
//  Parameters:	time
//  Returns:	None
//  Description: delay time * 1ms
//--------------------------------------------------------------------------------------
void delay_ms(UINT16 time)       		// ms
{
	UINT16 delay_time;
	UINT16 temp16;
	
	delay_time = time;
  	ms_counter = 0;    					// ms counter clear 
  	temp16 = 0;
  	tb3s = 1;          					// start timer B3 
  	while(temp16 < delay_time)
  	{
  		rec_com();       				// communication with panel  
		temp16 = ms_counter;  	
  	}
  	tb3s = 0;          					// stop timer B3 
}
//--------------------------------------------------------------------------------------
//  Name:		delay_us
//  Parameters:	time
//  Returns:	None
//  Description: delay time * 1us    time must more than 100
//--------------------------------------------------------------------------------------
void delay_us(UINT16 time)       // us
{
	UINT16 delay_time;
	UINT16 temp16;
	#if INSERPOINT_ENABLE
	tb4 = (FX*1/10000-1);
	#endif
	delay_time = time / 100;
  	us_counter = 0;    // us counter clear 
  	temp16 = 0;
  	tb4s = 1;          // start timer B4 
  	while(temp16 < delay_time)
  	{
  		temp16 = us_counter;
  	}
  	tb4s = 0;          // stop timer B4 
}


/***********************************************************************************
* COPYRIGHT(C) 2007
* ALL RIGHTS RESERVED 
***********************************************************************************/
	


