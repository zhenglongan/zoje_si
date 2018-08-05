/*************************************************************
 * bug detect 
 ************************************************************/
#include "..\..\include\variables.h"        // External variables declaration
#include "..\..\include\common.h"
#include "..\..\include\M_debug.h"


#if UART1_DEBUG_OUTPUT_MODE
extern void uart1_send_char(UINT8 ch);
#endif

void test_nop_emergency(UINT16 temp1,UINT16 temp2);
void test_dis_debug_info(UINT8 firstB,UINT8 secondB);

void test_nop_emergency(UINT16 temp1,UINT16 temp2)
{
	UINT16 test_temp16;
	if(temp1 != 0)
	{
		test_temp16 = temp1;
		de_bug.test1 = (UINT8)(test_temp16>>8);
		de_bug.test2 = (UINT8)(test_temp16&0x00FF);
	}
	if(temp2 != 0)
	{
		test_temp16 = temp2;
		de_bug.test3 = (UINT8)(test_temp16>>8);
		de_bug.test4 = (UINT8)(test_temp16&0x00FF);
	}
}
void set_func_code_info(UINT8 first,UINT8 second,UINT8 para1,UINT8 para2)
{	
	#if  QUERY_NEW_PROTOCOL
	if( debug_counter < 32 )
	{
		sys_de_bug[debug_write_point].test1 = first;
		sys_de_bug[debug_write_point].test2 = second;
		sys_de_bug[debug_write_point].test3 = para1;
		sys_de_bug[debug_write_point].test4 = para2;
		
		debug_write_point = (debug_write_point+1)%32;
		debug_counter++;
	}	
	#elif UART1_DEBUG_OUTPUT_MODE
		uart1_send_char(first);
		uart1_send_char(second);
		uart1_send_char(para1);
		uart1_send_char(para2);
		uart1_send_char(0xd);
		uart1_send_char(0xa);
	#endif
}

void get_func_debug_info(void)
{
	if( debug_counter > 0)
	{
		de_bug.test1 = sys_de_bug[debug_read_point].test1;
		de_bug.test2 = sys_de_bug[debug_read_point].test2;
		de_bug.test3 = sys_de_bug[debug_read_point].test3;
		de_bug.test4 = sys_de_bug[debug_read_point].test4;
		debug_read_point = (debug_read_point+1)%50;
		debug_counter--;
	}
	else
	{
		de_bug.test1 = 11;
		de_bug.test2 = 22;
		de_bug.test3 = 33;
		de_bug.test4 = 44;
	}
}

void set_protocol_usrful(UINT8 first,UINT8 second)
{
	de_bug.test3 = first;
	de_bug.test4 = second;
}
