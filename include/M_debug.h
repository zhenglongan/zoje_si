#ifndef M_DEBUG_H
#define M_DEBUG_H
extern void test_nop_emergency(UINT16 temp1,UINT16 temp2);
extern void set_protocol_usrful(UINT8 first,UINT8 second);
extern void set_func_code_info(UINT8 first,UINT8 second,UINT8 para1,UINT8 para2);
extern void get_func_debug_info(void);
extern void uart1_send_char(UINT8 ch);
extern void printf_uart_string(UINT8 far *str);
#endif