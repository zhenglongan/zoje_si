//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : initial.h
//  Description: Header file motor control
//  Version    Date     Author    Description
//  0.01     03/07/07   pbb        created
//  0.01     04/01/08   lm         modified
//  ...
//--------------------------------------------------------------------------------------
#ifndef SYSTEM_H
#define SYSTEM_H

extern void reserve_status(void);       // 00
extern void free_status(void);          // 01
extern void ready_status(void);         // 02
extern void run_status(void);           // 03
extern void error_status(void);         // 04
extern void prewind_status(void);       // 05
extern void wind_status(void);          // 06
extern void inpress_status(void);       // 07
extern void poweroff_status(void);      // 08
extern void single_status(void);        // 09
extern void manual_status(void);        // 10
extern void setout_status(void);        // 11
extern void emerstop_status(void);      // 12
extern void preedit_status(void);       // 13
extern void edit_status(void);          // 14
extern void noedit_status(void);        // 15
extern void finish_status(void);        // 16
extern void needle_status(void);        // 17
extern void waitoff_status(void);       // 18
extern void trim_status(void);          // 19
extern void slack_status(void);         // 20
extern void checki03_status(void);      // 21
extern void checki04_status(void);      // 22
extern void checki05_status(void);      // 23
extern void checki06_status(void);      // 24
extern void checki07_status(void);      // 25
extern void checki08_status(void);      // 26
extern void checki10_status(void);      // 27
extern void emermove_status(void);      // 28
extern void boardtest_status(void);     // 29
extern void download_status(void);
extern void pre_run_conditioning(void);
extern void checki11_status(void);
extern void downloaddsp_status(void);
extern void download_dsp_curve_status(void);
extern void multipule_io_status(void);
extern void download_multipul_program_status(void);
extern void rfid_read_write(void);
#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
