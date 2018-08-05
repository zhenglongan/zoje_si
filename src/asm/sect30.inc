;*******************************************************************************
;
; C Compiler for M16C/60
;       Copyright 1995,1996,1997 MITSUBISHI ELECTRIC CORPORATION
;       AND MITSUBISHI ELECTRIC SEMICONDUCTOR SYSTEMS CORPORATION
;       All Rights Reserved.
;
; Written by T.Aoyama
;
;       sect30.inc     : section definition
;       This program is applicable when using the basic I/O library
;
;       $Id: sect30.inc,v 1.5 97/03/25 22:23:51 aoyama Exp $
;
;******************************************************************************
;---------------------------------------------------------------
;
;  Arrangement of section
;
;---------------------------------------------------------------
; Near RAM data area
;---------------------------------------------------------------
; SBDATA area
 .section data_SE,DATA
 .org 400H
data_SE_top:

 .section bss_SE,DATA,ALIGN
bss_SE_top:

 .section data_SO,DATA
data_SO_top:

 .section bss_SO,DATA
bss_SO_top:

; near RAM area
 .section data_NE,DATA,ALIGN
data_NE_top:

 .section bss_NE,DATA,ALIGN
bss_NE_top:

 .section data_NO,DATA
data_NO_top:

 .section bss_NO,DATA
bss_NO_top:

;---------------------------------------------------------------
; Stack area
;---------------------------------------------------------------
 .section stack,DATA
 .blkb STACKSIZE
stack_top:

 .blkb ISTACKSIZE
istack_top:


;---------------------------------------------------------------
; heap section
;---------------------------------------------------------------
 .section heap,DATA
heap_top:
 .blkb HEAPSIZE

;---------------------------------------------------------------
; Near ROM data area
;---------------------------------------------------------------
 .section rom_NE,ROMDATA,ALIGN
rom_NE_top:
 .section rom_NO,ROMDATA
rom_NO_top:

;---------------------------------------------------------------
; Far RAM data area
;---------------------------------------------------------------
 .section data_FE,DATA
 .org  10000H
data_FE_top:

 .section bss_FE,DATA,ALIGN
bss_FE_top:
 
 .section data_FO,DATA
data_FO_top:
 
 .section bss_FO,DATA
bss_FO_top:

;---------------------------------------------------------------
; Far ROM data area
;---------------------------------------------------------------
 .section rom_FE,ROMDATA
 .org  0D0000H
rom_FE_top:

 .section rom_FO,ROMDATA
rom_FO_top:

;---------------------------------------------------------------
; Initial data of 'data' section
;---------------------------------------------------------------
 .section data_SEI,ROMDATA
data_SEI_top:

 .section data_SOI,ROMDATA
data_SOI_top:

 .section data_NEI,ROMDATA
data_NEI_top:
 
 .section data_NOI,ROMDATA
data_NOI_top:
 
 .section data_FEI,ROMDATA
data_FEI_top:
 
 .section data_FOI,ROMDATA
data_FOI_top:
 
;---------------------------------------------------------------
; code area
;---------------------------------------------------------------
 .section program

 .section program_S

 .section interrupt
 .org 0f9000H

;---------------------------------------------------------------
; variable vector section
;---------------------------------------------------------------
 .section vector  ; variable vector table
 .org VECTOR_ADR 

 .glb	_spiint,_uart_tra_int,_uart_rec_int,_uart1_tra_int,_uart1_rec_int,_pwm_int,_int1_int,_int2_int,_int0_int,_ta0_int,_tb3_int,_tb4_int

	.lword	dummy_int		; BRK	(vector 0)
	.lword	dummy_int		; 	(vector 1)
	.lword	dummy_int		; 	(vector 2)
	.lword	dummy_int		; 	(vector 3)
	.lword	dummy_int		; int3(for user)(vector 4)
	.lword	dummy_int    	        ; timerB5(for user)(vector 5)
	.lword	_tb4_int		; timerB4(for user)(vector 6)
	.lword	_tb3_int		; timerB3(for user)(vector 7)
	.lword	_spiint			; si/o4 /int5(for user)(vector 8)
	.lword	dummy_int		; si/o3 /int4(for user)(vector 9)
	.lword	dummy_int		; Bus collision detection(for user)(v10)
	.lword	dummy_int		; DMA0(for user)(vector 11)
	.lword	dummy_int		; DMA1(for user)(vector 12)
	.lword	dummy_int		; Key input interrupt(for user)(vect 13)
	.lword	dummy_int		; A-D(for user)(vector 14)
	.lword	dummy_int		; uart2 transmit(for user)(vector 15)
	.lword	dummy_int		; uart2 receive(for user)(vector 16)
	.lword	_uart_tra_int	        ; uart0 transmit(for user)(vector 17)
	.lword	_uart_rec_int	        ; uart0 receive(for user)(vector 18)
.IF 1
	.lword	_uart1_tra_int		; uart1 transmit(for user)(vctor 19)
	.lword	_uart1_rec_int		; uart1 receive(for user)(vector 20)
.ELSE	
	.lword	dummy_int		; uart2 transmit(for user)(vector 15)
	.lword	dummy_int		; uart2 receive(for user)(vector 16)
.ENDIF	
	.lword	_ta0_int		; timer A0(for user)(vector 21)
	.lword	dummy_int		; timer A1(for user)(vector 22)
	.lword	dummy_int		; timer A2(for user)(vector 23)
	.lword	dummy_int		; timer A3(for user)(vector 24)
	.lword	dummy_int		; timer A4(for user)(vector 25)
	.lword	dummy_int               ; timer B0(for user)(vector 26)
	.lword	dummy_int               ; timer B1(for user)(vector 27)
	.lword	_pwm_int		; timer B2(for user)(vector 28)
	.lword	_int0_int		; int0 (for user)(vector 29)
	.lword	_int1_int		; int1 (for user)(vector 30)
	.lword	_int2_int		; int2 (for user)(vector 31)
    .lword  dummy_int  ; vector 32 (for user or MR30)
    .lword  dummy_int  ; vector 33 (for user or MR30)
    .lword  dummy_int  ; vector 34 (for user or MR30)
    .lword  dummy_int  ; vector 35 (for user or MR30)
    .lword  dummy_int  ; vector 36 (for user or MR30)
    .lword  dummy_int  ; vector 37 (for user or MR30)
    .lword  dummy_int  ; vector 38 (for user or MR30)
    .lword  dummy_int  ; vector 39 (for user or MR30)
    .lword  dummy_int  ; vector 40 (for user or MR30)
    .lword  dummy_int  ; vector 41 (for user or MR30)
    .lword  dummy_int  ; vector 42 (for user or MR30)
    .lword  dummy_int  ; vector 43 (for user or MR30) 
    .lword  dummy_int  ; vector 44 (for user or MR30)
    .lword  dummy_int  ; vector 45 (for user or MR30)
    .lword  dummy_int  ; vector 46 (for user or MR30)
    .lword  dummy_int  ; vector 47 (for user or MR30)
 ; to vector 63 from vector 32 is used MR30 

;===============================================================
; fixed vector section
;---------------------------------------------------------------
 .section fvector   ; fixed vector table
;===============================================================
; special page defination
;---------------------------------------------------------------
; macro is defined in ncrt0.a30
; Format: SPECIAL number
;
;---------------------------------------------------------------
; SPECIAL 255
; SPECIAL 254
; SPECIAL 253
;    :
;    :
; (omitted)
;    :
;    :
; SPECIAL 24
; SPECIAL 23
; SPECIAL 22
; SPECIAL 21
; SPECIAL 20
; SPECIAL 19
; SPECIAL 18
;
;===============================================================
; fixed vector section
;---------------------------------------------------------------
.IF 0
 .org 0fffdch
UDI:
 .lword dummy_int
OVER_FLOW:
 .lword dummy_int
BRKI:
 .lword dummy_int
ADDRESS_MATCH:
 .lword dummy_int
SINGLE_STEP:
 .lword dummy_int
WDT:
 .lword dummy_int
DBC:
 .lword dummy_int
NMI:
 .lword dummy_int
RESET:
 .lword start
.ENDIF
; 
;*******************************************************************************
;
; C Compiler for M16C/60
;       Copyright 1995,1997,1997 MITSUBISHI ELECTRIC CORPORATION
;       AND MITSUBISHI ELECTRIC SEMICONDUCTOR SYSTEMS CORPORATION
;       All Rights Reserved.
;
;*******************************************************************************

