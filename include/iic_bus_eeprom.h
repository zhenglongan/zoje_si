//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : 
//  Description: 
//  Version    Date     Author    Description
//  0.01     21/02/06   liwenz    created
//  ... 
//  ...
//--------------------------------------------------------------------------------------
#include "typedef.h"      //Data type define
#define ACK		0
#define NOACK	1

#define WRITE_MODE	0
#define READ_MODE	1

typedef struct {
	UINT8 iic_DeviceAddress;
	UINT8 iic_MemoryAddress_h;
	UINT8 iic_MemoryAddress_l;
	UINT8 *iic_Data;
	UINT8 iic_NumberOfByte;
}IicPack;

void initIicBus(void);
UINT8 IicBusRead(IicPack *);
UINT8 IicBusWrite(IicPack *);
void StartCondition(void);
void StopCondition(void);
UINT8 ByteWrite(UINT8);
void ByteRead(UINT8 *, UINT8);

extern void init_eeprom(void);
extern UINT16 read_par(UINT16 par_num);
extern void write_par(UINT16 par_num,INT16 par);
extern void load_par(void);
extern void _WaitTime6ms(void);
extern void read_para_group(UINT16 add,UINT8 *point,UINT16 len);
extern void write_para_group(UINT16 add,UINT8 *point,UINT16 len);
extern void restore_para_from_eeprom(void);
extern void cpy_para_buff(void);
extern void init_para_variables(void);

//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------