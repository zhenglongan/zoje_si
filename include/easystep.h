#ifndef __EASYSTEP_H__
#define __EASYSTEP_H__

/*-----------------------------------------------------------------------------*/
//输入状态枚举
typedef enum
{
  State_NEG_LEV = 0,    //低电平
  State_POS_LEV = 1,    //高电平
  State_POS_EDG = 2,    //上升沿  
  State_NEG_EDG = 3,    //下降沿
  State_REVERSE = 4
}InputState;
/*-----------------------------------------------------------------------------*/
//输入操作枚举
typedef enum
{
  Operat_AND = 0,       // &&
  Operat_OR  = 1,       // ||
  Operat_END = 2,       // Over  
  Operat_GEQ = 3,       // >= :   equal or greater than
  Operat_LEQ = 4,       // <= :   equal or less than 
  Operat_EQU = 5,       // == :   equal 
  Operat_NEQ = 6,       // != :   not equal
  Operat_ADD = 7,       // + : 
  Operat_SUB = 8,       // - :
}InputOperat;
/*-----------------------------------------------------------------------------*/
//输出操作枚举
typedef enum
{
  Logic_LOW     = 0,        //Low
  Logic_HIG     = 1,        //Hig
  Logic_NOT     = 2,        //Tog
  
  Logic_INC     = 4,        // A = A + 1
  Logic_DEC     = 5,        // A = A - 1
  Logic_SET     = 6,        // A = B
  Logic_CLR     = 7,        // A = 0
  Logic_ADD     = 8,        // A = A + B
  Logic_SUB     = 9,        // A = A - B
  
  Logic_CALL    = 10,       //CALL
  Logic_RETURN  = 11,       //RETURN
  Logic_JUMP    = 12,       //JUMP
}OutputLogic;
/*-----------------------------------------------------------------------------*/
//数据类别枚举
typedef enum
{
  Class_NOP     = 0x00,
    
  Class_IN      = 0x00,
  Class_OUT     = 0x20,//0xE0 //同时out类相同编号，但是输入输出Class_OUT的值不同
  Class_RGST    = 0x40,
  
  Class_TIMER   = 0x50,
  Class_COUNT   = 0x60,
  
  Class_JUMP    = 0x80,
  Class_ALL     = 0xA0,
  Class_CALL    = 0xC0,
  Class_RETURN  = 0xE0,
  
  //  Class_OUT_S   = 0x20,
  //  Class_MODE    = 0xC0,
}DataClassUse;
/*-----------------------------------------------------------------------------*/
//输入函数个数
enum InputFunctionNumber
{
  FunNum_In1,
  FunNum_In2,
  //FunNum_In3,
  //FunNum_In4,
  //FunNum_In5,
  //FunNum_In6,
  //FunNum_In7,
  //FunNum_In8,
  FunNum_InN,
};
/*-----------------------------------------------------------------------------*/
//输出函数个数
enum OutputFunctionNumber
{
  FunNum_Out1,
  FunNum_Out2,
  FunNum_OutN,
};
/*-----------------------------------------------------------------------------*/
//输入结构体
typedef struct
{
  UINT8 Number;       //编号
  UINT8 State;        //状态
  UINT8 Operat;       //操作
}InPut_TypeDef;
/*-----------------------------------------------------------------------------*/
//输出结构体
typedef struct
{
  UINT16 Number;      //编号
  UINT8 Logic;        //逻辑
}OutPut_TypeDef;

/*-----------------------------------------------------------------------------*/
//指令行结构体
typedef struct
{
  InPut_TypeDef InPut[FunNum_InN];      //输入结构体 
  OutPut_TypeDef OutPut[FunNum_OutN];   //输出结构体
}Program_TypeDef;

/*-----------------------------------------------------------------------------*/
//指令支持条数
#define TOTAL_LINE_NUM                  30

#define DSP_COMMADN_VERSION1     			0x0001//1、读程序版本      0x0001、0x0006
#define DSP_COMMADN_VERSION2     			0x0006
#define DSP_COMMADN_UPDATE_BEGIN 			0x000F//2、开始进行升级  0x000F
#define DSP_COMMADN_UPDATE_DATA  			0x00F0//3、升级数据传输 0x00F0
#define DSP_COMMADN_UPDATE_QUERY 			0x0010//4、升级状态查询 0x0010
#define DSP_COMMADN_UPDATE_DONE  			0x00F0//5、升级完成指令 0x00F0

#define DSP_COMMADN_WRITE_REGISTER 			0x1000//6、写BOOL状态 0x1000
#define DSP_COMMADN_READ_REGISTER  		 	0x1001//7、读BOOL状态 0x1001
#define DSP_COMMADN_WRITE_STATUS   		 	0x1002//8、写WORD状态0x1002
#define DSP_COMMADN_READ_STATUS      	 	0x1003//9、读WORD状态0x1003

#define DSP_COMMADN_WRITE_INPUT_PORT     	0x1004//10、写输入端口状态 0x1004
#define DSP_COMMADN_READ_INPUT_PORT      	0x1005//11、读输入端口状态 0x1005  32bits
#define DSP_COMMADN_WRITE_OUTPU_PORT	    0x1006//12、写输出端口状态 0x1006
#define DSP_COMMADN_READ_OUTPUT_PORT 		0x1007//13、读输出端口状态 0x1007

#define DSP_COMMADN_PROGRAMING_BEGIN 		0x1100//14、切换到升级编程代码程序 0x1100
#define DSP_COMMADN_PROGRAMING_DATA   		0x1101//15、写入编程程序数据 0x1101
#define DSP_COMMADN_PROGRAMING_QUERY 		0x1102//16、写入编程状态查询 0x1102
#define DSP_COMMADN_PROGRAMING_DONE	 		0x1104//17、升级状态完成 0x1104

#define DSP_COMMADN_EMERGENCY_START 		0x1105//19、程序启动指令 0x1106
#define DSP_COMMADN_EMERGENCY_STOP 			0x1106//18、程序发止状态 0x1105

#define DSP_COMMADN_RESET_ALL 				0x1107//20、总体复位指令 0x1107
#define DSP_COMMADN_SINGLE_DEBUG 			0x1108//21、单步执行指令 0x1108
#define DSP_COMMADN_BREAK 			    	0x1109//22、暂停指令     0x1109
#define DSP_COMMADN_QUERY_ERROR				0x110A//23、读取错误指令 0x110A
#define DSP_COMMADN_QUERY_STATUS			0x110B//24、读取状态指令 0x110B

extern UINT8 Total_Program_Lines;
extern UINT8 LineCode[];
extern InPut_TypeDef         SystemInput[];	 //系统的输入缓冲区  引脚+条件码

extern void set_system_status_condiction(UINT8 number,UINT8 out_data);
extern void app_EasyStepInit(void);
extern void app_EasyStepLineProcess(void);
extern void app_GetProgramFromEeprom(void);
extern void read_para_io_fun1(UINT8 *buf);
extern void set_output_to_IO(UINT8 port,UINT8 reg,UINT8 data);


#endif
