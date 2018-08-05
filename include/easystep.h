#ifndef __EASYSTEP_H__
#define __EASYSTEP_H__

/*-----------------------------------------------------------------------------*/
//����״̬ö��
typedef enum
{
  State_NEG_LEV = 0,    //�͵�ƽ
  State_POS_LEV = 1,    //�ߵ�ƽ
  State_POS_EDG = 2,    //������  
  State_NEG_EDG = 3,    //�½���
  State_REVERSE = 4
}InputState;
/*-----------------------------------------------------------------------------*/
//�������ö��
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
//�������ö��
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
//�������ö��
typedef enum
{
  Class_NOP     = 0x00,
    
  Class_IN      = 0x00,
  Class_OUT     = 0x20,//0xE0 //ͬʱout����ͬ��ţ������������Class_OUT��ֵ��ͬ
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
//���뺯������
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
//�����������
enum OutputFunctionNumber
{
  FunNum_Out1,
  FunNum_Out2,
  FunNum_OutN,
};
/*-----------------------------------------------------------------------------*/
//����ṹ��
typedef struct
{
  UINT8 Number;       //���
  UINT8 State;        //״̬
  UINT8 Operat;       //����
}InPut_TypeDef;
/*-----------------------------------------------------------------------------*/
//����ṹ��
typedef struct
{
  UINT16 Number;      //���
  UINT8 Logic;        //�߼�
}OutPut_TypeDef;

/*-----------------------------------------------------------------------------*/
//ָ���нṹ��
typedef struct
{
  InPut_TypeDef InPut[FunNum_InN];      //����ṹ�� 
  OutPut_TypeDef OutPut[FunNum_OutN];   //����ṹ��
}Program_TypeDef;

/*-----------------------------------------------------------------------------*/
//ָ��֧������
#define TOTAL_LINE_NUM                  30

#define DSP_COMMADN_VERSION1     			0x0001//1��������汾      0x0001��0x0006
#define DSP_COMMADN_VERSION2     			0x0006
#define DSP_COMMADN_UPDATE_BEGIN 			0x000F//2����ʼ��������  0x000F
#define DSP_COMMADN_UPDATE_DATA  			0x00F0//3���������ݴ��� 0x00F0
#define DSP_COMMADN_UPDATE_QUERY 			0x0010//4������״̬��ѯ 0x0010
#define DSP_COMMADN_UPDATE_DONE  			0x00F0//5���������ָ�� 0x00F0

#define DSP_COMMADN_WRITE_REGISTER 			0x1000//6��дBOOL״̬ 0x1000
#define DSP_COMMADN_READ_REGISTER  		 	0x1001//7����BOOL״̬ 0x1001
#define DSP_COMMADN_WRITE_STATUS   		 	0x1002//8��дWORD״̬0x1002
#define DSP_COMMADN_READ_STATUS      	 	0x1003//9����WORD״̬0x1003

#define DSP_COMMADN_WRITE_INPUT_PORT     	0x1004//10��д����˿�״̬ 0x1004
#define DSP_COMMADN_READ_INPUT_PORT      	0x1005//11��������˿�״̬ 0x1005  32bits
#define DSP_COMMADN_WRITE_OUTPU_PORT	    0x1006//12��д����˿�״̬ 0x1006
#define DSP_COMMADN_READ_OUTPUT_PORT 		0x1007//13��������˿�״̬ 0x1007

#define DSP_COMMADN_PROGRAMING_BEGIN 		0x1100//14���л���������̴������ 0x1100
#define DSP_COMMADN_PROGRAMING_DATA   		0x1101//15��д���̳������� 0x1101
#define DSP_COMMADN_PROGRAMING_QUERY 		0x1102//16��д����״̬��ѯ 0x1102
#define DSP_COMMADN_PROGRAMING_DONE	 		0x1104//17������״̬��� 0x1104

#define DSP_COMMADN_EMERGENCY_START 		0x1105//19����������ָ�� 0x1106
#define DSP_COMMADN_EMERGENCY_STOP 			0x1106//18������ֹ״̬ 0x1105

#define DSP_COMMADN_RESET_ALL 				0x1107//20�����帴λָ�� 0x1107
#define DSP_COMMADN_SINGLE_DEBUG 			0x1108//21������ִ��ָ�� 0x1108
#define DSP_COMMADN_BREAK 			    	0x1109//22����ָͣ��     0x1109
#define DSP_COMMADN_QUERY_ERROR				0x110A//23����ȡ����ָ�� 0x110A
#define DSP_COMMADN_QUERY_STATUS			0x110B//24����ȡ״ָ̬�� 0x110B

extern UINT8 Total_Program_Lines;
extern UINT8 LineCode[];
extern InPut_TypeDef         SystemInput[];	 //ϵͳ�����뻺����  ����+������

extern void set_system_status_condiction(UINT8 number,UINT8 out_data);
extern void app_EasyStepInit(void);
extern void app_EasyStepLineProcess(void);
extern void app_GetProgramFromEeprom(void);
extern void read_para_io_fun1(UINT8 *buf);
extern void set_output_to_IO(UINT8 port,UINT8 reg,UINT8 data);


#endif
