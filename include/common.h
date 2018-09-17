//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : common.h
//  Description: common constants definition 
//  Version    Date     Author    Description
//  0.01     03/07/07   pbb        created
//  0.02     14/07/07   lm         modify for AMS-210E
//  0.03     03/06/08   lm         VSS
//  ...
//--------------------------------------------------------------------------------------
#ifndef COMMON_H
#define COMMON_H

//--------------------------------------------------------------------------------------
// 厂家、机型、软件版本信息  
//--------------------------------------------------------------------------------------
#define FACTORY_TYPE   			11  // 0-DAHAO; 2-ChangFeng;11-Zhongjie
#define MACHINE_TYPE   			3
#define DAHAO_TYPE     			400
#define VERSION_NUM_1   		07   
#define VERSION_NUM_2   		1    
#define VERSION_YEAR    		9    
#define VERSION_MONTH   		07  
#define VERSION_DAY     		21      
#define MainFatherVersion       4
//#define MainSVNVersion		   	2967//2966//2965//2964//2963//2962//2961//2952//2951//2950//2925
/*
	1、修改起缝加固打开后导致功能段前几针跳过的问题
	--问题描述：
     当K18设置为2，表示起缝使用前K19针进行加固，如果K19大于0表示使用V型线迹加固时，程序的加固方式是先
   空送到车缝后的K19针，然后再倒缝到起缝第一针，接着正常缝制下去，这样就完成了V型加固线迹，然而由于
   很多时候空送后是激光切割、画笔、旋转切刀、直线切刀等功能段（借用了车缝数据表示运动路径），因此导
   致这些功能段的前K19针没有处理，直接跳过了
   
	--解决方法：
     为了解决这个问题，本版本（2968）进行了相应的修改，增加了花样码分析程序文件pattern_check.c/.h，内部
   增加函数check_subsequent_special_segment(pat)用于判断传入的空送码后面出现的车缝码属于什么功能段，是车缝
   还是画笔、激光、旋转切刀、空送等段。然后在空送段处理函数go_beginpoint()中处理到最后一针空送对起缝加固
   进行处理的时候，增加这个函数机型判断，只有当接下来的是车缝段而不是其他功能段的时候才进行起缝加固的处理。
    
   **修改适用机型：激光、画笔、旋转切刀、直线切刀相关所有机型

   备注：暂时未经过上机实测验证
*/
#define MainSVNVersion		   	2968//2018-9-17



//--------------------------------------------------------------------------------------
// 机型配置信息  
//--------------------------------------------------------------------------------------

//-------------配置信号--------------子版本---名称-------机型编号---------平台----驱动--------DSP3----电机------------------------
#define MACHINE_CONFIG_NUMBER1       1 // 750黑金刚     ASC400A-MBJ      4代    3303/43            单X86+Y86+空+Z60开  
#define MACHINE_CONFIG_NUMBER2       2 // 900切刀换梭   ASC4054          4代    4009/4326   37     双X60+Y86+Z60开    +切刀60+换梭臂57+梭盘42
#define MACHINE_CONFIG_NUMBER3       3 // 900直线切刀   ASC4053          4代    4327/4327          单X60+Y86+切60+Z60开
#define MACHINE_CONFIG_NUMBER4       4 // 900旋转切刀   ASC4052          4代    4009/4326   37     双X60+Y86+Z60开    + 旋转60开
#define MACHINE_CONFIG_NUMBER5       5 // 750直线切刀   ASC400A-MBJ3     4代    43242/43242        单X86+Y86+切60+Z60开
#define MACHINE_CONFIG_NUMBER6       6 // 900普通       ASC405           4代    4009/4326          双X60+Y86+Z60开
#define MACHINE_CONFIG_NUMBER7       7 // 750激光切刀   ASC400A-MBJ4     4代    3303/43            单X86+Y86+空+Z60开 
#define MACHINE_CONFIG_NUMBER8       8 // 900自动换梭   ASC4051          4代    4009/4326   37     双X60+Y86+Z60开    +换梭臂57+梭盘42
#define MACHINE_CONFIG_NUMBER9       9 // 800普通       ASC400B          4代    4009/4326          双X60+Y86+Z60开
#define MACHINE_CONFIG_NUMBER10      10 //800激光切刀   ASC400B          4代    4009/4326          双X60+Y86+Z60开
#define MACHINE_CONFIG_NUMBER11      11 //900激光切刀   ASC405           4代    4009/4326          双X60+Y86+Z60开
#define MACHINE_CONFIG_NUMBER12      12 //900新激光切刀 ASC4055          4代    4009/4326          双X60+Y86+Z60开
#define MACHINE_CONFIG_NUMBER13      13 //900抓线       ASC4056          4代    4009/4326   37     双X60+Y86+Z60开   +抓线60
#define MACHINE_CONFIG_NUMBER14      14 //800随动中压脚 ASC400C          4代     4009/41           双X60+Y86+Z60闭1000 
#define MACHINE_CONFIG_NUMBER15      15 //800普通一体化 ASC400D          4代    15503/15503        双X60+Y86+Z60开   
#define MACHINE_CONFIG_NUMBER16      16 //900普通一体化 ASC405           4代    15503/15503        双X60+Y86+Z60开
#define MACHINE_CONFIG_NUMBER17      17 //800一体化5代  MASC400D         5代    15503/15503        单X60+Y86+Z60开     
#define MACHINE_CONFIG_NUMBER18      18 //800一体化2代  TASC400D         2代    15503/15503        X60+Y86+Z60闭环     6037
#define MACHINE_CONFIG_NUMBER19      19 //800一体化4代  ASC400D          4代    15503/15503        单X60+Y86+Z60开     12080
#define MACHINE_CONFIG_NUMBER20      20 //800普通一体化 ASC400D          4代    15503/15503        单X60+Y86+Z60开   
#define MACHINE_CONFIG_NUMBER21      21 //900普通一体化 ASC405           4代    15503/15503        单X60+Y86+Z60开
#define MACHINE_CONFIG_NUMBER22      22 //800一体化激光切刀   ASC400B    4代    15503/15503        双X60+Y86+Z60开
#define MACHINE_CONFIG_NUMBER23      23 //900一体化激光切刀   ASC405     4代    15561/15503        双X60+Y86+Z60开
#define MACHINE_CONFIG_NUMBER24      24 //800一体化随动中压脚 ASC400D    4代    15506/15506        单X60+Y86+Z60闭环1000线 
#define MACHINE_CONFIG_NUMBER25      25 //900一体化随动中压脚 ASC405     4代    15506/15506        单X60+Y86+Z60闭环1000线 

#define MACHINE_CONFIG_NUMBER26      26 //900一体化自动换梭 ASC4051      4代    15561/15561        单X60+Y86+Z60开环 + SC074C 
#define MACHINE_CONFIG_NUMBER27      27 //900一体化直线切刀
#define MACHINE_CONFIG_NUMBER28      28 //900一体化旋转切刀
#define MACHINE_CONFIG_NUMBER29      29 //900一体化5代  MASC400D         5代    15561/15561        单X60+Y86+Z60开  
#define MACHINE_CONFIG_NUMBER30      30 //800一体化5代  MASC400B         5代    15503/15503        单X60+Y86+Z60闭 
#define MACHINE_CONFIG_NUMBER31      31 //900一体化5代直线切刀MASC4053    5代    15561/15561        单X60+Y86+切60+Z60开
#define MACHINE_CONFIG_NUMBER32      32 //900一体化5代双丝杠MASC4055           5代   15561/15561        伺服X60+伺服86+空+Z60闭

#define MACHINE_CONFIG_NUMBER33      33 //800一体化5代激光  MASC400D         5代    15503/15503        单X60+Y86+Z60开    
#define MACHINE_CONFIG_NUMBER34      34 //900一体化5代激光  MASC4056         5代    15503/15503        单X60+Y86+Z60开    
#define MACHINE_CONFIG_NUMBER35      35 //800一体化5代 +RFID               5代 XYZ1000线闭环
#define MACHINE_CONFIG_NUMBER36      36 //900一体化5代 +RFID               5代 XYZ1000线闭环
#define MACHINE_CONFIG_NUMBER37      37 //800一体化5代 +RFID +直线切刀      5代 XYZ1000线闭环+切刀1000线
#define MACHINE_CONFIG_NUMBER38      38 //800一体化5代 +RFID +激光         5代 XYZ1000线闭环

#define MACHINE_CONFIG_NUMBER39      39 //900一体化5代PLUS自动换梭+条码    5代 XY1000线闭环+Z开环+抓臂电机+梭盘电机 后为改闭环
#define MACHINE_CONFIG_NUMBER40      40 //900一体化5代PLUS旋转切刀+条码    5代 XY1000线闭环+Z开环+旋转电机+伺服切刀电机（自购） 后改闭环
#define MACHINE_CONFIG_NUMBER41      41 //900一体化5代双丝杠款             5代 XYZ 1000线闭环

#define MACHINE_CONFIG_NUMBER42      42 //37 +随动中压脚
//#define MACHINE_CONFIG_NUMBER43    43 //39 +随动中压脚
#define MACHINE_CONFIG_NUMBER44      44 //40 +随动中压脚

#define MACHINE_CONFIG_NUMBER55      55 //800二代一体机，小RAM XY1000线,Z400线，XORG和IOGR没交换，工装上用

#define COMPILE_MACHINE_TYPE        MACHINE_CONFIG_NUMBER36

#define MainChildVersion            COMPILE_MACHINE_TYPE


//--------------------------------------------------------------------------------------
// 机型编译支持信息  
//--------------------------------------------------------------------------------------
#define MACHINE_NORMAL         		0  //标配机型       -1
#define MACHINE_MID            		1  //中配机型
#define MACHINE_BOBBIN_CUTTER  		2  //高配机型       -2  ASC4054
#define MACHINE_AUTO_LOCATE    		3  //自动机型
#define MACHINE_900_NEW        		4  //900机型        -6
#define MACHINE_SPEPPER_CUTTER 		5  //直线切刀机型   -5
#define MACHINE_800_NORMAL     		6  //800机型        -9
#define MACHINE_LASER_CUTTER   		7  //               -7
#define MACHINE_900_BOBBIN     		8  //-900机型       -8  ASC4051
#define MACHINE_900_SPEPPER_CUTTER  9  //-900机型       -3  ASC4053
#define MACHINE_900_CUTTER          0xa//-900机型       -4  ASC4052
#define MACHINE_800_LASER_CUTTER    0xb//-800机型       -10 ASC400C
#define MACHINE_900_LASER_CUTTER    0xc//-900机型       -11 
#define MACHINE_900_LASER_CUTTER2   0xd//-900机型       -12 ASC4055
#define MACHINE_900_CLAMP           0xe//-900机型       -13 ASC4056
#define MACHINE_SECOND_GENERATION   0xf
#define MACHINE_900_FIFTH_BOBBIN    0x10 //900一体化5代自动换梭
#define MACHINE_900_FIFTH_CUTTER    0x11 //900一体化5代旋转切刀+条码
#define MACHINE_800_BALLSCREW       0x12 //800一体化5代双丝杠
#define MACHINE_900_BALLSCREW       0x13 //900一体化5代双丝杠
#define MACHINE_800_STEPPER_CUTTER  0x14 //800一体化5代直线切刀 4003C

#define CONFIG_MACHINE_TYPE_6037       		   1//小模板机  
#define CONFIG_MACHINE_TYPE_12080_SCREW        2//12080丝杠 单X
#define CONFIG_MACHINE_TYPE_12080_SCREW_A      3//12080丝杠 双X
#define CONFIG_MACHINE_TYPE_12080_BELT         4//12080皮带 单X
#define CONFIG_MACHINE_TYPE_12080_BELT_A       5//12080皮带 双X
#define CONFIG_MACHINE_TYPE_DOUBLE_X           6//普通800,900双X轴
#define CONFIG_MACHINE_TYPE_6037_800   		   7//4代系统小模板机+激光切割
#define CONFIG_MACHINE_TYPE_FIFTH_DOUBLE_X     8//5代系统800,900机型
#define CONFIG_MACHINE_TYPE_SINGLE_X_INPERSSER 9//4代单X一体化驱动中压脚随动1000线
#define CONFIG_MACHINE_TYPE_FIFTH_BOBBIN       10//5代自动换梭


#define SECOND_GENERATION_PLATFORM             0
#define LITTLE_RAM_CPU                         0 //小RAM CPU
#if MainFatherVersion >= 5
#define INSTALLMENT          				   1 //分期付款方式
#else
#define INSTALLMENT   0
#endif	
#define SUPPORT_SEWING_MARKING_PEN             1 //支持记号笔用车缝数据表示形状
#define INPRESS_FOLLOW_MOTOR_ACTION            0 //中压脚随主轴动作
#define AUTO_BACKWARD_THREAD_BREAK             1 //断线后自动后退几针

#define DEBUG_RFID_DA1_FUN              	   0  //da1调试RFID识读过程
#define DEBUG_RFID_DA1_WRITE_READ        	   0  //RFID读写过程
#define DEBUG_ROTATED_CUTTER_DA1		       0  //调试旋转切刀

//------------------------具体配置内容-------------------------------------------------------------------------------------
#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER1		//1-750黑金刚	
	#define ENABLE_SCRATCH_FUN          1 
	#define FAN_POWER_ENABLE       		1
	#define CURRENT_MACHINE  			MACHINE_NORMAL
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER2	//2-900切刀换梭 
	//#define DOUBLE_X_60MOTOR            1
	//#define SUPPORT_NEW_DRIVER          1
	//#define ENABLE_SCRATCH_FUN          1 
	//#define DSP3_CUTER_DRIVER   	    1
	//#define SC0413                      1
	//#define CURRENT_MACHINE             MACHINE_BOBBIN_CUTTER
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角换算方法
    #define CURRENT_MACHINE             MACHINE_BOBBIN_CUTTER //MACHINE_900_BOBBIN
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_DOUBLE_X
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER3	//3-900直线切刀
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_LOOPER_CUTTER        1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define CURRENT_MACHINE             MACHINE_900_SPEPPER_CUTTER
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER4	//4-900旋转切刀
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
	#define DOUBLE_X_60MOTOR            1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define DSP3_CUTER_DRIVER           1
	#define SC0413                      1
	#define CURRENT_MACHINE             MACHINE_900_CUTTER
	
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER5	//5-750直线切刀
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_LOOPER_CUTTER        1
	#define ENABLE_SCRATCH_FUN          1 
	#define CURRENT_MACHINE             MACHINE_SPEPPER_CUTTER

#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER6	//6-900普通
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif	
	#define DOUBLE_X_60MOTOR            1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define CURRENT_MACHINE             MACHINE_900_NEW

#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER7	//7-750激光切刀
	#define ENABLE_LASER_CUTTER         1
	#define ENABLE_SCRATCH_FUN          1 
	#define CURRENT_MACHINE             MACHINE_LASER_CUTTER
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER8	//8-900自动换梭 
 	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
	#define DOUBLE_X_60MOTOR            1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
    #define CURRENT_MACHINE             MACHINE_900_BOBBIN
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER9	//9-800普通
	#define DOUBLE_X_60MOTOR            1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define CURRENT_MACHINE             MACHINE_800_NORMAL

#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER10	//10-800激光切刀
	#define DOUBLE_X_60MOTOR            1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_LASER_CUTTER         1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define CURRENT_MACHINE             MACHINE_800_LASER_CUTTER
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER11	//11-900激光切刀
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif	
	#define DOUBLE_X_60MOTOR       		1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_LASER_CUTTER         1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define CURRENT_MACHINE             MACHINE_900_LASER_CUTTER 
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER12	//12-900新激光切刀
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif	
	#define DOUBLE_X_60MOTOR       		1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_LASER_CUTTER         1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define INSERPOINT_ENABLE           1  //插补算法
	#define NEW_LASER_DEVICE            1
	#define LASER_DSP_PROCESS		    0 //激光款数据发DSP
	#define CURRENT_MACHINE             MACHINE_900_LASER_CUTTER2
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER13	//13-900抓线
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif	
	#define DOUBLE_X_60MOTOR            1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define CURRENT_MACHINE             MACHINE_900_CLAMP
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER14	//14-800随动中压脚
	#define DOUBLE_X_60MOTOR            1
	#define SUPPORT_NEW_DRIVER          1
	#define FOLLOW_INPRESS_FUN_ENABLE   1
	#define NEW_STRUCTURE_800_INPRESS   1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define CURRENT_MACHINE             MACHINE_800_NORMAL
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER15	//15-800普通一体化  
	#define DOUBLE_X_60MOTOR            1
	#define SUPPORT_NEW_DRIVER          1
	#define FOLLOW_INPRESS_FUN_ENABLE   0
	#define NEW_STRUCTURE_800_INPRESS   0
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 
	#define INSERPOINT_ENABLE           0  //插补算法
	#define ENABLE_LASER_CUTTER         0 
	#define FW_TYPE_SOLENOID_VALVE      0  //0-SOLENOID 1-VALVE
	#define NEW_CUT_MODE				0  //新剪线模式	
	#define FIRST_STITCH_NOT_ACTION     0
	#define CURRENT_MACHINE    			MACHINE_800_NORMAL
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_DOUBLE_X
           

#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER16	//16-900普通一体化
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif	
	#define DOUBLE_X_60MOTOR        	1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1  //一体化驱动
	#define CURRENT_MACHINE             MACHINE_900_NEW
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_DOUBLE_X
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER17	//17-800一体化5代
 	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        0  //RFID 接口
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角方法
	#define CURRENT_MACHINE             MACHINE_800_NORMAL
	
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_DOUBLE_X
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER18	//18-800一体化2代  6037
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define FOLLOW_INPRESS_FUN_ENABLE   1
	#define NEW_STRUCTURE_800_INPRESS   1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	
	#ifdef SECOND_GENERATION_PLATFORM   
	   #undef SECOND_GENERATION_PLATFORM
	   #define SECOND_GENERATION_PLATFORM   1//二代一体机平台
    #endif
	#define LASER_DSP_PROCESS           1  //支持DSP发激光数据
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        1  //RFID 接口
	#define INSERPOINT_ENABLE           1  //插补算法
	#define ENABLE_LASER_CUTTER         1 	
	#define NEW_LASER_DEVICE            1
	#define FW_TYPE_SOLENOID_VALVE      1  //0-SOLENOID 1-VALVE
	#define NEW_CUT_MODE				1  //新剪线模式	
	#define FIRST_STITCH_NOT_ACTION     1
	#define SEND_SERVO_PARA_ONLINE      0
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角方法
	#define ENBALE_TURNON_LASER_FUN     1  //风扇一直打开
	
	
	#define YORG_REVERSE_LEVEL          1  //0802光耦=>0712光耦
	#define IORG_REVERSE_LEVEL          1  //0802光耦=>0712光耦  
	#define CURRENT_MACHINE    			MACHINE_800_NORMAL
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_6037
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER19	//19-800一体化4代  12080
    #define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SUPPORT_NEW_DRIVER          1
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 
	#define SEND_SERVO_PARA_ONLINE      0  //实时下发伺服参数
	#define NEW_STEEPER_ANGLE_MODE      1  //新步距角换算方法
	#define CURRENT_MACHINE    	        MACHINE_800_NORMAL
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_12080_BELT       

#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER20	//20-800一体化4代  单X
    #define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SUPPORT_NEW_DRIVER          1//重复了一次
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 
	#define SEND_SERVO_PARA_ONLINE      0  //实时下发伺服参数
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角换算方法
	#define CURRENT_MACHINE    	        MACHINE_800_NORMAL
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_DOUBLE_X   
	    
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER21	//21-900一体化4代  单X
    #define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	//#define SUPPORT_NEW_DRIVER          1
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 
	#define SEND_SERVO_PARA_ONLINE      0  //实时下发伺服参数
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角换算方法
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif	
	#define CURRENT_MACHINE             MACHINE_900_NEW
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_DOUBLE_X       


#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER22	//22-800一体化激光切刀
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_LASER_CUTTER         1
	#define NEW_LASER_DEVICE            1
	#define LASER_DSP_PROCESS           1  //支持DSP发激光数据
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角换算方法
	#define ENBALE_TURNON_LASER_FUN     1  //风扇一直打开
	
	#define CURRENT_MACHINE             MACHINE_800_LASER_CUTTER
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_DOUBLE_X
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER23	//23-900一体化激光切刀
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif	
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_LASER_CUTTER         1
	#define NEW_LASER_DEVICE            1
	#define LASER_DSP_PROCESS           1  //支持DSP发激光数据
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 	
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角换算方法
	#define ENBALE_TURNON_LASER_FUN     1  //风扇一直打开
	
	#define CURRENT_MACHINE             MACHINE_900_LASER_CUTTER 
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_DOUBLE_X
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER24	//800一体化随动中压脚 单X60+Y86+Z60闭环1000线 
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define FOLLOW_INPRESS_FUN_ENABLE   1
	#define NEW_STRUCTURE_800_INPRESS   1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 
	
	#define CURRENT_MACHINE    	        MACHINE_800_NORMAL
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_SINGLE_X_INPERSSER   
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER25	//900一体化随动中压脚 单X60+Y86+Z60闭环1000线 
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define FOLLOW_INPRESS_FUN_ENABLE   1
	#define NEW_STRUCTURE_800_INPRESS   1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 
	#define CURRENT_MACHINE    	        MACHINE_900_NEW
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_SINGLE_X_INPERSSER   	

#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER26	//900一体化	自动换梭
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角换算方法
    #define CURRENT_MACHINE             MACHINE_900_BOBBIN
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_DOUBLE_X
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER27	//900一体化	直线切刀
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define ENABLE_LOOPER_CUTTER        1
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角换算方法
	
    #define CURRENT_MACHINE             MACHINE_900_SPEPPER_CUTTER
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_DOUBLE_X

#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER28	//900一体化	旋转切刀
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define DSP3_CUTER_DRIVER           1
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角换算方法	
    #define CURRENT_MACHINE             MACHINE_900_CUTTER
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_DOUBLE_X
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER29	//29-900一体化5代
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
 	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        0  //RFID 接口
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角方法
	#define CURRENT_MACHINE             MACHINE_900_NEW
	
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_DOUBLE_X	

#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER30	//30-800一体化5代
 	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        0  //RFID 接口
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角方法
	#define FOLLOW_INPRESS_FUN_ENABLE   1
	#define NEW_STRUCTURE_800_INPRESS   1
	#define FIRST_STITCH_NOT_ACTION     1
	
	#define CURRENT_MACHINE             MACHINE_800_NORMAL	
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_DOUBLE_X
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER31	//31-900一体化5代直线切刀 MASC4053
 	
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
	
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define ENABLE_LOOPER_CUTTER        1
	#define SC0413                      1
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        0  //RFID 接口
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角方法	
	
    #define CURRENT_MACHINE             MACHINE_900_SPEPPER_CUTTER
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_DOUBLE_X	
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER33	//33-800一体化5代激光
 	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define LASER_DSP_PROCESS           1  //支持DSP发激光数据
	#define ENABLE_LASER_CUTTER         1
	#define NEW_LASER_DEVICE            1
	#define INSERPOINT_ENABLE           0  //插补算法
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        0  //RFID 接口
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角方法
	#define CURRENT_MACHINE             MACHINE_800_NORMAL
	
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_DOUBLE_X	
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER34	//34-900一体化5代激光	
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
	#define LASER_DSP_PROCESS           1  //支持DSP发激光数据
	#define ENABLE_LASER_CUTTER         1
	#define NEW_LASER_DEVICE            1
 	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        0  //RFID 接口
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角方法
	#define CURRENT_MACHINE             MACHINE_900_NEW
	
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_DOUBLE_X	

#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER35	//35-800一体化5代 +RFID


 	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define SUPPORT_UNIFY_DRIVER        1 
	
	#define ENABLE_RFID_FUNCTION        1  //RFID 接口
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角方法
	#define FOLLOW_INPRESS_FUN_ENABLE   1
	#define NEW_STRUCTURE_800_INPRESS   1
	#define FIRST_STITCH_NOT_ACTION     1
	#define ENABLE_CONFIG_PARA          1  //支持参数配置
	
	#define CURRENT_MACHINE             MACHINE_800_NORMAL	
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_DOUBLE_X	
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER36	//36-900一体化5代+rfid
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
 	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        1  //RFID 接口
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角方法
	#define ENABLE_CONFIG_PARA          1  //支持参数配置
	#define FOLLOW_INPRESS_FUN_ENABLE   1
	#define NEW_STRUCTURE_800_INPRESS   1
	
	#define CURRENT_MACHINE             MACHINE_900_NEW	
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_DOUBLE_X

#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER37	//37-800一体化5代直线切刀 +RFID
 	
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define ENABLE_LOOPER_CUTTER        1
	#define SC0413                      1
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        1  //RFID 接口
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角方法	
	#define ENABLE_CONFIG_PARA          1  //支持参数配置
	#define USE_ENCODER_YJ_PULSE        1  //切刀原点用Z脉冲
	
    #define CURRENT_MACHINE             MACHINE_800_STEPPER_CUTTER
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_DOUBLE_X	

#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER38	//38-800一体化5代 +RFID+激光
 	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        1  //RFID 接口
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角方法
	#define FOLLOW_INPRESS_FUN_ENABLE   1
	#define NEW_STRUCTURE_800_INPRESS   1
	#define FIRST_STITCH_NOT_ACTION     1
	#define ENABLE_CONFIG_PARA          1  //支持参数配置
	
	#define ENBALE_TURNON_LASER_FUN     1  //风扇一直打开
	
	#define LASER_DSP_PROCESS           1  //支持DSP发激光数据
	#define ENABLE_LASER_CUTTER         1
	#define NEW_LASER_DEVICE            1

	#define CURRENT_MACHINE             MACHINE_800_NORMAL	
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_DOUBLE_X	
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39	//900一体化5代自动换梭+条码

	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
	#define MULTIPULE_IO_ENABLE         1 //多功能IO主控编程
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        1  //RFID 接口
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define NEW_STRUCTURE_800_INPRESS   1
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角换算方法
	#define ENABLE_CONFIG_PARA          1  //支持参数配置
	#define FOLLOW_INPRESS_FUN_ENABLE   1
//	#define NEW_STRUCTURE_800_INPRESS   1
	
    #define CURRENT_MACHINE             MACHINE_900_FIFTH_BOBBIN
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_BOBBIN	
	

#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER40	//900一体化5代旋转切刀+条码   5代 XY1000线闭环+Z开环+旋转电机+伺服切刀电机（自购）

	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
	#define MULTIPULE_IO_ENABLE         1 //多功能IO主控编程
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        1  //RFID 接口
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define NEW_STRUCTURE_800_INPRESS   1
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角换算方法
	#define ENABLE_CONFIG_PARA          1  //支持参数配置
	#define DSP3_CUTER_DRIVER           1
	#define FOLLOW_INPRESS_FUN_ENABLE   1

	
    #define CURRENT_MACHINE             MACHINE_900_FIFTH_CUTTER
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_BOBBIN	

#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER41	//800一体化5代双丝杠款
    
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define FOLLOW_INPRESS_FUN_ENABLE   1
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        1  //RFID 接口
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define NEW_STRUCTURE_800_INPRESS   1
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角换算方法
	#define ENABLE_CONFIG_PARA          1  //支持参数配置
	#define USE_ENCODER_Z_PULSE         1  //中压脚用Z脉冲

	#define CURRENT_MACHINE  MACHINE_800_BALLSCREW	
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_BOBBIN


#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER42	//37 +随动
 	
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define ENABLE_LOOPER_CUTTER        1
	#define SC0413                      1
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        1  //RFID 接口
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角方法	
	#define ENABLE_CONFIG_PARA          1  //支持参数配置
	#define USE_ENCODER_YJ_PULSE        1  //切刀原点用Z脉冲
	#define FOLLOW_INPRESS_FUN_ENABLE   1
	#define NEW_STRUCTURE_800_INPRESS   1
	
    #define CURRENT_MACHINE             MACHINE_800_STEPPER_CUTTER
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_DOUBLE_X	

#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER43	//39+ 随动 自动换梭
 	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
	#define MULTIPULE_IO_ENABLE         1 //多功能IO主控编程
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        1  //RFID 接口
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角换算方法
	#define ENABLE_CONFIG_PARA          1  //支持参数配置
	#define FOLLOW_INPRESS_FUN_ENABLE   1
	#define NEW_STRUCTURE_800_INPRESS   1
	
    #define CURRENT_MACHINE             MACHINE_900_FIFTH_BOBBIN
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_BOBBIN	
	
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER44	//40 +随动 旋转切刀
	#ifdef DAHAO_TYPE   
	   #undef DAHAO_TYPE
	   #define DAHAO_TYPE 405
    #endif
	#define MULTIPULE_IO_ENABLE         1 //多功能IO主控编程
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        1  //RFID 接口
	#define FIFTH_SC013K_PLATFORM       1  //五代新主板平台
	#define NEW_STEEPER_ANGLE_MODE16    1  //新步距角换算方法
	#define ENABLE_CONFIG_PARA          1  //支持参数配置
	#define DSP3_CUTER_DRIVER           1
	#define FOLLOW_INPRESS_FUN_ENABLE   1
	#define NEW_STRUCTURE_800_INPRESS   1
	
    #define CURRENT_MACHINE             MACHINE_900_FIFTH_CUTTER
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_FIFTH_BOBBIN
	
	
		
#elif COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER55
	
	#ifdef LITTLE_RAM_CPU 
		#undef LITTLE_RAM_CPU
	    #define LITTLE_RAM_CPU   		1//小RAM
    #endif
	
	#define SINGLE_X_MOTOR              1
	#define SUPPORT_NEW_DRIVER          1
	#define FOLLOW_INPRESS_FUN_ENABLE   1
	#define NEW_STRUCTURE_800_INPRESS   1
	#define ENABLE_SCRATCH_FUN          1 
	#define SC0413                      1
	
	#ifdef SECOND_GENERATION_PLATFORM   
	   #undef SECOND_GENERATION_PLATFORM
	   #define SECOND_GENERATION_PLATFORM   1//二代一体机平台
    #endif
	#define LASER_DSP_PROCESS           1  //支持DSP发激光数据
	#define SUPPORT_UNIFY_DRIVER        1 
	#define ENABLE_RFID_FUNCTION        1  //RFID 接口
	#define INSERPOINT_ENABLE           0  //插补算法
	#define ENABLE_LASER_CUTTER         1 
	#define FW_TYPE_SOLENOID_VALVE      1  //0-SOLENOID 1-VALVE
	#define NEW_CUT_MODE				1  //新剪线模式	
	#define FIRST_STITCH_NOT_ACTION     1
	#define SEND_SERVO_PARA_ONLINE      0
	#define NEW_STEEPER_ANGLE_MODE16      1  //新步距角方法
	
	#define YORG_REVERSE_LEVEL          1  //0802光耦=>0712光耦
	#define IORG_REVERSE_LEVEL          1  //0802光耦=>0712光耦  
	#define CURRENT_MACHINE    			MACHINE_800_NORMAL
	#define CURRENT_STEPPER_CONFIG_TYPE   CONFIG_MACHINE_TYPE_6037_55
	
#endif
//二代一体机中，DA1用于夹线器，五代中为DA0，在这里取反，这样可以统一按DA0处理
#if SECOND_GENERATION_PLATFORM
	#define		da1		da0_addr
	#define		da0		da1_addr
#else
	#define		da0		da0_addr 
	#define		da1		da1_addr
#endif


//--------------------------------------------------------------------------------------
// 功能配置、调试信息  
//--------------------------------------------------------------------------------------
#define DEBUG_NOPMOVE_SPEED              1  //调试空送速度用
#define COMPARE_NOPMOVE_SPEED            0  //对比空送速度用
#define USE_SC011N_PLATFORM              0  //换复位，增加RFID芯片
#define TEST_MAIN_MOTOR                  0  //测试主轴动作
#define DEBUG_XY_MOVEMODE                0  //调恒幅值支持
#define MAX_SPEED_3200                   1  //最高转速3200
#define ENABLE_3_5MM_HIGH_SPEED          0  //3.5MM提速
#define AUTO_CHANGE_PATTERN_FUNCTION     0  //自动换模板功能
#define CHANGE_DOUBLE_FRAMEWORK          0  //换双模板


#define USE_SEVERO_MOTOR_1635		     1  //1635主轴	
#define USE_SEVERO_MOTOR_0830       	 0
#define INPRESS_DELTA_RANGE              8
#define NEW_VERSION_CONTROL              1   //新版本定义 400B 405
#define NEW_CONTROL_PROTOCOL             0   //XY一包发出去
#define INPRESSER_LOWER_FUN              0   //前几针下降稍微低些。
#define SPECIAL_GO_ORIGIN_FUN            0   //每6次回原点
#define FUNCTION_AUTOSEWING              1   //自启动
#define AUTOSEWING_DETECTOR              0   //自启动--使用传感器
#define NEW_STRUCTURE_900_INPRESS        0   //900新结构中压脚
#define ENABLE_DSP2_USBLOADER            1  
#define DEBUG_MAIN_MOTOR   	   			 0
#define NEW_CLAMP_TYPE         			 1

#define CODE_BAR            			 1   //条码功能
#define Y_COMPENSATION      			 0   //Y轴齿轮间隙补偿
#define X_COMPENSATION      			 0   //X轴齿轮间隙补偿
#define ONESTEP_STOP        			 0   //一步停车到位
#define SPECIAL_STOP        		     0   //急停时继续走三针`
#define ENABLE_BLOW_AIR     			 0   //剪线后吹气
#define QUICK_MOVE_MODE     			 1  //空送提速
#define Y_AXIS_ORGIN_ENABLE 		     1
#define SUPPORT_NEW_Y_ORIGIN        	 0
#define STEPPER_WAITTING_PROTOCOL   	 1 //空送延时等待处理
#define ENABLE_SEND_STEPPER_SPEED  	     1 //允许步进协议发主轴转速

#define MACHINE_900_BOBBIN_DEBUG_MODE    0	//调试模式
#define DEBUG_POINT_COOR_MODE            0  //查询输出坐标和指针
#define UART1_DEBUG_OUTPUT_MODE          0
#define QUERY_NEW_PROTOCOL     	         0
#define DEBUG_DLG                        0
#define ERROR_OUTPUT_DEBUG	             0
#define DEBUG_CS3_INPUT        		     0
#define LASER_DEBUG_MODE                 0
#define DA1_OUTPUT_ANGLE    		     0   //输出动框角度
#define MOVING_DEBUG        			 0   //XY轴空跑
#define AUTO_DEBUG_MODE     		     0   //调试模式 
#define DEBUG_SPEED_OUT					 0   //查询状态输出速度规划
//============================================================
#if MAX_SPEED_3200
	#define MAX_SPEED_LIMIT     3200
#else
	#define MAX_SPEED_LIMIT     2800
#endif	

#if ENABLE_RFID_FUNCTION
	#define DMA_UART1_MODE 			0
#else
	#if UART1_DEBUG_OUTPUT_MODE
	#define DMA_UART1_MODE 			0
	#else
	#define DMA_UART1_MODE 			1
	#endif
#endif

//*****************************************************//

#define OPEN_LOOP_TIME      1
#define CLOSE_LOOP_TIME     0

#define MOTOR_INPRESS   	0
#define AIR_INPRESS    		1
#define NEW_MOTOR_INPRESS   2
#define FOLLOW_INPRESSER    3

#define TIME_PARAMETER_1    1

#define FOLLOW_INPRESS_HIGH 1
#define FOLLOW_INPRESS_LOW  0

#define DSP1  1
#define DSP2  2
#define DSP3  3
#define DSP4  4

#define CPU_PWM_PERIOD      30

//--------------------------------------------------------------------------------------
// condition compile
//--------------------------------------------------------------------------------------
#define AT_SOLENOID  1  // 1---tension+45 
//--------------------------------------------------------------------------------------
// frequency definition
//--------------------------------------------------------------------------------------
#define FX   24000000
#define PWM_FREQ 10000
#define DEADPOINT_SPD 150
//--------------------------------------------------------------------------------------
// system status definition
//--------------------------------------------------------------------------------------
#define RESERVE     0     // 00
#define FREE        1     // 01
#define READY       2     // 02
#define RUN         3     // 03
#define ERROR       4     // 04
#define PREWIND     5     // 05
#define WIND        6     // 06
#define INPRESS     7     // 07
#define POWEROFF    8     // 08
#define SINGLE      9     // 09
#define MANUAL      10    // 0A
#define SETOUT      11    // 0B
#define EMERSTOP    12    // 0C
#define PREEDIT     13    // 0D
#define EDIT        14    // 0E
#define NOEDIT      15    // 0F
#define FINISH      16    // 10
#define NEEDLE      17    // 11
#define WAITOFF     18    // 12
#define TRIM        19    // 13
#define SLACK       20    // 14
#define CHECKI03    21    // 15
#define CHECKI04    22    // 16
#define CHECKI05    23    // 17
#define CHECKI06    24    // 18
#define CHECKI07    25    // 19
#define CHECKI08    26    // 1A
#define CHECKI10    27    // 1B
#define EMERMOVE    28    // 1C
#define BOARDTEST   29    // 1D
#define RFID_WR     30    // 1E
#define CALSEW      31    // 1F
#define CHECKI11    32 
#define DOWNLOAD_DSP1    	  35
#define DOWNLOAD_DSP2    	  36
#define DOWNLOAD_DSP_CURVE    37
#define MULTIPULE_IO    	  38 
#define DOWNLOAD_DSP3    	  39
#define DOWNLOAD_DSP4    	  40
#define DOWNLOAD_SPFL         41

#define DOWNLOAD    72    //0x48

//--------------------------------------------------------------------------------------
// motor angle definition
//--------------------------------------------------------------------------------------
#define DEGREE_0    0 
#define DEGREE_43   122
#define DEGREE_53   150
#define DEGREE_63   179
#define DEGREE_180  512
#define DEGREE_360  1023
//--------------------------------------------------------------------------------------
// system error number definition
//--------------------------------------------------------------------------------------
#define OK 			0    // no error
#define ERROR_01	1    // pedal is not in normal position 
#define ERROR_02 	2    // emergency break
#define ERROR_03 	3    
#define ERROR_04 	4    // 300V undervoltage
#define ERROR_05 	5    // 300V overvoltage
#define ERROR_06 	6
#define ERROR_07 	7    // IPM overcurrent or overvoltage
#define ERROR_08 	8    // 24V overvoltage 
#define ERROR_09 	9    // 24V undervoltage
#define ERROR_10 	10
#define ERROR_11 	11
#define ERROR_12 	12
#define ERROR_13 	13   // no motor encoder connect
#define ERROR_14 	14   // motor is not normal 
#define ERROR_15 	15   // out of sewing range
#define ERROR_16	16   // needle is not in normal position 
#define ERROR_17 	17   // thread breakage detection
#define ERROR_18 	18   // cut knife is not in normal position	
#define ERROR_19 	19   // 急停开关未在正常位置
#define ERROR_20 	20   // machine overturn  	 	                    
#define ERROR_21 	21
#define ERROR_22 	22
#define ERROR_23 	23   // catcher is not in normal position  	 	           
#define ERROR_24 	24   // panel is not matching   	 	              
#define ERROR_25 	25   // X origin sensor is not normal  	 	        
#define ERROR_26 	26   // Y origin sensor is not normal  	 	        
#define ERROR_27 	27   // press origin sensor is not normal  	 	    
#define ERROR_28 	28   // catch thread origin sensor is not normal  
#define ERROR_29 	29   // inpress origin sensor is not normal  	 	  
#define ERROR_30 	30   // stepping motor driver communication is not normal
#define ERROR_31 	31   // stepping motor overcurrent
#define ERROR_32 	32   // stepping motor driver power is not normal
#define ERROR_33 	33
#define ERROR_34 	34
#define ERROR_35 	35
#define ERROR_36 	36
#define ERROR_37 	37
#define ERROR_38 	38
#define ERROR_39 	39
#define ERROR_40 	40
#define ERROR_41 	41
#define ERROR_42 	42
#define ERROR_43 	43
#define ERROR_44	44
#define ERROR_45	45
#define ERROR_46	46
#define ERROR_47	47
#define ERROR_48    48  //气压异常
#define ERROR_49    49
#define ERROR_50    50
#define ERROR_51    51
#define ERROR_52    52
#define ERROR_53    53

#define ERROR_66    66
#define ERROR_67    67
#define ERROR_68    68
#define ERROR_69    69
#define ERROR_70    70
#define ERROR_71    71

#define ERROR_80    80
#define ERROR_81    81
#define ERROR_82    82

#define ERROR_86    86
#define ERROR_87    87
#define ERROR_88    88
#define ERROR_89    89
#define ERROR_90    90
#define ERROR_91    91
#define ERROR_92    92
#define ERROR_93    93
#define ERROR_97    97
#define ERROR_98    98
#define ERROR_99    99
//--------------------------------------------------------------------------------------
// inpresser origin definition 
//--------------------------------------------------------------------------------------
#define IN_ORIGIN 71
//--------------------------------------------------------------------------------------
// definition
//--------------------------------------------------------------------------------------
#define UP         0
#define DOWN       1
#define OUT        0
#define IN         1
#define OFF        0
#define ON         1

#define DISABLE_FUN    0
#define ENABLE_FUN     1

//--------------------------------------------------------------------------------------
// pin definition
//--------------------------------------------------------------------------------------
#define UZKIN       p0_0
#define DV          p0_1
#define U24V        p0_2
#define FL1         p0_3   
#define POWER_FAULT p0_5
#define SNT_OVL     p0_5
#define OV_85V      p0_5
#define SENSOR8     p0_6        //INPUT 8
#define ADTCSM      p0_7        //INPUT 5

#if SECOND_GENERATION_PLATFORM == 0

 #if FIFTH_SC013K_PLATFORM
    #define POWER_OFF   p1_0
 	#define SNT_ON      p8_6
	#define SNT_H       p1_1
 #else
	#define SNT_ON      p1_0
	#define SNT_H       p1_1
 #endif
#endif

#define DVB         p1_2         
#define DVSM        p1_3
#define ADTC        p1_4
#define EXTSM       p1_5
#if FIFTH_SC013K_PLATFORM
	#define EXTEND      p10_6		//OUTPUT 1
#else
	#define EXTEND      p1_6		//OUTPUT 1
#endif	
#define DVA         p1_7     
  
#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER18    
	#define XORG        p2_2//p2_6    
	#define PORG        p2_6//小模板机未使用此引脚
#else
	#define XORG        p2_0 
	#define PORG        p2_2        //INPUT 1
#endif    


#define YORG        p2_1   

#define PSENS       p2_3        //INPUT 2  or 4
#define CORG        p2_4        //INPUT 4 or 2
#define CSENS       p2_5        //INPUT 3

#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER18    
	#define IORG        p2_0    
#else
	#define IORG        p2_6 
#endif  
      

#if SECOND_GENERATION_PLATFORM
#define SFSW        p2_3//p2_2 PSENS
#else
#define SFSW        p2_7
#endif

#define TH_BRK      p3_0
#define PAUSE       p3_1
#define T_OC        p3_2

#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER30
#define L_AIR       p4_2
#define FA          p3_3
#else
#define L_AIR       p3_3
#define FA          p4_2 

#endif
#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER20	|| COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER21 ||COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER29 ||COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER17 ||COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER23
	#define R_AIR       p4_5 //output-4
#else
	#define R_AIR       p3_4
#endif		
#define LM_AIR      p3_5	
#define FL_ON   	p3_6
#define FW   		p3_7		

#define OUTPUT_ON   p4_0
#define FL		   	p4_1   //FL
	
#define T_CLK       p4_3      //OUTPUT 6
#if AUTO_CHANGE_PATTERN_FUNCTION
#define T_DIR_2     p4_4      //OUTPUT 5
#define T_DIR       testpin      //OUTPUT 5
#else
#define T_DIR       p4_4      //OUTPUT 5
#endif
#define T_HALF      p4_5      //OUTPUT 4
#define BACKUP1  	p4_6
#define FR_ON  	    p4_7      //OUTPUT 2

#if SECOND_GENERATION_PLATFORM == 0
#define ALARM_LED   p5_1
#define PWR_LED     p5_2
#define SUM         p5_3
#endif
#define FK_OFF      p5_4      //OUTPUT 3

#define RDSET		p6_0   
#define RST_PN      p6_1
#define RXD0        p6_2
#define TXD0        p6_3

#define SDA         p7_0
#define SCL         p7_1
#define V           p7_2
#define V_          p7_3
#define W           p7_4
#define W_          p7_5

#define U           p8_0
#define U_          p8_1
#define ISM         p8_2
#define BLDC_ON     p8_3
#define PWR_ON      p8_4
#define P_TORG      p8_6   //CZ1923 扩展1
#define SNT5_ON     p8_6
#define AC_OVDT     p8_7

#define DA0         p9_3
#define DA1         p9_4

#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER40
#define SPISTE1     p10_1
#define SPISTE2     p10_0
#else
#define SPISTE1     p10_0
#define SPISTE2     p10_1
#endif

#if MULTIPULE_IO_ENABLE == 1
	#define SPISTE3     p0_4
	#define SPISTE4     p3_6
#else
	#define SPISTE3     p10_3
	#define SPISTE4     p10_2
#endif

#define SENSOR6     p10_4      //INPUT 6
#define SENSOR7     p10_5      //INPUT 7
#define P_TSENS     p10_6      //CZ1924 扩展2


#if SC0413
	#define TORG      ADTCSM   //输入5
	#define TSENS     PSENS    //输入2
#else
	#define TORG      P_TORG   //CZ1923 扩展1
	#define TSENS     P_TSENS  //CZ1924 扩展2
#endif

#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER20	|| COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER21 ||COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER29||COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER17
	#define INPRESS_PIN  			    T_DIR   //气阀5
#else
	#define INPRESS_PIN        		    FA
#endif	
#define BLOW_AIR           			    T_CLK  //气阀6
#define RED_ALARM_LED      			    EXTEND //气阀1
#define YELLOW_ALARM_LED   			    T_HALF //气阀4
#define GREEN_ALALRM_LED   			    FK_OFF //气阀3
#define BLOW_AIR2          			    T_HALF //气阀4
#define COOL_AIR    	   			    FR_ON  //气阀2
#define BLOW_AIR3          			    p3_4   //R_AIR

//==============旋转切刀==============================
#define DRILL_MOTOR_RUN    		        FK_OFF  //气阀3
#define DRILL_MOTOR_UPDOWN 			    T_CLK   //气阀6
#define DRILL_FOOTER       		        T_DIR   //气阀5
#define ROTATED_CUTTER_ORG      	    PORG    //输入1
#define ROTATED_CUTTER_DOWN_ORG         ADTCSM  //输入5

//==============自动换梭==============================
#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
	#define BOBBIN_CASE_ARM_ORG         PSENS   //输入2---5V
	#define BOBBIN_CASE_PLATFORM_ORG    ADTCSM  //输入5
	#define BOBBIN_CASE_SWITCH          PORG    //输入1
	#define MARKING_PEN 				T_CLK   //气阀6
#else
	#define BOBBIN_CASE_ARM_ORG         PSENS   //输入2---5V
	#define BOBBIN_CASE_PLATFORM_ORG    P_TORG  //扩展1
	#define BOBBIN_CASE_SWITCH          P_TSENS //扩展2
	#define MARKING_PEN 				R_AIR   //气动中压脚
	
#endif
#define BOBBIN_CASE_EMPTY_CHECK         CORG    //输入4
#define BOBBIN_CASE_ARM_OUT  		    T_HALF  //气阀4
#define BOBBIN_CASE_ARM_SCRATH          FL      //辅助


#define STEPPER_CUTTER_FOOTER           T_DIR   //气阀5
#define FAN_POWER                       T_HALF  //气阀4
#define CUTTER_HOLDING_PART             FK_OFF  //气阀3

#define OIL_EMPTY_DETECT        	    CSENS   //输入3--5V
#define CUTTER_DOWN_POSITON             PSENS   //输入2
//TASC
#if SECOND_GENERATION_PLATFORM

	#define LASER_POWER_PIN             FA      //X30
	#define LASET_POWER_SWITCH          T_DIR   //X32
	#define LASET_MIRROR_COVER          T_CLK   //X31
	#define LASER_FUN_PIN               T_HALF  //X33     
	#define LASER_HEADER_PROTECT        PORG    //输入1
	#define LASER_INDICATIOR_LED        da1
	#define LASER_LED_ON                150
#else
	#define LASER_POWER_PIN             FL      //辅助
	#define LASET_POWER_SWITCH          T_DIR   //气阀5
	#define LASET_MIRROR_COVER          T_CLK   //气阀6
	#define LASER_FUN_PIN               FK_OFF  //气阀3     
	#define LASER_HEADER_PROTECT        PORG    //输入1
	
#if FIFTH_SC013K_PLATFORM
	#define LASER_INDICATIOR_LED        p1_6	//
	#define LASER_LED_ON                1

#else
	#define LASER_INDICATIOR_LED        da1
	#define LASER_LED_ON                150
#endif

#endif
/*
二代一体机
X35---（3-FK_OFF),(4-L_AIR)
X36---（3-LM_AIR),(4-R_AIR)
X37---（3-FL),(4-FW)  剪线 ，拨线
X38--- （ACT1)
X39--- RFID
X41---TH_BRK
X42---IORG
X43---YORG
X44---XORG
X45---PAUSE
X46---PORG
X47---PSENS
X30---P4.2	FA
X31---P4.3	T_CLK
X32---P4.4  T_DIR
X33---P4.5	T_HALF
*/
#if AUTO_CHANGE_PATTERN_FUNCTION

	#define AUTO_FIRST_ASK_FRAMEWORK      T_HALF //气阀4 当前没有模板，请求送模板过来
    #define AUTO_ERROR_ALARM_INDICATOR	  EXTEND //气阀1 主控  是否在错误状态
	#define AUTO_ALLOW_FEEDARM_RELEASE    T_CLK  //气阀6 模板到位以后，夹紧压框，通知PLC机械臂离开
	#define AUTO_NOTIFY_READY_FOR_CHANGE  FK_OFF //气阀3 缝纫接近完成，通知PLC提前准备好更换模板
    #define AUTO_NEED_CHANGE_NOW          FL     //辅助气阀 缝纫真正完成了，允许机械臂更换模板
	#define AUTO_ALLOW_TAKE_OUT_FRAMEWORK T_DIR_2  //气阀5 放开压框，允许机械臂取走模板

    #define AUTO_FRAMEWORK_IN_POSITION    CORG   //输入4 机械臂将模板送到位了
	#define AUTO_ALLOW_SEWING_NOW         PORG   //输入1 机械臂取走模板或松开模板，允许主控开始缝制
    #define AUTO_ARM_HOLING_WELL          CSENS  //输入3 机械臂已经抓好模板

#endif

#if CHANGE_DOUBLE_FRAMEWORK 

	#define AUTO_LEFT_FRAME_STANDBY     FK_OFF   //左准备销 气阀3
	#define AUTO_RIGHT_FRAME_STANDBY    T_CLK    //右准备销 气阀6
	#define AUTO_LEFT_RUNNING_SWITCH    PSENS	 //输入2
	#define AUTO_RIGHT_RUNNING_SWITCH   ADTCSM   //输入5
	
#endif
	
//--------------------------------------------------------------------------------------
// communication definition
//--------------------------------------------------------------------------------------
#define BAUD 57600
#define BAUD_9600 		9600
#define BAUD_19200 		19200
#define BAUD_38400 		38400
#define BAUD_57600 		57600
#define BAUD_115200     115200
#define BAUD_RATE (FX/BAUD-8)>>4 
#define BAUD_RATE_9600  (FX/BAUD_9600-8)>>4
#define BAUD_RATE_115200 (FX/BAUD_115200-8)>>4 
#define BAUD_IIC 400000
#define BAUD_RATE_IIC (FX/BAUD_IIC-1)>>1 
#define BAUD_RATE_57600		(FX/BAUD_57600-8)>>4
#define BAUD_RATE_19200  		(FX/BAUD_19200-8)>>4
#define BAUD_RATE_38400  		(FX/BAUD_38400-8)>>4

//--------------------------------------------------------------------------------------
// interrupt priority definition
//--------------------------------------------------------------------------------------
#define TB1_IPL           0x01
#define TB3_IPL           0x01
#define TB4_IPL           0x01
#define TA0_IPL           0x03
#define UART_TRANSMIT_IPL 0x04
#define UART_RECEIVE_IPL  0x04

#define UART1_TRANSMIT_IPL_0 0x00
#define UART1_TRANSMIT_IPL_7 0x07
#define UART1_RECEIVE_IPL_0  0x00
#define UART1_RECEIVE_IPL_7  0x07

#define TB2_IPL           0x05
#define TB0_IPL           0x06
#define INT0_IPL          0x06
#define INT1_IPL          0x07
#define INT2_IPL          0x07
#define SPI_IPL           0x07   

#define CUT_START_ANGLE   		540  // 190 DEGREE
#define CUT_END_ANGLE	  		114  // 40 degree
#define WIPE_START_TIME   		10
#define WIPE_END_TIME 	  		30
#define TENSION_START_ANGLE		960
#define TENSION_END_ANGLE		114
#define INPRESS_DELAY_TIME  	30
#define RESOLUTION 				20   
#define MAIN_CONTROL_PARA_OFFSET  32
#if MAX_SPEED_3200
	#define MAXSPEED1				32
	#define MAXSPEED0				32
#else
	#define MAXSPEED1				28
	#define MAXSPEED0				28
#endif 	
#define MINSPEED				2
#define STOPANGLE				170
#define MOTORSTUCKTIME			1000
#define WIPEANDINPRESSTIME		50
//#define DEADPOINT				10
#define QUICKMOVETIME			30
#define ROTATE_ANGLE			230
#define MIN_X_DISTANCE			100
#define RotateOriginCheck  		1

#define	fabsm(z)	((z >= 0) ? (z) : -(z)) 


//2011-4-15
#define STEP_CURRENT_CONFIGURE 1
#define IGNORE_STOP        0xD0A0
#define OVC_DSP1           0xD1A1  // dsp1's motor overcurrent    //X OVER CURRENT   &Z
#define OVC_DSP2           0xD2A2  // dsp2's motor overcurrent    //Y OVER CURRENT
#define OVS_DSP1           0xD3A3  // 11 dsp1's motor            //X OVER SPEED  
#define OVS_DSP2           0xD4A4  // 74 dsp2's motor 	          //Y OVER SPEED 
#define OVD_DSP1           0xD5A5  // 12 dsp1's motor            //X OVER STEP
#define OVD_DSP2           0xD6A6  // 75 dsp2's motor            //Y OVER STEP
#define DSP_VERIFY_ERROR   			 0xC03F
#define DSP_UNDEFINE_COMMAND 		 0xC03E

#if LITTLE_RAM_CPU 
	#define TOTAL_STITCH_COUNTER         4000
#else
    #define TOTAL_STITCH_COUNTER         8000
#endif

#define HALF_STITCH_COUNTER          TOTAL_STITCH_COUNTER/2
#define HALF_WRITE_OFFSET            HALF_STITCH_COUNTER*3


#define PLATFORM_SC      1 
#define PLATFORM_ASC     2
#define PLATFORM_ESC     3
#define PLATFORM_HSC     4
#define PLATFORM_MSC     5
#define PLATFORM_MASC    6
#define PLATFORM_TASC    7

#define PATTERN_MACHINE  '4'

#define JUKI_STYLE        '0'
#define BROTHER_STYLE     '1'
#define DUERKOPP_STYLE    '2'
#define MITSUBISHI_STYLE  '4'

#define ENGINE_TYPE0       0
#define ENGINE_TYPE1       1
#define ENGINE_TYPE2       2
#define ENGINE_TYPE3       3
#define ENGINE_TYPE4       4
#define ENGINE_TYPE5       5
#define ENGINE_DEFAULT     '0'

#define MAIN_MOTOR_DEFINE  '0'
#define MAIN_MOTOR_0830    '1'
#define MAIN_MOTOR_0850    '2'
#define MAIN_MOTOR_1635    '3'
#define MAIN_MOTOR_1730    '4'
#define MAIN_MOTOR_2530    '5'

#define XY_FRAMES_NONE          '1'
#define XY_FRAMES_OPEN          '2'
#define XY_FRAMES_CLOSE_400     '3'
#define XY_FRAMES_CLOSE_1000    '4'
#define XY_FRAMES_SERVO         '5'

#define EXPAND_RESERVE          '0'

#define SOFTWARE_PLATFORM_SC    1
#define SOFTWARE_PLATFORM_AC    2


#if MULTIPULE_IO_ENABLE == 1

	#define NEW_CODE_FORMAT       1
	#define SPI_STM32_PORT        4
	#define DEF_FALSE             0
	#define DEF_TRUE              1
	typedef enum	//输入端口定义
	{
		xorg = 1, 		//1		X原点
		porg,			//2     输入1
		corg,			//3		输入4
		iorg,			//4		中压脚
		th_brk,			//5		断线
		adtcsm,			//6
		yorg,			//7
		psens,			//8		
		csens,			//9
		sfsw,			//10
		pause_sig,		//11
		dva,			//12
		dvb,			//13
		dvsm,			//14
		logic_input1,	//15
		logic_input2,	//16
		logic_input3,	//17
		logic_input4,	//18
		logic_input5,	//19
		logic_input6,	//20
		logic_input7,	//21
		logic_input8,	//22
		logic_input9,	//23
		logic_input10,	//24
		logic_input11,	//25
		logic_input12,	//26
		logic_input13,	//27
		logic_input14,	//28
		logic_input15,	//29
		logic_input16,	//30
		logic_input17,	//31
		logic_input18,	//32
		logic_input19,	//33
		logic_input20,	//34
		Xn,				//35
	  CODE_POWERON ,	//36
	  CODE_DVB_ACTION,	//37
	  CODE_DVA_ACTION,	//38
	  CODE_THREAD_BREAK,//39
	  CODE_WIPPER_ACTION,			//40
	  CODE_TRIM_ACTION,				//41
	  CODE_ORGIN_COMMAND,			//42
	  CODE_INPRESS_UP,				//43
	  CODE_INPRESS_DOWN,			//44
	  CODE_PAUSE_ACTION,			//45
	  CODE_STEP_FORWARD,			//46
	  CODE_STEP_BACKWARD,			//47
	  CODE_PATTERN_STOP,			//48
	  CODE_PATTERN_FUN1,			//49
	  CODE_PATTERN_FUN2,			//50
	  CODE_PATTERN_FUN3,			//51
	  CODE_PATTERN_FUN4,			//52
	  CODE_PATTERN_FUN5,			//53
	  CODE_PATTERN_FUN6,			//54
	  CODE_PATTERN_SECOND_ORIGIN,	//55
	  CODE_PATTERN_LASER_START,		//56
	  CODE_PATTERN_LASER_END,		//57
	  CODE_PATTERN_MARK,			//58
	  CODE_SCAN_CODEER,				//59
	  CODE_ERROR_STATUS,			//60
	  CODE_MAIN_MOTOR_RUNNING,		//61
	  CODE_IN_SEWING,				//62
	  CODE_BACK_TO_READY,			//63
	  CODE_WAITING_IO_RESPONE,		//64
	  CODE_PATTERN_ROTATE,			//65
	  CODE_PATTERN_PAUSE,			//66
	  CODE_PATTERN_NOPMOVE,			//67
	  Coden 						//68
	}X_TypeDef;

	typedef enum 
	{
		t_dir_extend = 1,	//1     气阀1
		fr_on,    			//2  	气阀2
		fk_off,       		//3	 	气阀3
		t_half,       		//4		气阀4
		t_dir,        		//5	 	气阀5
		t_clk,        		//6	 	气阀6
		lm_air,       		//7     外压框		
		r_air,        		//8     中压脚
		l_air,       		//9		气剪线
		fl,           		//10	辅助		
		fa,           		//11    电剪线
		fw,	          		//12    拨线
		da1v,				//13	
		Yn,					//14
		logic_output1,		//15
		logic_output2,		//16
		logic_output3,		//17
		logic_output4,		//18
		logic_output5,		//19
		logic_output6,		//20
		logic_output7,		//21
		logic_output8,		//22
		logic_output9,		//23
		logic_output10,		//24
		logic_output11,		//25
		logic_output12,		//26
		logic_output13,		//27
		logic_output14,		//28
		logic_output15,		//29
		logic_output16,		//30
		logic_output17,		//31
		logic_output18,		//32
		logic_output19,		//33
		auto_run_fun,		//34
		Ylogic,				//35
		clear_input,		//36
		motor_output1,		//37
		motor_output2,		//38
		motor_output3,		//39
		motor_output4,		//40
		motor_output5,		//41
		motor_output6,		//42
		motor_output7,		//43
		motor_output8,		//44
		motor_output9,		//45
		motor_output10,		//46
		motor_output11,		//47
		motor_output12,		//48
		motor_output13,		//49
		motor_output14,		//50
		motor_output15,		//51
		motor_output16,		//52
		sc0714_output1,     //53
		sc0714_output2,     //54
		sc0714_output3,     //55
		sc0714_output4,     //56
		sc0714_output5,     //57
		sc0714_output6,     //58
		sc0714_output7,     //59
		sc0714_output8,     //60
		sc0714_output9,     //61
		sc0714_output10,    //62
		sc0714_output11,     //63
		sc0714_output12,     //64
		sc0714_output13,     //65
		sc0714_output14,     //66
		sc0714_output15,     //67
		sc0714_output16,     //68
		sc0714_output17,     //69
		sc0714_output18,     //70
		sc0714_output19,     //71
		sc0714_output20,     //72
		sc0714_output21,     //73
		sc0714_output22,     //74
		sc0714_output23,     //75
		sc0714_output24,     //76
		sc0714_output25,     //77
		sc0714_output26,     //78
		sc0714_output27,     //79
		sc0714_output28,     //80
		sc0714_output29,     //81
		sc0714_output30,     //82
		sc0714_wait_signal1,  //83
		sc0714_wait_signal30 = 113,
		sc0714_register1 =114,
		sc0714_register30 =143,
		
	}Y_TypeDef;
#endif


 
#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------

