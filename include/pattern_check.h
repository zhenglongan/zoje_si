/*
--------------------------------------------------------------------------------------
      COPYRIGHT(C) 2018 Beijing xingdahao technology Co., Ltd.
                     ALL RIGHTS RESERVED 
  Project Number: sewing_machine_controller 
  File Name : pattern_check.c
  Description: pattern check functions 
  Version     Date     Author    Description
  0.0.1		2018.9.17  zla		

--------------------------------------------------------------------------------------

*/
/*
	用于判断花样码类型
*/
#ifndef __PATTERN_CHECK_H
#define __PATTERN_CHECK_H

#include "typedef.h"      //Data type define


/****************************************************************
					 单个花样码判断函数声明
				 
****************************************************************/
// 判断当前花样是不是花样开始码
UINT8 IsPatternStartCode(PATTERN_DATA* pat);
// 判断当前花样是不是花样结束码
UINT8 IsPatternEndCode(PATTERN_DATA* pat);
// 判断当前花样是不是改变缝制速度码
UINT8 IsChangeSewSpeedCode(PATTERN_DATA* pat);
// 判断当前花样是不是剪线码
UINT8 IsCutCode(PATTERN_DATA* pat);
// 判断当前花样是不是车缝码：0-非车缝码，1-车缝码，2-最后一针车缝码
UINT8 IsSewCode(PATTERN_DATA* pat);
// 判断当前花样是不是空送码：0-非空送码，1-空送码，2-最后一针空送码
UINT8 IsNopMoveCode(PATTERN_DATA* pat);
// 判断当前花样是不是激光结束码
UINT8 IsLaserEndCode(PATTERN_DATA* pat);
// 判断当前花样是不是激光开始码
UINT8 IsLaserStartCode(PATTERN_DATA* pat);
// 判断当前花样是不是画笔结束码
UINT8 IsPenEndCode(PATTERN_DATA* pat);
// 判断当前花样是不是画笔开始码
UINT8 IsPenStartCode(PATTERN_DATA* pat);
// 判断当前花样是不是旋转切刀结束码
UINT8 IsRotateCutterEndCode(PATTERN_DATA* pat);
// 判断当前花样是不是旋转切刀开始码
UINT8 IsRotateCutterStartCode(PATTERN_DATA* pat);
// 判断当前花样是不是旋转切刀角度设置码
UINT8 IsRotateCutterSetAngleCode(PATTERN_DATA* pat);
// 判断当前花样是不是改变空送速度码
UINT8 IsChangeNopMoveSpeedCode(PATTERN_DATA* pat);



/****************************************************************
					 功能性花样判断函数声明
				 
****************************************************************/
/*
	名称：判断本段空送后下一段是空送段、车缝段还是其他段
	参数：pat：必须指向空送或者最后一针空送码，这样方便判断空送后面的车缝是属于什么段
	返回值：-3-花样空送后已经结束，后面没有车缝码了
			-2-传入的参数不是空送码
			-1-后续没有找到最后一针空送，分析错误
			0-后续仍为空送段
			1-后续为车缝段
			2-后续为画笔段
			3-后续为激光切割段
			4-后续为直线切刀段
			5-后续为旋转切刀段
			6-后续为激光切割或者旋转切刀段，有时候花样数据是：空送码+激光开始码+旋转切刀
			  角度设置码，	  此时可能是激光切割也可能是旋转切刀，由机型决定
			其他-未定义
	说明：空送时调用此函数，可以判断后续的车缝码属于什么代码段，目前可以用于防止特殊段
	      使用V型起缝加固时调过前几针的情况
	最后修改日期：2018-9-17
*/
INT8 check_subsequent_special_segment(PATTERN_DATA* pat);


#endif

