
/*
--------------------------------------------------------------------------------------
      COPYRIGHT(C) 2018 Beijing xingdahao technology Co., Ltd.
                     ALL RIGHTS RESERVED 
  Project Number: sewing_machine_controller 
  File Name : pattern_check.c
  Description: pattern check functions 
  Version     Date     Author    Description
--------------------------------------------------------------------------------------

*/

#include "..\..\include\pattern_check.h"

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
INT8 check_subsequent_special_segment(PATTERN_DATA* pat)
{
	UINT8 ret=0;
	//判断传入的当前花样是否是空送码
	ret = IsNopMoveCode(pat);
	if(ret==0)
	{
		return -2;//传入的参数不是空送码
	}

	//==========如果是空送码，向后找到最后一针空送码==========
	if(ret==1)
	{
		do
		{
			pat++;
			ret = IsNopMoveCode(pat);
			if(ret!=2)//不是最后一针空送
			{
				if(IsPatternEndCode(pat)==1)//如果遇到花样结束码，那么强制退出
				{
					break;
				}
			}
		}while( (ret!=2) );
			
		if(IsPatternEndCode(pat)==1)
		{
			return -1;//后续没有找到最后一针空送，分析错误
		}
	}
	
	//==========程序运行到这里已经找到了最后一针空送码==========
	pat++;//查看空送段（最后一针空送码）后的下一针
	//跳过改变缝制速度码和改变空送速度码
	while( (IsChangeSewSpeedCode(pat)==1) || (IsChangeNopMoveSpeedCode(pat)==1) )
	{
		pat++;
	}
	
	if(IsPatternEndCode(pat)==1)//如果遇到花样结束码
	{
		return -3;//花样空送后已经结束，后面没有车缝码了
	}
	if( IsNopMoveCode(pat)!=0 )//后面的还是空送码
	{
		return 0;//后续仍为空送段
	}
	if(IsPenStartCode(pat)==1)//画笔开始码
	{
		return 2;//后续为画笔段
	}
	if(IsLaserStartCode(pat)==1)//激光开始码
	{
		//有时候花样数据是：空送码+激光开始码+旋转切刀角度设置码，此时可能是激光切割也可能是旋转切刀，由机型决定
		pat++;
		if(IsRotateCutterSetAngleCode(pat)==1)
		{
			return 6;//后续为激光切割或者旋转切刀段
		}
		return 3;//后续为激光切割段
	}
	
	//旋转切刀开始码或者旋转切刀角度设置码
	if( (IsRotateCutterStartCode(pat)==1) || (IsRotateCutterSetAngleCode(pat)==1) )
	{
		return 5;//后续为旋转切刀段
	}

	if(IsSewCode(pat)==1)//如果是车缝码，就得好好判断下到底是属于哪一段了
	{
		//==========程序运行到这里表示空送段后是车缝码，此时要好好判断下==========
		ret=0;
		do
		{
			pat--;//回到上一针，最开始第一次回到的是最后一针空送，然后挨个向前扫描
			if(IsPatternStartCode(pat)==1)
			{
				ret = 1;//直接找到了花样文件开始码，说明下一段就是单纯的缝制
			}
			else if( (IsPenEndCode(pat)==1) || (IsLaserEndCode(pat)==1) || (IsRotateCutterEndCode(pat)==1)  )
			{
				
				ret = 1;//最先找到各种功能段的结束码，说明下一段就是单纯的缝制
			}
			else if(IsRotateCutterSetAngleCode(pat)==1)
			{
				ret = 1;//最先找到旋转切刀角度设置码，说明下一段实际上是单纯的缝制（上一段是没有开启关闭码的旋转切刀段）
			}
			else if(IsPenStartCode(pat)==1)
			{
				ret = 2;//最先找到画笔开始码，上一段画笔段没有结束码直接空送了，那么下一段实际上还是画笔段
			}
			else if(IsLaserStartCode(pat)==1)
			{
				ret = 3;//最先找到激光开始码，上一段激光段没有结束码直接空送了，那么下一段实际上还是激光段
			}

			
		}while(ret==0);
		return ret;
		
	}
	
	return 1;//不会运行到此处,返回车缝段
}
// 判断当前花样是不是花样开始码
UINT8 IsPatternStartCode(PATTERN_DATA* pat)
{
	//当前花样是花样开始码
	if( ( ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x00) )
	{
		return 1;
	}
	return 0;
}
// 判断当前花样是不是花样结束码
UINT8 IsPatternEndCode(PATTERN_DATA* pat)
{
	//当前花样是花样结束码
	if( ( ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x01) )
	{
		return 1;
	}
	return 0;	
}
// 判断当前花样是不是剪线码
UINT8 IsCutCode(PATTERN_DATA* pat)
{
	//当前花样是剪线码
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x04) )
	{
		return 1;
	}
	return 0;
}

// 判断当前花样是不是车缝码：0-非车缝码，1-车缝码，2-最后一针车缝码
UINT8 IsSewCode(PATTERN_DATA* pat)
{
	//判断是不是车缝码
	if( ((pat->func&0xF0)==0x20) || ((pat->func&0xF0)==0x30) )
	{
		return 1;//车缝码
	}
	if( ((pat->func&0xF0)==0x60) || ((pat->func&0xF0)==0x70) )
	{
		return 2;//最后一针车缝码
	}
	
	return 0;
}

// 判断当前花样是不是空送码：0-非空送码，1-空送码，2-最后一针空送码
UINT8 IsNopMoveCode(PATTERN_DATA* pat)
{
	//判断是不是空送码
	if( ((pat->func&0xF0)==0x00)  )
	{
		return 1;//空送码
	}
	//判断是不是最后一针空送
	else if( ((pat->func&0xF0)==0x10) || ((pat->func&0xF0)==0x50) )
	{
		return 2;//最后一针空送码
	}

	return 0;
}

// 判断当前花样是不是改变缝制速度码
UINT8 IsChangeSewSpeedCode(PATTERN_DATA* pat)
{
	//是不是改变缝制速度代码
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xF0)==0xC0) )
		&&((pat->xstep&0xFF)==0x14)
		)
	{
		return 1;//改变缝制速度码
	}

	return 0;
}

// 判断当前花样是不是改变空送速度码
UINT8 IsChangeNopMoveSpeedCode(PATTERN_DATA* pat)
{
	//是不是改变空送速度码
	if( (((pat->func&0xFF)==0x80) )
		&&((pat->xstep&0xFF)==0x15)
		)
	{
		return 1;//改变空送速度码
	}

	return 0;
}


// 判断当前花样是不是激光结束码
UINT8 IsLaserEndCode(PATTERN_DATA* pat)
{
	//判断是不是激光结束码
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x0B || (pat->xstep&0xFF)==0x05)
		&&((pat->ystep&0xFF)==0x03) )
	{
		return 1;//激光切割结束码
	}
		
	return 0;
}

// 判断当前花样是不是激光开始码
UINT8 IsLaserStartCode(PATTERN_DATA* pat)
{
	//判断是不是激光开始码
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x0B || (pat->xstep&0xFF)==0x05)
		&&((pat->ystep&0xFF)==0x04) )
	{
		return 1;//激光切割开始码
	}
		
	return 0;
}

// 判断当前花样是不是画笔结束码
UINT8 IsPenEndCode(PATTERN_DATA* pat)
{
	//判断是不是画笔结束码
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x0B || (pat->xstep&0xFF)==0x05)
		&&((pat->ystep&0xFF)==0x00) )
	{
		return 1;//画笔结束码
	}
		
	return 0;
}

// 判断当前花样是不是画笔开始码
UINT8 IsPenStartCode(PATTERN_DATA* pat)
{
	//判断是不是画笔开始码
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x0B || (pat->xstep&0xFF)==0x05)
		&&((pat->ystep&0xFF)==0x01) )
	{
		return 1;//画笔开始码
	}

	return 0;
}

// 判断当前花样是不是旋转切刀结束码
UINT8 IsRotateCutterEndCode(PATTERN_DATA* pat)
{	
	//判断是不是旋转切刀结束码
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x07)
		&&((pat->ystep&0xFF)==0x01) )
	{
		return 1;//旋转切刀结束码
	}
	return 0;
}
// 判断当前花样是不是旋转切刀开始码
UINT8 IsRotateCutterStartCode(PATTERN_DATA* pat)
{
	//判断是不是旋转切刀开始码
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x07)
		&&((pat->ystep&0xFF)==0x00) )
	{
		return 1;//旋转切刀开始码
	}

	return 0;
}
// 判断当前花样是不是旋转切刀角度设置码
UINT8 IsRotateCutterSetAngleCode(PATTERN_DATA* pat)
{
	//判断是不是旋转切刀角度设置码
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x0E) )
	{
		return 1;//旋转切刀角度设置码
	}

	return 0;
}







