
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
	���ƣ��жϱ��ο��ͺ���һ���ǿ��ͶΡ�����λ���������
	������pat������ָ����ͻ������һ������룬���������жϿ��ͺ���ĳ���������ʲô��
	����ֵ��-3-�������ͺ��Ѿ�����������û�г�������
			-2-����Ĳ������ǿ�����
			-1-����û���ҵ����һ����ͣ���������
			0-������Ϊ���Ͷ�
			1-����Ϊ�����
			2-����Ϊ���ʶ�
			3-����Ϊ�����и��
			4-����Ϊֱ���е���
			5-����Ϊ��ת�е���
			6-����Ϊ�����и������ת�е��Σ���ʱ���������ǣ�������+���⿪ʼ��+��ת�е�
			  �Ƕ������룬	  ��ʱ�����Ǽ����и�Ҳ��������ת�е����ɻ��;���
			����-δ����
	˵��������ʱ���ô˺����������жϺ����ĳ���������ʲô����Σ�Ŀǰ�������ڷ�ֹ�����
	      ʹ��V�����ӹ�ʱ����ǰ��������
	����޸����ڣ�2018-9-17
*/
INT8 check_subsequent_special_segment(PATTERN_DATA* pat)
{
	UINT8 ret=0;
	//�жϴ���ĵ�ǰ�����Ƿ��ǿ�����
	ret = IsNopMoveCode(pat);
	if(ret==0)
	{
		return -2;//����Ĳ������ǿ�����
	}

	//==========����ǿ����룬����ҵ����һ�������==========
	if(ret==1)
	{
		do
		{
			pat++;
			ret = IsNopMoveCode(pat);
			if(ret!=2)//�������һ�����
			{
				if(IsPatternEndCode(pat)==1)//����������������룬��ôǿ���˳�
				{
					break;
				}
			}
		}while( (ret!=2) );
			
		if(IsPatternEndCode(pat)==1)
		{
			return -1;//����û���ҵ����һ����ͣ���������
		}
	}
	
	//==========�������е������Ѿ��ҵ������һ�������==========
	pat++;//�鿴���ͶΣ����һ������룩�����һ��
	//�����ı�����ٶ���͸ı�����ٶ���
	while( (IsChangeSewSpeedCode(pat)==1) || (IsChangeNopMoveSpeedCode(pat)==1) )
	{
		pat++;
	}
	
	if(IsPatternEndCode(pat)==1)//�����������������
	{
		return -3;//�������ͺ��Ѿ�����������û�г�������
	}
	if( IsNopMoveCode(pat)!=0 )//����Ļ��ǿ�����
	{
		return 0;//������Ϊ���Ͷ�
	}
	if(IsPenStartCode(pat)==1)//���ʿ�ʼ��
	{
		return 2;//����Ϊ���ʶ�
	}
	if(IsLaserStartCode(pat)==1)//���⿪ʼ��
	{
		//��ʱ���������ǣ�������+���⿪ʼ��+��ת�е��Ƕ������룬��ʱ�����Ǽ����и�Ҳ��������ת�е����ɻ��;���
		pat++;
		if(IsRotateCutterSetAngleCode(pat)==1)
		{
			return 6;//����Ϊ�����и������ת�е���
		}
		return 3;//����Ϊ�����и��
	}
	
	//��ת�е���ʼ�������ת�е��Ƕ�������
	if( (IsRotateCutterStartCode(pat)==1) || (IsRotateCutterSetAngleCode(pat)==1) )
	{
		return 5;//����Ϊ��ת�е���
	}

	if(IsSewCode(pat)==1)//����ǳ����룬�͵úú��ж��µ�����������һ����
	{
		//==========�������е������ʾ���Ͷκ��ǳ����룬��ʱҪ�ú��ж���==========
		ret=0;
		do
		{
			pat--;//�ص���һ�룬�ʼ��һ�λص��������һ����ͣ�Ȼ�󰤸���ǰɨ��
			if(IsPatternStartCode(pat)==1)
			{
				ret = 1;//ֱ���ҵ��˻����ļ���ʼ�룬˵����һ�ξ��ǵ����ķ���
			}
			else if( (IsPenEndCode(pat)==1) || (IsLaserEndCode(pat)==1) || (IsRotateCutterEndCode(pat)==1)  )
			{
				
				ret = 1;//�����ҵ����ֹ��ܶεĽ����룬˵����һ�ξ��ǵ����ķ���
			}
			else if(IsRotateCutterSetAngleCode(pat)==1)
			{
				ret = 1;//�����ҵ���ת�е��Ƕ������룬˵����һ��ʵ�����ǵ����ķ��ƣ���һ����û�п����ر������ת�е��Σ�
			}
			else if(IsPenStartCode(pat)==1)
			{
				ret = 2;//�����ҵ����ʿ�ʼ�룬��һ�λ��ʶ�û�н�����ֱ�ӿ����ˣ���ô��һ��ʵ���ϻ��ǻ��ʶ�
			}
			else if(IsLaserStartCode(pat)==1)
			{
				ret = 3;//�����ҵ����⿪ʼ�룬��һ�μ����û�н�����ֱ�ӿ����ˣ���ô��һ��ʵ���ϻ��Ǽ����
			}

			
		}while(ret==0);
		return ret;
		
	}
	
	return 1;//�������е��˴�,���س����
}
// �жϵ�ǰ�����ǲ��ǻ�����ʼ��
UINT8 IsPatternStartCode(PATTERN_DATA* pat)
{
	//��ǰ�����ǻ�����ʼ��
	if( ( ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x00) )
	{
		return 1;
	}
	return 0;
}
// �жϵ�ǰ�����ǲ��ǻ���������
UINT8 IsPatternEndCode(PATTERN_DATA* pat)
{
	//��ǰ�����ǻ���������
	if( ( ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x01) )
	{
		return 1;
	}
	return 0;	
}
// �жϵ�ǰ�����ǲ��Ǽ�����
UINT8 IsCutCode(PATTERN_DATA* pat)
{
	//��ǰ�����Ǽ�����
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x04) )
	{
		return 1;
	}
	return 0;
}

// �жϵ�ǰ�����ǲ��ǳ����룺0-�ǳ����룬1-�����룬2-���һ�복����
UINT8 IsSewCode(PATTERN_DATA* pat)
{
	//�ж��ǲ��ǳ�����
	if( ((pat->func&0xF0)==0x20) || ((pat->func&0xF0)==0x30) )
	{
		return 1;//������
	}
	if( ((pat->func&0xF0)==0x60) || ((pat->func&0xF0)==0x70) )
	{
		return 2;//���һ�복����
	}
	
	return 0;
}

// �жϵ�ǰ�����ǲ��ǿ����룺0-�ǿ����룬1-�����룬2-���һ�������
UINT8 IsNopMoveCode(PATTERN_DATA* pat)
{
	//�ж��ǲ��ǿ�����
	if( ((pat->func&0xF0)==0x00)  )
	{
		return 1;//������
	}
	//�ж��ǲ������һ�����
	else if( ((pat->func&0xF0)==0x10) || ((pat->func&0xF0)==0x50) )
	{
		return 2;//���һ�������
	}

	return 0;
}

// �жϵ�ǰ�����ǲ��Ǹı�����ٶ���
UINT8 IsChangeSewSpeedCode(PATTERN_DATA* pat)
{
	//�ǲ��Ǹı�����ٶȴ���
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xF0)==0xC0) )
		&&((pat->xstep&0xFF)==0x14)
		)
	{
		return 1;//�ı�����ٶ���
	}

	return 0;
}

// �жϵ�ǰ�����ǲ��Ǹı�����ٶ���
UINT8 IsChangeNopMoveSpeedCode(PATTERN_DATA* pat)
{
	//�ǲ��Ǹı�����ٶ���
	if( (((pat->func&0xFF)==0x80) )
		&&((pat->xstep&0xFF)==0x15)
		)
	{
		return 1;//�ı�����ٶ���
	}

	return 0;
}


// �жϵ�ǰ�����ǲ��Ǽ��������
UINT8 IsLaserEndCode(PATTERN_DATA* pat)
{
	//�ж��ǲ��Ǽ��������
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x0B || (pat->xstep&0xFF)==0x05)
		&&((pat->ystep&0xFF)==0x03) )
	{
		return 1;//�����и������
	}
		
	return 0;
}

// �жϵ�ǰ�����ǲ��Ǽ��⿪ʼ��
UINT8 IsLaserStartCode(PATTERN_DATA* pat)
{
	//�ж��ǲ��Ǽ��⿪ʼ��
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x0B || (pat->xstep&0xFF)==0x05)
		&&((pat->ystep&0xFF)==0x04) )
	{
		return 1;//�����иʼ��
	}
		
	return 0;
}

// �жϵ�ǰ�����ǲ��ǻ��ʽ�����
UINT8 IsPenEndCode(PATTERN_DATA* pat)
{
	//�ж��ǲ��ǻ��ʽ�����
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x0B || (pat->xstep&0xFF)==0x05)
		&&((pat->ystep&0xFF)==0x00) )
	{
		return 1;//���ʽ�����
	}
		
	return 0;
}

// �жϵ�ǰ�����ǲ��ǻ��ʿ�ʼ��
UINT8 IsPenStartCode(PATTERN_DATA* pat)
{
	//�ж��ǲ��ǻ��ʿ�ʼ��
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x0B || (pat->xstep&0xFF)==0x05)
		&&((pat->ystep&0xFF)==0x01) )
	{
		return 1;//���ʿ�ʼ��
	}

	return 0;
}

// �жϵ�ǰ�����ǲ�����ת�е�������
UINT8 IsRotateCutterEndCode(PATTERN_DATA* pat)
{	
	//�ж��ǲ�����ת�е�������
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x07)
		&&((pat->ystep&0xFF)==0x01) )
	{
		return 1;//��ת�е�������
	}
	return 0;
}
// �жϵ�ǰ�����ǲ�����ת�е���ʼ��
UINT8 IsRotateCutterStartCode(PATTERN_DATA* pat)
{
	//�ж��ǲ�����ת�е���ʼ��
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x07)
		&&((pat->ystep&0xFF)==0x00) )
	{
		return 1;//��ת�е���ʼ��
	}

	return 0;
}
// �жϵ�ǰ�����ǲ�����ת�е��Ƕ�������
UINT8 IsRotateCutterSetAngleCode(PATTERN_DATA* pat)
{
	//�ж��ǲ�����ת�е��Ƕ�������
	if( (((pat->func&0xFF)==0x80) || ((pat->func&0xFF)==0xC0))
		&&((pat->xstep&0xFF)==0x0E) )
	{
		return 1;//��ת�е��Ƕ�������
	}

	return 0;
}







