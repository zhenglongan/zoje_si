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
	�����жϻ���������
*/
#ifndef __PATTERN_CHECK_H
#define __PATTERN_CHECK_H

#include "typedef.h"      //Data type define


/****************************************************************
					 �����������жϺ�������
				 
****************************************************************/
// �жϵ�ǰ�����ǲ��ǻ�����ʼ��
UINT8 IsPatternStartCode(PATTERN_DATA* pat);
// �жϵ�ǰ�����ǲ��ǻ���������
UINT8 IsPatternEndCode(PATTERN_DATA* pat);
// �жϵ�ǰ�����ǲ��Ǹı�����ٶ���
UINT8 IsChangeSewSpeedCode(PATTERN_DATA* pat);
// �жϵ�ǰ�����ǲ��Ǽ�����
UINT8 IsCutCode(PATTERN_DATA* pat);
// �жϵ�ǰ�����ǲ��ǳ����룺0-�ǳ����룬1-�����룬2-���һ�복����
UINT8 IsSewCode(PATTERN_DATA* pat);
// �жϵ�ǰ�����ǲ��ǿ����룺0-�ǿ����룬1-�����룬2-���һ�������
UINT8 IsNopMoveCode(PATTERN_DATA* pat);
// �жϵ�ǰ�����ǲ��Ǽ��������
UINT8 IsLaserEndCode(PATTERN_DATA* pat);
// �жϵ�ǰ�����ǲ��Ǽ��⿪ʼ��
UINT8 IsLaserStartCode(PATTERN_DATA* pat);
// �жϵ�ǰ�����ǲ��ǻ��ʽ�����
UINT8 IsPenEndCode(PATTERN_DATA* pat);
// �жϵ�ǰ�����ǲ��ǻ��ʿ�ʼ��
UINT8 IsPenStartCode(PATTERN_DATA* pat);
// �жϵ�ǰ�����ǲ�����ת�е�������
UINT8 IsRotateCutterEndCode(PATTERN_DATA* pat);
// �жϵ�ǰ�����ǲ�����ת�е���ʼ��
UINT8 IsRotateCutterStartCode(PATTERN_DATA* pat);
// �жϵ�ǰ�����ǲ�����ת�е��Ƕ�������
UINT8 IsRotateCutterSetAngleCode(PATTERN_DATA* pat);
// �жϵ�ǰ�����ǲ��Ǹı�����ٶ���
UINT8 IsChangeNopMoveSpeedCode(PATTERN_DATA* pat);



/****************************************************************
					 �����Ի����жϺ�������
				 
****************************************************************/
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
INT8 check_subsequent_special_segment(PATTERN_DATA* pat);


#endif

