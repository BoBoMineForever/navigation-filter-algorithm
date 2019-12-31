%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                    ����������
%
%  �������:
%  t          ����ʱ��
%  T          ���沽��
%  atti       ��������������򣨵�λ���ȣ�
%  atti_rate  ������ʡ��������ʡ��������ʣ���λ����/�룩
%  veloB      �ɻ��˶��ٶȡ���X����Y��ͷ��Z���򣨵�λ����/�룩
%  acceB      �ɻ��˶����ٶȡ���X����Y��ͷ��Z���򣨵�λ����/��/�룩
%  posi       ������������ʼλ�þ��ȡ�γ�ȡ��߶ȣ���λ���ȡ��ȡ��ף�
%
%        
%                           ������ƣ�����  ���ڣ�2002/8/9
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [t,atti,atti_rate,veloB,acceB]=trace(t,T,atti,atti_rate,veloB,acceB)

    if( t==0 )
      acceB(2,1) = 0;               %��ʼ��׼ʱ��    
   elseif(t<=20)                       %����
      acceB(2,1)=1.0;
    elseif(t<=30)                   %��������
      atti_rate(2,1)=1.0;
      acceB(2,1)=1.0;            
   elseif (t<=64)                   %����
      atti_rate(2,1)=0.0;
   elseif (t<=74)                   %��ƽ
      atti_rate(2,1)=-1.0;
      acceB(2,1)=0.0;
   elseif (t<=668)                  %ƽֱ����
      atti_rate(2,1)=0.0;
   elseif (t<=698)                  %��бԤת��
      atti_rate(1,1)=1.0;
   elseif (t<=758)                  %ת��
      atti_rate(3,1)=1.5;
      atti_rate(1,1)=0.0;
   elseif (t<=788)                  %��ƽ
      atti_rate(3,1)=0.0;
      atti_rate(1,1)=-1.0;
   elseif (t<=1334)                 %ƽֱ����
      atti_rate(1,1)=0.0;
   elseif (t<=1342)                 %��������
      atti_rate(2,1)=7.5;
      acceB(2,1)=2.0;
   elseif (t<=1374)                 %��������
      atti_rate(2,1)=0.0;
   elseif (t<=1382)                 %��ƽ
      atti_rate(2,1)=-7.5;
      acceB(2,1)=0.0;
   elseif (t<=1862)                 %ƽֱ����
       atti_rate(2,1)=0.0;
   elseif (t<=1902)                 %���ٷ���
      acceB(2,1)=-2.5;
   elseif (t<=1905)                 %��бԤת��
      atti_rate(1,1)=10.0;
      acceB(2,1)=0.0;
   elseif (t<=1965)                 %ת��
      atti_rate(3,1)=1.5;
      atti_rate(1,1)=0.0;
   elseif (t<=1968)                 %��ƽ
      atti_rate(3,1)=0.0;
      atti_rate(1,1)=-10.0;
   elseif (t<=2568)                 %ƽֱ����
      atti_rate(1,1)=0.0;
   elseif (t<=2574)                 %��ͷ
      atti_rate(2,1)=-7.5;
   elseif (t<=2594)                 %����
      atti_rate(2,1)=0.0;
   elseif (t<=2600)                 %��ƽ
      atti_rate(2,1)=7.5;
   elseif (t<=2603)                 %��бԤת��
      atti_rate(2,1)=0.0;
      atti_rate(1,1)=10.0;
   elseif (t<=2663)                 %ת��
      atti_rate(3,1)=1.5;
      atti_rate(1,1)=0.0;
   elseif (t<=2666)                 %��ƽ
      atti_rate(3,1)=0.0;
      atti_rate(1,1)=-10.0;
   elseif (t<=3600)                 %ƽֱ����
      atti_rate(1,1)=0.0;            
   end

   veloB(2,1)=veloB(2,1)+acceB(2,1)*T;    

   atti(1,1)=atti(1,1)+atti_rate(1,1)*T;
   atti(2,1)=atti(2,1)+atti_rate(2,1)*T;
   atti(3,1)=atti(3,1)+atti_rate(3,1)*T;
  
%   if(atti(1,1)>360)
%	atti(1,1)=atti(1,1)-fix(atti(1,1)/360)*360;
%   end


