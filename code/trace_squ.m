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


function [t,atti,atti_rate,veloB,acceB]=trace_squ(t,T,atti,atti_rate,veloB,acceB)

   %%%%%%%%%%%%%%%%ģ����̬�仯%%%%%%%%%%%%%%%    
   %  ��ֹ״̬
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   if(t<=5)
      return;
   elseif(t<=10)
      acceB(2,1)=2.0;
   elseif(t<=50)
      acceB(2,1)=0.0;
   elseif(t<=55)
      acceB(2,1)=-2.0;
   elseif(t<=60)
      acceB(2,1)=0.0;
      atti_rate(3,1)=-18.0;
   elseif(t<=65)
      acceB(2,1)=2.0;
      atti_rate(3,1)=0.0;
   elseif(t<=105)
      acceB(2,1)=0.0;
  elseif(t<=110)
      acceB(2,1)=-2.0;
  elseif(t<=115)
      acceB(2,1)=0.0;
      atti_rate(3,1)=-18.0;
  elseif(t<=120)
      acceB(2,1)=2.0;
      atti_rate(3,1)=0.0;      
  elseif(t<=160)
      acceB(2,1)=0.0;
  elseif(t<=165)
      acceB(2,1)=-2.0;
  elseif(t<=170)
      acceB(2,1)=0.0;
      atti_rate(3,1)=-18.0;
  elseif(t<=175)
      acceB(2,1)=2.0;
      atti_rate(3,1)=0.0;
  elseif(t<=215)
      acceB(2,1)=0.0;
  elseif(t<=220)
      acceB(2,1)=-2.0;
  elseif(t<=225)
      acceB(2,1)=0.0;
      atti_rate(3,1)=-18.0;
  elseif(t<=250)
      atti_rate(3,1)=0.0;
   end

   veloB(2,1)=veloB(2,1)+acceB(2,1)*T;   %�ɻ���ʼ�˶��ٶȣ���/�룩������ͷ

   atti(1,1)=atti(1,1)+atti_rate(1,1)*T;
   atti(2,1)=atti(2,1)+atti_rate(2,1)*T;
   atti(3,1)=atti(3,1)+atti_rate(3,1)*T;
  

