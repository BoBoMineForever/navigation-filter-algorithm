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


function [t,atti,atti_rate,veloB,acceB]=trace_dyn(t,T,atti,atti_rate,veloB,acceB)

   %%%%%%%%%%%%%%%%ģ����̬�仯%%%%%%%%%%%%%%%    
   %  ��ֹ״̬
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
   if(t<=120)                  
      return;
   elseif (t<=125)                  
      atti_rate(1,1)=9.0;
   elseif (t<=240)                  
      atti_rate(1,1)=0.0;
   elseif (t<=245)                  
      atti_rate(1,1)=-9.0;
   elseif (t<=360)                  
      atti_rate(1,1)=0.0;
   elseif(t<=365)
      atti_rate(1,1)=-9.0;
   elseif(t<=480)
      atti_rate(1,1)=0.0;
   elseif(t<=485)
      atti_rate(1,1)=9.0;
   else
      atti_rate(1,1)=0.0;
   end
     %����仯

   if(t<=600)                  
      atti_rate(2,1)=0.0;
   elseif (t<=605)                  
      atti_rate(2,1)=9.0;
   elseif (t<=720)                  
      atti_rate(2,1)=0.0;
   elseif (t<=725)                  
      atti_rate(2,1)=-9.0;
   elseif (t<=840)                  
      atti_rate(2,1)=0.0;
   elseif(t<=845)
      atti_rate(2,1)=-9.0;
   elseif(t<=960)
      atti_rate(2,1)=0.0;
   elseif(t<=965)
      atti_rate(2,1)=9.0;
   else
      atti_rate(2,1)=0.0;
   end
     %�����仯
 
   if(t<=1080)                  
      atti_rate(3,1)=0.0;
   elseif (t<=1085)                  
      atti_rate(3,1)=9.0;
   elseif (t<=1200)                  
      atti_rate(3,1)=0.0;
   elseif (t<=1205)                  
      atti_rate(3,1)=-9.0;
   elseif (t<=1320)                  
      atti_rate(3,1)=0.0;
   elseif(t<=1325)
      atti_rate(3,1)=-9.0;
   elseif(t<=1440)
      atti_rate(3,1)=0.0;
   elseif(t<=1445)
      atti_rate(3,1)=9.0;
   else
      atti_rate(3,1)=0.0;
   end
     %����仯

   veloB(2,1)=veloB(2,1)+acceB(2,1)*T;   %�ɻ���ʼ�˶��ٶȣ���/�룩������ͷ

   atti(1,1)=atti(1,1)+atti_rate(1,1)*T;
   atti(2,1)=atti(2,1)+atti_rate(2,1)*T;
   atti(3,1)=atti(3,1)+atti_rate(3,1)*T;
  

